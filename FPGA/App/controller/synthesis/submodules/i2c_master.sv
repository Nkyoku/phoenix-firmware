/**
 * @file i2c_master.sv
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************

# Register Map

### CONTROL (0x0) Write only
|  15 ~ 9  |    8   |     7    |     6    |   5 ~ 3  |   2   |   1   |   0   |
|----------|--------|----------|----------|----------|-------|-------|-------|
| reserved | LENGTH | reserved | IADDR_SZ | reserved |  DIR  | START | RESET |

このレジスタはSTATUS.BUSYが0のときにしか操作してはいけない。
STARTに1を書き込むとI2Cアクセスが開始される。
DIRが0のとき書き込み、1のとき読み出しが行われる。
RESETに1を書き込むとSCLが9回トグルされる。
LENGTHには読み書きするバイト数-1を指定する。

### STATUS (0x2) Read only
|  15 ~ 2  |   1   |   0  |
|----------|-------|------|
| reserved |  ACK  | BUSY |

I2Cアクセスが実行されている最中はBUSYが1にセットされる。
I2Cアクセスが正常に行われた場合ACKが1にセットされる。

### INTFLAG (0x4) Read/Write
|  15 ~ 1  |     0    |
|----------|----------|
| reserved | COMPLETE |

I2Cアクセスが完了したとき成否に関わらずCOMPLETEが1にセット、irqがアサートされる。
このレジスタへ何らかの値を書き込むとCOMPLETEが0にクリアされ、irqがデアサートされる。

### DADDR (0x6) Write only
|  15 ~ 7  |  6 ~ 0  |
|----------|---------|
| reserved |  DADDR  |

I2Cアクセスを開始する前にDADDRにアクセス対象のI2CデバイスのI2Cアドレスを書き込んでおく。

### IADDR (0x8) Write only
|  15 ~ 8  |  7 ~ 0  |
|----------|---------|
| reserved |  IADDR  |

アクセス対象のI2Cデバイスが内部レジスタアドレス持つ場合IADDRにそのレジスタアドレスを書き込み、
CONTROL.IADDR_SZにアドレスのバイト数を書き込むとI2Cアクセスの際に自動的にレジスタアドレスの設定と読み書きシーケンスが実行される。

### TXDATA (0xA) Write only
|  15 ~ 8 |  7 ~ 0  |
|---------|---------|
| TXDATA1 | TXDATA0 |

書き込むデータを指定する。

### RXDATA (0xC) Read only
|  15 ~ 8 |  7 ~ 0  |
|---------|---------|
| RXDATA1 | RXDATA0 |

読み出されたデータが格納される。

*******************************************************************************/

module i2c_master #(
        parameter int PRESCALER = 4 // fscl = clk / (4 * PRESCALER)
    ) (
        input  wire        clk,
        input  wire        reset,
        input  wire [2:0]  slave_address,
        input  wire        slave_read,
        output reg  [15:0] slave_readdata,
        input  wire        slave_write,
        input  wire [15:0] slave_writedata,
        output reg         irq,
        input  wire        i2c_scl_in,
        input  wire        i2c_sda_in,
        output wire        i2c_scl_oe,
        output wire        i2c_sda_oe
    );
    
    // I2C transceiver
    logic txrx_in_start = 1'b0;
    logic txrx_in_stop = 1'b0;
    logic txrx_in_ack = 1'b0;
    logic [7:0] txrx_in_data = '1;
    logic txrx_in_valid = 1'b0;
    logic txrx_in_ready;
    logic txrx_out_ack;
    logic [7:0] txrx_out_data;
    logic txrx_out_valid;
    i2c_master_tx_rx #(
        .PRESCALER(PRESCALER)
    ) txrx (
        .clk        (clk),
        .reset      (reset),
        .in_start   (txrx_in_start),
        .in_stop    (txrx_in_stop),
        .in_ack     (txrx_in_ack),
        .in_data    (txrx_in_data),
        .in_valid   (txrx_in_valid),
        .in_ready   (txrx_in_ready),
        .out_ack    (txrx_out_ack),
        .out_data   (txrx_out_data),
        .out_valid  (txrx_out_valid),
        .out_ready  (1'b1),
        .i2c_scl_in (i2c_scl_in),
        .i2c_sda_in (i2c_sda_in),
        .i2c_scl_oe (i2c_scl_oe),
        .i2c_sda_oe (i2c_sda_oe)
    );
    
    // Configuration registers
    logic reg_start = 1'b0;
    logic reg_reset = 1'b0;
    logic reg_dir = 1'b0;         // Direction, 0=>Write, 1=>Read
    logic [6:0] reg_daddr = '0;   // Device address
    logic reg_iaddr_size = '0;    // Internal address size, 0=>None, 1=>1byte
    logic [7:0] reg_iaddr = '0;   // Internal address
    logic [0:0] reg_length = '0;  // R/W length minus 1
    logic [7:0] reg_txdata [0:1];
    logic [7:0] reg_rxdata [0:1];
    logic reg_complete = 1'b0;
    
    // State machine
    enum {
        IDLE,
        START,
        DEVICE_ADDRESS,
        INTERNAL_ADDRESS,
        WRITE,
        REPEATED_START,
        REPEATED_DEVICE_ADDRESS,
        READ,
        STOP,
        RESET
    } state;
    logic [0:0] state_counter = '0;
    logic state_wait = 1'b0;
    logic state_ack = 1'b0;
    wire  state_busy = (state != IDLE);
    logic state_complete = 1'b0;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            txrx_in_start <= 1'b0;
            txrx_in_stop <= 1'b0;
            txrx_in_ack <= 1'b0;
            txrx_in_data <= '1;
            txrx_in_valid <= 1'b0;
            state <= IDLE;
            state_counter <= '0;
            state_wait <= 1'b0;
            state_ack <= 1'b0;
            state_complete <= 1'b0;
        end
        else begin
            state_counter <= '0;
            state_complete <= 1'b0;
            if (state == IDLE) begin
                txrx_in_start <= 1'b0; // IDLE
                txrx_in_stop <= 1'b0;
                txrx_in_ack <= 1'b0;
                txrx_in_data <= '1;
                txrx_in_valid <= 1'b0;
                state_wait <= 1'b0;
                if (reg_reset == 1'b1) begin
                    state <= RESET;
                end
                else if (reg_start == 1'b1) begin
                    state <= START;
                end
            end
            else if (state == START) begin
                txrx_in_start <= 1'b1; // START
                txrx_in_stop <= 1'b0;
                txrx_in_ack <= 1'b0;
                txrx_in_data <= '1;
                txrx_in_valid <= ~state_wait & (~txrx_in_valid | ~txrx_in_ready);
                state_wait <= (state_wait & ~txrx_out_valid) | (txrx_in_valid & txrx_in_ready);
                if (txrx_out_valid) begin
                    state <= DEVICE_ADDRESS;
                end
            end
            else if (state == DEVICE_ADDRESS) begin
                txrx_in_start <= 1'b0; // ADDRESS
                txrx_in_stop <= 1'b0;
                txrx_in_ack <= 1'b0;
                txrx_in_data <= {reg_daddr, (reg_iaddr_size == 0) ? reg_dir : 1'b0};
                txrx_in_valid <= ~state_wait & (~txrx_in_valid | ~txrx_in_ready);
                state_wait <= (state_wait & ~txrx_out_valid) | (txrx_in_valid & txrx_in_ready);
                if (txrx_out_valid) begin
                    state <= txrx_out_ack ? ((reg_iaddr_size == 0) ? (reg_dir ? READ : WRITE) : INTERNAL_ADDRESS) : STOP;
                    state_ack <= txrx_out_ack;
                end
            end
            else if (state == INTERNAL_ADDRESS) begin
                txrx_in_start <= 1'b0; // WRITE
                txrx_in_stop <= 1'b0;
                txrx_in_ack <= 1'b0;
                txrx_in_data <= reg_iaddr;
                txrx_in_valid <= ~state_wait & (~txrx_in_valid | ~txrx_in_ready);
                state_wait <= (state_wait & ~txrx_out_valid) | (txrx_in_valid & txrx_in_ready);
                if (txrx_out_valid) begin
                    state <= txrx_out_ack ? (reg_dir ? REPEATED_START : WRITE) : STOP;
                    state_ack <= txrx_out_ack;
                end
            end
            else if (state == WRITE) begin
                txrx_in_start <= 1'b0; // WRITE
                txrx_in_stop <= 1'b0;
                txrx_in_ack <= 1'b0;
                txrx_in_data <= reg_txdata[state_counter];
                txrx_in_valid <= ~state_wait & (~txrx_in_valid | ~txrx_in_ready);
                state_wait <= (state_wait & ~txrx_out_valid) | (txrx_in_valid & txrx_in_ready);
                if (txrx_out_valid) begin
                    state <= (txrx_out_ack & (state_counter < reg_length)) ? WRITE : STOP;
                    state_ack <= txrx_out_ack;
                    state_counter <= state_counter + 1'b1;
                end
                else begin
                    state_counter <= state_counter;
                end
            end
            else if (state == REPEATED_START) begin
                txrx_in_start <= 1'b1; // Repeated START
                txrx_in_stop <= 1'b1;
                txrx_in_ack <= 1'b0;
                txrx_in_data <= '1;
                txrx_in_valid <= ~state_wait & (~txrx_in_valid | ~txrx_in_ready);
                state_wait <= (state_wait & ~txrx_out_valid) | (txrx_in_valid & txrx_in_ready);
                if (txrx_out_valid) begin
                    state <= REPEATED_DEVICE_ADDRESS;
                end
            end
            else if (state == REPEATED_DEVICE_ADDRESS) begin
                txrx_in_start <= 1'b0; // ADDRESS
                txrx_in_stop <= 1'b0;
                txrx_in_ack <= 1'b0;
                txrx_in_data <= {reg_daddr, 1'b1};
                txrx_in_valid <= ~state_wait & (~txrx_in_valid | ~txrx_in_ready);
                state_wait <= (state_wait & ~txrx_out_valid) | (txrx_in_valid & txrx_in_ready);
                if (txrx_out_valid) begin
                    state <= READ;
                    state_ack <= txrx_out_ack;
                end
            end
            else if (state == READ) begin
                txrx_in_start <= 1'b0; // READ
                txrx_in_stop <= 1'b0;
                txrx_in_ack <= (state_counter < reg_length);
                txrx_in_data <= '1;
                txrx_in_valid <= ~state_wait & (~txrx_in_valid | ~txrx_in_ready);
                state_wait <= (state_wait & ~txrx_out_valid) | (txrx_in_valid & txrx_in_ready);
                if (txrx_out_valid) begin
                    state <= (txrx_out_ack | (state_counter < reg_length)) ? READ : STOP;
                    state_counter <= state_counter + 1'b1;
                    reg_rxdata[state_counter] <= txrx_out_data;
                end
                else begin
                    state_counter <= state_counter;
                end
            end
            else if (state == STOP) begin
                txrx_in_start <= 1'b0; // STOP
                txrx_in_stop <= 1'b1;
                txrx_in_ack <= 1'b0;
                txrx_in_data <= '1;
                txrx_in_valid <= ~state_wait & (~txrx_in_valid | ~txrx_in_ready);
                state_wait <= (state_wait & ~txrx_out_valid) | (txrx_in_valid & txrx_in_ready);
                if (txrx_out_valid) begin
                    state <= IDLE;
                    state_complete <= 1'b1;
                end
            end
            else begin // state == RESET
                txrx_in_start <= 1'b0; // Send 9 clocks
                txrx_in_stop <= 1'b0;
                txrx_in_ack <= 1'b0;
                txrx_in_data <= '1;
                txrx_in_valid <= ~state_wait & (~txrx_in_valid | ~txrx_in_ready);
                state_wait <= (state_wait & ~txrx_out_valid) | (txrx_in_valid & txrx_in_ready);
                if (txrx_out_valid) begin
                    state <= STOP;
                    state_ack <= 1'b1;
                end
            end
        end
    end
    
    // Avalon-MM
    typedef enum {
        REGISTER_CONTROL = 'h00,
        REGISTER_STATUS  = 'h01,
        REGISTER_INTFLAG = 'h02,
        REGISTER_DADDR   = 'h03,
        REGISTER_IADDR   = 'h04,
        REGISTER_TXDATA  = 'h05,
        REGISTER_RXDATA  = 'h06
    } REGISTER_MAP;
    assign irq = reg_complete;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            reg_start <= 1'b0;
            reg_dir <= 1'b0;
            reg_daddr <= '0;
            reg_iaddr_size <= '0;
            reg_iaddr <= '0;
            reg_length <= '0;
            reg_complete <= 1'b0;
        end
        else begin
            reg_start <= 1'b0;
            reg_reset <= 1'b0;
            reg_complete <= reg_complete | state_complete;
            if (slave_read == 1'b1) begin
                case (slave_address)
                    REGISTER_CONTROL : slave_readdata <= 16'h0000;
                    REGISTER_STATUS  : slave_readdata <= {14'h0000, state_ack, state_busy};
                    REGISTER_INTFLAG : slave_readdata <= {15'h0000, reg_complete};
                    REGISTER_DADDR   : slave_readdata <= 16'h0000; // ロジック節約のため必要なレジスタ以外はライトオンリー
                    REGISTER_IADDR   : slave_readdata <= 16'h0000;
                    REGISTER_TXDATA  : slave_readdata <= 16'h0000;
                    REGISTER_RXDATA  : slave_readdata <= {reg_rxdata[1], reg_rxdata[0]};
                    default          : slave_readdata <= 16'h0000;
                endcase
            end
            if (slave_write == 1'b1) begin
                if (slave_address == REGISTER_CONTROL) begin
                    reg_reset      <= slave_writedata[0];
                    reg_start      <= slave_writedata[1];
                    reg_dir        <= slave_writedata[2];
                    reg_iaddr_size <= slave_writedata[6+:1];
                    reg_length     <= slave_writedata[8+:1];
                end
                if (slave_address == REGISTER_INTFLAG) begin
                    reg_complete <= 1'b0;
                end
                if (slave_address == REGISTER_DADDR) begin
                    reg_daddr <= slave_writedata[6:0];
                end
                if (slave_address == REGISTER_IADDR) begin
                    reg_iaddr <= slave_writedata[7:0];
                end
                if (slave_address == REGISTER_TXDATA) begin
                    {reg_txdata[1], reg_txdata[0]} <= slave_writedata[15:0];
                end
            end
        end
    end
endmodule

module i2c_master_tx_rx #(
        parameter int PRESCALER = 4
    ) (
        input  wire       clk,
        input  wire       reset,
        input  wire       in_start,
        input  wire       in_stop,
        input  wire       in_ack,
        input  wire [7:0] in_data,
        input  wire       in_valid,
        output reg        in_ready,
        output wire       out_ack,
        output wire [7:0] out_data,
        output reg        out_valid,
        input  wire       out_ready,
        input  wire       i2c_scl_in,
        input  wire       i2c_sda_in,
        output reg        i2c_scl_oe,
        output reg        i2c_sda_oe
    );
    
    // Clock
    localparam int PRESC_WIDTH = $clog2(PRESCALER);
    logic clock_enable = 1'b1;
    logic unsigned [PRESC_WIDTH-1:0] clock_prescaler = '0;
    logic [1:0] clock_state = 2'b11;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            clock_enable <= 1'b1;
            clock_prescaler <= '0;
            clock_state <= 2'b11;
        end
        else begin
            if (in_ready == 1'b1) begin
                clock_enable <= 1'b0;
                clock_prescaler <= '0;
                clock_state <= 2'b11;
            end
            else begin
                clock_enable <= (clock_prescaler == 0);
                clock_prescaler <= (clock_prescaler < (PRESCALER - 1)) ? (clock_prescaler + 1'b1) : '0;
                if (clock_prescaler == 0) begin
                    clock_state <= clock_state + 1'b1;
                end
            end
        end
    end
    
    // I2C pins
    logic scl_output1 = 1'b1;
    logic scl_output2 = 1'b1;
    logic sda_output1 = 1'b1;
    logic sda_output2 = 1'b1;
    logic sda_input = 1'b1;
    //logic clock_enable_delayed = 1'b1;
    always @(posedge clk) begin
        //clock_enable_delayed <= clock_enable;
        if (clock_enable == 1'b1) begin
            if (clock_state == 0) begin
                i2c_scl_oe <= ~scl_output1;
            end
            else if (clock_state == 3) begin
                i2c_scl_oe <= ~scl_output2;
            end
            else begin
                i2c_scl_oe <= 1'b0;
            end
            if (clock_state < 2) begin
                i2c_sda_oe <= ~sda_output1;
            end
            else begin
                i2c_sda_oe <= ~sda_output2;
            end
        end
        if (clock_enable & (clock_state == 2)) begin
            sda_input <= i2c_sda_in;
        end
    end
    
    // In, Out interface
    logic [3:0] counter = '0;
    logic [8:0] sda_sr = '1;
    logic assert_ready = 1'b1;
    assign out_data = sda_sr[8:1];
    assign out_ack = ~sda_sr[0];
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            scl_output1 <= 1'b1;
            scl_output2 <= 1'b1;
            sda_output1 <= 1'b1;
            sda_output2 <= 1'b1;
            counter <= '0;
            sda_sr <= '1;
            assert_ready <= 1'b1;
            in_ready <= 1'b0;
            out_valid <= 1'b0;
        end
        else begin
            if (assert_ready == 1'b1) begin
                assert_ready <= 1'b0;
                in_ready <= 1'b1;
            end
            if (out_valid & out_ready) begin
                out_valid <= 1'b0;
                in_ready <= 1'b1;
            end
            if (in_valid & in_ready) begin
                if (in_start == 1'b1) begin
                    // START or Repeated START
                    scl_output1 <= 1'b1 ^ in_stop;
                    scl_output2 <= 1'b0;
                    sda_output1 <= 1'b1;
                    sda_output2 <= 1'b0;
                    counter <= 4'b1;
                end
                else if (in_stop == 1'b1) begin
                    // STOP
                    scl_output1 <= 1'b0;
                    scl_output2 <= 1'b1;
                    sda_output1 <= 1'b0;
                    sda_output2 <= 1'b1;
                    counter <= 4'b1;
                end
                else begin
                    // Data and ACK/NACK
                    scl_output1 <= 1'b0;
                    scl_output2 <= 1'b0;
                    sda_output1 <= in_data[7];
                    sda_output2 <= in_data[7];
                    counter <= 4'd9;
                end
                sda_sr <= {in_data[6:0], ~in_ack, 1'b1};
                in_ready <= 1'b0;
                out_valid <= 1'b0;
            end
            else if (clock_enable & (clock_state == 3) & (0 < counter)) begin
                counter <= counter - 1'b1;
                sda_sr <= {sda_sr[7:0], sda_input};
                if (1 < counter) begin
                    sda_output1 <= sda_sr[8];
                    sda_output2 <= sda_sr[8];
                    out_valid <= 1'b0;
                end
                else begin
                    //sda_sr <= '1;
                    sda_output1 <= 1'b1;
                    sda_output2 <= 1'b1;
                    out_valid <= 1'b1;
                end
            end
        end
    end
endmodule
