/**
 * @file motor_controller.sv
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************

# Register Map

### STATUS (0x0) Read Only
|  15 ~ 3  |     2     |      1      |      0       |
|----------|-----------|-------------|--------------|
| reserved | DRV_OTW_N | DRV_FAULT_N | HALL_FAULT_N |

それぞれstatus_driver_otw_n, status_driver_fault_n, status_hall_fault_nの現在値。


### INTFLAG (0x2) Read Only
|  15 ~ 3  |     2     |      1      |      0       |
|----------|-----------|-------------|--------------|
| reserved | DRV_OTW_N | DRV_FAULT_N | HALL_FAULT_N |

平常時は全てのビットが1だがSTATUSの各ビットが0になる瞬間があったら対応するビットが0にクリアされる。
DRV_OTW_N, DRV_FAULT_N, HALL_FAULT_Nのいずれかがクリアされた瞬間にフォルト出力と割り込み出力がアサートされる。
読むと全てのビットが1にセットされる。


### FAULT (0x4) Read/Write
|  15 ~ 4  |     3     |     2     |     1     |     0     |
|----------|-----------|-----------|-----------|-----------|
| reserved | BRAKE_CLR | BRAKE_SET | FAULT_CLR | FAULT_SET |

FAULT_CLRに1を書き込むとフォルト出力がデアサートされる。
FAULT_SETに1を書き込むとフォルト出力がアサートされる。
FAULT_CLRよりFAULT_SETのほうが優先される。
現在のフォルト出力の状態がFAULT_SETの値として読める。

BRAKE_CLRに1を書き込むとショートブレーキが解除される。
BRAKE_SETに1を書き込むとショートブレーキになる。
BRAKE_CLRよりBRAKE_SETのほうが優先される。
現在のショートブレーキの状態はBRAKE_SETの値として読める。


### POWER (0x6) Read/Write
|      15 ~ 0     |
|-----------------|
|  PWM ON cycles  |
| (-2985 ~ +2985) |

PWMドライバに指令値を送る。
PWMデューティ比はPOWERを3000で割った値になる。


*******************************************************************************/

module motor_controller (
        input  wire        clk,
        input  wire        reset,
		output reg         fault,
        output reg         brake,
        input  wire        status_driver_otw_n,
		input  wire        status_driver_fault_n,
		input  wire        status_hall_fault_n,
        output reg  [15:0] pwm_source_data,
        output reg         pwm_source_valid,
        input  wire        pwm_source_ready,
        input  wire [1:0]  slave_address,
        output reg  [15:0] slave_readdata,
        input  wire [15:0] slave_writedata,
        input  wire        slave_read,
        input  wire        slave_write,
		output reg         irq
    );
    
    // Register Address Map
    typedef enum {
        REGISTER_STATUS   = 'h0,
        REGISTER_INTFLAG  = 'h1, 
        REGISTER_FAULT    = 'h2,
        REGISTER_POWER    = 'h3
    } REGISTER_MAP;

    // Fault
    logic fault_set;
    logic fault_clear;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            fault <= 1'b0;
        end
        else begin
            if (~status_driver_otw_n | ~status_driver_fault_n | ~status_hall_fault_n) begin
                fault <= 1'b1;
            end
            else begin
                fault <= (fault & ~fault_clear) | fault_set;
            end
        end
    end
    
    // Brake
    logic brake_set;
    logic brake_clear;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            brake <= 1'b0;
        end
        else begin
            brake <= (brake & ~brake_clear) | brake_set;
        end
    end

    // Interrupt Flag
    logic status_driver_otw_n_ff = 1'b1;
    logic status_driver_fault_n_ff = 1'b1;
    logic status_hall_fault_n_ff = 1'b1;
    logic intflag_driver_otw_n = 1'b1;
    logic intflag_driver_fault_n = 1'b1;
    logic intflag_hall_fault_n = 1'b1;
    wire intflag_clear = slave_read & (slave_address == REGISTER_INTFLAG);
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            status_driver_otw_n_ff <= 1'b1;
            status_driver_fault_n_ff <= 1'b1;
            status_hall_fault_n_ff <= 1'b1;
            intflag_driver_otw_n <= 1'b1;
            intflag_driver_fault_n <= 1'b1;
            intflag_hall_fault_n <= 1'b1;
            irq <= 1'b0;
        end
        else begin
            status_driver_otw_n_ff   <= fault_clear | status_driver_otw_n;
            status_driver_fault_n_ff <= fault_clear | status_driver_fault_n;
            status_hall_fault_n_ff   <= fault_clear | status_hall_fault_n;
            intflag_driver_otw_n   <= (intflag_clear | intflag_driver_otw_n  ) & (status_driver_otw_n   | ~status_driver_otw_n_ff  );
            intflag_driver_fault_n <= (intflag_clear | intflag_driver_fault_n) & (status_driver_fault_n | ~status_driver_fault_n_ff);
            intflag_hall_fault_n   <= (intflag_clear | intflag_hall_fault_n  ) & (status_hall_fault_n   | ~status_hall_fault_n_ff  );
            irq <= ~intflag_driver_otw_n | ~intflag_driver_fault_n | ~intflag_hall_fault_n;
        end
    end

    // Avalon-MM
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            fault_set <= 1'b0;
            fault_clear <= 1'b0;
            brake_set <= 1'b0;
            brake_clear <= 1'b0;
            pwm_source_data <= '0;
            pwm_source_valid <= 1'b0;
        end
        else begin
            fault_set <= 1'b0;
            fault_clear <= 1'b0;
            brake_set <= 1'b0;
            brake_clear <= 1'b0;
            if (pwm_source_valid & pwm_source_ready) begin
                pwm_source_valid <= 1'b0;
            end
            if (slave_read == 1'b1) begin
                case (slave_address)
                    REGISTER_STATUS  : slave_readdata <= {13'h0000, status_driver_otw_n, status_driver_fault_n, status_hall_fault_n};
                    REGISTER_INTFLAG : slave_readdata <= {13'h0000, intflag_driver_otw_n, intflag_driver_fault_n, intflag_hall_fault_n};
                    REGISTER_FAULT   : slave_readdata <= {13'h0000, brake, 1'b0, fault};
                    REGISTER_POWER   : slave_readdata <= pwm_source_data;
                    default          : slave_readdata <= 16'h0000;
                endcase
            end
            if (slave_write == 1'b1) begin
                if (slave_address == REGISTER_FAULT) begin
                    fault_set <= slave_writedata[0];
                    fault_clear <= slave_writedata[1];
                    brake_set <= slave_writedata[2];
                    brake_clear <= slave_writedata[3];
                end
                if (slave_address == REGISTER_POWER) begin
                    pwm_source_data <= slave_writedata[15:0];
                    pwm_source_valid <= 1'b1;
                end
            end
            
        end
    end
endmodule
