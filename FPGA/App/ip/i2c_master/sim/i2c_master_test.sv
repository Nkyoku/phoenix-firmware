`timescale 1 ns / 100 ps
module test ();
    logic reset = 1'b1;
    logic clk = 1'b0;
    
    logic in_start = 1'b0;
    logic in_stop = 1'b0;
    logic in_ack = 1'b0;
    logic [7:0] in_data = '0;
    logic in_valid = 1'b0;
    logic in_ready;
    
    logic out_ack;
    logic [7:0] out_data;
    logic out_valid;
    logic out_ready = 1'b1;
    
    logic i2c_scl_oe, i2c_sda_oe1, i2c_sda_oe2;
    wire i2c_scl = i2c_scl_oe ? 1'b0 : 1'b1;
    wire i2c_sda = (i2c_sda_oe1 | i2c_sda_oe2) ? 1'b0 : 1'b1;
    
    i2c_master_tx_rx #(
        .PRESCALER(5)
    ) i2cm (
        .clk(clk),
        .reset(reset),
        .in_start(in_start),
        .in_stop(in_stop),
        .in_ack(in_ack),
        .in_data(in_data),
        .in_valid(in_valid),
        .in_ready(in_ready),
        .out_ack(out_ack),
        .out_data(out_data),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .i2c_scl_in(i2c_scl),
        .i2c_sda_in(i2c_sda),
        .i2c_scl_oe(i2c_scl_oe),
        .i2c_sda_oe(i2c_sda_oe1)
    );
    
    i2c_slave #(
        .I2C_ADDRESS(8'h24)
    ) i2cs (
        .clk(clk),
        .reset(reset),
        .i2c_scl_in(i2c_scl),
        .i2c_sda_in(i2c_sda),
        .i2c_sda_oe(i2c_sda_oe2)
    );
    
    // Control
    initial begin
        @(negedge reset);
        @(posedge clk);
        
        write_data(1'b1, 1'b0, 8'hFF, 1'b0); // START
        write_data(1'b0, 1'b0, 8'h48, 1'b0); // Address
        write_data(1'b0, 1'b0, 8'h55, 1'b0); // Data
        write_data(1'b0, 1'b0, 8'hAA, 1'b0); // Data
        write_data(1'b0, 1'b1, 8'hFF, 1'b0); // STOP
        
        @(posedge in_ready);
        @(posedge clk);
        
        write_data(1'b1, 1'b0, 8'hFF, 1'b0); // START
        write_data(1'b0, 1'b0, 8'h49, 1'b0); // Address
        write_data(1'b0, 1'b0, 8'hFF, 1'b1); // Data
        write_data(1'b0, 1'b0, 8'hFF, 1'b0); // Data
        write_data(1'b0, 1'b1, 8'hFF, 1'b0); // STOP
        
        @(posedge in_ready);
        @(posedge clk);
        
        write_data(1'b1, 1'b0, 8'hFF, 1'b0); // START
        write_data(1'b0, 1'b0, 8'h48, 1'b0); // Address
        write_data(1'b0, 1'b0, 8'h55, 1'b0); // Data
        write_data(1'b1, 1'b1, 8'hFF, 1'b0); // Repeated START
        write_data(1'b0, 1'b0, 8'h49, 1'b0); // Address
        write_data(1'b0, 1'b0, 8'hFF, 1'b1); // Data
        write_data(1'b0, 1'b0, 8'hFF, 1'b0); // Data
        write_data(1'b0, 1'b1, 8'hFF, 1'b0); // STOP
        
        @(posedge in_ready);
        repeat(100) @(posedge clk);
        $stop;
    end
    
    task write_data(input start, input stop, input [7:0] data, input ack);
        @(posedge clk);
        in_start <= start;
        in_stop <= stop;
        in_ack <= ack;
        in_data <= data;
        in_valid = 1'b1;
        @(posedge clk);
        while (in_ready == 1'b0) begin
            @(posedge clk);
        end
        in_start <= 1'bX;
        in_stop <= 1'bX;
        in_ack <= 1'bX;
        in_data <= 'X;
        in_valid <= 1'b0;
    endtask
    
    // Output
    logic [7:0] out_data_latch = 'X;
    logic out_ack_latch = 1'bX;
    always @(posedge clk) begin
        if (out_valid == 1'b1) begin
            out_data_latch <= out_data;
            out_ack_latch <= out_ack;
        end
    end
    
    // Clock Generation
    always #5ns begin
        clk <= ~clk;
    end
    
    // Reset Generaton
    initial begin
        reset <= 1'b1;
        repeat(10) @(posedge clk);
        reset <= 1'b0;
        repeat(10000) @(posedge clk);
        $stop;
    end
endmodule



module i2c_slave #(
        parameter int I2C_ADDRESS = 1
    ) (
        input  wire       clk,
        input  wire       reset,
        input  wire       i2c_scl_in,
        input  wire       i2c_sda_in,
        output reg        i2c_sda_oe
    );
    
    logic last_scl = 1'b1;
    logic last_sda = 1'b1;
    logic scl_rise;
    logic scl_fall;
    logic sda_rise;
    logic sda_fall;
    
    enum {
        IDLE,
        ADDRESS,
        DATA_WRITE,
        DATA_READ
    } state;
    
    logic ack = 1'b0;
    logic [3:0] counter = '0;
    logic [7:0] written_data = '0;
    logic [7:0] read_data = '0;
    
    always @(posedge clk, posedge reset) begin
        last_scl <= i2c_scl_in;
        last_sda <= i2c_sda_in;
        scl_rise = ~last_scl & i2c_scl_in;
        scl_fall = last_scl & ~i2c_scl_in;
        sda_rise = ~last_sda & i2c_sda_in;
        sda_fall = last_sda & ~i2c_sda_in;
        if (reset == 1'b1) begin
            i2c_sda_oe <= 1'b0;
            state = IDLE;
            counter <= '0;
            written_data <= '1;
            read_data <= '1;
        end
        else begin
            if (i2c_scl_in & sda_fall) begin
                // START, repeated START
                state <= ADDRESS;
                counter <= '0;
                written_data <= '1;
                read_data <= '1;
                $display("START");
            end
            else if (i2c_scl_in & sda_rise) begin
                // STOP
                state <= IDLE;
                counter <= '0;
                written_data <= '1;
                read_data <= '1;
                $display("STOP");
            end
            else if (scl_rise) begin
                counter <= (counter < 8) ? (counter + 1'b1) : '0;
                written_data <= {written_data[6:0], i2c_sda_in};
                if (state == ADDRESS) begin
                    ack <= (written_data[6:0] == I2C_ADDRESS);
                end
                else if (state == DATA_WRITE) begin
                    ack <= 1'b1;
                end
                else if ((counter == 8) & (state == DATA_READ)) begin
                    if (i2c_sda_in == 1'b1) begin
                        $display("NACKed");
                        state <= IDLE;
                    end
                    else begin
                        $display("ACKed");
                    end
                end
            end
            else if (scl_fall) begin
                if (counter == 8) begin
                    if (state == ADDRESS) begin
                        $display("ADDRESS=%02X, R/W=%d", written_data[7:1], written_data[0]);
                        if (ack == 1'b1)
                            $display("ACK");
                        else
                            $display("NACK");
                        i2c_sda_oe = ack;
                        state <= written_data[0] ? DATA_READ : DATA_WRITE;
                        read_data <= 8'hA5;
                    end
                    else if (state == DATA_WRITE) begin
                        $display("WRITE=%02X", written_data[7:0]);
                        if (ack == 1'b1)
                            $display("ACK");
                        else
                            $display("NACK");
                        i2c_sda_oe = ack;
                    end
                    else if (state == DATA_READ) begin
                        i2c_sda_oe <= 1'b0;
                        $display("READ=%02X", read_data[7:0]);
                    end
                    else begin
                        i2c_sda_oe <= 1'b0;
                    end
                end
                else if ((counter < 8) & (state == DATA_READ)) begin
                    i2c_sda_oe <= ~read_data[7];
                    read_data <= {read_data[6:0], read_data[7]};
                end
                else begin
                    i2c_sda_oe <= 1'b0;
                end
            end
        end
    end
    
endmodule