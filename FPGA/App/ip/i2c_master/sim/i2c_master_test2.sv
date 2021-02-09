`timescale 1 ns / 100 ps
module test2 ();
    logic reset = 1'b1;
    logic clk = 1'b0;
    
    logic [2:0] slave_address;
    logic slave_read = 1'b0;
    logic [15:0] slave_readdata;
    logic slave_write = '0;
    logic [15:0] slave_writedata;
    logic irq;
    
    logic i2c_scl_oe, i2c_sda_oe1, i2c_sda_oe2;
    wire i2c_scl = i2c_scl_oe ? 1'b0 : 1'b1;
    wire i2c_sda = (i2c_sda_oe1 | i2c_sda_oe2) ? 1'b0 : 1'b1;
    
    i2c_master #(
        .PRESCALER(5)
    ) i2cm (
        .clk(clk),
        .reset(reset),
        .slave_address(slave_address),
        .slave_read(slave_read),
        .slave_readdata(slave_readdata),
        .slave_write(slave_write),
        .slave_writedata(slave_writedata),
        .irq(irq),
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
        
        write_data(3, 16'h0024); // DADDR<=0x24
        write_data(5, 16'h0055); // TXDATA0<=0x55
        write_data(0, 16'h0002); // LENGTH<=0, IADDR_SZ<=0, DIR<=0, START<=1, RESET<=0
        @(posedge irq);
        write_data(2, 16'h0000); // Clear INTFLAG
        
        repeat(100) @(posedge clk);
        
        write_data(3, 16'h0024); // DADDR<=0x24
        write_data(5, 16'hAA55); // TXDATA0<=0x55, TXDATA1<=0xAA
        write_data(0, 16'h0102); // LENGTH<=1, IADDR_SZ<=0, DIR<=0, START<=1, RESET<=0
        @(posedge irq);
        write_data(2, 16'h0000); // Clear INTFLAG
        
        repeat(100) @(posedge clk);
        
        write_data(3, 16'h0024); // DADDR<=0x24
        write_data(4, 16'h0093); // IADDR<=0x93
        write_data(5, 16'hAA55); // TXDATA0<=0xAA, TXDATA1<=0x55
        write_data(0, 16'h0142); // LENGTH<=1, IADDR_SZ<=1, DIR<=0, START<=1, RESET<=0
        @(posedge irq);
        write_data(2, 16'h0000); // Clear INTFLAG
        
        repeat(100) @(posedge clk);
        
        write_data(3, 16'h0024); // DADDR<=0x24
        write_data(0, 16'h0006); // LENGTH<=0, IADDR_SZ<=0, DIR<=1, START<=1, RESET<=0
        @(posedge irq);
        write_data(2, 16'h0000); // Clear INTFLAG
        
        repeat(100) @(posedge clk);
        
        write_data(3, 16'h0024); // DADDR<=0x24
        write_data(0, 16'h0106); // LENGTH<=1, IADDR_SZ<=0, DIR<=1, START<=1, RESET<=0
        @(posedge irq);
        write_data(2, 16'h0000); // Clear INTFLAG
        
        repeat(100) @(posedge clk);
        
        write_data(3, 16'h0124); // DADDR<=0x24
        write_data(4, 16'h0093); // IADDR<=0x93
        write_data(0, 16'h0146); // LENGTH<=1, IADDR_SZ<=1, DIR<=1, START<=1, RESET<=0
        @(posedge irq);
        write_data(2, 16'h0000); // Clear INTFLAG
        
        repeat(100) @(posedge clk);
        
        write_data(3, 16'h0030); // DADDR<=0x30
        write_data(5, 16'h0055); // TXDATA0<=0x55
        write_data(0, 16'h0002); // LENGTH<=0, IADDR_SZ<=0, DIR<=0, START<=1, RESET<=0
        @(posedge irq);
        write_data(2, 16'h0000); // Clear INTFLAG
        
        repeat(100) @(posedge clk);
        
        write_data(3, 16'h0030); // DADDR<=0x30
        write_data(0, 16'h0006); // LENGTH<=0, IADDR_SZ<=0, DIR<=0, START<=1, RESET<=0
        @(posedge irq);
        write_data(2, 16'h0000); // Clear INTFLAG
        
        repeat(100) @(posedge clk);
        
        write_data(0, 16'h0001); // LENGTH<=0, IADDR_SZ<=0, DIR<=0, START<=0, RESET<=1
        @(posedge irq);
        write_data(2, 16'h0000); // Clear INTFLAG
        
        repeat(100) @(posedge clk);
        $stop;
    end
    
    task write_data(input [2:0] address, input [15:0] data);
        @(posedge clk);
        slave_address <= address;
        slave_write <= 1'b1;
        slave_writedata <= data;
        @(posedge clk);
        slave_address <= 'X;
        slave_write <= 1'b0;
        slave_writedata <= 'X;
    endtask
    
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