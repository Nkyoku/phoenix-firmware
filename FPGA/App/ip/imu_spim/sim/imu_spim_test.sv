`timescale 1 ns / 1 ps
module test ();
    reg reset = 1'b1;
    reg clk100mhz = 1'b0;
    
    reg [2:0] slave_address = '0;
    reg [15:0] slave_writedata = '0;
    reg slave_write = 1'b0;
    
    // IMU Simulation
    reg [15:0] imu_result[7] = {16'h1234, 16'h2345, 16'h3456, 16'h4567, 16'h5678, 16'h6789, 16'h789A};
    reg [15:0] imu_dout = '0;
    reg [7:0] imu_din = '0;
    reg spi_cs_n = 1'b1;
    reg spi_sclk = 1'b0;
    reg spi_mosi = 1'b0;
    wire spi_miso = spi_cs_n ? 1'bz : imu_dout[15];
    reg [3:0] imu_bit_counter = '0;
    reg [3:0] imu_word_counter = '0;
    always @(spi_sclk, posedge spi_cs_n) begin
        if (spi_cs_n == 1'b0) begin
            if (spi_sclk == 1'b1) begin
                imu_din <= {imu_din[6:0], spi_mosi};
                imu_bit_counter <= imu_bit_counter + 1;
            end
            else begin
                if (imu_bit_counter == 0) begin
                    imu_dout <= imu_result[imu_word_counter];
                    imu_word_counter <= imu_word_counter + 1;
                end
                else begin
                    imu_dout <= {imu_dout[14:0], 1'b0};
                end
            end
        end
        else begin
            imu_bit_counter <= 8;
            imu_word_counter <= 0;
            imu_dout <= '0;
        end
    end
    
    reg spi_int_n = 1'b1;
    always begin
        # 100ns
        spi_int_n <= 1'b1;
        # 1us
        spi_int_n <= 1'b0;
        # 1ms
        spi_int_n <= 1'b1;
    end
    
    imu_spim #(
        .PRESCALER(4)
    ) imu_spim_0 (
        .reset(reset),
        .clk(clk100mhz),
        .slave_address(slave_address),
        .slave_read(1'b0),
        .slave_readdata(),
        .slave_write(slave_write),
        .slave_writedata(slave_writedata),
        .SCLK(1'b0),
        .MOSI(1'b0),
        .MISO(),
        .SS_n(1'b1),
        .spim_mosi(spi_mosi),
        .spim_miso(spi_miso),
        .spim_sclk(spi_sclk),
        .spim_cs_n(spi_cs_n),
        .spim_int_n(spi_int_n)
    );
    
    task write_data(input [2:0] address, input [15:0] data);
        slave_address <= address;
        slave_writedata <= data;
        slave_write <= 1'b1;
        @(posedge clk100mhz);
        slave_write <= 1'b0;
    endtask
    
    initial begin
        @(negedge reset);
        repeat(3) @(posedge clk100mhz);
        write_data(3'b0, 16'h0000);
    end
    
    // Clock 100MHz generation
    always #5ns begin
        clk100mhz <= ~clk100mhz;
    end
    
    // Reset generaton
    initial begin
        reset <= 1'b1;
        repeat(5) @(negedge clk100mhz);
        reset <= 1'b0;
        repeat(1500) @(negedge clk100mhz);
        $stop;
    end
endmodule
