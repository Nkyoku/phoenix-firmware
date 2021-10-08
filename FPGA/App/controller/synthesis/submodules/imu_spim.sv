/**
 * @file imu_spim.sv
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

module imu_spim #(
		parameter int PRESCALER = 1
	) (
		input  wire        reset,           // reset.reset
		input  wire        clk,             //   clk.clk
		input  wire [2:0]  slave_address,   // slave.address
		input  wire        slave_read,      //      .read
		output reg  [15:0] slave_readdata,  //      .readdata
		input  wire        slave_write,     //      .write
		input  wire [15:0] slave_writedata, //      .writedata
		input  wire        SCLK,            //  spis.export
		input  wire        MOSI,            //      .export
		output wire        MISO,            //      .export
		input  wire        SS_n,            //      .export
		output wire        spim_mosi,       //  spim.mosi
		input  wire        spim_miso,       //      .miso
		output wire        spim_sclk,       //      .sclk
		output wire        spim_cs_n,       //      .cs_n
		input  wire        spim_int_n       //      .int_n
	);
	
	// SPI Selector
	logic pass_through = 1'b1;
	logic internal_mosi;
	logic internal_sclk = 1'b0;
	logic internal_cs_n = 1'b1;
	assign spim_mosi = pass_through ? MOSI : internal_mosi;
	assign spim_sclk = pass_through ? SCLK : internal_sclk;
	assign spim_cs_n = pass_through ? SS_n : internal_cs_n;
	assign MISO = spim_miso;
	
	// Register
	wire [15:0] reg_control = {15'h0000, pass_through};
	logic [15:0] reg_temp = '0;
	logic [15:0] reg_accel_x = '0;
	logic [15:0] reg_accel_y = '0;
	logic [15:0] reg_accel_z = '0;
	logic [15:0] reg_gyro_x = '0;
	logic [15:0] reg_gyro_y = '0;
	logic [15:0] reg_gyro_z = '0;
	always @(posedge clk, posedge reset) begin
		if (reset == 1'b1) begin
			slave_readdata <= '0;
			pass_through <= 1'b1;
		end
		else begin
			if (slave_read == 1'b1) begin
                case (slave_address)
                    0       : slave_readdata <= reg_control;
                    1       : slave_readdata <= reg_temp;
                    2       : slave_readdata <= reg_accel_x;
                    3       : slave_readdata <= reg_accel_y;
                    4       : slave_readdata <= reg_accel_z;
                    5       : slave_readdata <= reg_gyro_x;
                    6       : slave_readdata <= reg_gyro_y;
                    7       : slave_readdata <= reg_gyro_z;
                    default : slave_readdata <= 16'h0000;
                endcase
            end
			if ((slave_write == 1'b1) & (slave_address == 0)) begin
				pass_through <= slave_writedata[0];
			end
		end
	end
	
	// Interrupt synchronizer
	logic [1:0] spim_int_n_ff = '0;
	always @(posedge clk) begin
		spim_int_n_ff <= {spim_int_n_ff[0], spim_int_n};
	end
	
	// Clock generator
	logic [$clog2(PRESCALER):0] prescaler_counter = '0;
	wire prescaler_out = (prescaler_counter == '0);
	always @(posedge clk) begin
		if ((PRESCALER - 1) <= prescaler_counter) begin
			prescaler_counter <= '0;
		end
		else begin
			prescaler_counter <= prescaler_counter + 1'b1;
		end
	end
	
	// SPI Master
	logic int_n_last = 1'b0;
	wire int_assert = int_n_last & ~spim_int_n_ff[1];
	logic [6:0] spi_counter = '0;
	logic [7:0] spi_dout = '0;
	logic [15:0] spi_din = '0;
	assign internal_mosi = spi_dout[7];
	logic spi_latch = 1'b0;
	always @(posedge clk, posedge reset) begin
		if (reset == 1'b1) begin
			internal_cs_n <= 1'b1;
			internal_sclk <= 1'b0;
		end
		else begin
			spi_latch <= 1'b0;
			if (prescaler_out == 1'b1) begin
				int_n_last <= spim_int_n_ff[1];
				if ((int_assert == 1'b1) & (pass_through == 1'b0)) begin
					internal_cs_n <= 1'b0;
					internal_sclk <= 1'b0;
					spi_counter <= '0;
					spi_dout <= 8'h9D; // Read from 0x1D
				end
				else if (internal_cs_n == 1'b0) begin
					internal_sclk <= (spi_counter < 119) ? ~internal_sclk : 1'b1;
					if (internal_sclk == 1'b0) begin
						spi_din <= {spi_din[14:0], spim_miso};
						spi_latch <= 1'b1;
					end
					else begin
						spi_counter <= spi_counter + 1'b1;
						if (119 <= spi_counter) begin
							internal_cs_n <= 1'b1;
						end
						spi_dout <= {spi_dout[6:0], 1'b0};
					end
				end
			end
			if (spi_latch == 1'b1) begin
				if (spi_counter == 23) begin
					reg_temp <= spi_din;
				end
				if (spi_counter == 39) begin
					reg_accel_x <= spi_din;
				end
				if (spi_counter == 55) begin
					reg_accel_y <= spi_din;
				end
				if (spi_counter == 71) begin
					reg_accel_z <= spi_din;
				end
				if (spi_counter == 87) begin
					reg_gyro_x <= spi_din;
				end
				if (spi_counter == 103) begin
					reg_gyro_y <= spi_din;
				end
				if (spi_counter == 119) begin
					reg_gyro_z <= spi_din;
				end
			end
		end
	end
endmodule
