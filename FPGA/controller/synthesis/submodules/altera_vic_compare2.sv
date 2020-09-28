// (C) 2001-2020 Intel Corporation. All rights reserved.
// Your use of Intel Corporation's design tools, logic functions and other 
// software and tools, and its AMPP partner logic functions, and any output 
// files from any of the foregoing (including device programming or simulation 
// files), and any associated documentation or information are expressly subject 
// to the terms and conditions of the Intel Program License Subscription 
// Agreement, Intel FPGA IP License Agreement, or other applicable 
// license agreement, including, without limitation, that your use is for the 
// sole purpose of programming logic devices manufactured by Intel and sold by 
// Intel or its authorized distributors.  Please refer to the applicable 
// agreement for further details.


`timescale 1ns / 1ns

module altera_vic_compare2 #(parameter PRIORITY_WIDTH = 6, 
                             parameter DATA_WIDTH = 20,
                             parameter REGISTER_OUTPUT = 1)
(  
   input  wire                   int_validA,
   input  wire  [DATA_WIDTH-1:0] int_dataA,
   
   input  wire                   int_validB,
   input  wire  [DATA_WIDTH-1:0] int_dataB,
      
   output reg                    int_validZ,
   output reg   [DATA_WIDTH-1:0] int_dataZ,
   
   input  wire                   clk
);


// ********************************************************************
// Module Wiring

wire  [PRIORITY_WIDTH:0]   LevelA;
wire  [PRIORITY_WIDTH:0]   LevelB;

reg   [PRIORITY_WIDTH+1:0] LevelDiffZ;


// ********************************************************************
// Module Logic - 1st compare stage clocked

assign LevelA = {int_validA, int_dataA[PRIORITY_WIDTH-1:0]};
assign LevelB = {int_validB, int_dataB[PRIORITY_WIDTH-1:0]};


always @(LevelA, LevelB) begin
   LevelDiffZ = (LevelA - LevelB);
end

generate
if (REGISTER_OUTPUT == 1) begin
	always @(posedge clk) begin
	   if (LevelDiffZ[PRIORITY_WIDTH+1]) begin
	      int_validZ <= int_validB;
	      int_dataZ  <= int_dataB;
	   end
	   else begin
	      int_validZ <= int_validA;
	      int_dataZ  <= int_dataA;
	   end
	end
end else begin
	always @(*) begin
	   if (LevelDiffZ[PRIORITY_WIDTH+1]) begin
	      int_validZ = int_validB;
	      int_dataZ  = int_dataB;
	   end
	   else begin
	      int_validZ = int_validA;
	      int_dataZ  = int_dataA;
	   end
	end
end
endgenerate


endmodule
