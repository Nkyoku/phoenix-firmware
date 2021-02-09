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


// $Id: //acds/rel/20.1std/ip/merlin/altera_irq_mapper/altera_irq_mapper.sv.terp#1 $
// $Revision: #1 $
// $Date: 2019/10/06 $
// $Author: psgswbuild $

// -------------------------------------------------------
// Altera IRQ Mapper
//
// Parameters
//   NUM_RCVRS        : 10
//   SENDER_IRW_WIDTH : 10
//   IRQ_MAP          : 0:0,1:4,2:6,3:2,4:3,5:8,6:9,7:1,8:7,9:5
//
// -------------------------------------------------------

`timescale 1 ns / 1 ns

module controller_irq_mapper
(
    // -------------------
    // Clock & Reset
    // -------------------
    input clk,
    input reset,

    // -------------------
    // IRQ Receivers
    // -------------------
    input                receiver0_irq,
    input                receiver1_irq,
    input                receiver2_irq,
    input                receiver3_irq,
    input                receiver4_irq,
    input                receiver5_irq,
    input                receiver6_irq,
    input                receiver7_irq,
    input                receiver8_irq,
    input                receiver9_irq,

    // -------------------
    // Command Source (Output)
    // -------------------
    output reg [9 : 0] sender_irq
);


    always @* begin
	sender_irq = 0;

        sender_irq[0] = receiver0_irq;
        sender_irq[4] = receiver1_irq;
        sender_irq[6] = receiver2_irq;
        sender_irq[2] = receiver3_irq;
        sender_irq[3] = receiver4_irq;
        sender_irq[8] = receiver5_irq;
        sender_irq[9] = receiver6_irq;
        sender_irq[1] = receiver7_irq;
        sender_irq[7] = receiver8_irq;
        sender_irq[5] = receiver9_irq;
    end

endmodule

