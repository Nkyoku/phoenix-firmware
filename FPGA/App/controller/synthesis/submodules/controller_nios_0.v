// controller_nios_0.v

// This file was auto-generated from altera_nios2_hw.tcl.  If you edit it your changes
// will probably be lost.
// 
// Generated using ACDS version 20.1 720

`timescale 1 ps / 1 ps
module controller_nios_0 (
		input  wire        clk,                                 //                                  clk.clk
		input  wire        reset_n,                             //                                reset.reset_n
		input  wire        reset_req,                           //                                     .reset_req
		output wire [15:0] d_address,                           //                          data_master.address
		output wire [3:0]  d_byteenable,                        //                                     .byteenable
		output wire        d_read,                              //                                     .read
		input  wire [31:0] d_readdata,                          //                                     .readdata
		input  wire        d_waitrequest,                       //                                     .waitrequest
		output wire        d_write,                             //                                     .write
		output wire [31:0] d_writedata,                         //                                     .writedata
		output wire        debug_mem_slave_debugaccess_to_roms, //                                     .debugaccess
		output wire [15:0] i_address,                           //                   instruction_master.address
		output wire        i_read,                              //                                     .read
		input  wire [31:0] i_readdata,                          //                                     .readdata
		input  wire        i_waitrequest,                       //                                     .waitrequest
		input  wire [31:0] dtcm0_readdata,                      //        tightly_coupled_data_master_0.readdata
		output wire [15:0] dtcm0_address,                       //                                     .address
		output wire        dtcm0_read,                          //                                     .read
		output wire        dtcm0_clken,                         //                                     .clken
		output wire        dtcm0_write,                         //                                     .write
		output wire [31:0] dtcm0_writedata,                     //                                     .writedata
		output wire [3:0]  dtcm0_byteenable,                    //                                     .byteenable
		input  wire [31:0] itcm0_readdata,                      // tightly_coupled_instruction_master_0.readdata
		output wire [14:0] itcm0_address,                       //                                     .address
		output wire        itcm0_read,                          //                                     .read
		output wire        itcm0_clken,                         //                                     .clken
		input  wire        eic_port_valid,                      //              interrupt_controller_in.valid
		input  wire [44:0] eic_port_data,                       //                                     .data
		output wire        debug_reset_request,                 //                  debug_reset_request.reset
		input  wire [8:0]  debug_mem_slave_address,             //                      debug_mem_slave.address
		input  wire [3:0]  debug_mem_slave_byteenable,          //                                     .byteenable
		input  wire        debug_mem_slave_debugaccess,         //                                     .debugaccess
		input  wire        debug_mem_slave_read,                //                                     .read
		output wire [31:0] debug_mem_slave_readdata,            //                                     .readdata
		output wire        debug_mem_slave_waitrequest,         //                                     .waitrequest
		input  wire        debug_mem_slave_write,               //                                     .write
		input  wire [31:0] debug_mem_slave_writedata,           //                                     .writedata
		input  wire        A_ci_multi_done,                     //            custom_instruction_master.done
		input  wire [31:0] A_ci_multi_result,                   //                                     .multi_result
		output wire [4:0]  A_ci_multi_a,                        //                                     .multi_a
		output wire [4:0]  A_ci_multi_b,                        //                                     .multi_b
		output wire [4:0]  A_ci_multi_c,                        //                                     .multi_c
		output wire        A_ci_multi_clk_en,                   //                                     .clk_en
		output wire        A_ci_multi_clock,                    //                                     .clk
		output wire        A_ci_multi_reset,                    //                                     .reset
		output wire        A_ci_multi_reset_req,                //                                     .reset_req
		output wire [31:0] A_ci_multi_dataa,                    //                                     .multi_dataa
		output wire [31:0] A_ci_multi_datab,                    //                                     .multi_datab
		output wire [7:0]  A_ci_multi_n,                        //                                     .multi_n
		output wire        A_ci_multi_readra,                   //                                     .multi_readra
		output wire        A_ci_multi_readrb,                   //                                     .multi_readrb
		output wire        A_ci_multi_start,                    //                                     .start
		output wire        A_ci_multi_writerc,                  //                                     .multi_writerc
		input  wire [31:0] E_ci_combo_result,                   //                                     .result
		output wire [4:0]  E_ci_combo_a,                        //                                     .a
		output wire [4:0]  E_ci_combo_b,                        //                                     .b
		output wire [4:0]  E_ci_combo_c,                        //                                     .c
		output wire [31:0] E_ci_combo_dataa,                    //                                     .dataa
		output wire [31:0] E_ci_combo_datab,                    //                                     .datab
		output wire        E_ci_combo_estatus,                  //                                     .estatus
		output wire [31:0] E_ci_combo_ipending,                 //                                     .ipending
		output wire [7:0]  E_ci_combo_n,                        //                                     .n
		output wire        E_ci_combo_readra,                   //                                     .readra
		output wire        E_ci_combo_readrb,                   //                                     .readrb
		output wire        E_ci_combo_writerc                   //                                     .writerc
	);

	controller_nios_0_cpu cpu (
		.clk                                 (clk),                                 //                                  clk.clk
		.reset_n                             (reset_n),                             //                                reset.reset_n
		.reset_req                           (reset_req),                           //                                     .reset_req
		.d_address                           (d_address),                           //                          data_master.address
		.d_byteenable                        (d_byteenable),                        //                                     .byteenable
		.d_read                              (d_read),                              //                                     .read
		.d_readdata                          (d_readdata),                          //                                     .readdata
		.d_waitrequest                       (d_waitrequest),                       //                                     .waitrequest
		.d_write                             (d_write),                             //                                     .write
		.d_writedata                         (d_writedata),                         //                                     .writedata
		.debug_mem_slave_debugaccess_to_roms (debug_mem_slave_debugaccess_to_roms), //                                     .debugaccess
		.i_address                           (i_address),                           //                   instruction_master.address
		.i_read                              (i_read),                              //                                     .read
		.i_readdata                          (i_readdata),                          //                                     .readdata
		.i_waitrequest                       (i_waitrequest),                       //                                     .waitrequest
		.dtcm0_readdata                      (dtcm0_readdata),                      //        tightly_coupled_data_master_0.readdata
		.dtcm0_address                       (dtcm0_address),                       //                                     .address
		.dtcm0_read                          (dtcm0_read),                          //                                     .read
		.dtcm0_clken                         (dtcm0_clken),                         //                                     .clken
		.dtcm0_write                         (dtcm0_write),                         //                                     .write
		.dtcm0_writedata                     (dtcm0_writedata),                     //                                     .writedata
		.dtcm0_byteenable                    (dtcm0_byteenable),                    //                                     .byteenable
		.itcm0_readdata                      (itcm0_readdata),                      // tightly_coupled_instruction_master_0.readdata
		.itcm0_address                       (itcm0_address),                       //                                     .address
		.itcm0_read                          (itcm0_read),                          //                                     .read
		.itcm0_clken                         (itcm0_clken),                         //                                     .clken
		.eic_port_valid                      (eic_port_valid),                      //              interrupt_controller_in.valid
		.eic_port_data                       (eic_port_data),                       //                                     .data
		.debug_reset_request                 (debug_reset_request),                 //                  debug_reset_request.reset
		.debug_mem_slave_address             (debug_mem_slave_address),             //                      debug_mem_slave.address
		.debug_mem_slave_byteenable          (debug_mem_slave_byteenable),          //                                     .byteenable
		.debug_mem_slave_debugaccess         (debug_mem_slave_debugaccess),         //                                     .debugaccess
		.debug_mem_slave_read                (debug_mem_slave_read),                //                                     .read
		.debug_mem_slave_readdata            (debug_mem_slave_readdata),            //                                     .readdata
		.debug_mem_slave_waitrequest         (debug_mem_slave_waitrequest),         //                                     .waitrequest
		.debug_mem_slave_write               (debug_mem_slave_write),               //                                     .write
		.debug_mem_slave_writedata           (debug_mem_slave_writedata),           //                                     .writedata
		.A_ci_multi_done                     (A_ci_multi_done),                     //            custom_instruction_master.done
		.A_ci_multi_result                   (A_ci_multi_result),                   //                                     .multi_result
		.A_ci_multi_a                        (A_ci_multi_a),                        //                                     .multi_a
		.A_ci_multi_b                        (A_ci_multi_b),                        //                                     .multi_b
		.A_ci_multi_c                        (A_ci_multi_c),                        //                                     .multi_c
		.A_ci_multi_clk_en                   (A_ci_multi_clk_en),                   //                                     .clk_en
		.A_ci_multi_clock                    (A_ci_multi_clock),                    //                                     .clk
		.A_ci_multi_reset                    (A_ci_multi_reset),                    //                                     .reset
		.A_ci_multi_reset_req                (A_ci_multi_reset_req),                //                                     .reset_req
		.A_ci_multi_dataa                    (A_ci_multi_dataa),                    //                                     .multi_dataa
		.A_ci_multi_datab                    (A_ci_multi_datab),                    //                                     .multi_datab
		.A_ci_multi_n                        (A_ci_multi_n),                        //                                     .multi_n
		.A_ci_multi_readra                   (A_ci_multi_readra),                   //                                     .multi_readra
		.A_ci_multi_readrb                   (A_ci_multi_readrb),                   //                                     .multi_readrb
		.A_ci_multi_start                    (A_ci_multi_start),                    //                                     .start
		.A_ci_multi_writerc                  (A_ci_multi_writerc),                  //                                     .multi_writerc
		.E_ci_combo_result                   (E_ci_combo_result),                   //                                     .result
		.E_ci_combo_a                        (E_ci_combo_a),                        //                                     .a
		.E_ci_combo_b                        (E_ci_combo_b),                        //                                     .b
		.E_ci_combo_c                        (E_ci_combo_c),                        //                                     .c
		.E_ci_combo_dataa                    (E_ci_combo_dataa),                    //                                     .dataa
		.E_ci_combo_datab                    (E_ci_combo_datab),                    //                                     .datab
		.E_ci_combo_estatus                  (E_ci_combo_estatus),                  //                                     .estatus
		.E_ci_combo_ipending                 (E_ci_combo_ipending),                 //                                     .ipending
		.E_ci_combo_n                        (E_ci_combo_n),                        //                                     .n
		.E_ci_combo_readra                   (E_ci_combo_readra),                   //                                     .readra
		.E_ci_combo_readrb                   (E_ci_combo_readrb),                   //                                     .readrb
		.E_ci_combo_writerc                  (E_ci_combo_writerc)                   //                                     .writerc
	);

endmodule
