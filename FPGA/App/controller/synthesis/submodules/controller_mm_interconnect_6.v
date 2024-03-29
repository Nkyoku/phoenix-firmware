// controller_mm_interconnect_6.v

// This file was auto-generated from altera_mm_interconnect_hw.tcl.  If you edit it your changes
// will probably be lost.
// 
// Generated using ACDS version 20.1 720

`timescale 1 ps / 1 ps
module controller_mm_interconnect_6 (
		input  wire        clk_0_clk_clk,                                        //                                      clk_0_clk.clk
		input  wire        instruction_rom_0_reset1_reset_bridge_in_reset_reset, // instruction_rom_0_reset1_reset_bridge_in_reset.reset
		input  wire        nios_0_reset_reset_bridge_in_reset_reset,             //             nios_0_reset_reset_bridge_in_reset.reset
		input  wire [14:0] nios_0_tightly_coupled_instruction_master_0_address,  //    nios_0_tightly_coupled_instruction_master_0.address
		input  wire        nios_0_tightly_coupled_instruction_master_0_read,     //                                               .read
		output wire [31:0] nios_0_tightly_coupled_instruction_master_0_readdata, //                                               .readdata
		input  wire        nios_0_tightly_coupled_instruction_master_0_clken,    //                                               .clken
		output wire [12:0] instruction_rom_0_s1_address,                         //                           instruction_rom_0_s1.address
		output wire        instruction_rom_0_s1_write,                           //                                               .write
		input  wire [31:0] instruction_rom_0_s1_readdata,                        //                                               .readdata
		output wire [31:0] instruction_rom_0_s1_writedata,                       //                                               .writedata
		output wire [3:0]  instruction_rom_0_s1_byteenable,                      //                                               .byteenable
		output wire        instruction_rom_0_s1_chipselect,                      //                                               .chipselect
		output wire        instruction_rom_0_s1_clken                            //                                               .clken
	);

	wire         nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_waitrequest;   // instruction_rom_0_s1_translator:uav_waitrequest -> nios_0_tightly_coupled_instruction_master_0_translator:uav_waitrequest
	wire  [31:0] nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_readdata;      // instruction_rom_0_s1_translator:uav_readdata -> nios_0_tightly_coupled_instruction_master_0_translator:uav_readdata
	wire         nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_debugaccess;   // nios_0_tightly_coupled_instruction_master_0_translator:uav_debugaccess -> instruction_rom_0_s1_translator:uav_debugaccess
	wire  [14:0] nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_address;       // nios_0_tightly_coupled_instruction_master_0_translator:uav_address -> instruction_rom_0_s1_translator:uav_address
	wire         nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_read;          // nios_0_tightly_coupled_instruction_master_0_translator:uav_read -> instruction_rom_0_s1_translator:uav_read
	wire   [3:0] nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_byteenable;    // nios_0_tightly_coupled_instruction_master_0_translator:uav_byteenable -> instruction_rom_0_s1_translator:uav_byteenable
	wire         nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_readdatavalid; // instruction_rom_0_s1_translator:uav_readdatavalid -> nios_0_tightly_coupled_instruction_master_0_translator:uav_readdatavalid
	wire         nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_lock;          // nios_0_tightly_coupled_instruction_master_0_translator:uav_lock -> instruction_rom_0_s1_translator:uav_lock
	wire         nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_write;         // nios_0_tightly_coupled_instruction_master_0_translator:uav_write -> instruction_rom_0_s1_translator:uav_write
	wire  [31:0] nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_writedata;     // nios_0_tightly_coupled_instruction_master_0_translator:uav_writedata -> instruction_rom_0_s1_translator:uav_writedata
	wire         nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_clken;         // nios_0_tightly_coupled_instruction_master_0_translator:uav_clken -> instruction_rom_0_s1_translator:uav_clken
	wire   [2:0] nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_burstcount;    // nios_0_tightly_coupled_instruction_master_0_translator:uav_burstcount -> instruction_rom_0_s1_translator:uav_burstcount

	altera_merlin_master_translator #(
		.AV_ADDRESS_W                (15),
		.AV_DATA_W                   (32),
		.AV_BURSTCOUNT_W             (1),
		.AV_BYTEENABLE_W             (4),
		.UAV_ADDRESS_W               (15),
		.UAV_BURSTCOUNT_W            (3),
		.USE_READ                    (1),
		.USE_WRITE                   (0),
		.USE_BEGINBURSTTRANSFER      (0),
		.USE_BEGINTRANSFER           (0),
		.USE_CHIPSELECT              (0),
		.USE_BURSTCOUNT              (0),
		.USE_READDATAVALID           (0),
		.USE_WAITREQUEST             (0),
		.USE_READRESPONSE            (0),
		.USE_WRITERESPONSE           (0),
		.AV_SYMBOLS_PER_WORD         (4),
		.AV_ADDRESS_SYMBOLS          (1),
		.AV_BURSTCOUNT_SYMBOLS       (0),
		.AV_CONSTANT_BURST_BEHAVIOR  (0),
		.UAV_CONSTANT_BURST_BEHAVIOR (0),
		.AV_LINEWRAPBURSTS           (0),
		.AV_REGISTERINCOMINGSIGNALS  (0)
	) nios_0_tightly_coupled_instruction_master_0_translator (
		.clk                    (clk_0_clk_clk),                                                                                  //                       clk.clk
		.reset                  (nios_0_reset_reset_bridge_in_reset_reset),                                                       //                     reset.reset
		.uav_address            (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_address),       // avalon_universal_master_0.address
		.uav_burstcount         (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_burstcount),    //                          .burstcount
		.uav_read               (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_read),          //                          .read
		.uav_write              (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_write),         //                          .write
		.uav_waitrequest        (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_waitrequest),   //                          .waitrequest
		.uav_readdatavalid      (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_readdatavalid), //                          .readdatavalid
		.uav_byteenable         (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_byteenable),    //                          .byteenable
		.uav_readdata           (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_readdata),      //                          .readdata
		.uav_writedata          (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_writedata),     //                          .writedata
		.uav_lock               (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_lock),          //                          .lock
		.uav_debugaccess        (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_debugaccess),   //                          .debugaccess
		.uav_clken              (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_clken),         //                          .clken
		.av_address             (nios_0_tightly_coupled_instruction_master_0_address),                                            //      avalon_anti_master_0.address
		.av_read                (nios_0_tightly_coupled_instruction_master_0_read),                                               //                          .read
		.av_readdata            (nios_0_tightly_coupled_instruction_master_0_readdata),                                           //                          .readdata
		.av_clken               (nios_0_tightly_coupled_instruction_master_0_clken),                                              //                          .clken
		.av_waitrequest         (),                                                                                               //               (terminated)
		.av_burstcount          (1'b1),                                                                                           //               (terminated)
		.av_byteenable          (4'b1111),                                                                                        //               (terminated)
		.av_beginbursttransfer  (1'b0),                                                                                           //               (terminated)
		.av_begintransfer       (1'b0),                                                                                           //               (terminated)
		.av_chipselect          (1'b0),                                                                                           //               (terminated)
		.av_readdatavalid       (),                                                                                               //               (terminated)
		.av_write               (1'b0),                                                                                           //               (terminated)
		.av_writedata           (32'b00000000000000000000000000000000),                                                           //               (terminated)
		.av_lock                (1'b0),                                                                                           //               (terminated)
		.av_debugaccess         (1'b0),                                                                                           //               (terminated)
		.uav_response           (2'b00),                                                                                          //               (terminated)
		.av_response            (),                                                                                               //               (terminated)
		.uav_writeresponsevalid (1'b0),                                                                                           //               (terminated)
		.av_writeresponsevalid  ()                                                                                                //               (terminated)
	);

	altera_merlin_slave_translator #(
		.AV_ADDRESS_W                   (13),
		.AV_DATA_W                      (32),
		.UAV_DATA_W                     (32),
		.AV_BURSTCOUNT_W                (1),
		.AV_BYTEENABLE_W                (4),
		.UAV_BYTEENABLE_W               (4),
		.UAV_ADDRESS_W                  (15),
		.UAV_BURSTCOUNT_W               (3),
		.AV_READLATENCY                 (1),
		.USE_READDATAVALID              (0),
		.USE_WAITREQUEST                (0),
		.USE_UAV_CLKEN                  (1),
		.USE_READRESPONSE               (0),
		.USE_WRITERESPONSE              (0),
		.AV_SYMBOLS_PER_WORD            (4),
		.AV_ADDRESS_SYMBOLS             (0),
		.AV_BURSTCOUNT_SYMBOLS          (0),
		.AV_CONSTANT_BURST_BEHAVIOR     (0),
		.UAV_CONSTANT_BURST_BEHAVIOR    (0),
		.AV_REQUIRE_UNALIGNED_ADDRESSES (0),
		.CHIPSELECT_THROUGH_READLATENCY (0),
		.AV_READ_WAIT_CYCLES            (0),
		.AV_WRITE_WAIT_CYCLES           (0),
		.AV_SETUP_WAIT_CYCLES           (0),
		.AV_DATA_HOLD_CYCLES            (0)
	) instruction_rom_0_s1_translator (
		.clk                    (clk_0_clk_clk),                                                                                  //                      clk.clk
		.reset                  (instruction_rom_0_reset1_reset_bridge_in_reset_reset),                                           //                    reset.reset
		.uav_address            (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_address),       // avalon_universal_slave_0.address
		.uav_burstcount         (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_burstcount),    //                         .burstcount
		.uav_read               (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_read),          //                         .read
		.uav_write              (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_write),         //                         .write
		.uav_waitrequest        (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_waitrequest),   //                         .waitrequest
		.uav_readdatavalid      (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_readdatavalid), //                         .readdatavalid
		.uav_byteenable         (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_byteenable),    //                         .byteenable
		.uav_readdata           (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_readdata),      //                         .readdata
		.uav_writedata          (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_writedata),     //                         .writedata
		.uav_lock               (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_lock),          //                         .lock
		.uav_debugaccess        (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_debugaccess),   //                         .debugaccess
		.uav_clken              (nios_0_tightly_coupled_instruction_master_0_translator_avalon_universal_master_0_clken),         //                         .clken
		.av_address             (instruction_rom_0_s1_address),                                                                   //      avalon_anti_slave_0.address
		.av_write               (instruction_rom_0_s1_write),                                                                     //                         .write
		.av_readdata            (instruction_rom_0_s1_readdata),                                                                  //                         .readdata
		.av_writedata           (instruction_rom_0_s1_writedata),                                                                 //                         .writedata
		.av_byteenable          (instruction_rom_0_s1_byteenable),                                                                //                         .byteenable
		.av_chipselect          (instruction_rom_0_s1_chipselect),                                                                //                         .chipselect
		.av_clken               (instruction_rom_0_s1_clken),                                                                     //                         .clken
		.av_read                (),                                                                                               //              (terminated)
		.av_begintransfer       (),                                                                                               //              (terminated)
		.av_beginbursttransfer  (),                                                                                               //              (terminated)
		.av_burstcount          (),                                                                                               //              (terminated)
		.av_readdatavalid       (1'b0),                                                                                           //              (terminated)
		.av_waitrequest         (1'b0),                                                                                           //              (terminated)
		.av_writebyteenable     (),                                                                                               //              (terminated)
		.av_lock                (),                                                                                               //              (terminated)
		.av_debugaccess         (),                                                                                               //              (terminated)
		.av_outputenable        (),                                                                                               //              (terminated)
		.uav_response           (),                                                                                               //              (terminated)
		.av_response            (2'b00),                                                                                          //              (terminated)
		.uav_writeresponsevalid (),                                                                                               //              (terminated)
		.av_writeresponsevalid  (1'b0)                                                                                            //              (terminated)
	);

endmodule
