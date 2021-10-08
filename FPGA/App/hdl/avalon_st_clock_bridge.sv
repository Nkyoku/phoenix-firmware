/**
 * @file avalon_st_clock_bridge.sv
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

(* altera_attribute = {"-name SDC_STATEMENT \"set_false_path -from [get_registers *avalon_st_clock_bridge:*|source_data[*]]\"; -name SDC_STATEMENT \"set_false_path -from [get_registers *avalon_st_clock_bridge:*|source_channel[*]]\"; -name SDC_STATEMENT \"set_false_path -from [get_registers *avalon_st_clock_bridge:*|req] -to [get_registers *avalon_st_clock_bridge:*|req_ff1]\"; -name SDC_STATEMENT \"set_false_path -from [get_registers *avalon_st_clock_bridge:*|ack] -to [get_registers *avalon_st_clock_bridge:*|ack_ff1]\""} *)
module avalon_st_clock_bridge #(
        parameter DATA_WIDTH = 8,
        parameter CHANNEL_WIDTH = 1
    ) (
        input  wire clk1,
        input  wire reset1,
        input  wire [DATA_WIDTH-1:0] sink_data,
        input  wire [CHANNEL_WIDTH-1:0] sink_channel,
        input  wire sink_valid,
        input  wire sink_error,
        output wire sink_ready,
        input  wire clk2,
        input  wire reset2,
        output reg  [DATA_WIDTH-1:0] source_data,
        output reg  [CHANNEL_WIDTH-1:0] source_channel,
        output reg  source_valid,
        output reg  source_error,
        input  wire source_ready
    );
    
    reg req;
    reg req_ff1;
    reg req_ff2;
    reg ack;
    reg ack_ff1;
    reg ack_ff2;
    assign sink_ready = ~req & ~ack_ff2;
    
    always @(posedge clk1, posedge reset1) begin
        if (reset1 == 1'b1) begin
            req <= 1'b0;
            ack_ff1 <= 1'b0;
            ack_ff2 <= 1'b0;
            source_data <= '0;
            source_channel <= '0;
            source_error <= 1'b0;
        end
        else begin
            req <= (sink_valid | req) & ~ack_ff2;
            ack_ff1 <= ack;
            ack_ff2 <= ack_ff1;
            if (sink_valid & sink_ready) begin
                source_data <= sink_data;
                source_channel <= sink_channel;
                source_error <= sink_error;
            end
        end
    end
    
    always @(posedge clk2, posedge reset2) begin
        if (reset2 == 1'b1) begin
            req_ff1 <= 1'b0;
            req_ff2 <= 1'b0;
            ack <= 1'b0;
            source_valid <= 1'b0;
        end
        else begin
            req_ff1 <= req;
            req_ff2 <= req_ff1;
            ack <= (source_valid | req_ff2) | (source_valid & source_ready);
            source_valid <= (~source_valid & req_ff2 & ~ack) | (source_valid & ~source_ready);
        end
    end
endmodule
