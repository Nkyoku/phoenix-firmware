/**
 * @file clarke_transform.sv
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

module clarke_transform #(
        parameter int CHANNEL_WIDTH = 1,
        parameter int DATA_WIDTH = 16
    ) (
        input  wire                     clk,
        input  wire                     reset,
        input  wire [2*DATA_WIDTH-1:0]  in_data, // {u[DATA_WIDTH-1:0], v[DATA_WIDTH-1:0]}
        input  wire [CHANNEL_WIDTH-1:0] in_channel,
        input  wire                     in_valid,
        output reg                      in_ready,
        output wire [2*DATA_WIDTH-1:0]  out_data, // {a[DATA_WIDTH-1:0], b[DATA_WIDTH-1:0]}
        output wire [CHANNEL_WIDTH-1:0] out_channel,
        output reg                      out_valid,
        input  wire                     out_ready
    );
    
    localparam int SCALE = 12;
    localparam int K_WIDTH = SCALE + 2;
    localparam int DATA_MAX = 2**(DATA_WIDTH - 1) - 1;
    localparam int DATA_MIN = -2**(DATA_WIDTH - 1);
    
    logic signed [K_WIDTH-1:0] k_a = 5017; // sqrt(3 / 2)
    logic signed [K_WIDTH-1:0] k_b = 2896; // sqrt(1 / 2)
    
    logic [3:0] state = '0;
    logic signed [DATA_WIDTH-1:0] u = '0;
    logic signed [DATA_WIDTH-1:0] v = '0;
    logic [CHANNEL_WIDTH-1:0] channel = '0;
    logic signed [DATA_WIDTH-1:0] a = '0;
    logic signed [DATA_WIDTH-1:0] b = '0;
    assign out_data = {a, b};
    assign out_channel = channel;
    
    // Calculate a <= u * k_a
    // Calculate b <= (u + 2 * v) * k_b
    logic signed [DATA_WIDTH+1:0] mult_in1 = '0;
    logic signed [K_WIDTH-1:0] mult_in2 = '0;
    logic signed [K_WIDTH+DATA_WIDTH+1:0] mult_out = '0;
    wire  signed [K_WIDTH+DATA_WIDTH-SCALE+1:0] ab = mult_out[K_WIDTH+DATA_WIDTH+1:SCALE];
    wire  signed [DATA_WIDTH-1:0] ab_sat = (ab < DATA_MIN) ? DATA_WIDTH'(DATA_MIN) : ((DATA_MAX < ab) ? DATA_WIDTH'(DATA_MAX) : ab[DATA_WIDTH-1:0]);
    always @(posedge clk) begin
        if (state[0] == 1'b1) begin
            mult_in1 <= u;
            mult_in2 <= k_a;
        end
        else begin
            mult_in1 <= (DATA_WIDTH+2)'(u) + {(DATA_WIDTH+1)'(v), 1'b0}; // mult_in1 <= u + 2 * v
            mult_in2 <= k_b;
        end
        mult_out <= mult_in1 * mult_in2;
        if (state[2] == 1'b1) begin
            a <= ab_sat;
        end
        if (state[3] == 1'b1) begin
            b <= ab_sat;
        end
    end
    
    // Avalon-ST
    logic assert_ready = 1'b1;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            out_valid <= 1'b0;
            in_ready <= 1'b0;
            state <= '0;
            assert_ready <= 1'b1;
            u <= '0;
            v <= '0;
            channel <= '0;
        end
        else begin
            assert_ready <= 1'b0;
            if (assert_ready == 1'b1) begin
                in_ready <= 1'b1;
            end
            state <= {state[$bits(state)-2:0], 1'b0};
            if (in_valid & in_ready) begin
                u <= in_data[DATA_WIDTH+:DATA_WIDTH];
                v <= in_data[0+:DATA_WIDTH];
                channel <= in_channel;
                state[0] <= 1'b1;
                in_ready <= 1'b0;
            end
            if (out_valid & out_ready) begin
                out_valid <= 1'b0;
                in_ready <= 1'b1;
            end
            if (state[3] == 1'b1) begin
                out_valid <= 1'b1;
            end
        end
    end
endmodule
