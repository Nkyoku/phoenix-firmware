/**
 * @file current_fir_filter.sv
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

// サンプリング周波数 Fs = 50kHz 
// カットオフ周波数   Fc = 4kHz (-3dB)

module current_fir_filter #(
        parameter int DATA_WIDTH = 16,
        parameter int DATA_COUNT = 1
    ) (
        input  wire                             clk,
        input  wire                             reset,
        input  wire [DATA_COUNT*DATA_WIDTH-1:0] in_data, // {x0, x1, ...}
        input  wire                             in_valid,
        output wire                             in_ready,
        output reg  [DATA_COUNT*DATA_WIDTH-1:0] out_data, // {y0, y1, ...}
        output reg                              out_valid,
        input  wire                             out_ready
    );
    
    localparam int COEF_WIDTH = 16;
    localparam int COEF_COUNT = 8;
    localparam int TAP = 7;
    localparam int TAP_WIDTH = DATA_WIDTH + $clog2(TAP);
    localparam int DATA_MIN = -2**(DATA_WIDTH - 1);
    localparam int DATA_MAX = 2**(DATA_WIDTH - 1) - 1;
    
    localparam int INDEX_MAX = DATA_COUNT * TAP + 1;
    localparam int INDEX_WIDTH = $clog2(INDEX_MAX + 1);
    
    // Coefficients memory Q16
    logic signed [COEF_WIDTH-1:0] coefficients [0:COEF_COUNT-1] = '{
        1966, 8263, 13986, 17106, 13986, 8263, 1966, 0
    };
    
    // Multiply and Add
    logic signed [DATA_WIDTH-1:0] mult_in1 = '0;
    logic signed [COEF_WIDTH-1:0] mult_in2 = '0;
    logic signed [TAP_WIDTH-1:0] add_in = '0;
    wire signed [TAP_WIDTH+COEF_WIDTH-1:0] mult_out = mult_in1 * mult_in2 + 2**(COEF_WIDTH - 1); // +2**(COEF_WIDTH-1)で四捨五入する
    logic signed [TAP_WIDTH-1:0] mac_out = '0;
    //wire signed [DATA_WIDTH-1:0] mac_out_sat = (mac_out < DATA_MIN) ? DATA_WIDTH'(DATA_MIN) : ((DATA_MAX < mac_out) ? DATA_WIDTH'(DATA_MAX) : mac_out[DATA_WIDTH-1:0]);
    always @(posedge clk) begin
        mac_out <= $signed(mult_out[COEF_WIDTH+:DATA_WIDTH]) + add_in;
    end
    
    // Delay memory
    logic signed [TAP_WIDTH-1:0] az [0:TAP*DATA_COUNT-1];
    initial begin // Set initial value
        int i;
        for (i = 0; i < (TAP*DATA_COUNT); i = i + 1) begin
            az[i] = '0;
        end
    end
    logic [INDEX_WIDTH-1:0] index = '0;
    wire [INDEX_WIDTH-1:0] read_address = index;
    wire read_valid = (read_address < (DATA_COUNT * TAP));
    wire [INDEX_WIDTH-1:0] write_address = index - INDEX_WIDTH'(DATA_COUNT + 2);
    wire write_enable = (write_address < (DATA_COUNT * (TAP - 1)));
    always @(posedge clk) begin
        int i;
        mult_in2 <= coefficients[(read_address / DATA_COUNT) % COEF_COUNT];
        if (read_valid == 1'b1) begin
            add_in <= az[read_address];
        end
        if (write_enable == 1'b1) begin
            az[write_address] <= mac_out;
        end
        for (i = 0; i < DATA_COUNT; i = i + 1) begin
            if (index == (i + 2)) begin
                out_data[DATA_WIDTH * i +: DATA_WIDTH] <= mac_out[DATA_WIDTH-1:0];//mac_out_sat; この係数では飽和演算は不要
            end
        end
    end
    
    // Avalon-ST
    logic [DATA_WIDTH*DATA_COUNT-1:0] x;// [0:DATA_COUNT-1];
    assign in_ready = (index == 0) & ~out_valid;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            out_valid <= 1'b0;
            index <= '0;
        end
        else begin
            if (in_valid & in_ready) begin
                index <= 1'b1;
            end
            else if (index != 0) begin
                if (index < INDEX_MAX) begin
                    index <= index + 1'b1;
                end
                else begin
                    index <= '0;
                end
            end
            if (in_valid & in_ready) begin
                x <= in_data;
                mult_in1 <= in_data[DATA_WIDTH-1:0];
            end
            else begin
                mult_in1 <= x[DATA_WIDTH * (index % DATA_COUNT) +: DATA_WIDTH];
            end
            if (index == (DATA_COUNT + 1)) begin
                out_valid <= 1'b1;
            end
            else if (out_valid & out_ready) begin
                out_valid <= 1'b0;
            end
        end
    end
endmodule
