/**
 * @file park_transform.sv
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

module park_transform #(
        parameter int CHANNEL_WIDTH = 1,
        parameter int DATA_WIDTH = 16,
        parameter int DATA_MAX = 2**(DATA_WIDTH - 1) - 1,
        parameter int DATA_MIN = -2**(DATA_WIDTH - 1),
        parameter int THETA_WIDTH = 9
    ) (
        input  wire                              clk,
        input  wire                              reset,
        input  wire [THETA_WIDTH+2*DATA_WIDTH:0] in_data, // {inverse, theta[THETA_WIDTH-1:0], x[DATA_WIDTH-1:0], y[DATA_WIDTH-1:0]}
        input  wire [CHANNEL_WIDTH-1:0]          in_channel,
        input  wire                              in_valid,
        output reg                               in_ready,
        output wire [2*DATA_WIDTH-1:0]           out_data, // {x[DATA_WIDTH-1:0], y[DATA_WIDTH-1:0]}
        output wire [CHANNEL_WIDTH-1:0]          out_channel,
        output reg                               out_valid,
        input  wire                              out_ready
    );
    
    localparam int TABLE_WIDTH = 12;
    localparam int TABLE_SCALE = 10;
        
    // Sine Table
    logic unsigned [TABLE_WIDTH-1:0] table_rom [0:2**(THETA_WIDTH-1)-1] = '{
        0, 13, 25, 38, 50, 63, 75, 88, 100, 113, 125, 138, 150, 163, 175, 187, 
        200, 212, 224, 237, 249, 261, 273, 285, 297, 309, 321, 333, 345, 357, 369, 380, 
        392, 403, 415, 426, 438, 449, 460, 472, 483, 494, 505, 516, 526, 537, 548, 558, 
        569, 579, 590, 600, 610, 620, 630, 640, 650, 659, 669, 678, 688, 697, 706, 715, 
        724, 733, 742, 750, 759, 767, 775, 784, 792, 799, 807, 815, 822, 830, 837, 844, 
        851, 858, 865, 872, 878, 885, 891, 897, 903, 909, 915, 920, 926, 931, 936, 941, 
        946, 951, 955, 960, 964, 968, 972, 976, 980, 983, 987, 990, 993, 996, 999, 1002, 
        1004, 1007, 1009, 1011, 1013, 1015, 1016, 1018, 1019, 1020, 1021, 1022, 1023, 1023, 1024, 1024, 
        1024, 1024, 1024, 1023, 1023, 1022, 1021, 1020, 1019, 1018, 1016, 1015, 1013, 1011, 1009, 1007, 
        1004, 1002, 999, 996, 993, 990, 987, 983, 980, 976, 972, 968, 964, 960, 955, 951, 
        946, 941, 936, 931, 926, 920, 915, 909, 903, 897, 891, 885, 878, 872, 865, 858, 
        851, 844, 837, 830, 822, 815, 807, 799, 792, 784, 775, 767, 759, 750, 742, 733, 
        724, 715, 706, 697, 688, 678, 669, 659, 650, 640, 630, 620, 610, 600, 590, 579, 
        569, 558, 548, 537, 526, 516, 505, 494, 483, 472, 460, 449, 438, 426, 415, 403, 
        392, 380, 369, 357, 345, 333, 321, 309, 297, 285, 273, 261, 249, 237, 224, 212, 
        200, 187, 175, 163, 150, 138, 125, 113, 100, 88, 75, 63, 50, 38, 25, 13
    };
    logic [THETA_WIDTH-2:0] table_address;
    logic unsigned [TABLE_WIDTH-1:0] table_data;
    always @(posedge clk) begin
        table_data <= table_rom[table_address];
    end
    
    // Multiplier
    logic signed [DATA_WIDTH-1:0] mult_in1;
    wire  signed [TABLE_WIDTH-1:0] mult_in2 = table_data;
    logic signed [TABLE_WIDTH+DATA_WIDTH-1:0] mult_out_raw;
    wire  signed [TABLE_WIDTH+DATA_WIDTH-TABLE_SCALE:0] mult_out = mult_out_raw[TABLE_WIDTH+DATA_WIDTH-1:TABLE_SCALE-1];
    always @(posedge clk) begin
        mult_out_raw <= mult_in1 * mult_in2;
    end
    
    logic [2:0] index = '0;
    logic inverse;
    logic unsigned [THETA_WIDTH-1:0] theta;
    wire  unsigned [THETA_WIDTH-1:0] theta_cos = theta + THETA_WIDTH'(2**(THETA_WIDTH-2));
    logic signed [DATA_WIDTH-1:0] x, y;
    logic [CHANNEL_WIDTH-1:0] channel;
    wire inverse_sin = inverse ^ theta[THETA_WIDTH-1];
    wire inverse_cos = theta[THETA_WIDTH-1] ^ theta[THETA_WIDTH-2];
    
    logic signed [$bits(mult_out):0] sum;
    wire  signed [DATA_WIDTH:0] out_unsat = sum[1+:DATA_WIDTH+1];
    wire  signed [DATA_WIDTH:0] out_sat = (out_unsat < DATA_MIN) ? DATA_WIDTH'(DATA_MIN) : ((DATA_MAX < out_unsat) ? DATA_WIDTH'(DATA_MAX) : out_unsat);
    logic signed [DATA_WIDTH-1:0] out_x;
    logic signed [DATA_WIDTH-1:0] out_y;
    
    always @(posedge clk) begin
        // sinテーブルを参照する
        if (in_valid & in_ready) begin
            table_address <= in_data[2*DATA_WIDTH+:THETA_WIDTH-1];
        end
        else if (index == 1) begin
            table_address <= theta_cos[THETA_WIDTH-2:0];
        end
        else if (index == 2) begin
            table_address <= theta[THETA_WIDTH-2:0];
        end
        else if (index == 3) begin
            table_address <= theta_cos[THETA_WIDTH-2:0];
        end
        else begin
            table_address <= 'X;
        end
        
        // 入力とsin,cosを乗算する
        if (index == 1) begin
            mult_in1 <= ~inverse_sin ? y : -y;
        end
        else if (index == 2) begin
            mult_in1 <= ~inverse_cos ? x : -x;
        end
        else if (index == 3) begin
            mult_in1 <= ~inverse_sin ? -x : x;
        end
        else if (index == 4) begin
            mult_in1 <= ~inverse_cos ? y : -y;
        end
        else begin
            mult_in1 <= 'X;
        end
        
        // 乗算結果を加算する
        if (index == 3) begin
            sum <= mult_out; // y * sin(theta)
        end
        else if (index == 4) begin
            sum <= sum + mult_out; // x * cos(theta)
        end
        else if (index == 5) begin
            sum <= mult_out; // -x * sin(theta)
        end
        else if (index == 6) begin
            sum <= sum + mult_out; // y * cos(theta)
        end
        else begin
            sum <= 'X;
        end
    end
    
    // Avalon-ST
    logic assert_ready = 1'b1;
    assign out_data = {out_x, out_y};
    assign out_channel = channel;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            out_valid <= 1'b0;
            in_ready <= 1'b0;
            index <= '0;
            assert_ready <= 1'b1;
            inverse <= 1'b0;
            theta <= '0;
            x <= '0;
            y <= '0;
            channel <= '0;
            out_x <= '0;
            out_y <= '0;
        end
        else begin
            assert_ready <= 1'b0;
            if (assert_ready == 1'b1) begin
                in_ready <= 1'b1;
            end
            else if (in_valid & in_ready) begin
                in_ready <= 1'b0;
            end
            else if (out_valid & out_ready) begin
                in_ready <= 1'b1;
            end
            
            if (in_valid & in_ready) begin
                inverse <= in_data[2*DATA_WIDTH+THETA_WIDTH];
                theta <= in_data[2*DATA_WIDTH+:THETA_WIDTH];
                x <= in_data[DATA_WIDTH+:DATA_WIDTH];
                y <= in_data[0+:DATA_WIDTH];
                channel <= in_channel;
                in_ready <= 1'b0;
                index <= 1'b1;
            end
            else if (index != 0) begin
                index <= (index < 7) ? (index + 1'b1) : '0;
            end
            
            if (index == 7) begin
                out_valid <= 1'b1;
            end
            else if (out_valid & out_ready) begin
                out_valid <= 1'b0;
            end
            
            // 飽和演算を行って出力する
            if (index == 5) begin
                out_x <= out_sat[DATA_WIDTH-1:0];
            end
            if (index == 7) begin
                out_y <= out_sat[DATA_WIDTH-1:0];
            end
        end
    end
endmodule
