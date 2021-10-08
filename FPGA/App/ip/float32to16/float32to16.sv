/**
 * float32to16
 *
 * Copyright (c) 2021 Fujii Naomichi
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 *
 * SPDX-License-Identifier: MIT
 */

module float32to16 (
        input  wire        slave_reset,
        input  wire        slave_clk,
        input  wire        slave_clk_en,
        input  wire        slave_start,
        output reg         slave_done,
        input  wire [31:0] slave_dataa,
        output wire [31:0] slave_result
    );

    logic sign_32;
    logic unsigned [7:0] exp_32;
    logic unsigned [22:0] coef_32;
    assign {sign_32, exp_32, coef_32} = slave_dataa;

    logic sign_16;
    logic unsigned [4:0] exp_16;
    logic unsigned [9:0] coef_16;
    assign slave_result = {16'h0000, sign_16, exp_16, coef_16};

    logic unsigned [23:0] round_target;
    logic unsigned [9:0] round_target_shifted;
    logic unsigned [4:0] round_offset;
    logic unsigned [23:0] round_mask1;
    logic unsigned [23:0] round_mask2;

    logic [1:0] state;

    always @(posedge slave_clk) begin
        if (slave_reset == 1'b1) begin
            slave_done <= 1'b0;
            sign_16 <= 1'bX;
            exp_16 <= 'X;
            coef_16 <= 'X;
            state <= '0;

        end
        else if (slave_clk_en == 1'b1) begin
            slave_done <= 1'b0;
            state[1] <= state[0];
            state[0] <= 1'b0;
            sign_16 <= sign_32;
            if (slave_start == 1'b1) begin
                if (exp_32 == 8'hFF) begin
                    // NaN or Inf
                    exp_16 <= '1;
                    coef_16 <= ((coef_32 != '0) ? 10'h200 : 10'h000) | coef_32[22:13];
                    slave_done <= 1'b1;
                end
                else if (143 <= exp_32) begin
                    // Inf
                    exp_16 <= '1;
                    coef_16 <= '0;
                    slave_done <= 1'b1;
                end
                else if (exp_32 < 102) begin
                    // Zero
                    exp_16 <= '0;
                    coef_16 <= '0;
                    slave_done <= 1'b1;
                end
                else if (exp_32 <= 112) begin
                    // Convert to Denormalized Number
                    exp_16 <= '0;
                    coef_16 <= 'X;
                    round_offset <= 125 - exp_32;
                    round_target <= {1'b1, coef_32};
                    state[0] <= 1'b1;
                end
                else begin
                    // Convert to Normalized Number
                    exp_16 <= exp_32 - 112;
                    coef_16 <= 'X;
                    round_offset <= 12;
                    round_target <= {1'b0, coef_32};
                    state[0] <= 1'b1;
                end
            end
            else if (state[1] == 1'b1) begin
                if (((round_target & round_mask1) != 0) && ((round_target & round_mask2) != 0)) begin
                    exp_16 <= exp_16 + &round_target_shifted;
                    coef_16 <= round_target_shifted + 1'b1;
                end
                else begin
                    coef_16 <= round_target_shifted;
                end
                slave_done <= 1'b1;
            end
        end
    end

    // Shift Coeffient
    always @(posedge slave_clk) begin
        if (~slave_reset & slave_clk_en) begin
            case (round_offset)
                12 : round_target_shifted <= round_target >> 13;
                13 : round_target_shifted <= round_target >> 14;
                14 : round_target_shifted <= round_target >> 15;
                15 : round_target_shifted <= round_target >> 16;
                16 : round_target_shifted <= round_target >> 17;
                17 : round_target_shifted <= round_target >> 18;
                18 : round_target_shifted <= round_target >> 19;
                19 : round_target_shifted <= round_target >> 20;
                20 : round_target_shifted <= round_target >> 21;
                21 : round_target_shifted <= round_target >> 22;
                22 : round_target_shifted <= round_target >> 23;
                23 : round_target_shifted <= round_target >> 24;
                default : round_target_shifted <= 'X;
            endcase
        end
    end

    // Generate Round Bit Mask
    int i;
    always @(round_offset) begin
        round_mask1[11:0] <= '0;
        round_mask2[11:0] <= '1;
        for (i = 12; i <= 23; i++) begin
            round_mask1[i] <= (i == round_offset) ? 1'b1 : 1'b0;
            round_mask2[i] <= ((i == (round_offset + 1)) | (i < round_offset)) ? 1'b1 : 1'b0;
        end
    end
endmodule
