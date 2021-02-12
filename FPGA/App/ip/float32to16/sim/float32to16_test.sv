`timescale 1 ns / 1 ns

// FP16をFP32に変換する
// ソフトウェア向けコードを変換したもの
function [31:0] to_fp32(input [15:0] fp16);
    logic unsigned [31:0] sign_16;
    logic unsigned [31:0] exp_16;
    logic unsigned [31:0] coef_16;
    begin
        sign_16 = {fp16[15], 31'h00000000};
        exp_16 = {27'h0000000, fp16[14:10]};
        coef_16 = {9'h000, fp16[9:0], 13'h0000};
        if (exp_16 == 31) begin
            if (coef_16 == 0) begin
                // Inf
                to_fp32 = sign_16 | 32'h7F800000 | coef_16;
            end
            else begin
                // NaN
                to_fp32 = sign_16 | 32'h7FC00000 | coef_16;
            end
        end
        else if ((exp_16 == 0) & (coef_16 == 0)) begin
            // Zero
            to_fp32 = sign_16;
        end
        else begin
            if (exp_16 == 0) begin
                // Normalize Denormalized Numbers
                exp_16 = exp_16 + 1;
                while ((coef_16 & 32'h7F800000) == 0) begin
                    coef_16 = coef_16 << 1;
                    exp_16 = exp_16 - 1;
                end
                coef_16 = coef_16 & 32'h007FFFFF;
            end
            to_fp32 = sign_16 | ((exp_16 + 112) << 23) | coef_16;
        end
    end
endfunction

// FP32をFP16に変換する
// ソフトウェア向けコードを変換したもの
function [15:0] to_fp16(input [31:0] fp32);
    logic unsigned [15:0] sign_16;
    logic unsigned [31:0] exp_32;
    logic unsigned [31:0] coef_32;
    begin
        sign_16 = {fp32[31], 15'h0000};
        exp_32 = {1'b0, fp32[30:23], 23'h000000};
        coef_32 = {9'h000, fp32[22:0]};
        if (exp_32 == 32'h7F800000) begin
            if (coef_32 != '0) begin
                // NaN
                to_fp16 = sign_16 | 16'h7E00 | (coef_32 >> 13);
            end
            else begin
                // Inf
                to_fp16 = sign_16 | 16'h7C00 | (coef_32 >> 13);
            end
        end
        else begin
            logic signed [31:0] exp;
            exp = (exp_32 >> 23) - 112;
            if (31 <= exp) begin
                // 大きすぎる値をInfに変換
                to_fp16 = sign_16 | 16'h7C00;
            end
            else if (exp <= 0) begin
                if (exp < -10) begin
                    // 小さすぎる値を0に変換
                    to_fp16 = sign_16;
                end
                else begin
                    // 非正規化数に変換
                    logic unsigned [31:0] c;
                    logic unsigned [15:0] coef_16;
                    logic unsigned [31:0] round_bit;
                    c = coef_32 | 32'h00800000;
                    coef_16 = c >> (14 - exp);
                    round_bit = 1 << (13 - exp);
                    if (((c & round_bit) != 0) & ((c & (3 * round_bit - 1)) != 0)) begin
                        coef_16 = coef_16 + 1;
                    end
                    to_fp16 = sign_16 | coef_16;
                end
            end
            else begin
                // 正規化数に変換
                logic unsigned [15:0] coef_16;
                logic unsigned [31:0] round_bit;
                coef_16 = coef_32 >> 13;
                round_bit = 32'h00001000;
                if (((coef_32 & round_bit) != 0) & ((coef_32 & (3 * round_bit - 1)) != 0)) begin
                    to_fp16 = (sign_16 | (exp << 10) | coef_16) + 1;
                end
                else begin
                    to_fp16 = sign_16 | (exp << 10) | coef_16;
                end
            end
        end
    end
endfunction

module test ();
    logic reset = 1'b1;
    logic clk = 1'b0;
    logic clken = 1'b0;
    logic start = 1'b0;
    logic done;
    logic [31:0] dataa;
    logic [31:0] result;
    logic [15:0] result_latch;
    
    float32to16 uut (
        .slave_reset(reset),
        .slave_clk(clk),
        .slave_clken(clken),
        .slave_start(start),
        .slave_done(done),
        .slave_dataa(dataa),
        .slave_result(result)
    );
    
    // テストシーケンス
    initial begin
        @(negedge reset);
        repeat(10) @(posedge clk);
        
        // テスト1
        // +0から+InfのFP16の値をFP32に変換した値がFP16に戻せることを確認する
        for (int i = 0; i <= 31744; i++) begin
            logic [15:0] fp16_input;
            logic [31:0] fp32;
            fp16_input = i;
            fp32 = to_fp32(fp16_input);
            execute(fp32);
            if (fp16_input !== result_latch) begin
                $display("Mismatch : in=0x%04X, out=0x%04X", fp16_input, result_latch);
                $stop;
            end
        end
        $display("Test 1 was Finished without error");

        // テスト2
        // 小さい正規化数が0になることと、大きい正規化数がInfになることを確認する
        for (int i = -126; i <= 127; i++) begin
            logic [31:0] fp32;
            logic [15:0] proper_fp16;
            fp32 = {1'b0, 8'(i + 127), 23'h000000};
            execute(fp32);
            if (i < -25) begin
                // 0になる
                proper_fp16 = 16'h0000;
            end
            else if (i == -25) begin
                // 切り捨てられて0になる
                proper_fp16 = 16'h0000;
            end
            else if (i < -15) begin
                // 非正規化数になる
                proper_fp16 = {6'h00, 10'(1 << (24 + i))};
            end
            else if (i == -15) begin
                // 非正規化数になる
                proper_fp16 = {1'b0, 5'h00, 10'h200};
            end
            else if (i < 15) begin
                // 正規化数になる
                proper_fp16 = {1'b0, 5'(i + 15), 10'h000};
            end
            else if (i == 15) begin
                // 正規化数になる
                proper_fp16 = {1'b0, 5'h1E, 10'h000};
            end
            else begin
                // Infになる
                proper_fp16 = 16'h7C00;
            end
            if (result_latch !== proper_fp16) begin
                $display("Error : in=0x%04X, result=0x%04X, proper=%04X", fp32, result_latch, proper_fp16);
                $stop;
            end
        end
        $display("Test 2 was Finished without error");

        // テスト3
        // 切り上げ有で小さい正規化数が0になることと大きい正規化数がInfになることを確認する
        for (int i = -126; i <= 127; i++) begin
            logic [31:0] fp32;
            logic [15:0] proper_fp16;
            fp32 = {1'b0, 8'(i + 127), 23'h7FFFFF};
            execute(fp32);
            if (i < -25) begin
                // 0になる
                proper_fp16 = 16'h0000;
            end
            else if (i == -25) begin
                // 切り上げられて非正規化数になる
                proper_fp16 = 16'h0001;
            end
            else if (i < -15) begin
                // 非正規化数になる
                proper_fp16 = {1'b0, 5'h00, 10'(1 << (25 + i))};
            end
            else if (i == -15) begin
                // 切り上げられて正規化数になる
                proper_fp16 = {1'b0, 5'h01, 10'h000};
            end
            else if (i < 15) begin
                // 正規化数になる
                proper_fp16 = {1'b0, 5'(i + 16), 10'h000};
            end
            else if (i == 15) begin
                // 切り上げられてInfになる
                proper_fp16 = 16'h7C00;
            end
            else begin
                // Infになる
                proper_fp16 = 16'h7C00;
            end
            if (result_latch !== proper_fp16) begin
                $display("Error : in=0x%04X, result=0x%04X, proper=%04X", fp32, result_latch, proper_fp16);
                $stop;
            end
        end
        $display("Test 3 was Finished without error");

        $display("Simulation End");
        $stop;
    end
    
    task execute(input [31:0] fp32);
        clken <= 1'b1;
        start <= 1'b1;
        dataa <= fp32;
        @(posedge clk);
        start <= 1'b0;
        while (done == 1'b0) begin
            @(posedge clk);
        end
        @(posedge clk);
    endtask

    always @(posedge clk) begin
        if (reset == 1'b1) begin
            result_latch <= 'X;
        end
        else begin
            if (done == 1'b1) begin
                result_latch <= result[15:0];
                dataa <= 'X;
                clken <= 1'b0;
                if (clken == 1'b0) begin
                    $display("Error : done==1 instead of clken==0");
                    $stop;
                end
            end
        end
    end
    
    // Clock Generation
    always #5ns begin
        clk <= ~clk;
    end
    
    // Reset Generaton
    initial begin
        reset <= 1'b1;
        repeat(3) @(posedge clk);
        reset <= 1'b0;
    end
endmodule
