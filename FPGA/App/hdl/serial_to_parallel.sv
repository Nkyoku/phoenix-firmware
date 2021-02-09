module SerialToParallel #(
        parameter PRESCALER_WIDTH = 1,
        parameter INVERSE = 0
    ) (
        input  wire reset,
        input  wire clk,
        output reg  sr_load_n,
        output reg  sr_clk,
        input  wire sr_dout,
        output wire input_ser,
        output wire input_a,
        output wire input_b,
        output wire input_c,
        output wire input_d,
        output wire input_e,
        output wire input_f,
        output wire input_g,
        output wire input_h
    );
    
    reg [PRESCALER_WIDTH-1:0] prescaler = '0;
    reg [3:0] counter = '0;
    reg [8:0] sr_ff = '0;
    reg [8:0] sr_input = '0;
    assign input_ser = sr_input[0];
    assign input_a = sr_input[1];
    assign input_b = sr_input[2];
    assign input_c = sr_input[3];
    assign input_d = sr_input[4];
    assign input_e = sr_input[5];
    assign input_f = sr_input[6];
    assign input_g = sr_input[7];
    assign input_h = sr_input[8];
    
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            prescaler <= '0;
            sr_ff <= '0;
            sr_clk <= 1'b0;
            sr_load_n <= 1'b1;
            sr_input <= '0;
        end
        else begin
            prescaler <= prescaler + 1'b1;
            sr_clk <= prescaler[PRESCALER_WIDTH-1];
            if (prescaler == 0) begin
                sr_load_n <= (counter < 9);
                sr_ff <= {sr_ff[7:1], sr_ff[0] ^ 1'(INVERSE), sr_dout};
                if (9 <= counter) begin
                    sr_input <= {sr_ff[7:1], sr_ff[0] ^ 1'(INVERSE)};
                end
            end
            if (prescaler == (1 << (PRESCALER_WIDTH - 1))) begin
                counter <= (counter < 9) ? (counter + 1'b1) : '0;
            end
        end
    end
endmodule
