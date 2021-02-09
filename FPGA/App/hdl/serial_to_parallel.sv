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
    
    logic [PRESCALER_WIDTH-1:0] prescaler = '0;
    logic [3:0] counter = '0;
    logic [8:0] shift_reg = '0;
    logic [8:0] input_filtered;
    assign input_ser = input_filtered[0];
    assign input_a = input_filtered[1];
    assign input_b = input_filtered[2];
    assign input_c = input_filtered[3];
    assign input_d = input_filtered[4];
    assign input_e = input_filtered[5];
    assign input_f = input_filtered[6];
    assign input_g = input_filtered[7];
    assign input_h = input_filtered[8];

    logic valid = 1'b0;

    genvar i;
    generate
        for (i = 0; i <= 8; i = i + 1) begin : filters
            bipolar_sync_deglitch #(
                .DELAY(16),
                .DEFAULT_OUTPUT(0)
            ) filter (
                .reset  (reset),
                .clk    (clk),
                .clk_en (valid),
                .in     (shift_reg[i]),
                .out    (input_filtered[i])
            );
        end
    endgenerate
    
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            prescaler <= '0;
            counter <= '0;
            shift_reg <= '0;
            sr_clk <= 1'b0;
            sr_load_n <= 1'b1;
        end
        else begin
            valid <= 1'b0;
            prescaler <= prescaler + 1'b1;
            sr_clk <= prescaler[PRESCALER_WIDTH-1];
            if (prescaler == 0) begin
                sr_load_n <= (counter < 9);
                shift_reg <= {shift_reg[7:0], sr_dout ^ 1'(INVERSE)};
                if (counter == 8) begin
                    valid <= 1'b1;
                end
            end
            if (prescaler == (1 << (PRESCALER_WIDTH - 1))) begin
                counter <= (counter < 9) ? (counter + 1'b1) : '0;
            end
        end
    end
endmodule
