module quadrature_decoder #(
        parameter INVERSE = 0,
        parameter DATA_WIDTH = 16
    ) (
        input  wire                  clk,
        input  wire                  reset,
        input  wire                  latch,
        input  wire                  enc_a,
        input  wire                  enc_b,
        output reg                   inc,
        output reg                   dec,
        output reg  [DATA_WIDTH-1:0] counter
    );
    
    // Filter inputs
    logic [3:0] enc_a_ff = 4'b0000;
    logic [3:0] enc_b_ff = 4'b0000;
    wire enc_a_filtered = (2 <= (2'(enc_a_ff[3]) + 2'(enc_a_ff[2]) + 2'(enc_a_ff[1])));
    wire enc_b_filtered = (2 <= (2'(enc_b_ff[3]) + 2'(enc_b_ff[2]) + 2'(enc_b_ff[1])));
    always @(posedge clk) begin
        enc_a_ff <= {enc_a_ff[2:0], (INVERSE == 0) ? enc_a : enc_b};
        enc_b_ff <= {enc_b_ff[2:0], (INVERSE == 0) ? enc_b : enc_a};
    end
    
    // State Machine
    logic previous_enc_a = 1'b0;
    logic previous_enc_b = 1'b0;
    wire [3:0] state = {previous_enc_b, previous_enc_a, enc_b_filtered, enc_a_filtered};
    always @(posedge clk) begin
        previous_enc_a <= enc_a_filtered;
        previous_enc_b <= enc_b_filtered;
        case (state)
            4'b0000 : begin inc <= 1'b0; dec <= 1'b0; end
            4'b0001 : begin inc <= 1'b1; dec <= 1'b0; end // +1
            4'b0010 : begin inc <= 1'b0; dec <= 1'b1; end // -1
            4'b0011 : begin inc <= 1'b1; dec <= 1'b1; end // error
            4'b0100 : begin inc <= 1'b0; dec <= 1'b1; end // -1
            4'b0101 : begin inc <= 1'b0; dec <= 1'b0; end
            4'b0110 : begin inc <= 1'b1; dec <= 1'b1; end // error
            4'b0111 : begin inc <= 1'b1; dec <= 1'b0; end // +1
            4'b1000 : begin inc <= 1'b1; dec <= 1'b0; end // +1
            4'b1001 : begin inc <= 1'b1; dec <= 1'b1; end // error
            4'b1010 : begin inc <= 1'b0; dec <= 1'b0; end
            4'b1011 : begin inc <= 1'b0; dec <= 1'b1; end // -1
            4'b1100 : begin inc <= 1'b1; dec <= 1'b1; end // error
            4'b1101 : begin inc <= 1'b0; dec <= 1'b1; end // -1
            4'b1110 : begin inc <= 1'b1; dec <= 1'b0; end // +1
            4'b1111 : begin inc <= 1'b0; dec <= 1'b0; end
        endcase
    end
    
    // Counter and latch
    logic [DATA_WIDTH-1:0] counter_internal = '0;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            counter <= '0;
            counter_internal <= '0;
        end
        else begin
            if (latch == 1'b1) begin
                counter <= counter_internal;
            end
            if (inc & ~dec) begin
                counter_internal <= (~latch ? counter_internal : '0) + 1'b1;
            end
            else if (~inc & dec) begin
                counter_internal <= (~latch ? counter_internal : '0) - 1'b1;
            end
            else if (latch == 1'b1) begin
                counter_internal <= '0;
            end
        end
    end
endmodule
