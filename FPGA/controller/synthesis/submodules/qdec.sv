module qdec #(
        parameter COUNTER_WIDTH = 16,
        parameter INVERT = 0
    ) (
        input  wire        clk,
        input  wire        reset,
        input  wire        clk_latch,
        input  wire        slave_address,
        input  wire        slave_read,
        output reg  [31:0] slave_readdata,
        input  wire        enc_a,
        input  wire        enc_b
    );
    
    // Filter inputs
    reg [3:0] enc_a_ff = 4'b0000;
    reg [3:0] enc_b_ff = 4'b0000;
    wire enc_a_filtered = (2 <= (enc_a_ff[3] + enc_a_ff[2] + enc_a_ff[1]));
    wire enc_b_filtered = (2 <= (enc_b_ff[3] + enc_b_ff[2] + enc_b_ff[1]));
    always @(posedge clk) begin
        enc_a_ff <= {enc_a_ff[2:0], enc_a};
        enc_b_ff <= {enc_b_ff[2:0], enc_b};
    end
    
    // State Machine
    reg previous_enc_a = 1'b0;
    reg previous_enc_b = 1'b0;
    wire [3:0] state = {previous_enc_b, previous_enc_a, enc_b_filtered, enc_a_filtered};
    logic inc;
    logic dec;
    always_comb begin
        case (state)
            4'b0000 : begin inc = 1'b0; dec = 1'b0; end
            4'b0001 : begin inc = 1'b1; dec = 1'b0; end // +1
            4'b0010 : begin inc = 1'b0; dec = 1'b1; end // -1
            4'b0011 : begin inc = 1'b1; dec = 1'b1; end // error
            4'b0100 : begin inc = 1'b0; dec = 1'b1; end // -1
            4'b0101 : begin inc = 1'b0; dec = 1'b0; end
            4'b0110 : begin inc = 1'b1; dec = 1'b1; end // error
            4'b0111 : begin inc = 1'b1; dec = 1'b0; end // +1
            4'b1000 : begin inc = 1'b1; dec = 1'b0; end // +1
            4'b1001 : begin inc = 1'b1; dec = 1'b1; end // error
            4'b1010 : begin inc = 1'b0; dec = 1'b0; end
            4'b1011 : begin inc = 1'b0; dec = 1'b1; end // -1
            4'b1100 : begin inc = 1'b1; dec = 1'b1; end // error
            4'b1101 : begin inc = 1'b0; dec = 1'b1; end // -1
            4'b1110 : begin inc = 1'b1; dec = 1'b0; end // +1
            4'b1111 : begin inc = 1'b0; dec = 1'b0; end
        endcase
    end
    always @(posedge clk) begin
        previous_enc_a <= enc_a_filtered;
        previous_enc_b <= enc_b_filtered;
    end
    
    // Counter
    reg signed [COUNTER_WIDTH-1:0] counter = '0;
    reg signed [COUNTER_WIDTH-1:0] counter_latched = '0;
    reg fault = 1'b0;
    reg fault_latched = 1'b0;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            counter <= '0;
            counter_latched <= '0;
            fault <= 1'b0;
            fault_latched <= 1'b0;
        end
        else begin
            if (inc & ~dec) begin
                if (INVERT == 0) begin
                    counter <= (clk_latch ? '0 : counter) + 1'b1;
                end
                else begin
                    counter <= (clk_latch ? '0 : counter) - 1'b1;
                end
            end
            else if (~inc & dec) begin
                if (INVERT == 0) begin
                    counter <= (clk_latch ? '0 : counter) - 1'b1;
                end
                else begin
                    counter <= (clk_latch ? '0 : counter) + 1'b1;
                end
            end
            else begin
                counter <= (clk_latch ? '0 : counter);
            end
            fault <= (inc & dec) | (fault & ~clk_latch);
            if (clk_latch == 1'b1) begin
                counter_latched <= counter;
                fault_latched <= fault;
            end
        end
    end
    
    // Avalon-MM slave interface
    typedef enum {
        REGISTER_STATUS  = 'h0,
        REGISTER_COUNTER = 'h1
    } REGISTER_MAP;
    always @(posedge clk) begin
        if (slave_read == 1'b1) begin
            case (slave_address)
                REGISTER_STATUS  : slave_readdata <= {31'h00000000, fault_latched};
                REGISTER_COUNTER : slave_readdata <= 32'(counter_latched);
                default          : slave_readdata <= 32'h00000000;
            endcase
        end
    end
endmodule
