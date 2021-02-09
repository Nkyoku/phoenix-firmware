// Unipolar Deglitch Circuit for Synchronized Input
module deglitch #(
        parameter int COUNTER_VALUE = 1,
        parameter int COUNTER_WIDTH = $clog2(COUNTER_VALUE + 1),
        parameter int DEFAULT_LOGIC = 0
    ) (
        input  wire clk,
        input  wire in,
        output wire out
    );
    localparam default_logic = 1'(DEFAULT_LOGIC);
    logic first_ff = default_logic;
    logic second_ff = default_logic;
    logic [COUNTER_WIDTH-1:0] counter = '0;
    assign out = (counter < COUNTER_VALUE) ? default_logic : ~default_logic;
    always @(posedge clk) begin
        first_ff <= in;
        second_ff <= first_ff;
        if (second_ff == default_logic) begin
            counter <= '0;
        end
        else begin
            if (counter < COUNTER_VALUE) begin
                counter <= counter + 1'b1;
            end
        end
    end
endmodule

// Bipolar Deglitch Circuit for Clock Synchronized Input
module bipolar_sync_deglitch #(
        parameter int DELAY = 1,
        parameter int DEFAULT_OUTPUT = 0
    ) (
        input  wire reset,
        input  wire clk,
        input  wire clk_en,
        input  wire in,
        output reg  out
    );
    
    localparam int COUNTER_WIDTH = $clog2(DELAY + 1);

    logic unsigned [COUNTER_WIDTH-1:0] counter = '0;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            out <= 1'(DEFAULT_OUTPUT);
            counter <= '0;
        end
        else if (clk_en == 1'b1) begin
            if (in == out) begin
                counter <= '0;
            end
            else if (counter != '1) begin
                counter <= counter + 1'b1;
            end
            if (DELAY <= counter) begin
                out <= in;
            end
        end
    end
endmodule
