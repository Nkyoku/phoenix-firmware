module deglitch #(
        parameter COUNTER_VALUE = 1,
        parameter COUNTER_WIDTH = $clog2(COUNTER_VALUE + 1),
        parameter DEFAULT_LOGIC = 0
    ) (
        input  wire clk,
        input  wire in,
        output wire out
    );
    localparam default_logic = 1'(DEFAULT_LOGIC);
    reg first_ff = default_logic;
    reg second_ff = default_logic;
    reg [COUNTER_WIDTH-1:0] counter = '0;
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