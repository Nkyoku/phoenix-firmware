(* altera_attribute = {"-name SDC_STATEMENT \"set_false_path -to [get_registers *ResetSynchronizer:*\|ff1]\"; -name SDC_STATEMENT \"set_false_path -to [get_registers *ResetSynchronizer:*\|ff2]\""} *)
module ResetSynchronizer (
        input  wire clk,
        input  wire reset_in,
        output wire reset_out
    );

    logic ff1 = 1'b1;
    logic ff2 = 1'b1;
    assign reset_out = ff2;
    always @(posedge clk, posedge reset_in) begin
        if (reset_in == 1'b1) begin
            ff1 <= 1'b1;
            ff2 <= 1'b1;
        end
        else begin
            ff1 <= 1'b0;
            ff2 <= ff1;
        end
    end
endmodule
