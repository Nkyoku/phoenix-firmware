`timescale 1 ns / 1 ps
module test ();
    reg reset = 1'b1;
    reg clk = 1'b0;
    
    logic in_valid = 1'b0;
    logic signed [15:0] in_data = '0;
    
    MyFIR fir0 (
        .reset(reset),
        .clk(clk),
        .in_ready(),
        .in_valid(in_valid),
        .in_data(in_data),
        .out_valid(),
        .out_data()
    );
    
    initial begin
        @(negedge reset);
        repeat(3) @(posedge clk);
        in_valid <= 1'b1;
        in_data <= 32767;
        @(posedge clk);
        in_valid <= 1'b0;
        in_data <= '0;
        repeat(10) begin
            repeat(10) @(posedge clk);
            in_valid <= 1'b1;
            @(posedge clk);
            in_valid <= 1'b0;
        end
        repeat(4) @(posedge clk);
        repeat(10) begin
            in_valid <= 1'b1;
            in_data <= -32768;
            @(posedge clk);
            in_valid <= 1'b0;
            in_data <= 0;
            repeat(10) @(posedge clk);
        end
    end
    
    // Clock generation
    always #5ns begin
        clk <= ~clk;
    end
    
    // Reset generaton
    initial begin
        reset <= 1'b1;
        repeat(5) @(negedge clk);
        reset <= 1'b0;
        repeat(300) @(negedge clk);
        $stop;
    end
endmodule
