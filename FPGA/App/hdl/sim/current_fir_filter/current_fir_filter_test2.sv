`timescale 1 ns / 1 ns
module test2 ();
    reg reset = 1'b1;
    reg clk = 1'b0;
    
    logic [15:0] x [0:7];
    logic in_valid = 1'b0;
    logic in_ready;
    logic [15:0] y [0:7];
    logic out_valid;
    logic out_ready = 1'b0;
    
    current_fir_filter #(
        .DATA_WIDTH(16),
        .DATA_COUNT(8)
    ) fir (
        .clk(clk),
        .reset(reset),
        .in_data({x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7]}),
        .in_valid(in_valid),
        .in_ready(in_ready),
        .out_data({y[0], y[1], y[2], y[3], y[4], y[5], y[6], y[7]}),
        .out_valid(out_valid),
        .out_ready(out_ready)
    );
    
    initial begin
        @(negedge reset);
        repeat(1) @(posedge clk);
        fork
            begin
                repeat(10) begin
                    write_data(1000, -1000, 2000, -2000, 3000, -3000, 32767, -32768);
                    repeat(10) @(posedge clk);
                end
                repeat(10) begin
                    write_data(0, 0, 0, 0, 0, 0, 0, 0);
                    repeat(10) @(posedge clk);
                end
            end
            begin
                repeat(100) @(posedge clk);
                out_ready <= 1'b1;
            end
        join
        repeat(50) @(posedge clk);
        $stop;
    end
    
    task write_data(input [15:0] in0, input [15:0] in1, input [15:0] in2, input [15:0] in3, input [15:0] in4, input [15:0] in5, input [15:0] in6, input [15:0] in7);
        x[0] <= in0;
        x[1] <= in1;
        x[2] <= in2;
        x[3] <= in3;
        x[4] <= in4;
        x[5] <= in5;
        x[6] <= in6;
        x[7] <= in7;
        in_valid <= 1'b1;
        @(posedge clk);
        while (in_ready == 1'b0) begin
            @(posedge clk);
        end
        x[0] <= 'X;
        x[1] <= 'X;
        x[2] <= 'X;
        x[3] <= 'X;
        x[4] <= 'X;
        x[5] <= 'X;
        x[6] <= 'X;
        x[7] <= 'X;
        in_valid <= 1'b0;
    endtask
    
    // Clock generation
    always #5ns begin
        clk <= ~clk;
    end
    
    // Reset generaton
    initial begin
        reset <= 1'b1;
        repeat(5) @(negedge clk);
        reset <= 1'b0;
        repeat(10000) @(posedge clk);
        $stop;
    end
endmodule
