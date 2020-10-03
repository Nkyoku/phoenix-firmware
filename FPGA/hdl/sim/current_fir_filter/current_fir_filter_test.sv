`timescale 1 ns / 1 ns
module test ();
    reg reset = 1'b1;
    reg clk = 1'b0;
    
    logic [15:0] x0, x1, x2;
    logic in_valid1 = 1'b0;
    logic in_valid2 = 1'b0;
    logic in_ready1;
    logic in_ready2;
    logic [15:0] y0, y1, y2;
    logic out_ready = 1'b0;
    
    current_fir_filter #(
        .DATA_WIDTH(16),
        .DATA_COUNT(2)
    ) fir (
        .clk(clk),
        .reset(reset),
        .in_data({x0, x1}),
        .in_valid(in_valid1),
        .in_ready(in_ready1),
        .out_data({y0, y1}),
        .out_valid(),
        .out_ready(out_ready)
    );
    
    current_fir_filter #(
        .DATA_WIDTH(16),
        .DATA_COUNT(1)
    ) fir_1 (
        .clk(clk),
        .reset(reset),
        .in_data(x2),
        .in_valid(in_valid2),
        .in_ready(in_ready2),
        .out_data(y2),
        .out_valid(),
        .out_ready(out_ready)
    );
    
    initial begin
        @(negedge reset);
        repeat(2) @(posedge clk);
        fork
            begin
                repeat(10) begin
                    write_data1(1000, -1000);
                    repeat(10) @(posedge clk);
                end
            end
            begin
                repeat(10) begin
                    write_data2(1000);
                    repeat(4) @(posedge clk);
                end
            end
            begin
                repeat(25) @(posedge clk);
                out_ready <= 1'b1;
            end
        join
        repeat(10) @(posedge clk);
        $stop;
    end
    
    task write_data1(input [15:0] in0, input [15:0] in1);
        x0 <= in0;
        x1 <= in1;
        in_valid1 <= 1'b1;
        @(posedge clk);
        while (in_ready1 == 1'b0) begin
            @(posedge clk);
        end
        in_valid1 <= 1'b0;
        x0 <= 'X;
        x1 <= 'X;
    endtask
    
    task write_data2(input [15:0] in2);
        x2 <= in2;
        in_valid2 <= 1'b1;
        @(posedge clk);
        while (in_ready2 == 1'b0) begin
            @(posedge clk);
        end
        in_valid2 <= 1'b0;
        x2 <= 'X;
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
        repeat(2000) @(posedge clk);
        $stop;
    end
endmodule
