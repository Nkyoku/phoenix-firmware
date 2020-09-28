`timescale 1 ns / 1 ns
module test ();
    reg reset = 1'b1;
    reg clk = 1'b0;
    
    logic [15:0] a, b;
    logic in_valid = 1'b0;
    logic in_ready;
    logic [15:0] u, v, w;
    logic out_valid;
    
    real PI = 3.1415926535;
    
    space_vector_modulator svm (
        .clk(clk),
        .reset(reset),
        .in_data({a, b}),
        .in_valid(in_valid),
        .in_ready(in_ready),
        .out_data({u, v, w}),
        .out_valid(out_valid)
    );
    
    initial begin
        int in_a, in_b;
        
        @(negedge reset);
        repeat(1) @(posedge clk);
        
        for(int i = 0; i < 550; i = i + 1) begin
            in_a = 2960 * $cos(2 * PI * i / 512);
            in_b = 2960 * $sin(2 * PI * i / 512);
            write_data(in_a, in_b);
            repeat(1) @(posedge clk);
        end
        
        repeat(10) @(posedge clk);
        $stop;
    end
    
    task write_data(input [15:0] in_a, input [15:0] in_b);
        a <= in_a;
        b <= in_b;
        in_valid <= 1'b1;
        @(posedge clk);
        while (in_ready == 1'b0) begin
            @(posedge clk);
        end
        in_valid <= 1'b0;
        a <= 'X;
        b <= 'X;
    endtask
    
    logic [15:0] u_out, v_out, w_out;
    always @(posedge clk) begin
        if (out_valid == 1'b1) begin
            u_out <= u;
            v_out <= v;
            w_out <= w;
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
        repeat(10000) @(posedge clk);
        $stop;
    end
endmodule
