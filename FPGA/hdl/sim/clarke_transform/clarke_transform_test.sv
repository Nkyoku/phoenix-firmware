`timescale 1 ns / 1 ns
module test ();
    logic reset = 1'b1;
    logic clk = 1'b0;
    
    logic signed [15:0] in_u;
    logic signed [15:0] in_v;
    logic in_valid = 1'b0;
    logic in_ready;
    logic signed [15:0] out_a;
    logic signed [15:0] out_b;
    logic out_valid;
    logic out_ready = 1'b1;
    
    clarke_transform #(
        .CHANNEL_WIDTH(1),
        .DATA_WIDTH(16)
    ) sut (
        .clk(clk),
        .reset(reset),
        .in_data({in_v, in_u}),
        .in_channel(1'b0),
        .in_valid(in_valid),
        .in_ready(in_ready),
        .out_data({out_b, out_a}),
        .out_channel(),
        .out_valid(out_valid),
        .out_ready(out_ready)
    );
    
    // Data Input
    real i_u;
    real i_v;
    real PI = 3.1415926535;
    initial begin
        @(negedge reset);
        repeat(3) @(posedge clk);
        
        //write_data(2000, 1000);
        
        for (int theta = 0; theta < 512; theta++) begin
            i_u = 32767 * $cos(PI * 2 * theta / 512 + PI / 6);
            i_v = 32767 * $cos(PI * 2 * theta / 512 - PI / 2);
            write_data($rtoi(i_u), $rtoi(i_v));
            //repeat(10) @(posedge clk);
        end
        
        repeat(16) @(posedge clk);
        $stop;
    end
    
    task write_data(input [15:0] u, input [15:0] v);
        in_u <= u;
        in_v <= v;
        in_valid <= 1'b1;
        @(posedge clk);
        while (in_ready == 1'b0) begin
            @(posedge clk);
        end
        in_u <= 'X;
        in_v <= 'X;
        in_valid <= 1'b0;
    endtask
    
    // Data Output
    logic signed [15:0] out_a_latched;
    logic signed [15:0] out_b_latched;
    always @(posedge clk) begin
        if (out_valid & out_ready) begin
            out_a_latched <= out_a;
            out_b_latched <= out_b;
        end
    end
    
    always begin
        out_ready <= 1'b0;
        repeat($urandom_range(10, 0)) @(posedge clk);
        out_ready <= 1'b1;
        repeat($urandom_range(10, 0)) @(posedge clk);
    end
    
    // Clock Generation
    always #5ns begin
        clk <= ~clk;
    end
    
    // Reset Generaton
    initial begin
        reset <= 1'b1;
        repeat(10) @(posedge clk);
        reset <= 1'b0;
        //repeat(10000) @(posedge clk);
        //$stop;
    end
endmodule
