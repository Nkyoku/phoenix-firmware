`timescale 1 ns / 1 ns
module test ();
    logic reset = 1'b1;
    logic clk = 1'b0;
    
    logic in_inverse;
    logic unsigned [8:0] in_theta;
    logic signed [15:0] in_y;
    logic signed [15:0] in_x;
    logic in_valid = 1'b0;
    logic in_ready;
    logic signed [15:0] out_y;
    logic signed [15:0] out_x;
    logic out_valid;
    logic out_ready = 1'b1;
    
    park_transform #(
        .CHANNEL_WIDTH(1),
        .DATA_WIDTH(16)
    ) park_0 (
        .clk(clk),
        .reset(reset),
        .in_data({in_inverse, in_theta, in_y, in_x}),
        .in_channel(1'b0),
        .in_valid(in_valid),
        .in_ready(in_ready),
        .out_data({out_y, out_x}),
        .out_channel(),
        .out_valid(out_valid),
        .out_ready(out_ready)
    );
    
    // Data Input
    initial begin
        @(negedge reset);
        repeat(3) @(posedge clk);
        
        //write_data(0, 43, 0, -10000);
        
        //repeat(7) @(posedge clk);
        //write_data(0, 81, 2000, 1000);
        
        
        for (int theta = 0; theta < 512; theta++) begin
            write_data(0, theta, 20000, -10000);
            repeat(16) @(posedge clk);
        end
        
        repeat(16) @(posedge clk);
        $stop;
    end
    
    task write_data(input inverse, input [8:0] theta, input [15:0] y, input [15:0] x);
        in_inverse <= inverse;
        in_theta <= theta;
        in_y <= y;
        in_x <= x;
        in_valid <= 1'b1;
        @(posedge clk);
        while (in_ready == 1'b0) begin
            @(posedge clk);
        end
        in_inverse <= 1'bX;
        in_theta <= 'X;
        in_y <= 'X;
        in_x <= 'X;
        in_valid <= 1'b0;
    endtask
    
    // Data Output
    logic signed [15:0] out_x_latched;
    logic signed [15:0] out_y_latched;
    always @(posedge clk) begin
        if (out_valid & out_ready) begin
            out_x_latched <= out_x;
            out_y_latched <= out_y;
        end
    end
    
    /*initial begin
        out_ready <= 1'b0;
        repeat(24) @(posedge clk);
        out_ready <= 1'b1;
    end*/
    
    // Clock Generation
    always #5ns begin
        clk <= ~clk;
    end
    
    // Reset Generaton
    initial begin
        reset <= 1'b1;
        repeat(10) @(posedge clk);
        reset <= 1'b0;
    end
endmodule
