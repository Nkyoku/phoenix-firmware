`timescale 1 ns / 1 ps
module test ();
    reg reset = 1'b1;
    reg clk = 1'b0;
    
    logic in_ready;
    logic in_valid = 1'b0;
    logic signed [15:0] in_data;
    logic [1:0] in_channel;
    
    logic out_valid;
    logic [1:0] out_channel;
    logic [15:0] out_data;
    
    MyMultiFIR fir0 (
        .reset(reset),
        .clk(clk),
        .in_ready(in_ready),
        .in_valid(in_valid),
        .in_data(in_data),
        .in_channel(in_channel),
        .out_valid(out_valid),
        .out_data(out_data),
        .out_channel(out_channel)
    );
    
    initial begin
        @(negedge reset);
        repeat(3) @(posedge clk);
        write(0, 32767);
        write(1, 32767);
        write(2, 32767);
        write(3, 32767);
        repeat(2) @(posedge clk);
        repeat(8) begin
            write(0, 0);
            write(1, 0);
            write(2, 0);
            write(3, 0);
            repeat(2) @(posedge clk);
        end
        repeat(8) begin
            write(0, -32768);
            write(1, -32768);
            write(2, -32768);
            write(3, -32768);
        end
        repeat(20) @(posedge clk);
        $stop;
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
        repeat(1000) @(negedge clk);
        $stop;
    end
    
    logic [15:0] out_data_valid;
    logic [1:0] out_channel_valid;
    always @(posedge clk) begin
        if (out_valid == 1'b1) begin
            out_data_valid <= out_data;
            out_channel_valid <= out_channel;
        end
        else begin
            out_data_valid <= 'X;
            out_channel_valid <= 'X;
        end
    end
    
    task write(input [1:0] channel, input [15:0] data);
        in_channel <= channel;
        in_data <= data;
        in_valid <= 1'b1;
        @(posedge clk);
        while (in_ready == 1'b0) begin
            @(posedge clk);
        end
        in_channel <= 'X;
        in_data <= 'X;
        in_valid <= 1'b0;
    endtask
endmodule
