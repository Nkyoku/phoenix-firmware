`timescale 1 ns / 1 ps
module test ();
    reg reset = 1'b1;
    reg clk = 1'b0;
    
    logic sink_ready;
    logic sink_valid = 1'b0;
    logic [15:0] sink_data;
    logic [7:0] sink_channel;
    
    logic source_valid;
    logic source_ready = 1'b0;
    logic [7:0] source_data;
    
    avalon_st_byte_converter dut (
        .clk(clk),
        .reset(reset),
        .sink_data(sink_data),
        .sink_channel(sink_channel),
        .sink_valid(sink_valid),
        .sink_ready(sink_ready),
        .source_valid(source_valid),
        .source_data(source_data),
        .source_ready(source_ready)
    );
    
    initial begin
        @(negedge reset);
        repeat(3) @(posedge clk);
        write(8'h01, 16'h1234);
        write(8'h02, 16'h5678);
        write(8'h03, 16'h9ABC);
        write(8'h04, 16'hDEF0);
        repeat(1) @(posedge clk);
        write(8'h05, 16'h1234);
        repeat(2) @(posedge clk);
        write(8'h02, 16'h5678);
        repeat(3) @(posedge clk);
        write(8'h03, 16'h9ABC);
        repeat(4) @(posedge clk);
        write(8'h04, 16'hDEF0);
        repeat(1) @(posedge clk);
        write(8'h7A, 16'h1234);
        write(8'h05, 16'hAA7B);
        write(8'h06, 16'h7C55);
        write(8'h7A, 16'hAA7B);
        write(8'h7D, 16'h7C55);
        write(8'h7A, 16'h7C7B);
        repeat(1) @(posedge clk);
        write(8'h10, 16'h2211);
        write(8'h11, 16'h7C33);
        write(8'h12, 16'h6655);
        write(8'h7B, 16'h8877);
        write(8'h14, 16'hAA99);
        write(8'h15, 16'h7D7B);
        write(8'h16, 16'hEEDD);
        write(8'h17, 16'h00FF);
        repeat(1) @(posedge clk);
        repeat(20) @(posedge clk);
        $stop;
    end
    
    initial begin
        @(negedge reset);
        repeat(4) @(posedge clk);
        source_ready <= 1'b1;
        repeat(88) @(posedge clk);
        source_ready <= 1'b0;
        repeat(1) @(posedge clk);
        source_ready <= 1'b1;
        repeat(2) @(posedge clk);
        source_ready <= 1'b0;
        repeat(3) @(posedge clk);
        source_ready <= 1'b1;
        repeat(4) @(posedge clk);
        source_ready <= 1'b0;
        repeat(4) @(posedge clk);
        source_ready <= 1'b1;
        repeat(5) @(posedge clk);
        source_ready <= 1'b0;
        repeat(5) @(posedge clk);
        source_ready <= 1'b1;
    end
    
    // Clock generation
    always #5ns begin
        clk <= ~clk;
    end
    
    // Reset generaton
    initial begin
        reset <= 1'b1;
        repeat(4) @(negedge clk);
        reset <= 1'b0;
        repeat(10000) @(posedge clk);
        $stop;
    end
    
    logic [7:0] source_data_valid;
    always @(posedge clk) begin
        if (source_valid  & source_ready) begin
            source_data_valid <= source_data;
        end
        else begin
            source_data_valid <= 'X;
        end
    end
    
    task write(input [7:0] channel, input [15:0] data);
        sink_channel <= channel;
        sink_data <= data;
        sink_valid <= 1'b1;
        @(posedge clk);
        while (sink_ready == 1'b0) begin
            @(posedge clk);
        end
        sink_channel <= 'X;
        sink_data <= 'X;
        sink_valid <= 1'b0;
    endtask
endmodule
