`timescale 1 ns / 1 ps
module test ();
    reg reset = 1'b1;
    reg clk1 = 1'b0;
    reg clk2 = 1'b0;
    reg [7:0] sink_data = '0;
    reg sink_valid = 1'b0;
    wire [7:0] source_data;
    wire source_valid;
    reg source_ready = 1'b1;
    reg [7:0] output_data = 8'hXX;
    
    avalon_st_clock_bridge dut (
        .clk1(clk1),
        .reset1(reset),
        .sink_data(sink_data),
        .sink_valid(sink_valid),
        .sink_error(1'b0),
        .sink_ready(),
        .clk2(clk2),
        .reset2(reset),
        .source_data(source_data),
        .source_valid(source_valid),
        .source_error(),
        .source_ready(source_ready)
    );
    
    task send_data(input [7:0] data, input int duration);
        sink_data <= data;
        sink_valid <= 1'b1;
        repeat(duration) @(posedge clk1);
        sink_valid <= 1'b0;
    endtask
    
    initial begin
        @(negedge reset);
        @(posedge clk1);
        send_data(8'h01, 1);
        repeat(10) @(posedge clk1);
        send_data(8'h02, 1);
        send_data(8'h03, 1);
        send_data(8'h04, 1);
        send_data(8'h05, 1);
        send_data(8'h06, 1);
        send_data(8'h07, 1);
        send_data(8'h08, 1);
        send_data(8'h09, 1);
        send_data(8'h0A, 1);
        send_data(8'h0B, 1);
        send_data(8'h0C, 1);
        send_data(8'h0D, 1);
        send_data(8'h0E, 1);
        send_data(8'h0F, 1);
        repeat(10) @(posedge clk1);
        send_data(8'h10, 1);
        @(posedge clk1);
        send_data(8'h11, 1);
        @(posedge clk1);
        send_data(8'h12, 1);
        @(posedge clk1);
        send_data(8'h13, 1);
        @(posedge clk1);
        send_data(8'h14, 1);
        @(posedge clk1);
        send_data(8'h15, 1);
        repeat(10) @(posedge clk1);
        send_data(8'h16, 1);
        @(posedge clk1);
        source_ready <= 1'b0;
        repeat(10) @(posedge clk1);
        source_ready <= 1'b1;
        @(posedge clk1);
        send_data(8'h17, 1);
        send_data(8'h18, 1);
        send_data(8'h19, 1);
        send_data(8'h1A, 1);
        source_ready <= 1'b0;
        send_data(8'h1B, 1);
        send_data(8'h1C, 1);
        source_ready <= 1'b1;
        send_data(8'h1D, 1);
        send_data(8'h1E, 1);
        send_data(8'h1F, 1);
    end
    
    always @(posedge clk2) begin
        if (source_valid & source_ready) begin
            output_data <= source_data;
        end
        else begin
            output_data <= 8'hXX;
        end
    end
    
    // Clock 1 100MHz Generation
    always #5ns begin
        clk1 <= ~clk1;
    end
    
    // Clock 2 150MHz Generation
    always #3.3ns begin
        clk2 <= ~clk2;
    end
    
    // Reset Generaton
    initial begin
        reset <= 1'b1;
        repeat(5) @(negedge clk1);
        reset <= 1'b0;
        repeat(150) @(negedge clk1);
        $stop;
    end
endmodule
