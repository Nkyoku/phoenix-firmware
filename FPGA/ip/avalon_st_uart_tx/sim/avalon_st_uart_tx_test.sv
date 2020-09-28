`timescale 1 ns / 1 ps
module test ();
    reg reset = 1'b1;
    reg clk = 1'b0;
    
    logic sink_ready;
    logic sink_valid = 1'b0;
    logic [7:0] sink_data;
    
    avalon_st_uart_tx #(
        .PRESCALER(25)
    ) dut (
        .clk(clk),
        .reset(reset),
        .sink_data(sink_data),
        .sink_valid(sink_valid),
        .sink_ready(sink_ready),
        .uart_txd()
    );
    
    initial begin
        @(negedge reset);
        repeat(3) @(posedge clk);
        write(8'h12);
        write(8'h34);
        write(8'h56);
        write(8'h78);
        write(8'h9A);
        repeat(40) @(posedge clk);
        write(8'hBC);
        repeat(41) @(posedge clk);
        write(8'hDE);
        repeat(42) @(posedge clk);
        write(8'hF0);
        repeat(50) @(posedge clk);
        $stop;
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
    
    task write(input [7:0] data);
        sink_data <= data;
        sink_valid <= 1'b1;
        @(posedge clk);
        while (sink_ready == 1'b0) begin
            @(posedge clk);
        end
        sink_data <= 'X;
        sink_valid <= 1'b0;
    endtask
endmodule
