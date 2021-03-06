`timescale 1 ns / 1 ps
module test ();
    reg reset = 1'b1;
    reg clk = 1'b0;
    
    logic trigger = 1'b0;
    logic [15:0] kp = 23456;
    logic [15:0] ki = 12345;
    
    logic signed [15:0] in_ref_data = 'X;
    logic in_ref_valid = 1'b0;
    logic signed [15:0] in_proc_data = 'X;
    logic in_proc_valid = 1'b0;
    logic signed [15:0] out_data;
    logic out_valid;
    
    pi_controller #(
        .SCALE(14),
        .GAIN_WIDTH(16),
        .DATA_WIDTH(16),
        .DATA_COUNT(1),
        .DATA_LIMIT(2960)
    ) pi (
        .clk(clk),
        .reset(reset),
        .trigger(trigger),
        .param_kp(kp),
        .param_ki(ki),
        .in_ref_data(in_ref_data),
        .in_ref_valid(in_ref_valid),
        .in_proc_data(in_proc_data),
        .in_proc_valid(in_proc_valid),
        .out_data(out_data),
        .out_valid(out_valid)
    );
    
    initial begin
        @(negedge reset);
        repeat(1) @(posedge clk);
        in_ref_valid <= 1'b1;
        in_proc_valid <= 1'b1;
        in_ref_data <= 1000;
        in_proc_data <= 0;
        trigger <= 1'b1;
        @(posedge clk);
        in_ref_valid <= 1'b0;
        in_proc_valid <= 1'b0;
        in_ref_data <= 'X;
        in_proc_data <= 'X;
        trigger <= 1'b0;
        repeat(10) @(posedge clk);
        
        fork
            repeat(2) begin
                trigger <= 1'b1;
                @(posedge clk);
                trigger <= 1'b0;
                repeat(10) @(posedge clk);
            end
            /*begin
                repeat(20) @(posedge trigger);
                repeat(3) @(posedge clk);
                reset <= 1'b1;
                @(posedge clk);
                reset <= 1'b0;
                repeat(2) @(posedge trigger);
                repeat(7) @(posedge clk);
                in_ref_valid <= 1'b1;
                in_proc_valid <= 1'b1;
                in_ref_data <= 1000;
                @(posedge clk);
                in_ref_valid <= 1'b0;
                in_proc_valid <= 1'b0;
                in_ref_data <= 'X;
            end*/
        join
        
        $stop;
    end
    
    always @(posedge clk) begin
        in_proc_valid <= 1'b0;
        if (out_valid == 1'b1) begin
            in_proc_data <= 0.2 * out_data + 0.8 * in_proc_data;
            in_proc_valid <= 1'b1;
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
        repeat(1000) @(negedge clk);
        $stop;
    end
endmodule
