`timescale 1 ns / 1 ns
module test ();
    logic reset = 1'b1;
    logic clk = 1'b0;
    
    logic trigger = 1'b0;
    logic fault = 1'b0;
    logic pwm_valid = 1'b0;
    logic [15:0] pwm_u_data = '0;
    logic [15:0] pwm_v_data = '0;
    logic [15:0] pwm_w_data = '0;
    
    ds_pwm_driver #(
        .PERIOD(100),
        .MAX_ON_CYCLES(95)
    ) driver (
        .clk(clk),
        .reset(reset),
        .trigger(trigger),
        .fault(fault),
        .pwm_valid(pwm_valid),
        .pwm_data({pwm_u_data, pwm_v_data, pwm_w_data}),
        .driver_otw_n(1'b1),
        .driver_fault_n(1'b1),
        .driver_pwm(),
        .driver_reset_n(),
        .status_otw_n(),
        .status_fault_n()
    );
    
    // Data Input
    initial begin
        @(negedge reset);
        @(posedge clk);
        
        fork
            begin
                trigger <= 1'b1;
                @(posedge clk);
                trigger <= 1'b0;
                repeat(99) @(posedge clk);
            end
        join
        repeat(10) @(posedge clk);
        
        fork
            begin
                repeat(10) @(posedge clk);
                write_data(10, 20, 30);
                repeat(99) @(posedge clk);
                write_data(20, 30, 40);
                repeat(89) @(posedge clk);
                write_data(30, 40, 50);
                repeat(79) @(posedge clk);
                write_data(40, 50, 60);
                repeat(69) @(posedge clk);
                write_data(50, 60, 70);
                repeat(59) @(posedge clk);
                write_data(60, 70, 80);
                repeat(49) @(posedge clk);
                write_data(70, 80, 90);
                repeat(149) @(posedge clk);
                fault <= 1'b1;
                @(posedge clk);
                fault <= 1'b0;
                repeat(149) @(posedge clk);
                write_data(1, 51, 99);
            end
            for (int i = 0; i < 10; i++) begin
                trigger <= 1'b1;
                @(posedge clk);
                trigger <= 1'b0;
                repeat(99) @(posedge clk);
            end
        join
        repeat(10) @(posedge clk);
        
        fork
            begin
                write_data(50, 50, 50);
                repeat(99) @(posedge clk);
                write_data(40, 40, 40);
                repeat(99) @(posedge clk);
                write_data(30, 30, 30);
                repeat(99) @(posedge clk);
                write_data(30, 30, 30);
                repeat(99) @(posedge clk);
            end
            begin
                fault <= 1'b1;
                @(posedge clk);
                fault <= 1'b0;
                repeat(99) @(posedge clk);
                fault <= 1'b1;
                @(posedge clk);
                fault <= 1'b0;
                repeat(100) @(posedge clk);
                fault <= 1'b1;
                @(posedge clk);
                fault <= 1'b0;
                repeat(97) @(posedge clk);
                fault <= 1'b1;
                @(posedge clk);
                fault <= 1'b0;
            end
            for (int i = 0; i < 10; i++) begin
                trigger <= 1'b1;
                @(posedge clk);
                trigger <= 1'b0;
                repeat(99) @(posedge clk);
            end
        join
        repeat(10) @(posedge clk);
        
        
        
        
        
        
        $stop;
    end
    
    task write_data(input [15:0] u, input [15:0] v, input [15:0] w);
        pwm_valid <= 1'b1;
        pwm_u_data <= u;
        pwm_v_data <= v;
        pwm_w_data <= w;
        @(posedge clk);
        pwm_valid <= 1'b0;
        pwm_u_data <= 'X;
        pwm_v_data <= 'X;
        pwm_w_data <= 'X;
    endtask
    
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
