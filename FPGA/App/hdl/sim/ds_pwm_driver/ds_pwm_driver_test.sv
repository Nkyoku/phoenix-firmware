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
        .INITIAL_DIRECTION(0),
        .PERIOD(100),
        .MAX_ON_CYCLES(95),
        .DATA_WIDTH(16)
    ) driver (
        .clk(clk),
        .reset(reset),
        .trigger(trigger),
        .fault(fault),
        .pwm_valid(pwm_valid),
        .pwm_data({pwm_u_data, pwm_v_data, pwm_w_data}),
        .driver_pwm(),
        .driver_reset_n()
    );
    
    // Data Input
    initial begin
        @(negedge reset);
        repeat(2) @(posedge trigger);
        repeat(10) @(posedge clk);
        
        write_data(0, 0, 0);
        repeat(99) @(posedge clk);

        for (int i = 0; i <= 33; i++) begin
            write_data(3 * i, 3 * i + 1, 3 * i + 2);
            repeat(99) @(posedge clk);
        end

        /*fork
            begin
                repeat(10) @(posedge clk);
                write_data(11, 22, 33);
                repeat(99) @(posedge clk);
                write_data(23, 34, 44);
                repeat(99) @(posedge clk);
                write_data(35, 45, 55);
                repeat(99) @(posedge clk);
                write_data(46, 56, 66);
                repeat(99) @(posedge clk);
                write_data(57, 67, 77);
                repeat(99) @(posedge clk);
                write_data(68, 78, 88);
                repeat(99) @(posedge clk);
                write_data(79, 89, 99);
                repeat(99) @(posedge clk);
                write_data(79, 89, 99);
                repeat(99) @(posedge clk);

                repeat(149) @(posedge clk);
                write_data(1, 51, 99);
            end
            for (int i = 0; i < 10; i++) begin
                trigger <= 1'b1;
                @(posedge clk);
                trigger <= 1'b0;
                repeat(99) @(posedge clk);
            end
        join*/
        
        repeat(100) @(posedge clk);
        $stop;
    end
    
    // Trigger Pulse
    always begin
        repeat(99) @(posedge clk);
        trigger <= 1'b1;
        @(posedge clk);
        trigger <= 1'b0;
    end

    task write_data(input [15:0] u, input [15:0] v, input [15:0] w);
        pwm_valid <= 1'b1;
        pwm_u_data <= u;
        pwm_v_data <= v;
        pwm_w_data <= w;
        @(posedge clk);
        pwm_valid <= 1'b0;
        //pwm_u_data <= 'X;
        //pwm_v_data <= 'X;
        //pwm_w_data <= 'X;
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
