`timescale 1 ns / 1 ps
module test ();
    reg reset = 1'b1;
    reg clk100mhz = 1'b0;
    reg trigger = 1'b0;
    reg fault = 1'b0;
    reg [2:0] hall_uvw = 3'b011;
    reg [15:0] pwm_data = 16'hXXXX;
    reg pwm_valid = 1'b0;

    pwm_driver #(
        .PWM_PERIOD_CYCLES(100),
        .PWM_MAX_ON_CYCLES(95),
        .DATA_WIDTH(16)
    ) driver (
        .clk(clk100mhz),
        .reset(reset),
        .trigger(trigger),
        .fault(fault),
        .pwm_sink_data(pwm_data),
        .pwm_sink_valid(pwm_valid),
        .sensor_hall_uvw(hall_uvw),
        .driver_otw_n(1'b1),
        .driver_fault_n(1'b1),
        .driver_pwm(),
        .driver_reset_n(),
        .status_driver_otw_n(),
        .status_driver_fault_n(),
        .status_hall_fault_n()
    );
    
    task write_data(input [15:0] data);
        pwm_valid <= 1'b1;
        pwm_data <= data;
        @(posedge clk100mhz);
        pwm_valid <= 1'b0;
        pwm_data <= 16'hXXXX;
    endtask
    
    // Trigger generation
    int cnt = 80;
    always begin
        repeat(10) @(posedge clk100mhz);
        trigger <= 1'b1;
        @(posedge clk100mhz);
        trigger <= 1'b0;
        repeat(10) @(posedge clk100mhz);
        write_data((cnt & 1) ? -cnt : cnt);
        cnt <= cnt + 1;
        repeat(78) @(posedge clk100mhz);
    end

    // Hall generation
    always begin
        # 3us
        hall_uvw <= 3'b001;
        # 3us
        hall_uvw <= 3'b011;
        # 3us
        hall_uvw <= 3'b010;
        # 3us
        hall_uvw <= 3'b110;
        # 3us
        hall_uvw <= 3'b100;
        # 3us
        hall_uvw <= 3'b101;
    end
    
    // Clock 1 100MHz generation
    always #5ns begin
        clk100mhz <= ~clk100mhz;
    end
    
    // Reset generaton
    initial begin
        reset <= 1'b1;
        repeat(5) @(negedge clk100mhz);
        reset <= 1'b0;
        repeat(4000) @(negedge clk100mhz);
        $stop;
    end
endmodule
