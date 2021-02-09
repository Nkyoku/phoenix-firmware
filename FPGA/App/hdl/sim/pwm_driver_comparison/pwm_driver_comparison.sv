`timescale 1 ns / 1 ps
module test ();
    localparam int THETA_WIDTH = 9;
    localparam int DATA_WIDTH = 16;
    
    reg reset = 1'b1;
    reg clk = 1'b0;
    reg trigger = 1'b0;
    reg fault = 1'b0;

    // Simulate Hall sensors and Encoder
    logic [2:0] hall_uvw = '0;
    logic [1:0] enc_ab = '0;
    real theta_in_deg = 0.0;
    always @(theta_in_deg) begin
        int t;
        t = $rtoi(8000 * theta_in_deg) % 2880000;
        if (t < 0) t = t + 2880000;
        hall_uvw[2] <= (t < 240000) | (1680000 <= t);
        hall_uvw[1] <= (t < 1200000) | (2640000 <= t);
        hall_uvw[0] <= (720000 < t) & (t <= 2160000);
        enc_ab[1] <= (t / 11250) % 2;
        enc_ab[0] <= ((t + 5625) / 11250) % 2;
    end

    // Single Slope PWM Driver
    logic [DATA_WIDTH-1:0] ss_pwm_data = 16'hXXXX;
    logic ss_pwm_valid = 1'b0;
    pwm_driver #(
        .PWM_PERIOD_CYCLES(100),
        .PWM_MAX_ON_CYCLES(95),
        .DATA_WIDTH(DATA_WIDTH)
    ) ss (
        .clk(clk),
        .reset(reset),
        .trigger(trigger),
        .fault(fault),
        .pwm_sink_data(ss_pwm_data),
        .pwm_sink_valid(ss_pwm_valid),
        .sensor_hall_uvw(hall_uvw),
        .driver_otw_n(1'b1),
        .driver_fault_n(1'b1),
        .driver_pwm(),
        .driver_reset_n(),
        .status_driver_otw_n(),
        .status_driver_fault_n(),
        .status_hall_fault_n()
    );

    // Quadrature Encoder Decoder
    logic qdec_inc;
    logic qdec_dec;
    assign status_encoder_fault_n = ~(qdec_inc & qdec_dec);
    quadrature_decoder #(
        .INVERSE(1'b1),
        .DATA_WIDTH(DATA_WIDTH)
    ) qdec (
        .clk(clk),
        .reset(reset),
        .latch(1'b0),
        .enc_a(enc_ab[1]),
        .enc_b(enc_ab[0]),
        .inc(qdec_inc),
        .dec(qdec_dec),
        .counter()
    );
    
    // Position Estimator
    logic [THETA_WIDTH-1:0] theta_data;
    position_estimator #(
        .THETA_WIDTH(THETA_WIDTH)
    ) pos (
        .clk(clk),
        .reset(reset),
        .hall_uvw(hall_uvw),
        .qdec_inc(qdec_inc),
        .qdec_dec(qdec_dec),
        .theta_data(theta_data),
        .theta_error(),
        .theta_uncertain()
    );

    // Park transform
    logic [DATA_WIDTH-1:0] ds_pwm_d_data, ds_pwm_q_data;
    logic ds_pwm_dq_valid = 1'b0;
    logic [DATA_WIDTH-1:0] ds_pwm_a_data, ds_pwm_b_data;
    logic ds_pwm_ab_valid;
    logic ds_pwm_ab_ready;
    park_transform #(
        .CHANNEL_WIDTH(1),
        .DATA_WIDTH(DATA_WIDTH),
        .THETA_WIDTH(THETA_WIDTH)
    ) park (
        .clk(clk),
        .reset(reset),
        .in_data({1'b1, theta_data, ds_pwm_d_data, ds_pwm_q_data}),
        .in_channel(1'b0),
        .in_valid(ds_pwm_dq_valid),
        .in_ready(),
        .out_data({ds_pwm_a_data, ds_pwm_b_data}),
        .out_channel(),
        .out_valid(ds_pwm_ab_valid),
        .out_ready(ds_pwm_ab_ready)
    );
    
    // Space Vector Modulation
    logic unsigned [DATA_WIDTH-1:0] ds_driver_pwm_u_data, ds_driver_pwm_v_data, ds_driver_pwm_w_data;
    logic ds_driver_pwm_valid;
    space_vector_modulator #(
        .DATA_WIDTH(DATA_WIDTH),
        .OUT_WIDTH(DATA_WIDTH),
        .PERIOD(100),
        .OUT_MIN(10),
        .OUT_MAX(90)
    ) svm (
        .clk(clk),
        .reset(reset),
        .in_data({ds_pwm_a_data, ds_pwm_b_data}),
        .in_valid(ds_pwm_ab_valid),
        .in_ready(ds_pwm_ab_ready),
        .out_data({ds_driver_pwm_u_data, ds_driver_pwm_v_data, ds_driver_pwm_w_data}),
        .out_valid(ds_driver_pwm_valid)
    );

    // Dual Slope PWM Driver
    ds_pwm_driver #(
        .PERIOD(100),
        .MAX_ON_CYCLES(95),
        .DATA_WIDTH(DATA_WIDTH)
    ) ds (
        .clk(clk),
        .reset(reset),
        .trigger(trigger),
        .fault(fault),
        .pwm_valid(ds_driver_pwm_valid),
        .pwm_data({ds_driver_pwm_u_data, ds_driver_pwm_v_data, ds_driver_pwm_w_data}),
        .driver_pwm(),
        .driver_reset_n()
    );

    // Trigger generation
    always begin
        repeat(10) @(posedge clk);
        ss_pwm_data = 50;
        ss_pwm_valid = 1'b1;
        ds_pwm_d_data = 0;
        ds_pwm_q_data = 50;
        ds_pwm_dq_valid = 1'b1;
        @(posedge clk);
        ss_pwm_data = 'X;
        ss_pwm_valid = 1'b0;
        ds_pwm_d_data = 'X;
        ds_pwm_q_data = 'X;
        ds_pwm_dq_valid = 1'b0;
        @(posedge clk);
        trigger <= 1'b1;
        @(posedge clk);
        trigger <= 1'b0;
        repeat(87) @(posedge clk);
    end

    // Change Theta
    always begin
        # 20ns
        theta_in_deg <= theta_in_deg + 0.1;
    end
    
    // Clock Generation
    always #5ns begin
        clk <= ~clk;
    end
    
    // Reset generaton
    initial begin
        reset <= 1'b1;
        repeat(5) @(negedge clk);
        reset <= 1'b0;
        repeat(8000) @(negedge clk);
        $stop;
    end
endmodule
