module vector_controller #(
        parameter int INVERSE_ENCODER = 0,
        parameter int THETA_WIDTH = 9,
        parameter int PWM_WIDTH = 16,
        parameter int DRIVER_WIDTH = 12,
        parameter int CURRENT_WIDTH = 16,
        parameter int ENCODER_WIDTH = 16,
        parameter int GAIN_WIDTH = 16,
        parameter int GAIN_SCALE = 14
    ) (
        input  wire                       clk,
        input  wire                       reset,
        input  wire                       fault, // フォルト発生時に内部状態をリセットする
        input  wire                       pulse_1khz, // エンコーダ回転速度のラッチタイミングパルス
        input  wire                       pulse_8khz, // 電流制御のタイミングパルス
        input  wire [2:0]                 sensor_hall_uvw, // {u, v, w}
        input  wire [1:0]                 sensor_encoder_ab, // {a, b}
        input  wire                       driver_otw_n,
        input  wire                       driver_fault_n,
        output wire [3*DRIVER_WIDTH-1:0]  driver_pwm_data, // {u, v, w}
        output wire                       driver_pwm_valid,
        input  wire                       driver_pwm_ready,
        output wire                       status_driver_otw_n,
        output wire                       status_driver_fault_n,
        output wire                       status_hall_fault_n,
        output wire                       status_encoder_fault_n,
        output reg  [THETA_WIDTH-1:0]     position_theta,
        output reg                        position_error,
        output reg                        position_uncertain,
        input  wire [2*CURRENT_WIDTH-1:0] current_uv_data,
        input  wire                       current_uv_valid,
        input  wire [2*CURRENT_WIDTH-1:0] current_reference_data, // {d, q}
        input  wire                       current_reference_valid,
        output wire [2*CURRENT_WIDTH-1:0] current_measurement_data, // {d, q}
        output wire                       current_measurement_valid,
        output reg  [ENCODER_WIDTH-1:0]   encoder_data,
        input  wire [GAIN_WIDTH-1:0]      param_kp,
        input  wire [GAIN_WIDTH-1:0]      param_ki
    );
    
    // Filter nFAULT, nOTW
    deglitch #(
        .COUNTER_VALUE(15),
        .DEFAULT_LOGIC(1)
    ) deglitch_0 (
         .clk (clk),
        .in  (driver_otw_n),
        .out (status_driver_otw_n)
    );
    deglitch #(
        .COUNTER_VALUE(15),
        .DEFAULT_LOGIC(1)
    ) deglitch_1 (
        .clk (clk),
        .in  (driver_fault_n),
        .out (status_driver_fault_n)
    );
    
    // Quadrature Encoder Decoder
    logic qdec_inc;
    logic qdec_dec;
    assign status_encoder_fault_n = ~(qdec_inc & qdec_dec);
    quadrature_decoder #(
        .INVERSE(INVERSE_ENCODER),
        .DATA_WIDTH(ENCODER_WIDTH)
    ) qdec (
        .clk(clk),
        .reset(reset),
        .latch(pulse_1khz),
        .enc_a(sensor_encoder_ab[0]),
        .enc_b(sensor_encoder_ab[1]),
        .inc(qdec_inc),
        .dec(qdec_dec),
        .counter(encoder_data)
    );
    
    // Position Estimator
    logic [THETA_WIDTH-1:0] theta_data;
    logic theta_error;
    logic theta_uncertain;
    assign status_hall_fault_n = (sensor_hall_uvw != 3'b000) & (sensor_hall_uvw != 3'b111);
    position_estimator #(
        .THETA_WIDTH(THETA_WIDTH)
    ) pos (
        .clk(clk),
        .reset(reset),
        .hall_uvw(sensor_hall_uvw),
        .qdec_inc(qdec_inc),
        .qdec_dec(qdec_dec),
        .theta_data(theta_data),
        .theta_error(theta_error),
        .theta_uncertain(theta_uncertain)
    );
    
    // Clarke Transform
    logic [CURRENT_WIDTH-1:0] current_a_data, current_b_data;
    logic current_ab_valid;
    logic current_ab_ready;
    clarke_transform #(
        .CHANNEL_WIDTH(1),
        .DATA_WIDTH(CURRENT_WIDTH)
    ) clarke (
        .clk(clk),
        .reset(reset),
        .in_data(current_uv_data),
        .in_channel(1'b0),
        .in_valid(current_uv_valid),
        .in_ready(),
        .out_data({current_a_data, current_b_data}),
        .out_channel(),
        .out_valid(current_ab_valid),
        .out_ready(current_ab_ready)
    );
    
    /*// Current FIR Filter
    logic [CURRENT_WIDTH-1:0] current_a_data;
    logic [CURRENT_WIDTH-1:0] current_b_data;
    logic current_ab_valid;
    logic current_ab_ready;
    current_fir_filter #(
        .DATA_WIDTH(16),
        .DATA_COUNT(2)
    ) fir (
        .clk(clk),
        .reset(reset),
        .in_data({current_a_raw_data, current_b_raw_data}),
        .in_valid(current_ab_raw_valid),
        .in_ready(current_ab_raw_ready),
        .out_data({current_a_data, current_b_data}),
        .out_valid(current_ab_valid),
        .out_ready(current_ab_ready)
    );*/
    
    // Latching Position Estimator's output
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            position_theta <= '0;
            position_error <= 1'b1;
            position_uncertain <= 1'b1;
        end
        else begin
            if (current_uv_valid == 1'b1) begin
                position_theta <= theta_data;
                position_error <= theta_error;
                position_uncertain <= theta_uncertain;
            end
        end
    end
    
    // PI Controller
    logic [PWM_WIDTH-1:0] pi_d_data, pi_q_data;
    logic pi_dq_valid;
    logic [PWM_WIDTH-1:0] pwm_d_data, pwm_q_data;
    logic pwm_dq_available = 1'b0;
    logic pwm_dq_valid = 1'b0;
    logic pwm_dq_ready;
    pi_controller #(
        .SCALE(GAIN_SCALE),
        .GAIN_WIDTH(GAIN_WIDTH),
        .DATA_WIDTH(PWM_WIDTH),
        .DATA_COUNT(2),
        .DATA_LIMIT(2960)
    ) pi (
        .clk(clk),
        .reset(reset | fault),
        .trigger(pulse_8khz),
        .param_kp(param_kp),
        .param_ki(param_ki),
        .in_ref_data(current_reference_data),
        .in_ref_valid(current_reference_valid),
        .in_proc_data(current_measurement_data),
        .in_proc_valid(current_measurement_valid),
        .out_data({pi_d_data, pi_q_data}),
        .out_valid(pi_dq_valid)
    );
    always @(posedge clk, posedge reset, posedge fault) begin
        if (reset | fault) begin
            pwm_d_data <= '0;
            pwm_q_data <= '0;
            pwm_dq_available <= 1'b0;
            pwm_dq_valid <= 1'b0;
        end
        else begin
            if (pi_dq_valid == 1'b1) begin
                pwm_dq_available <= 1'b1;
                pwm_d_data <= pi_d_data;
                pwm_q_data <= pi_q_data;
            end
            if (pwm_dq_available & current_measurement_valid) begin
                pwm_dq_valid <= 1'b1;
            end
            else if (pwm_dq_valid & pwm_dq_ready) begin
                pwm_dq_valid <= 1'b0;
            end
        end
    end
    
    // (Inverse) Park Transform
    logic signed [CURRENT_WIDTH-1:0] current_meas_d_data, current_meas_q_data;
    assign current_measurement_data = {current_meas_d_data, current_meas_q_data};
    logic [PWM_WIDTH-1:0] pwm_a_data, pwm_b_data;
    logic pwm_ab_valid;
    logic pwm_ab_ready;
    muxed_park_transform #(
        .DATA_WIDTH(PWM_WIDTH), // PWM_WIDTH == CURRENT_WIDTH
        .THETA_WIDTH(THETA_WIDTH)
    ) park (
        .clk(clk),
        .reset(reset | fault),
        .in1_data({1'b0, position_theta, current_a_data, current_b_data}), // {normal, theta, a, b}
        .in1_valid(current_ab_valid),
        .in1_ready(current_ab_ready),
        .in2_data({1'b1, position_theta, pwm_d_data, pwm_q_data}), // {inverse, theta, d, q}
        .in2_valid(pwm_dq_valid),
        .in2_ready(pwm_dq_ready),
        .out1_data({current_meas_d_data, current_meas_q_data}), // {d, q}
        .out1_valid(current_measurement_valid),
        .out1_ready(1'b1),
        .out2_data({pwm_a_data, pwm_b_data}),
        .out2_valid(pwm_ab_valid),
        .out2_ready(pwm_ab_ready)
    );
    
    // Space Vector Modulation
    logic unsigned [DRIVER_WIDTH-1:0] driver_pwm_u_data, driver_pwm_v_data, driver_pwm_w_data;
    assign driver_pwm_data = {driver_pwm_u_data, driver_pwm_v_data, driver_pwm_w_data};
    space_vector_modulator #(
        .DATA_WIDTH(PWM_WIDTH),
        .OUT_WIDTH(DRIVER_WIDTH),
        .PERIOD(3000),
        .OUT_MIN(20),
        .OUT_MAX(2980)
    ) svm (
        .clk(clk),
        .reset(reset | fault),
        .in_data({pwm_a_data, pwm_b_data}),
        .in_valid(pwm_ab_valid),
        .in_ready(pwm_ab_ready),
        .out_data({driver_pwm_u_data, driver_pwm_v_data, driver_pwm_w_data}),
        .out_valid(driver_pwm_valid)
        // unused : driver_pwm_ready
    );
endmodule

module muxed_park_transform #(
        parameter int DATA_WIDTH = 16,
        parameter int THETA_WIDTH = 9
    ) (
        input  wire                              clk,
        input  wire                              reset,
        input  wire [THETA_WIDTH+2*DATA_WIDTH:0] in1_data,
        input  wire                              in1_valid,
        output reg                               in1_ready,
        input  wire [THETA_WIDTH+2*DATA_WIDTH:0] in2_data,
        input  wire                              in2_valid,
        output reg                               in2_ready,
        output wire [2*DATA_WIDTH-1:0]           out1_data,
        output wire                              out1_valid,
        input  wire                              out1_ready,
        output wire [2*DATA_WIDTH-1:0]           out2_data,
        output wire                              out2_valid,
        input  wire                              out2_ready
    );
    
    // Park transform
    logic [THETA_WIDTH+2*DATA_WIDTH:0] park_in_data;
    logic park_in_channel;
    logic park_in_valid;
    logic park_in_ready;
    logic [2*DATA_WIDTH-1:0] park_out_data;
    logic park_out_channel;
    logic park_out_valid;
    logic park_out_ready;
    park_transform #(
        .CHANNEL_WIDTH(1),
        .DATA_WIDTH(DATA_WIDTH),
        .THETA_WIDTH(THETA_WIDTH)
    ) park (
        .clk(clk),
        .reset(reset),
        .in_data(park_in_data),
        .in_channel(park_in_channel),
        .in_valid(park_in_valid),
        .in_ready(park_in_ready),
        .out_data(park_out_data),
        .out_channel(park_out_channel),
        .out_valid(park_out_valid),
        .out_ready(park_out_ready)
    );
    
    // Input multiplexer
    assign park_in_data = ~park_in_channel ? in1_data : in2_data;
    assign park_in_channel = ~in1_valid & in2_valid;
    assign park_in_valid = in1_valid | in2_valid;
    assign in1_ready = park_in_ready;
    assign in2_ready = ~in1_valid & park_in_ready;
    
    // Output demultiplexer
    assign out1_data = park_out_data;
    assign out2_data = park_out_data;
    assign out1_valid = park_out_valid & ~park_out_channel;
    assign out2_valid = park_out_valid & park_out_channel;
    assign park_out_ready = ~park_out_channel ? out1_ready : out2_ready;
endmodule
