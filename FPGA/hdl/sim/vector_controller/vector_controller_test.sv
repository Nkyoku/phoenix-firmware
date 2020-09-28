`timescale 1 ns / 1 ns
module test ();
    real PI = 3.14159265359;
    real Motor_P = 8; // 極ペア
    real Motor_R = 6.89 * 3;
    real Motor_L = 5.85e-3 * 3;
    real Motor_Kv = 0.131352085; // 逆起電力係数 V/(rad/s)
    real Motor_Kt = 0.131; // トルク係数 Nm/A
    real Motor_J = 0.0000181; // Kgm^2
    real Motor_D = 0.001;
    real F_PWM = 50000;
    
    int param_kp = 10000;
    int param_ki = 1000;
    
    // Clock (10MHz)
    logic clk = 1'b0;
    always #50ns begin
        clk <= ~clk;
    end
    
    // Reset
    logic reset = 1'b1;
    initial begin
        reset <= 1'b1;
        repeat(5) @(negedge clk);
        reset <= 1'b0;
        repeat(10000) @(negedge clk);
        //$stop;
    end
    
    // 1kHz Pulse
    logic pulse_1khz = 1'b0;
    always begin
        repeat(10) @(posedge clk);
        pulse_1khz <= 1'b1;
        @(posedge clk);
        pulse_1khz <= 1'b0;
        repeat(9989) @(posedge clk);
    end
    
    // 8kHz Pulse
    logic pulse_8khz = 1'b0;
    always begin
        repeat(10) @(posedge clk);
        pulse_8khz <= 1'b1;
        @(posedge clk);
        pulse_8khz <= 1'b0;
        repeat(1239) @(posedge clk);
    end
    
    // 50kHz Current sensing pulse
    logic pulse_50khz = 1'b0;
    always begin
        repeat(100) @(posedge clk);
        pulse_50khz <= 1'b1;
        @(posedge clk);
        pulse_50khz <= 1'b0;
        repeat(99) @(posedge clk);
    end
    
    // Status
    logic fault = 1'b0;
    logic status_driver_otw_n;
    logic status_driver_fault_n;
    logic status_hall_fault_n;
    logic status_encoder_fault_n;
    always @(posedge clk) begin
        if (~status_driver_otw_n | ~status_driver_fault_n | ~status_hall_fault_n | ~status_encoder_fault_n) begin
            fault <= 1'b1;
        end
        else if (pulse_1khz == 1'b1) begin
            fault <= 1'b0;
        end
    end
    
    // Motor driver (実際には8回転で回転子は1回転する)
    logic driver_otw_n = 1'b1;
    logic driver_fault_n = 1'b1;
    logic unsigned [15:0] driver_pwm_u_data;
    logic unsigned [15:0] driver_pwm_v_data;
    logic unsigned [15:0] driver_pwm_w_data;
    logic driver_pwm_valid = 1'b0;
    
    // Current sensor
    logic signed [15:0] current_u_data;
    logic signed [15:0] current_v_data;
    logic signed [15:0] current_w_data;
    logic current_uv_valid = 1'b0;
    logic signed [15:0] current_ref_d_data;
    logic signed [15:0] current_ref_q_data;
    logic current_ref_dq_valid = 1'b0;
    
    logic signed [15:0] current_a, current_b;
    logic signed [15:0] current_d, current_q;
    
    // Motor model
    real theta = 0.4; // rad
    real omega = 0.0; // rad/s
    real V_u = 0.0, V_v = 0.0, V_w = 0.0;
    real V_d, V_q;
    real I_d, I_q;
    real I_u, I_v, I_w;
    real T;
    always @(posedge clk) begin
        real dI_d_dt, dI_q_dt;
        real domega_dt;
        if (driver_pwm_valid == 1'b1) begin
            V_u = (2 * $itor(driver_pwm_u_data) - $itor(driver_pwm_v_data) - $itor(driver_pwm_w_data)) / 3000 * 48;
            V_v = (2 * $itor(driver_pwm_v_data) - $itor(driver_pwm_u_data) - $itor(driver_pwm_w_data)) / 3000 * 48;
            V_w = (2 * $itor(driver_pwm_w_data) - $itor(driver_pwm_u_data) - $itor(driver_pwm_v_data)) / 3000 * 48;
        end
        if (pulse_50khz == 1'b1) begin
            V_d = (V_u * $cos(theta * Motor_P) + V_v * $cos(theta * Motor_P - 2 * PI / 3) + V_w * $cos(theta * Motor_P + 2 * PI / 3)) * 0.816496581;
            V_q = (V_u * $sin(theta * Motor_P) + V_v * $sin(theta * Motor_P - 2 * PI / 3) + V_w * $sin(theta * Motor_P + 2 * PI / 3)) * -0.816496581;
            dI_d_dt = -Motor_R / Motor_L * I_d + omega * I_q + V_d / Motor_L;
            dI_q_dt = -omega * I_d - Motor_R / Motor_L * I_q + V_q / Motor_L - omega * Motor_Kv / Motor_L;
            I_d = I_d + dI_d_dt / F_PWM;
            I_q = I_q + dI_q_dt / F_PWM;
            I_u = (I_d * $cos(theta * Motor_P) - I_q * $sin(theta * Motor_P)) * 0.816496581;
            I_v = (I_d * $cos(theta * Motor_P - 2 * PI / 3) - I_q * $sin(theta * Motor_P - 2 * PI / 3)) * 0.816496581;
            I_w = (I_d * $cos(theta * Motor_P + 2 * PI / 3) - I_q * $sin(theta * Motor_P + 2 * PI / 3)) * 0.816496581;
            T = Motor_P * Motor_Kt * I_q;
            domega_dt = -Motor_D / Motor_J * omega + T / Motor_J;
            omega = omega + domega_dt / F_PWM;
            theta <= theta + omega / F_PWM / Motor_P;
            current_u_data <= $rtoi((I_u < -13.6 ? -13.6 : (13.6 < I_u ? 13.6 : I_u)) * 0.003 * 50 / 2.048 * 4096 * 6.18);
            current_v_data <= $rtoi((I_v < -13.6 ? -13.6 : (13.6 < I_v ? 13.6 : I_v)) * 0.003 * 50 / 2.048 * 4096 * 6.18);
            current_w_data <= -current_u_data - current_v_data;
            current_uv_valid <= 1'b1;
            current_a <= $rtoi((I_u - 0.5 * I_v - 0.5 * (-I_u - I_v)) * 0.003 * 50 / 2.048 * 4096 * 6.18 * 0.816496581);
            current_b <= $rtoi((0.866025404 * I_u + 0.866025404 * 2 * I_v) * 0.003 * 50 / 2.048 * 4096 * 6.18 * 0.816496581);
            current_d <= $rtoi((I_u * $cos(theta * Motor_P) + I_v * $cos(theta * Motor_P - 2 * PI / 3) + I_w * $cos(theta * Motor_P + 2 * PI / 3)) * 0.003 * 50 / 2.048 * 4096 * 6.18 * 0.816496581);
            current_q <= $rtoi((I_u * $sin(theta * Motor_P) + I_v * $sin(theta * Motor_P - 2 * PI / 3) + I_w * $sin(theta * Motor_P + 2 * PI / 3)) * 0.003 * 50 / 2.048 * 4096 * 6.18 * -0.816496581);
        end
        else begin
            current_u_data <= 'X;
            current_v_data <= 'X;
            current_uv_valid <= 1'b0;
        end
    end
    
    // Error test
    logic error_test = 1'b0;
    initial begin
        repeat(500000) @(posedge clk);
        error_test <= 1'b1;
        repeat(4567) @(posedge clk);
        error_test <= 1'b0;
    end
    
    // Hall sensors and encoder
    logic [2:0] hall_uvw = '0;
    logic [1:0] enc_ab = '0;
    real theta_deg;
    assign theta_deg = theta * Motor_P / PI * 180;
    always @(theta_deg) begin
        int t;
        t = $rtoi(8000 * theta_deg) % 2880000;
        if (t < 0) t = t + 2880000;
        hall_uvw[2] <= ~error_test ? ((t < 720000) | (2160000 <= t)) : 1'b0;
        hall_uvw[1] <= ~error_test ? ((240000 < t) & (t <= 1680000)) : 1'b0;
        hall_uvw[0] <= ~error_test ? ((1200000 < t) & (t <= 2640000)) : 1'b0;
        enc_ab[1] <= (t / 11250) % 2;
        enc_ab[0] <= ((t + 5625) / 11250) % 2;
    end
    
    // Vector Controller
    vector_controller #(
        .INVERSE_ENCODER(0)
    ) vec (
        .clk(clk),
        .reset(reset),
        .fault(fault),
        .pulse_1khz(pulse_1khz),
        .pulse_8khz(pulse_8khz),
        .sensor_hall_uvw(hall_uvw),
        .sensor_encoder_ab(enc_ab),
        .driver_otw_n(driver_otw_n),
        .driver_fault_n(driver_fault_n),
        .driver_pwm_data({driver_pwm_u_data, driver_pwm_v_data, driver_pwm_w_data}),
        .driver_pwm_valid(driver_pwm_valid),
        .driver_pwm_ready(1'b1),
        .status_driver_otw_n(status_driver_otw_n),
        .status_driver_fault_n(status_driver_fault_n),
        .status_hall_fault_n(status_hall_fault_n),
        .status_encoder_fault_n(status_encoder_fault_n),
        .current_u_data(current_u_data),
        .current_u_valid(current_uv_valid),
        .current_v_data(current_v_data),
        .current_v_valid(current_uv_valid),
        .current_reference_data({current_ref_d_data, current_ref_q_data}),
        .current_reference_valid(current_ref_dq_valid),
        .current_measurement_data(),
        .current_measurement_valid(),
        .encoder_data(),
        .param_kp(param_kp),
        .param_ki(param_ki)
    );
    //assign driver_pwm_u_data = 1500;
    //assign driver_pwm_v_data = 2000;
    //assign driver_pwm_w_data = 1000;
    
    // Reference
    always begin
        repeat(10) @(posedge clk);
        current_ref_dq_valid <= 1'b1;
        current_ref_d_data <= '0;
        current_ref_q_data <= 1000;
        @(posedge clk);
        current_ref_dq_valid <= 1'b0;
        current_ref_d_data <= 'X;
        current_ref_q_data <= 'X;
        repeat(9989) @(posedge clk);
    end
endmodule
