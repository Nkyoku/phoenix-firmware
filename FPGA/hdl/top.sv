module PhoenixFPGA (
        input  wire ADC1_CLKOUT,
        output wire ADC1_SCK,
        input  wire ADC1_SDO1,
        input  wire ADC1_SDO2,
        input  wire ADC1_SDO3,
        input  wire ADC1_SDO4,
        input  wire ADC1_SDO5,
        input  wire ADC1_SDO6,
        input  wire ADC1_SDO7,
        input  wire ADC1_SDO8,
        output wire ADC1_CNV_N,
        inout  wire ADC2_SCL,
        inout  wire ADC2_SDA,
        input  wire CLK25MHz,
        output wire DCLK,
        inout  wire ASDO,
        inout  wire ASDI,
        inout  wire DATA2,
        inout  wire DATA3,
        output wire nCSO,
        output wire FPGA_INT,
        output wire FPGA_SPI_MISO,
        input  wire FPGA_SPI_MOSI,
        input  wire FPGA_SPI_SCLK,
        input  wire FPGA_SPI_CS0_N,
        input  wire FPGA_UART_RX,
        output wire FPGA_UART_TX,
        input  wire GP_STOP_N,
        output wire HALL1_CLK,
        input  wire HALL1_DOUT,
        output wire HALL1_LOAD_N,
        output wire HALL2_CLK,
        input  wire HALL2_DOUT,
        output wire HALL2_LOAD_N,
        output wire IMU_CLKIN,
        input  wire IMU_INT_N,
        input  wire IMU_MISO,
        output wire IMU_MOSI,
        output wire IMU_SCLK,
        output wire IMU_CS_N,
        input  wire MOTOR1_ENC_A,
        input  wire MOTOR1_ENC_B,
        output wire MOTOR1_LED,
        output wire MOTOR1_PWM_A,
        output wire MOTOR1_PWM_B,
        output wire MOTOR1_PWM_C,
        output wire MOTOR1_SW_EN,
        input  wire MOTOR1_SW_FLT_N,
        input  wire MOTOR1_FAULT_N,
        input  wire MOTOR1_OTW_N,
        output wire MOTOR1_RESET_A_N,
        output wire MOTOR1_RESET_B_N,
        output wire MOTOR1_RESET_C_N,
        input  wire MOTOR2_ENC_A,
        input  wire MOTOR2_ENC_B,
        output wire MOTOR2_LED,
        output wire MOTOR2_PWM_A,
        output wire MOTOR2_PWM_B,
        output wire MOTOR2_PWM_C,
        output wire MOTOR2_SW_EN,
        input  wire MOTOR2_SW_FLT_N,
        input  wire MOTOR2_FAULT_N,
        input  wire MOTOR2_OTW_N,
        output wire MOTOR2_RESET_A_N,
        output wire MOTOR2_RESET_B_N,
        output wire MOTOR2_RESET_C_N,
        input  wire MOTOR3_ENC_A,
        input  wire MOTOR3_ENC_B,
        output wire MOTOR3_LED,
        output wire MOTOR3_PWM_A,
        output wire MOTOR3_PWM_B,
        output wire MOTOR3_PWM_C,
        output wire MOTOR3_SW_EN,
        input  wire MOTOR3_SW_FLT_N,
        input  wire MOTOR3_FAULT_N,
        input  wire MOTOR3_OTW_N,
        output wire MOTOR3_RESET_A_N,
        output wire MOTOR3_RESET_B_N,
        output wire MOTOR3_RESET_C_N,
        input  wire MOTOR4_ENC_A,
        input  wire MOTOR4_ENC_B,
        output wire MOTOR4_LED,
        output wire MOTOR4_PWM_A,
        output wire MOTOR4_PWM_B,
        output wire MOTOR4_PWM_C,
        output wire MOTOR4_SW_EN,
        input  wire MOTOR4_SW_FLT_N,
        input  wire MOTOR4_FAULT_N,
        input  wire MOTOR4_OTW_N,
        output wire MOTOR4_RESET_A_N,
        output wire MOTOR4_RESET_B_N,
        output wire MOTOR4_RESET_C_N,
        output wire MOTOR5_LED,
        output wire MOTOR5_PWM_A,
        output wire MOTOR5_PWM_B,
        output wire MOTOR5_PWM_C,
        output wire MOTOR5_SW_EN,
        input  wire MOTOR5_SW_FLT_N,
        input  wire MOTOR5_FAULT_N,
        input  wire MOTOR5_OTW_N,
        output wire MOTOR5_RESET_A_N,
        output wire MOTOR5_RESET_B_N,
        output wire MOTOR5_RESET_C_N,
        output wire PSRAM1_SCLK,
        inout  wire PSRAM1_SIO0,
        inout  wire PSRAM1_SIO1,
        inout  wire PSRAM1_SIO2,
        inout  wire PSRAM1_SIO3,
        output wire PSRAM1_CS_N,
        output wire PSRAM2_SCLK,
        inout  wire PSRAM2_SIO0,
        inout  wire PSRAM2_SIO1,
        inout  wire PSRAM2_SIO2,
        inout  wire PSRAM2_SIO3,
        output wire PSRAM2_CS_N,
        input  wire MOD_SLEEP_N
    );
    
    //assign ADC1_SCK = 1'b0;
    //assign ADC1_CNV_N = 1'b1;
    //assign ADC2_SCL = 1'bz;
    //assign ADC2_SDA = 1'bz;
    assign DCLK = 1'b0;
    assign ASDI = 1'bz;
    assign ASDO = 1'bz;
    assign DATA2 = 1'bz;
    assign DATA3 = 1'bz;
    assign nCSO = 1'b1;
    //assign FPGA_INT = 1'b0;
    //assign FPGA_SPI_MISO = 1'bz;
    //assign FPGA_UART_TX = 1'bz;
    //assign HALL1_CLK = 1'b0;
    //assign HALL1_LOAD_N = 1'b1;
    //assign HALL2_CLK = 1'b0;
    //assign HALL2_LOAD_N = 1'b1;
    //assign IMU_CLKIN = 1'b0;
    //assign IMU_MOSI = 1'b0;
    //assign IMU_SCLK = 1'b0;
    //assign IMU_CS_N = 1'b1;
    //assign MOTOR1_LED = 1'b1;
    //assign MOTOR1_PWM_A = 1'b0;
    //assign MOTOR1_PWM_B = 1'b0;
    //assign MOTOR1_PWM_C = 1'b0;
    //assign MOTOR1_SW_EN = 1'b0;
    //assign MOTOR1_RESET_A_N = 1'b0;
    //assign MOTOR1_RESET_B_N = 1'b0;
    //assign MOTOR1_RESET_C_N = 1'b0;
    //assign MOTOR2_LED = 1'b1;
    //assign MOTOR2_PWM_A = 1'b0;
    //assign MOTOR2_PWM_B = 1'b0;
    //assign MOTOR2_PWM_C = 1'b0;
    //assign MOTOR2_SW_EN = 1'b0;
    //assign MOTOR2_RESET_A_N = 1'b0;
    //assign MOTOR2_RESET_B_N = 1'b0;
    //assign MOTOR2_RESET_C_N = 1'b0;
    //assign MOTOR3_LED = 1'b1;
    //assign MOTOR3_PWM_A = 1'b0;
    //assign MOTOR3_PWM_B = 1'b0;
    //assign MOTOR3_PWM_C = 1'b0;
    //assign MOTOR3_SW_EN = 1'b0;
    //assign MOTOR3_RESET_A_N = 1'b0;
    //assign MOTOR3_RESET_B_N = 1'b0;
    //assign MOTOR3_RESET_C_N = 1'b0;
    //assign MOTOR4_LED = 1'b1;
    //assign MOTOR4_PWM_A = 1'b0;
    //assign MOTOR4_PWM_B = 1'b0;
    //assign MOTOR4_PWM_C = 1'b0;
    //assign MOTOR4_SW_EN = 1'b0;
    //assign MOTOR4_RESET_A_N = 1'b0;
    //assign MOTOR4_RESET_B_N = 1'b0;
    //assign MOTOR4_RESET_C_N = 1'b0;
    //assign MOTOR5_LED = 1'b1;
    //assign MOTOR5_PWM_A = 1'b0;
    //assign MOTOR5_PWM_B = 1'b0;
    //assign MOTOR5_PWM_C = 1'b0;
    //assign MOTOR5_SW_EN = 1'b0;
    //assign MOTOR5_RESET_A_N = 1'b0;
    //assign MOTOR5_RESET_B_N = 1'b0;
    //assign MOTOR5_RESET_C_N = 1'b0;
    assign PSRAM1_SCLK = 1'b0;
    assign PSRAM1_SIO0 = 1'bz;
    assign PSRAM1_SIO1 = 1'bz;
    assign PSRAM1_SIO2 = 1'bz;
    assign PSRAM1_SIO3 = 1'bz;
    assign PSRAM1_CS_N = 1'b1;
    assign PSRAM2_SCLK = 1'b0;
    assign PSRAM2_SIO0 = 1'bz;
    assign PSRAM2_SIO1 = 1'bz;
    assign PSRAM2_SIO2 = 1'bz;
    assign PSRAM2_SIO3 = 1'bz;
    assign PSRAM2_CS_N = 1'b1;

    // Clock and reset generation
    logic clk_150mhz;
    logic clk_100mhz;
    logic clk_75mhz;
    logic clk_64khz;
    logic pll_locked;
    logic reset_25mhz = 1'b1;
    pll pll_0 (
	    .inclk0 (CLK25MHz),
	    .c0     (clk_150mhz),
	    .c1     (clk_100mhz),
        .c2     (clk_75mhz),
        .c3     (clk_64khz),
	    .locked (pll_locked)
    );
    always @(posedge CLK25MHz) begin
        reset_25mhz <= ~pll_locked;
    end
    
    // Reset generation for clk_150mhz
    logic reset_150mhz;
    ResetSynchronizer reset_0 (
        .clk       (clk_150mhz),
        .reset_in  (reset_25mhz),
        .reset_out (reset_150mhz)
    );
    
    // Reset generation for clk_100mhz
    logic reset_100mhz;
    ResetSynchronizer reset_1 (
        .clk       (clk_100mhz),
        .reset_in  (reset_25mhz),
        .reset_out (reset_100mhz)
    );
    
    // Reset generation for clk_75mhz
    logic reset_75mhz;
    ResetSynchronizer reset_2 (
        .clk       (clk_75mhz),
        .reset_in  (reset_25mhz),
        .reset_out (reset_75mhz)
    );
    
    // Generate control timing pulse from IMU's interrupt signal
    logic pulse_8khz;
    logic pulse_1khz;
    ControlPulseGenerator pulse_0 (
        .clk        (clk_75mhz),
        .reset      (reset_75mhz),
        .clk_64khz  (clk_64khz),
        .imu_clkin  (IMU_CLKIN),
        .imu_int_n  (IMU_INT_N),
        .pulse_8khz (pulse_8khz),
        .pulse_1khz (pulse_1khz)
    );
    
    // Hall sensor inputs
    logic [2:0] sensor_hall_uvw [1:5]; // {u, v, w}
    SerialToParallel #(
        .INVERSE(1)
    ) serial_0 (
        .reset     (reset_25mhz),
        .clk       (CLK25MHz),
        .sr_load_n (HALL1_LOAD_N),
        .sr_clk    (HALL1_CLK),
        .sr_dout   (HALL1_DOUT),
        .input_ser (),
        .input_a   (sensor_hall_uvw[2][0]),
        .input_b   (sensor_hall_uvw[2][1]),
        .input_c   (sensor_hall_uvw[2][2]),
        .input_d   (),
        .input_e   (sensor_hall_uvw[1][0]),
        .input_f   (sensor_hall_uvw[1][1]),
        .input_g   (sensor_hall_uvw[1][2]),
        .input_h   ()
    );
    SerialToParallel #(
        .INVERSE(1)
    ) serial_1 (
        .reset     (reset_25mhz),
        .clk       (CLK25MHz),
        .sr_load_n (HALL2_LOAD_N),
        .sr_clk    (HALL2_CLK),
        .sr_dout   (HALL2_DOUT),
        .input_ser (sensor_hall_uvw[5][2]),
        .input_a   (sensor_hall_uvw[5][1]),
        .input_b   (sensor_hall_uvw[4][0]),
        .input_c   (sensor_hall_uvw[4][1]),
        .input_d   (sensor_hall_uvw[4][2]),
        .input_e   (sensor_hall_uvw[3][0]),
        .input_f   (sensor_hall_uvw[3][1]),
        .input_g   (sensor_hall_uvw[3][2]),
        .input_h   (sensor_hall_uvw[5][0])
    );
    
    // Filter of GP_STOP_N
    logic gp_stop_filtered_n;
    deglitch #(
        .COUNTER_VALUE(125),
        .DEFAULT_LOGIC(1)
    ) deglitch_0 (
        .clk (CLK25MHz),
        .in  (GP_STOP_N),
        .out (gp_stop_filtered_n)
    );
    
    // Filter of MOD_SLEEP_N
    logic mod_sleep_filtered_n;
    deglitch #(
        .COUNTER_VALUE(125),
        .DEFAULT_LOGIC(1)
    ) deglitch_1 (
        .clk (CLK25MHz),
        .in  (MOD_SLEEP_N),
        .out (mod_sleep_filtered_n)
    );
    
    // ADC1 (LTC2320-12)
    logic ain_valid;
    logic signed [12:0] ain_data[1:8];
    ltc2320 ltc2320_0 (
        .clk_100mhz   (clk_100mhz),
        .reset_100mhz (reset_100mhz),
        .clk_150mhz   (clk_150mhz),
        .reset_150mhz (reset_150mhz),
        .adc_sck      (ADC1_SCK),
        .adc_cnv_n    (ADC1_CNV_N),
        .adc_sdo      ({ADC1_SDO8, ADC1_SDO7, ADC1_SDO6, ADC1_SDO5, ADC1_SDO4, ADC1_SDO3, ADC1_SDO2, ADC1_SDO1}),
        .adc_clkout   (ADC1_CLKOUT),
        .ain_valid    (ain_valid),
        .ain1_data    (ain_data[1]),
        .ain2_data    (ain_data[2]),
        .ain3_data    (ain_data[3]),
        .ain4_data    (ain_data[4]),
        .ain5_data    (ain_data[5]),
        .ain6_data    (ain_data[6]),
        .ain7_data    (ain_data[7]),
        .ain8_data    (ain_data[8])
    );
    
    // ADC1 CIC Filters
    logic [1:8] cic_valid_150mhz;
    logic [1:8] cic_valid;
    logic signed [15:0] cic_data_150mhz[1:8];
    logic signed [15:0] cic_data[1:8];
    logic signed [15:0] cic_u_data[1:4];
    wire [1:4] cic_u_valid = {cic_valid[3], cic_valid[5], cic_valid[7], cic_valid[1]};
    logic signed [15:0] cic_v_data[1:4];
    wire [1:4] cic_v_valid = {cic_valid[4], cic_valid[6], cic_valid[8], cic_valid[2]};
    assign {cic_u_data[1], cic_u_data[2], cic_u_data[3], cic_u_data[4]} = {cic_data[3], cic_data[5], cic_data[7], cic_data[1]};
    assign cic_v_data[1] = cic_data[4];
    assign cic_v_data[2] = cic_data[6];
    assign cic_v_data[3] = cic_data[8];
    assign cic_v_data[4] = cic_data[2];
    genvar i;
    generate
        for (i = 1; i <= 8; i = i + 1) begin : filters
            current_cic_filter cic (
                .clk(clk_150mhz),
                .reset(reset_150mhz),
                .in_valid(ain_valid),
                .in_data(ain_data[i]),
                .out_valid(cic_valid_150mhz[i]),
                .out_data(cic_data_150mhz[i])
            );
            avalon_st_clock_bridge #(
                .DATA_WIDTH(16)
            ) bridge (
                .clk1(clk_150mhz),
                .reset1(reset_150mhz),
                .sink_data(cic_data_150mhz[i]),
                .sink_valid(cic_valid_150mhz[i]),
                .clk2(clk_75mhz),
                .reset2(reset_75mhz),
                .source_data(cic_data[i]),
                .source_valid(cic_valid[i]),
                .source_ready(1'b1)
            );
            /*cdb_data_module #(
                .DATA_BITWIDTH(16)
            ) bridge (
                .in_rst(reset_150mhz),
                .in_clk(clk_150mhz),
                .in_valid(cic_valid_150mhz[i]),
                .in_data(cic_data_150mhz[i]),
                .out_rst(reset_75mhz),
                .out_clk(clk_75mhz),
                .out_valid(cic_valid[i]),
                .out_data(cic_data[i])
            );*/
        end
    endgenerate
    
    // PWM drivers
    // Controller 1...4 -> Driver 1...4 -> Motor 1...4 (Wheel motors)
    // Controller 5     -> Driver 5     -> Motor 5 (Dribble motor)
    wire pwm_trigger = cic_valid_150mhz[1];
    logic pwm_fault, pwm_fault_5;
    logic pwm_fault_150mhz, pwm_fault_5_150mhz;
    logic [47:0] controller_pwm_data [1:4];
    logic [15:0] controller_5_pwm_data;
    logic [1:5] controller_pwm_valid;
    logic [1:5] controller_pwm_ready;
    logic [47:0] driver_pwm_data [1:4];
    logic [15:0] driver_5_pwm_data;
    logic [1:5] driver_pwm_valid;
    logic [2:0] driver_pwm [1:5];
    logic [2:0] driver_reset_n [1:5];
    assign MOTOR1_PWM_A = driver_pwm[1][0];
    assign MOTOR1_PWM_B = driver_pwm[1][1];
    assign MOTOR1_PWM_C = driver_pwm[1][2];
    assign MOTOR2_PWM_A = driver_pwm[2][0];
    assign MOTOR2_PWM_B = driver_pwm[2][1];
    assign MOTOR2_PWM_C = driver_pwm[2][2];
    assign MOTOR3_PWM_A = driver_pwm[3][0];
    assign MOTOR3_PWM_B = driver_pwm[3][1];
    assign MOTOR3_PWM_C = driver_pwm[3][2];
    assign MOTOR4_PWM_A = driver_pwm[4][0];
    assign MOTOR4_PWM_B = driver_pwm[4][1];
    assign MOTOR4_PWM_C = driver_pwm[4][2];
    assign MOTOR5_PWM_A = driver_pwm[5][0];
    assign MOTOR5_PWM_B = driver_pwm[5][1];
    assign MOTOR5_PWM_C = driver_pwm[5][2];
    assign MOTOR1_RESET_A_N = driver_reset_n[1][0];
    assign MOTOR1_RESET_B_N = driver_reset_n[1][1];
    assign MOTOR1_RESET_C_N = driver_reset_n[1][2];
    assign MOTOR2_RESET_A_N = driver_reset_n[2][0];
    assign MOTOR2_RESET_B_N = driver_reset_n[2][1];
    assign MOTOR2_RESET_C_N = driver_reset_n[2][2];
    assign MOTOR3_RESET_A_N = driver_reset_n[3][0];
    assign MOTOR3_RESET_B_N = driver_reset_n[3][1];
    assign MOTOR3_RESET_C_N = driver_reset_n[3][2];
    assign MOTOR4_RESET_A_N = driver_reset_n[4][0];
    assign MOTOR4_RESET_B_N = driver_reset_n[4][1];
    assign MOTOR4_RESET_C_N = driver_reset_n[4][2];
    assign MOTOR5_RESET_A_N = driver_reset_n[5][0];
    assign MOTOR5_RESET_B_N = driver_reset_n[5][1];
    assign MOTOR5_RESET_C_N = driver_reset_n[5][2];
    wire [1:5] driver_otw_n = {MOTOR1_OTW_N, MOTOR2_OTW_N, MOTOR3_OTW_N, MOTOR4_OTW_N, MOTOR5_OTW_N};
    wire [1:5] driver_fault_n = {MOTOR1_FAULT_N, MOTOR2_FAULT_N, MOTOR3_FAULT_N, MOTOR4_FAULT_N, MOTOR5_FAULT_N};
    wire [1:4] sensor_encoder_a = {MOTOR1_ENC_A, MOTOR2_ENC_A, MOTOR3_ENC_A, MOTOR4_ENC_A};
    wire [1:4] sensor_encoder_b = {MOTOR1_ENC_B, MOTOR2_ENC_B, MOTOR3_ENC_B, MOTOR4_ENC_B};
    logic [5:1] status_driver_otw_n;
    logic [5:1] status_driver_fault_n;
    logic [5:1] status_hall_fault_n;
    logic [4:1] status_encoder_fault_n;
    logic [4:1] status_pos_error;
    logic [4:1] status_pos_uncertain;
    logic [31:0] current_reference_data [1:4];
    logic [1:4] current_reference_valid;
    logic [31:0] current_measurement_data [1:4];
    logic [1:4] current_measurement_valid;
    logic signed [15:0] encoder_data [1:4];
    logic signed [15:0] param_kp;
    logic signed [15:0] param_ki;
    generate
        for (i = 1; i <= 4; i = i + 1) begin : drivers // Motor 1...4 PWM driver
            ds_pwm_driver #(
                .PERIOD(3000),
                .MAX_ON_CYCLES(2985)
            ) driver (
                .clk(clk_150mhz),
                .reset(reset_150mhz),
                .trigger(pwm_trigger),
                .fault(pwm_fault_150mhz),
                .pwm_valid(driver_pwm_valid[i]),
                .pwm_data(driver_pwm_data[i]),
                .driver_pwm(driver_pwm[i]),
                .driver_reset_n(driver_reset_n[i])
            );
            avalon_st_clock_bridge #(
                .DATA_WIDTH(48)
            ) bridge (
                .clk1(clk_75mhz),
                .reset1(reset_75mhz),
                .sink_data(controller_pwm_data[i]),
                .sink_valid(controller_pwm_valid[i]),
                .sink_ready(controller_pwm_ready[i]),
                .clk2(clk_150mhz),
                .reset2(reset_150mhz),
                .source_data(driver_pwm_data[i]),
                .source_valid(driver_pwm_valid[i]),
                .source_ready(1'b1)
            );
            /*cdb_data_module #(
                .DATA_BITWIDTH(48)
            ) bridge (
                .in_rst(reset_75mhz),
                .in_clk(clk_75mhz),
                .in_valid(controller_pwm_valid[i]),
                .in_data(controller_pwm_data[i]),
                .in_ready(controller_pwm_ready[i]),
                .out_rst(reset_150mhz),
                .out_clk(clk_150mhz),
                .out_valid(driver_pwm_valid[i]),
                .out_data(driver_pwm_data[i])
            );*/
            vector_controller #(
                .INVERSE_ENCODER(1)
            ) controller (
                .clk(clk_75mhz),
                .reset(reset_75mhz),
                .fault(pwm_fault),
                .pulse_1khz(pulse_1khz),
                .pulse_8khz(pulse_8khz),
                .sensor_hall_uvw(sensor_hall_uvw[i]),
                .sensor_encoder_ab({sensor_encoder_a[i], sensor_encoder_b[i]}),
                .driver_otw_n(driver_otw_n[i]),
                .driver_fault_n(driver_fault_n[i]),
                .driver_pwm_data(controller_pwm_data[i]),
                .driver_pwm_valid(controller_pwm_valid[i]),
                .driver_pwm_ready(controller_pwm_ready[i]),
                .status_driver_otw_n(status_driver_otw_n[i]),
                .status_driver_fault_n(status_driver_fault_n[i]),
                .status_hall_fault_n(status_hall_fault_n[i]),
                .status_encoder_fault_n(status_encoder_fault_n[i]),
                .position_theta(),
                .position_error(status_pos_error[i]),
                .position_uncertain(status_pos_uncertain[i]),
                .current_u_data(cic_u_data[i]),
                .current_u_valid(cic_u_valid[i]),
                .current_v_data(cic_v_data[i]),
                .current_v_valid(cic_v_valid[i]),
                .current_reference_data(current_reference_data[i]), // {d, q}
                .current_reference_valid(current_reference_valid[i]),
                .current_measurement_data(current_measurement_data[i]), // {d, q}
                .current_measurement_valid(current_measurement_valid[i]),
                .encoder_data(encoder_data[i]),
                .param_kp(param_kp),
                .param_ki(param_ki)
            );
        end
    endgenerate
    pwm_driver #( // Motor 5 PWM driver
        .PWM_MAX_ON_CYCLES(2985)
    ) driver_5 (
        .clk(clk_150mhz),
        .reset(reset_150mhz),
        .trigger(pwm_trigger),
        .fault(pwm_fault_5_150mhz),
        .pwm_sink_data(driver_5_pwm_data),
        .pwm_sink_valid(driver_pwm_valid[5]),
        .sensor_hall_uvw(sensor_hall_uvw[5]),
        .driver_otw_n(driver_otw_n[5]),
        .driver_fault_n(driver_fault_n[5]),
        .driver_pwm(driver_pwm[5]),
        .driver_reset_n(driver_reset_n[5]),
        .status_driver_otw_n(status_driver_otw_n[5]),
        .status_driver_fault_n(status_driver_fault_n[5]),
        .status_hall_fault_n(status_hall_fault_n[5])
    );
    avalon_st_clock_bridge #(
        .DATA_WIDTH(16)
    ) bridge_5 (
        .clk1(clk_75mhz),
        .reset1(reset_75mhz),
        .sink_data(controller_5_pwm_data),
        .sink_valid(controller_pwm_valid[5]),
        .sink_ready(controller_pwm_ready[5]),
        .clk2(clk_150mhz),
        .reset2(reset_150mhz),
        .source_data(driver_5_pwm_data),
        .source_valid(driver_pwm_valid[5]),
        .source_ready(1'b1)
    );
    /*cdb_data_module #(
        .DATA_BITWIDTH(16)
    ) bridge (
        .in_rst(reset_75mhz),
        .in_clk(clk_75mhz),
        .in_valid(controller_pwm_valid[5]),
        .in_data(controller_5_pwm_data),
        .in_ready(controller_pwm_ready[5]),
        .out_rst(reset_150mhz),
        .out_clk(clk_150mhz),
        .out_valid(driver_pwm_valid[5]),
        .out_data(driver_5_pwm_data)
    );*/
    cdb_signal_module pwm_fault_bridge (
        .in_rst(reset_75mhz),
        .in_clk(clk_75mhz),
        .in_sig(pwm_fault),
        .out_rst(reset_150mhz),
        .out_clk(clk_150mhz),
        .out_sig(pwm_fault_150mhz)
    );
    cdb_signal_module pwm_fault_5_bridge (
        .in_rst(reset_75mhz),
        .in_clk(clk_75mhz),
        .in_sig(pwm_fault_5),
        .out_rst(reset_150mhz),
        .out_clk(clk_150mhz),
        .out_sig(pwm_fault_5_150mhz)
    );
    
    
    
    /*wire msgdma_ready;
    wire msgdma_valid;
    wire [15:0] msgdma_data;
    wire [7:0] msgdma_channel;*/
    
    
    wire adc2_sda_oe;
    wire adc2_scl_oe;
    assign ADC2_SDA = adc2_sda_oe ? 1'b0 : 1'bz;
    assign ADC2_SCL = adc2_scl_oe ? 1'b0 : 1'bz;
    
    wire [0:0] pio_0_in = {
        pulse_1khz             // [0]
    };
    wire [31:0] pio_1_in = {
        1'b0,
        sensor_hall_uvw[5][0], // [30]
        sensor_hall_uvw[5][1], // [29]
        sensor_hall_uvw[5][2], // [28]
        MOTOR4_ENC_B,          // [27]
        MOTOR4_ENC_A,          // [26]
        sensor_hall_uvw[4][0], // [25]
        sensor_hall_uvw[4][1], // [24]
        sensor_hall_uvw[4][2], // [23]
        MOTOR3_ENC_B,          // [22]
        MOTOR3_ENC_A,          // [21]
        sensor_hall_uvw[3][0], // [20]
        sensor_hall_uvw[3][1], // [19]
        sensor_hall_uvw[3][2], // [18]
        MOTOR2_ENC_B,          // [17]
        MOTOR2_ENC_A,          // [16]
        sensor_hall_uvw[2][0], // [15]
        sensor_hall_uvw[2][1], // [14]
        sensor_hall_uvw[2][2], // [13]
        MOTOR1_ENC_B,          // [12]
        MOTOR1_ENC_A,          // [11]
        sensor_hall_uvw[1][0], // [10]
        sensor_hall_uvw[1][1], // [9]
        sensor_hall_uvw[1][2], // [8]
        1'b0,
        MOTOR5_SW_FLT_N,       // [6]
        MOTOR4_SW_FLT_N,       // [5]
        MOTOR3_SW_FLT_N,       // [4]
        MOTOR2_SW_FLT_N,       // [3]
        MOTOR1_SW_FLT_N,       // [2]
        mod_sleep_filtered_n,  // [1]
        gp_stop_filtered_n     // [0]
    };
    wire [10:0] pio_2_out;
    assign MOTOR1_SW_EN = pio_2_out[10];
    assign MOTOR2_SW_EN = pio_2_out[9];
    assign MOTOR3_SW_EN = pio_2_out[8];
    assign MOTOR4_SW_EN = pio_2_out[7];
    assign MOTOR5_SW_EN = pio_2_out[6];
    assign MOTOR5_LED   = pio_2_out[5];
    assign MOTOR4_LED   = pio_2_out[4];
    assign MOTOR3_LED   = pio_2_out[3];
    assign MOTOR2_LED   = pio_2_out[2];
    assign MOTOR1_LED   = pio_2_out[1];
    assign FPGA_INT     = pio_2_out[0];
    controller ctrl (
        .reset_reset_n             (~reset_75mhz),
        .clk_clk                   (clk_75mhz),
        .pio_0_export              (pio_0_in),
		.pio_1_export              (pio_1_in),
        .pio_2_export              (pio_2_out),
        .adc2_i2c_sda_in           (ADC2_SDA),
		.adc2_i2c_scl_in           (ADC2_SCL),
		.adc2_i2c_sda_oe           (adc2_sda_oe),
		.adc2_i2c_scl_oe           (adc2_scl_oe),
		.imu_spi_miso              (IMU_MISO),
		.imu_spi_mosi              (IMU_MOSI),
		.imu_spi_sclk              (IMU_SCLK),
		.imu_spi_cs_n              (IMU_CS_N),
		.imu_spi_int_n             (IMU_INT_N),
        .vc_fault_fault            (pwm_fault),
		.vc_status_driver_otw_n    (status_driver_otw_n[4:1]),
		.vc_status_driver_fault_n  (status_driver_fault_n[4:1]),
		.vc_status_hall_fault_n    (status_hall_fault_n[4:1]),
		.vc_status_encoder_fault_n (status_encoder_fault_n[4:1]),
		.vc_status_pos_error       (status_pos_error[4:1]),
        .vc_status_pos_uncertain   (status_pos_uncertain[4:1]),
        .vc_encoder_encoder_1_data (encoder_data[1]),
		.vc_encoder_encoder_2_data (encoder_data[2]),
		.vc_encoder_encoder_3_data (encoder_data[3]),
		.vc_encoder_encoder_4_data (encoder_data[4]),
		.vc_imeas1_data            (current_measurement_data[1]),
		.vc_imeas1_valid           (current_measurement_valid[1]),
		.vc_imeas2_data            (current_measurement_data[2]),
		.vc_imeas2_valid           (current_measurement_valid[2]),
		.vc_imeas3_data            (current_measurement_data[3]),
		.vc_imeas3_valid           (current_measurement_valid[3]),
		.vc_imeas4_data            (current_measurement_data[4]),
		.vc_imeas4_valid           (current_measurement_valid[4]),
		.vc_iref1_data             (current_reference_data[1]),
		.vc_iref1_valid            (current_reference_valid[1]),
		.vc_iref2_data             (current_reference_data[2]),
		.vc_iref2_valid            (current_reference_valid[2]),
		.vc_iref3_data             (current_reference_data[3]),
		.vc_iref3_valid            (current_reference_valid[3]),
		.vc_iref4_data             (current_reference_data[4]),
		.vc_iref4_valid            (current_reference_valid[4]),
		.vc_param_kp               (param_kp),
		.vc_param_ki               (param_ki),
        .mc_fault_fault            (pwm_fault_5),
		.mc_status_driver_otw_n    (status_driver_otw_n[5]),
		.mc_status_driver_fault_n  (status_driver_fault_n[5]),
		.mc_status_hall_fault_n    (status_hall_fault_n[5]),
		.mc_pwm_data               (controller_5_pwm_data),
		.mc_pwm_valid              (controller_pwm_valid[5]),
		.mc_pwm_ready              (controller_pwm_ready[5])
		/*.msgdma_source_ready       (msgdma_ready),
		.msgdma_source_valid       (msgdma_valid),
        .msgdma_source_data        (msgdma_data),
		.msgdma_source_channel     (msgdma_channel)*/
    );
    
    /*assign FPGA_SPI_MISO = 1'bz;
    host host_0 (
		.clk_clk             (clk_75mhz),
        .reset_reset_n       (~reset_75mhz),
		.clk100mhz_clk       (clk_100mhz),
		.reset100mhz_reset_n (~reset_100mhz),
        .msgdma_sink_ready   (msgdma_ready),
		.msgdma_sink_valid   (msgdma_valid),
		.msgdma_sink_data    (msgdma_data),
		.msgdma_sink_channel (msgdma_channel),
		.uart_txd_txd        (FPGA_UART_TX)
	);*/
endmodule
