/**
 * @file top.sv
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

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
        input  wire FPGA_MODE,
        output wire FPGA_SPI_MISO,
        input  wire FPGA_SPI_MOSI,
        input  wire FPGA_SPI_SCLK,
        input  wire FPGA_SPI_CS0_N,
        input  wire FPGA_UART_RX,
        output wire FPGA_UART_TX,
        input  wire FPGA_STOP_N,
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
    
    assign ADC1_SCK = 1'b0;
    assign ADC1_CNV_N = 1'b1;
    assign ADC2_SCL = 1'bz;
    assign ADC2_SDA = 1'bz;
    assign FPGA_UART_TX = 1'bz;
    assign HALL1_CLK = 1'b0;
    assign HALL1_LOAD_N = 1'b1;
    assign HALL2_CLK = 1'b0;
    assign HALL2_LOAD_N = 1'b1;
    assign IMU_CLKIN = 1'b0;
    assign IMU_MOSI = 1'b0;
    assign IMU_SCLK = 1'b0;
    assign IMU_CS_N = 1'b1;
    assign MOTOR1_PWM_A = 1'b0;
    assign MOTOR1_PWM_B = 1'b0;
    assign MOTOR1_PWM_C = 1'b0;
    assign MOTOR1_SW_EN = 1'b0;
    assign MOTOR1_RESET_A_N = 1'b0;
    assign MOTOR1_RESET_B_N = 1'b0;
    assign MOTOR1_RESET_C_N = 1'b0;
    assign MOTOR2_PWM_A = 1'b0;
    assign MOTOR2_PWM_B = 1'b0;
    assign MOTOR2_PWM_C = 1'b0;
    assign MOTOR2_SW_EN = 1'b0;
    assign MOTOR2_RESET_A_N = 1'b0;
    assign MOTOR2_RESET_B_N = 1'b0;
    assign MOTOR2_RESET_C_N = 1'b0;
    assign MOTOR3_PWM_A = 1'b0;
    assign MOTOR3_PWM_B = 1'b0;
    assign MOTOR3_PWM_C = 1'b0;
    assign MOTOR3_SW_EN = 1'b0;
    assign MOTOR3_RESET_A_N = 1'b0;
    assign MOTOR3_RESET_B_N = 1'b0;
    assign MOTOR3_RESET_C_N = 1'b0;
    assign MOTOR4_PWM_A = 1'b0;
    assign MOTOR4_PWM_B = 1'b0;
    assign MOTOR4_PWM_C = 1'b0;
    assign MOTOR4_SW_EN = 1'b0;
    assign MOTOR4_RESET_A_N = 1'b0;
    assign MOTOR4_RESET_B_N = 1'b0;
    assign MOTOR4_RESET_C_N = 1'b0;
    assign MOTOR5_PWM_A = 1'b0;
    assign MOTOR5_PWM_B = 1'b0;
    assign MOTOR5_PWM_C = 1'b0;
    assign MOTOR5_SW_EN = 1'b0;
    assign MOTOR5_RESET_A_N = 1'b0;
    assign MOTOR5_RESET_B_N = 1'b0;
    assign MOTOR5_RESET_C_N = 1'b0;
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

    // Bypass SPI signals
    logic bypass_spi = 1'b0;
    assign nCSO = bypass_spi ? FPGA_SPI_CS0_N : 1'b1;
    assign DCLK = bypass_spi ? FPGA_SPI_SCLK : 1'b0;
    assign ASDO = bypass_spi ? FPGA_SPI_MOSI : 1'bz;
    assign FPGA_SPI_MISO = (bypass_spi & ~FPGA_SPI_CS0_N) ? ASDI : 1'bz;

    // Remote Update IP
    logic rsu_reset = 1'b1;
    logic rsu_busy;
    logic rsu_reconfig = 1'b0;
    logic [2:0] rsu_param = '0;
    logic rsu_read_param = 1'b0;
    logic rsu_write_param = 1'b0;
    logic [28:0] rsu_data_out;
    logic [23:0] rsu_data_in = '0;
    rsu rsu_0 (
        .reset       (rsu_reset),
        .clock       (CLK25MHz),
		.busy        (rsu_busy),
		.reset_timer (1'b0),
		.reconfig    (rsu_reconfig),
        .param       (rsu_param),
        .read_param  (rsu_read_param),
        .write_param (rsu_write_param),
		.data_out    (rsu_data_out),
        .data_in     (rsu_data_in),
		.read_source (2'b00)
	);

    // State Machine
    enum {
        STATE_RESET,
        STATE_WAIT,
        STATE_READ_TRIGGER,
        STATE_FETCH_TRIGGER,
        STATE_WAIT_DEASSERT,
        STATE_WRITE_ADDRESS,
        STATE_DISABLE_WDT,
        STATE_RECONFIG,
        STATE_ERROR
    } state = STATE_RESET;
    logic [4:0] trigger = '0;
    always @(posedge CLK25MHz) begin
        rsu_read_param <= 1'b0;
        rsu_write_param <= 1'b0;
        if (~(rsu_busy | rsu_read_param | rsu_write_param) | rsu_reset) begin
            case (state)
            STATE_RESET: begin
                rsu_reset <= 1'b0;
                state <= STATE_WAIT;
            end
            STATE_WAIT : begin
                state <= STATE_READ_TRIGGER;
            end
            STATE_READ_TRIGGER : begin
                rsu_param <= 3'b111;
                rsu_read_param <= 1'b1;
                state <= STATE_FETCH_TRIGGER;
            end
            STATE_FETCH_TRIGGER : begin
                trigger <= rsu_data_out[4:0];
                if ((rsu_data_out[4:0] & 5'b01011) != '0) begin
                    state <= STATE_ERROR;
                end
                else begin
                    state <= STATE_WAIT_DEASSERT;
                end
            end
            STATE_WAIT_DEASSERT : begin
                if (FPGA_MODE == 1'b0) begin
                    bypass_spi <= 1'b1;
                    state <= STATE_WAIT_DEASSERT;
                end
                else begin
                    bypass_spi <= 1'b0;
                    state <= STATE_WRITE_ADDRESS;
                end
            end
            STATE_WRITE_ADDRESS : begin
                rsu_param <= 3'b100;
                rsu_data_in <= 24'h100000;
                rsu_write_param <= 1'b1;
                state <= STATE_DISABLE_WDT;
            end
            STATE_DISABLE_WDT : begin
                rsu_param <= 3'b011;
                rsu_data_in <= 24'h000000;
                rsu_write_param <= 1'b1;
                state <= STATE_RECONFIG;
            end
            STATE_RECONFIG : begin
                rsu_reconfig <= 1'b1;
                state <= STATE_ERROR;
            end
            default : begin
                state <= STATE_ERROR;
            end
            endcase
        end
    end

    // Blink LEDs
    logic [21:0] blink_counter = '0;
    logic [4:0] blink_shift_register = 5'b10000;
    logic blink_polarity;
    assign blink_polarity = trigger[3] | trigger[1] | trigger[0] | rsu_reconfig;
    assign MOTOR1_LED = blink_shift_register[0] ^ blink_polarity;
    assign MOTOR2_LED = blink_shift_register[1] ^ blink_polarity;
    assign MOTOR3_LED = blink_shift_register[2] ^ blink_polarity;
    assign MOTOR4_LED = blink_shift_register[3] ^ blink_polarity;
    assign MOTOR5_LED = blink_shift_register[4] ^ blink_polarity;
    always @(posedge CLK25MHz) begin
        if (2499999 <= blink_counter) begin
            blink_counter <= '0;
            blink_shift_register <= {blink_shift_register[3:0], blink_shift_register[4]};
        end
        else begin
            blink_counter <= blink_counter + 1'b1;
        end
    end
endmodule
