
module controller (
	adc2_i2c_scl_in,
	adc2_i2c_sda_in,
	adc2_i2c_scl_oe,
	adc2_i2c_sda_oe,
	clk_100mhz_clk,
	clk_sys_clk,
	host_spi_mosi_to_the_spislave_inst_for_spichain,
	host_spi_nss_to_the_spislave_inst_for_spichain,
	host_spi_miso_to_and_from_the_spislave_inst_for_spichain,
	host_spi_sclk_to_the_spislave_inst_for_spichain,
	imu_spi_mosi,
	imu_spi_miso,
	imu_spi_sclk,
	imu_spi_cs_n,
	imu_spi_int_n,
	mc5_fault_fault,
	mc5_fault_brake,
	mc5_pwm_data,
	mc5_pwm_valid,
	mc5_pwm_ready,
	mc5_status_driver_otw_n,
	mc5_status_driver_fault_n,
	mc5_status_hall_fault_n,
	pio_0_export,
	pio_1_export,
	pio_2_export,
	reset_100mhz_reset_n,
	reset_sys_reset_n,
	uart_txd,
	vc_encoder_encoder_1_data,
	vc_encoder_encoder_2_data,
	vc_encoder_encoder_3_data,
	vc_encoder_encoder_4_data,
	vc_fault_fault,
	vc_imeas1_data,
	vc_imeas1_valid,
	vc_imeas2_data,
	vc_imeas2_valid,
	vc_imeas3_data,
	vc_imeas3_valid,
	vc_imeas4_data,
	vc_imeas4_valid,
	vc_iref1_data,
	vc_iref1_valid,
	vc_iref2_data,
	vc_iref2_valid,
	vc_iref3_data,
	vc_iref3_valid,
	vc_iref4_data,
	vc_iref4_valid,
	vc_param_kp,
	vc_param_ki,
	vc_status_driver_otw_n,
	vc_status_driver_fault_n,
	vc_status_hall_fault_n,
	vc_status_encoder_fault_n,
	vc_status_pos_error,
	vc_status_pos_uncertain);	

	input		adc2_i2c_scl_in;
	input		adc2_i2c_sda_in;
	output		adc2_i2c_scl_oe;
	output		adc2_i2c_sda_oe;
	input		clk_100mhz_clk;
	input		clk_sys_clk;
	input		host_spi_mosi_to_the_spislave_inst_for_spichain;
	input		host_spi_nss_to_the_spislave_inst_for_spichain;
	inout		host_spi_miso_to_and_from_the_spislave_inst_for_spichain;
	input		host_spi_sclk_to_the_spislave_inst_for_spichain;
	output		imu_spi_mosi;
	input		imu_spi_miso;
	output		imu_spi_sclk;
	output		imu_spi_cs_n;
	input		imu_spi_int_n;
	output		mc5_fault_fault;
	output		mc5_fault_brake;
	output	[15:0]	mc5_pwm_data;
	output		mc5_pwm_valid;
	input		mc5_pwm_ready;
	input		mc5_status_driver_otw_n;
	input		mc5_status_driver_fault_n;
	input		mc5_status_hall_fault_n;
	input		pio_0_export;
	input	[31:0]	pio_1_export;
	output	[9:0]	pio_2_export;
	input		reset_100mhz_reset_n;
	input		reset_sys_reset_n;
	output		uart_txd;
	input	[15:0]	vc_encoder_encoder_1_data;
	input	[15:0]	vc_encoder_encoder_2_data;
	input	[15:0]	vc_encoder_encoder_3_data;
	input	[15:0]	vc_encoder_encoder_4_data;
	output		vc_fault_fault;
	input	[31:0]	vc_imeas1_data;
	input		vc_imeas1_valid;
	input	[31:0]	vc_imeas2_data;
	input		vc_imeas2_valid;
	input	[31:0]	vc_imeas3_data;
	input		vc_imeas3_valid;
	input	[31:0]	vc_imeas4_data;
	input		vc_imeas4_valid;
	output	[31:0]	vc_iref1_data;
	output		vc_iref1_valid;
	output	[31:0]	vc_iref2_data;
	output		vc_iref2_valid;
	output	[31:0]	vc_iref3_data;
	output		vc_iref3_valid;
	output	[31:0]	vc_iref4_data;
	output		vc_iref4_valid;
	output	[15:0]	vc_param_kp;
	output	[15:0]	vc_param_ki;
	input	[3:0]	vc_status_driver_otw_n;
	input	[3:0]	vc_status_driver_fault_n;
	input	[3:0]	vc_status_hall_fault_n;
	input	[3:0]	vc_status_encoder_fault_n;
	input	[3:0]	vc_status_pos_error;
	input	[3:0]	vc_status_pos_uncertain;
endmodule
