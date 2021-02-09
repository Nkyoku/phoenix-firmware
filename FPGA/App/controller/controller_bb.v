
module controller (
	adc2_i2c_sda_in,
	adc2_i2c_scl_in,
	adc2_i2c_sda_oe,
	adc2_i2c_scl_oe,
	clk_clk,
	imu_spi_mosi,
	imu_spi_miso,
	imu_spi_sclk,
	imu_spi_cs_n,
	imu_spi_int_n,
	mc_fault_fault,
	mc_pwm_data,
	mc_pwm_valid,
	mc_pwm_ready,
	mc_status_driver_otw_n,
	mc_status_driver_fault_n,
	mc_status_hall_fault_n,
	msgdma_source_data,
	msgdma_source_valid,
	msgdma_source_ready,
	msgdma_source_channel,
	pio_0_export,
	pio_1_export,
	pio_2_export,
	reset_reset_n,
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
	vc_status_pos_uncertain,
	uart_rxd,
	uart_txd);	

	input		adc2_i2c_sda_in;
	input		adc2_i2c_scl_in;
	output		adc2_i2c_sda_oe;
	output		adc2_i2c_scl_oe;
	input		clk_clk;
	output		imu_spi_mosi;
	input		imu_spi_miso;
	output		imu_spi_sclk;
	output		imu_spi_cs_n;
	input		imu_spi_int_n;
	output		mc_fault_fault;
	output	[15:0]	mc_pwm_data;
	output		mc_pwm_valid;
	input		mc_pwm_ready;
	input		mc_status_driver_otw_n;
	input		mc_status_driver_fault_n;
	input		mc_status_hall_fault_n;
	output	[15:0]	msgdma_source_data;
	output		msgdma_source_valid;
	input		msgdma_source_ready;
	output	[7:0]	msgdma_source_channel;
	input		pio_0_export;
	input	[31:0]	pio_1_export;
	output	[10:0]	pio_2_export;
	input		reset_reset_n;
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
	input		uart_rxd;
	output		uart_txd;
endmodule
