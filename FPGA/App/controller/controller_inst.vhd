	component controller is
		port (
			adc2_i2c_scl_in                                          : in    std_logic                     := 'X';             -- scl_in
			adc2_i2c_sda_in                                          : in    std_logic                     := 'X';             -- sda_in
			adc2_i2c_scl_oe                                          : out   std_logic;                                        -- scl_oe
			adc2_i2c_sda_oe                                          : out   std_logic;                                        -- sda_oe
			clk_100mhz_clk                                           : in    std_logic                     := 'X';             -- clk
			clk_sys_clk                                              : in    std_logic                     := 'X';             -- clk
			host_spi_mosi_to_the_spislave_inst_for_spichain          : in    std_logic                     := 'X';             -- mosi_to_the_spislave_inst_for_spichain
			host_spi_nss_to_the_spislave_inst_for_spichain           : in    std_logic                     := 'X';             -- nss_to_the_spislave_inst_for_spichain
			host_spi_miso_to_and_from_the_spislave_inst_for_spichain : inout std_logic                     := 'X';             -- miso_to_and_from_the_spislave_inst_for_spichain
			host_spi_sclk_to_the_spislave_inst_for_spichain          : in    std_logic                     := 'X';             -- sclk_to_the_spislave_inst_for_spichain
			imu_spi_mosi                                             : out   std_logic;                                        -- mosi
			imu_spi_miso                                             : in    std_logic                     := 'X';             -- miso
			imu_spi_sclk                                             : out   std_logic;                                        -- sclk
			imu_spi_cs_n                                             : out   std_logic;                                        -- cs_n
			imu_spi_int_n                                            : in    std_logic                     := 'X';             -- int_n
			mc5_fault_fault                                          : out   std_logic;                                        -- fault
			mc5_pwm_data                                             : out   std_logic_vector(15 downto 0);                    -- data
			mc5_pwm_valid                                            : out   std_logic;                                        -- valid
			mc5_pwm_ready                                            : in    std_logic                     := 'X';             -- ready
			mc5_status_driver_otw_n                                  : in    std_logic                     := 'X';             -- driver_otw_n
			mc5_status_driver_fault_n                                : in    std_logic                     := 'X';             -- driver_fault_n
			mc5_status_hall_fault_n                                  : in    std_logic                     := 'X';             -- hall_fault_n
			pio3_export                                              : in    std_logic_vector(31 downto 0) := (others => 'X'); -- export
			pio4_export                                              : in    std_logic_vector(31 downto 0) := (others => 'X'); -- export
			pio_0_export                                             : in    std_logic                     := 'X';             -- export
			pio_1_export                                             : in    std_logic_vector(31 downto 0) := (others => 'X'); -- export
			pio_2_export                                             : out   std_logic_vector(9 downto 0);                     -- export
			reset_100mhz_reset_n                                     : in    std_logic                     := 'X';             -- reset_n
			reset_sys_reset_n                                        : in    std_logic                     := 'X';             -- reset_n
			uart_txd                                                 : out   std_logic;                                        -- txd
			vc_encoder_encoder_1_data                                : in    std_logic_vector(15 downto 0) := (others => 'X'); -- encoder_1_data
			vc_encoder_encoder_2_data                                : in    std_logic_vector(15 downto 0) := (others => 'X'); -- encoder_2_data
			vc_encoder_encoder_3_data                                : in    std_logic_vector(15 downto 0) := (others => 'X'); -- encoder_3_data
			vc_encoder_encoder_4_data                                : in    std_logic_vector(15 downto 0) := (others => 'X'); -- encoder_4_data
			vc_fault_fault                                           : out   std_logic;                                        -- fault
			vc_imeas1_data                                           : in    std_logic_vector(31 downto 0) := (others => 'X'); -- data
			vc_imeas1_valid                                          : in    std_logic                     := 'X';             -- valid
			vc_imeas2_data                                           : in    std_logic_vector(31 downto 0) := (others => 'X'); -- data
			vc_imeas2_valid                                          : in    std_logic                     := 'X';             -- valid
			vc_imeas3_data                                           : in    std_logic_vector(31 downto 0) := (others => 'X'); -- data
			vc_imeas3_valid                                          : in    std_logic                     := 'X';             -- valid
			vc_imeas4_data                                           : in    std_logic_vector(31 downto 0) := (others => 'X'); -- data
			vc_imeas4_valid                                          : in    std_logic                     := 'X';             -- valid
			vc_iref1_data                                            : out   std_logic_vector(31 downto 0);                    -- data
			vc_iref1_valid                                           : out   std_logic;                                        -- valid
			vc_iref2_data                                            : out   std_logic_vector(31 downto 0);                    -- data
			vc_iref2_valid                                           : out   std_logic;                                        -- valid
			vc_iref3_data                                            : out   std_logic_vector(31 downto 0);                    -- data
			vc_iref3_valid                                           : out   std_logic;                                        -- valid
			vc_iref4_data                                            : out   std_logic_vector(31 downto 0);                    -- data
			vc_iref4_valid                                           : out   std_logic;                                        -- valid
			vc_param_kp                                              : out   std_logic_vector(15 downto 0);                    -- kp
			vc_param_ki                                              : out   std_logic_vector(15 downto 0);                    -- ki
			vc_status_driver_otw_n                                   : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- driver_otw_n
			vc_status_driver_fault_n                                 : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- driver_fault_n
			vc_status_hall_fault_n                                   : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- hall_fault_n
			vc_status_encoder_fault_n                                : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- encoder_fault_n
			vc_status_pos_error                                      : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- pos_error
			vc_status_pos_uncertain                                  : in    std_logic_vector(3 downto 0)  := (others => 'X')  -- pos_uncertain
		);
	end component controller;

	u0 : component controller
		port map (
			adc2_i2c_scl_in                                          => CONNECTED_TO_adc2_i2c_scl_in,                                          --     adc2_i2c.scl_in
			adc2_i2c_sda_in                                          => CONNECTED_TO_adc2_i2c_sda_in,                                          --             .sda_in
			adc2_i2c_scl_oe                                          => CONNECTED_TO_adc2_i2c_scl_oe,                                          --             .scl_oe
			adc2_i2c_sda_oe                                          => CONNECTED_TO_adc2_i2c_sda_oe,                                          --             .sda_oe
			clk_100mhz_clk                                           => CONNECTED_TO_clk_100mhz_clk,                                           --   clk_100mhz.clk
			clk_sys_clk                                              => CONNECTED_TO_clk_sys_clk,                                              --      clk_sys.clk
			host_spi_mosi_to_the_spislave_inst_for_spichain          => CONNECTED_TO_host_spi_mosi_to_the_spislave_inst_for_spichain,          --     host_spi.mosi_to_the_spislave_inst_for_spichain
			host_spi_nss_to_the_spislave_inst_for_spichain           => CONNECTED_TO_host_spi_nss_to_the_spislave_inst_for_spichain,           --             .nss_to_the_spislave_inst_for_spichain
			host_spi_miso_to_and_from_the_spislave_inst_for_spichain => CONNECTED_TO_host_spi_miso_to_and_from_the_spislave_inst_for_spichain, --             .miso_to_and_from_the_spislave_inst_for_spichain
			host_spi_sclk_to_the_spislave_inst_for_spichain          => CONNECTED_TO_host_spi_sclk_to_the_spislave_inst_for_spichain,          --             .sclk_to_the_spislave_inst_for_spichain
			imu_spi_mosi                                             => CONNECTED_TO_imu_spi_mosi,                                             --      imu_spi.mosi
			imu_spi_miso                                             => CONNECTED_TO_imu_spi_miso,                                             --             .miso
			imu_spi_sclk                                             => CONNECTED_TO_imu_spi_sclk,                                             --             .sclk
			imu_spi_cs_n                                             => CONNECTED_TO_imu_spi_cs_n,                                             --             .cs_n
			imu_spi_int_n                                            => CONNECTED_TO_imu_spi_int_n,                                            --             .int_n
			mc5_fault_fault                                          => CONNECTED_TO_mc5_fault_fault,                                          --    mc5_fault.fault
			mc5_pwm_data                                             => CONNECTED_TO_mc5_pwm_data,                                             --      mc5_pwm.data
			mc5_pwm_valid                                            => CONNECTED_TO_mc5_pwm_valid,                                            --             .valid
			mc5_pwm_ready                                            => CONNECTED_TO_mc5_pwm_ready,                                            --             .ready
			mc5_status_driver_otw_n                                  => CONNECTED_TO_mc5_status_driver_otw_n,                                  --   mc5_status.driver_otw_n
			mc5_status_driver_fault_n                                => CONNECTED_TO_mc5_status_driver_fault_n,                                --             .driver_fault_n
			mc5_status_hall_fault_n                                  => CONNECTED_TO_mc5_status_hall_fault_n,                                  --             .hall_fault_n
			pio3_export                                              => CONNECTED_TO_pio3_export,                                              --         pio3.export
			pio4_export                                              => CONNECTED_TO_pio4_export,                                              --         pio4.export
			pio_0_export                                             => CONNECTED_TO_pio_0_export,                                             --        pio_0.export
			pio_1_export                                             => CONNECTED_TO_pio_1_export,                                             --        pio_1.export
			pio_2_export                                             => CONNECTED_TO_pio_2_export,                                             --        pio_2.export
			reset_100mhz_reset_n                                     => CONNECTED_TO_reset_100mhz_reset_n,                                     -- reset_100mhz.reset_n
			reset_sys_reset_n                                        => CONNECTED_TO_reset_sys_reset_n,                                        --    reset_sys.reset_n
			uart_txd                                                 => CONNECTED_TO_uart_txd,                                                 --         uart.txd
			vc_encoder_encoder_1_data                                => CONNECTED_TO_vc_encoder_encoder_1_data,                                --   vc_encoder.encoder_1_data
			vc_encoder_encoder_2_data                                => CONNECTED_TO_vc_encoder_encoder_2_data,                                --             .encoder_2_data
			vc_encoder_encoder_3_data                                => CONNECTED_TO_vc_encoder_encoder_3_data,                                --             .encoder_3_data
			vc_encoder_encoder_4_data                                => CONNECTED_TO_vc_encoder_encoder_4_data,                                --             .encoder_4_data
			vc_fault_fault                                           => CONNECTED_TO_vc_fault_fault,                                           --     vc_fault.fault
			vc_imeas1_data                                           => CONNECTED_TO_vc_imeas1_data,                                           --    vc_imeas1.data
			vc_imeas1_valid                                          => CONNECTED_TO_vc_imeas1_valid,                                          --             .valid
			vc_imeas2_data                                           => CONNECTED_TO_vc_imeas2_data,                                           --    vc_imeas2.data
			vc_imeas2_valid                                          => CONNECTED_TO_vc_imeas2_valid,                                          --             .valid
			vc_imeas3_data                                           => CONNECTED_TO_vc_imeas3_data,                                           --    vc_imeas3.data
			vc_imeas3_valid                                          => CONNECTED_TO_vc_imeas3_valid,                                          --             .valid
			vc_imeas4_data                                           => CONNECTED_TO_vc_imeas4_data,                                           --    vc_imeas4.data
			vc_imeas4_valid                                          => CONNECTED_TO_vc_imeas4_valid,                                          --             .valid
			vc_iref1_data                                            => CONNECTED_TO_vc_iref1_data,                                            --     vc_iref1.data
			vc_iref1_valid                                           => CONNECTED_TO_vc_iref1_valid,                                           --             .valid
			vc_iref2_data                                            => CONNECTED_TO_vc_iref2_data,                                            --     vc_iref2.data
			vc_iref2_valid                                           => CONNECTED_TO_vc_iref2_valid,                                           --             .valid
			vc_iref3_data                                            => CONNECTED_TO_vc_iref3_data,                                            --     vc_iref3.data
			vc_iref3_valid                                           => CONNECTED_TO_vc_iref3_valid,                                           --             .valid
			vc_iref4_data                                            => CONNECTED_TO_vc_iref4_data,                                            --     vc_iref4.data
			vc_iref4_valid                                           => CONNECTED_TO_vc_iref4_valid,                                           --             .valid
			vc_param_kp                                              => CONNECTED_TO_vc_param_kp,                                              --     vc_param.kp
			vc_param_ki                                              => CONNECTED_TO_vc_param_ki,                                              --             .ki
			vc_status_driver_otw_n                                   => CONNECTED_TO_vc_status_driver_otw_n,                                   --    vc_status.driver_otw_n
			vc_status_driver_fault_n                                 => CONNECTED_TO_vc_status_driver_fault_n,                                 --             .driver_fault_n
			vc_status_hall_fault_n                                   => CONNECTED_TO_vc_status_hall_fault_n,                                   --             .hall_fault_n
			vc_status_encoder_fault_n                                => CONNECTED_TO_vc_status_encoder_fault_n,                                --             .encoder_fault_n
			vc_status_pos_error                                      => CONNECTED_TO_vc_status_pos_error,                                      --             .pos_error
			vc_status_pos_uncertain                                  => CONNECTED_TO_vc_status_pos_uncertain                                   --             .pos_uncertain
		);

