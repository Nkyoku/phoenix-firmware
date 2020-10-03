	component spislave is
		port (
			clk_clk       : in    std_logic                     := 'X';             -- clk
			reset_reset_n : in    std_logic                     := 'X';             -- reset_n
			sink_ready    : out   std_logic;                                        -- ready
			sink_valid    : in    std_logic                     := 'X';             -- valid
			sink_data     : in    std_logic_vector(15 downto 0) := (others => 'X'); -- data
			sink_channel  : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- channel
			source_ready  : in    std_logic                     := 'X';             -- ready
			source_valid  : out   std_logic;                                        -- valid
			source_data   : out   std_logic_vector(7 downto 0);                     -- data
			spi_mosi      : in    std_logic                     := 'X';             -- mosi
			spi_nss       : in    std_logic                     := 'X';             -- nss
			spi_miso      : inout std_logic                     := 'X';             -- miso
			spi_sclk      : in    std_logic                     := 'X'              -- sclk
		);
	end component spislave;

	u0 : component spislave
		port map (
			clk_clk       => CONNECTED_TO_clk_clk,       --    clk.clk
			reset_reset_n => CONNECTED_TO_reset_reset_n, --  reset.reset_n
			sink_ready    => CONNECTED_TO_sink_ready,    --   sink.ready
			sink_valid    => CONNECTED_TO_sink_valid,    --       .valid
			sink_data     => CONNECTED_TO_sink_data,     --       .data
			sink_channel  => CONNECTED_TO_sink_channel,  --       .channel
			source_ready  => CONNECTED_TO_source_ready,  -- source.ready
			source_valid  => CONNECTED_TO_source_valid,  --       .valid
			source_data   => CONNECTED_TO_source_data,   --       .data
			spi_mosi      => CONNECTED_TO_spi_mosi,      --    spi.mosi
			spi_nss       => CONNECTED_TO_spi_nss,       --       .nss
			spi_miso      => CONNECTED_TO_spi_miso,      --       .miso
			spi_sclk      => CONNECTED_TO_spi_sclk       --       .sclk
		);

