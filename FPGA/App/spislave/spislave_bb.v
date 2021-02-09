
module spislave (
	clk_clk,
	reset_reset_n,
	sink_ready,
	sink_valid,
	sink_data,
	sink_channel,
	source_ready,
	source_valid,
	source_data,
	spi_mosi,
	spi_nss,
	spi_miso,
	spi_sclk);	

	input		clk_clk;
	input		reset_reset_n;
	output		sink_ready;
	input		sink_valid;
	input	[15:0]	sink_data;
	input	[7:0]	sink_channel;
	input		source_ready;
	output		source_valid;
	output	[7:0]	source_data;
	input		spi_mosi;
	input		spi_nss;
	inout		spi_miso;
	input		spi_sclk;
endmodule
