	spislave u0 (
		.clk_clk       (<connected-to-clk_clk>),       //    clk.clk
		.reset_reset_n (<connected-to-reset_reset_n>), //  reset.reset_n
		.sink_ready    (<connected-to-sink_ready>),    //   sink.ready
		.sink_valid    (<connected-to-sink_valid>),    //       .valid
		.sink_data     (<connected-to-sink_data>),     //       .data
		.sink_channel  (<connected-to-sink_channel>),  //       .channel
		.source_ready  (<connected-to-source_ready>),  // source.ready
		.source_valid  (<connected-to-source_valid>),  //       .valid
		.source_data   (<connected-to-source_data>),   //       .data
		.spi_mosi      (<connected-to-spi_mosi>),      //    spi.mosi
		.spi_nss       (<connected-to-spi_nss>),       //       .nss
		.spi_miso      (<connected-to-spi_miso>),      //       .miso
		.spi_sclk      (<connected-to-spi_sclk>)       //       .sclk
	);

