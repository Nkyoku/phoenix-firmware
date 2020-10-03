`timescale 1 ps / 1 ps
module test ();
    logic reset = 1'b1;
    logic clk_sys = 1'b0;
    logic clk_100mhz = 1'b0;
    logic clk_150mhz = 1'b0;
    logic adc_sck;
    logic adc_cnv_n;
    logic [7:0] adc_sdo;
    logic adc_clkout;
    
    logic ain_valid;
    logic [12:0] ain1_data;
    logic [12:0] ain2_data;
    logic [12:0] ain3_data;
    logic [12:0] ain4_data;
    logic [12:0] ain5_data;
    logic [12:0] ain6_data;
    logic [12:0] ain7_data;
    logic [12:0] ain8_data;
    
    ltc2320 adc (
        .clk_sys        (clk_sys),
		.reset_sys      (reset),
		.clk_100mhz     (clk_100mhz),
        .reset_100mhz   (reset),
		.clk_150mhz     (clk_150mhz),
		.reset_150mhz   (reset),
        .adc_sck        (adc_sck),
		.adc_cnv_n      (adc_cnv_n),
		.adc_sdo        (adc_sdo),
		.adc_clkout     (adc_clkout),
        .ain_valid_150mhz(),
        .ain_valid      (ain_valid),
        .ain1_data      (ain1_data),
        .ain2_data      (ain2_data),
        .ain3_data      (ain3_data),
        .ain4_data      (ain4_data),
        .ain5_data      (ain5_data),
        .ain6_data      (ain6_data),
        .ain7_data      (ain7_data),
        .ain8_data      (ain8_data)
	);
    
    // ADC Clock
    always @(adc_sck) begin
        adc_clkout <= #3ns adc_sck;
    end
    
    // ADC Data
    always @(adc_cnv_n) begin
        if (adc_cnv_n == 1'b1) begin
            adc_sdo <= 8'bzzzzzzzz;
        end
        else begin
            adc_sdo <= 8'bxxxxxxxx;
            # 450ns adc_sdo <= 8'b11111111;
            @(negedge adc_clkout) adc_sdo <= 8'b00000000;
            @(negedge adc_clkout) adc_sdo <= 8'b11111111;
            @(negedge adc_clkout) adc_sdo <= 8'b00000000;
            @(negedge adc_clkout) adc_sdo <= 8'b00000000;
            @(negedge adc_clkout) adc_sdo <= 8'b11111111;
            @(negedge adc_clkout) adc_sdo <= 8'b11111111;
            @(negedge adc_clkout) adc_sdo <= 8'b00000000;
            @(negedge adc_clkout) adc_sdo <= 8'b11111111;
            @(negedge adc_clkout) adc_sdo <= 8'b11111111;
            @(negedge adc_clkout) adc_sdo <= 8'b00000000;
            @(negedge adc_clkout) adc_sdo <= 8'b00000000;
            @(negedge adc_clkout) adc_sdo <= 8'b11111111;
            @(negedge adc_clkout) adc_sdo <= 8'b11111111;
            @(negedge adc_clkout) adc_sdo <= 8'b11111111;
            @(negedge adc_clkout) adc_sdo <= 8'b00000000;
            @(negedge adc_clkout) adc_sdo <= 8'b11111111;
        end
    end
    
    // clk_sys Generation
    always #6.543ns begin
        clk_sys <= ~clk_sys;
    end
    
    // clk_100mhz Generation
    always #5ns begin
        clk_100mhz <= ~clk_100mhz;
    end
    
    // clk_150mhz Generation
    always #3.333ns begin
        clk_150mhz <= ~clk_150mhz;
    end
    
    // Reset Generaton
    initial begin
        reset <= 1'b1;
        repeat(10) @(posedge clk_100mhz);
        reset <= 1'b0;
        repeat(500) @(posedge clk_100mhz);
        $stop;
    end
endmodule
