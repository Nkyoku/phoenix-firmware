`timescale 1 ps / 1 ps
module test ();
    reg reset = 1'b1;
    reg clk_100mhz = 1'b0;
    reg clk_150mhz = 1'b0;
    wire adc_sck;
    wire adc_cnv_n;
    reg [7:0] adc_sdo;
    reg adc_clkout;
    
    wire ain_valid;
    wire [12:0] ain1_data;
    wire [12:0] ain2_data;
    wire [12:0] ain3_data;
    wire [12:0] ain4_data;
    wire [12:0] ain5_data;
    wire [12:0] ain6_data;
    wire [12:0] ain7_data;
    wire [12:0] ain8_data;
    
    ltc2320 adc (
		.reset_100mhz   (reset),
        .reset_150mhz   (reset),
		.clk_100mhz     (clk_100mhz),
		.clk_150mhz     (clk_150mhz),
		.adc_sck        (adc_sck),
		.adc_cnv_n      (adc_cnv_n),
		.adc_sdo        (adc_sdo),
		.adc_clkout     (adc_clkout),
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
            @(negedge adc_clkout) adc_sdo <= 8'b11111111;
            @(negedge adc_clkout) adc_sdo <= 8'b00000000;
            @(negedge adc_clkout) adc_sdo <= 8'b11111111;
            @(negedge adc_clkout) adc_sdo <= 8'b00000000;
            @(negedge adc_clkout) adc_sdo <= 8'b11111111;
            @(negedge adc_clkout) adc_sdo <= 8'b00000000;
            @(negedge adc_clkout) adc_sdo <= 8'b11111111;
            @(negedge adc_clkout) adc_sdo <= 8'b00000000;
            @(negedge adc_clkout) adc_sdo <= 8'b11111111;
            @(negedge adc_clkout) adc_sdo <= 8'b00000000;
            @(negedge adc_clkout) adc_sdo <= 8'b11111111;
            @(negedge adc_clkout) adc_sdo <= 8'b00000000;
            @(negedge adc_clkout) adc_sdo <= 8'b11111111;
        end
    end
    
    // CLOCK 100MHz Generation
    always #5ns begin
        clk_100mhz <= ~clk_100mhz;
    end
    
    // CLOCK 150MHz Generation
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
