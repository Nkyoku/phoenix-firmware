`timescale 1 ns / 1 ps
module test ();
    reg reset = 1'b1;
    reg clk100mhz = 1'b0;
    reg clk150mhz = 1'b0;
    
    reg [2:0] sensor_hall_wvu = 3'b111;
    
    reg [1:0] slave_address = '0;
    reg [31:0] slave_writedata = '0;
    reg slave_write = 1'b0;
    
    reg [15:0] current_sink_data;
    reg current_sink_valid;
    reg current_sink_error;
    
    wire [15:0] pwm_sink_data;
    wire pwm_sink_valid;
    wire pwm_sink_error;
    
    wire [15:0] pwm_source_data;
    wire pwm_source_valid;
    wire pwm_source_error;
    wire pwm_source_ready;
    
    wire [2:0] conduit_pwm;
    wire [2:0] conduit_reset_n;
    
    reg ain_valid = 1'b0;
    wire [12:0] ain_data_u = ~conduit_reset_n[0] ? 13'h0000 : (conduit_pwm[0] ? 13'h03E8 : (((conduit_reset_n[1] & conduit_pwm[1]) | (conduit_reset_n[2] & conduit_pwm[2])) ? 13'h1C18 : 13'h0000));
    wire [12:0] ain_data_v = ~conduit_reset_n[1] ? 13'h0000 : (conduit_pwm[1] ? 13'h03E8 : (((conduit_reset_n[0] & conduit_pwm[0]) | (conduit_reset_n[2] & conduit_pwm[2])) ? 13'h1C18 : 13'h0000));
    always begin
        //repeat(99) @(posedge clk150mhz);
        repeat(9) @(posedge clk150mhz);
        ain_valid <= 1'b1;
        @(posedge clk150mhz);
        ain_valid <= 1'b0;
    end
    /* wire adc_sck;
    reg adc_clkout;
    ltc2320 adc (
        .reset(reset),
        .clk_100mhz(clk100mhz),
        .clk_150mhz(clk150mhz),
        .adc_sck(adc_sck),
        .adc_cnv_n(),
        .adc_sdo(8'h00),
        .adc_clkout(adc_clkout),
        .ain_valid(ain_valid),
        .ain1_data(ain_data_u),
        .ain2_data(ain_data_v),
        .ain3_data(),
        .ain4_data(),
        .ain5_data(),
        .ain6_data(),
        .ain7_data(),
        .ain8_data()
    );
    always @(adc_sck) begin
        adc_clkout <= #3ns adc_sck;
    end*/
    
    /*wire ain_cic_u_valid;
    wire ain_cic_v_valid;
    wire [15:0] ain_cic_u_data;
    wire [15:0] ain_cic_v_data;
    wire clk_pwm = ain_cic_u_valid;
    MyCIC #(
        .NUM_STAGES(4),
        .RATE(30),
        .RATE_WIDTH(5),
        .SCALE_WIDTH(20),
        .IN_WIDTH(13),
        .OUT_WIDTH(16)
    ) cic1 (
        .clk(clk150mhz),
        .reset(reset),
        .in_valid(ain_valid),
        .in_data(ain_data_u),
        .out_valid(ain_cic_u_valid),
        .out_data(ain_cic_u_data)
    );
    MyCIC #(
        .NUM_STAGES(4),
        .RATE(30),
        .RATE_WIDTH(5),
        .SCALE_WIDTH(20),
        .IN_WIDTH(13),
        .OUT_WIDTH(16)
    ) cic2 (
        .clk(clk150mhz),
        .reset(reset),
        .in_valid(ain_valid),
        .in_data(ain_data_v),
        .out_valid(ain_cic_v_valid),
        .out_data(ain_cic_v_data)
    );*/
    wire ain_cic_sink_valid;
    wire [12:0] ain_cic_sink_data;
    wire current_source_valid;
    wire [15:0] current_source_data;
    wire clk_pwm = current_source_valid;
    MyCIC #(
        .NUM_STAGES(4),
        .RATE(30),
        .RATE_WIDTH(5),
        .SCALE_WIDTH(20),
        .IN_WIDTH(13),
        .OUT_WIDTH(16)
    ) cic (
        .clk(clk150mhz),
        .reset(reset),
        .in_valid(ain_cic_sink_valid),
        .in_data(ain_cic_sink_data),
        .out_valid(current_source_valid),
        .out_data(current_source_data)
    );
        
    pwm_driver #(
        .PWM_MAX_ON_CYCLES(2985),
        .CS_CONFIGURATION(3)
    ) drv (
        .reset(reset),
        .clk(clk150mhz),
        .clk_pwm(clk_pwm),
        .sensor_hall_wvu(sensor_hall_wvu),
        .current_sink_u_data(ain_data_u),
        .current_sink_u_valid(ain_valid),
        .current_sink_v_data(ain_data_v),
        .current_sink_v_valid(ain_valid),
        .current_sink_w_data(16'h0000),
        .current_sink_w_valid(1'b0),
        .current_source_data(ain_cic_sink_data),
        .current_source_valid(ain_cic_sink_valid),
        .pwm_sink_data(pwm_sink_data),
        .pwm_sink_valid(pwm_sink_valid),
        .pwm_sink_error(pwm_sink_error),
        .conduit_otw_n   (1'b1),
		.conduit_fault_n (1'b1),
		.conduit_pwm     (conduit_pwm),
		.conduit_reset_n (conduit_reset_n),
		.status_otw      (),
		.status_fault    (),
        .status_hall_fault()
    );
    
    avalon_st_clock_bridge #(
        .DATA_WIDTH(16)
    ) pwm_bridge (
        .reset(reset),
        .clk1(clk100mhz),
        .sink_data(pwm_source_data),
        .sink_valid(pwm_source_valid),
        .sink_error(pwm_source_error),
        .sink_ready(pwm_source_ready),
        .clk2(clk150mhz),
        .source_data(pwm_sink_data),
        .source_valid(pwm_sink_valid),
        .source_error(pwm_sink_error)
    );
    
    avalon_st_clock_bridge #(
        .DATA_WIDTH(16)
    ) adc_bridge (
        .reset(reset),
        .clk1(clk150mhz),
        .sink_data(current_source_data),
        .sink_valid(current_source_valid),
        .sink_error(1'b0),
        .sink_ready(),
        .clk2(clk100mhz),
        .source_data(current_sink_data),
        .source_valid(current_sink_valid),
        .source_error(current_sink_error)
    );
    
    motor_controller dut (
        .reset(reset),
        .clk(clk100mhz),
        .slave_address(slave_address),
        .slave_readdata(),
        .slave_writedata(slave_writedata),
        .slave_read(1'b0),
        .slave_write(slave_write),
        .current_sink_data(current_sink_data),
        .current_sink_valid(current_sink_valid),
        .current_sink_error(current_sink_error),
        .pwm_source_data(pwm_source_data),
        .pwm_source_valid(pwm_source_valid),
        .pwm_source_error(pwm_source_error),
        .pwm_source_ready(pwm_source_ready)
    );
    
    task write_data(input [1:0] address, input [31:0] data);
        slave_address <= address;
        slave_writedata <= data;
        slave_write <= 1'b1;
        @(posedge clk100mhz);
        slave_write <= 1'b0;
    endtask
    
    initial begin
        @(negedge reset);
        repeat(5) @(posedge clk100mhz);
        repeat(10) @(posedge clk100mhz);
        write_data(2'b01, 32'h00000001);
        repeat(2) @(posedge clk100mhz);
        write_data(2'b10, 32'h00000400);
        repeat(20) @(posedge clk100mhz);
        write_data(2'b10, 32'h0000F800);
    end
    
    // Hall generation
    always begin
        # 7us
        sensor_hall_wvu <= 3'b001;
        # 7us
        sensor_hall_wvu <= 3'b011;
        # 7us
        sensor_hall_wvu <= 3'b010;
        # 7us
        sensor_hall_wvu <= 3'b110;
        # 7us
        sensor_hall_wvu <= 3'b100;
        # 7us
        sensor_hall_wvu <= 3'b101;
    end
    
    // Clock 1 100MHz generation
    always #5ns begin
        clk100mhz <= ~clk100mhz;
    end
    
    // Clock 2 150MHz generation
    always #3.3ns begin
        clk150mhz <= ~clk150mhz;
    end
    
    // Reset generaton
    initial begin
        reset <= 1'b1;
        repeat(5) @(negedge clk100mhz);
        reset <= 1'b0;
        repeat(4050) @(negedge clk100mhz);
        $stop;
    end
endmodule
