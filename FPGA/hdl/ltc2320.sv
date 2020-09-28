(* altera_attribute = {"-name SDC_STATEMENT \"set_false_path -from [get_registers *ltc2320:*|sdo*_sr[*]]\"; -name SDC_STATEMENT \"set_false_path -from [get_registers *ltc2320:*|conv_output]\""} *)
module ltc2320 (
        input  wire        clk_100mhz,
        input  wire        reset_100mhz,
        input  wire        clk_150mhz,
        input  wire        reset_150mhz,
        output wire        adc_sck,
        output reg         adc_cnv_n,
        input  wire [7:0]  adc_sdo,
        input  wire        adc_clkout,
        output wire        ain_valid,
        output wire [12:0] ain1_data,
        output wire [12:0] ain2_data,
        output wire [12:0] ain3_data,
        output wire [12:0] ain4_data,
        output wire [12:0] ain5_data,
        output wire [12:0] ain6_data,
        output wire [12:0] ain7_data,
        output wire [12:0] ain8_data
    );
    
    reg [15:0] sdo1_sr = '0;
    reg [15:0] sdo2_sr = '0;
    reg [15:0] sdo3_sr = '0;
    reg [15:0] sdo4_sr = '0;
    reg [15:0] sdo5_sr = '0;
    reg [15:0] sdo6_sr = '0;
    reg [15:0] sdo7_sr = '0;
    reg [15:0] sdo8_sr = '0;
    
    reg conv_output = 1'b1;
    reg [6:0] conv_counter = '0;
    
    reg [2:0] conv_to_acquire_ff = 3'b111;
    
    reg [5:0] sck_counter = '0;
    wire sck_output_enable = (44 <= sck_counter) && (sck_counter < 60);
    assign adc_sck = sck_output_enable ? clk_100mhz : 1'b0;
    
    reg data_valid = 1'b0;
    assign ain_valid = data_valid;
    assign ain1_data = sdo1_sr[15:3];
    assign ain2_data = sdo2_sr[15:3];
    assign ain3_data = sdo3_sr[15:3];
    assign ain4_data = sdo4_sr[15:3];
    assign ain5_data = sdo5_sr[15:3];
    assign ain6_data = sdo6_sr[15:3];
    assign ain7_data = sdo7_sr[15:3];
    assign ain8_data = sdo8_sr[15:3];
    
    // Latch SDOs
    always @(posedge adc_clkout) begin
        sdo1_sr <= {sdo1_sr[14:0], adc_sdo[0]};
        sdo2_sr <= {sdo2_sr[14:0], adc_sdo[1]};
        sdo3_sr <= {sdo3_sr[14:0], adc_sdo[2]};
        sdo4_sr <= {sdo4_sr[14:0], adc_sdo[3]};
        sdo5_sr <= {sdo5_sr[14:0], adc_sdo[4]};
        sdo6_sr <= {sdo6_sr[14:0], adc_sdo[5]};
        sdo7_sr <= {sdo7_sr[14:0], adc_sdo[6]};
        sdo8_sr <= {sdo8_sr[14:0], adc_sdo[7]};
    end
    
    // Generate CNV
    always @(posedge clk_150mhz, posedge reset_150mhz) begin
        if (reset_150mhz == 1'b1) begin
            adc_cnv_n <= 1'b1;
            conv_output <= 1'b1;
            conv_counter <= '0;
        end
        else begin
            if (95 <= conv_counter) begin
                adc_cnv_n <= 1'b1;
                conv_output <= 1'b1;
            end
            else begin
                adc_cnv_n <= 1'b0;
                conv_output <= 1'b0;
            end
            if (99 <= conv_counter) begin
                conv_counter <= '0;
                data_valid <= 1'b1;
            end
            else begin
                conv_counter <= conv_counter + 1'b1;
                data_valid <= 1'b0;
            end
        end
    end
    
    // Generate SCK
    always @(negedge clk_100mhz, posedge reset_100mhz) begin
        if (reset_100mhz == 1'b1) begin
            conv_to_acquire_ff <= 3'b111;
            sck_counter <= '0;
        end
        else begin
            conv_to_acquire_ff <= {conv_to_acquire_ff[1:0], conv_output};
            if (conv_to_acquire_ff[1] == 1'b1) begin
                sck_counter <= '0;
            end
            else begin
                sck_counter <= sck_counter + 1'b1;
            end
        end
    end
    
    // Generate Data Valid
    //always @(posedge clk_100mhz, posedge reset) begin
    //    if (reset == 1'b1) begin
    //        data_valid <= 1'b0;
    //    end
    //    else begin
    //        data_valid <= (conv_to_acquire_ff[2:1] == 2'b01);
    //    end
    //end
endmodule
