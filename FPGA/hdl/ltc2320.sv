(* altera_attribute = {"-name SDC_STATEMENT \"set_false_path -from [get_registers *ltc2320:*|sdo*_sr[*]] -to [get_registers *ltc2320:*|ain*_data[*]]\"; -name SDC_STATEMENT \"set_false_path -from [get_registers *ltc2320:*|conv_output_ff[0]] -to [get_registers *ltc2320:*|conv_to_acquire_ff[0]]\"; -name SDC_STATEMENT \"set_false_path -from [get_registers *ltc2320:*|conv_output_ff[0]] -to [get_registers *ltc2320:*|conv_to_valid_ff[0]]\""} *)
module ltc2320 (
        input  wire        clk_sys,
        input  wire        reset_sys,
        input  wire        clk_100mhz,
        input  wire        reset_100mhz,
        input  wire        clk_150mhz,
        input  wire        reset_150mhz,
        output reg         adc_sck,
        output reg         adc_cnv_n,
        input  wire [7:0]  adc_sdo,
        input  wire        adc_clkout,
        output reg         ain_valid,
        output wire        ain_valid_150mhz,
        output reg  [12:0] ain1_data,
        output reg  [12:0] ain2_data,
        output reg  [12:0] ain3_data,
        output reg  [12:0] ain4_data,
        output reg  [12:0] ain5_data,
        output reg  [12:0] ain6_data,
        output reg  [12:0] ain7_data,
        output reg  [12:0] ain8_data
    );
    
    // Latch SDOs
    logic [15:0] sdo1_sr = '0;
    logic [15:0] sdo2_sr = '0;
    logic [15:0] sdo3_sr = '0;
    logic [15:0] sdo4_sr = '0;
    logic [15:0] sdo5_sr = '0;
    logic [15:0] sdo6_sr = '0;
    logic [15:0] sdo7_sr = '0;
    logic [15:0] sdo8_sr = '0;
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
    
    // Generate CNV_N
    logic [1:0] conv_output_ff = 2'b11;
    logic [6:0] conv_counter = '0;
    assign ain_valid_150mhz = ~conv_output_ff[1] & conv_output_ff[0];
    always @(posedge clk_150mhz, posedge reset_150mhz) begin
        if (reset_150mhz == 1'b1) begin
            adc_cnv_n <= 1'b1;
            conv_output_ff <= 2'b11;
            conv_counter <= '0;
        end
        else begin
            conv_output_ff[1] <= conv_output_ff[0];
            if (95 <= conv_counter) begin
                adc_cnv_n <= 1'b1;
                conv_output_ff[0] <= 1'b1;
            end
            else begin
                adc_cnv_n <= 1'b0;
                conv_output_ff[0] <= 1'b0;
            end
            if (99 <= conv_counter) begin
                conv_counter <= '0;
            end
            else begin
                conv_counter <= conv_counter + 1'b1;
            end
        end
    end
    
    // Generate SCK
    logic [5:0] sck_counter = '0;
    wire sck_output_enable = (44 <= sck_counter) && (sck_counter < 60);
    assign adc_sck = sck_output_enable ? ~clk_100mhz : 1'b0;
    logic [1:0] conv_to_acquire_ff = '1;
    always @(posedge clk_100mhz, posedge reset_100mhz) begin
        if (reset_100mhz == 1'b1) begin
            sck_counter <= '0;
            conv_to_acquire_ff <= '1;
        end
        else begin
            conv_to_acquire_ff <= {conv_to_acquire_ff[0], conv_output_ff[0]};
            if (conv_to_acquire_ff[1] == 1'b1) begin
                sck_counter <= '0;
            end
            else begin
                sck_counter <= sck_counter + 1'b1;
            end
        end
    end
    
    // Generate Data Valid
    logic [2:0] conv_to_valid_ff = '1;
    //assign ain_valid = ~conv_to_valid_ff[2] & conv_to_valid_ff[1];
    //assign ain1_data = sdo1_sr[15:3];
    //assign ain2_data = sdo2_sr[15:3];
    //assign ain3_data = sdo3_sr[15:3];
    //assign ain4_data = sdo4_sr[15:3];
    //assign ain5_data = sdo5_sr[15:3];
    //assign ain6_data = sdo6_sr[15:3];
    //assign ain7_data = sdo7_sr[15:3];
    //assign ain8_data = sdo8_sr[15:3];
    always @(posedge clk_sys, posedge reset_sys) begin
        if (reset_sys == 1'b1) begin
            conv_to_valid_ff <= '1;
            ain_valid <= 1'b0;
        end
        else begin
            conv_to_valid_ff <= {conv_to_valid_ff[1:0], conv_output_ff[0]};
            ain_valid <= ~conv_to_valid_ff[2] & conv_to_valid_ff[1];
        end
    end
    always @(posedge clk_sys) begin
        if (~conv_to_valid_ff[2] & conv_to_valid_ff[1]) begin
            ain1_data <= sdo1_sr[15:3];
            ain2_data <= sdo2_sr[15:3];
            ain3_data <= sdo3_sr[15:3];
            ain4_data <= sdo4_sr[15:3];
            ain5_data <= sdo5_sr[15:3];
            ain6_data <= sdo6_sr[15:3];
            ain7_data <= sdo7_sr[15:3];
            ain8_data <= sdo8_sr[15:3];
        end
    end
endmodule
