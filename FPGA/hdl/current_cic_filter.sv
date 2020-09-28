// Original : http://rapidnack.com/?p=1417

// Gain = RATE^NUM_STAGES / 2^(IN_WIDTH+SCALE_WIDTH-OUT_WIDTH)
//      = 6.18

module current_cic_filter
    #(
		parameter NUM_STAGES = 4,
        parameter RATE = 30,
		parameter RATE_WIDTH = 5,
		parameter SCALE_WIDTH = 20, // ceil(log2(RATE) * NUM_STAGES)
        parameter IN_WIDTH = 13,
		parameter OUT_WIDTH = 16
    ) (
        input wire clk,
        input wire reset,
        input wire in_valid,
        input wire signed [IN_WIDTH-1:0] in_data,
        output reg out_valid,
        output reg signed [OUT_WIDTH-1:0] out_data
    );

	localparam WIDTH = SCALE_WIDTH + IN_WIDTH;
	
	int i;

	logic signed [WIDTH-1:0] integ [0:NUM_STAGES];
	logic signed [WIDTH-1:0] diff [0:NUM_STAGES];
	logic signed [WIDTH-1:0] diff_d [0:NUM_STAGES];
	always @(posedge clk, posedge reset) begin
		if (reset == 1'b1) begin
			for (i = 1; i <= NUM_STAGES; i = i + 1) begin
				integ[i] <= '0;
			end
			
			diff[1] <= '0;
		end
        else begin
            if (in_valid == 1'b1) begin
                integ[1] <= in_data + integ[1];
                
                for (i = 2; i <= NUM_STAGES; i = i + 1) begin
                    integ[i] <= integ[i-1] + integ[i];
                end
                
                diff[1] <= integ[NUM_STAGES-1] + integ[NUM_STAGES];
            end
		end
	end
	
    logic [RATE_WIDTH-1:0] count;
	logic next_out_valid;
	always @(posedge clk, posedge reset) begin
		if (reset == 1'b1) begin
			count <= '0;
			next_out_valid <= 1'b1;
		end
        else begin
            if (in_valid == 1'b1) begin        
                if (count == (RATE - 1)) begin
                    count <= '0;
                    next_out_valid <= 1'b1;
                end
                else begin
                    count <= count + 1'b1;
                    next_out_valid <= 1'b0;
                end
            end
            else begin
                next_out_valid <= 1'b0;
            end
		end
	end
	
	wire signed [WIDTH-1:0] out_data_raw = diff[NUM_STAGES] - diff_d[NUM_STAGES];
	always @(posedge clk, posedge reset) begin
		if (reset == 1'b1) begin
			for (i = 1; i <= NUM_STAGES; i = i + 1) begin
				diff_d[i] <= '0;
			end
			
			for (i = 2; i <= NUM_STAGES; i = i + 1) begin
				diff[i] <= '0;
			end
			out_data <= '0;
            out_valid <= 1'b0;
		end
        else begin
            out_valid <= next_out_valid;
			if (next_out_valid == 1'b1) begin
				for (i = 1; i <= NUM_STAGES; i = i + 1) begin
					diff_d[i] <= diff[i];
				end
			
				for (i = 2; i <= NUM_STAGES; i = i + 1) begin
					diff[i] <= diff[i-1] - diff_d[i-1];
				end
				
				//out_data <= (diff[NUM_STAGES] - diff_d[NUM_STAGES]) >>> SCALE_WIDTH;
				out_data <= out_data_raw[WIDTH-1:WIDTH-OUT_WIDTH];
			end
		end
	end
endmodule
