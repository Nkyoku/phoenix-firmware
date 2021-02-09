(* altera_attribute = {"-name SDC_STATEMENT \"set_false_path -from {*ControlPulseGenerator:*|clk_32khz} -to {*ControlPulseGenerator:*|clk_32khz_ff[0]}\""} *)
module ControlPulseGenerator #(
        parameter PULSE_1KHZ_START = 5
    ) (
        input  wire clk,
        input  wire reset,
        input  wire clk_64khz,
        output wire imu_clkin,
        input  wire imu_int_n,
        output reg  pulse_8khz,
        output reg  pulse_1khz
    );
    
    logic reset_64khz;
    ResetSynchronizer reset_0 (
        .clk       (clk_64khz),
        .reset_in  (reset),
        .reset_out (reset_64khz)
    );
    
    // Generate 32kHz clock for IMU
    reg clk_32khz = 1'b0;
    assign imu_clkin = clk_32khz;
    always @(posedge clk_64khz, posedge reset_64khz) begin
        if (reset_64khz == 1'b1) begin
            clk_32khz <= 1'b0;
        end
        else begin
            clk_32khz <= ~clk_32khz;
        end
    end
    
    // Generate 8kHz and 1kHz pulse
    reg imu_int_n_previous = 1'b1;
    reg [2:0] clk_32khz_ff = 3'b000;
    reg [5:0] pulse_1khz_counter = '0;
    reg [2:0] pulse_8khz_counter = '0;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            imu_int_n_previous <= 1'b1;
            clk_32khz_ff <= 3'b000;
            pulse_1khz_counter <= '0;
            pulse_8khz_counter <= '0;
            pulse_8khz <= 1'b0;
            pulse_1khz <= 1'b0;
        end
        else begin
            clk_32khz_ff <= {clk_32khz_ff[1:0], clk_32khz};
            if (~clk_32khz_ff[2] & clk_32khz_ff[1]) begin
                imu_int_n_previous <= imu_int_n;
                pulse_1khz_counter <= (~imu_int_n & imu_int_n_previous) ? '0 : (pulse_1khz_counter + 1'b1);
                pulse_8khz_counter <= (pulse_1khz_counter == PULSE_1KHZ_START) ? '0 : (pulse_8khz_counter + 1'b1);
                pulse_1khz <= (pulse_1khz_counter == PULSE_1KHZ_START) ? 1'b1 : 1'b0;
                pulse_8khz <= (pulse_8khz_counter == 3'b111) ? 1'b1 : 1'b0;
            end
            else begin
                pulse_1khz <= 1'b0;
                pulse_8khz <= 1'b0;
            end
        end
    end
endmodule
