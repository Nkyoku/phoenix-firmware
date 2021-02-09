module position_estimator #(
        parameter int THETA_WIDTH = 9
    ) (
        input  wire                   clk,
        input  wire                   reset,
        input  wire [2:0]             hall_uvw,
        input  wire                   qdec_inc,
        input  wire                   qdec_dec,
        output reg  [THETA_WIDTH-1:0] theta_data,
        output reg                    theta_error,
        output reg                    theta_uncertain
    );
    
    localparam int PULSE_PER_ROTATION = 2**THETA_WIDTH;
    
    // エンコーダ無しでホールセンサーから位置を割り出すために使うテーブル
    logic [THETA_WIDTH-1:0] default_theta_table [0:7] = '{
                                    // U, V, W
        0,                          // 0, 0, 0 : error
        2 * PULSE_PER_ROTATION / 3, // 0, 0, 1 : 240 deg
        PULSE_PER_ROTATION / 3,     // 0, 1, 0 : 120 deg
        PULSE_PER_ROTATION / 2,     // 0, 1, 1 : 180 deg
        0,                          // 1, 0, 0 :   0 deg
        5 * PULSE_PER_ROTATION / 6, // 1, 0, 1 : 300 deg
        PULSE_PER_ROTATION / 6,     // 1, 1, 0 :  60 deg
        0                           // 1, 1, 1 : error
    };
    
    wire hall_error = (hall_uvw == '0) | (hall_uvw == '1);
    logic [2:0] hall_uvw_latched = '0;
    wire [2:0] hall_uvw_transition = hall_uvw_latched ^ hall_uvw;
    
    always @(posedge clk) begin
        hall_uvw_latched <= hall_uvw;
    end
    
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            theta_data <= '0;
            theta_error <= 1'b1;
            theta_uncertain <= 1'b1;
        end
        else begin
            if (hall_error == 1'b1) begin
                // Error
                theta_data <= '0;
                theta_error <= 1'b1;
                theta_uncertain <= 1'b1;
            end
            else if ((hall_uvw_transition == 3'b100) & hall_uvw[1] & ~hall_uvw[0] & ~theta_error) begin
                // 90 deg
                theta_data <= THETA_WIDTH'(PULSE_PER_ROTATION / 4);
                theta_error <= 1'b0;
                theta_uncertain <= 1'b0;
            end
            else if ((hall_uvw_transition == 3'b100) & ~hall_uvw[1] & hall_uvw[0] & ~theta_error) begin
                // 270 deg
                theta_data <= THETA_WIDTH'(3 * PULSE_PER_ROTATION / 4);
                theta_error <= 1'b0;
                theta_uncertain <= 1'b0;
            end
            else if ((hall_uvw_transition == 3'b010) & hall_uvw[2] & ~hall_uvw[0] & ~theta_error) begin
                // 30 deg
                theta_data <= THETA_WIDTH'(PULSE_PER_ROTATION / 12);
                theta_error <= 1'b0;
                theta_uncertain <= 1'b0;
            end
            else if ((hall_uvw_transition == 3'b010) & ~hall_uvw[2] & hall_uvw[0] & ~theta_error) begin
                // 210 deg
                theta_data <= THETA_WIDTH'(7 * PULSE_PER_ROTATION / 12);
                theta_error <= 1'b0;
                theta_uncertain <= 1'b0;
            end
            else if ((hall_uvw_transition == 3'b001) & hall_uvw[2] & ~hall_uvw[1] & ~theta_error) begin
                // 330 deg
                theta_data <= THETA_WIDTH'(11 * PULSE_PER_ROTATION / 12);
                theta_error <= 1'b0;
                theta_uncertain <= 1'b0;
            end
            else if ((hall_uvw_transition == 3'b001) & ~hall_uvw[2] & hall_uvw[1] & ~theta_error) begin
                // 150 deg
                theta_data <= THETA_WIDTH'(5 * PULSE_PER_ROTATION / 12);
                theta_error <= 1'b0;
                theta_uncertain <= 1'b0;
            end
            else if (hall_uvw_transition != 3'b000) begin
                // 不正な遷移
                theta_data <= '0;
                theta_error <= 1'b1;
                theta_uncertain <= 1'b1;
            end
            else if (theta_uncertain == 1'b1) begin
                theta_data <= default_theta_table[hall_uvw];
                theta_error <= 1'b0;
                theta_uncertain <= 1'b1;
            end
            else if (qdec_inc & ~qdec_dec) begin
                theta_data <= theta_data + 1'b1;
            end
            else if (~qdec_inc & qdec_dec) begin
                theta_data <= theta_data - 1'b1;
            end
        end
    end
endmodule
