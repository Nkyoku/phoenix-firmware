module ds_pwm_driver #(
        parameter int INITIAL_DIRECTION = 0,
        parameter int PERIOD            = 3000,
        parameter int MAX_ON_CYCLES     = 2980,
        parameter int DATA_WIDTH        = 16
    ) (
        input  wire                    clk,
        input  wire                    reset,
        input  wire                    trigger,      // 1のときPWMサイクルを開始する
        input  wire                    fault,        // 1のときPWMサイクルを停止し内部状態をリセットする
        input  wire                    pwm_valid,
        input  wire [3*DATA_WIDTH-1:0] pwm_data,     // {u, v, w}
        output reg  [2:0]              driver_pwm,
        output reg  [2:0]              driver_reset_n
    );
    
    localparam int COUNTER_WIDTH = $clog2(PERIOD);
    
    // PWM Counter
    wire [DATA_WIDTH-1:0] pwm_u_data = pwm_data[2*DATA_WIDTH+:DATA_WIDTH];
    wire [DATA_WIDTH-1:0] pwm_v_data = pwm_data[1*DATA_WIDTH+:DATA_WIDTH];
    wire [DATA_WIDTH-1:0] pwm_w_data = pwm_data[0*DATA_WIDTH+:DATA_WIDTH];
    logic dir = 1'(INITIAL_DIRECTION); // 0=>上昇, 1=>下降
    logic [COUNTER_WIDTH-1:0] counter;
    logic drive = 1'b0;
    logic [COUNTER_WIDTH-1:0] u_compare = '0;
    logic [COUNTER_WIDTH-1:0] v_compare = '0;
    logic [COUNTER_WIDTH-1:0] w_compare = '0;
    logic update = 1'b0;
    logic [COUNTER_WIDTH-1:0] u_compare_update = '0;
    logic [COUNTER_WIDTH-1:0] v_compare_update = '0;
    logic [COUNTER_WIDTH-1:0] w_compare_update = '0;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            dir <= 1'(INITIAL_DIRECTION);
            drive <= 1'b0;
            u_compare <= '0;
            v_compare <= '0;
            w_compare <= '0;
            update <= 1'b0;
        end
        else begin
            update <= (pwm_valid | (update & ~trigger)) & ~fault;
            if (pwm_valid == 1'b1) begin
                u_compare_update <= (pwm_u_data < MAX_ON_CYCLES) ? COUNTER_WIDTH'(pwm_u_data) : COUNTER_WIDTH'(MAX_ON_CYCLES);
                v_compare_update <= (pwm_v_data < MAX_ON_CYCLES) ? COUNTER_WIDTH'(pwm_v_data) : COUNTER_WIDTH'(MAX_ON_CYCLES);
                w_compare_update <= (pwm_w_data < MAX_ON_CYCLES) ? COUNTER_WIDTH'(pwm_w_data) : COUNTER_WIDTH'(MAX_ON_CYCLES);
            end
            if (trigger == 1'b1) begin
                dir <= ~dir;
            end
            if (fault == 1'b1) begin
                drive <= 1'b0;
            end
            else if (trigger == 1'b1) begin
                counter <= dir ? '0 : COUNTER_WIDTH'(PERIOD - 1);
                if (update == 1'b1) begin
                    drive <= 1'b1;
                    u_compare <= u_compare_update;
                    v_compare <= v_compare_update;
                    w_compare <= w_compare_update;
                end
            end
            else begin
                if (dir == 1'b0) begin
                    counter <= (counter < COUNTER_WIDTH'(PERIOD - 1)) ? (counter + 1'b1) : COUNTER_WIDTH'(PERIOD - 1);
                end
                else begin
                    counter <= (0 < counter) ? (counter - 1'b1) : '0;
                end
            end
        end
    end
    
    // PWM Wave Output
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            driver_pwm <= 3'b000;
            driver_reset_n <= 3'b000;
        end
        else begin
            driver_pwm[0] <= drive & (counter < u_compare);
            driver_pwm[1] <= drive & (counter < v_compare);
            driver_pwm[2] <= drive & (counter < w_compare);
            driver_reset_n <= {3{drive}};
        end
    end
endmodule
