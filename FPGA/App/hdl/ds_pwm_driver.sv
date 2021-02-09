module ds_pwm_driver #(
        parameter int INITIAL_DIRECTION = 0,
        parameter int PERIOD            = 3000,
        parameter int MAX_ON_CYCLES     = 2980,
        parameter int DATA_WIDTH        = 12
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
    
    localparam int COUNTER_WIDTH = $clog2(PERIOD / 2);
    
    // PWM Counter
    wire [DATA_WIDTH-1:0] pwm_u_data = pwm_data[2*DATA_WIDTH+:DATA_WIDTH];
    wire [DATA_WIDTH-1:0] pwm_v_data = pwm_data[1*DATA_WIDTH+:DATA_WIDTH];
    wire [DATA_WIDTH-1:0] pwm_w_data = pwm_data[0*DATA_WIDTH+:DATA_WIDTH];
    logic dir = 1'(INITIAL_DIRECTION); // 0=>上昇, 1=>下降
    logic unsigned [COUNTER_WIDTH-1:0] counter;
    logic drive = 1'b0;
    logic unsigned [COUNTER_WIDTH:0] u_compare = '0;
    logic unsigned [COUNTER_WIDTH:0] v_compare = '0;
    logic unsigned [COUNTER_WIDTH:0] w_compare = '0;
    logic update = 1'b0;
    logic unsigned [COUNTER_WIDTH:0] u_compare_update = '0;
    logic unsigned [COUNTER_WIDTH:0] v_compare_update = '0;
    logic unsigned [COUNTER_WIDTH:0] w_compare_update = '0;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            dir <= 1'b0;
            counter <= COUNTER_WIDTH'(PERIOD / 2 - 1);
            drive <= 1'b0;
            u_compare <= '0;
            v_compare <= '0;
            w_compare <= '0;
            update <= 1'b0;
        end
        else begin
            update <= (pwm_valid | (update & ~trigger)) & ~fault;
            if (pwm_valid == 1'b1) begin
                u_compare_update <= (pwm_u_data < MAX_ON_CYCLES) ? (COUNTER_WIDTH+1)'(pwm_u_data) : (COUNTER_WIDTH+1)'(MAX_ON_CYCLES);
                v_compare_update <= (pwm_v_data < MAX_ON_CYCLES) ? (COUNTER_WIDTH+1)'(pwm_v_data) : (COUNTER_WIDTH+1)'(MAX_ON_CYCLES);
                w_compare_update <= (pwm_w_data < MAX_ON_CYCLES) ? (COUNTER_WIDTH+1)'(pwm_w_data) : (COUNTER_WIDTH+1)'(MAX_ON_CYCLES);
            end
            if (fault == 1'b1) begin
                drive <= 1'b0;
                dir <= 1'b0;
                counter <= COUNTER_WIDTH'(PERIOD / 2 - 1);
            end
            else if (trigger == 1'b1) begin
                dir <= 1'b0;
                counter <= COUNTER_WIDTH'(PERIOD / 2 - 1);
                if (update == 1'b1) begin
                    drive <= 1'b1;
                    u_compare <= u_compare_update;
                    v_compare <= v_compare_update;
                    w_compare <= w_compare_update;
                end
            end
            else begin
                if (dir == 1'b0) begin
                    if (0 < counter) begin
                        dir <= 1'b0;
                        counter <= counter - 1'b1;
                    end
                    else begin
                        dir <= 1'b1;
                        counter <= counter;
                    end
                end
                else begin
                    dir <= 1'b1;
                    counter <= (counter < (PERIOD / 2 - 1)) ? (counter + 1'b1) : COUNTER_WIDTH'(PERIOD / 2 - 1);
                end
            end
        end
    end
    
    // PWM Wave Output
    logic [2:0] driver_pwm_ff = '0;
    logic driver_reset_n_ff = '0;
    wire unsigned [COUNTER_WIDTH-1:0] u_compare_add_dir = (u_compare + dir) >> 1;
    wire unsigned [COUNTER_WIDTH-1:0] v_compare_add_dir = (v_compare + dir) >> 1;
    wire unsigned [COUNTER_WIDTH-1:0] w_compare_add_dir = (w_compare + dir) >> 1;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            driver_pwm_ff <= '0;
            driver_reset_n_ff <= 1'b0;
            driver_pwm <= '0;
            driver_reset_n <= '0;
        end
        else begin
            driver_pwm_ff[2] <= drive & (counter < u_compare_add_dir);
            driver_pwm_ff[1] <= drive & (counter < v_compare_add_dir);
            driver_pwm_ff[0] <= drive & (counter < w_compare_add_dir);
            driver_reset_n_ff <= drive;
            driver_pwm <= driver_pwm_ff;
            driver_reset_n <= {3{driver_reset_n_ff}};
        end
    end
endmodule
