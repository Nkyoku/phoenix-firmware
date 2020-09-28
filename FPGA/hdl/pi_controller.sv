module pi_controller #(
        parameter int SCALE = 16,
        parameter int GAIN_WIDTH = 16,
        parameter int DATA_WIDTH = 16,
        parameter int DATA_COUNT = 1,
        parameter int DATA_LIMIT = 2960
    ) (
        input  wire                             clk,
        input  wire                             reset,
        input  wire                             trigger,
        input  wire signed [GAIN_WIDTH-1:0]     param_kp, // 比例ゲイン
        input  wire signed [GAIN_WIDTH-1:0]     param_ki, // 積分ゲイン
        input  wire [DATA_COUNT*DATA_WIDTH-1:0] in_ref_data, // 目標量 {x0, x1, ...}
        input  wire                             in_ref_valid,
        input  wire [DATA_COUNT*DATA_WIDTH-1:0] in_proc_data, // 制御量 {y0, y1, ...}
        input  wire                             in_proc_valid,
        output reg  [DATA_COUNT*DATA_WIDTH-1:0] out_data, // 操作量 {u0, u1, ...}
        output reg                              out_valid
    );
    
    localparam int DATA_MIN = -DATA_LIMIT * 2**SCALE;
    localparam int DATA_MAX = DATA_LIMIT * 2**SCALE;
    localparam int TEMP_WIDTH = GAIN_WIDTH + DATA_WIDTH;
    
    logic r_valid = 1'b0;
    logic y_valid = 1'b0;
    logic signed [DATA_WIDTH-1:0] r [0:DATA_COUNT-1];
    logic signed [DATA_WIDTH-1:0] y [0:DATA_COUNT-1];
    logic [$clog2(DATA_COUNT+1)-1:0] count = '0;
    logic [4:0] state = '0;
    
    // Error
    logic signed [DATA_WIDTH-1:0] e;
    always @(posedge clk) begin
        e <= r[count] - y[count];
    end
    
    // Gain
    logic signed [DATA_WIDTH-1:0] gain_in1;
    logic signed [GAIN_WIDTH-1:0] gain_in2;
    wire  signed [TEMP_WIDTH-1:0] gain_out = gain_in1 * gain_in2;
    logic signed [DATA_WIDTH-1:0] diff_e;
    always @(posedge clk) begin
        if (state[1] == 1'b1) begin
            gain_in1 <= e;
            gain_in2 <= param_ki;
        end
        else if (state[2] == 1'b1) begin
            gain_in1 <= diff_e;
            gain_in2 <= param_kp;
        end
    end
    
    // Derivation
    logic signed [DATA_WIDTH-1:0] e_z [0:DATA_COUNT-1];
    always @(posedge clk) begin
        if (state[1] == 1'b1) begin
            diff_e <= e - e_z[count];
        end
    end
    always @(posedge clk, posedge reset) begin
        int i;
        if (reset) begin
            for (i = 0; i < DATA_COUNT; i = i + 1) begin
                e_z[i] <= '0;
            end
        end
        else begin
            if (state[1] == 1'b1) begin
                e_z[count] <= e;
            end
        end
    end
    
    // Integerator
    logic signed [TEMP_WIDTH-1:0] integ;
    logic signed [TEMP_WIDTH-1:0] u_z [0:DATA_COUNT-1];
    wire  signed [TEMP_WIDTH-1:0] u_raw = (integ < DATA_MIN) ? TEMP_WIDTH'(DATA_MIN) : ((DATA_MAX < integ) ? TEMP_WIDTH'(DATA_MAX) : integ);
    wire  signed [DATA_WIDTH-1:0] u = u_raw[SCALE+:DATA_WIDTH];
    always @(posedge clk) begin
        if (state[1] == 1'b1) begin
            integ <= u_z[count];
        end
        else if (state[2] | state[3]) begin
            integ <= integ + gain_out;
        end
        if (state[4] == 1'b1) begin
            out_data[DATA_WIDTH * count +: DATA_WIDTH] <= u;
        end
    end
    always @(posedge clk, posedge reset) begin
        int i;
        if (reset) begin
            for (i = 0; i < DATA_COUNT; i = i + 1) begin
                u_z[i] <= '0;
            end
        end
        else begin
            if (state[4] == 1'b1) begin
                u_z[count] <= u_raw;
            end
        end
    end
    
    always @(posedge clk, posedge reset) begin
        int i;
        if (reset) begin
            out_valid <= 1'b0;
            count <= '0;
            state <= '0;
            r_valid <= 1'b0;
            y_valid <= 1'b0;
            for (i = 0; i < DATA_COUNT; i = i + 1) begin
                r[i] <= '0;
                y[i] <= '0;
            end
        end
        else begin
            out_valid <= 1'b0;
            if (in_ref_valid == 1'b1) begin
                r_valid <= 1'b1;
                for (i = 0; i < DATA_COUNT; i = i + 1) begin
                    r[i] <= in_ref_data[DATA_WIDTH * i +: DATA_WIDTH];
                end
            end
            if (in_proc_valid == 1'b1) begin
                y_valid <= 1'b1;
                for (i = 0; i < DATA_COUNT; i = i + 1) begin
                    y[i] <= in_proc_data[DATA_WIDTH * i +: DATA_WIDTH];
                end
            end
            state <= {state[$bits(state)-2:0], 1'b0};
            if (trigger & r_valid & y_valid & (count == '0) & (state == '0)) begin
                count <= '0;
                state[0] <= 1'b1;
            end
            else if (state[$bits(state)-1] == 1'b1) begin
                if (count < (DATA_COUNT - 1)) begin
                    count <= count + 1'b1;
                    state[0] <= 1'b1;
                end
                else begin
                    count <= '0;
                    out_valid <= 1'b1;
                end
            end
        end
    end
endmodule
