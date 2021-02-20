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
        input  wire                             brake,
        input  wire unsigned [GAIN_WIDTH-1:0]   param_kp, // 比例ゲイン
        input  wire unsigned [GAIN_WIDTH-1:0]   param_ki, // 積分ゲイン
        input  wire [DATA_COUNT*DATA_WIDTH-1:0] in_ref_data, // 目標量 {r0, r1, ...}
        input  wire                             in_ref_valid,
        input  wire [DATA_COUNT*DATA_WIDTH-1:0] in_proc_data, // 制御量 {y0, y1, ...}
        input  wire                             in_proc_valid,
        output reg  [DATA_COUNT*DATA_WIDTH-1:0] out_data, // 操作量 {u0, u1, ...}
        output reg                              out_valid
    );
    
    localparam int DATA_MIN = -DATA_LIMIT * 2**SCALE;
    localparam int DATA_MAX = DATA_LIMIT * 2**SCALE;
    localparam int TEMP_WIDTH = GAIN_WIDTH + DATA_WIDTH;
    localparam int SAT_WIDTH = $clog2(DATA_LIMIT + 1) + 1 + SCALE; // 飽和演算の出力幅
    localparam int INDEX_MAX = 3 + 3 * DATA_COUNT;
    
    int i;
    
    logic [$clog2(INDEX_MAX+1)-1:0] index = '0;
    logic [2:0] state = '0;
    logic brake_enabled = 1'b0;
    logic r_valid = 1'b0;
    logic y_valid = 1'b0;
    logic signed [DATA_WIDTH-1:0] r [0:DATA_COUNT-1];
    logic signed [DATA_WIDTH-1:0] y [0:DATA_COUNT-1];
    
    // Error
    logic signed [DATA_WIDTH:0] e;
    always @(posedge clk) begin
        if (state[0] & (index <= (3 * DATA_COUNT - 2))) begin
            e <= 'X;
            for (i = 0; i < DATA_COUNT; i = i + 1) begin
                if (index == (1 + 3 * i)) begin
                    e <= (DATA_WIDTH+1)'(r[i]) - (DATA_WIDTH+1)'(y[i]);
                end
            end
        end
        else begin
            e <= 'X;
        end
    end
    
    // Derivation
    logic signed [DATA_WIDTH:0] e_z [0:DATA_COUNT-1];
    logic signed [DATA_WIDTH+1:0] diff_e;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            for (i = 0; i < DATA_COUNT; i = i + 1) begin
                e_z[i] <= '0;
            end
        end
        else for (i = 0; i < DATA_COUNT; i = i + 1) begin
            if (index == (2 + 3 * i)) begin
                e_z[i] <= e;
            end
        end
    end
    always @(posedge clk) begin
        if (state[1] & (index <= (3 * DATA_COUNT - 1))) begin
            diff_e <= 'X;
            for (i = 0; i < DATA_COUNT; i = i + 1) begin
                if (index == (2 + 3 * i)) begin
                    diff_e <= (DATA_WIDTH+2)'(e) - (DATA_WIDTH+2)'(e_z[i]);
                end
            end
        end
        else begin
            diff_e <= 'X;
        end
    end
    
    // Gain
    logic signed [DATA_WIDTH+1:0] gain_in1;
    logic signed [GAIN_WIDTH:0] gain_in2;
    logic signed [GAIN_WIDTH+DATA_WIDTH+2:0] gain_out_raw;
    wire  signed [GAIN_WIDTH+DATA_WIDTH+1:0] gain_out = gain_out_raw[GAIN_WIDTH+DATA_WIDTH+1:0];
    always @(posedge clk) begin
        if (state[1] == 1'b1) begin
            gain_in1 <= e;
            gain_in2 <= {1'b0, param_ki};
        end
        else if (state[2] == 1'b1) begin
            gain_in1 <= diff_e;
            gain_in2 <= {1'b0, param_kp};
        end
        else begin
            gain_in1 <= 'X;
            gain_in2 <= 'X;
        end
        gain_out_raw = gain_in1 * gain_in2;
    end
    
    // Integrator
    logic signed [GAIN_WIDTH+DATA_WIDTH+2:0] integ;
    logic signed [SAT_WIDTH-1:0] u_z [0:DATA_COUNT-1];
    wire  signed [SAT_WIDTH-1:0] u_sat = (integ < DATA_MIN) ? SAT_WIDTH'(DATA_MIN) : ((DATA_MAX < integ) ? SAT_WIDTH'(DATA_MAX) : SAT_WIDTH'(integ));
    wire  signed [DATA_WIDTH-1:0] u = DATA_WIDTH'((u_sat + 2**(SCALE - 1)) >>> SCALE);
    always @(posedge clk) begin
        if (state[2] & (index <= (3 * DATA_COUNT))) begin
            integ <= 'X;
            for (i = 0; i < DATA_COUNT; i = i + 1) begin
                if (index == (3 + 3 * i)) begin
                    integ <= brake_enabled ? '0 : (GAIN_WIDTH+DATA_WIDTH+3)'(u_z[i]);
                end
            end
        end
        else if ((3 < index) & (state[0] | state[1]) & (index <= (3 * DATA_COUNT + 2))) begin
            integ <= brake_enabled ? '0 : (integ + (GAIN_WIDTH+DATA_WIDTH+3)'(gain_out));
        end
        else begin
            integ <= 'X;
        end
        for (i = 0; i < DATA_COUNT; i = i + 1) begin
            if (index == (6 + 3 * i)) begin
                out_data[DATA_WIDTH * (DATA_COUNT - 1 - i) +: DATA_WIDTH] <= u;
            end
        end
    end
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            for (i = 0; i < DATA_COUNT; i = i + 1) begin
                u_z[i] <= '0;
            end
        end
        else for (i = 0; i < DATA_COUNT; i = i + 1) begin
            if (index == (6 + 3 * i)) begin
                u_z[i] <= u_sat;
            end
        end
    end
    
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            out_valid <= 1'b0;
            index <= '0;
            state <= '0;
            brake_enabled <= 1'b0;
            r_valid <= 1'b0;
            y_valid <= 1'b0;
            for (i = 0; i < DATA_COUNT; i = i + 1) begin
                r[i] <= '0;
                y[i] <= '0;
            end
        end
        else begin
            if (in_ref_valid == 1'b1) begin
                r_valid <= 1'b1;
                for (i = 0; i < DATA_COUNT; i = i + 1) begin
                    r[i] <= in_ref_data[DATA_WIDTH * (DATA_COUNT - 1 - i) +: DATA_WIDTH];
                end
            end
            if (in_proc_valid == 1'b1) begin
                y_valid <= 1'b1;
                for (i = 0; i < DATA_COUNT; i = i + 1) begin
                    y[i] <= in_proc_data[DATA_WIDTH * (DATA_COUNT - 1 - i) +: DATA_WIDTH];
                end
            end
            if (trigger & (index == 0)) begin
                index <= 1'b1;
                state <= 3'b001;
                brake_enabled <= brake;
            end
            else begin
                index <= ((0 < index) & (index < INDEX_MAX)) ? (index + 1'b1) : '0;
                state <= (index < INDEX_MAX) ? {state[1:0], state[2]} : '0;
            end
            out_valid <= (INDEX_MAX <= index);
        end
    end
endmodule
