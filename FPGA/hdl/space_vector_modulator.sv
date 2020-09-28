module space_vector_modulator #(
        parameter int DATA_WIDTH = 16,
        parameter int PERIOD = 3000,
        parameter int DATA_MIN = 20,
        parameter int DATA_MAX = 2980
    ) (
        input  wire                    clk,
        input  wire                    reset,
        input  wire [2*DATA_WIDTH-1:0] in_data, // {a, b}
        input  wire                    in_valid,
        output reg                     in_ready,
        output wire [3*DATA_WIDTH-1:0] out_data, // {u, v, w}
        output reg                     out_valid
    );
    
    localparam int SCALE = 10;
    logic signed [SCALE+1:0] SQRT3 = 1774;
    
    logic [4:0] state = '0;
    
    logic signed [DATA_WIDTH-1:0] a = '0;
    logic signed [DATA_WIDTH-1:0] b = '0;
    wire signed [SCALE+DATA_WIDTH+1:0] a_sqrt3_raw = a * SQRT3;
    logic signed [DATA_WIDTH-1:0] a_sqrt3;
    wire inverse = b[DATA_WIDTH-1];
    wire signed [DATA_WIDTH-1:0] abs_a_sqrt3 = a_sqrt3[DATA_WIDTH-1] ? -a_sqrt3 : a_sqrt3;
    wire signed [DATA_WIDTH-1:0] abs_b = b[DATA_WIDTH-1] ? -b : b;
    logic signed [DATA_WIDTH-1:0] t1, t2;
    wire signed [DATA_WIDTH-1:0] t3_half = (DATA_WIDTH'(PERIOD) - t1 - t2) >>> 1;
    logic signed [DATA_WIDTH-1:0] u0, v0, w0;
    logic signed [DATA_WIDTH-1:0] u1, v1, w1;
    logic signed [DATA_WIDTH-1:0] u_out, v_out, w_out;
    assign out_data = {u_out, v_out, w_out};
    always @(posedge clk) begin
        a_sqrt3 = a_sqrt3_raw[SCALE+:DATA_WIDTH];
        if (abs_b < abs_a_sqrt3) begin
            t1 <= abs_b;
            t2 <= (abs_a_sqrt3 - abs_b) >>> 1;
            if (a_sqrt3[$bits(a_sqrt3)-1] == inverse) begin
                u0 <= t1 + t2;
                v0 <= t1;
                w0 <= '0;
            end
            else begin
                u0 <= '0;
                v0 <= t1 + t2;
                w0 <= t2;
            end
        end
        else begin
            t1 <= (abs_b + a_sqrt3) >>> 1;
            t2 <= (abs_b - a_sqrt3) >>> 1;
            u0 <= inverse ? t2 : t1;
            v0 <= t1 + t2;
            w0 <= '0;
        end
        u1 <= inverse ? (DATA_WIDTH'(PERIOD) - (u0 + t3_half)) : (u0 + t3_half);
        v1 <= inverse ? (DATA_WIDTH'(PERIOD) - (v0 + t3_half)) : (v0 + t3_half);
        w1 <= inverse ? (DATA_WIDTH'(PERIOD) - (w0 + t3_half)) : (w0 + t3_half);
        if (state[4] == 1'b1) begin
            u_out = (u1 < DATA_MIN) ? DATA_WIDTH'(DATA_MIN) : ((DATA_MAX < u1) ? DATA_WIDTH'(DATA_MAX) : u1);
            v_out = (v1 < DATA_MIN) ? DATA_WIDTH'(DATA_MIN) : ((DATA_MAX < v1) ? DATA_WIDTH'(DATA_MAX) : v1);
            w_out = (w1 < DATA_MIN) ? DATA_WIDTH'(DATA_MIN) : ((DATA_MAX < w1) ? DATA_WIDTH'(DATA_MAX) : w1);
        end
    end
    
    logic assert_ready = 1'b1;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            out_valid <= 1'b0;
            in_ready <= 1'b0;
            state <= '0;
            assert_ready <= 1'b1;
            a <= '0;
            b <= '0;
        end
        else begin
            assert_ready <= 1'b0;
            if (assert_ready == 1'b1) begin
                in_ready <= 1'b1;
            end
            state <= {state[$bits(state)-2:0], in_valid & in_ready};
            out_valid <= state[4];
            if (state[4] == 1'b1) begin
                in_ready <= 1'b1;
            end
            if (in_valid & in_ready) begin
                a <= in_data[DATA_WIDTH+:DATA_WIDTH];
                b <= in_data[0+:DATA_WIDTH];
                in_ready <= 1'b0;
            end
        end
    end
endmodule
