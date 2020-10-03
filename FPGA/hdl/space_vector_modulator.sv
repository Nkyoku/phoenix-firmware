module space_vector_modulator #(
        parameter int DATA_WIDTH = 16,
        parameter int OUT_WIDTH = 12,
        parameter int PERIOD = 3000,
        parameter int OUT_MIN = 20,
        parameter int OUT_MAX = 2980
    ) (
        input  wire                    clk,
        input  wire                    reset,
        input  wire [2*DATA_WIDTH-1:0] in_data, // {a, b}
        input  wire                    in_valid,
        output reg                     in_ready,
        output wire [3*OUT_WIDTH-1:0]  out_data, // {u, v, w}
        output reg                     out_valid
    );
    
    localparam int SCALE = 10;
    logic signed [SCALE+1:0] SQRT3 = 1774;
    
    logic [3:0] state = '0;
    
    logic signed [DATA_WIDTH-1:0] a = '0;
    logic signed [DATA_WIDTH-1:0] b = '0;
    logic signed [SCALE+DATA_WIDTH+1:0] a_sqrt3_raw;
    wire signed [DATA_WIDTH-1:0] a_sqrt3 = a_sqrt3_raw[SCALE+:DATA_WIDTH];
    wire inverse = b[$bits(b)-1];
    wire signed [DATA_WIDTH-1:0] abs_a_sqrt3 = a_sqrt3[$bits(a_sqrt3)-1] ? -a_sqrt3 : a_sqrt3;
    wire signed [DATA_WIDTH-1:0] abs_b = b[$bits(b)-1] ? -b : b;
    logic signed [DATA_WIDTH-1:0] t1, t2, t3;
    //wire signed [DATA_WIDTH-1:0] t3 = t1 + t2;
    logic signed [DATA_WIDTH-1:0] u0, v0, w0;
    logic signed [DATA_WIDTH-1:0] u1, v1, w1;
    logic signed [DATA_WIDTH-1:0] u2, v2, w2;
    logic unsigned [OUT_WIDTH-1:0] u_out, v_out, w_out;
    assign u1 = u0 - (t3 >> 1);
    assign v1 = v0 - (t3 >> 1);
    assign w1 = w0 - (t3 >> 1);
    always @(posedge clk) begin
        a_sqrt3_raw <= a * SQRT3;
        if (abs_b < abs_a_sqrt3) begin
            t1 = abs_b;
            t2 = (abs_a_sqrt3 - abs_b) >> 1;
            t3 = (abs_a_sqrt3 + abs_b) >> 1;
            if (inverse == a_sqrt3[$bits(a_sqrt3)-1]) begin
                u0 <= t3;
                v0 <= t1;
                w0 <= '0;
            end
            else begin
                u0 <= '0;
                v0 <= t3;
                w0 <= t2;
            end
        end
        else begin
            t1 = (abs_b + abs_a_sqrt3) >> 1;
            t2 = (abs_b - abs_a_sqrt3) >> 1;
            t3 = abs_b;
            u0 <= (inverse == a_sqrt3[$bits(a_sqrt3)-1]) ? t1 : t2;
            v0 <= t3;
            w0 <= '0;
        end
        u2 <= DATA_WIDTH'(PERIOD / 2) + (~inverse ? u1 : -u1);
        v2 <= DATA_WIDTH'(PERIOD / 2) + (~inverse ? v1 : -v1);
        w2 <= DATA_WIDTH'(PERIOD / 2) + (~inverse ? w1 : -w1);
        if (state[3] == 1'b1) begin
            u_out = (u2 < OUT_MIN) ? OUT_WIDTH'(OUT_MIN) : ((OUT_MAX < u2) ? OUT_WIDTH'(OUT_MAX) : OUT_WIDTH'(u2));
            v_out = (v2 < OUT_MIN) ? OUT_WIDTH'(OUT_MIN) : ((OUT_MAX < v2) ? OUT_WIDTH'(OUT_MAX) : OUT_WIDTH'(v2));
            w_out = (w2 < OUT_MIN) ? OUT_WIDTH'(OUT_MIN) : ((OUT_MAX < w2) ? OUT_WIDTH'(OUT_MAX) : OUT_WIDTH'(w2));
        end
    end
    
    logic assert_ready = 1'b1;
    assign out_data = {u_out, v_out, w_out};
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
            out_valid <= state[$bits(state)-1];
            if (state[$bits(state)-1] == 1'b1) begin
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
