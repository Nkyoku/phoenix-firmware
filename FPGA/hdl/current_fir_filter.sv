// サンプリング周波数 Fs = 50kHz 
// カットオフ周波数   Fc = 4kHz (-3dB)

module current_fir_filter #(
        parameter DATA_WIDTH = 16,
        parameter DATA_COUNT = 1
    ) (
        input  wire                             clk,
        input  wire                             reset,
        input  wire [DATA_COUNT*DATA_WIDTH-1:0] in_data, // {x0, x1, ...}
        input  wire                             in_valid,
        output wire                             in_ready,
        output reg  [DATA_COUNT*DATA_WIDTH-1:0] out_data, // {y0, y1, ...}
        output reg                              out_valid,
        input  wire                             out_ready
    );
    
    localparam int COEF_WIDTH = 16;
    localparam int TAP = 7;
    localparam int TAP_WIDTH = DATA_WIDTH + $clog2(TAP);
    localparam int DATA_MIN = -2**(DATA_WIDTH - 1);
    localparam int DATA_MAX = 2**(DATA_WIDTH - 1) - 1;
    
    // Coefficients memory Q16
    logic signed [COEF_WIDTH-1:0] coefficients [0:7] = '{
        1966, 8263, 13986, 17106, 13986, 8263, 1966, 0
    };
    
    // Delay memory
    logic signed [0:TAP-1][TAP_WIDTH-1:0] az [0:DATA_COUNT-1];
    initial begin
        int i, j;
        for (j = 0; j < DATA_COUNT; j = j + 1) begin
            for (i = 0; i < TAP; i = i + 1) begin
                az[j][i] = '0;//10 * j + i;
            end
        end
    end
    
    // Multiply and Add
    logic signed [DATA_WIDTH-1:0] mult_in1 = '0;
    logic signed [COEF_WIDTH-1:0] mult_in2 = '0;
    logic signed [TAP_WIDTH-1:0] add_in = '0;
    wire signed [TAP_WIDTH+COEF_WIDTH-1:0] mult_out = mult_in1 * mult_in2 + 2**(COEF_WIDTH - 1);
    //wire signed [TAP_WIDTH-1:0] mult_out2 = mult_out[COEF_WIDTH+:TAP_WIDTH];
    logic signed [TAP_WIDTH-1:0] mac_out = '0;
    wire signed [DATA_WIDTH-1:0] mac_out_sat = (mac_out < DATA_MIN) ? DATA_WIDTH'(DATA_MIN) : ((DATA_MAX < mac_out) ? DATA_WIDTH'(DATA_MAX) : mac_out[DATA_WIDTH-1:0]);
    always @(posedge clk) begin
        //mult_out <= mult_in1 * mult_in2;
        mac_out <= $signed(mult_out[COEF_WIDTH+:DATA_WIDTH]) + add_in;
    end
    
    logic [DATA_WIDTH-1:0] x[0:DATA_COUNT-1];
    logic [DATA_COUNT*TAP:0] state = '0;
    logic [$clog2(DATA_COUNT+1)-1:0] count = '0;
    logic [$clog2(TAP)-1:0] index_k = '0;
    wire busy = (count != 0) | (index_k != 0);
    assign in_ready = ~busy & ~out_valid;
    
    always @(posedge clk) begin
        int i;
        mult_in2 <= coefficients[index_k];
        add_in <= az[count][index_k];
        for (i = 0; i < DATA_COUNT; i = i + 1) begin
            if (state[i + 1] == 1'b1) begin
                out_data[DATA_WIDTH * i +: DATA_WIDTH] <= mac_out_sat;
            end
        end
    end
    
    always @(posedge clk, posedge reset) begin
        int i, j;
        if (reset == 1'b1) begin
            out_valid <= 1'b0;
            count <= '0;
            state <= '0;
            index_k <= '0;
        end
        else begin
            state <= {state[$bits(state)-2:0], in_valid & in_ready};
            if (in_valid & in_ready) begin
                for (i = 0; i < DATA_COUNT; i = i + 1) begin
                    x[i] <= in_data[DATA_WIDTH * i +: DATA_WIDTH];
                end
                count <= (DATA_COUNT < 2) ? 1'b0 : 1'b1;
                mult_in1 <= in_data[DATA_WIDTH-1:0];
            end
            else if (busy == 1'b1) begin
                count <= count < (DATA_COUNT - 1) ? (count + 1'b1) : '0;
                mult_in1 <= x[count];
            end
            if (state[DATA_COUNT] == 1'b1) begin
                out_valid <= 1'b1;
            end
            if (out_valid & out_ready) begin
                out_valid <= 1'b0;
            end
            if (in_valid & in_ready) begin
                index_k <= (DATA_COUNT < 2) ? 1'b1 : 1'b0;
            end
            else if (((DATA_COUNT - 1) <= count) & busy) begin
                if (index_k < (TAP - 1)) begin
                    index_k <= index_k + 1'b1;
                end
                else begin
                    index_k <= '0;
                end
            end
            for (i = 0; i < (TAP - 1); i = i + 1) begin
                for (j = 0; j < DATA_COUNT; j = j + 1) begin
                    if (state[DATA_COUNT * (i + 1) + j + 1] == 1'b1) begin
                        az[j][i] <= mac_out;
                    end
                end
            end
        end
    end
endmodule
