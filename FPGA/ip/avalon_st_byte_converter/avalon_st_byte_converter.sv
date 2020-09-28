module avalon_st_byte_converter #(
        parameter CHANNEL_WIDTH = 8,
        parameter SYMBOL_WIDTH = 8,
        parameter SINK_SYMBOLS = 2
    ) (
        input  wire clk,
        input  wire reset,
        input  wire [SYMBOL_WIDTH*SINK_SYMBOLS-1:0] sink_data,
        input  wire [CHANNEL_WIDTH-1:0] sink_channel,
        input  wire sink_valid,
        output reg  sink_ready,
        output reg  [SYMBOL_WIDTH-1:0] source_data,
        output reg  source_valid,
        input  wire source_ready
    );
    
    enum {
        IDLE,
        CHANNEL,
        DATA
    } next_state = IDLE;
    logic escaped = 1'b0;
    logic [$clog2(SINK_SYMBOLS)-1:0] next_address = '0;
    logic [SYMBOL_WIDTH*SINK_SYMBOLS-1:0] latched_sink_data = '0;
    logic [SYMBOL_WIDTH-1:0] latched_sink_channel = '0;
    logic [SYMBOL_WIDTH-1:0] next_data;
    assign next_data = latched_sink_data[SYMBOL_WIDTH*next_address +: SYMBOL_WIDTH];
    
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            sink_ready <= 1'b1;
            source_valid <= 1'b0;
            next_state <= IDLE;
            escaped <= 1'b0;
        end
        else begin
            if (sink_valid & sink_ready) begin
                sink_ready <= 1'b0;
                latched_sink_data <= sink_data;
                latched_sink_channel <= sink_channel;
                source_data <= 8'h7C;
                source_valid <= 1'b1;
                next_state <= CHANNEL;
                escaped <= 1'b0;
            end
            else if (source_valid & source_ready) begin
                if (next_state == CHANNEL) begin
                    if (is_control(latched_sink_channel) & ~escaped) begin
                        source_data <= 8'h7D;
                        next_state <= CHANNEL;
                        escaped <= 1'b1;
                    end
                    else begin
                        source_data <= latched_sink_channel ^ (is_control(latched_sink_channel) ? 8'h20 : 8'h00);
                        next_address <= '0;
                        next_state <= DATA;
                        escaped <= 1'b0;
                    end
                    source_valid <= 1'b1;
                end
                else if (next_state == DATA) begin
                    if (is_control(next_data) & ~escaped) begin
                        source_data <= 8'h7D;
                        next_state <= DATA;
                        escaped <= 1'b1;
                    end
                    else begin
                        source_data <= next_data ^ (is_control(next_data) ? 8'h20 : 8'h00);
                        if (next_address < (SINK_SYMBOLS - 1)) begin
                            next_state <= DATA;
                        end
                        else begin
                            next_state <= IDLE;
                        end
                        next_address <= next_address + 1'b1;
                        escaped <= 1'b0;
                    end
                    source_valid <= 1'b1;
                end
                else begin
                    sink_ready <= 1'b1;
                    source_valid <= 1'b0;
                    next_state <= IDLE;
                    escaped <= 1'b0;
                end
            end
        end
    end
    
    function is_control(input [7:0] c);
        is_control = (8'h7A <= c) & (c <= 8'h7D);
    endfunction
endmodule
