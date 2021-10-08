/**
 * @file avalon_st_uart_tx.sv
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

module avalon_st_uart_tx
    #(
        parameter PRESCALER = 2
    ) (
        input  wire       clk,        //   clk.clk
        input  wire       reset,      // reset.reset
        output reg        sink_ready, //      .ready
        input  wire       sink_valid, //  sink.valid
        input  wire [7:0] sink_data,  //      .data
        output reg        uart_txd    //  uart.txd
    );
    
    reg [$clog2(PRESCALER)-1:0] prescaler;
    reg [10:0] tx_buffer = '1;
    reg [3:0] tx_counter = '0;
    
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            uart_txd <= 1'b1;
            sink_ready <= 1'b0;
            prescaler <= PRESCALER - 1'b1;
            tx_buffer <= '1;
            tx_counter <= '0;
        end
        else begin
            uart_txd <= tx_buffer[0];
            if (sink_valid & sink_ready) begin
                sink_ready <= 1'b0;
                tx_buffer <= {2'b11, sink_data, 1'b0};
                tx_counter <= 11;
                prescaler <= PRESCALER - 1'b1;
            end
            else if (0 < tx_counter) begin
                sink_ready <= (prescaler <= 1) & (tx_counter <= 1);
                if (prescaler == 0) begin
                    tx_buffer <= {1'b1, tx_buffer[10:1]};
                    tx_counter <= tx_counter - 1'b1;
                    prescaler <= PRESCALER - 1'b1;
                end
                else begin
                    prescaler <= prescaler - 1'b1;
                end
            end
            else begin
                sink_ready <= 1'b1;
                prescaler <= '0;
            end
        end
    end
endmodule
