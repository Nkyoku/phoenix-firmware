module AvalonStUartBridge
    #(
        parameter FULL_DUPLEX = 0,
        parameter INVERT_TXD_RXD = 0,
        parameter INVERT_OE = 0,
        parameter PRESCALER = 1
    ) (
        input  wire       clk,                  //    clk.clk
        input  wire       reset,                //  reset.reset
        output reg        sink_ready,           //       .ready
        input  wire       sink_valid,           //   sink.valid
        input  wire [7:0] sink_data,            //       .data
        input  wire       sink_startofpacket,   //       .startofpacket
        input  wire       sink_endofpacket,     //       .endofpacket
        input  wire       source_ready,         // source.ready
        output reg        source_valid,         //       .valid
        output reg  [7:0] source_data,          //       .data
        output reg        source_startofpacket, //       .startofpacket
        output reg        source_endofpacket,   //       .endofpacket
        output reg        source_error,         //       .error
        output reg        uart_txd,             //   uart.txd
        input  wire       uart_rxd,             //       .rxd
        output reg        uart_oe               //       .oe
    );

    
    // Prescaler
    reg [$clog2(PRESCALER)-1:0] prescaler;
    wire clk_enable = (prescaler == 0) ? 1'b1 : 1'b0;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            prescaler <= '0;
        end
        else begin
            if (prescaler == (PRESCALER - 1)) begin
                prescaler <= '0;
            end
            else begin
                prescaler <= prescaler + 1;
            end
        end
    end
    
    
    // Arbitration signal
    reg tx_in_transaction;
    reg rx_in_transaction;
    
    // Tx logic
    reg [2:0] tx_prescaler;
    reg [9:0] tx_buffer;
    reg [3:0] tx_counter;
    reg tx_sop;
    reg tx_eop;
    
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            uart_txd <= (INVERT_TXD_RXD == 0) ? 1'b1 : 1'b0;
            uart_oe <= (INVERT_OE == 0) ? 1'b0 : 1'b1;
            sink_ready <= 1'b1;
            tx_prescaler <= '0;
            tx_buffer <= '0;
            tx_counter <= '0;
            tx_sop <= 1'b0;
            tx_eop <= 1'b0;
            tx_in_transaction <= 1'b0;
        end
        else begin
            if (sink_valid & sink_ready) begin
                uart_txd <= (INVERT_TXD_RXD == 0) ? 1'b1 : 1'b0;
                tx_buffer <= {1'b1, sink_data, 1'b0};
                tx_counter <= 10;
                tx_prescaler <= '0;
                tx_sop <= 1'b1;
                tx_eop <= sink_endofpacket;
                sink_ready <= 1'b0;
            end
            else begin
                if ((FULL_DUPLEX != 0) | ~rx_in_transaction) begin
                    tx_in_transaction <= tx_in_transaction | tx_sop;
                    tx_sop <= 1'b0;
                end
                if (clk_enable & tx_in_transaction) begin
                    tx_prescaler <= tx_prescaler + 1;
                    if (tx_prescaler == 0) begin
                        if (0 < tx_counter) begin
                            uart_oe <= (INVERT_OE == 0) ? 1'b1 : 1'b0;
                            uart_txd <= (INVERT_TXD_RXD == 0) ? tx_buffer[0] : ~tx_buffer[0];
                            tx_buffer <= {1'b1, tx_buffer[9:1]};
                            tx_counter <= tx_counter - 1;
                        end
                        else begin
                            uart_oe <= (INVERT_OE == 0) ? 1'b0 : 1'b1;
                            uart_txd <= (INVERT_TXD_RXD == 0) ? 1'b1 : 1'b0;
                            tx_in_transaction <= tx_in_transaction & ~tx_eop;
                            tx_eop <= 1'b0;
                            sink_ready <= 1'b1;
                        end
                    end
                end
            end
        end
    end
    
    
    // Rx logic
    reg [2:0] rx_buffered_rxd;
    wire rx_filtered_rxd = (INVERT_TXD_RXD != 0) ^ ((rx_buffered_rxd[0] & rx_buffered_rxd[1]) | (rx_buffered_rxd[1] & rx_buffered_rxd[2]) | (rx_buffered_rxd[0] & rx_buffered_rxd[2]));
    reg [2:0] rx_prescaler;
    reg [3:0] rx_counter;
    reg [8:0] rx_buffer;
    reg rx_sop;
    reg rx_idle;
    reg rx_hold;
    reg rx_break_n;
    
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            source_valid <= 1'b0;
            source_data <= '0;
            source_startofpacket <= 1'b0;
            source_endofpacket <= 1'b0;
            source_error <= 1'b0;
            rx_buffered_rxd <= (INVERT_TXD_RXD == 0) ? '1 : '0;
            rx_prescaler <= '0;
            rx_counter <= '0;
            rx_buffer <= '0;
            rx_in_transaction <= 1'b0;
            rx_sop <= 1'b0;
            rx_idle <= 1'b1;
            rx_hold <= 1'b0;
            rx_break_n <= 1'b1;
        end
        else begin
            rx_buffered_rxd[0] <= uart_rxd;
            if (clk_enable == 1'b1) begin
                rx_buffered_rxd[2:1] <= rx_buffered_rxd[1:0];
            end
            
            if (source_valid & source_ready) begin
                source_valid <= 1'b0;
                source_startofpacket <= 1'b0;
                source_endofpacket <= 1'b0;
            end
            
            if (clk_enable == 1'b1) begin
                if (((FULL_DUPLEX == 0) & tx_in_transaction) | ~rx_break_n) begin
                    rx_break_n <= rx_filtered_rxd;
                    rx_in_transaction <= 1'b0;
                    rx_idle <= 1'b1;
                    rx_hold <= 1'b0;
                end
                else begin
                    rx_prescaler <= rx_prescaler + 1;
                    
                    if (rx_idle == 1'b1) begin
                        if (rx_prescaler == 0) begin
                            rx_counter <= rx_counter - 1;
                        end
                        
                        if (~rx_filtered_rxd | ((rx_prescaler == 0) & (rx_counter == 0))) begin
                            rx_counter <= 0;
                            rx_prescaler <= 0;
                            rx_idle <= rx_filtered_rxd;
                            rx_in_transaction <= ~rx_filtered_rxd;
                            rx_sop <= ~rx_in_transaction;
                            rx_hold <= 1'b0;
                            
                            if (rx_hold == 1'b1) begin
                                source_valid <= 1'b1;
                                source_data <= rx_buffer[7:0];
                                source_startofpacket <= rx_sop;
                                source_endofpacket <= rx_filtered_rxd | ~rx_buffer[8];
                                source_error <= ~rx_buffer[8];
                                rx_break_n <= rx_buffer[8];
                            end
                        end
                    end
                    else begin
                        if (rx_prescaler == 3) begin
                            rx_counter <= rx_counter + 1;
                            rx_buffer <= {rx_filtered_rxd, rx_buffer[8:1]};
                            if (9 <= rx_counter) begin
                                rx_idle <= 1'b1;
                                rx_hold <= 1'b1;
                            end
                        end
                    end
                end
            end
        end
    end
endmodule
