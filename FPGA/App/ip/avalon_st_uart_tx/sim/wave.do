onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk
add wave -noupdate /test/dut/uart_txd
add wave -noupdate -divider AvalonST
add wave -noupdate /test/sink_ready
add wave -noupdate /test/sink_valid
add wave -noupdate -radix hexadecimal /test/sink_data
add wave -noupdate -divider Internal
add wave -noupdate -radix unsigned /test/dut/prescaler
add wave -noupdate -radix unsigned /test/dut/tx_counter
add wave -noupdate -radix hexadecimal /test/dut/tx_buffer
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ps} 0}
quietly wave cursor active 0
configure wave -namecolwidth 150
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 2
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {0 ps} {1 ns}
