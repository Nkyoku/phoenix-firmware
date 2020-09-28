onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk1
add wave -noupdate /test/clk2
add wave -noupdate -radix hexadecimal /test/sink_data
add wave -noupdate /test/sink_valid
add wave -noupdate /test/dut/sink_ready
add wave -noupdate -radix hexadecimal /test/dut/source_data
add wave -noupdate /test/dut/source_valid
add wave -noupdate /test/dut/source_ready
add wave -noupdate /test/dut/req
add wave -noupdate /test/dut/req_ff1
add wave -noupdate /test/dut/req_ff2
add wave -noupdate /test/dut/ack
add wave -noupdate /test/dut/ack_ff1
add wave -noupdate /test/dut/ack_ff2
add wave -noupdate -radix hexadecimal /test/output_data
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {904870 ps} 0}
quietly wave cursor active 1
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
WaveRestoreZoom {515742 ps} {983180 ps}
