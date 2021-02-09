onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk
add wave -noupdate /test/in_start
add wave -noupdate /test/in_stop
add wave -noupdate /test/in_ack
add wave -noupdate -radix hexadecimal /test/in_data
add wave -noupdate /test/in_valid
add wave -noupdate /test/in_ready
add wave -noupdate /test/out_ack_latch
add wave -noupdate -radix hexadecimal /test/out_data_latch
add wave -noupdate /test/out_valid
add wave -noupdate -divider {New Divider}
add wave -noupdate /test/i2c_scl
add wave -noupdate /test/i2c_sda
add wave -noupdate -divider {New Divider}
add wave -noupdate /test/i2c_sda_oe1
add wave -noupdate /test/i2c_sda_oe2
add wave -noupdate -divider {New Divider}
add wave -noupdate /test/i2cm/clock_enable
add wave -noupdate /test/i2cm/clock_state
add wave -noupdate -radix unsigned /test/i2cm/counter
add wave -noupdate /test/i2cm/scl_output1
add wave -noupdate /test/i2cm/scl_output2
add wave -noupdate /test/i2cm/sda_output1
add wave -noupdate /test/i2cm/sda_output2
add wave -noupdate -radix binary /test/i2cm/sda_sr
add wave -noupdate /test/i2cm/sda_input
add wave -noupdate -divider {New Divider}
add wave -noupdate /test/i2cs/state
add wave -noupdate /test/i2cs/counter
add wave -noupdate /test/i2cs/written_data
add wave -noupdate /test/i2cs/read_data
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {33 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 149
configure wave -valuecolwidth 82
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
WaveRestoreZoom {5752 ns} {7808 ns}
