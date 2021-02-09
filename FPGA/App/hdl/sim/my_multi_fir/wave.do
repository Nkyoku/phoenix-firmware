onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk
add wave -noupdate /test/in_ready
add wave -noupdate /test/in_valid
add wave -noupdate -radix unsigned /test/in_channel
add wave -noupdate /test/in_data
add wave -noupdate /test/out_valid
add wave -noupdate -radix unsigned /test/out_channel
add wave -noupdate -radix decimal /test/out_data
add wave -noupdate -divider {New Divider}
add wave -noupdate /test/fir0/kx
add wave -noupdate -radix unsigned /test/fir0/k_index
add wave -noupdate -radix hexadecimal /test/fir0/valid
add wave -noupdate /test/fir0/k
add wave -noupdate -divider {New Divider}
add wave -noupdate -radix unsigned /test/out_channel_valid
add wave -noupdate -radix decimal /test/out_data_valid
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
WaveRestoreZoom {811605 ps} {1396234 ps}
