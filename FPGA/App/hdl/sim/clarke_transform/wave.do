onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk
add wave -noupdate /test/in_u
add wave -noupdate /test/in_v
add wave -noupdate /test/in_valid
add wave -noupdate /test/in_ready
add wave -noupdate /test/out_a
add wave -noupdate /test/out_b
add wave -noupdate /test/out_valid
add wave -noupdate /test/out_ready
add wave -noupdate -divider {New Divider}
add wave -noupdate /test/sut/state
add wave -noupdate /test/sut/mult_in1
add wave -noupdate /test/sut/mult_in2
add wave -noupdate /test/sut/mult_out
add wave -noupdate /test/sut/mult_out
add wave -noupdate /test/sut/ab
add wave -noupdate /test/sut/ab_sat
add wave -noupdate /test/sut/a
add wave -noupdate /test/sut/b
add wave -noupdate -divider {New Divider}
add wave -noupdate -format Analog-Step -height 74 -max 32766.999999999993 -min -32768.0 /test/out_a_latched
add wave -noupdate -format Analog-Step -height 74 -max 32766.999999999993 -min -32768.0 /test/out_b_latched
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {43079 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 168
configure wave -valuecolwidth 123
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
WaveRestoreZoom {4776 ns} {53425 ns}
