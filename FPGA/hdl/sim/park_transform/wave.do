onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk
add wave -noupdate /test/in_inverse
add wave -noupdate -radix unsigned /test/in_theta
add wave -noupdate -radix decimal /test/in_y
add wave -noupdate -radix decimal /test/in_x
add wave -noupdate /test/in_valid
add wave -noupdate /test/in_ready
add wave -noupdate -radix decimal /test/out_y
add wave -noupdate -radix decimal /test/out_x
add wave -noupdate /test/out_valid
add wave -noupdate /test/out_ready
add wave -noupdate -divider {New Divider}
add wave -noupdate /test/park_0/state
add wave -noupdate -radix unsigned /test/park_0/table_address
add wave -noupdate -radix decimal /test/park_0/table_data
add wave -noupdate -radix decimal /test/park_0/mult_in1
add wave -noupdate -radix unsigned /test/park_0/mult_in2
add wave -noupdate /test/park_0/mult_out
add wave -noupdate /test/park_0/sum
add wave -noupdate /test/park_0/out_unsat
add wave -noupdate /test/park_0/out_sat
add wave -noupdate -divider {New Divider}
add wave -noupdate -format Analog-Step -height 74 -max 32767.0 -min -32767.0 /test/out_x_latched
add wave -noupdate -format Analog-Step -height 74 -max 32767.0 -min -32767.0 /test/out_y_latched
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ns} 0}
quietly wave cursor active 0
configure wave -namecolwidth 147
configure wave -valuecolwidth 156
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
WaveRestoreZoom {0 ns} {315 ns}
