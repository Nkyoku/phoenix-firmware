onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk
add wave -noupdate -radix decimal /test/x0
add wave -noupdate -radix decimal /test/x1
add wave -noupdate /test/in_valid1
add wave -noupdate /test/in_ready1
add wave -noupdate -radix decimal /test/y0
add wave -noupdate -radix decimal /test/y1
add wave -noupdate /test/fir/out_valid
add wave -noupdate /test/fir/out_ready
add wave -noupdate -divider {New Divider}
add wave -noupdate -radix unsigned /test/fir/index
add wave -noupdate -divider {New Divider}
add wave -noupdate /test/fir/mult_in1
add wave -noupdate /test/fir/mult_in2
add wave -noupdate /test/fir/mult_out
add wave -noupdate /test/fir/add_in
add wave -noupdate /test/fir/mac_out
add wave -noupdate -divider {New Divider}
add wave -noupdate -radix unsigned /test/fir/read_address
add wave -noupdate /test/fir/read_valid
add wave -noupdate -radix unsigned /test/fir/write_address
add wave -noupdate /test/fir/write_enable
add wave -noupdate /test/fir/az
add wave -noupdate -divider {New Divider}
add wave -noupdate -radix decimal /test/x2
add wave -noupdate /test/in_valid2
add wave -noupdate /test/in_ready2
add wave -noupdate -radix decimal /test/y2
add wave -noupdate /test/fir_1/out_valid
add wave -noupdate /test/fir_1/out_ready
add wave -noupdate -divider {New Divider}
add wave -noupdate -radix unsigned /test/fir_1/index
add wave -noupdate -divider {New Divider}
add wave -noupdate /test/fir_1/mult_in1
add wave -noupdate /test/fir_1/mult_in2
add wave -noupdate /test/fir_1/mult_out
add wave -noupdate /test/fir_1/add_in
add wave -noupdate /test/fir_1/mac_out
add wave -noupdate -divider {New Divider}
add wave -noupdate -radix unsigned /test/fir_1/read_address
add wave -noupdate /test/fir_1/read_valid
add wave -noupdate -radix unsigned /test/fir_1/write_address
add wave -noupdate /test/fir_1/write_enable
add wave -noupdate /test/fir_1/az
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {55 ns} 0}
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
WaveRestoreZoom {35 ns} {310 ns}
