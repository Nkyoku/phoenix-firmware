onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk
add wave -noupdate /test/trigger
add wave -noupdate /test/fault
add wave -noupdate /test/pwm_valid
add wave -noupdate -radix unsigned /test/pwm_u_data
add wave -noupdate -radix unsigned /test/pwm_v_data
add wave -noupdate -radix unsigned /test/pwm_w_data
add wave -noupdate -divider {New Divider}
add wave -noupdate /test/driver/update
add wave -noupdate -radix unsigned /test/driver/u_compare_update
add wave -noupdate /test/driver/drive
add wave -noupdate /test/driver/dir
add wave -noupdate -radix unsigned /test/driver/counter
add wave -noupdate -radix unsigned /test/driver/u_compare
add wave -noupdate -radix unsigned /test/driver/v_compare
add wave -noupdate -radix unsigned /test/driver/w_compare
add wave -noupdate -radix unsigned /test/driver/u_compare_add_dir
add wave -noupdate -radix unsigned /test/driver/v_compare_add_dir
add wave -noupdate -radix unsigned /test/driver/w_compare_add_dir
add wave -noupdate -divider {New Divider}
add wave -noupdate -expand /test/driver/driver_pwm
add wave -noupdate /test/driver/driver_reset_n
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {4258 ns} 0}
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
WaveRestoreZoom {3978 ns} {5287 ns}
