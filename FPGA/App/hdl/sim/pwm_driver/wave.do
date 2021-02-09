onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk100mhz
add wave -noupdate /test/driver/pwm_sink_valid
add wave -noupdate -radix decimal /test/driver/pwm_sink_data
add wave -noupdate /test/trigger
add wave -noupdate /test/driver/trigger_delayed
add wave -noupdate /test/hall_uvw
add wave -noupdate -expand /test/driver/driver_pwm
add wave -noupdate -expand /test/driver/driver_reset_n
add wave -noupdate -divider {New Divider}
add wave -noupdate -radix decimal /test/driver/next_pwm_compare
add wave -noupdate /test/driver/next_pwm_available
add wave -noupdate -radix unsigned /test/driver/pwm_compare
add wave -noupdate -radix unsigned /test/driver/pwm_counter
add wave -noupdate /test/driver/pwm_drive
add wave -noupdate -divider {New Divider}
add wave -noupdate /test/driver/comm_direction
add wave -noupdate /test/driver/comm_input
add wave -noupdate /test/driver/comm_phase_enable
add wave -noupdate /test/driver/comm_phase_polarity
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {28205156 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 218
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
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
WaveRestoreZoom {26878426 ps} {29562280 ps}
