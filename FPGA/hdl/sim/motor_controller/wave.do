onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk150mhz
add wave -noupdate -divider ADC
add wave -noupdate /test/ain_valid
add wave -noupdate -radix hexadecimal /test/ain_data_u
add wave -noupdate -radix hexadecimal /test/ain_data_v
add wave -noupdate -divider CIC
add wave -noupdate /test/ain_cic_u_valid
add wave -noupdate -radix decimal /test/ain_cic_u_data
add wave -noupdate -radix decimal /test/ain_cic_v_data
add wave -noupdate /test/current_source_valid
add wave -noupdate -radix decimal /test/current_source_data
add wave -noupdate -divider Commutator
add wave -noupdate /test/drv/clk_pwm
add wave -noupdate /test/drv/sensor_hall_wvu
add wave -noupdate /test/drv/comm_phase_enable
add wave -noupdate /test/drv/comm_phase_polarity
add wave -noupdate -divider Driver
add wave -noupdate /test/drv/pwm_sink_valid
add wave -noupdate -radix decimal /test/drv/pwm_sink_data
add wave -noupdate /test/drv/pwm_sink_error
add wave -noupdate /test/drv/next_pwm_valid
add wave -noupdate /test/drv/next_pwm_drive
add wave -noupdate -radix unsigned /test/drv/next_pwm_compare
add wave -noupdate /test/drv/pwm_drive
add wave -noupdate -radix unsigned /test/drv/pwm_compare
add wave -noupdate -radix unsigned /test/drv/pwm_counter
add wave -noupdate /test/drv/conduit_pwm
add wave -noupdate /test/drv/conduit_reset_n
add wave -noupdate -divider Controller
add wave -noupdate /test/clk100mhz
add wave -noupdate /test/pwm_source_valid
add wave -noupdate /test/pwm_source_error
add wave -noupdate -radix decimal /test/pwm_source_data
add wave -noupdate /test/pwm_source_ready
add wave -noupdate -radix hexadecimal /test/current_sink_data
add wave -noupdate /test/current_sink_valid
add wave -noupdate -divider Bus
add wave -noupdate /test/slave_address
add wave -noupdate /test/slave_write
add wave -noupdate -radix hexadecimal /test/slave_writedata
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {19789561 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 196
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
WaveRestoreZoom {16411804 ps} {25807500 ps}
