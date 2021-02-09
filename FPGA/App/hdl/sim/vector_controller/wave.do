onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk
add wave -noupdate /test/pulse_1khz
add wave -noupdate /test/pulse_8khz
add wave -noupdate /test/pulse_50khz
add wave -noupdate -divider Motor
add wave -noupdate /test/theta
add wave -noupdate /test/omega
add wave -noupdate /test/T
add wave -noupdate -format Analog-Step -height 74 -max 83.536000000000001 -min -83.328000000000003 /test/V_u
add wave -noupdate -format Analog-Step -height 74 -max 83.263999999999996 -min -83.215999999999994 /test/V_v
add wave -noupdate -format Analog-Step -height 74 -max 83.216000000000008 -min -83.120000000000005 /test/V_w
add wave -noupdate /test/V_d
add wave -noupdate /test/V_q
add wave -noupdate /test/I_d
add wave -noupdate /test/I_q
add wave -noupdate -format Analog-Step -height 74 -max 1.4108099999999999 -min -0.52379799999999999 /test/I_u
add wave -noupdate -format Analog-Step -height 74 -max 0.64793499999999982 -min -2.4215300000000002 /test/I_v
add wave -noupdate -format Analog-Step -height 74 -max 2.4823400000000002 -min -0.97218899999999997 /test/I_w
add wave -noupdate -divider Vec
add wave -noupdate -format Analog-Step -height 74 -max 511.0 -radix unsigned /test/vec/theta_data_latch
add wave -noupdate /test/vec/theta_error_latch
add wave -noupdate /test/vec/theta_uncertain_latch
add wave -noupdate /test/vec/current_a_data
add wave -noupdate /test/vec/current_b_data
add wave -noupdate /test/vec/current_meas_d_data
add wave -noupdate /test/vec/current_meas_q_data
add wave -noupdate /test/vec/current_reference_data
add wave -noupdate /test/vec/pi_dq_data
add wave -noupdate /test/vec/pwm_d_data
add wave -noupdate /test/vec/pwm_q_data
add wave -noupdate /test/vec/pwm_a_data
add wave -noupdate /test/vec/pwm_b_data
add wave -noupdate -radix unsigned /test/vec/pwm_u_data
add wave -noupdate -radix unsigned /test/vec/pwm_v_data
add wave -noupdate -radix unsigned /test/vec/pwm_w_data
add wave -noupdate /test/fault
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {13960576 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 194
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
WaveRestoreZoom {10352763 ns} {20507750 ns}
