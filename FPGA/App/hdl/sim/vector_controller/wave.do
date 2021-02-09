onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk
add wave -noupdate /test/pulse_1khz
add wave -noupdate /test/pulse_8khz
add wave -noupdate /test/pulse_50khz
add wave -noupdate /test/fault
add wave -noupdate -divider Motor
add wave -noupdate /test/theta
add wave -noupdate /test/omega
add wave -noupdate /test/T
add wave -noupdate /test/V_u
add wave -noupdate /test/V_v
add wave -noupdate /test/V_w
add wave -noupdate -format Analog-Step -height 74 -max 76.007300000000001 -min -36.937199999999997 /test/V_d
add wave -noupdate -format Analog-Step -height 74 -max 91.003 /test/V_q
add wave -noupdate -format Analog-Step -height 74 -max 2.0419900000000002 -min -0.45986700000000003 /test/I_d
add wave -noupdate -format Analog-Step -height 74 -max 0.36590700000000004 -min -1.8077099999999999 /test/I_q
add wave -noupdate /test/I_u
add wave -noupdate /test/I_v
add wave -noupdate /test/I_w
add wave -noupdate -divider Vec
add wave -noupdate -radix decimal /test/vec/current_a_data
add wave -noupdate -radix decimal /test/vec/current_b_data
add wave -noupdate -radix decimal /test/vec/current_meas_d_data
add wave -noupdate -radix decimal /test/vec/current_meas_q_data
add wave -noupdate -radix decimal /test/vec/current_reference_data
add wave -noupdate -radix decimal /test/vec/pwm_d_data
add wave -noupdate -radix decimal /test/vec/pwm_q_data
add wave -noupdate -radix decimal /test/vec/pwm_a_data
add wave -noupdate -radix decimal /test/vec/pwm_b_data
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {35513728 ns} 0}
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
configure wave -timelineunits ns
update
WaveRestoreZoom {30936399 ns} {43322880 ns}
