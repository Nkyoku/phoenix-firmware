onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk
add wave -noupdate /test/trigger
add wave -noupdate -divider Common
add wave -noupdate -format Analog-Step -height 74 -max 400.19999999999999 /test/theta_in_deg
add wave -noupdate /test/hall_uvw
add wave -noupdate /test/enc_ab
add wave -noupdate -divider {SS PWM Driver}
add wave -noupdate -expand /test/ss/driver_pwm
add wave -noupdate -expand /test/ss/driver_reset_n
add wave -noupdate -divider {DS PWM Driver}
add wave -noupdate -expand /test/ds/driver_pwm
add wave -noupdate -expand /test/ds/driver_reset_n
add wave -noupdate -radix decimal /test/ds_pwm_a_data
add wave -noupdate -radix decimal /test/ds_pwm_b_data
add wave -noupdate -radix decimal /test/ds_driver_pwm_u_data
add wave -noupdate -radix decimal /test/ds_driver_pwm_v_data
add wave -noupdate -radix decimal /test/ds_driver_pwm_w_data
add wave -noupdate -divider DS
add wave -noupdate /test/ds/dir
add wave -noupdate /test/ds/counter
add wave -noupdate /test/ds/u_compare
add wave -noupdate /test/ds/v_compare
add wave -noupdate /test/ds/w_compare
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {39170549 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 150
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1
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
WaveRestoreZoom {35809250 ps} {44991928 ps}
