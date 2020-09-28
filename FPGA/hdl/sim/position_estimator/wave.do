onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk
add wave -noupdate -format Analog-Step -height 64 -max 500.625 /test/theta_deg
add wave -noupdate /test/theta
add wave -noupdate /test/enc_a
add wave -noupdate /test/enc_b
add wave -noupdate -expand /test/hall_uvw
add wave -noupdate -divider QDEC
add wave -noupdate /test/qdec_inc
add wave -noupdate /test/qdec_dec
add wave -noupdate -divider PE
add wave -noupdate -format Analog-Step -height 64 -max 511.0 -radix unsigned /test/est/theta_data
add wave -noupdate /test/est/theta_error
add wave -noupdate /test/est/theta_uncertain
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {799071 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 152
configure wave -valuecolwidth 82
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
WaveRestoreZoom {11457256 ps} {37550478 ps}
