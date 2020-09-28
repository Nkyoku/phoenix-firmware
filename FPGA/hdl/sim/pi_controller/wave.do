onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk
add wave -noupdate /test/in_ref_data
add wave -noupdate /test/in_ref_valid
add wave -noupdate /test/in_proc_data
add wave -noupdate /test/in_proc_valid
add wave -noupdate /test/out_data
add wave -noupdate /test/out_valid
add wave -noupdate /test/trigger
add wave -noupdate /test/fault
add wave -noupdate -divider {New Divider}
add wave -noupdate /test/pi/count
add wave -noupdate /test/pi/state
add wave -noupdate /test/pi/r
add wave -noupdate /test/pi/y
add wave -noupdate /test/pi/e
add wave -noupdate /test/pi/e_z
add wave -noupdate /test/pi/diff_e
add wave -noupdate /test/pi/integ
add wave -noupdate /test/pi/u_raw
add wave -noupdate /test/pi/u_z
add wave -noupdate /test/pi/u
add wave -noupdate /test/pi/gain_in1
add wave -noupdate /test/pi/gain_in2
add wave -noupdate /test/pi/gain_out
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {99663 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 153
configure wave -valuecolwidth 160
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
WaveRestoreZoom {2277032 ps} {3553452 ps}
