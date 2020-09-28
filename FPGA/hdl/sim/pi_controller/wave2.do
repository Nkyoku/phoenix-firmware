onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test2/clk
add wave -noupdate /test2/r0
add wave -noupdate /test2/r1
add wave -noupdate /test2/in_ref_valid
add wave -noupdate /test2/y0
add wave -noupdate /test2/y1
add wave -noupdate /test2/in_proc_valid
add wave -noupdate /test2/u0
add wave -noupdate /test2/u1
add wave -noupdate /test2/out_valid
add wave -noupdate /test2/trigger
add wave -noupdate /test2/fault
add wave -noupdate -divider {New Divider}
add wave -noupdate /test2/pi/count
add wave -noupdate /test2/pi/state
add wave -noupdate {/test2/pi/r[0]}
add wave -noupdate {/test2/pi/r[1]}
add wave -noupdate {/test2/pi/y[0]}
add wave -noupdate {/test2/pi/y[1]}
add wave -noupdate /test2/pi/e
add wave -noupdate {/test2/pi/e_z[0]}
add wave -noupdate {/test2/pi/e_z[1]}
add wave -noupdate /test2/pi/diff_e
add wave -noupdate /test2/pi/gain_in1
add wave -noupdate /test2/pi/gain_in2
add wave -noupdate /test2/pi/gain_out
add wave -noupdate /test2/pi/integ
add wave -noupdate /test2/pi/u_raw
add wave -noupdate /test2/pi/u
add wave -noupdate {/test2/pi/u_z[0]}
add wave -noupdate {/test2/pi/u_z[1]}
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {12045 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 157
configure wave -valuecolwidth 125
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
WaveRestoreZoom {0 ps} {50682 ps}
