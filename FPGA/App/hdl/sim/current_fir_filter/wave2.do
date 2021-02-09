onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test2/clk
add wave -noupdate -radix decimal -childformat {{{/test2/x[0]} -radix decimal} {{/test2/x[1]} -radix decimal} {{/test2/x[2]} -radix decimal} {{/test2/x[3]} -radix decimal} {{/test2/x[4]} -radix decimal} {{/test2/x[5]} -radix decimal} {{/test2/x[6]} -radix decimal} {{/test2/x[7]} -radix decimal}} -expand -subitemconfig {{/test2/x[0]} {-height 15 -radix decimal} {/test2/x[1]} {-height 15 -radix decimal} {/test2/x[2]} {-height 15 -radix decimal} {/test2/x[3]} {-height 15 -radix decimal} {/test2/x[4]} {-height 15 -radix decimal} {/test2/x[5]} {-height 15 -radix decimal} {/test2/x[6]} {-height 15 -radix decimal} {/test2/x[7]} {-height 15 -radix decimal}} /test2/x
add wave -noupdate /test2/in_valid
add wave -noupdate /test2/in_ready
add wave -noupdate -radix decimal -childformat {{{/test2/y[0]} -radix decimal} {{/test2/y[1]} -radix decimal} {{/test2/y[2]} -radix decimal} {{/test2/y[3]} -radix decimal} {{/test2/y[4]} -radix decimal} {{/test2/y[5]} -radix decimal} {{/test2/y[6]} -radix decimal} {{/test2/y[7]} -radix decimal}} -expand -subitemconfig {{/test2/y[0]} {-height 15 -radix decimal} {/test2/y[1]} {-height 15 -radix decimal} {/test2/y[2]} {-height 15 -radix decimal} {/test2/y[3]} {-height 15 -radix decimal} {/test2/y[4]} {-height 15 -radix decimal} {/test2/y[5]} {-height 15 -radix decimal} {/test2/y[6]} {-height 15 -radix decimal} {/test2/y[7]} {-height 15 -radix decimal}} /test2/y
add wave -noupdate /test2/out_valid
add wave -noupdate /test2/out_ready
add wave -noupdate -divider {New Divider}
add wave -noupdate -radix unsigned /test2/fir/index
add wave -noupdate -radix unsigned /test2/fir/read_address
add wave -noupdate /test2/fir/read_valid
add wave -noupdate -radix unsigned /test2/fir/write_address
add wave -noupdate /test2/fir/write_enable
add wave -noupdate -radix decimal -childformat {{{/test2/fir/az[0]} -radix decimal} {{/test2/fir/az[1]} -radix decimal} {{/test2/fir/az[2]} -radix decimal} {{/test2/fir/az[3]} -radix decimal} {{/test2/fir/az[4]} -radix decimal} {{/test2/fir/az[5]} -radix decimal} {{/test2/fir/az[6]} -radix decimal} {{/test2/fir/az[7]} -radix decimal} {{/test2/fir/az[8]} -radix decimal} {{/test2/fir/az[9]} -radix decimal} {{/test2/fir/az[10]} -radix decimal} {{/test2/fir/az[11]} -radix decimal} {{/test2/fir/az[12]} -radix decimal} {{/test2/fir/az[13]} -radix decimal} {{/test2/fir/az[14]} -radix decimal} {{/test2/fir/az[15]} -radix decimal} {{/test2/fir/az[16]} -radix decimal} {{/test2/fir/az[17]} -radix decimal} {{/test2/fir/az[18]} -radix decimal} {{/test2/fir/az[19]} -radix decimal} {{/test2/fir/az[20]} -radix decimal} {{/test2/fir/az[21]} -radix decimal} {{/test2/fir/az[22]} -radix decimal} {{/test2/fir/az[23]} -radix decimal} {{/test2/fir/az[24]} -radix decimal} {{/test2/fir/az[25]} -radix decimal} {{/test2/fir/az[26]} -radix decimal} {{/test2/fir/az[27]} -radix decimal} {{/test2/fir/az[28]} -radix decimal} {{/test2/fir/az[29]} -radix decimal} {{/test2/fir/az[30]} -radix decimal} {{/test2/fir/az[31]} -radix decimal} {{/test2/fir/az[32]} -radix decimal} {{/test2/fir/az[33]} -radix decimal} {{/test2/fir/az[34]} -radix decimal} {{/test2/fir/az[35]} -radix decimal} {{/test2/fir/az[36]} -radix decimal} {{/test2/fir/az[37]} -radix decimal} {{/test2/fir/az[38]} -radix decimal} {{/test2/fir/az[39]} -radix decimal} {{/test2/fir/az[40]} -radix decimal} {{/test2/fir/az[41]} -radix decimal} {{/test2/fir/az[42]} -radix decimal} {{/test2/fir/az[43]} -radix decimal} {{/test2/fir/az[44]} -radix decimal} {{/test2/fir/az[45]} -radix decimal} {{/test2/fir/az[46]} -radix decimal} {{/test2/fir/az[47]} -radix decimal} {{/test2/fir/az[48]} -radix decimal} {{/test2/fir/az[49]} -radix decimal} {{/test2/fir/az[50]} -radix decimal} {{/test2/fir/az[51]} -radix decimal} {{/test2/fir/az[52]} -radix decimal} {{/test2/fir/az[53]} -radix decimal} {{/test2/fir/az[54]} -radix decimal} {{/test2/fir/az[55]} -radix decimal}} -subitemconfig {{/test2/fir/az[0]} {-height 15 -radix decimal} {/test2/fir/az[1]} {-height 15 -radix decimal} {/test2/fir/az[2]} {-height 15 -radix decimal} {/test2/fir/az[3]} {-height 15 -radix decimal} {/test2/fir/az[4]} {-height 15 -radix decimal} {/test2/fir/az[5]} {-height 15 -radix decimal} {/test2/fir/az[6]} {-height 15 -radix decimal} {/test2/fir/az[7]} {-height 15 -radix decimal} {/test2/fir/az[8]} {-height 15 -radix decimal} {/test2/fir/az[9]} {-height 15 -radix decimal} {/test2/fir/az[10]} {-height 15 -radix decimal} {/test2/fir/az[11]} {-height 15 -radix decimal} {/test2/fir/az[12]} {-height 15 -radix decimal} {/test2/fir/az[13]} {-height 15 -radix decimal} {/test2/fir/az[14]} {-height 15 -radix decimal} {/test2/fir/az[15]} {-height 15 -radix decimal} {/test2/fir/az[16]} {-height 15 -radix decimal} {/test2/fir/az[17]} {-height 15 -radix decimal} {/test2/fir/az[18]} {-height 15 -radix decimal} {/test2/fir/az[19]} {-height 15 -radix decimal} {/test2/fir/az[20]} {-height 15 -radix decimal} {/test2/fir/az[21]} {-height 15 -radix decimal} {/test2/fir/az[22]} {-height 15 -radix decimal} {/test2/fir/az[23]} {-height 15 -radix decimal} {/test2/fir/az[24]} {-height 15 -radix decimal} {/test2/fir/az[25]} {-height 15 -radix decimal} {/test2/fir/az[26]} {-height 15 -radix decimal} {/test2/fir/az[27]} {-height 15 -radix decimal} {/test2/fir/az[28]} {-height 15 -radix decimal} {/test2/fir/az[29]} {-height 15 -radix decimal} {/test2/fir/az[30]} {-height 15 -radix decimal} {/test2/fir/az[31]} {-height 15 -radix decimal} {/test2/fir/az[32]} {-height 15 -radix decimal} {/test2/fir/az[33]} {-height 15 -radix decimal} {/test2/fir/az[34]} {-height 15 -radix decimal} {/test2/fir/az[35]} {-height 15 -radix decimal} {/test2/fir/az[36]} {-height 15 -radix decimal} {/test2/fir/az[37]} {-height 15 -radix decimal} {/test2/fir/az[38]} {-height 15 -radix decimal} {/test2/fir/az[39]} {-height 15 -radix decimal} {/test2/fir/az[40]} {-height 15 -radix decimal} {/test2/fir/az[41]} {-height 15 -radix decimal} {/test2/fir/az[42]} {-height 15 -radix decimal} {/test2/fir/az[43]} {-height 15 -radix decimal} {/test2/fir/az[44]} {-height 15 -radix decimal} {/test2/fir/az[45]} {-height 15 -radix decimal} {/test2/fir/az[46]} {-height 15 -radix decimal} {/test2/fir/az[47]} {-height 15 -radix decimal} {/test2/fir/az[48]} {-height 15 -radix decimal} {/test2/fir/az[49]} {-height 15 -radix decimal} {/test2/fir/az[50]} {-height 15 -radix decimal} {/test2/fir/az[51]} {-height 15 -radix decimal} {/test2/fir/az[52]} {-height 15 -radix decimal} {/test2/fir/az[53]} {-height 15 -radix decimal} {/test2/fir/az[54]} {-height 15 -radix decimal} {/test2/fir/az[55]} {-height 15 -radix decimal}} /test2/fir/az
add wave -noupdate -divider {New Divider}
add wave -noupdate /test2/fir/mult_in1
add wave -noupdate /test2/fir/mult_in2
add wave -noupdate /test2/fir/add_in
add wave -noupdate /test2/fir/mac_out
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {24 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 171
configure wave -valuecolwidth 104
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
WaveRestoreZoom {11906 ns} {12127 ns}
