onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk
add wave -noupdate -radix decimal /test/a
add wave -noupdate -radix decimal /test/b
add wave -noupdate /test/in_valid
add wave -noupdate /test/in_ready
add wave -noupdate -radix decimal /test/u
add wave -noupdate -radix decimal /test/v
add wave -noupdate -radix decimal /test/w
add wave -noupdate /test/out_valid
add wave -noupdate -divider {New Divider}
add wave -noupdate /test/svm/inverse
add wave -noupdate -radix decimal /test/svm/a
add wave -noupdate -radix decimal /test/svm/b
add wave -noupdate -radix decimal -childformat {{{/test/svm/a_sqrt3[15]} -radix decimal} {{/test/svm/a_sqrt3[14]} -radix decimal} {{/test/svm/a_sqrt3[13]} -radix decimal} {{/test/svm/a_sqrt3[12]} -radix decimal} {{/test/svm/a_sqrt3[11]} -radix decimal} {{/test/svm/a_sqrt3[10]} -radix decimal} {{/test/svm/a_sqrt3[9]} -radix decimal} {{/test/svm/a_sqrt3[8]} -radix decimal} {{/test/svm/a_sqrt3[7]} -radix decimal} {{/test/svm/a_sqrt3[6]} -radix decimal} {{/test/svm/a_sqrt3[5]} -radix decimal} {{/test/svm/a_sqrt3[4]} -radix decimal} {{/test/svm/a_sqrt3[3]} -radix decimal} {{/test/svm/a_sqrt3[2]} -radix decimal} {{/test/svm/a_sqrt3[1]} -radix decimal} {{/test/svm/a_sqrt3[0]} -radix decimal}} -subitemconfig {{/test/svm/a_sqrt3[15]} {-height 15 -radix decimal} {/test/svm/a_sqrt3[14]} {-height 15 -radix decimal} {/test/svm/a_sqrt3[13]} {-height 15 -radix decimal} {/test/svm/a_sqrt3[12]} {-height 15 -radix decimal} {/test/svm/a_sqrt3[11]} {-height 15 -radix decimal} {/test/svm/a_sqrt3[10]} {-height 15 -radix decimal} {/test/svm/a_sqrt3[9]} {-height 15 -radix decimal} {/test/svm/a_sqrt3[8]} {-height 15 -radix decimal} {/test/svm/a_sqrt3[7]} {-height 15 -radix decimal} {/test/svm/a_sqrt3[6]} {-height 15 -radix decimal} {/test/svm/a_sqrt3[5]} {-height 15 -radix decimal} {/test/svm/a_sqrt3[4]} {-height 15 -radix decimal} {/test/svm/a_sqrt3[3]} {-height 15 -radix decimal} {/test/svm/a_sqrt3[2]} {-height 15 -radix decimal} {/test/svm/a_sqrt3[1]} {-height 15 -radix decimal} {/test/svm/a_sqrt3[0]} {-height 15 -radix decimal}} /test/svm/a_sqrt3
add wave -noupdate -radix unsigned /test/svm/abs_a_sqrt3
add wave -noupdate -radix unsigned /test/svm/abs_b
add wave -noupdate -radix unsigned /test/svm/t1
add wave -noupdate -radix unsigned /test/svm/t2
add wave -noupdate -radix decimal /test/svm/t3
add wave -noupdate -radix unsigned /test/svm/u0
add wave -noupdate -radix unsigned /test/svm/v0
add wave -noupdate -radix unsigned /test/svm/w0
add wave -noupdate -radix decimal /test/svm/u1
add wave -noupdate -radix decimal /test/svm/v1
add wave -noupdate -radix decimal /test/svm/w1
add wave -noupdate -radix decimal /test/svm/u2
add wave -noupdate -radix decimal /test/svm/v2
add wave -noupdate -radix decimal /test/svm/w2
add wave -noupdate -radix unsigned /test/svm/u_out
add wave -noupdate -radix unsigned /test/svm/v_out
add wave -noupdate -radix unsigned /test/svm/w_out
add wave -noupdate -divider {New Divider}
add wave -noupdate -format Analog-Step -height 192 -max 3000.0 -radix unsigned /test/u_out
add wave -noupdate -format Analog-Step -height 192 -max 3000.0 -radix unsigned /test/v_out
add wave -noupdate -format Analog-Step -height 192 -max 3000.0 -radix unsigned /test/w_out
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {5975 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 141
configure wave -valuecolwidth 120
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
WaveRestoreZoom {6307 ns} {7331 ns}
