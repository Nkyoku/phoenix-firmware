onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk
add wave -noupdate /test/fir0/in_ready
add wave -noupdate /test/fir0/in_valid
add wave -noupdate -radix decimal /test/fir0/in_data
add wave -noupdate /test/fir0/out_valid
add wave -noupdate -radix decimal /test/fir0/out_data
add wave -noupdate -divider Internal
add wave -noupdate -radix unsigned /test/fir0/k_index
add wave -noupdate -radix decimal /test/fir0/k
add wave -noupdate -radix decimal /test/fir0/x
add wave -noupdate -radix decimal /test/fir0/mult_out
add wave -noupdate -radix decimal /test/fir0/kx
add wave -noupdate -radix decimal {/test/fir0/az[0]}
add wave -noupdate -radix decimal {/test/fir0/az[1]}
add wave -noupdate -radix decimal {/test/fir0/az[2]}
add wave -noupdate -radix decimal {/test/fir0/az[3]}
add wave -noupdate -radix decimal {/test/fir0/az[4]}
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {3026124 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 134
configure wave -valuecolwidth 129
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
WaveRestoreZoom {0 ps} {488383 ps}
