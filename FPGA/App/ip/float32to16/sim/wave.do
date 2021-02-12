onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk
add wave -noupdate /test/clken
add wave -noupdate /test/start
add wave -noupdate /test/done
add wave -noupdate -radix hexadecimal /test/dataa
add wave -noupdate -radix hexadecimal /test/result
add wave -noupdate -divider UUT
add wave -noupdate /test/uut/sign_32
add wave -noupdate -radix hexadecimal /test/uut/exp_32
add wave -noupdate -radix hexadecimal /test/uut/coef_32
add wave -noupdate /test/uut/sign_16
add wave -noupdate -radix hexadecimal /test/uut/exp_16
add wave -noupdate -radix hexadecimal /test/uut/coef_16
add wave -noupdate -radix hexadecimal /test/uut/round_target
add wave -noupdate -radix hexadecimal /test/uut/round_target_shifted
add wave -noupdate -radix hexadecimal /test/uut/round_offset
add wave -noupdate -radix hexadecimal /test/uut/round_mask1
add wave -noupdate -radix hexadecimal /test/uut/round_mask2
add wave -noupdate /test/uut/state
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ns} 0}
quietly wave cursor active 0
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
WaveRestoreZoom {0 ns} {1 us}
