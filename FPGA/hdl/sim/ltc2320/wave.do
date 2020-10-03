onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -divider ADC
add wave -noupdate /test/clk_100mhz
add wave -noupdate /test/clk_150mhz
add wave -noupdate /test/adc_cnv_n
add wave -noupdate /test/adc_sck
add wave -noupdate /test/adc_clkout
add wave -noupdate /test/adc_sdo
add wave -noupdate /test/adc/ain_valid_150mhz
add wave -noupdate -divider System
add wave -noupdate /test/clk_sys
add wave -noupdate /test/ain_valid
add wave -noupdate /test/ain1_data
add wave -noupdate /test/ain2_data
add wave -noupdate /test/ain3_data
add wave -noupdate /test/ain4_data
add wave -noupdate /test/ain5_data
add wave -noupdate /test/ain6_data
add wave -noupdate /test/ain7_data
add wave -noupdate /test/ain8_data
add wave -noupdate -divider Internal
add wave -noupdate -radix unsigned /test/adc/sck_counter
add wave -noupdate -radix unsigned /test/adc/conv_counter
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {1212442 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 150
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
WaveRestoreZoom {4875171 ps} {5106570 ps}
