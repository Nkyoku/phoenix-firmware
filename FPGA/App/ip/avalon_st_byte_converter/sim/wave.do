onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/clk
add wave -noupdate -divider Sink
add wave -noupdate /test/sink_ready
add wave -noupdate /test/sink_valid
add wave -noupdate -radix hexadecimal /test/sink_channel
add wave -noupdate -radix hexadecimal -childformat {{{/test/sink_data[15]} -radix hexadecimal} {{/test/sink_data[14]} -radix hexadecimal} {{/test/sink_data[13]} -radix hexadecimal} {{/test/sink_data[12]} -radix hexadecimal} {{/test/sink_data[11]} -radix hexadecimal} {{/test/sink_data[10]} -radix hexadecimal} {{/test/sink_data[9]} -radix hexadecimal} {{/test/sink_data[8]} -radix hexadecimal} {{/test/sink_data[7]} -radix hexadecimal} {{/test/sink_data[6]} -radix hexadecimal} {{/test/sink_data[5]} -radix hexadecimal} {{/test/sink_data[4]} -radix hexadecimal} {{/test/sink_data[3]} -radix hexadecimal} {{/test/sink_data[2]} -radix hexadecimal} {{/test/sink_data[1]} -radix hexadecimal} {{/test/sink_data[0]} -radix hexadecimal}} -subitemconfig {{/test/sink_data[15]} {-height 15 -radix hexadecimal} {/test/sink_data[14]} {-height 15 -radix hexadecimal} {/test/sink_data[13]} {-height 15 -radix hexadecimal} {/test/sink_data[12]} {-height 15 -radix hexadecimal} {/test/sink_data[11]} {-height 15 -radix hexadecimal} {/test/sink_data[10]} {-height 15 -radix hexadecimal} {/test/sink_data[9]} {-height 15 -radix hexadecimal} {/test/sink_data[8]} {-height 15 -radix hexadecimal} {/test/sink_data[7]} {-height 15 -radix hexadecimal} {/test/sink_data[6]} {-height 15 -radix hexadecimal} {/test/sink_data[5]} {-height 15 -radix hexadecimal} {/test/sink_data[4]} {-height 15 -radix hexadecimal} {/test/sink_data[3]} {-height 15 -radix hexadecimal} {/test/sink_data[2]} {-height 15 -radix hexadecimal} {/test/sink_data[1]} {-height 15 -radix hexadecimal} {/test/sink_data[0]} {-height 15 -radix hexadecimal}} /test/sink_data
add wave -noupdate -divider Source
add wave -noupdate /test/source_ready
add wave -noupdate /test/source_valid
add wave -noupdate -radix hexadecimal /test/source_data
add wave -noupdate -radix hexadecimal /test/source_data_valid
add wave -noupdate -divider Internal
add wave -noupdate /test/dut/escaped
add wave -noupdate /test/dut/next_state
add wave -noupdate -radix unsigned /test/dut/next_address
add wave -noupdate -radix hexadecimal /test/dut/next_data
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {794897 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 176
configure wave -valuecolwidth 69
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
WaveRestoreZoom {548192 ps} {1365304 ps}
