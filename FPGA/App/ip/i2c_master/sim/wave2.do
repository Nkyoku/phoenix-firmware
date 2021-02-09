onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test2/clk
add wave -noupdate /test2/slave_address
add wave -noupdate /test2/slave_write
add wave -noupdate /test2/slave_writedata
add wave -noupdate /test2/slave_read
add wave -noupdate /test2/slave_readdata
add wave -noupdate /test2/irq
add wave -noupdate -divider {New Divider}
add wave -noupdate /test2/i2c_scl
add wave -noupdate /test2/i2c_sda
add wave -noupdate -divider {New Divider}
add wave -noupdate /test2/i2c_sda_oe1
add wave -noupdate /test2/i2c_sda_oe2
add wave -noupdate -divider {New Divider}
add wave -noupdate /test2/i2cm/reg_start
add wave -noupdate /test2/i2cm/reg_dir
add wave -noupdate /test2/i2cm/reg_length
add wave -noupdate /test2/i2cm/reg_daddr
add wave -noupdate /test2/i2cm/reg_iaddr
add wave -noupdate /test2/i2cm/reg_iaddr_size
add wave -noupdate /test2/i2cm/reg_txdata
add wave -noupdate /test2/i2cm/reg_rxdata
add wave -noupdate -divider {New Divider}
add wave -noupdate /test2/i2cm/state
add wave -noupdate /test2/i2cm/state_ack
add wave -noupdate /test2/i2cm/state_busy
add wave -noupdate /test2/i2cm/state_wait
add wave -noupdate /test2/i2cm/state_counter
add wave -noupdate /test2/i2cm/state_complete
add wave -noupdate -divider {New Divider}
add wave -noupdate /test2/i2cm/txrx_in_start
add wave -noupdate /test2/i2cm/txrx_in_stop
add wave -noupdate /test2/i2cm/txrx_in_ack
add wave -noupdate /test2/i2cm/txrx_in_data
add wave -noupdate /test2/i2cm/txrx_in_valid
add wave -noupdate /test2/i2cm/txrx_in_ready
add wave -noupdate /test2/i2cm/txrx_out_ack
add wave -noupdate /test2/i2cm/txrx_out_data
add wave -noupdate /test2/i2cm/txrx_out_valid
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ps} 0}
quietly wave cursor active 0
configure wave -namecolwidth 163
configure wave -valuecolwidth 113
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
WaveRestoreZoom {0 ps} {50800 ps}
