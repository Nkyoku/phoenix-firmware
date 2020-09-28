onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /test/imu_spim_0/clk
add wave -noupdate /test/imu_spim_0/spim_cs_n
add wave -noupdate /test/imu_spim_0/spim_sclk
add wave -noupdate /test/imu_spim_0/spim_mosi
add wave -noupdate /test/imu_spim_0/spim_miso
add wave -noupdate /test/imu_spim_0/spim_int
add wave -noupdate -divider SPI
add wave -noupdate -radix unsigned /test/imu_spim_0/spi_counter
add wave -noupdate -radix hexadecimal /test/imu_spim_0/spi_din
add wave -noupdate -radix hexadecimal /test/imu_spim_0/spi_dout
add wave -noupdate -divider Register
add wave -noupdate -radix hexadecimal /test/imu_spim_0/reg_temp
add wave -noupdate -radix hexadecimal /test/imu_spim_0/reg_accel_x
add wave -noupdate -radix hexadecimal /test/imu_spim_0/reg_accel_y
add wave -noupdate -radix hexadecimal /test/imu_spim_0/reg_accel_z
add wave -noupdate -radix hexadecimal /test/imu_spim_0/reg_gyro_x
add wave -noupdate -radix hexadecimal /test/imu_spim_0/reg_gyro_y
add wave -noupdate -radix hexadecimal /test/imu_spim_0/reg_gyro_z
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ps} 0}
quietly wave cursor active 0
configure wave -namecolwidth 252
configure wave -valuecolwidth 159
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
WaveRestoreZoom {0 ps} {47750 ps}
