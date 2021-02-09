create_clock -period 40 [get_ports {CLK25MHz}]
create_clock -period 10 [get_ports {ADC1_CLKOUT}]
create_clock -period 20 [get_ports {FPGA_SPI_SCLK}]
derive_pll_clocks
derive_clock_uncertainty
set_false_path -from {pll_0|altpll_component|auto_generated|pll1|clk[2]} -to {FPGA_SPI_SCLK}

# ADC1 I/O timing constraints
set ADC1_BD 0.5
create_clock -name ADC1_CLKOUT_virtual -period 10.000
set_input_delay -clock {ADC1_CLKOUT_virtual} -max [expr 5.0 + $ADC1_BD] [get_ports {ADC1_SDO*}]
set_input_delay -clock {ADC1_CLKOUT_virtual} -min [expr 3.5 - $ADC1_BD] [get_ports {ADC1_SDO*}]
