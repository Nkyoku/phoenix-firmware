create_clock -period 40 [get_ports {CLK25MHz}]
create_clock -period 10 [get_ports {ADC1_CLKOUT}]
derive_pll_clocks
derive_clock_uncertainty

