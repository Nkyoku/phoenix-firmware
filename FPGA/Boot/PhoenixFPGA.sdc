create_clock -period 40 [get_ports {CLK25MHz}]
derive_pll_clocks
derive_clock_uncertainty
