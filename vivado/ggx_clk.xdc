# 200 MHz target for the single-lane GGX datapath (out-of-context: no board pins).
create_clock -period 5.000 -name aclk [get_ports s00_axis_aclk]
