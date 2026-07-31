# Logic-vs-route classification of the worst paths + phys_opt_design probe.
create_project -in_memory -part xc7z020clg400-1
read_verilog -sv {
  ../hdl/ggx_latency_pkg.sv
  ../hdl/axis_ggx_control.sv ../hdl/axis_skid_buffer.sv ../hdl/axis_fifo_2deep.sv
  ../hdl/axis_ggx_event_basis.sv ../hdl/axis_top_lvl_sampler.sv
  ../hdl/axis_ggx_projected_area.sv ../hdl/axis_ggx_reproject_normalize.sv
  ../hdl/axis_fixed_norm3.sv ../hdl/axis_fixed_sqrt.sv ../hdl/axis_fixed_div.sv
  ../hdl/axis_fixed_inv_sqrt_nodsp.sv ../hdl/axis_sobol2d_stateless.sv
  ../hdl/axis_nested_uniform_scramble.sv ../hdl/axis_hash_combine_2d.sv ../hdl/axis_trig_lut.sv
}
read_verilog -sv { ../hdl/axis_pre_ggx_sampler.v }
synth_design -top axis_ggx_control -part xc7z020clg400-1 -mode out_of_context
create_clock -period 5.000 -name clk [get_ports s00_axis_aclk]
opt_design
place_design
route_design

# Text report of the 30 worst paths -- includes "Logic Levels" and
# "Data Path Delay: N (logic a (x%) route b (y%))" per path.
report_timing -setup -max_paths 30 -nworst 1 -unique_pins -path_type full -file top_paths_text.rpt
puts "PRE_PHYSOPT_WNS [get_property SLACK [lindex [get_timing_paths -setup -max_paths 1] 0]]"

# Probe: does the free post-route phys_opt (fanout replication + critical placement) help?
phys_opt_design -directive AggressiveExplore
route_design
report_timing_summary -file summary_physopt.rpt
puts "POST_PHYSOPT_WNS [get_property SLACK [lindex [get_timing_paths -setup -max_paths 1] 0]]"
puts "ANALYSIS2_DONE"
