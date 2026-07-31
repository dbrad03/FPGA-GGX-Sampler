# Vivado Synthesis TCL Script for GGX Sampler Baseline
create_project -in_memory -part xc7z020clg400-1

# Source files
read_verilog -sv {
  ../hdl/ggx_latency_pkg.sv
  ../hdl/axis_ggx_control.sv
  ../hdl/axis_skid_buffer.sv
  ../hdl/axis_fifo_2deep.sv
  ../hdl/axis_ggx_event_basis.sv
  ../hdl/axis_top_lvl_sampler.sv
  ../hdl/axis_ggx_projected_area.sv
  ../hdl/axis_ggx_reproject_normalize.sv
  ../hdl/axis_fixed_norm3.sv
  ../hdl/axis_fixed_sqrt.sv
  ../hdl/axis_fixed_div.sv
  ../hdl/axis_fixed_inv_sqrt_nodsp.sv
  ../hdl/axis_sobol2d_stateless.sv
  ../hdl/axis_nested_uniform_scramble.sv
  ../hdl/axis_hash_combine_2d.sv
  ../hdl/axis_trig_lut.sv
}
read_verilog {
  ../hdl/axis_pre_ggx_sampler.v
}

# Run synthesis
synth_design -top axis_ggx_control -part xc7z020clg400-1 -mode out_of_context

# Create constraints for clock definition to report realistic timing
create_clock -period 5.000 -name clk [get_ports s00_axis_aclk]

# Report timing and resource usage
report_utilization -file utilization.rpt
report_timing -delay_type max -max_paths 10 -file timing.rpt
report_timing_summary -file timing_summary.rpt
