# Vivado Synthesis TCL Script for GGX Sampler Baseline
create_project -in_memory -part xc7z020clg400-1

# Source files
source [file join [file dirname [info script]] rtl_sources.tcl]
read_verilog -sv $RTL_SOURCES(axis_ggx_control)
# Run synthesis
synth_design -top axis_ggx_control -part xc7z020clg400-1 -mode out_of_context

# Create constraints for clock definition to report realistic timing
create_clock -period 5.000 -name clk [get_ports s00_axis_aclk]

# Report timing and resource usage
report_utilization -file utilization.rpt
report_timing -delay_type max -max_paths 10 -file timing.rpt
report_timing_summary -file timing_summary.rpt
