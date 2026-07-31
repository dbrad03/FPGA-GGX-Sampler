# Synthesize and dump ALL sub-zero-slack timing paths for datapath survey.
create_project -in_memory -part xc7z020clg400-1

source [file join [file dirname [info script]] rtl_sources.tcl]
read_verilog -sv $RTL_SOURCES(axis_ggx_control)
synth_design -top axis_ggx_control -part xc7z020clg400-1 -mode out_of_context
create_clock -period 5.000 -name clk [get_ports s00_axis_aclk]

# Up to 4000 worst violating paths, one per endpoint, so wide buses collapse.
report_timing -delay_type max -max_paths 4000 -nworst 1 \
  -slack_lesser_than 0.0 -sort_by slack -file violations_all.rpt

report_timing_summary -file timing_summary.rpt
