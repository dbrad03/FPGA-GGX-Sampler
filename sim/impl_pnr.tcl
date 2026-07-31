# Full place & route (OOC) to get REAL routing delays, vs synth_baseline.tcl's
# estimated routing. Same design/clock; adds opt/place/route + post-route timing.
create_project -in_memory -part xc7z020clg400-1

source [file join [file dirname [info script]] rtl_sources.tcl]
read_verilog -sv $RTL_SOURCES(axis_ggx_control)
synth_design -top axis_ggx_control -part xc7z020clg400-1 -mode out_of_context
create_clock -period 5.000 -name clk [get_ports s00_axis_aclk]

opt_design
place_design
route_design

report_utilization -file util_impl.rpt
report_timing_summary -file timing_summary_impl.rpt
report_timing -delay_type max -max_paths 15 -file timing_impl.rpt
puts "PNR_DONE"
