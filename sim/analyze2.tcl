# Logic-vs-route classification of the worst paths + phys_opt_design probe.
create_project -in_memory -part xc7z020clg400-1
source [file join [file dirname [info script]] rtl_sources.tcl]
read_verilog -sv $RTL_SOURCES(axis_ggx_control)
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
