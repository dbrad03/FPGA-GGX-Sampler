# Phase-0 baseline + full neg-slack enumeration for the GGX single-lane re-eval.
# = impl_breakdown.tcl (OOC synth+P&R at 5 ns) PLUS a logic-bound vs route-bound
# classification of the worst path in each top failing module bucket, so one P&R
# run produces the whole Phase-0 map. No phys_opt (see impl_breakdown.tcl note).
create_project -in_memory -part xc7z020clg400-1

source [file join [file dirname [info script]] rtl_sources.tcl]
read_verilog -sv $RTL_SOURCES(axis_ggx_control)
synth_design -top axis_ggx_control -part xc7z020clg400-1 -mode out_of_context
create_clock -period 5.000 -name clk [get_ports s00_axis_aclk]
opt_design
place_design
route_design

report_timing_summary -file timing_summary_impl.rpt
report_utilization -file util_impl.rpt

# Bucket every failing setup endpoint by its module hierarchy (2 levels deep),
# deduplicated by endpoint pin. Track worst slack AND the worst path object per
# bucket so we can classify it logic- vs route-bound afterwards.
set paths [get_timing_paths -setup -max_paths 40000 -slack_lesser_than 0.0 -nworst 1 -unique_pins]
puts "TOTAL_FAILING_PATHS [llength $paths]"
array unset cnt
array unset wns
array unset wpath
foreach p $paths {
  set ep [get_property ENDPOINT_PIN $p]
  set nm [get_property NAME $ep]
  set sl [get_property SLACK $p]
  set toks [split $nm /]
  if {[llength $toks] >= 2} {
    set key "[lindex $toks 0]/[lindex $toks 1]"
  } else {
    set key [lindex $toks 0]
  }
  if {[info exists cnt($key)]} {
    incr cnt($key)
    if {$sl < $wns($key)} { set wns($key) $sl ; set wpath($key) $p }
  } else {
    set cnt($key) 1
    set wns($key) $sl
    set wpath($key) $p
  }
}
set rows {}
foreach k [array names cnt] { lappend rows [list $cnt($k) $wns($k) $k] }
set rows [lsort -integer -decreasing -index 0 $rows]
set fp [open "viol_breakdown.rpt" w]
puts $fp [format "%-8s %-10s %s" "ENDPTS" "WNS(ns)" "MODULE"]
foreach r $rows {
  puts $fp [format "%-8d %-10.3f %s" [lindex $r 0] [lindex $r 1] [lindex $r 2]]
}
close $fp

# Save a routed checkpoint so logic-vs-route queries (and any what-if) can reopen
# instantly instead of re-running the ~10 min place+route.
write_checkpoint -force phase0_routed.dcp

# Logic-bound vs route-bound: the path object exposes DATAPATH_DELAY but NOT a
# logic/route split, so dump report_timing text (which prints
# "Data Path Delay: N (logic L (x%) route R (y%))") for the worst path of each of
# the top-20 module buckets. Pipelining only helps logic; only area-spread /
# floorplan helps route.
set fp2 [open "phase0_logic_vs_route.rpt" w]
close $fp2
set topn 0
foreach r $rows {
  if {$topn >= 20} { break }
  incr topn
  set k [lindex $r 2]
  report_timing -of_objects $wpath($k) -input_pins -append \
    -file phase0_logic_vs_route.rpt
}

# Also the 25 worst unique paths overall (belt-and-suspenders).
report_timing -setup -max_paths 25 -nworst 1 -unique_pins -input_pins \
  -file phase0_worst25.rpt

puts "BREAKDOWN_DONE"
