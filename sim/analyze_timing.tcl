# Detailed failing-path analysis: per-module buckets + top-path logic/route split.
create_project -in_memory -part xc7z020clg400-1

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
read_verilog { ../hdl/axis_pre_ggx_sampler.v }

synth_design -top axis_ggx_control -part xc7z020clg400-1 -mode out_of_context
create_clock -period 5.000 -name clk [get_ports s00_axis_aclk]
opt_design
place_design
route_design

report_timing_summary -file analyze_summary.rpt

# Per-module bucket (2 levels), dedup by endpoint pin, worst slack + count.
set paths [get_timing_paths -setup -max_paths 40000 -slack_lesser_than 0.0 -nworst 1 -unique_pins]
puts "TOTAL_FAILING [llength $paths]"
array unset cnt
array unset wns
foreach p $paths {
  set ep [get_property ENDPOINT_PIN $p]
  set nm [get_property NAME $ep]
  set sl [get_property SLACK $p]
  set parts [split $nm "/"]
  set key [join [lrange $parts 0 1] "/"]
  if {![info exists cnt($key)]} { set cnt($key) 0; set wns($key) $sl }
  incr cnt($key)
  if {$sl < $wns($key)} { set wns($key) $sl }
}
set fh [open module_buckets.rpt w]
puts $fh [format "%-8s %-10s %s" "ENDPTS" "WNS(ns)" "MODULE"]
foreach key [lsort -real -index 1 -decreasing [lmap k [array names cnt] {list $k $cnt($k)}]] {
  set k [lindex $key 0]
  puts $fh [format "%-8d %-10.3f %s" $cnt($k) $wns($k) $k]
}
close $fh

# Top 25 worst setup paths: slack, logic levels, logic vs route delay.
set fh2 [open top_paths.rpt w]
set tp [get_timing_paths -setup -max_paths 25 -nworst 1 -unique_pins]
puts $fh2 [format "%-8s %-6s %-9s %-9s %-6s  %s -> %s" "SLACK" "LVLS" "LOGIC" "ROUTE" "ROUTE%" "START" "END"]
foreach p $tp {
  set sl [get_property SLACK $p]
  set ll [get_property LOGIC_LEVELS $p]
  set ld [get_property DATAPATH_LOGIC_DELAY $p]
  set rd [get_property DATAPATH_ROUTE_DELAY $p]
  set tot [expr {$ld + $rd}]
  set rp [expr {$tot > 0 ? 100.0*$rd/$tot : 0}]
  set st [get_property STARTPOINT_PIN $p]
  set en [get_property ENDPOINT_PIN $p]
  puts $fh2 [format "%-8.3f %-6d %-9.3f %-9.3f %-6.1f  %s -> %s" $sl $ll $ld $rd $rp [get_property NAME $st] [get_property NAME $en]]
}
close $fh2
puts "ANALYSIS_DONE"
