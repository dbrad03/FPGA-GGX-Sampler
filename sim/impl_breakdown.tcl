# Full P&R + per-module breakdown of all failing endpoints.
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
  ../hdl/axis_fixed_inv_sqrt_folded.sv
  ../hdl/axis_sobol2d_stateless.sv
  ../hdl/axis_nested_uniform_scramble.sv
  ../hdl/axis_hash_combine_2d.sv
  ../hdl/axis_trig_lut.sv
}
read_verilog -sv { ../hdl/axis_pre_ggx_sampler.v }

synth_design -top axis_ggx_control -part xc7z020clg400-1 -mode out_of_context
create_clock -period 5.000 -name clk [get_ports s00_axis_aclk]
opt_design
place_design
route_design
# NOTE: phys_opt_design is ineffective at the current slack (Vivado: "most
# effective when WNS above -0.5ns"; it churns placement and worsened TNS here).
# Keep it OUT of the measurement flow for clean trend comparison; re-enable it
# only for final bitstream sign-off once WNS is near closure.

report_timing_summary -file timing_summary_impl.rpt

# Bucket every failing setup endpoint by its module hierarchy (2 levels deep),
# deduplicated by endpoint pin, and also track the worst slack per bucket.
set paths [get_timing_paths -setup -max_paths 40000 -slack_lesser_than 0.0 -nworst 1 -unique_pins]
puts "TOTAL_FAILING_PATHS [llength $paths]"
array unset cnt
array unset wns
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
    if {$sl < $wns($key)} { set wns($key) $sl }
  } else {
    set cnt($key) 1
    set wns($key) $sl
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
puts "BREAKDOWN_DONE"
