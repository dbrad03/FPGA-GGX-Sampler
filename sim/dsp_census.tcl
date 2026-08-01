create_project -in_memory -part xc7z020clg400-1
source [file join [file dirname [info script]] rtl_sources.tcl]
read_verilog -sv $RTL_SOURCES(axis_ggx_control)
synth_design -top axis_ggx_control -part xc7z020clg400-1 -mode out_of_context
array set c {}
foreach cell [get_cells -hier -filter {REF_NAME =~ DSP48*}] {
  set p [get_property PARENT $cell]
  set parts [split $p /]
  set key [join [lrange $parts 0 1] /]
  if {[info exists c($key)]} { incr c($key) } else { set c($key) 1 }
}
foreach k [lsort [array names c]] { puts "DSPCENSUS $c($k) $k" }
puts "CENSUS_DONE"
