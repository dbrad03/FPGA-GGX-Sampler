# Measurement flow for the timing regression harness (sim/harness.py).
#
# Identical in substance to impl_breakdown.tcl / phase0_enum.tcl -- OOC synth +
# opt/place/route at 5 ns, no phys_opt -- but it prints every number the harness
# records on stdout in one machine-readable form instead of leaving them to be
# read off a report by eye. Those scripts stay as they are: they exist to be read
# by a human mid-investigation, this one exists to be parsed.
#
# Contract with harness.py: every recorded quantity is printed as
#   GGXMETRIC <key> <value>
# and the run is only considered complete when HARNESS_PNR_DONE is printed.
# Vivado exits 0 on plenty of failures, so the sentinel is what proves the flow
# reached the end rather than dying after route_design.
#
# DSP and CARRY4 come from cell queries, because the query IS what the number
# means and `get_cells -hier -filter` does not move between Vivado releases.
# LUTs do not: "Slice LUTs" counts SITES, so a packed LUT5 pair is one site but
# two cells, and the cell query reports ~300 more than every LUT figure ever
# quoted for this design (9159 vs 8862 on the same netlist). Slice LUTs is both
# what competes for the device and what the record is in, so it is taken from
# the report -- guarded by the regexp below, which fails loudly if that table
# ever stops looking like itself.
#
# The design $readmemh's ggx_trig_rom.mem by BARE NAME, so it resolves against
# the CWD Vivado was launched in, not against hdl/. harness.py stages the .mem
# files into the run directory for exactly this reason -- and the check below is
# what makes a staging mistake loud. Vivado only warns on a missing $readmem
# file: it leaves the ROM uninitialized, constant-folds the whole LUT away, and
# then reports a perfectly plausible WNS for a design ~24 DSPs lighter than the
# real one. That is the single most expensive failure this harness exists to
# prevent, so it is checked here rather than left to the DSP-delta guard, which
# has nothing to compare against on a first row.

create_project -in_memory -part xc7z020clg400-1

source [file join [file dirname [info script]] rtl_sources.tcl]

# $readmemh resolves against the CWD, so require the ROM here, where the CWD
# actually is. rtl_sources.tcl's check proves the ROM was generated; this one
# proves the run can read it.
foreach rom {ggx_trig_rom.mem} {
  if {![file exists $rom]} {
    error "missing ./$rom in [pwd] -- \$readmemh resolves against the CWD, and\
           Vivado would only WARN and constant-fold the ROM away"
  }
}

read_verilog -sv $RTL_SOURCES(axis_ggx_control)
synth_design -top axis_ggx_control -part xc7z020clg400-1 -mode out_of_context
create_clock -period 5.000 -name clk [get_ports s00_axis_aclk]

opt_design
place_design
route_design
# No phys_opt_design -- see the note in impl_breakdown.tcl. Every row in the
# history file must come from the same flow or the trend is meaningless.

report_timing_summary -file timing_summary_harness.rpt
report_utilization -file util_harness.rpt
set util_text [report_utilization -return_string]

# WNS, TNS and the failing endpoint count come from the Design Timing Summary --
# Vivado's own published figures, and the ones every existing note about this
# design quotes. Summing per-path slacks in Tcl instead looks equivalent and is
# not: it accumulates values already rounded to 3 decimals, which came out
# 0.032 ns off Vivado's TNS over 1852 endpoints. A harness whose numbers are
# almost the tool's is worse than useless for comparing against the record.
#
# The table's data row is 12 numeric columns and the design-level summary is the
# first such row in the report, ahead of the per-clock breakdown:
#   WNS  TNS  TNS-failing  TNS-total  WHS  THS  THS-failing  THS-total  WPWS ...
set summary [report_timing_summary -return_string]
if {![regexp -line \
      {^\s*(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(\d+)\s+(\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(\d+)\s+(\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(\d+)\s+(\d+)\s*$} \
      $summary -> wns tns failing total_endpoints whs ths thsfail thstotal wpws tpws pwsfail pwstotal]} {
  error "report_timing_summary printed no Design Timing Summary data row --\
         either the design has no constrained paths, or the report layout changed"
}

proc ggx_count {pattern} {
  set cells [get_cells -quiet -hier -filter "REF_NAME =~ $pattern"]
  return [llength $cells]
}

set dsp    [ggx_count {DSP48*}]
set carry4 [ggx_count {CARRY4*}]

# "| Slice LUTs | 8862 | ..." -- see the site-vs-cell note in the header.
if {![regexp {\|\s*Slice LUTs\s*\|\s*(\d+)\s*\|} $util_text -> lut]} {
  error "report_utilization printed no 'Slice LUTs' row -- its table layout\
         changed, and the LUT column would silently stop being comparable"
}
set ramb36 [ggx_count {RAMB36*}]
set ramb18 [ggx_count {RAMB18*}]
# A RAMB18 is half a Block RAM Tile, which is the unit utilization reports and
# the datasheet budget are both in.
set bram [expr {$ramb36 + 0.5 * $ramb18}]

puts "GGXMETRIC wns $wns"
puts "GGXMETRIC tns $tns"
puts "GGXMETRIC failing_endpoints $failing"
puts "GGXMETRIC dsp $dsp"
puts "GGXMETRIC lut $lut"
puts "GGXMETRIC bram [format %.1f $bram]"
puts "GGXMETRIC carry4 $carry4"
puts "HARNESS_PNR_DONE"
