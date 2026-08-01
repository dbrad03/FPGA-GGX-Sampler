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
# Most resource counts come from cell queries rather than report_utilization's
# tables, because the queries are what the numbers mean and `get_cells -hier
# -filter` does not move between Vivado releases. LUTs are the exception: "Slice
# LUTs" counts SITES, so a LUT6_2 or a packed LUT5 pair is one, and a cell query
# reports ~100 more than every LUT figure ever quoted for this design. That one
# number is taken from the report so it stays comparable to the existing record.
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

# WNS: worst setup slack in the design, failing or not. -max_paths 1 without
# -slack_lesser_than so a design that closes still reports its real margin.
set worst [get_timing_paths -setup -max_paths 1]
set wns [get_property SLACK [lindex $worst 0]]

# TNS is the sum over FAILING ENDPOINTS of their worst slack, which is exactly
# one path per endpoint: -nworst 1 -unique_pins. The same query gives the
# failing endpoint count, so the two numbers can never disagree about which
# endpoints they counted.
set paths [get_timing_paths -setup -max_paths 200000 -slack_lesser_than 0.0 \
             -nworst 1 -unique_pins]
set failing [llength $paths]
set tns 0.0
foreach p $paths { set tns [expr {$tns + [get_property SLACK $p]}] }

proc ggx_count {pattern} {
  set cells [get_cells -quiet -hier -filter "REF_NAME =~ $pattern"]
  return [llength $cells]
}

set dsp    [ggx_count {DSP48*}]
set carry4 [ggx_count {CARRY4*}]

# "| Slice LUTs | 8947 | ..." -- see the site-vs-cell note in the header.
if {![regexp {\|\s*Slice LUTs\s*\|\s*(\d+)\s*\|} $util_text -> lut]} {
  error "report_utilization printed no 'Slice LUTs' row -- its table layout\
         changed, and the LUT column would silently stop being comparable"
}
set ramb36 [ggx_count {RAMB36*}]
set ramb18 [ggx_count {RAMB18*}]
# A RAMB18 is half a Block RAM Tile, which is the unit utilization reports and
# the datasheet budget are both in.
set bram [expr {$ramb36 + 0.5 * $ramb18}]

puts "GGXMETRIC wns [format %.3f $wns]"
puts "GGXMETRIC tns [format %.3f $tns]"
puts "GGXMETRIC failing_endpoints $failing"
puts "GGXMETRIC dsp $dsp"
puts "GGXMETRIC lut $lut"
puts "GGXMETRIC bram [format %.1f $bram]"
puts "GGXMETRIC carry4 $carry4"
puts "HARNESS_PNR_DONE"
