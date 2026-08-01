# Smoke-test the T16 survey's Tcl against a saved routed checkpoint (~1 min),
# instead of discovering a wrong property name 12 minutes into a place-and-route.
#
#     vivado -mode batch -source sim/survey_check.tcl
#
# The checkpoint is an OLD netlist (2026-07-27, pre-Oct32), so the NUMBERS it
# prints are meaningless and must never be quoted or recorded. What it proves is
# that every property, filter and regexp the survey uses still resolves against
# this Vivado. That is the part that broke: DATAPATH_ROUTE_DELAY does not exist
# on a timing_path in 2025.1, and one 12-minute run was spent finding out.

set dcp [file join [file dirname [info script]] phase0_routed.dcp]
if {![file exists $dcp]} {
  error "no $dcp to check against -- regenerate with sim/phase0_enum.tcl, or\
         run the real thing: python sim/harness.py survey"
}
open_checkpoint $dcp

puts "SURVEY_CHECK: numbers below are from a STALE netlist -- mechanics only."
source [file join [file dirname [info script]] survey_body.tcl]
