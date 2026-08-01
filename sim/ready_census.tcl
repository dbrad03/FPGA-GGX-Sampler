# How many failing endpoints does each ready/valid net actually carry?
#
# The T16 survey (issue #22, docs/surveys/t16-path-shapes.md) found that the
# worst path in each of scram0, scram1 and hash1 is the same combinational
# backpressure chain, and INFERRED from three shared startpoints that ~820
# endpoints lie on it. Inference is not measurement. This counts them.
#
#     vivado -mode batch -source sim/ready_census.tcl -tclargs <routed.dcp>
#
# `get_timing_paths -through <net>` is exact: it returns the failing paths that
# actually route through that net, one per endpoint (-nworst 1 -unique_pins), so
# the count is the endpoint count and not an estimate of one.

set dcp [lindex $argv 0]
if {$dcp eq "" || ![file exists $dcp]} {
  error "usage: vivado -mode batch -source sim/ready_census.tcl -tclargs <routed.dcp>\
         -- a harness run leaves one at sim/harness_runs/<run>/routed.dcp"
}
open_checkpoint $dcp

# The chain as the survey traced it, source to sink. There are only TWO ready
# nets in the sampler (axis_pre_ggx_sampler.v:107): scram0_in_ready serves hash0
# AND scram0, scram1_in_ready serves hash1 AND scram1 -- which is already a hint
# about why those buckets move together.
#
# Matched hierarchically, because synthesis pushes these nets down into the
# submodule that drives them (the survey found scram0_in_ready reported as
# u_sampler/scram0/scram0_in_ready). Each pattern reports how many nets it
# resolved to, so a rename shows up as a zero to investigate rather than
# vanishing out of a glob.
set NETS {
  *sqrt_t_in_ready
  *s1b_to_s2
  *s1_valid_reg_0
  *scram0_in_ready
  *scram1_in_ready
}

set all [get_timing_paths -setup -max_paths 200000 -slack_lesser_than 0.0 \
           -nworst 1 -unique_pins]
puts "READYCENSUS total_failing [llength $all]"

# Endpoints reached through ANY of the chain's nets -- deduplicated, because the
# nets are in series and the same endpoint is reached through several of them.
# Summing the per-net counts would double-count badly; the union is the number
# that matters for "how much of the design does one fix move".
array unset union_eps
array unset union_slack
foreach pat $NETS {
  set n [get_nets -quiet -hier -filter "NAME =~ $pat"]
  if {[llength $n] == 0} {
    puts "READYCENSUS pattern=$pat MISSING -- matched no net in this netlist"
    continue
  }
  set paths [get_timing_paths -setup -max_paths 200000 -slack_lesser_than 0.0 \
               -nworst 1 -unique_pins -through $n]
  set worst 0.0
  foreach p $paths {
    set epn [get_property NAME [get_property ENDPOINT_PIN $p]]
    set union_eps($epn) 1
    set s [get_property SLACK $p]
    set union_slack($epn) $s
    if {$s < $worst} { set worst $s }
  }
  puts [format "READYCENSUS pattern=%s nets=%d endpoints=%d worst=%.3f names={%s}" \
        $pat [llength $n] [llength $paths] $worst [join [lsort [get_property NAME $n]] " "]]
}

# How much of the design's TNS these endpoints carry -- the number that sizes
# the ticket. TNS is the sum of per-endpoint worst slack, so summing the union's
# slacks gives exactly the share one fix could remove.
set union_tns 0.0
foreach {ep s} [array get union_slack] { set union_tns [expr {$union_tns + $s}] }
set total_tns 0.0
foreach p $all { set total_tns [expr {$total_tns + [get_property SLACK $p]}] }
puts [format "READYCENSUS union_endpoints %d of %d (%.1f%%)" \
      [array size union_eps] [llength $all] \
      [expr {100.0 * [array size union_eps] / [llength $all]}]]
puts [format "READYCENSUS union_tns %.3f of %.3f (%.1f%%)" \
      $union_tns $total_tns [expr {$total_tns < 0 ? 100.0 * $union_tns / $total_tns : 0}]]
puts "READY_CENSUS_DONE"
