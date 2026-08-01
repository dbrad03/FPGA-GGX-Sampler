# T16 (issue #22): per-path SHAPE for the blocks the campaign is ordered around.
#
# The existing breakdown (impl_breakdown.tcl, phase0_enum.tcl) answers "where is
# the violation" -- endpoint counts and worst slack per module. That is not
# enough to order the campaign, because it does not say what the path is MADE OF,
# and the shape decides the fix. Two carry chains sharing a cycle is a register
# split that costs nothing; one long chain, or a path pinned by a DSP, is a
# structural change with a different price.
#
# So this run captures, per block: logic levels, the CARRY4 count ON the path,
# the logic-vs-route split, and both endpoints. Classification is deliberately
# NOT done here -- Tcl records what is measurable, a human reads the shapes and
# argues the order. See docs/surveys/.
#
# Run with:  python sim/harness.py survey
# which sources this AFTER harness_pnr.tcl's flow, so one place-and-route
# produces both the recorded row and this survey.

source [file join [file dirname [info script]] harness_pnr.tcl]

# Block name -> endpoint-hierarchy prefix. These are the five blocks the
# remaining campaign tickets are written against:
#   sampler_*         -> #23 (T17, DSP-free constant multiplies)
#   reproject_binder  -> #24 (T18, the a-section multiply holding WNS)
#   basis_normalize   -> #25 (T19, normalize_warped_view)
#   projected_area_sq -> #27 (T21, the square)
#   oct32             -> #28 (T22, keep or revert)
# The sampler is split into its four sub-buckets because #23 treats them as one
# change and the survey should say whether that is justified.
set BLOCKS {
  sampler_scram0    u_sampler/scram0
  sampler_scram1    u_sampler/scram1
  sampler_hash0     u_sampler/hash0
  sampler_hash1     u_sampler/hash1
  basis_normalize   u_basis/normalize_warped_view
  projected_area_sq u_projected_area/c0_t1_sq
  reproject_binder  u_reproject_normalize/mul_pre
  oct32             u_reproject_normalize/u_oct32
}

# One query, bucketed in a single pass, so the endpoint counts here and the
# failing-endpoint total in the recorded row can never disagree.
set paths [get_timing_paths -setup -max_paths 200000 -slack_lesser_than 0.0 \
             -nworst 1 -unique_pins]
puts "GGXSURVEY total_failing [llength $paths]"

array unset cnt
array unset worst
array unset wslack
foreach p $paths {
  set ep_name [get_property NAME [get_property ENDPOINT_PIN $p]]
  set slack [get_property SLACK $p]
  foreach {block prefix} $BLOCKS {
    if {[string match "$prefix*" $ep_name]} {
      if {![info exists cnt($block)]} {
        set cnt($block) 0
        set wslack($block) 0.0
      }
      incr cnt($block)
      if {$slack < $wslack($block)} {
        set wslack($block) $slack
        set worst($block) $p
      }
      break
    }
  }
}

set fh [open survey_paths.rpt w]
puts $fh "T16 path-shape survey -- worst failing path per block, post-route."
close $fh

foreach {block prefix} $BLOCKS {
  if {![info exists worst($block)]} {
    # Loud, not skipped. A survey that silently covers six of eight buckets
    # would re-derive the campaign order from a hole.
    puts "GGXSURVEY MISSING $block prefix=$prefix -- no failing endpoint matched"
    continue
  }
  set p $worst($block)
  set ll [get_property LOGIC_LEVELS $p]
  set ld [get_property DATAPATH_LOGIC_DELAY $p]
  set rd [get_property DATAPATH_ROUTE_DELAY $p]
  set tot [expr {$ld + $rd}]
  set rpct [expr {$tot > 0 ? 100.0 * $rd / $tot : 0.0}]

  # What the path is MADE OF -- the question the endpoint counts cannot answer.
  set cells [get_cells -quiet -of_objects $p]
  set n_carry 0
  set n_dsp 0
  set n_lut 0
  foreach c $cells {
    set ref [get_property REF_NAME $c]
    if {[string match CARRY4* $ref]} { incr n_carry }
    if {[string match DSP48* $ref]}  { incr n_dsp }
    if {[string match LUT? $ref]}    { incr n_lut }
  }

  puts [format "GGXSURVEY block=%s endpoints=%d slack=%.3f levels=%d logic=%.3f route=%.3f route_pct=%.1f carry4=%d dsp=%d lut=%d" \
        $block $cnt($block) $wslack($block) $ll $ld $rd $rpct $n_carry $n_dsp $n_lut]
  puts [format "GGXSURVEYPIN block=%s start=%s end=%s" $block \
        [get_property NAME [get_property STARTPOINT_PIN $p]] \
        [get_property NAME [get_property ENDPOINT_PIN $p]]]

  set fh [open survey_paths.rpt a]
  puts $fh "\n\n================ $block ($prefix) ================"
  close $fh
  report_timing -of_objects $p -input_pins -append -file survey_paths.rpt
}

puts "HARNESS_SURVEY_DONE"
