# The T16 survey's analysis, over whatever routed design is already in memory.
# Sourced by survey_paths.tcl after a real place-and-route, and by
# survey_check.tcl against a saved checkpoint to smoke-test the Tcl mechanics.

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

  # The logic-vs-route split comes from the report TEXT, not from properties.
  # A timing_path exposes DATAPATH_DELAY and DATAPATH_LOGIC_DELAY but there is
  # no DATAPATH_ROUTE_DELAY in Vivado 2025.1 -- asking for one is an error, not
  # an empty string. phase0_enum.tcl says the same thing in its own comment.
  # (sim/analyze_timing.tcl still asks for it, and would die here too.)
  set txt [report_timing -of_objects $p -input_pins -return_string]
  if {![regexp {Data Path Delay:\s+([\d.]+)ns\s+\(logic\s+([\d.]+)ns\s+\(([\d.]+)%\)\s+route\s+([\d.]+)ns\s+\(([\d.]+)%\)\)} \
        $txt -> tot ld lpct rd rpct]} {
    error "could not read the logic/route split for $block -- report_timing's\
           'Data Path Delay' line changed shape"
  }

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
  puts $fh $txt
  close $fh
}

puts "HARNESS_SURVEY_DONE"
