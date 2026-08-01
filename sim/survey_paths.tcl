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

# The analysis itself assumes only that a routed design is in memory, so it can
# be exercised against a saved checkpoint in ~1 minute instead of by paying for
# a 12-minute place-and-route to find out a property name was wrong:
#     vivado -mode batch -source sim/survey_check.tcl
source [file join [file dirname [info script]] survey_body.tcl]
