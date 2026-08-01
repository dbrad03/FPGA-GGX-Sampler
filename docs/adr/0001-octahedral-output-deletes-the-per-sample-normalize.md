# Octahedral output deletes the per-sample normalize

Octahedral encoding is exactly scale-invariant — `p = v/‖v‖₁` gives `oct(cv) = oct(v)` for any `c > 0`,
and the `z < 0` fold is a pure function of `p`. The per-sample pipeline currently ends by
L2-normalizing the reconstructed half-vector (a ~165-cycle `axis_fixed_norm3` wrapping a digit-recurrence
sqrt and a 116-stage split divider), which is the largest block in the design and the holder of worst
negative slack. If the output contract is octahedral, that normalize is not merely expensive but
**redundant**: `oct(h_unnorm)` and `oct(h_unnorm/‖h_unnorm‖₂)` are identically equal. We therefore
adopt an octahedral output and delete the per-sample normalize, replacing it with one narrow reciprocal
of `|x|+|y|+|z|`.

## Considered options

- **Floorplan the normalize** — pin it and its feeding skid buffer into adjacent clock regions so its
  critical read stops crossing the die. Zero RTL risk and already set up, but it optimizes the
  placement of a block that does not need to exist. Retained as a fallback.
- **Swap in the CORDIC normalize** (`hdl/axis_cordic_normalize.sv`, written and unintegrated) — makes
  the L2 normalize dividerless and much smaller. Strictly weaker than deleting it. Retained as a
  second fallback.
- **Keep both output modes behind a parameter** — preserves the full-precision contract and the
  existing bias gate, at the cost of maintaining and synthesizing two tails. Rejected: the lane we are
  trying to close would then be only the encoded configuration anyway.

## Consequences

- **The width is Oct32 (2×16b), not the Oct16 (2×8b) named in the original plan.** 8-bit fields
  quantize direction to roughly 0.5–1°, a large fraction of the GGX lobe at the smallest roughness we
  test (α = 0.12), which would distort the sampled distribution. Oct32 is ~0.002° and still reduces the
  output from 128 to 32 bits — enough for the DDR3 bandwidth argument that motivated encoding in the
  first place. Scale-invariance is what deletes the normalize; the field width is independent of it.
- **This step is not bit-identical**, unlike every prior step on this branch. Byte-comparison against a
  stored baseline stops being available as evidence here, which is why the stream-integrity gate must
  exist before this work starts.
- **The bias gate moves off the output boundary.** At 16-bit fields the quantization step swamps the
  8e-6 threshold, so the gate is asserted on the pre-encoder half-vector via a hierarchical tap, and
  the encoder is checked separately by its own unit test. This is a deliberate loss of black-box purity
  in exchange for keeping the project's sharpest instrument measurable.
- Octahedral encoding moves out of the long-term roadmap and into the timing-closure plan. A future
  reader finding a "compression" feature ranked ahead of floorplanning should read it as a
  normalize-elimination change that happens to also save bandwidth.

## Outcome (annotated 2026-08-01, after implementation)

**The timing premise of this ADR was wrong, and is recorded here so the next reader does not trust it
the way its author did.**

This ADR argued the per-sample normalize was "the largest block in the design and the holder of worst
negative slack", implying its deletion would help close the Lane. It was deleted. Measured post-route,
out-of-context, 5 ns, no phys_opt:

| | before | after |
|---|---|---|
| WNS | −1.178 | **−1.330** |
| TNS | −543 | −407 |
| failing endpoints | 2081 | 1852 |

Deleting the largest block **cost 0.15 ns of worst negative slack.** TNS and endpoint count improved,
so the change reduced the bulk of the violation, but it did not do the thing it was adopted to do. The
worst path simply reverted to `u_reproject_normalize`'s a-section DSP multiply — the same path that
held it before — and got worse from added congestion.

Two reasons, both worth carrying forward:

- **The replacement is not free.** A ~151-cycle normalize was swapped for an encoder containing two
  78-stage dividers on the per-sample path. Fewer DSPs and fewer LUTs, but not obviously easier to
  place.
- **Area reduction is not a timing lever on this design.** This is now the third measured instance, and
  `docs/handoff.md` had already recorded the first two: the fold (75x fewer FFs, WNS −1.344 → −1.454)
  and the width de-inflation (bit-identical, −1.454 → −1.903, reverted). Removing logic from a
  congestion-bound design tends to densify it and route worse.

**What this ADR got right is unaffected.** Octahedral encoding is exactly scale-invariant, the
normalize genuinely was redundant under an Oct32 output contract, and the Oct32-over-Oct16 field-width
argument was confirmed by measurement (issue #13): 8-bit fields put p99 angular error at 7.6% of the
lobe at α = 0.12 with total-variation distance 3–4x above the sampling-noise floor, where 16-bit sits
~50x below it. The output contract shrank from 128 to 32 bits and the Lane shed 6 DSPs. The decision
stands on bandwidth, on DSP count and on correctness.

It does not stand on timing, and it should never have been sequenced as a timing-closure measure. The
keep-or-revert call is tracked in the closure campaign, to be made when 0.15 ns is visible against a
near-closed design rather than lost inside a 1.3 ns problem.

## Decision (2026-08-01, issue #28): Oct32 is KEPT

The near-closed design that call was waiting for arrived at `e27421f` — WNS −0.301, TNS −5.789, 94
failing endpoints (`sim/timing_history.csv`). Against it:

**Oct32's entire remaining timing cost is 3 failing endpoints at −0.034**, in `u_div_x`'s carry cone.
It is not the WNS holder and it is not among the top blocks. Issue #32, which existed to split that
cone, was closed on the same measurement as not worth a place-and-route run.

Most of the 0.15 ns this ADR was charged with was never Oct32's to begin with. The a-section binder
held WNS before and after the encoder landed, and #24, #25 and #33 have since taken that binder apart
— WNS moved −1.330 → −0.301 without the encoder changing at all.

**What this decision does not rest on.** Issue #28 asked for both configurations placed and routed side
by side at the campaign's end state. That was never done. The decision rests on the cost of *keeping*
being small and measured, not on a head-to-head, and if Oct32 is ever reconsidered that measurement is
still the honest way to do it.

This annotates the ADR rather than superseding it. Everything above stands, including — especially —
the finding that its timing premise was wrong.
