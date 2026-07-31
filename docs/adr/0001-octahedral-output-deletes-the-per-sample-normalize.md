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
