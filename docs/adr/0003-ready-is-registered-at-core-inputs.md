# Ready is registered at core inputs

AXI-Stream `tready` propagates *backwards* and *combinationally*. Every core in this design implements
it the same way — `s00_axis_tready = pipe_en = m00_axis_tready || !m00_axis_tvalid` — which is correct,
cheap, and composes into a disaster: chain three such cores and a stall at the end of the chain must
reach the beginning of it inside one clock cycle, through one LUT and one high-fanout net per hop.

That is not a hypothetical. Measured at `394ea9f` (`sim/ready_census.tcl`, issue #22, #31), one such
chain ran from `u_projected_area/d1b_valid_reg` through `sqrt_t` → `trig` → `sqrt_r`, out of
`projected_area` entirely, into the sampler, through `scram0` and into `hash0`, ending on DSP48 **`CEP`
pins** — clock enables, not data. Four LUT levels, **78% route**, the final hop 1.626 ns on a
fanout-704 net. It carried **875 of the design's 1852 failing endpoints (47.2%) and 42.9% of its TNS**,
with no arithmetic on it anywhere to pipeline.

We therefore adopt the rule: **a `tready` may cross at most one core boundary combinationally.** Where
it would cross a second, an `axis_fifo_2deep` is inserted as a *cut* — its `s_axis_tready` is a
flip-flop output, so backpressure stops there and continues from a register next cycle.

## Considered options

- **Pipeline the arithmetic on the path.** The reflex, and the campaign's default lever everywhere
  else. Inapplicable: the survey measured 4 logic levels and **zero CARRY4** on this path. There is no
  arithmetic on it. The multiplies it appears to run through are reached at their *enable* pins.
- **Register duplication / `MAX_FANOUT` alone.** Attacks the 626- and 704-load nets without touching
  the chain. Rejected as the primary fix because it shortens each hop but leaves four of them in
  series inside one cycle; it remains available as a *second* lever, and #31 asks explicitly which of
  the two bought the slack.
- **Make the cores unstallable and buffer their outputs.** Removes `pipe_en` and its fanout entirely,
  which is the actual root cause. Rejected for now: it changes the cores that the campaign has been
  holding Bit-identical, for a benefit the cut already delivers.
- **A new fully-registered register slice built from flip-flops.** `axis_fifo_2deep`'s output is an
  asynchronous read of a 2-entry array, which Vivado implements as distributed RAM — and a distributed
  RAM read in `u_skid_proj` is what currently holds WNS (#24). Rejected anyway: the project already
  uses `axis_fifo_2deep` for exactly this purpose in two places in `reproject_normalize`
  (`u_sqrt_fifo`, `u_norm_fifo`, both commented as registering the ready path), and a second primitive
  that does the same job differently is how a codebase stops being reviewable. If the LUTRAM read
  becomes a binder, replacing the primitive is one edit in one file.

## Consequences

- **Every cut costs exactly one cycle of latency.** `FIFO_2DEEP_LATENCY` and `FIFO_2DEEP_DEPTH` are in
  `ggx_latency_pkg` and `fifo_2deep_span()` folds both into the bound an elastic sideband FIFO must
  clear, per ADR-0002. A cut placed in front of a core that a sideband FIFO rides alongside *widens
  that sideband's span* — by the cut's latency and by its occupancy — and getting it wrong does not
  mis-pair by one, it overflows the ring and emits a spurious TLAST.
- **Cuts go after joins, never before them.** The sampler's cut sits on the joined `{hash, sobol}`
  pair rather than on either path, so both are delayed identically and `HASH_LATENCY`/`SOBOL_LATENCY`
  — and therefore `ALIGN_DELAY`, an exact-match delay line — are untouched. A cut on one leg of a join
  is a Skew defect.
- **One cut per fanout cone, not one shared.** The sampler could have used a single cut for both
  dimensions. Its registered ready would then drive both dimensions' hash enables, ~1400 loads on one
  net, trading a chain-depth problem for a fanout problem.
- **II=1 survives.** `axis_fifo_2deep` sustains one beat per cycle with both ports firing; its second
  entry exists precisely so that a ready which is one cycle late is still lossless. This is the
  property to check first when a cut is added: elastic buffering is exactly the change that can
  silently cost a cycle per Sample, and `test_ggx_control`'s first-Burst gap check is what catches it.
- **The rule is checkable, but not at elaboration.** "No `tready` crosses more than one core boundary"
  is a property of the routed netlist, not of the source. `sim/ready_census.tcl` measures it against a
  checkpoint; that measurement, not the source, is the evidence.
