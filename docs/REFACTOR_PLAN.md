# Plan: close one lane at 200 MHz

The single objective of this branch (`refactor-dsp-quantize`). Everything past a closed lane —
multi-lane expansion, Zybo bring-up, the BVH engine — is out of scope and deliberately unplanned:
all of it is premised on a 200 MHz Lane existing, and its numbers should be re-derived once one does.

Background and the reasoning behind the Q-format, DSP-reduction and fold decisions already landed:
[handoff.md](./handoff.md). Vocabulary: [CONTEXT.md](../CONTEXT.md).

## Definition of done

**OOC post-route WNS ≥ 0 at 5 ns on `xc7z020clg400-1`, without `phys_opt`, with the full test cascade
green.**

Each clause is load-bearing:

- **Post-route, not synth.** OOC synthesis timing is badly optimistic on this design.
- **No `phys_opt`.** Every measurement on this branch was taken without it, so the closing number stays
  comparable to the −1.454 we start from. `phys_opt` is also recorded as ineffective above −0.5 ns here.
- **Full cascade green**, including `test_integration_dual` and the new stream-integrity gate (A).
- **OOC closure is not in-context closure.** A lane that closes out-of-context has not been shown to
  close inside a block design with AXI DMA and the PS. That is the next milestone, not part of this one.

## Starting point

WNS −1.454, and two distinct walls:

1. **−1.454, 72% route, per-sample.** `u_skid_proj` FIFO read → `u_reproject_normalize`. reproject is
   the biggest block in the design (19684 cells) smeared across 4 clock regions.
2. **−1.235, logic-bound, per-burst.** `u_basis` t2a/t2b band, CARRY4 = 5 from the wide t1b shift.
   Floorplanning cannot help this one.

## Order of work

**A → B → C → D.** A is first because it is the *instrument*: B and C both justify themselves with
"bit-identical", and that claim currently rests on a test that structurally cannot see the sampler.
D is not bit-identical at all, so A is the only correctness evidence it will have.

---

## A. Stream-integrity gate

`control_ref_sequence` (`sim/test_ggx_control.py:356`) is a complete end-to-end model — it drives
sobol → scramble → hash itself from `(seed, index)` — and it is **never called**. Meanwhile
`test_ggx_control`'s oracle is built by a monitor tapping the reproject input *inside the DUT*, so it
checks reproject and norm3 math against whatever arrives and never that the u1/u2 sequence is correct.
A passing `test_ggx_control` is not evidence the sampler works.

**Add, do not replace.** Two gates with two failure modes:

- **Bias gate** (existing, `reproj_input_model`): stays, because it can only hold `mean_signed < 8e-6`
  by consuming the DUT's own t1/t2/basis.
- **Stream-integrity gate** (new, `control_ref_sequence`): true black box, no DUT taps. Its job is to
  prove the Sample sequence, its index order, and its Burst boundaries — skew, mis-pairing and
  cross-burst leakage all move `h` by O(0.1–1), far above any float-model error.

Assert on `h` at a tolerance **measured from a clean run and set 5–10× above it** (expected low 1e-3).
Document it in-line as a stream-integrity guard, **not** an accuracy budget. Do not reuse the existing
`TOL = 6.5e-2` — it is arbitrary and would pass through large real corruption.

**Done when:** the new gate fails if the scramble `SIDEBAND_DEPTH` is perturbed by ±1 (verify by
temporarily reintroducing the skew), and passes on HEAD.

---

## B. Latency package

See [ADR-0002](./adr/0002-latency-package-is-law.md). Verified working on Icarus 13.0.

1. Add `hdl/ggx_latency_pkg.sv` — latencies as functions of module parameters.
2. **Producers derive from it too**, not just consumers. A consumers-only package is documentation,
   and documentation is what drifted.
3. Add `initial ... $fatal` depth checks comparing data-path depth against sideband depth as
   belt-and-braces.
4. Delete `NORM_LATENCY` (unread) and `INVSQRT_LATENCY` (unread, and wrong for the folded engine) from
   `axis_ggx_event_basis.sv`.
5. Centralize the 20 cocotb runner source lists into `sim/sources.py`; patch the 8 Vivado Tcl lists
   by hand.

**Done when:** full cascade green and `test_ggx_control` output is byte-identical to the pre-B
baseline. This step changes no arithmetic; if a byte moves, it is a bug.

---

## C. event_basis handshake rebuild

`stage_2a_ready = stage_2b_ready || !s2a_valid_reg` (`axis_ggx_event_basis.sv:324`) — the
`!s2a_valid_reg` term bypasses `pipe_en`. The stage-2 `always_ff` (line 353) is ungated by `pipe_en`,
so with `pipe_en=0, s2a_valid=1, s2a_valid_reg=0`: `s2b_advance` asserts, stage 2 clears `s2a_valid`,
but stage 2a (line 377, `else if (pipe_en)`) never latches it. **The payload is dropped silently.**

**Latent, not live.** `basis_out_ready = (state == ST_WAIT_BASIS)` and
`basis_in_valid = (state == ST_BASIS_REQ)` (`axis_ggx_control.sv:61,64`) mean exactly one basis item is
ever in flight and the FSM is parked in `ST_WAIT_BASIS` whenever the output is valid, so `pipe_en` is
effectively stuck at 1. It is reachable **today** from `test_ggx_event_basis` with backpressure and
multiple items, and it goes live the moment basis computation overlaps commands — which is on the
5-lane path. Because only one item is ever in flight, the bypass term also buys **zero throughput**.

Rebuild the stage 2 / 2a / 2b chain on a uniform elastic pattern (`axis_skid_buffer.sv` already
exists) so correctness is structural rather than argued.

While in here: **rename `s2a`/`s2b`**. The handshake stages and the T2 math stages (`t2a`/`t2b`,
lines 553–610) are one character apart in the same 630-line file, and the collision has already caused
confusion. Note that these are *different bands* — this rebuild does not touch the −1.235 critical path.

**Land alone, with its own P&R run.** The width de-inflation experiment was bit-identical yet regressed
WNS −1.454 → −1.903, and that was only diagnosable because nothing else moved.

**Done when:** cascade green, `test_ggx_control` byte-identical, a directed test drives backpressure
with multiple items through `test_ggx_event_basis` without loss, and P&R shows no regression.

---

## D. Timing

Ordered by leverage, then by risk. **Re-measure after each step and stop when WNS ≥ 0.**

### D1. Oct32 output encoder — the primary lever

See [ADR-0001](./adr/0001-octahedral-output-deletes-the-per-sample-normalize.md).

Octahedral projection is exactly scale-invariant, so encoding the output makes the per-sample
L2 normalize **redundant, not merely expensive**. `u_norm_h` (`axis_ggx_reproject_normalize.sv:749`) —
a ~165-cycle `axis_fixed_norm3` wrapping sqrt plus the 116-stage split divider, and the block holding
WNS — is **deleted**, replaced by one reciprocal of `|x|+|y|+|z|`.

- Width **Oct32 (2×16b)**, not the Oct16 the old plan named: 8-bit fields quantize to ~0.5–1°, a large
  fraction of the GGX lobe at α = 0.12.
- The divide **reuses `axis_fixed_div`** narrowly parameterized (~18–20b quotient). 0 DSP, already
  verified, already split into SUB+SELECT stages, and its latency now comes from the package built in B.
- **The bias gate moves to a pre-encoder `h_unnorm` tap.** At 16-bit fields the quantization step
  swamps 8e-6. The encoder gets its own dedicated unit test against a Python model.
- Open, and deliberately deferred to implementation because it does not affect closure: how Oct32
  samples pack into AXIS beats (one 32b sample per beat, or two packed into 64b for DMA efficiency).

**This step is not bit-identical.** The bias gate and the stream-integrity gate from A are its only
correctness evidence.

### D2. Floorplan — fallback

Only if D1 leaves the per-sample path short. Zero RTL risk; the project is already built and
synthesized: `vivado/ggx_floorplan/ggx_floorplan.xpr`, see `vivado/FLOORPLAN_GUIDE.md`. Pin reproject
and its skid into X1Y1+X1Y2 so the FIFO read stops crossing the die. Note D1 may leave a much smaller
block that no longer needs pinning.

### D3. CORDIC normalize — second fallback

`hdl/axis_cordic_normalize.sv` is written, has a bit-accurate Python reference, and is unintegrated
(commit `f3a6266`). Largely mooted by D1 for the reproject path — it makes the L2 normalize cheaper,
where D1 removes the need for it — but it remains available if a normalize survives anywhere on the
per-sample path.

### D4. t2a/t2b band — independent of D1–D3

The −1.235 logic-bound wall in `u_basis` (CARRY4 = 5 from the wide t1b shift). Untouched by any of the
above. Either pipeline the band or fold the t2a/t2b math — it is **per-burst**, so latency is nearly
free and folding is available. Must clear this wall regardless of how the per-sample path closes.

---

## How to measure

**Tests** — from `sim/`, in `.venv`, cocotb 1.9.2: `python test_<name>.py`. Full cascade after any RTL
change:

```
test_fixed_sqrt → test_fixed_inv_sqrt_nodsp → test_fixed_inv_sqrt_folded
  → test_ggx_event_basis → test_ggx_reproject_normalize
  → test_integration_dual      # per-index sampler scoreboard
  → test_ggx_control           # bias gate + stream-integrity gate
```

`test_integration_dual` is not optional. It was, until the new gate in A exists, the only per-index
sampler scoreboard in the project.

**Timing** — OOC full place-and-route only: `sim/phase0_enum.tcl` or `sim/impl_breakdown.tcl`, ~10 min,
no `phys_opt`. Reports land in `sim/timing_summary_impl.rpt`, `viol_breakdown.rpt`,
`phase0_logic_vs_route.rpt`. These and the checkpoints are gitignored — regenerate them.

**One change per P&R run.** Attribution is the whole reason the width de-inflation regression was
diagnosable.
