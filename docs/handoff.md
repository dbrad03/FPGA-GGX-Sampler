# FPGA-GGX-Sampler — refactor handoff (branch `refactor-dsp-quantize`)

Snapshot for restarting with fresh context. Captures the reasoning behind the Q-format /
DSP-reduction / timing-closure decisions on this branch — the part that doesn't survive in the diff.

## Goal (staged)
1. **Close ONE lane at 200 MHz** (5 ns) on xc7z020clg400-**1**, then bring it up on real Zybo Z7-20
   silicon. 200 MHz is reachable — the fabric/DSP/BRAM handle it; the failing paths are ~4.5–5 ns of
   *logic + routing*, i.e. a pipelining/placement problem, not a device ceiling.
2. **Then scale to 5 lanes** (1 Gsps) as a 3-way bin-pack over DSP(220)/LUT(53k)/BRAM(140) — possibly
   *heterogeneous* lanes (some DSP, some DSP-free fabric) to balance across the FPGA's columns.

## Pipeline (top = `axis_ggx_control`)
`u_basis` (event_basis, **PER-BURST** — computed once, then N samples stream) → then the **PER-SAMPLE**
chain: `u_sampler` (sobol → scramble → hash) → `u_projected_area` → `u_reproject_normalize`.
Per-burst vs per-sample is the key axis: per-burst latency is ~free (can serialize/fold); per-sample
must stay pipelined at II=1.

## Q-format decisions (and WHY)
- **sqrt (`axis_fixed_sqrt`)**: input UQ0.32, output UQ0.32 = `floor(sqrt(x)·2^32)` emitted directly
  (the old "Q1.31 / root<<15" header comments were stale — verified against perfect squares:
  x=k² → out=k<<16). Non-restoring recurrence (add/sub mode = registered remainder sign; root bit =
  result sign; no compare, no restore mux). **`SIG_BITS=24`** (was 32): the root is MSB-first so
  24 significant bits = `floor(sqrt(x)·2^24)`, left-aligned back into UQ0.32. Bias ~−3e-8, negligible
  vs the 8e-6 gate. Why: shrinks the per-iteration remainder add (the timing binder) and area.
- **div (`axis_fixed_div`)**: non-restoring long division, quotient Q7.25 (WIDTH=32, FRAC_BITS=25 →
  57 steps). Each step **split into SUB + SELECT** stages (isolates the WIDTH+1 subtract's carry chain
  from the borrow-mux/quotient-bit), latency 59→116, bit-identical. Why: the fused step was the binder.
- **inv_sqrt (`axis_fixed_inv_sqrt_nodsp`)** = S/sqrt(x) with S=0.25: sqrt then divide 0.25/sqrt(x) →
  Q7.25. Sideband (x/y/z/shift) rides two exact-match delay lines matching sqrt+div latency.
- **Quantize-to-1-DSP (the DSP-count lever: 139→85 DSP)**: a full 32×32 signed multiply is a 4-DSP
  cascade whose A→PCOUT delay is 4.21 ns with **zero logic levels** → cannot be pipelined under 5 ns,
  must be narrowed. Truncate operands to **≤18b × ≤25b** so the product fits ONE DSP48E1 (Vivado fills
  AREG/MREG/PREG automatically, no fabric cascade). Q1.31 → Q1.17 (18b) and Q1.24 (25b) → **Q2.41**
  product. `mul_pre_*` helpers pre-round+narrow the operands into a register one stage upstream so the
  multiply is a clean reg→DSP path (no rounding carry chain fused into the multiply cycle).
- **Round-half-up (`rnd_s18/s24/s25/u18/u25`)**: plain bit-slicing truncates toward −∞ → the quant
  error is **systematically BIASED, not zero-mean**, and bias does NOT average out under Monte-Carlo
  (noise does). Fix = add the MSB of the dropped field (an increment of the narrowed value + a
  top-of-range equality guard, NOT a full-width pre-add — keeps the DSP path short). **The real
  accuracy gate is `mean_signed` < BIAS_TOL 8e-6, not `max_err`** (max_err is blind to bias). NOTE the
  0.065 tolerance in some tests is arbitrary (Dylan picked it) — never cite it as a safety margin.
- **Width de-inflation (Q2.62→Q2.41, Q8.56→Q8.34)**: the `<<<21`/`<<<22` products carried 21/22
  guaranteed-zero low bits. Carrying the natural 43-bit product and re-indexing the downstream slices
  is accuracy-neutral. DONE + committed for norm3 lensq, reproject **b-section**, event_basis
  z2/t2a, projected_area. **CAUTION (learned this session):** applying it to the reproject **a-section**
  + event_basis **t1b** was bit-identical but **REGRESSED timing** (WNS −1.454 → −1.903) — on this
  congestion-bound design the "dead" register bits are load-bearing for placer spreading; removing them
  densifies and reroutes worse. That attempt was **reverted**. Width reduction is a 5-lane *area* lever,
  not a single-lane *timing* lever here.
- **Constant multiplies (hash/scramble) MUST NOT be truncated**: these are hash arithmetic — every
  input bit must reach every output bit (avalanche), quantizing biases sample PLACEMENT. Done as a
  **bit-exact 16-bit split**: `x·C = x_lo·C_lo + ((x_lo·C_hi + x_hi·C_lo)<<16) mod 2^32` (three 16×16
  partial products = 1 DSP each, no cascade; `cmul_pp` lands in the existing mult reg slot, `cmul_sum`
  next stage → no added latency). `×5` strength-reduced to `x+(x<<2)`. Scramble `cmul_sum` later
  carry-split into its own reg ahead of the XOR (SIDEBAND_DEPTH grew to keep VALID_DATA_OFFSET=−4).

## THE FOLD (this session's main change — being committed now)
`event_basis` was **50% of the design's cells smeared across all 6 clock regions** because it held two
fully-unrolled `inv_sqrt_nodsp` (sqrt + 116-stage split div + two 100-bit sideband delay lines) in a
block that runs **once per burst**. That over-provisioning was the design's dominant route-bound
congestion (`normalize_warped_view` = 2184 failing endpoints @ 85% route).
- **New `hdl/axis_fixed_inv_sqrt_folded.sv`**: sequential resource-shared engine — the SAME sqrt(24) +
  div(57) recurrences run one iteration/cycle over a single small datapath (an FSM). Single subtract
  per loop (trivially meets 5 ns); the sideband just waits in one register so the delay lines vanish;
  ~75× fewer FFs. Per-iteration arithmetic copied bit-for-bit from `axis_fixed_sqrt(SIG_BITS=24)` +
  `axis_fixed_div(32,25)` → **bit-identical**. `s00_axis_tready` low while busy (~85 cyc).
- **`axis_fixed_norm3`**: new param `FOLD_INVSQRT` (0 = pipelined for per-sample reproject; 1 = folded)
  chosen via generate. `event_basis` uses folded for BOTH its inv_sqrt (norm3 + standalone);
  `reproject`'s norm3 stays pipelined (per-sample, needs throughput).
- **VERIFIED bit-identical** (not untested): folded unit test + `test_ggx_event_basis` +
  `test_ggx_reproject_normalize` (unaffected) all green, and **`test_ggx_control` is BYTE-identical to
  the pre-fold baseline** (max|err| 1.8e-5, mean_signed +9.7e-7/−2.8e-6/−1.1e-6). event_basis running
  once per burst hides the folded latency (still 480 outputs).
- **P&R impact (OOC, 5 ns, no phys_opt):** WNS −1.344 → −1.454 (staircase — see below), but
  TNS −1176 → −569 (−52%), failing eps 4069 → 2265 (−44%), CARRY4 2862 → 1772 (−38%), `u_basis`
  26591 cells (6 regions) → 15785 (2 regions, un-smeared). A structural/congestion/area win.

## What's VERIFIED vs LEFT
- **Verified:** entire cascade bit-identical after the fold. Accuracy gate held at every step.
- **State:** working tree being committed (fold + folded.sv + tests). Prior width-de-inflation
  experiment reverted (reproject back to HEAD, event_basis fold-only).
- **WNS now −1.454; new walls (the staircase):**
  1. **WNS holder −1.454, 72% ROUTE (per-sample):** `u_skid_proj` FIFO read → `u_reproject_normalize`
     Q2.62 square DSP. reproject is now the biggest block (19684 cells) smeared across 4 regions.
     → **FLOORPLAN target.** A Vivado project is ready: `vivado/ggx_floorplan/ggx_floorplan.xpr`
     (OOC, 5 ns, fold included, synthesized). See `vivado/FLOORPLAN_GUIDE.md`. Pin reproject (+ its
     skid) into X1Y1+X1Y2 so that read stops crossing the die.
  2. **−1.235, LOGIC-bound (per-burst):** `u_basis` t2a/t2b band (CARRY4=5 from the wide t1b shift).
     Floorplan won't help this one — needs pipelining (or fold the t2a/t2b math, since it's per-burst).
- **Then:** Phase 2 = real bitstream + Zybo bring-up (NO `create_bd.tcl` exists — the tracked
  `vivado/design_1_wrapper.bit` is the OLD pre-refactor design; reuse the `timing-recovery-loop`
  repo's `create_bd.tcl` + PYNQ Overlay pattern). Phase 3 = 5-lane resource-vector bin-pack.

## How to measure / verify
- **Tests** (from `sim/`, `.venv`, cocotb 1.9.2): `python test_<name>.py`. Cascade after any RTL change:
  `test_fixed_sqrt` → `test_fixed_inv_sqrt_nodsp` (and `test_fixed_inv_sqrt_folded`) →
  `test_ggx_event_basis` → `test_ggx_reproject_normalize` → **`test_ggx_control`** (the byte-identical
  accuracy+bias gate).
- **Timing** = OOC full place+route only (`sim/phase0_enum.tcl` or `sim/impl_breakdown.tcl`, ~10 min,
  **no phys_opt** — it's ineffective above −0.5 ns here). OOC *synth* timing is badly optimistic on
  this design — always trust post-route. Reports land in `sim/timing_summary_impl.rpt`,
  `viol_breakdown.rpt`, `phase0_logic_vs_route.rpt`. (These + checkpoints are gitignored; regenerate.)
