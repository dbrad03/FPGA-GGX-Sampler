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
  **CAVEAT (2026-07-31):** that byte-identical claim validates the FOLD, not the sampler. The pre-fold
  baseline itself carried a misaligned sample stream (see "Sampler TDATA/TVALID skew" below), and
  `test_ggx_control` structurally cannot see it: the test builds its expected values from a monitor
  tapping the *reproject input inside the DUT*, so it checks reproject+norm3 math against whatever
  arrives — never that the u1/u2 sequence is correct. `control_ref_sequence` (the true end-to-end model)
  is defined in the file but **never called**. Only `test_integration_dual` scoreboards the sampler
  output per-index.
  **RESOLVED 2026-07-31 (issue #5):** `control_ref_sequence` is now wired into `test_ggx_control` as a
  second, independent Stream-integrity gate, built offline from the Command list with no DUT signal
  contributing, and demonstrated to FAIL on a ±1 scramble sideband perturbation while the Bias gate
  still passes. Note it was not merely unused — it was **wrong**: it scrambled the sobol *index*,
  which the RTL does not do, putting it O(1) away from the hardware. `test_ggx_control` can now see
  the whole Lane.
- **P&R impact (OOC, 5 ns, no phys_opt):** WNS −1.344 → −1.454 (staircase — see below), but
  TNS −1176 → −569 (−52%), failing eps 4069 → 2265 (−44%), CARRY4 2862 → 1772 (−38%), `u_basis`
  26591 cells (6 regions) → 15785 (2 regions, un-smeared). A structural/congestion/area win.
  **STALE — these figures no longer reproduce; see "Timing re-baseline" below.** They describe a
  25925-FF intermediate tree; the committed design is 18290 FF. The structural/congestion argument
  still holds, the numbers do not.

## Sampler TDATA/TVALID skew (found + fixed 2026-07-31)
The pipeline rewrites left the sideband taps deeper than the data paths, and the *module* tests then
codified the skew as a "characterised offset" instead of failing on it:
- **hash**: data 17 regs vs `valid_pipeline[17]` (18) → TDATA led TVALID by 1. Masked mid-burst because
  the hashed field is `tdata[31:0]` = `seed_base`, a **per-burst constant** — but it leaked the *next*
  burst's seed on each burst's last beat. Fix: added `final_mix_pipe[5]` (data 18 = valid 18).
- **scramble**: data 19 regs vs `SIDEBAND_DEPTH+1` = 15 → TDATA lagged TVALID by 4. This one carries
  real per-sample data, so it **corrupted the stream**: first 4 samples junk, everything shifted,
  cross-burst leakage. Fix: `SIDEBAND_DEPTH` 14 → 18 (both paths 19).
- Both `VALID_DATA_OFFSET` constants pinned to **0**. On `main` both blocks were aligned (10/10 and 7/7)
  — the "offset the system co-tunes around" never existed; it was a regression, not a design property.
- **Lesson:** a passing `test_ggx_control` was NOT evidence the sampler is correct. Still run
  `test_integration_dual` in the cascade — but as of issue #5 `test_ggx_control` carries its own
  Stream-integrity gate and would now catch this skew class itself. A ±1 perturbation of
  `SCRAMBLE_SIDEBAND_DEPTH` also fails the **build** now, via the elaboration checks from issue #10.

## What's VERIFIED vs LEFT
- **Verified:** entire cascade bit-identical after the fold. Accuracy gate held at every step.
- **State (2026-07-31):** fold committed. Since then, workstreams A and B are done — the
  Stream-integrity gate (#1/#5) and the latency package as law (#2, #6–#10). Full suite 19/19 green;
  `test_ggx_control` byte-identical to its pre-refactor baseline throughout, and post-route
  netlist-equivalent (see re-baseline below). Workstream C (#3, #11/#12) is also done: the
  event_basis stage 2/2a/2b chain is rebuilt as four uniform elastic register stages
  (`hs_zop -> hs_sq -> hs_sub -> hs_clamp`), closing a latent dropped-payload hazard that lost
  4 of 6 items under backpressure. Next in the chain is workstream D (#4, Oct32) and #17 (the
  per-burst T2 arithmetic wall, which now holds WNS) — **neither started.**
- **WNS is −2.139, not −1.454 — see "Timing re-baseline" below before planning against it.**
- **Then:** Phase 2 = real bitstream + Zybo bring-up (NO `create_bd.tcl` exists — the tracked
  `vivado/design_1_wrapper.bit` is the OLD pre-refactor design; reuse the `timing-recovery-loop`
  repo's `create_bd.tcl` + PYNQ Overlay pattern). Phase 3 = 5-lane resource-vector bin-pack.

## Timing re-baseline (2026-07-31, Vivado 2025.1)

The **−1.454 figure above does not reproduce.** Measured today with `sim/impl_breakdown.tcl`
(OOC, xc7z020clg400-1, 5 ns, no phys_opt):

| | documented (stale) | measured 2026-07-31 |
|---|---|---|
| WNS | −1.454 | **−2.078** |
| TNS | −569 | **−1653.089** |
| Failing endpoints | 2265 | **5929** |
| Total endpoints | 64989 | **49028** |
| Slice registers | 25925 | **18290** |
| LUTs / DSP / BRAM | — | 9296 / 85 / 4.5 |
| CARRY4 | 1772 | **1352** |

**This is not a regression from the latency-package work.** The same script was run on `e4dfd50`
(the commit before that work) and on the current tree: `timing_summary_impl.rpt` and
`viol_breakdown.rpt` come out **byte-identical**, WNS −2.078 on both. The package refactor is
netlist-equivalent post-route, which is stronger evidence than the bit-identical simulation output.

The stale numbers came from `sim/*_fold.rpt` / `*_baseline.rpt`, saved 07-27 17:15–17:21 — three days
*before* the fold was committed (09ffccb, 07-30 18:02). They describe a 25925-FF intermediate tree,
not what is committed. Endpoint count alone (64989 vs 49028) shows it is a different design.

**Two traps that make bad timing numbers look plausible — both now fixed, both worth knowing:**
- `sim/ggx_trig_rom.mem` is gitignored, and Vivado does **not** error on a missing `$readmem` file. It
  warns, leaves the ROM uninitialized, constant-folds the whole trig LUT away, and reports timing for
  a design 12 DSPs and 2.5 BRAMs lighter. Run `python sim/gen_roms.py` first; `rtl_sources.tcl` now
  aborts if the file is absent.
- 5 of the 8 Tcl scripts were missing `axis_fixed_inv_sqrt_folded.sv` and had failed synthesis
  outright since the fold. All 8 now take their file list from `sim/sources.py`.

### After workstream C (44def36, event_basis elastic rebuild)

Its own P&R run, nothing else in it:

| | before (031323a) | after (44def36) |
|---|---|---|
| WNS | −2.078 | **−2.139** (−0.061, a regression) |
| TNS | −1653.089 | **−1242.465** (−25%) |
| Failing endpoints | 5929 | **3249** (−45%) |

The WNS holder moved to `u_basis/t2a_x_reg` / `t2a_z0_reg` — the **T2 arithmetic band**, which that
change does not touch. Read as placement/routing churn on an unrelated path rather than causation,
which is what ADR-adjacent issue #3 predicted. It is still a regression and is recorded as one.

### The staircase, as measured at 031323a
The top five violating paths are now **all per-burst `u_basis` t1b→t2 DSP paths**, which inverts the
old priority — the reproject floorplan was wall #1, and is not the binder any more:

1. **−2.078 (per-burst):** `u_basis/t1b_x_shift_reg[32]` → `u_basis/t2b_y_r_reg` (DSP48E1 A port).
   8 logic levels, CARRY4=5, logic 2.209 ns (42%) / route 3.029 ns (58%).
2. **−1.887 (per-burst):** `t1b_y_shift_reg[32]` → `t2a_x_reg` / `t2a_z0_reg`.
3. **−1.430 (per-burst):** `t1b_x_shift_reg[32]` → `t2b_z_r_reg/D[16]`.
4. **−1.419 (per-sample):** `u_reproject_normalize` `meta0_wr_ptr` → `meta0_data` RAM write address.

`u_reproject_normalize/u_norm_h` still dominates **TNS** (2056 failing endpoints at −1.200), so
reproject is the bulk-of-violations problem while `u_basis` holds WNS.

**Why this is good news:** the WNS holder is **per-burst** work, where latency is nearly free per
`CONTEXT.md`. Pipelining or folding the t2a/t2b band costs throughput nothing, and is a far cheaper
lever than floorplanning the per-sample reproject datapath. The old wall #2 note said exactly this;
it is now wall #1.

## How to measure / verify
- **Tests** (from `sim/`, `.venv`, cocotb 1.9.2): `python test_<name>.py`. Cascade after any RTL change:
  `test_fixed_sqrt` → `test_fixed_inv_sqrt_nodsp` (and `test_fixed_inv_sqrt_folded`) →
  `test_ggx_event_basis` → `test_ggx_reproject_normalize` → **`test_integration_dual`** (the ONLY
  per-index sampler scoreboard — catches stream skew that `test_ggx_control` is blind to) →
  **`test_ggx_control`** (the accuracy+bias gate; see the caveat above — it is not end-to-end).
- **Timing** = OOC full place+route only (`sim/phase0_enum.tcl` or `sim/impl_breakdown.tcl`, ~10 min,
  **no phys_opt** — it's ineffective above −0.5 ns here). OOC *synth* timing is badly optimistic on
  this design — always trust post-route. Reports land in `sim/timing_summary_impl.rpt`,
  `viol_breakdown.rpt`, `phase0_logic_vs_route.rpt`. (These + checkpoints are gitignored; regenerate.)
  **Run `python sim/gen_roms.py` first** — see the ROM trap under "Timing re-baseline". All 8 Tcl
  scripts now source `sim/rtl_sources.tcl`, generated from `sim/sources.py`; after adding or renaming
  an RTL file run `python sim/sources.py --emit-tcl` (`--check-tcl` verifies it is current).
- **Comparing two trees:** always diff endpoint counts and DSP/FF/BRAM as well as WNS. A silently
  degenerate build (missing ROM, missing source file) reports a perfectly plausible WNS; the resource
  counts are what give it away.
