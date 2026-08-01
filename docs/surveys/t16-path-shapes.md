# T16 — path shapes, and the campaign order re-derived from them

Issue #22. Measured 2026-08-01 at `bb5cfbd`, one OOC post-route run at 5 ns, no
phys_opt: **WNS −1.330, TNS −406.686, 1852 failing endpoints, 79 DSP, 8862 LUT,
4.5 BRAM, 1302 CARRY4** (`sim/timing_history.csv`). Reproduce with:

```
python sim/harness.py survey
```

Full `report_timing` text for every path below is in that run's
`survey_paths.rpt`. The endpoint counts and worst slacks reproduce the table in
#22 exactly, which is the cross-check that this is the same design the campaign
was planned against.

## What was measured

The worst failing path in each block, and **what it is made of** — logic levels,
CARRY4 count on the path, the logic-vs-route split, and both endpoint pins.

| block | endpts | slack | lvls | CARRY4 | DSP | logic | route | route% | shape |
|---|---|---|---|---|---|---|---|---|---|
| `reproject_binder` | 6 | **−1.330** | 4 | 0 | 1 | 1.339 | 3.151 | 70.2% | RAM→LUT→DSP operand |
| `basis_normalize` | 211 | −0.822 | 9 | **5** | 0 | 2.204 | 3.565 | 61.8% | carry chains |
| `sampler_scram1` | 332 | −0.695 | 4 | 0 | 1 | 1.090 | 3.997 | 78.6% | **ready chain** |
| `projected_area_sq` | 48 | −0.633 | **0** | 0 | **2** | 4.009 | 1.090 | 21.4% | **DSP→DSP** |
| `sampler_scram0` | 348 | −0.617 | 4 | 0 | 1 | 1.090 | 3.919 | 78.2% | **ready chain** |
| `sampler_hash0` | 101 | −0.484 | 5 | 3 | 2 | 1.890 | 2.965 | 61.1% | carry chains |
| `sampler_hash1` | 140 | −0.481 | 4 | 0 | 1 | 1.090 | 3.783 | 77.6% | **ready chain** |
| `oct32` | 35 | −0.315 | 9 | **6** | 0 | 3.096 | 2.167 | 41.2% | carry chains |

Classification rules, stated so they can be disagreed with:

- **Pipelineable** — ≥2 CARRY4 on the path. Two carry chains inside one cycle is
  a register split between them, which costs latency and nothing else. This is
  the lever that bought 1.15 ns on the T2 band.
- **Route-bound** — route ≥70% with no CARRY4. There is almost no logic to cut;
  the delay is wire, and pipelining the arithmetic buys nothing.
- **Structural** — no fabric logic to split at all (0 levels, DSP to DSP). The
  fix changes the arithmetic or the DSP's own pipelining, not the fabric.

## Finding 1 — the sampler's bulk is one ready chain, not its multiplies

**820 of the sampler's 921 failing endpoints are in three buckets whose worst
paths are the same path.** `scram0`, `scram1` and `hash1` all start at
`u_projected_area/d1b_valid_reg/C`, all end on a **DSP48 `CEP` pin** — a clock
enable, not a data input — and all report identical logic delay (1.090 ns) and
levels (4). They are one net structure seen three times:

```
d1b_valid_reg ─LUT6→ sqrt_t_in_ready  (fo=626) ─LUT4→ s1b_to_s2 (fo=46)
              ─LUT5→ s1_valid_reg_0   (fo=617) ─LUT4→ scram0_in_ready (fo=704)
              → DSP48E1/CEP
```

This is AXI-Stream **backpressure propagating combinationally** across
`sqrt_t` → `trig` → `scram0`, each hop fanning out to hundreds of DSP clock
enables. Route is 78% of the delay and the final hop alone is **1.626 ns on a
fanout-704 net**.

**This changes what #23 (T17) is for.** That ticket proposes DSP-free constant
multiplies to fix the sampler. But these paths never touch the multiply logic —
they end at its *enable* pins. Rewriting the multiplies leaves the ready chain
exactly where it is. Only `hash0` (101 endpoints, −0.484) is genuinely inside
the constant-multiply logic.

The fix is the one this project already owns: break the combinational ready path
with elastic buffering. `axis_skid_buffer.sv` and `axis_fifo_2deep.sv` exist,
and `44def36` did precisely this for the stage 2/2a/2b chain.

**Limit of this claim:** what is measured is that the *worst* path in each of the
three buckets is this chain. That 820 endpoints all lie on it is inference from
three shared startpoints, one shared endpoint type and identical logic delay —
strong, but not individually traced. Confirm by counting failing endpoints whose
path passes through `scram0_in_ready` / `s1_valid_reg_0` before committing
effort on that basis.

## Finding 2 — the WNS holder is not the multiply it is named after

`reproject_binder` holds WNS at −1.330 across **6 endpoints**. Its path is:

```
u_skid_proj/rd_ptr_reg/C → RAMD32 read → LUT4 → LUT5 → LUT4
  → u_reproject_normalize/mul_pre_q131_q131_to_q262/A[0]
```

The multiply is the *destination*. The delay is a distributed-RAM read in the
skid buffer plus operand conditioning getting to the DSP's A port — 70% route,
zero carry chains. The lever is a **register on the operand between the skid
buffer and the DSP**, not restructuring the a-section multiply as #24 (T18)
assumes. Worth doing for WNS; worth almost nothing for TNS, at 6 of 1852
endpoints.

## Finding 3 — `projected_area_sq` is structural, and the estimate assumes it is not

#22 asks explicitly for this flag. `projected_area_sq` has **zero logic levels**:
DSP48E1 output → one net → DSP48E1 `B[8]`, with 4.009 ns of the 5.099 ns being
the DSP's own propagation. There is nothing in the fabric to split. The fix is
the DSP's internal pipeline registers or a restructured square — a latency and
arithmetic change, not a register split.

**#27 (T21)'s cost estimate does not hold.** It is the one block in the survey
where the campaign's "this is a pipelining problem" assumption is false.

## Finding 4 — Oct32's own violation is the shallowest, and is a clean split

`oct32` is −0.315 over 35 endpoints, with **6 CARRY4** in one path inside
`u_div_x` and 59% of the delay in logic. Textbook pipelineable, and the smallest
violation in the survey.

#28 (T22) asks whether to keep or revert Oct32 on measured evidence. This is
evidence for **keep**: what Oct32 costs in timing is a standard register split on
a divider, not a structural problem — and ADR-0001 records that it already
bought the deletion of the per-sample normalize.

## The re-derived order

Ordered by slack recovered per unit of work and risk, not by endpoint count:

| # | work | block | endpts | why here |
|---|---|---|---|---|
| 1 | **new — break the ready chain** | scram0/1, hash1 | ~820 | one change, ~44% of all failing endpoints, uses buffers the repo already has |
| 2 | #25 T19 | `basis_normalize` | 211 | 5 carry chains, the cleanest split in the survey, 2nd-worst slack |
| 3 | #28 T22 | `oct32` | 35 | cheap split; settles keep-or-revert with evidence rather than opinion |
| 4 | #24 T18 | `reproject_binder` | 6 | holds WNS, so it gates the headline number — but operand register, not multiply rewrite |
| 5 | #23 T17 | `sampler_hash0` | 101 | the only bucket that really is the constant multiplies |
| 6 | #27 T21 | `projected_area_sq` | 48 | structural; needs re-estimating before it is scheduled |

### Where this differs from the endpoint-count order, and why

- **The sampler splits in two.** The endpoint order treats its 921 endpoints as
  one problem for #23. They are two: ~820 on a ready chain, ~101 in the
  multiplies. The big half is not what #23 proposes to fix.
- **The WNS holder drops to 4th.** −1.330 is the headline, but 6 endpoints is
  0.3% of the failures. It gates the *number*, not the design.
- **Oct32 rises.** Ranked last by slack, but it is 35 endpoints of ordinary carry
  chain and it closes an open decision (#28) for very little work.
- **`projected_area_sq` drops to last.** Not because it is small, but because it
  is the only one whose fix is not the kind of change the campaign is costed for.

## What was not measured

Only the **worst** path per block. A block can hold a second, differently-shaped
population under its worst path — most plausibly `basis_normalize` and `hash0`,
which have real endpoint counts and mixed shapes. If a fix lands and the block's
slack barely moves, survey its *next* worst path before assuming the fix failed.
