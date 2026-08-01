# Regression harness

Two commands, so that every claim this project makes about itself is a recorded
measurement rather than a figure someone transcribed from a report that has
since been overwritten.

| Claim | Command | Record |
| --- | --- | --- |
| "It closes / it got N ps better" | `python sim/harness.py pnr` | `sim/timing_history.csv` |
| "Bit-identical" | `python sim/harness.py baseline check` | `sim/baselines/ggx_control_oct32.txt` |

Both records are tracked in git. The tool logs and reports behind them are not —
they land in `sim/harness_runs/<timestamp>-<sha>/`, which is gitignored.

## Timing: `python sim/harness.py pnr [--note TEXT]`

Runs the OOC synth + `opt` / `place` / `route` flow at 5 ns (`sim/harness_pnr.tcl`,
no `phys_opt` — see the note in `impl_breakdown.tcl`), takes ~10 minutes, and
appends one row to `sim/timing_history.csv`:

```
commit,dirty,date,wns,tns,failing_endpoints,dsp,lut,bram,carry4,note
```

**Read a row across, not just its WNS.** Resource counts sit next to slack
because a build that quietly lost a source file or a `$readmem` ROM still routes
and still reports a perfectly plausible WNS. Its DSP count is what gives it away.
Two guards enforce that:

- The run is **refused outright** if Vivado could not read a ROM, or if the flow
  did not print its completion sentinel. Vivado exits 0 in both cases.
- A row whose **DSP count differs from the previous row** is refused unless the
  run says why: `--note "folded the inv_sqrt"`. `--allow-dsp-change` records it
  with no explanation, and is the wrong answer nearly every time.

`dirty=True` means the tree had uncommitted changes, so the row cannot be
reproduced from its SHA. Prefer committing first.

### What each column is, exactly

Ambiguity about *which* number was recorded is how a trend quietly stops meaning
anything, so the extraction is pinned:

| Column | Extracted as |
| --- | --- |
| `wns` | slack of the worst setup path, failing or not |
| `tns`, `failing_endpoints` | one worst path per failing endpoint (`-nworst 1 -unique_pins`); TNS is their sum |
| `dsp`, `carry4` | post-route cell count, `REF_NAME =~ DSP48*` / `CARRY4*` |
| `lut` | **`Slice LUTs`** from `report_utilization` — sites, after LUT combining |
| `bram` | Block RAM Tiles: `RAMB36* + 0.5 × RAMB18*` |

`lut` is the one column where the choice is not obvious. `Slice LUTs` (8862 at
`82276de`) counts *sites* and so is what actually competes for the device; the
raw `LUTn` primitive count for the same netlist is 9159, because a combined LUT
pair occupies one site but is two cells. **The pre-harness figure of 8947 quoted
for this design in `docs/handoff.md` is neither**, and does not appear anywhere
in that build's post-route report — it was measured at some other point in the
flow, and how is no longer recoverable. That is the whole argument for this file:
compare rows against rows, and treat any resource figure not in
`timing_history.csv` as unsourced.

### If a row looks impossibly good

Compare DSP, LUT, BRAM and CARRY4 against the previous row before believing the
WNS. The failure mode is not subtle once you look: the missing-ROM build measured
during this harness's own development reported a believable −1.368 ns with 55 DSP
against the real design's 79.

## Bit-identical: `python sim/harness.py baseline capture|check`

`check` runs `test_ggx_control` and compares the Oct32 word the Lane emitted for
every Sample against the committed baseline, reporting the first differences by
burst and sample index. `capture` overwrites the baseline — do that only when the
output is *meant* to change, and say so in the commit message.

The baseline is the raw 32-bit output word per Sample, not a decoded direction:
"bit-identical" is a claim about the bits on the stream, and decoding first would
hide any change smaller than the decode's own rounding.

### Diffing two runs

The baseline file holds one line per Sample —
`<burst> <sample_idx> <oct32_hex> <tlast>` — and by design contains **no
wall-clock time, no simulation time, no tool version and no absolute path**.
There is nothing volatile to filter out, so a plain `diff` is a complete
comparison:

```
GGX_OCT32_DUMP=/tmp/run.txt python sim/test_ggx_control.py
diff sim/baselines/ggx_control_oct32.txt /tmp/run.txt   # header lines only
```

`harness.py baseline check` is the same comparison with the burst/sample index
resolved for you, and it will not compare against a run whose gates failed.

Do **not** diff the cocotb console log instead. It carries simulation timestamps
and absolute build paths on nearly every line, and its per-Sample output stops
after the first five.

## Adding a metric

Print it from `sim/harness_pnr.tcl` as `GGXMETRIC <key> <value>`, add the key to
`METRICS` in `sim/harness.py`, and add the field to `Row`. `read_rows` reads by
column name rather than position, so column order is free — but every existing
row needs the new column backfilled, since a row that is missing it will not
parse at all. That is deliberate: silently reading old rows as zero would put a
fabricated number in a trend.
