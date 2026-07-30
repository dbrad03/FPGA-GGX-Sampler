# GGX single-lane floorplanning guide (Vivado GUI)

Project: `vivado/ggx_floorplan/ggx_floorplan.xpr` — out-of-context, 5 ns clock (200 MHz),
`axis_ggx_control` as top. **Includes the folded per-burst inv_sqrt** (bit-identical, already
cut `u_basis` ~41% and un-smeared it). Baseline to beat: **WNS −1.454, TNS −569** (fold, no floorplan).

## Open + floorplan loop
1. `vivado vivado/ggx_floorplan/ggx_floorplan.xpr`
2. Flow Navigator → **Open Synthesized Design** (synthesis is already run).
3. **Layout → Floorplanning** perspective. In the **Device** view, click **Draw Pblock** (toolbar),
   drag a rectangle over a clock region, then in the **Netlist** pane drag a hierarchy instance
   (e.g. `u_basis`) onto that Pblock. Repeat per block.
4. **Run Implementation** (uses the Pblock constraints). Check **Timing** → Report Timing Summary.
5. Iterate: adjust Pblock rectangles, re-implement.

To also *see current congestion* first, open the routed checkpoint separately:
`vivado sim/fold_routed.dcp` → Reports → Report Design Analysis / open the Device view (the
congestion heat-map + the −1.454 critical path highlight show where to pull things apart).

## Where things are today (from the fold routed checkpoint) — device has 6 clock regions X0Y0..X1Y2
- `u_basis` (event_basis, PER-BURST) — 15785 cells, mostly **X1Y0** + some X0Y0. Already compact.
- `u_reproject_normalize` (PER-SAMPLE) — 19684 cells, **smeared across X0Y0/X0Y1/X1Y0/X1Y1**. This is
  now the biggest spread block and holds the **WNS path** (a skid-FIFO read → reproject DSP, 72% route).
- `u_projected_area` (PER-SAMPLE) — 3103 cells, X0Y0/X1Y1.
- `u_sampler` (sobol/scramble/hash) — 3552 cells, X0Y0/X1Y0.

## Suggested starting Pblocks (give each big block its own home, stop them interleaving)
- `u_reproject_normalize` → **X1Y1 + X1Y2** (right column, 2 regions — it's the biggest, needs room).
- `u_basis` → **X1Y0** (already lives there; pin it so it stops leaking into other regions).
- `u_sampler` + `u_projected_area` → **X0Y0** (+ X0Y1 if tight).
- Leave a little slack — don't pack a Pblock over ~70% or routing gets worse, not better.

The two current walls a good floorplan should attack:
1. **WNS −1.454, 72% route**: `u_skid_proj` FIFO read → `u_reproject_normalize` DSP. Co-locate the
   reproject block (and its skid buffer) so that read doesn't cross the die.
2. **−1.235 logic band**: `u_basis` t2a/t2b (CARRY4=5) — logic-bound, floorplan won't fix this one;
   it needs pipelining/width work. Focus the floorplan on the route-bound reproject paths.

## Tips
- After drawing a Pblock, right-click → **check** its RESIZE/utilization; the Statistics tab shows
  LUT/FF/DSP fit per region so you don't over-fill.
- Pblock constraints land in `ggx_clk.xdc` (or a new XDC) as `create_pblock` / `add_cells_to_pblock`
  / `resize_pblock` — you can copy those out to reuse in the script flow later.
- Keep `-mode out_of_context`; timing then matches the OOC numbers we've been tracking.
