# The latency package is law: producers derive from it too

Pipeline latencies are currently hand-maintained magic numbers scattered across modules
(`DIV_DLY = 115`, `SQRT_LATENCY = 33`, `SIDEBAND_DEPTH = 18`, a `NORM_LATENCY = 10` that nothing reads,
an `INVSQRT_LATENCY = 150` self-labelled "unused; informational" and wrong for the folded engine). A
data path and its sideband drifting out of step is not a hypothetical: it is exactly what produced the
TDATA/TVALID skew, which module tests then codified as a "characterised offset" instead of failing on.
We introduce `hdl/ggx_latency_pkg.sv` as the single source of these numbers, and require that
**producing modules build their own pipeline depths from it, not just consumers** — a package that
only consumers read is documentation, and documentation is what drifted.

## Considered options

- **Consumers only** — much smaller diff, and it avoids reopening the digit-recurrence cores that were
  recently verified bit-identical. Rejected: it recreates the exact failure shape of the stale
  `INVSQRT_LATENCY`, but with the added harm of looking authoritative.
- **No package; `initial $fatal` depth checks only** — catches the same bug class at elaboration with
  zero new files, and was the initial recommendation on the assumption that Icarus's SystemVerilog
  package support was too weak to rely on. That assumption was tested and found false (Icarus 13.0
  compiles a package with `automatic` functions called in `localparam` context, wildcard import, and
  `$fatal`). The assertions are kept anyway as belt-and-braces, but they are no longer the mechanism.

## Consequences

- The package becomes a dependency of nearly every module, so it must appear first in every source
  list. Rather than prepend it to 20 hand-maintained cocotb runner lists, those are centralized into
  `sim/sources.py`. The 8 Vivado Tcl scripts are patched by hand — they serve the synthesis and P&R
  flow, and coupling the way timing is measured to a Python module is not a trade worth making
  mid-closure.
- Changing a core's pipeline depth now changes its consumers' delay lines automatically. That is the
  point, but it means a depth change is no longer a local edit and must be re-verified bit-identical
  across the whole cascade.
