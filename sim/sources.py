#!/usr/bin/env python3
"""Single source of the cocotb runners' RTL dependency lists.

Every runner in `sim/` takes its `sources=` from here instead of carrying its
own hand-maintained copy, so that adding, renaming or splitting an RTL file is
a one-line edit in one place rather than twenty edits that can be partially
applied.

The lists are deliberately PER-TOPLEVEL, not one global list: each test must
keep compiling only what it needs, so that centralizing does not silently widen
what a module test exercises. `sources_for()` raises on an unknown toplevel
rather than falling back to something broader.
"""
from pathlib import Path

HDL_DIR = Path(__file__).resolve().parent.parent / "hdl"

# ggx_latency_pkg leads every list. SystemVerilog packages must be analyzed
# before the modules that import them, and per docs/adr/0002 this one becomes a
# dependency of nearly every module. Putting it first unconditionally is why the
# lists were centralized here rather than prepended to twenty copies by hand.
LATENCY_PKG = HDL_DIR / "ggx_latency_pkg.sv"


def _hdl(*names):
    return [LATENCY_PKG] + [HDL_DIR / name for name in names]


# Keyed by `hdl_toplevel`. Order is preserved as each runner had it.
SOURCES = {
    "axis_cordic_normalize": _hdl(
        "axis_cordic_normalize.sv",
    ),
    "axis_fixed_div": _hdl(
        "axis_fixed_div.sv",
    ),
    "axis_fixed_inv_sqrt": _hdl(
        "axis_fixed_inv_sqrt.sv",
    ),
    "axis_fixed_inv_sqrt_folded": _hdl(
        "axis_fixed_inv_sqrt_folded.sv",
    ),
    "axis_fixed_inv_sqrt_nodsp": _hdl(
        "axis_fixed_sqrt.sv",
        "axis_fixed_div.sv",
        "axis_fixed_inv_sqrt_nodsp.sv",
    ),
    "axis_fixed_norm3": _hdl(
        "axis_fixed_norm3.sv",
        "axis_fixed_sqrt.sv",
        "axis_fixed_div.sv",
        "axis_fixed_inv_sqrt_nodsp.sv",
    ),
    "axis_fixed_sqrt": _hdl(
        "axis_fixed_sqrt.sv",
    ),
    "axis_ggx_control": _hdl(
        "axis_ggx_control.sv",
        "axis_skid_buffer.sv",
        "axis_fifo_2deep.sv",
        "axis_ggx_event_basis.sv",
        "axis_pre_ggx_sampler.v",
        "axis_top_lvl_sampler.sv",
        "axis_ggx_projected_area.sv",
        "axis_ggx_reproject_normalize.sv",
        "axis_fixed_norm3.sv",
        "axis_fixed_sqrt.sv",
        "axis_fixed_div.sv",
        "axis_fixed_inv_sqrt_nodsp.sv",
        "axis_fixed_inv_sqrt_folded.sv",
        "axis_sobol2d_stateless.sv",
        "axis_nested_uniform_scramble.sv",
        "axis_hash_combine_2d.sv",
        "axis_trig_lut.sv",
    ),
    "axis_ggx_event_basis": _hdl(
        "axis_ggx_event_basis.sv",
        "axis_fixed_norm3.sv",
        "axis_fixed_sqrt.sv",
        "axis_fixed_div.sv",
        "axis_fixed_inv_sqrt_nodsp.sv",
        "axis_fixed_inv_sqrt_folded.sv",
    ),
    "axis_ggx_projected_area": _hdl(
        "axis_ggx_projected_area.sv",
        "axis_fixed_sqrt.sv",
        "axis_trig_lut.sv",
    ),
    "axis_ggx_reproject_normalize": _hdl(
        "axis_ggx_reproject_normalize.sv",
        "axis_fifo_2deep.sv",
        "axis_fixed_sqrt.sv",
        "axis_fixed_div.sv",
        "axis_fixed_inv_sqrt_nodsp.sv",
        "axis_fixed_inv_sqrt_folded.sv",
        "axis_fixed_norm3.sv",
    ),
    "axis_hash_combine_2d": _hdl(
        "axis_hash_combine_2d.sv",
    ),
    "axis_nested_uniform_scramble": _hdl(
        "axis_nested_uniform_scramble.sv",
    ),
    "axis_pre_ggx_sampler": _hdl(
        "axis_pre_ggx_sampler.v",
        "axis_top_lvl_sampler.sv",
        "axis_sobol2d_stateless.sv",
        "axis_hash_combine_2d.sv",
        "axis_nested_uniform_scramble.sv",
    ),
    "axis_sobol2d_stateless": _hdl(
        "axis_sobol2d_stateless.sv",
    ),
    "axis_top_lvl_sampler": _hdl(
        "axis_top_lvl_sampler.sv",
    ),
    "axis_trig_lut": _hdl(
        "axis_trig_lut.sv",
    ),
}


def sources_for(hdl_toplevel):
    """RTL sources needed to elaborate `hdl_toplevel`.

    Returns a fresh list so a caller cannot mutate the shared one.
    """
    try:
        return list(SOURCES[hdl_toplevel])
    except KeyError:
        raise KeyError(
            f"no source list for toplevel {hdl_toplevel!r}; add one to "
            f"{Path(__file__).name}. Known toplevels: {', '.join(sorted(SOURCES))}"
        ) from None
