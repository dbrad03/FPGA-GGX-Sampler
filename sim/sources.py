#!/usr/bin/env python3
"""Single source of the RTL dependency lists, for cocotb AND for Vivado.

Every runner in `sim/` takes its `sources=` from here instead of carrying its
own hand-maintained copy, so that adding, renaming or splitting an RTL file is
a one-line edit in one place rather than twenty edits that can be partially
applied.

The lists are deliberately PER-TOPLEVEL, not one global list: each test must
keep compiling only what it needs, so that centralizing does not silently widen
what a module test exercises. `sources_for()` raises on an unknown toplevel
rather than falling back to something broader.

The 8 Vivado Tcl scripts consume the same lists via the GENERATED
`sim/rtl_sources.tcl`, which is checked in. ADR 0002 originally kept those
scripts hand-patched to avoid coupling timing measurement to a Python module,
and that concern is respected here: Vivado sources a plain Tcl file and never
runs Python. What it does not survive is hand-maintenance -- the fold commit
added axis_fixed_inv_sqrt_folded.sv to 3 of the 8 scripts, leaving the other 5
failing synthesis outright for the whole timing-closure effort before anyone
noticed.

    python sim/sources.py --emit-tcl     regenerate rtl_sources.tcl
    python sim/sources.py --check-tcl    fail if it is out of date
"""
import sys
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


# ---------------------------------------------------------------------------
# Vivado Tcl emission
# ---------------------------------------------------------------------------

TCL_PATH = Path(__file__).resolve().parent / "rtl_sources.tcl"


def tcl_text():
    """The contents of the generated rtl_sources.tcl."""
    out = [
        "# GENERATED FILE -- DO NOT EDIT.",
        "#",
        "# Regenerate:  python sim/sources.py --emit-tcl",
        "# Verify:      python sim/sources.py --check-tcl",
        "# Source of truth: sim/sources.py",
        "#",
        "# Paths resolve relative to THIS file, so it can be sourced from sim/ or",
        "# vivado/ alike. Every list leads with ggx_latency_pkg.sv, because a",
        "# SystemVerilog package must be analyzed before the modules importing it.",
        "#",
        "# Read these with `read_verilog -sv`, including axis_pre_ggx_sampler.v:",
        "# it keeps a .v extension but imports ggx_latency_pkg, and Vivado would",
        "# otherwise infer Verilog-2001 from the extension and reject the package",
        "# scope resolution.",
        "",
        "set _rtl_dir [file normalize [file join [file dirname [info script]] .. hdl]]",
        "",
        "# The trig LUT's ROM is gitignored, so a fresh clone or worktree lacks it.",
        "# Vivado does NOT error on a missing $readmem file -- it warns, leaves the",
        "# ROM uninitialized and constant-folds the LUT away, then reports timing for",
        "# a design 12 DSPs lighter than the real one. Fail loudly instead.",
        "set _trig_rom [file join [file dirname [info script]] ggx_trig_rom.mem]",
        "if {![file exists $_trig_rom]} {",
        "  error \"missing $_trig_rom -- run: python sim/gen_roms.py\"",
        "}",
        "",
    ]
    for top, files in sorted(SOURCES.items()):
        out.append(f"set RTL_SOURCES({top}) [list \\")
        for f in files:
            out.append(f"  $_rtl_dir/{f.name} \\")
        out.append("]")
        out.append("")
    return "\n".join(out)


def _main(argv):
    if "--emit-tcl" in argv:
        TCL_PATH.write_text(tcl_text(), encoding="ascii")
        print(f"wrote {TCL_PATH}")
        return 0
    if "--check-tcl" in argv:
        current = TCL_PATH.read_text(encoding="ascii") if TCL_PATH.exists() else ""
        if current != tcl_text():
            print(
                f"ERROR: {TCL_PATH.name} is out of date with sources.py.\n"
                "       Run: python sim/sources.py --emit-tcl",
                file=sys.stderr,
            )
            return 1
        print(f"{TCL_PATH.name} is up to date")
        return 0
    print(__doc__)
    return 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
