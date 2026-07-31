#!/usr/bin/env python3
"""Generate the ROM .mem files the Vivado flow reads.

The cocotb tests build their own ROMs into sim/sim_build/ via their
ensure_*_rom helpers, but the Vivado scripts run from sim/ and $readmemh
'ggx_trig_rom.mem' out of THIS directory. That file is gitignored, so a fresh
clone or a git worktree does not have it -- and Vivado does not treat a missing
$readmem file as an error. It emits a CRITICAL WARNING, leaves the ROM
uninitialized, and constant-folds the whole trig LUT away. The design still
synthesizes and still reports timing; it is simply not the design (12 DSPs and
2.5 BRAMs lighter, and correspondingly optimistic).

That failure is silent and produces a plausible-looking number, so run this
before any synthesis or P&R:

    python sim/gen_roms.py

rtl_sources.tcl refuses to proceed if the file is missing.
"""
import numpy as np
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
TRIG_ADDR_BITS = 10
Q31 = 2**31


def _float_to_q131(f):
    if f >= 1.0:
        f = np.nextafter(1.0, 0.0)
    if f < -1.0:
        f = -1.0
    v = int(np.round(f * Q31))
    v = min(max(v, -0x8000_0000), 0x7FFF_FFFF)
    return v & 0xFFFF_FFFF


def write_trig_rom(path):
    """{sin, cos} pairs in Q1.31, packed 64-bit. Matches ensure_trig_rom()."""
    n = 1 << TRIG_ADDR_BITS
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="ascii") as f:
        for i in range(n):
            ang = (i / n) * 2.0 * np.pi
            c = _float_to_q131(np.cos(ang))
            s = _float_to_q131(np.sin(ang))
            f.write(f"{((s << 32) | c) & 0xFFFF_FFFF_FFFF_FFFF:016x}\n")
    return n


if __name__ == "__main__":
    target = SIM_DIR / "ggx_trig_rom.mem"
    n = write_trig_rom(target)
    print(f"wrote {target} ({n} entries)")
