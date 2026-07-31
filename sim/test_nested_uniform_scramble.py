#!/usr/bin/env python3
"""Unit test for axis_nested_uniform_scramble.

This module had no direct coverage, which mattered once its four 32x32 constant
multiplies were rewritten as 16-bit split partial products. The Laine-Karras
permutation must be BIT-EXACT -- it decorrelates Sobol points, so an error of
one ULP silently changes sample placement rather than adding a small numeric
error.

NOTE ON ALIGNMENT: this design does NOT align TVALID with TDATA per module. The
sampler combines streams at system level (scram0_v = hash0_v && sobol0_v, plus a
16-deep delay_index on the Sobol path) and the module pipeline depths are
co-tuned to that, so each module's valid tap is offset from its own data by a
fixed amount. Retapping a module to "fix" its local alignment breaks the system
(verified: it fails test_ggx_control). So this test characterises the offset and
asserts the ARITHMETIC is bit-exact there -- which is what the split-constant
multiply rewrite needed to prove -- and pins the offset so any latency change is
caught rather than silently absorbed.
"""
import os
import random
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge, ReadOnly
from cocotb.runner import get_runner
import rtl_sources

test_file = os.path.basename(__file__).replace(".py", "")
proj_path = Path(__file__).resolve().parent.parent

SEED = 0x1234_5678
# TVALID and TDATA are aligned (sideband depth matches the 19-reg data path).
VALID_DATA_OFFSET = 0


def u32(x):
    return x & 0xFFFF_FFFF


def reverse_bits(x):
    return int(f"{u32(x):032b}"[::-1], 2)


def laine_karras_permutation(x, seed):
    x = u32(x + seed)
    x ^= u32(x * 0x6C50B47C)
    x = u32(x)
    x ^= u32(x * 0xB82F1E52)
    x = u32(x)
    x ^= u32(x * 0xC7AFE638)
    x = u32(x)
    x ^= u32(x * 0x8D22F6E6)
    return u32(x)


def nested_uniform_scramble_ref(x, seed):
    x = reverse_bits(x)
    x = laine_karras_permutation(x, seed)
    return reverse_bits(x)


def stimulus():
    edges = [
        0x0000_0000,
        0xFFFF_FFFF,
        0x0000_FFFF,  # carry out of the low partial product
        0xFFFF_0000,  # high half only
        0x0000_0001,
        0x8000_0000,
        0x7FFF_FFFF,
        0x0001_0000,
    ]
    rng = random.Random(0x5EED)
    return edges + [rng.getrandbits(32) for _ in range(300)]


@cocotb.test()
async def test_nested_uniform_scramble(dut):
    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    dut.s00_axis_aresetn.value = 0
    dut.s00_axis_tvalid.value = 0
    dut.s00_axis_tlast.value = 0
    dut.s00_axis_tdata.value = 0
    dut.s00_axis_tstrb.value = 0xFF
    dut.m00_axis_tready.value = 1
    await ClockCycles(dut.s00_axis_aclk, 5)
    dut.s00_axis_aresetn.value = 1
    await ClockCycles(dut.s00_axis_aclk, 2)

    values = stimulus()
    got = []

    async def collect():
        while True:
            await RisingEdge(dut.s00_axis_aclk)
            await ReadOnly()
            if dut.m00_axis_tvalid.value == 1 and dut.m00_axis_tready.value == 1:
                got.append(int(dut.m00_axis_tdata.value) & 0xFFFF_FFFF)

    cocotb.start_soon(collect())

    for v in values:
        await FallingEdge(dut.s00_axis_aclk)
        dut.s00_axis_tdata.value = (SEED << 32) | v
        dut.s00_axis_tvalid.value = 1
    await FallingEdge(dut.s00_axis_aclk)
    dut.s00_axis_tvalid.value = 0

    await ClockCycles(dut.s00_axis_aclk, 40)

    assert len(got) == len(values), f"expected {len(values)} outputs, saw {len(got)}"

    best_off, best_n = max(
        (
            (
                off,
                sum(
                    1
                    for i, g in enumerate(got)
                    if 0 <= i + off < len(values)
                    and g == nested_uniform_scramble_ref(values[i + off], SEED)
                ),
            )
            for off in range(-8, 9)
        ),
        key=lambda t: t[1],
    )
    assert best_off == VALID_DATA_OFFSET, (
        f"pipeline alignment changed: outputs now track inputs at offset {best_off:+d}, "
        f"expected {VALID_DATA_OFFSET:+d}. TVALID must stay aligned with TDATA: grow or "
        "shrink SIDEBAND_DEPTH to match the data-path register count."
    )

    comparable = [
        (i, values[i + best_off], g)
        for i, g in enumerate(got)
        if 0 <= i + best_off < len(values)
    ]
    mismatches = [
        f"  #{i} in=0x{v:08x} got=0x{g:08x} exp=0x{nested_uniform_scramble_ref(v, SEED):08x}"
        for i, v, g in comparable
        if g != nested_uniform_scramble_ref(v, SEED)
    ]
    assert not mismatches, (
        f"{len(mismatches)}/{len(comparable)} scramble mismatches -- the split-constant "
        "multiply is NOT bit-exact:\n" + "\n".join(mismatches[:10])
    )
    dut._log.info(
        f"axis_nested_uniform_scramble: {len(comparable)} values bit-exact vs "
        f"Laine-Karras reference at alignment offset {best_off:+d}"
    )


def scramble_runner():
    sim = os.getenv("SIM", "icarus")
    sources = rtl_sources.sources_for("axis_nested_uniform_scramble")
    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel="axis_nested_uniform_scramble",
        always=True,
        build_args=["-Wall", "-I", str(proj_path / "hdl")],
    )
    runner.test(hdl_toplevel="axis_nested_uniform_scramble", test_module=test_file)


if __name__ == "__main__":
    scramble_runner()
