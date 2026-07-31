#!/usr/bin/env python3
"""Unit test for axis_hash_combine_2d.

This module had no direct coverage, which mattered once its four 32x32 constant
multiplies were rewritten as 16-bit split partial products (x*C = x_lo*C_lo +
((x_lo*C_hi + x_hi*C_lo) << 16), mod 2^32). That rewrite must be BIT-EXACT: a
hash's whole purpose is that every input bit reaches every output bit, so an
error of one ULP is not a small error, it is a different hash.

The test drives random values plus edge cases chosen to stress the split
specifically -- the 16-bit boundary, carries out of the low half, and operands
whose halves are individually zero -- against an independent Python MurmurHash3.

NOTE ON ALIGNMENT: every module in the sampler chain aligns its own TVALID with
its own TDATA (as on main). The system-level combining in axis_pre_ggx_sampler
(scram0_v = hash0_v && sobol0_v, plus the delay_index line on the Sobol path)
assumes that per-module alignment. This test pins the offset at 0 so any latency
change that skews data against valid is caught by the module test instead of
surfacing as a scrambled sample stream downstream (see test_integration_dual,
which caught exactly that during the DSP-quantize refactor).
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

DIMENSION = 0
LATENCY = 18
# TVALID and TDATA are aligned (18-reg data path, 18-deep valid pipeline).
VALID_DATA_OFFSET = 0


def u32(x):
    return x & 0xFFFF_FFFF


def rotl32(x, r):
    x = u32(x)
    return u32((x << r) | (x >> (32 - r)))


def fmix32(h):
    h = u32(h)
    h ^= h >> 16
    h = u32(h * 0x85EBCA6B)
    h ^= h >> 13
    h = u32(h * 0xC2B2AE35)
    h ^= h >> 16
    return u32(h)


def mix(h, k):
    k = u32(u32(k) * 0xCC9E2D51)
    k = rotl32(k, 15)
    k = u32(k * 0x1B873593)
    h = u32(h) ^ k
    h = rotl32(h, 13)
    return u32(h * 5 + 0xE6546B64)


def hash_combine_ref(value, dimension=DIMENSION):
    """Matches the RTL: mix(mix(H0, value), DIMENSION) then fmix32."""
    h = 0x9747B28C
    h = mix(h, value)
    h = mix(h, dimension)
    return fmix32(h)


def stimulus():
    """Random values plus cases that specifically stress the 16-bit split."""
    edges = [
        0x0000_0000,
        0xFFFF_FFFF,
        0x0000_FFFF,  # low half all ones -> carry out of p_ll into the middle
        0xFFFF_0000,  # high half only -> exercises p_hl alone
        0x0001_0000,
        0x0000_0001,
        0x8000_0000,
        0x7FFF_FFFF,
        0xFFFF_8000,
        0x8000_FFFF,
    ]
    rng = random.Random(0xC0FFEE)
    return edges + [rng.getrandbits(32) for _ in range(300)]


@cocotb.test()
async def test_hash_combine_2d(dut):
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
        dut.s00_axis_tdata.value = v
        dut.s00_axis_tvalid.value = 1
    await FallingEdge(dut.s00_axis_aclk)
    dut.s00_axis_tvalid.value = 0

    await ClockCycles(dut.s00_axis_aclk, LATENCY + 20)

    assert len(got) == len(values), f"expected {len(values)} outputs, saw {len(got)}"

    best_off, best_n = max(
        (
            (
                off,
                sum(
                    1
                    for i, g in enumerate(got)
                    if 0 <= i + off < len(values) and g == hash_combine_ref(values[i + off])
                ),
            )
            for off in range(-4, 5)
        ),
        key=lambda t: t[1],
    )
    assert best_off == VALID_DATA_OFFSET, (
        f"pipeline alignment changed: outputs now track inputs at offset {best_off:+d}, "
        f"expected {VALID_DATA_OFFSET:+d}. The system aligns these streams itself, so a "
        "change here means module latency moved and axis_pre_ggx_sampler needs rechecking."
    )

    comparable = [
        (i, values[i + best_off], g)
        for i, g in enumerate(got)
        if 0 <= i + best_off < len(values)
    ]
    mismatches = [
        f"  #{i} in=0x{v:08x} got=0x{g:08x} exp=0x{hash_combine_ref(v):08x}"
        for i, v, g in comparable
        if g != hash_combine_ref(v)
    ]
    assert not mismatches, (
        f"{len(mismatches)}/{len(comparable)} hash mismatches -- the split-constant "
        "multiply is NOT bit-exact:\n" + "\n".join(mismatches[:10])
    )
    dut._log.info(f"axis_hash_combine_2d: {len(comparable)} values bit-exact vs MurmurHash3 "
        f"reference at alignment offset {best_off:+d}")


def hash_combine_runner():
    sim = os.getenv("SIM", "icarus")
    sources = rtl_sources.sources_for("axis_hash_combine_2d")
    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel="axis_hash_combine_2d",
        always=True,
        build_args=["-Wall", "-I", str(proj_path / "hdl")],
        parameters={"DIMENSION": DIMENSION},
    )
    runner.test(hdl_toplevel="axis_hash_combine_2d", test_module=test_file)


if __name__ == "__main__":
    hash_combine_runner()
