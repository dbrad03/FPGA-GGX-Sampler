#!/usr/bin/env python3
"""Unit test for axis_cordic_normalize (Step 1.3 CORDIC vector normalize).

Drives random + edge-case 3D vectors and checks the hardware output against a
NumPy reference (v/|v|). The RTL mirrors the verified 2-pass dividerless CORDIC
(scratchpad cordic_dev.py): vectoring to measure, inverse rotations to
reconstruct, with inter-plane 1/G rescale and a z-channel xG correction.

Tolerance: the fixed-point model (N=18, W=24) predicts max direction error
~1.1e-5, so allow 5e-5 with margin. Also check |output| ~= 1.
"""
import os
import random
from pathlib import Path

import numpy as np
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge, ReadOnly
from cocotb.runner import get_runner

test_file = os.path.basename(__file__).replace(".py", "")
proj_path = Path(__file__).resolve().parent.parent

Q31 = 2**31
DIR_TOL = 5e-5
MAG_TOL = 2e-4


def to_q131(f):
    v = int(np.floor(f * Q31 + 0.5))
    v = max(-Q31, min(Q31 - 1, v))
    return v & 0xFFFFFFFF


def from_q131(u):
    u &= 0xFFFFFFFF
    if u & 0x80000000:
        u -= 0x100000000
    return u / Q31


def pack(v):
    x = to_q131(v[0]); y = to_q131(v[1]); z = to_q131(v[2])
    return (z << 64) | (y << 32) | x


def unpack(d):
    x = from_q131(d & 0xFFFFFFFF)
    y = from_q131((d >> 32) & 0xFFFFFFFF)
    z = from_q131((d >> 64) & 0xFFFFFFFF)
    return np.array([x, y, z])


def stimulus():
    rng = random.Random(0xCED1C)
    vs = []
    # edge cases: axis-aligned (both signs), diagonal, tiny-off-axis, x<0 range
    for e in ([0.9, 0, 0], [-0.9, 0, 0], [0, 0.9, 0], [0, 0, 0.9], [0, 0, -0.9],
              [0.5, 0.5, 0.5], [-0.5, 0.5, -0.5], [-0.01, 0.01, 0.9],
              [0.9, -0.001, 0.001], [0.7, -0.6, 0.03]):
        vs.append(np.array(e, float))
    for _ in range(400):
        v = np.array([rng.uniform(-0.95, 0.95) for _ in range(3)])
        if np.linalg.norm(v) > 0.05:
            vs.append(v)
    return vs


@cocotb.test()
async def test_cordic_normalize(dut):
    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    dut.s00_axis_aresetn.value = 0
    dut.s00_axis_tvalid.value = 0
    dut.s00_axis_tlast.value = 0
    dut.s00_axis_tdata.value = 0
    dut.s00_axis_tstrb.value = 0xFFFF
    dut.m00_axis_tready.value = 1
    await ClockCycles(dut.s00_axis_aclk, 5)
    dut.s00_axis_aresetn.value = 1
    await ClockCycles(dut.s00_axis_aclk, 2)

    vs = stimulus()
    got = []

    async def collect():
        while True:
            await RisingEdge(dut.s00_axis_aclk)
            await ReadOnly()
            if dut.m00_axis_tvalid.value == 1 and dut.m00_axis_tready.value == 1:
                got.append(unpack(int(dut.m00_axis_tdata.value)))

    cocotb.start_soon(collect())

    for v in vs:
        await FallingEdge(dut.s00_axis_aclk)
        dut.s00_axis_tdata.value = pack(v)
        dut.s00_axis_tvalid.value = 1
    await FallingEdge(dut.s00_axis_aclk)
    dut.s00_axis_tvalid.value = 0

    await ClockCycles(dut.s00_axis_aclk, 4 * 18 + 30)

    assert len(got) == len(vs), f"expected {len(vs)} outputs, saw {len(got)}"

    max_dir = max_mag = 0.0
    worst = None
    for v, g in zip(vs, got):
        ref = v / np.linalg.norm(v)
        de = float(np.linalg.norm(g - ref))
        me = abs(float(np.linalg.norm(g)) - 1.0)
        if de > max_dir:
            max_dir = de; worst = (v, g, ref)
        max_mag = max(max_mag, me)

    dut._log.info(f"axis_cordic_normalize: n={len(vs)} max_dir_err={max_dir:.3e} "
                  f"max_mag_dev={max_mag:.3e} (dir_tol={DIR_TOL:.1e})")
    if worst is not None:
        dut._log.info(f"  worst v={np.round(worst[0],4)} got={np.round(worst[1],4)} "
                      f"ref={np.round(worst[2],4)}")
    assert max_dir < DIR_TOL, f"direction error {max_dir:.3e} exceeds {DIR_TOL:.1e}"
    assert max_mag < MAG_TOL, f"magnitude deviation {max_mag:.3e} exceeds {MAG_TOL:.1e}"


def cordic_runner():
    sim = os.getenv("SIM", "icarus")
    sources = [proj_path / "hdl" / "axis_cordic_normalize.sv"]
    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel="axis_cordic_normalize",
        always=True,
        build_args=["-Wall", "-I", str(proj_path / "hdl")],
    )
    runner.test(hdl_toplevel="axis_cordic_normalize", test_module=test_file)


if __name__ == "__main__":
    cordic_runner()
