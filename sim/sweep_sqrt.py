#!/usr/bin/env python3
"""Sweep axis_fixed_sqrt SIG_BITS and measure sqrt-local error vs true sqrt.
Run: SIG_BITS=<n> python sweep_sqrt.py   (one build per invocation)
"""
import os, sys
import numpy as np
from pathlib import Path
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge, ReadOnly
from sources import sources_for
try:
    from cocotb.runner import get_runner
except ImportError:
    from cocotb_tools.runner import get_runner

test_file = os.path.basename(__file__).replace(".py", "")
proj_path = Path(__file__).resolve().parent.parent
SIG_BITS = int(os.getenv("SIG_BITS", "32"))


@cocotb.test()
async def sweep(dut):
    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    dut.s00_axis_aresetn.value = 0
    dut.s00_axis_tvalid.value = 0
    dut.s00_axis_tstrb.value = 0xF
    dut.s00_axis_tlast.value = 0
    dut.m00_axis_tready.value = 1
    await ClockCycles(dut.s00_axis_aclk, 5)
    dut.s00_axis_aresetn.value = 1
    await ClockCycles(dut.s00_axis_aclk, 2)

    # Input distribution: emphasise small x (where sqrt is most sensitive) + uniform.
    rng = np.random.default_rng(1)
    xs = np.concatenate([
        rng.uniform(0, 2**-16, 4000),
        rng.uniform(2**-16, 2**-8, 4000),
        rng.uniform(2**-8, 1.0, 8000),
        np.array([np.nextafter(1.0, 0.0), 2**-32, 0.25, 0.5, 0.75]),
    ])
    xu = np.clip(np.floor(xs * (1 << 32)).astype(np.int64), 0, (1 << 32) - 1)

    outs = []
    # drive + collect (simple: one in per cycle, latency 34, tready always high)
    idx = 0
    N = len(xu)
    collected = 0
    # feed all, then drain
    async def drive():
        for v in xu:
            await FallingEdge(dut.s00_axis_aclk)
            dut.s00_axis_tdata.value = int(v)
            dut.s00_axis_tvalid.value = 1
            await RisingEdge(dut.s00_axis_aclk)
        await FallingEdge(dut.s00_axis_aclk)
        dut.s00_axis_tvalid.value = 0

    cocotb.start_soon(drive())
    for _ in range(N + 60):
        await RisingEdge(dut.s00_axis_aclk)
        await ReadOnly()
        if dut.m00_axis_tvalid.value == 1 and dut.m00_axis_tready.value == 1:
            outs.append(int(dut.m00_axis_tdata.value))

    outs = np.array(outs[:N], dtype=np.float64)
    x = xu[:len(outs)] / (1 << 32)
    act = outs / (1 << 32)
    ref = np.sqrt(x)
    err = act - ref
    dut._log.info(
        f"SIG_BITS={SIG_BITS}  n={len(outs)}  "
        f"max|err|={np.max(np.abs(err)):.3e}  "
        f"mean_signed={np.mean(err):+.3e}  "
        f"predicted_lsb=2^-{SIG_BITS}={2.0**-SIG_BITS:.3e}"
    )


def main():
    sim = os.getenv("SIM", "icarus")
    sources = sources_for("axis_fixed_sqrt")
    runner = get_runner(sim)
    runner.build(sources=sources, hdl_toplevel="axis_fixed_sqrt", always=True,
                 build_args=["-Wall", "-I", str(proj_path / "hdl")],
                 parameters={"SIG_BITS": SIG_BITS}, timescale=("1ns", "1ps"))
    runner.test(hdl_toplevel="axis_fixed_sqrt", test_module=test_file)


if __name__ == "__main__":
    main()
