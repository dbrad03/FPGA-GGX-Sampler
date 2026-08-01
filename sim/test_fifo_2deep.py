#!/usr/bin/env python3
"""axis_fifo_2deep -- the Cut primitive.

This block is not interesting as arithmetic; it is interesting because the whole
design now leans on three properties of it (ADR-0003), and two of them are the
kind that a passing cascade would not notice breaking:

1. **Its ready is REGISTERED.** That is the entire reason it is used as a Cut. A
   version whose `s_axis_tready` fell out of `m_axis_tready` combinationally
   would still pass every data test in this repo and would silently re-connect
   the Ready chain that issue #31 measured at 875 failing endpoints.
2. **It is lossless with a ready that is one cycle late.** The second entry
   exists only for the beat already in flight when ready drops.
3. **It sustains II=1.** Elastic buffering is exactly the change that can
   silently cost a cycle per Sample.

Run:  python test_fifo_2deep.py
"""
import os
import random
import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.runner import get_runner
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge, Timer

from sources import sources_for

test_file = os.path.basename(__file__).replace(".py", "")
proj_path = Path(__file__).resolve().parent.parent

DATA_WIDTH = 64
MASK = (1 << DATA_WIDTH) - 1


async def reset(dut, cycles=3):
    dut.resetn.value = 0
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tdata.value = 0
    dut.s_axis_tlast.value = 0
    dut.m_axis_tready.value = 0
    await ClockCycles(dut.clk, cycles)
    await RisingEdge(dut.clk)
    dut.resetn.value = 1
    await RisingEdge(dut.clk)


@cocotb.test()
async def test_ready_is_registered(dut):
    """s_axis_tready must not react to m_axis_tready within the same cycle.

    Fill the FIFO so its ready is low, then wiggle m_axis_tready mid-cycle. A
    combinational ready would follow it; a registered one cannot move until the
    next edge. This is the property that makes this block a Cut.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset(dut)

    # Push two beats with the output held off, so the FIFO fills and drops ready.
    dut.m_axis_tready.value = 0
    for i in range(2):
        dut.s_axis_tvalid.value = 1
        dut.s_axis_tdata.value = i + 1
        await RisingEdge(dut.clk)
    dut.s_axis_tvalid.value = 0
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert dut.s_axis_tready.value == 0, "FIFO should be full and not ready"

    # Mid-cycle: assert m_axis_tready and settle combinationally. A registered
    # ready cannot have moved.
    await RisingEdge(dut.clk)
    await Timer(1, units="ns")
    dut.m_axis_tready.value = 1
    await Timer(1, units="ns")
    assert dut.s_axis_tready.value == 0, (
        "s_axis_tready followed m_axis_tready inside one cycle -- this FIFO is "
        "not a Cut, and the Ready chain of issue #31 is reconnected"
    )


@cocotb.test()
async def test_full_throughput(dut):
    """One beat per cycle, both ports firing, for the length of a Burst."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset(dut)

    n = 64
    got = []
    dut.m_axis_tready.value = 1

    async def sink():
        while len(got) < n:
            await ReadOnly()
            if dut.m_axis_tvalid.value and dut.m_axis_tready.value:
                got.append(int(dut.m_axis_tdata.value))
            await RisingEdge(dut.clk)

    task = cocotb.start_soon(sink())

    accepted = 0
    cycles = 0
    dut.s_axis_tvalid.value = 1
    while accepted < n:
        dut.s_axis_tdata.value = accepted + 1
        await ReadOnly()
        if dut.s_axis_tready.value:
            accepted += 1
        cycles += 1
        assert cycles <= n + 4, f"stalled: accepted {accepted} of {n} in {cycles} cycles"
        await RisingEdge(dut.clk)
    dut.s_axis_tvalid.value = 0

    await task
    assert got == list(range(1, n + 1)), f"data reordered or lost: {got[:8]}"
    assert cycles == n, f"II=1 lost: {n} beats took {cycles} cycles"


@cocotb.test()
async def test_lossless_under_random_backpressure(dut):
    """No loss, no reorder, TLAST preserved, with both ports gapped randomly."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset(dut)

    rng = random.Random(0xC0FFEE)
    n = 400
    sent = [((rng.getrandbits(DATA_WIDTH), 1 if (i % 37) == 36 else 0)) for i in range(n)]
    got = []

    async def source():
        i = 0
        while i < n:
            data, last = sent[i]
            drive = rng.random() < 0.7
            dut.s_axis_tvalid.value = 1 if drive else 0
            dut.s_axis_tdata.value = data
            dut.s_axis_tlast.value = last
            await ReadOnly()
            if drive and dut.s_axis_tready.value:
                i += 1
            await RisingEdge(dut.clk)
        dut.s_axis_tvalid.value = 0

    async def sink():
        while len(got) < n:
            dut.m_axis_tready.value = 1 if rng.random() < 0.6 else 0
            await ReadOnly()
            if dut.m_axis_tvalid.value and dut.m_axis_tready.value:
                got.append((int(dut.m_axis_tdata.value) & MASK,
                            int(dut.m_axis_tlast.value)))
            await RisingEdge(dut.clk)

    src = cocotb.start_soon(source())
    snk = cocotb.start_soon(sink())
    await src
    await snk

    assert got == sent, (
        "stream corrupted under backpressure: first mismatch at index "
        f"{next(i for i, (a, b) in enumerate(zip(got, sent)) if a != b)}"
    )


def fifo_2deep_runner():
    sim = os.getenv("SIM", "icarus")
    sources = sources_for("axis_fifo_2deep")
    build_test_args = ["-Wall", "-I", str(proj_path / "hdl")]
    sys.path.append(str(proj_path / "sim"))
    runner = get_runner(sim)
    hdl_toplevel = "axis_fifo_2deep"

    runner.build(
        sources=sources,
        hdl_toplevel=hdl_toplevel,
        always=True,
        build_args=build_test_args,
        parameters={"DATA_WIDTH": DATA_WIDTH},
        timescale=("1ns", "1ps"),
        waves=True,
    )
    runner.test(
        hdl_toplevel=hdl_toplevel,
        test_module=test_file,
        test_args=[],
        waves=True,
    )


if __name__ == "__main__":
    fifo_2deep_runner()
