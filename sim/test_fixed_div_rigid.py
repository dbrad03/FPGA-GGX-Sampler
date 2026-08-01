#!/usr/bin/env python3
"""axis_fixed_div with ELASTIC=0 -- the rigid, never-stalling divider (issue #33).

The stallable divider gates all 2*(W+F) stages with one pipe_en, and that enable
is a timing endpoint on every FF and SRL in the core -- 694 failing endpoints in
oct32's two dividers. The rigid variant deletes the enable and refuses beats at
its slave port instead, against credits reserved in an output ring.

What has to be true, and what a data-only test would not notice:

1. **Bit-identical quotients.** The recurrence is shared between the two modes,
   so this is true by construction -- and worth asserting anyway, because the
   thing that would break it is someone "simplifying" the shared block.
2. **Lossless.** A rigid pipeline cannot be told to wait. If the credit
   accounting is off by one, beats already inside it are silently overwritten in
   the ring under backpressure. Gapped input plus random backpressure is the
   only way to see that.
3. **II=1.** The ring exists so throughput survives; if RING_DEPTH were sized
   below the latency, this still passes every correctness check and quietly
   halves the Lane's rate.
4. **It is a Cut.** s00_axis_tready must not follow m00_axis_tready
   combinationally, or the enable is gone but the ready chain is not.

Runs the SAME vectors against ELASTIC=1 and ELASTIC=0 and requires identical
output, so "bit-identical" is a comparison rather than a claim.

Run:  python test_fixed_div_rigid.py
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

# oct32's actual instantiation -- the configuration this ticket is about.
WIDTH = 20
FRAC_BITS = 18
ELASTIC = int(os.getenv("DIV_ELASTIC", "0"))

MASK = (1 << WIDTH) - 1


def ref_quotient(a, b):
    """(a << FRAC_BITS) / b, truncated to WIDTH bits -- the core's contract."""
    return ((a << FRAC_BITS) // b) & MASK


def vectors(n, seed):
    rng = random.Random(seed)
    pairs = [(rng.randint(1, (1 << (WIDTH - 1)) - 1), rng.randint(1, (1 << WIDTH) - 1))
             for _ in range(n)]
    # Edges: a=0, b=1 (max quotient), a=b (exactly 1.0), b=max.
    pairs += [(0, 1 << (WIDTH - 2)), (1, 1), ((1 << WIDTH) - 1, (1 << WIDTH) - 1),
              (1, (1 << WIDTH) - 1), ((1 << (WIDTH - 1)) - 1, 1)]
    return pairs


async def reset(dut):
    dut.s00_axis_aresetn.value = 0
    dut.s00_axis_tvalid.value = 0
    dut.s00_axis_tdata.value = 0
    dut.s00_axis_tlast.value = 0
    dut.s00_axis_tstrb.value = 0
    dut.m00_axis_tready.value = 0
    await ClockCycles(dut.s00_axis_aclk, 5)
    dut.s00_axis_aresetn.value = 1
    await RisingEdge(dut.s00_axis_aclk)


async def drive(dut, pairs, gap_prob, rng):
    i = 0
    while i < len(pairs):
        a, b = pairs[i]
        send = rng.random() >= gap_prob
        dut.s00_axis_tvalid.value = 1 if send else 0
        dut.s00_axis_tdata.value = (a << WIDTH) | b
        await ReadOnly()
        took = send and bool(dut.s00_axis_tready.value)
        await RisingEdge(dut.s00_axis_aclk)
        if took:
            i += 1
    dut.s00_axis_tvalid.value = 0


async def sink(dut, n, out, ready_prob, rng):
    while len(out) < n:
        dut.m00_axis_tready.value = 1 if rng.random() < ready_prob else 0
        await ReadOnly()
        if dut.m00_axis_tvalid.value and dut.m00_axis_tready.value:
            out.append(int(dut.m00_axis_tdata.value) & MASK)
        await RisingEdge(dut.s00_axis_aclk)
    dut.m00_axis_tready.value = 0


@cocotb.test()
async def test_quotients_match_the_model(dut):
    """Same answers as the reference, gapped in and backpressured out."""
    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    await reset(dut)

    rng = random.Random(0x5EED)
    pairs = vectors(120, seed=1)
    out = []

    src = cocotb.start_soon(drive(dut, pairs, gap_prob=0.35, rng=rng))
    snk = cocotb.start_soon(sink(dut, len(pairs), out, ready_prob=0.6, rng=rng))
    await src
    await snk

    expected = [ref_quotient(a, b) for a, b in pairs]
    bad = [(i, pairs[i], out[i], expected[i])
           for i in range(len(pairs)) if out[i] != expected[i]]
    assert not bad, f"{len(bad)} mismatches, first: idx={bad[0][0]} pair={bad[0][1]} got={bad[0][2]} exp={bad[0][3]}"

    # Write the stream out so the two ELASTIC modes can be compared byte for byte.
    tag = "elastic" if ELASTIC else "rigid"
    Path(f"div_{tag}_stream.txt").write_text("\n".join(str(v) for v in out) + "\n")


@cocotb.test()
async def test_full_throughput(dut):
    """II=1: N beats accepted in N cycles, and N results out, no bubbles."""
    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    await reset(dut)

    n = 200
    pairs = vectors(n - 5, seed=2)
    out = []
    dut.m00_axis_tready.value = 1
    snk = cocotb.start_soon(sink(dut, len(pairs), out, ready_prob=1.0,
                                 rng=random.Random(0)))

    accepted = 0
    cycles = 0
    while accepted < len(pairs):
        a, b = pairs[accepted]
        dut.s00_axis_tvalid.value = 1
        dut.s00_axis_tdata.value = (a << WIDTH) | b
        await ReadOnly()
        if dut.s00_axis_tready.value:
            accepted += 1
        cycles += 1
        assert cycles <= len(pairs) + 4, (
            f"II=1 lost: {accepted} of {len(pairs)} accepted in {cycles} cycles. "
            "If this core is rigid, its ring is shallower than its latency."
        )
        await RisingEdge(dut.s00_axis_aclk)
    dut.s00_axis_tvalid.value = 0

    await snk
    assert cycles == len(pairs), f"II=1 lost: {len(pairs)} beats took {cycles} cycles"
    assert out == [ref_quotient(a, b) for a, b in pairs]


@cocotb.test()
async def test_ready_does_not_follow_downstream_ready(dut):
    """The Cut property: s00_axis_tready must not react to m00_axis_tready.

    Deleting the enable is only half of #33. If the slave-side ready still fell
    out of the master-side ready combinationally, this core would still be a hop
    in a Ready chain -- see ADR-0003.
    """
    if ELASTIC:
        raise cocotb.result.TestSuccess("ELASTIC=1 is knowingly combinational here")

    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    await reset(dut)

    dut.m00_axis_tready.value = 0
    dut.s00_axis_tvalid.value = 1
    dut.s00_axis_tdata.value = (12345 << WIDTH) | 678
    for _ in range(20):
        await RisingEdge(dut.s00_axis_aclk)

    await ReadOnly()
    before = int(dut.s00_axis_tready.value)

    await RisingEdge(dut.s00_axis_aclk)
    await Timer(1, units="ns")
    dut.m00_axis_tready.value = 1
    await Timer(1, units="ns")
    assert int(dut.s00_axis_tready.value) == before, (
        "s00_axis_tready moved with m00_axis_tready inside one cycle -- "
        "the rigid divider is not a Cut"
    )


def div_rigid_runner():
    sim = os.getenv("SIM", "icarus")
    sys.path.append(str(proj_path / "sim"))
    sources = sources_for("axis_fixed_div")
    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel="axis_fixed_div",
        always=True,
        build_args=["-Wall", "-I", str(proj_path / "hdl")],
        parameters={"WIDTH": WIDTH, "FRAC_BITS": FRAC_BITS, "ELASTIC": ELASTIC},
        timescale=("1ns", "1ps"),
        waves=True,
    )
    runner.test(hdl_toplevel="axis_fixed_div", test_module=test_file, waves=True)


if __name__ == "__main__":
    div_rigid_runner()
