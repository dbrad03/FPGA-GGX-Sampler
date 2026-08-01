#!/usr/bin/env python3
"""Unit test for axis_oct32_encode against the issue #13 Python model.

Asserts at the module's own AXI-Stream boundary. The oracle is
sim/oct32_model.py -- the same model that justified the 16-bit field width --
so the RTL is checked against the thing the width decision was made on, not
against a second hand-written copy of the same idea.

The bound is MEASURED, not chosen: a clean run's worst field error is recorded
and the threshold sits just above it, with the reason for its size written down.
"""
import cocotb
import os
import sys
import numpy as np
from pathlib import Path
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge, ReadOnly
from cocotb.runner import get_runner
from cocotb_bus.drivers import BusDriver
from cocotb_bus.monitors import BusMonitor
from sources import sources_for
from oct32_model import oct_encode, oct_decode, angle_deg

test_file = os.path.basename(__file__).replace(".py", "")
proj_path = Path(__file__).resolve().parent.parent

# The encoder divides scaled integers, so its (u, w) carry a little truncation
# below the model's exact ratio before the field mapping rounds. That shows up
# as at most a small number of field LSBs. Measured on a clean run (see the
# assertion message); the bound is set just above what was observed.
# MEASURED on a clean run over 241 vectors: worst field error 1 LSB, worst
# decoded-direction error 0.0073 deg. Both bounds sit just above that. The 1 LSB
# comes from the divider truncating its quotient below the model's exact ratio,
# which can tip a value across a field boundary.
FIELD_TOL = 2          # LSBs of a 16-bit field (observed max 1)
ANGLE_TOL_DEG = 0.015  # decoded direction (observed max 0.0073)


def q131(f):
    v = int(np.round(np.clip(f, -1.0, 0.9999999995) * (2**31)))
    return v & 0xFFFF_FFFF


class Mon(BusMonitor):
    def __init__(self, dut, name, clk, callback=None):
        self._signals = ["axis_tvalid", "axis_tready", "axis_tlast", "axis_tdata", "axis_tstrb"]
        BusMonitor.__init__(self, dut, name, clk, callback=callback)
        self.clock = clk
        self.transactions = 0

    async def _monitor_recv(self):
        while True:
            await RisingEdge(self.clock)
            if self.bus.axis_tvalid.value and self.bus.axis_tready.value:
                self.transactions += 1
                self._recv(int(self.bus.axis_tdata.value))


class Drv(BusDriver):
    def __init__(self, dut, name, clk, role="M"):
        self._signals = ["axis_tvalid", "axis_tready", "axis_tlast", "axis_tdata", "axis_tstrb"]
        BusDriver.__init__(self, dut, name, clk)
        self.clock = clk
        self.role = role
        if role == "M":
            self.bus.axis_tdata.value = 0
            self.bus.axis_tstrb.value = 0
            self.bus.axis_tlast.value = 0
            self.bus.axis_tvalid.value = 0
        else:
            self.bus.axis_tready.value = 0

    async def _driver_send(self, value, sync=True):
        if self.role == "M":
            if value.get("type") == "burst":
                for d in value["data"]:
                    await FallingEdge(self.clock)
                    self.bus.axis_tdata.value = int(d)
                    self.bus.axis_tstrb.value = (1 << len(self.bus.axis_tstrb)) - 1
                    self.bus.axis_tvalid.value = 1
                    await ReadOnly()
                    if self.bus.axis_tready.value == 0:
                        await RisingEdge(self.bus.axis_tready)
                    await RisingEdge(self.clock)
                await FallingEdge(self.clock)
                self.bus.axis_tvalid.value = 0
        else:
            if value.get("type") == "ready":
                for _ in range(value.get("duration", 1)):
                    await FallingEdge(self.clock)
                    self.bus.axis_tready.value = 1
                    await RisingEdge(self.clock)
            elif value.get("type") == "pause":
                await FallingEdge(self.clock)
                self.bus.axis_tready.value = 0
                await ClockCycles(self.clock, value.get("duration", 1))


def build_vectors():
    """Directed coverage first, then random. Names travel with the vectors so a
    failure says which case broke."""
    named = []
    # Axis-aligned: the octahedron's poles and equator vertices.
    for v, n in [((0, 0, 1), "+z pole"), ((0, 0, -1), "-z pole (fold)"),
                 ((1, 0, 0), "+x vertex"), ((-1, 0, 0), "-x vertex"),
                 ((0, 1, 0), "+y vertex"), ((0, -1, 0), "-y vertex")]:
        named.append((np.array(v, dtype=float), n))
    # Octahedron EDGE: |x|+|y|+|z| with one component zero -> diamond boundary.
    for sx in (1, -1):
        for sy in (1, -1):
            named.append((np.array([sx * 0.5, sy * 0.5, 0.0]), f"edge z=0 ({sx},{sy})"))
    # CORNERS of the unit square in (u,w): the fold's diagonal extremes.
    for sx in (1, -1):
        for sy in (1, -1):
            named.append((np.array([sx / 3, sy / 3, -1 / 3]), f"corner fold ({sx},{sy})"))
    # Just either side of the z=0 fold boundary.
    for eps, n in [(1e-6, "z just above fold"), (-1e-6, "z just below fold")]:
        named.append((np.array([0.6, -0.3, eps]), n))
    # Scale invariance: same direction, wildly different magnitudes. Scales are
    # capped so every component stays inside Q1.31's [-1, 1) -- a larger scale
    # would be clipped by the input format itself, which changes the direction
    # and would be testing the clip rather than the encoder.
    for s in (1e-3, 1e-2, 0.25, 1.0, 1.6):
        named.append((np.array([0.4, -0.5, 0.6]) * s, f"scaled x{s}"))

    rng = np.random.default_rng(20260731)
    for i in range(220):
        v = rng.normal(size=3)
        v /= np.linalg.norm(v)
        # Un-normalized on purpose -- the encoder must not care about scale --
        # but every component has to stay inside Q1.31's [-1, 1), which is the
        # input format's own domain. Scaling past that tests the clip, not the
        # encoder.
        v *= rng.uniform(0.2, 0.99 / np.max(np.abs(v)))
        named.append((v, f"random#{i}"))
    return named


@cocotb.test()
async def test_oct32_encode(dut):
    vectors = build_vectors()
    got = []
    outm = Mon(dut, "m00", dut.s00_axis_aclk, callback=lambda d: got.append(int(d)))
    ind = Drv(dut, "s00", dut.s00_axis_aclk, "M")
    outd = Drv(dut, "m00", dut.s00_axis_aclk, "S")

    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    dut.s00_axis_aresetn.value = 0
    await ClockCycles(dut.s00_axis_aclk, 5)
    dut.s00_axis_aresetn.value = 1
    await ClockCycles(dut.s00_axis_aclk, 2)

    packed = []
    for v, _ in vectors:
        packed.append((q131(v[2]) << 64) | (q131(v[1]) << 32) | q131(v[0]))
    ind.append({"type": "burst", "data": packed})

    # Backpressure: a stalling consumer must not drop or duplicate a Sample.
    for _ in range(40):
        outd.append({"type": "ready", "duration": 9})
        outd.append({"type": "pause", "duration": 3})
    outd.append({"type": "ready", "duration": 4000})

    await ClockCycles(dut.s00_axis_aclk, 400 + 14 * len(vectors))

    assert len(got) == len(vectors), (
        f"expected {len(vectors)} encoded Samples, saw {len(got)} -- the encoder "
        f"dropped or duplicated under backpressure"
    )

    worst_field, worst_ang, worst_name = 0, 0.0, ""
    for (v, name), out in zip(vectors, got):
        gu, gw = out & 0xFFFF, (out >> 16) & 0xFFFF
        eu, ew = oct_encode(v, 16)
        du = abs(int(gu) - int(eu))
        dw = abs(int(gw) - int(ew))
        if max(du, dw) > worst_field:
            worst_field, worst_name = max(du, dw), name
        # What actually matters is the direction the fields decode to.
        ang = float(angle_deg(oct_decode(gu, gw, 16),
                              oct_decode(eu, ew, 16)))
        worst_ang = max(worst_ang, ang)
        assert du <= FIELD_TOL and dw <= FIELD_TOL, (
            f"{name}: field mismatch u {gu} vs {eu} (d={du}), w {gw} vs {ew} (d={dw}); "
            f"tol {FIELD_TOL} LSB"
        )
        assert ang <= ANGLE_TOL_DEG, f"{name}: decoded direction off by {ang:.5f} deg"

    dut._log.info(
        f"oct32 encoder: {len(got)} Samples, worst field error {worst_field} LSB "
        f"(at {worst_name}), worst decoded angle {worst_ang:.6f} deg"
    )


def runner():
    sys.path.append(str(proj_path / "sim"))
    r = get_runner(os.getenv("SIM", "icarus"))
    r.build(sources=sources_for("axis_oct32_encode"), hdl_toplevel="axis_oct32_encode",
            always=True, build_args=["-I", str(proj_path / "hdl")], timescale=("1ns", "1ps"))
    r.test(hdl_toplevel="axis_oct32_encode", test_module=test_file)


if __name__ == "__main__":
    runner()
