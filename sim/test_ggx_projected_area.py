#!/usr/bin/env python3
import cocotb
import os
import sys
import numpy as np
from pathlib import Path
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge, ReadOnly
from cocotb.runner import get_runner
from cocotb.utils import get_sim_time as gst
from cocotb_bus.bus import Bus
from cocotb_bus.drivers import BusDriver
from cocotb_bus.monitors import BusMonitor
from sources import sources_for

test_file = os.path.basename(__file__).replace(".py", "")
proj_path = Path(__file__).resolve().parent.parent

FRAC_BITS = 32
Q31 = 2**31
ADDR_BITS = 10


class AXISMonitor(BusMonitor):
    transactions = 0

    def __init__(self, dut, name, clk, callback=None):
        self._signals = ["axis_tvalid", "axis_tready", "axis_tlast", "axis_tdata", "axis_tstrb"]
        BusMonitor.__init__(self, dut, name, clk, callback=callback)
        self.clock = clk
        self.transactions = 0

    async def _monitor_recv(self):
        rising_edge = RisingEdge(self.clock)
        falling_edge = FallingEdge(self.clock)
        read_only = ReadOnly()
        while True:
            await rising_edge
            await falling_edge
            await read_only
            if self.bus.axis_tvalid.value and self.bus.axis_tready.value:
                self.transactions += 1
                self._recv(
                    {
                        "data": int(self.bus.axis_tdata.value),
                        "last": int(self.bus.axis_tlast.value),
                    }
                )


class AXISDriver(BusDriver):
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
        elif role == "S":
            self.bus.axis_tready.value = 0
        else:
            raise ValueError("role must be M or S")

    async def _driver_send(self, value, sync=True):
        rising_edge = RisingEdge(self.clock)
        falling_edge = FallingEdge(self.clock)
        read_only = ReadOnly()
        if self.role == "M":
            if value.get("type") == "write_burst":
                data = value.get("contents").get("data")
                for i, d in enumerate(data):
                    await self._driver_send(
                        {
                            "type": "write_beat",
                            "data": int(d),
                            "last": 1 if i == len(data) - 1 else 0,
                        },
                        sync=sync,
                    )
            elif value.get("type") == "write_beat":
                await falling_edge
                self.bus.axis_tdata.value = int(value.get("data", 0))
                self.bus.axis_tstrb.value = (1 << len(self.bus.axis_tstrb)) - 1
                self.bus.axis_tlast.value = int(value.get("last", 0))
                self.bus.axis_tvalid.value = 1
                while True:
                    await read_only
                    if self.bus.axis_tready.value == 1:
                        break
                    await rising_edge
                    await falling_edge
                await rising_edge
                self.bus.axis_tvalid.value = 0
                self.bus.axis_tlast.value = 0
            elif value.get("type") == "pause":
                await falling_edge
                self.bus.axis_tvalid.value = 0
                await ClockCycles(self.clock, value.get("duration", 1))
        else:
            if value.get("type") == "read_burst":
                for _ in range(value.get("duration", 1)):
                    await falling_edge
                    self.bus.axis_tready.value = 1
                    await read_only
                    if self.bus.axis_tvalid.value == 0:
                        await RisingEdge(self.bus.axis_tvalid)
                    await rising_edge
            elif value.get("type") == "pause":
                await falling_edge
                self.bus.axis_tready.value = 0
                await ClockCycles(self.clock, value.get("duration", 1))
            elif value.get("type") == "ready_high":
                for _ in range(value.get("duration", 1)):
                    await falling_edge
                    self.bus.axis_tready.value = 1
                    await rising_edge
            elif value.get("type") == "random_ready":
                rng = np.random.default_rng(value.get("seed", 1))
                ready_prob = float(value.get("ready_prob", 0.5))
                for _ in range(value.get("duration", 1)):
                    await falling_edge
                    self.bus.axis_tready.value = 1 if rng.random() < ready_prob else 0
                    await rising_edge


async def reset(clk, rst, cycles_held=3, polarity=0):
    rst.value = polarity
    await ClockCycles(clk, cycles_held)
    rst.value = int(not polarity)


def u32(x):
    return x & 0xFFFF_FFFF


def uq032_to_float(u):
    return float(u32(u)) / (2**32)


def q131_to_float(x):
    x = u32(x)
    if x & 0x8000_0000:
        x -= 0x1_0000_0000
    return float(x) / Q31


def float_to_uq032(f):
    if f <= 0.0:
        return 0
    if f >= 1.0:
        f = np.nextafter(1.0, 0.0)
    return int(np.floor(f * (2**32))) & 0xFFFF_FFFF


def float_to_q131(f):
    if f >= 1.0:
        f = np.nextafter(1.0, 0.0)
    if f < -1.0:
        f = -1.0
    v = int(np.round(f * Q31))
    if v > 0x7FFF_FFFF:
        v = 0x7FFF_FFFF
    if v < -0x8000_0000:
        v = -0x8000_0000
    return v & 0xFFFF_FFFF


def projected_ref(vhz_q131, u2_uq032, u1_uq032):
    vhz = q131_to_float(vhz_q131)
    u1 = uq032_to_float(u1_uq032)

    # Mirror the LUT behavior used in axis_trig_lut
    idx = (u32(u2_uq032) >> (32 - ADDR_BITS)) & ((1 << ADDR_BITS) - 1)
    angle = (idx / float(1 << ADDR_BITS)) * 2.0 * np.pi
    c = np.cos(angle)
    s = np.sin(angle)

    r = np.sqrt(max(0.0, u1))
    t1 = r * c
    t2 = r * s

    blend = 0.5 * (1.0 + vhz)
    t2 = (1.0 - blend) * np.sqrt(max(0.0, 1.0 - t1 * t1)) + blend * t2
    return t1, t2


def ensure_trig_rom(filename: Path):
    n = 1 << ADDR_BITS
    with open(filename, "w", encoding="ascii") as f:
        for i in range(n):
            ang = (i / n) * 2.0 * np.pi
            c = float_to_q131(np.cos(ang))
            s = float_to_q131(np.sin(ang))
            packed = ((u32(s) << 32) | u32(c)) & 0xFFFF_FFFF_FFFF_FFFF
            f.write(f"{packed:016x}\n")


@cocotb.test()
async def test_projected_area(dut):
    expected = []
    seen = {"n": 0, "out_last": 0}
    in_last_seen = {"n": 0}
    stats = {"max_d1": 0.0, "max_d2": 0.0}
    TOL = 1.2e-2

    def model_cb(transaction):
        packed = int(transaction["data"]) & ((1 << 96) - 1)
        exp_last = int(transaction["last"])
        u1 = packed & 0xFFFF_FFFF
        u2 = (packed >> 32) & 0xFFFF_FFFF
        vhz = (packed >> 64) & 0xFFFF_FFFF
        t1_f, t2_f = projected_ref(vhz, u2, u1)
        expected.append((t1_f, t2_f, vhz, u2, u1, exp_last))
        in_last_seen["n"] += exp_last

    def check_cb(transaction):
        if not expected:
            raise AssertionError("Output with empty expected queue")

        exp_t1, exp_t2, vhz, u2, u1, exp_last = expected.pop(0)
        packed = int(transaction["data"]) & 0xFFFF_FFFF_FFFF_FFFF
        got_last = int(transaction["last"])
        got_t1 = q131_to_float(packed & 0xFFFF_FFFF)
        got_t2 = q131_to_float((packed >> 32) & 0xFFFF_FFFF)

        d1 = abs(got_t1 - exp_t1)
        d2 = abs(got_t2 - exp_t2)
        stats["max_d1"] = max(stats["max_d1"], d1)
        stats["max_d2"] = max(stats["max_d2"], d2)
        seen["n"] += 1
        seen["out_last"] += got_last
        if seen["n"] <= 5:
            dut._log.info(
                f"#{seen['n']} in(vhz={q131_to_float(vhz):.4f}, u2={uq032_to_float(u2):.4f}, u1={uq032_to_float(u1):.4f}) "
                f"got(t1={got_t1:.5f}, t2={got_t2:.5f}) exp(t1={exp_t1:.5f}, t2={exp_t2:.5f})"
            )

        if got_last != exp_last:
            raise AssertionError(
                f"TLAST mismatch: got_last={got_last} exp_last={exp_last} at transaction {seen['n']}"
            )

        if d1 > TOL or d2 > TOL:
            raise AssertionError(
                f"Mismatch: got=({got_t1:.6f},{got_t2:.6f}) exp=({exp_t1:.6f},{exp_t2:.6f}) "
                f"d=({d1:.6f},{d2:.6f}) tol={TOL}"
            )

    inm = AXISMonitor(dut, "s00", dut.s00_axis_aclk, callback=model_cb)
    outm = AXISMonitor(dut, "m00", dut.s00_axis_aclk, callback=check_cb)
    ind = AXISDriver(dut, "s00", dut.s00_axis_aclk, "M")
    outd = AXISDriver(dut, "m00", dut.s00_axis_aclk, "S")

    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    await reset(dut.s00_axis_aclk, dut.s00_axis_aresetn, cycles_held=5, polarity=0)

    np.random.seed(7)
    rng_in = np.random.default_rng(17)
    payloads = []
    u1_near_one = np.nextafter(1.0, 0.0)
    corners = [
        (1.0, 0.0, 0.0),            # s -> 1.0
        (-1.0, 0.25, u1_near_one),  # s -> 0.0
        (1.0, 0.5, u1_near_one),    # cos ~ -1, t1 near -1
        (-1.0, 0.0, u1_near_one),   # cos ~ +1, t1 near +1
        (0.0, 0.75, u1_near_one),
        (0.5, 0.5, 0.5),
    ]
    for vhz_f, u2_f, u1_f in corners:
        payloads.append(
            (float_to_q131(vhz_f) << 64) |
            (float_to_uq032(u2_f) << 32) |
            float_to_uq032(u1_f)
        )

    for _ in range(320):
        vhz_f = float(np.random.uniform(-1.0, 1.0))
        u2_f = float(np.random.uniform(0.0, 0.999999))
        u1_f = float(np.random.uniform(0.0, 0.999999))
        payloads.append(
            (float_to_q131(vhz_f) << 64) |
            (float_to_uq032(u2_f) << 32) |
            float_to_uq032(u1_f)
        )

    for i, payload in enumerate(payloads):
        ind.append(
            {
                "type": "write_beat",
                "data": payload,
                "last": 1 if i == len(payloads) - 1 else 0,
            }
        )
        if i != len(payloads) - 1 and rng_in.random() < 0.45:
            ind.append({"type": "pause", "duration": int(rng_in.integers(1, 4))})

    # Stress sideband alignment and tail behavior with repeated stalls.
    outd.append({"type": "random_ready", "duration": 700, "ready_prob": 0.45, "seed": 23})
    outd.append({"type": "pause", "duration": 180})
    outd.append({"type": "random_ready", "duration": 900, "ready_prob": 0.35, "seed": 71})
    outd.append({"type": "ready_high", "duration": 2500})

    await ClockCycles(dut.s00_axis_aclk, 45 * len(payloads))

    assert len(expected) == 0, f"{len(expected)} expected transactions not observed"
    assert inm.transactions == outm.transactions, f"count mismatch in={inm.transactions} out={outm.transactions}"
    assert in_last_seen["n"] == 1, f"expected one input TLAST, saw {in_last_seen['n']}"
    assert seen["out_last"] == 1, f"expected one output TLAST, saw {seen['out_last']}"
    dut._log.info(
        f"projected_area stats: max|t1_err|={stats['max_d1']:.6f}, "
        f"max|t2_err|={stats['max_d2']:.6f}, tol={TOL:.6f}"
    )


def projected_area_runner():
    sim = os.getenv("SIM", "icarus")
    sys.path.append(str(proj_path / "hdl"))
    sys.path.append(str(proj_path / "sim"))

    rom_path = proj_path / "sim_build" / "ggx_trig_rom.mem"
    ensure_trig_rom(rom_path)

    sources = sources_for("axis_ggx_projected_area")

    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel="axis_ggx_projected_area",
        always=True,
        build_args=["-Wall", "-I", str(proj_path / "hdl")],
        timescale=("1ns", "1ps"),
        waves=True,
    )
    runner.test(
        hdl_toplevel="axis_ggx_projected_area",
        test_module=test_file,
        waves=True,
    )


if __name__ == "__main__":
    projected_area_runner()
