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

test_file = os.path.basename(__file__).replace(".py", "")
proj_path = Path(__file__).resolve().parent.parent

FRAC_BITS = 32
Q31 = 2**31
ADDR_BITS = 10
NORM_X_MIN = 2**-15


class AXISMonitor(BusMonitor):
    transactions = 0

    def __init__(self, dut, name, clk, callback=None):
        self._signals = ["axis_tvalid", "axis_tready", "axis_tlast", "axis_tdata", "axis_tstrb"]
        BusMonitor.__init__(self, dut, name, clk, callback=callback)
        self.clock = clk
        self.transactions = 0

    async def _monitor_recv(self):
        rising_edge = RisingEdge(self.clock)
        while True:
            await rising_edge
            if self.bus.axis_tvalid.value and self.bus.axis_tready.value:
                self.transactions += 1
                self._recv(
                    {
                        "data": int(self.bus.axis_tdata.value),
                        "last": int(self.bus.axis_tlast.value),
                        "cycle": int(gst(units="ns") // 10),
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


def unpack_vec96_q131_xyz(packed96):
    packed96 &= (1 << 96) - 1
    x = packed96 & 0xFFFF_FFFF
    y = (packed96 >> 32) & 0xFFFF_FFFF
    z = (packed96 >> 64) & 0xFFFF_FFFF
    return np.array([q131_to_float(x), q131_to_float(y), q131_to_float(z)], dtype=np.float64)


def pack_vec96_q131_xyz(v):
    x = float_to_q131(float(v[0]))
    y = float_to_q131(float(v[1]))
    z = float_to_q131(float(v[2]))
    return ((u32(z) << 64) | (u32(y) << 32) | u32(x)) & ((1 << 96) - 1)


def normalize3(v):
    v = np.asarray(v, dtype=np.float64)
    n = np.linalg.norm(v)
    if n < 1e-30:
        return np.array([0.0, 0.0, 0.0], dtype=np.float64)
    return (v / n).astype(np.float64)


def inv_sqrt_ref(x):
    x_min = 2**-15
    s = 0.25
    if x < x_min:
        x = x_min
    return s / np.sqrt(x)


def norm3_ref_like_rtl(v):
    v = np.asarray(v, dtype=np.float64)
    lensq = float(np.dot(v, v))
    if lensq < 1e-30:
        n = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    else:
        k = 0
        lensq_scaled = lensq
        while lensq_scaled >= 1.0:
            lensq_scaled *= 0.25
            k += 1
        lensq_scaled = max(lensq_scaled, NORM_X_MIN)
        inv_sqrt_scaled = inv_sqrt_ref(lensq_scaled)
        inv_len = (inv_sqrt_scaled * 4.0) / (2.0**k)
        n = v * inv_len

    return np.array(
        [
            q131_to_float(float_to_q131(n[0])),
            q131_to_float(float_to_q131(n[1])),
            q131_to_float(float_to_q131(n[2])),
        ],
        dtype=np.float64,
    )


def event_basis_ref(alpha_uq032, vx_q131, vy_q131, vz_q131):
    alpha = uq032_to_float(alpha_uq032)
    view = normalize3([q131_to_float(vx_q131), q131_to_float(vy_q131), q131_to_float(vz_q131)])
    vh = normalize3([alpha * view[0], alpha * view[1], view[2]])

    lensq = vh[0] * vh[0] + vh[1] * vh[1]
    if lensq > 1e-20:
        inv_len = inv_sqrt_ref(max(lensq, NORM_X_MIN)) * 4.0
        t1 = np.array([-vh[1] * inv_len, vh[0] * inv_len, 0.0], dtype=np.float64)
        t2 = np.cross(vh, t1)
    else:
        t1 = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        t2 = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    return vh, t1, t2


def projected_ref(vhz_q131, u2_uq032, u1_uq032):
    vhz = q131_to_float(vhz_q131)
    u1 = uq032_to_float(u1_uq032)

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


def build_consistent_payload(alpha_f, view_xyz, u2_f, u1_f):
    alpha_u = float_to_uq032(alpha_f)
    vx_q = float_to_q131(view_xyz[0])
    vy_q = float_to_q131(view_xyz[1])
    vz_q = float_to_q131(view_xyz[2])
    vh, t1_vec, t2_vec = event_basis_ref(alpha_u, vx_q, vy_q, vz_q)
    vhz_q = float_to_q131(vh[2])
    u2_u = float_to_uq032(u2_f)
    u1_u = float_to_uq032(u1_f)
    t1_f, t2_f = projected_ref(vhz_q, u2_u, u1_u)

    payload = (
        (pack_vec96_q131_xyz(vh) << 256)
        | (pack_vec96_q131_xyz(t2_vec) << 160)
        | (pack_vec96_q131_xyz(t1_vec) << 64)
        | (u32(float_to_q131(t2_f)) << 32)
        | u32(float_to_q131(t1_f))
    )
    return payload & ((1 << 352) - 1)


def make_random_view(rng):
    v = rng.normal(size=3)
    v[2] = abs(v[2]) + 1e-4
    return normalize3(v)


def ensure_inv_sqrt_rom(filename: Path, addr_bits: int = 14):
    n = 1 << addr_bits
    q = 25
    scale = 1 << q
    x_min = 2**-15
    s = 0.25
    filename.parent.mkdir(parents=True, exist_ok=True)
    with open(filename, "w", encoding="ascii") as f:
        for i in range(n):
            m = max((i + 0.5) / n, x_min)
            y = s / np.sqrt(m)
            y = min(y, 63.99999997)
            val = int(np.round(y * scale)) & 0xFFFF_FFFF
            f.write(f"{val:08x}\n")


@cocotb.test()
async def test_reproject_normalize(dut):
    expected = []
    seen = {"n": 0, "out_last": 0}
    in_last_seen = {"n": 0}
    throughput_cycles = []
    stats = {"max_err": 0.0, "max_norm_dev": 0.0}
    TOL = 6.5e-2
    THROUGHPUT_SAMPLES = 96

    def model_cb(transaction):
        packed = int(transaction["data"]) & ((1 << 352) - 1)
        exp_last = int(transaction["last"])

        t1 = q131_to_float(packed & 0xFFFF_FFFF)
        t2 = q131_to_float((packed >> 32) & 0xFFFF_FFFF)
        t1_vec = unpack_vec96_q131_xyz((packed >> 64) & ((1 << 96) - 1))
        t2_vec = unpack_vec96_q131_xyz((packed >> 160) & ((1 << 96) - 1))
        vh_vec = unpack_vec96_q131_xyz((packed >> 256) & ((1 << 96) - 1))

        t3 = np.sqrt(max(0.0, 1.0 - t1 * t1 - t2 * t2))
        h_unnorm = t1 * t1_vec + t2 * t2_vec + t3 * vh_vec
        h_norm = norm3_ref_like_rtl(h_unnorm)

        expected.append(
            {
                "h": h_norm,
                "last": exp_last,
                "in_t1": t1,
                "in_t2": t2,
                "in_vhz": vh_vec[2],
            }
        )
        in_last_seen["n"] += exp_last

    def check_cb(transaction):
        if not expected:
            raise AssertionError("Output with empty expected queue")

        exp = expected.pop(0)
        packed = int(transaction["data"]) & ((1 << 96) - 1)
        got_last = int(transaction["last"])
        got_h = unpack_vec96_q131_xyz(packed)

        err = np.max(np.abs(got_h - exp["h"]))
        got_norm = float(np.linalg.norm(got_h))
        exp_norm = float(np.linalg.norm(exp["h"]))
        norm_dev = abs(got_norm - exp_norm)
        stats["max_err"] = max(stats["max_err"], float(err))
        stats["max_norm_dev"] = max(stats["max_norm_dev"], float(norm_dev))
        seen["n"] += 1
        seen["out_last"] += got_last
        if seen["n"] <= THROUGHPUT_SAMPLES:
            throughput_cycles.append(int(transaction["cycle"]))

        if seen["n"] <= 5:
            dut._log.info(
                f"#{seen['n']} in(t1={exp['in_t1']:.5f}, t2={exp['in_t2']:.5f}, vhz={exp['in_vhz']:.5f}) "
                f"got(hx={got_h[0]:.5f}, hy={got_h[1]:.5f}, hz={got_h[2]:.5f}) "
                f"exp(hx={exp['h'][0]:.5f}, hy={exp['h'][1]:.5f}, hz={exp['h'][2]:.5f})"
            )

        if got_last != exp["last"]:
            raise AssertionError(
                f"TLAST mismatch: got_last={got_last} exp_last={exp['last']} at transaction {seen['n']}"
            )

        if err > TOL:
            raise AssertionError(
                f"Mismatch: got={got_h} exp={exp['h']} max_err={err:.6f} tol={TOL}"
            )

    inm = AXISMonitor(dut, "s00", dut.s00_axis_aclk, callback=model_cb)
    outm = AXISMonitor(dut, "m00", dut.s00_axis_aclk, callback=check_cb)
    ind = AXISDriver(dut, "s00", dut.s00_axis_aclk, "M")
    outd = AXISDriver(dut, "m00", dut.s00_axis_aclk, "S")

    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    await reset(dut.s00_axis_aclk, dut.s00_axis_aresetn, cycles_held=5, polarity=0)

    rng = np.random.default_rng(33)
    payloads = []

    directed = [
        (0.12, normalize3([0.0, 0.0, 1.0]), 0.0, 0.0),
        (0.30, normalize3([0.15, -0.35, 0.92]), 0.25, np.nextafter(1.0, 0.0)),
        (0.80, normalize3([0.95, 0.02, 0.30]), 0.50, np.nextafter(1.0, 0.0)),
        (0.95, normalize3([-0.80, 0.22, 0.56]), 0.75, 0.50),
        (0.55, normalize3([0.10, 0.98, 0.16]), 0.125, 0.75),
        (0.45, normalize3([-0.33, -0.56, 0.76]), 0.625, 0.12),
    ]
    for alpha, view, u2, u1 in directed:
        payloads.append(build_consistent_payload(alpha, view, u2, u1))

    while len(payloads) < THROUGHPUT_SAMPLES:
        alpha = float(rng.uniform(0.05, 0.98))
        view = make_random_view(rng)
        u2 = float(rng.uniform(0.0, 0.999999))
        u1 = float(rng.uniform(0.0, 0.999999))
        payloads.append(build_consistent_payload(alpha, view, u2, u1))

    for _ in range(220):
        alpha = float(rng.uniform(0.05, 0.98))
        view = make_random_view(rng)
        u2 = float(rng.uniform(0.0, 0.999999))
        u1 = float(rng.uniform(0.0, 0.999999))
        p = build_consistent_payload(alpha, view, u2, u1)
        if rng.random() < 0.08:
            t1_over = float(rng.choice([-0.96, -0.90, 0.90, 0.96]))
            t2_over = float(rng.choice([-0.96, -0.90, 0.90, 0.96]))
            p = (p & ~((1 << 64) - 1)) | (u32(float_to_q131(t2_over)) << 32) | u32(float_to_q131(t1_over))
        payloads.append(p & ((1 << 352) - 1))

    for i, payload in enumerate(payloads):
        ind.append(
            {
                "type": "write_beat",
                "data": payload,
                "last": 1 if i == len(payloads) - 1 else 0,
            }
        )
        if i >= THROUGHPUT_SAMPLES and i != len(payloads) - 1 and rng.random() < 0.40:
            ind.append({"type": "pause", "duration": int(rng.integers(1, 4))})

    outd.append({"type": "ready_high", "duration": 1600})
    outd.append({"type": "random_ready", "duration": 1700, "ready_prob": 0.45, "seed": 17})
    outd.append({"type": "pause", "duration": 120})
    outd.append({"type": "random_ready", "duration": 1800, "ready_prob": 0.35, "seed": 41})
    outd.append({"type": "ready_high", "duration": 4500})

    await ClockCycles(dut.s00_axis_aclk, 70 * len(payloads))

    assert len(expected) == 0, f"{len(expected)} expected transactions not observed"
    assert inm.transactions == outm.transactions, f"count mismatch in={inm.transactions} out={outm.transactions}"
    assert in_last_seen["n"] == 1, f"expected one input TLAST, saw {in_last_seen['n']}"
    assert seen["out_last"] == 1, f"expected one output TLAST, saw {seen['out_last']}"
    assert len(throughput_cycles) == THROUGHPUT_SAMPLES, (
        f"need {THROUGHPUT_SAMPLES} throughput samples, saw {len(throughput_cycles)}"
    )

    gaps = [b - a for a, b in zip(throughput_cycles, throughput_cycles[1:])]
    bubble_idx = [i for i, g in enumerate(gaps, start=1) if g != 1]
    assert not bubble_idx, f"throughput bubbles before backpressure at output indices {bubble_idx[:8]}"
    dut._log.info(
        f"reproject_normalize stats: max|component_err|={stats['max_err']:.6f}, "
        f"max|norm_dev|={stats['max_norm_dev']:.6f}, tol={TOL:.6f}"
    )


def reproject_normalize_runner():
    sim = os.getenv("SIM", "icarus")
    sys.path.append(str(proj_path / "hdl"))
    sys.path.append(str(proj_path / "sim"))

    rom_path = proj_path / "sim_build" / "inv_sqrt_rom.mem"
    ensure_inv_sqrt_rom(rom_path, addr_bits=14)

    sources = [
        proj_path / "hdl" / "axis_ggx_reproject_normalize.sv",
        proj_path / "hdl" / "axis_fixed_sqrt.sv",
        proj_path / "hdl" / "axis_fixed_div.sv",
        proj_path / "hdl" / "axis_fixed_inv_sqrt_nodsp.sv",
        proj_path / "hdl" / "axis_fixed_norm3.sv",
    ]

    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel="axis_ggx_reproject_normalize",
        always=True,
        build_args=["-Wall", "-I", str(proj_path / "hdl")],
        timescale=("1ns", "1ps"),
        waves=True,
    )
    runner.test(
        hdl_toplevel="axis_ggx_reproject_normalize",
        test_module=test_file,
        waves=True,
    )


if __name__ == "__main__":
    reproject_normalize_runner()
