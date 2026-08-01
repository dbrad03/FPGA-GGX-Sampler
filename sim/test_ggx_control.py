#!/usr/bin/env python3
import cocotb
import os
import sys
import numpy as np
from pathlib import Path
from functools import reduce
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

Q31 = 2**31
# Max tolerated |mean signed error| per component -- see the bias guard below.
BIAS_TOL = 8e-06
# Stream-integrity guard -- see the two-gate note in test_ggx_control(). This is
# NOT an Accuracy tolerance and must never be cited as numerical margin: the
# reference model behind it is float, not bit-faithful to the datapath, so its
# own disagreement with the fixed-point pipeline sets the floor here. Measured
# on a clean run: max deviation 1.871e-03 over 480 Samples. Set ~6.4x above
# that, which leaves it far below the O(0.1-1) shift that Skew, index
# mis-pairing or cross-burst leakage produce -- the defects this gate is for.
# Do NOT reuse the arbitrary 6.5e-2 TOL below for this; it is wide enough to
# pass substantial real corruption.
STREAM_TOL = 1.2e-2
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


def normalize3(v):
    v = np.asarray(v, dtype=np.float64)
    n = np.linalg.norm(v)
    if n < 1e-30:
        return np.array([0.0, 0.0, 0.0], dtype=np.float64)
    return (v / n).astype(np.float64)


def unpack_vec96_q131_xyz(packed96):
    packed96 &= (1 << 96) - 1
    x = packed96 & 0xFFFF_FFFF
    y = (packed96 >> 32) & 0xFFFF_FFFF
    z = (packed96 >> 64) & 0xFFFF_FFFF
    return np.array([q131_to_float(x), q131_to_float(y), q131_to_float(z)], dtype=np.float64)


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


def reverse_bits(x, nbits=32):
    out = 0
    for _ in range(nbits):
        out <<= 1
        out |= (x & 1)
        x >>= 1
    return u32(out)


def sobol_direction_numbers_2d(frac_bits=32):
    v = np.zeros((2, frac_bits), dtype=np.uint32)

    for k in range(frac_bits):
        v[0, k] = np.uint32(1 << (frac_bits - 1 - k))

    s = 2
    a = 1
    m = [1, 3]
    for i in range(s):
        v[1, i] = np.uint32(u32(m[i] << (frac_bits - 1 - i)))
    for i in range(s, frac_bits):
        val = u32(int(v[1, i - s])) ^ u32(int(v[1, i - s]) >> s)
        for j in range(1, s):
            if (a >> (s - 1 - j)) & 1:
                val ^= u32(int(v[1, i - j]) >> j)
        v[1, i] = np.uint32(u32(val))
    return v


def sobol_u32_stateless(dim, index, dir_table):
    index = u32(index)
    g = index ^ (index >> 1)
    x = 0
    bit = 0
    num_bits = dir_table.shape[1]
    while g and bit < num_bits:
        if g & 1:
            x ^= int(dir_table[dim][bit])
        g >>= 1
        bit += 1
    return u32(x)


def rotl32(x, r):
    x = u32(x)
    return u32((x << r) | (x >> (32 - r)))


def fmix32(h):
    h = u32(h)
    h ^= (h >> 16)
    h = u32(h * 0x85EBCA6B)
    h ^= (h >> 13)
    h = u32(h * 0xC2B2AE35)
    h ^= (h >> 16)
    return u32(h)


def mix(h, k):
    h = u32(h)
    k = u32(k)
    k = u32(k * 0xCC9E2D51)
    k = rotl32(k, 15)
    k = u32(k * 0x1B873593)
    h ^= k
    h = rotl32(h, 13)
    h = u32(h * 5 + 0xE6546B64)
    return u32(h)


def hash_combine(*vals):
    if len(vals) == 0:
        raise ValueError("hash_combine needs at least 1 value")
    h0 = 0x9747B28C
    h = reduce(mix, (u32(v) for v in vals), u32(h0))
    return fmix32(h)


def laine_karras_permutation(x, seed):
    x = u32(x + seed)
    x ^= u32(x * 0x6C50B47C)
    x ^= u32(x * 0xB82F1E52)
    x ^= u32(x * 0xC7AFE638)
    x ^= u32(x * 0x8D22F6E6)
    return x


def nested_uniform_scramble(x, seed):
    x = reverse_bits(x)
    x = laine_karras_permutation(x, seed)
    x = reverse_bits(x)
    return x


def scrambled_sobol2d(index, seed, direction_table):
    # Mirrors the RTL sampler exactly: axis_top_lvl_sampler emits the RAW
    # sample_idx, axis_sobol2d_stateless consumes it unshuffled, and only the
    # sobol OUTPUT is scrambled, under hash_combine(seed_base, DIMENSION).
    # There is deliberately no nested_uniform_scramble on the index -- an
    # earlier version of this model had one, which is why it disagreed with the
    # hardware by O(1). test_integration_dual scoreboards this same mapping.
    s0 = sobol_u32_stateless(0, index, direction_table)
    s1 = sobol_u32_stateless(1, index, direction_table)
    u0 = nested_uniform_scramble(s0, hash_combine(seed, 0))
    u1 = nested_uniform_scramble(s1, hash_combine(seed, 1))
    return u0, u1


def control_ref_sequence(seed, burst_len, vx_q, vy_q, vz_q, alpha_q, direction_table, cmd_id):
    vh, t1_vec, t2_vec = event_basis_ref(alpha_q, vx_q, vy_q, vz_q)
    vhz_q = float_to_q131(vh[2])
    out = []
    for i in range(int(burst_len) + 1):
        dim0_u1, dim1_u2 = scrambled_sobol2d(i, seed, direction_table)
        t1_f, t2_f = projected_ref(vhz_q, dim1_u2, dim0_u1)
        t3 = np.sqrt(max(0.0, 1.0 - t1_f * t1_f - t2_f * t2_f))
        h_unnorm = t1_f * t1_vec + t2_f * t2_vec + t3 * vh
        h = norm3_ref_like_rtl(h_unnorm)
        out.append(
            {
                "h": h,
                "last": 1 if i == int(burst_len) else 0,
                "cmd_id": int(cmd_id),
                "sample_idx": i,
            }
        )
    return out


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


def ensure_trig_rom(filename: Path):
    n = 1 << ADDR_BITS
    filename.parent.mkdir(parents=True, exist_ok=True)
    with open(filename, "w", encoding="ascii") as f:
        for i in range(n):
            ang = (i / n) * 2.0 * np.pi
            c = float_to_q131(np.cos(ang))
            s = float_to_q131(np.sin(ang))
            packed = ((u32(s) << 32) | u32(c)) & 0xFFFF_FFFF_FFFF_FFFF
            f.write(f"{packed:016x}\n")


def make_view(rng):
    v = rng.normal(size=3)
    v[2] = abs(v[2]) + 1e-3
    return normalize3(v)


@cocotb.test()
async def test_ggx_control(dut):
    # ------------------------------------------------------------------
    # This test runs TWO independent gates over one stimulus. Neither is
    # redundant; do not delete either.
    #
    #   Bias gate (expected_math / BIAS_TOL): builds its expected values from
    #     a monitor tapping the reproject input INSIDE the DUT. That coupling
    #     is what lets it hold mean_signed < 8e-6 and guard every fixed-point
    #     narrowing decision -- but it also means it verifies reproject+norm3
    #     against whatever Sample arrives, never that the arriving Sample
    #     sequence is right. It is blind to sampler corruption by construction.
    #
    #   Stream-integrity gate (stream_expected / STREAM_TOL): a true black box.
    #     The whole expected Sample sequence is built offline from the Command
    #     list before the run starts -- no monitor, no DUT signal contributes.
    #     Its job is the sequence, its index order and its Burst boundaries.
    #     Skew, index mis-pairing and cross-burst leakage move the output
    #     half-vector by O(0.1-1), far above the float reference's own error,
    #     so a loose bound catches them decisively. It is deliberately NOT a
    #     numerical instrument -- that is the Bias gate's job.
    #
    # A sampler Skew regression once kept this file green from end to end. The
    # stream gate exists so that cannot happen again; the bias gate exists
    # because the stream gate's float reference can never be that tight.
    # ------------------------------------------------------------------
    TOL = 6.5e-2
    expected_math = []
    burst_sizes = []
    seen = {"n": 0, "out_last": 0}
    stats = {"max_err": 0.0, "signed_sum": np.zeros(3), "abs_sum": np.zeros(3), "n_err": 0}
    parser = {"beats": [], "cmd_count": 0}
    out_state = {"cmd_idx": 0, "sample_idx": 0}
    throughput_cycles_cmd0 = []
    stream_expected = []
    stream_state = {"idx": 0, "max_dev": 0.0}
    stream_failures = []
    # Bias gate, relocated (issue #15). See the tap note below.
    expected_pre = []
    pre_stats = {"n": 0, "signed_sum": np.zeros(3), "abs_sum": np.zeros(3), "max_err": 0.0}

    def input_model_cb(transaction):
        data = int(transaction["data"]) & 0xFFFF_FFFF_FFFF_FFFF
        last = int(transaction["last"])
        parser["beats"].append((data, last))
        n = len(parser["beats"])

        if n == 1 and last != 0:
            raise AssertionError("Command beat0 must have TLAST=0")
        if n == 2 and last != 0:
            raise AssertionError("Command beat1 must have TLAST=0")
        if n < 3:
            return
        if n > 3:
            raise AssertionError("More than 3 beats observed before command completion")

        b0, b1, b2 = parser["beats"][0], parser["beats"][1], parser["beats"][2]
        if b2[1] != 1:
            raise AssertionError("Command beat2 must have TLAST=1")

        beat0 = b0[0]
        burst_len = (beat0 >> 32) & 0xFFFF
        burst_sizes.append(int(burst_len) + 1)
        parser["cmd_count"] += 1
        parser["beats"].clear()

    async def reproj_input_model():
        rising_edge = RisingEdge(dut.s00_axis_aclk)
        while True:
            await rising_edge
            if dut.reproj_in_valid.value and dut.reproj_in_ready.value:
                packed = int(dut.reproj_in_data.value) & ((1 << 352) - 1)
                t1 = q131_to_float(packed & 0xFFFF_FFFF)
                t2 = q131_to_float((packed >> 32) & 0xFFFF_FFFF)
                t1_vec = unpack_vec96_q131_xyz((packed >> 64) & ((1 << 96) - 1))
                t2_vec = unpack_vec96_q131_xyz((packed >> 160) & ((1 << 96) - 1))
                vh_vec = unpack_vec96_q131_xyz((packed >> 256) & ((1 << 96) - 1))

                t3 = np.sqrt(max(0.0, 1.0 - t1 * t1 - t2 * t2))
                h_unnorm = t1 * t1_vec + t2 * t2_vec + t3 * vh_vec
                expected_math.append(norm3_ref_like_rtl(h_unnorm))
                expected_pre.append(np.asarray(h_unnorm, dtype=np.float64))

    async def pre_encoder_bias_model():
        # RELOCATED BIAS GATE TAP (issue #15, ADR-0001 exception).
        #
        # This reaches INTO the DUT, at the un-normalized half-vector feeding
        # u_norm_h, rather than asserting at the output boundary. That is a
        # deliberate exception, not an oversight: once the Oct32 encoder
        # replaces the per-sample normalize, the 16-bit field quantization step
        # is ~3e-5 and swamps the 8e-6 systematic-bias threshold outright. The
        # gate cannot survive at the boundary, and it is the project's sharpest
        # instrument, so it moves inward instead of being weakened.
        #
        # Do NOT "fix" this back to the output boundary. The encoder is checked
        # separately by test_oct32_encode against the model in oct32_model.py.
        rising_edge = RisingEdge(dut.s00_axis_aclk)
        node = dut.u_reproject_normalize
        while True:
            await rising_edge
            if node.fifo_norm_valid.value and node.fifo_norm_ready.value:
                packed = int(node.fifo_norm_data.value) & ((1 << 96) - 1)
                got = unpack_vec96_q131_xyz(packed)
                if not expected_pre:
                    raise AssertionError("pre-encoder tap fired with no expectation queued")
                exp = expected_pre.pop(0)
                d = got - exp
                pre_stats["signed_sum"] += d
                pre_stats["abs_sum"] += np.abs(d)
                pre_stats["max_err"] = max(pre_stats["max_err"], float(np.max(np.abs(d))))
                pre_stats["n"] += 1

    def output_check_cb(transaction):
        # --- Stream-integrity gate ------------------------------------
        # Runs first and records rather than raises, so that a stream defect
        # is reported in full at the end of the run instead of aborting on the
        # first beat -- and so that the Bias gate still gets to render its own
        # verdict on the same run.
        si = stream_state["idx"]
        stream_state["idx"] = si + 1
        packed_h = unpack_vec96_q131_xyz(int(transaction["data"]) & ((1 << 96) - 1))
        if si >= len(stream_expected):
            stream_failures.append(
                f"STREAM-INTEGRITY GATE: output #{si} is past the end of the expected "
                f"sequence ({len(stream_expected)} Samples)"
            )
        else:
            ref = stream_expected[si]
            dev = float(np.max(np.abs(packed_h - ref["h"])))
            stream_state["max_dev"] = max(stream_state["max_dev"], dev)
            if dev > STREAM_TOL:
                stream_failures.append(
                    f"STREAM-INTEGRITY GATE: cmd={ref['cmd_id']} sample_idx={ref['sample_idx']} "
                    f"(out#{si}): dev={dev:.6f} > {STREAM_TOL:.1e} "
                    f"got=({packed_h[0]:+.5f},{packed_h[1]:+.5f},{packed_h[2]:+.5f}) "
                    f"exp=({ref['h'][0]:+.5f},{ref['h'][1]:+.5f},{ref['h'][2]:+.5f})"
                )
            if int(transaction["last"]) != ref["last"]:
                stream_failures.append(
                    f"STREAM-INTEGRITY GATE: cmd={ref['cmd_id']} sample_idx={ref['sample_idx']} "
                    f"(out#{si}): TLAST got={int(transaction['last'])} exp={ref['last']}"
                )

        # --- Bias gate ------------------------------------------------
        if not expected_math:
            raise AssertionError("Output with empty expected queue")
        if out_state["cmd_idx"] >= len(burst_sizes):
            raise AssertionError("Output observed before command was fully parsed")

        exp_h = expected_math.pop(0)
        packed = int(transaction["data"]) & ((1 << 96) - 1)
        got_last = int(transaction["last"])
        got_h = unpack_vec96_q131_xyz(packed)
        err = float(np.max(np.abs(got_h - exp_h)))
        stats["max_err"] = max(stats["max_err"], err)
        # Signed residual per component. Truncation (bit-slicing) biases toward
        # -inf, which max|err| cannot see; a non-zero mean here is systematic
        # error that Monte Carlo averaging will NOT remove.
        stats["signed_sum"] += (got_h - exp_h)
        stats["abs_sum"] += np.abs(got_h - exp_h)
        stats["n_err"] += 1

        exp_last = 1 if out_state["sample_idx"] == (burst_sizes[out_state["cmd_idx"]] - 1) else 0

        seen["n"] += 1
        seen["out_last"] += got_last
        if out_state["cmd_idx"] == 0:
            throughput_cycles_cmd0.append(int(transaction["cycle"]))

        if seen["n"] <= 5:
            dut._log.info(
                f"#{seen['n']} cmd={out_state['cmd_idx']} idx={out_state['sample_idx']} "
                f"got=({got_h[0]:.5f},{got_h[1]:.5f},{got_h[2]:.5f}) "
                f"exp=({exp_h[0]:.5f},{exp_h[1]:.5f},{exp_h[2]:.5f})"
            )

        if got_last != exp_last:
            raise AssertionError(
                f"TLAST mismatch at out#{seen['n']}: got={got_last} exp={exp_last} "
                f"(cmd={out_state['cmd_idx']} idx={out_state['sample_idx']})"
            )
        if err > TOL:
            raise AssertionError(
                f"Mismatch at out#{seen['n']}: max_err={err:.6f} tol={TOL:.6f} "
                f"(cmd={out_state['cmd_idx']} idx={out_state['sample_idx']})"
            )

        if exp_last:
            out_state["cmd_idx"] += 1
            out_state["sample_idx"] = 0
        else:
            out_state["sample_idx"] += 1

    inm = AXISMonitor(dut, "s00", dut.s00_axis_aclk, callback=input_model_cb)
    outm = AXISMonitor(dut, "m00", dut.s00_axis_aclk, callback=output_check_cb)
    ind = AXISDriver(dut, "s00", dut.s00_axis_aclk, "M")
    outd = AXISDriver(dut, "m00", dut.s00_axis_aclk, "S")

    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    cocotb.start_soon(reproj_input_model())
    cocotb.start_soon(pre_encoder_bias_model())
    await reset(dut.s00_axis_aclk, dut.s00_axis_aresetn, cycles_held=5, polarity=0)

    rng = np.random.default_rng(121)
    cmds = []
    params = [
        (127, 0x1234_5678, make_view(rng), 0.35),
        (191, 0x1020_3040, make_view(rng), 0.62),
        (159, 0xA5A5_55AA, make_view(rng), 0.12),
    ]
    # The stream-integrity gate's oracle: the entire expected Sample sequence,
    # built here from the Command list alone, before a single beat is driven.
    dir_table = sobol_direction_numbers_2d()
    for cmd_id, (burst_len, seed, view, alpha) in enumerate(params):
        vx_q = float_to_q131(view[0])
        vy_q = float_to_q131(view[1])
        vz_q = float_to_q131(view[2])
        alpha_q = float_to_uq032(alpha)
        stream_expected.extend(
            control_ref_sequence(seed, burst_len, vx_q, vy_q, vz_q, alpha_q, dir_table, cmd_id)
        )
        beat0 = ((burst_len & 0xFFFF) << 32) | u32(seed)
        beat1 = (u32(vy_q) << 32) | u32(vx_q)
        beat2 = (u32(alpha_q) << 32) | u32(vz_q)
        cmds.append((beat0, beat1, beat2))

    for cidx, (b0, b1, b2) in enumerate(cmds):
        ind.append({"type": "write_beat", "data": b0, "last": 0})
        if cidx > 0 and rng.random() < 0.5:
            ind.append({"type": "pause", "duration": int(rng.integers(1, 4))})
        ind.append({"type": "write_beat", "data": b1, "last": 0})
        if cidx > 0 and rng.random() < 0.5:
            ind.append({"type": "pause", "duration": int(rng.integers(1, 4))})
        ind.append({"type": "write_beat", "data": b2, "last": 1})
        if cidx != len(cmds) - 1:
            ind.append({"type": "pause", "duration": int(rng.integers(1, 6))})

    outd.append({"type": "ready_high", "duration": 800})
    outd.append({"type": "random_ready", "duration": 2200, "ready_prob": 0.42, "seed": 13})
    outd.append({"type": "pause", "duration": 150})
    outd.append({"type": "random_ready", "duration": 2200, "ready_prob": 0.35, "seed": 57})
    outd.append({"type": "ready_high", "duration": 6000})

    expected_total = sum((p[0] + 1) for p in params)
    await ClockCycles(dut.s00_axis_aclk, 120 * expected_total)

    assert len(parser["beats"]) == 0, "Dangling partial command beats at end of test"

    dut._log.info(
        f"stream-integrity gate: samples={stream_state['idx']}/{len(stream_expected)} "
        f"max_dev={stream_state['max_dev']:.6f} tol={STREAM_TOL:.1e} "
        f"mismatches={len(stream_failures)}"
    )
    # Systematic-bias guard. Truncating fixed-point narrowing biases toward -inf,
    # and unlike random noise that bias does NOT average out under Monte Carlo
    # integration -- it is a permanent error in the sampled distribution. max_err
    # is structurally blind to it, so assert on the signed mean directly.
    # Measured with round-half-up narrowing: (+5.8e-07, -2.7e-06, -1.3e-06).
    # This threshold is a regression guard on the ROUNDING, not a physical budget.
    n_err = max(stats["n_err"], 1)
    mean_signed = stats["signed_sum"] / n_err
    assert np.all(np.abs(mean_signed) < BIAS_TOL), (
        f"BIAS GATE: systematic bias exceeded {BIAS_TOL:.1e}: mean_signed="
        f"({mean_signed[0]:+.3e},{mean_signed[1]:+.3e},{mean_signed[2]:+.3e}). "
        "A fixed-point width change has likely reintroduced truncation in place of "
        "round-half-up; check the rnd_* helpers and any bare >>> on a product."
    )

    # --- Stream-integrity gate verdict --------------------------------
    # Asserted after the Bias gate so that a stream defect leaves the Bias
    # gate's verdict on the record: the two gates are independent, and which
    # one fired tells you whether the arithmetic or the Sample stream broke.
    # A truncated stream must fail here rather than pass vacuously, so every
    # expected Sample has to have been observed.
    assert stream_state["idx"] == len(stream_expected), (
        f"STREAM-INTEGRITY GATE: observed {stream_state['idx']} Samples, expected "
        f"{len(stream_expected)} -- the output stream is truncated or over-long"
    )
    assert not stream_failures, (
        f"STREAM-INTEGRITY GATE: {len(stream_failures)} mismatch(es) against the "
        f"independent Command-derived reference (max_dev={stream_state['max_dev']:.6f}, "
        f"tol={STREAM_TOL:.1e}; first 8 shown):\n  " + "\n  ".join(stream_failures[:8])
    )

    # --- Bias gate at the relocated pre-encoder tap (issue #15) ----------
    assert pre_stats["n"] > 0, "pre-encoder bias tap never fired -- the tap is broken, not the design"
    pre_mean = pre_stats["signed_sum"] / pre_stats["n"]
    dut._log.info(
        f"BIAS GATE (pre-encoder tap): n={pre_stats['n']} "
        f"mean_signed=({pre_mean[0]:+.3e},{pre_mean[1]:+.3e},{pre_mean[2]:+.3e}) "
        f"mean_abs=({pre_stats['abs_sum'][0]/pre_stats['n']:.3e},"
        f"{pre_stats['abs_sum'][1]/pre_stats['n']:.3e},"
        f"{pre_stats['abs_sum'][2]/pre_stats['n']:.3e}) "
        f"max|err|={pre_stats['max_err']:.3e}"
    )
    assert np.all(np.abs(pre_mean) < BIAS_TOL), (
        f"BIAS GATE (pre-encoder tap): systematic bias exceeded {BIAS_TOL:.1e}: "
        f"mean_signed=({pre_mean[0]:+.3e},{pre_mean[1]:+.3e},{pre_mean[2]:+.3e})"
    )

    assert parser["cmd_count"] == len(cmds), f"expected {len(cmds)} commands, saw {parser['cmd_count']}"
    assert len(burst_sizes) == len(cmds), f"expected {len(cmds)} parsed bursts, saw {len(burst_sizes)}"
    assert len(expected_math) == 0, f"{len(expected_math)} expected output samples not observed"
    assert outm.transactions == expected_total, f"output count mismatch {outm.transactions} != {expected_total}"
    assert seen["out_last"] == len(cmds), f"expected {len(cmds)} output TLASTs, saw {seen['out_last']}"
    assert out_state["cmd_idx"] == len(cmds), f"output stream closed only {out_state['cmd_idx']} bursts"
    assert out_state["sample_idx"] == 0, "output burst index not aligned at end of test"

    assert len(throughput_cycles_cmd0) == burst_sizes[0], (
        f"first command output count mismatch: saw {len(throughput_cycles_cmd0)} expected {burst_sizes[0]}"
    )
    cmd0_gaps = [b - a for a, b in zip(throughput_cycles_cmd0, throughput_cycles_cmd0[1:])]
    print(f"DEBUG_GAPS: cmd0_gaps={cmd0_gaps}")
    bubble_idx = [i for i, g in enumerate(cmd0_gaps, start=1) if g != 1]
    assert not bubble_idx, f"first-command throughput bubbles at output indices {bubble_idx[:8]}"

    dut._log.info(
        f"axis_ggx_control stats: outputs={seen['n']} max|component_err|={stats['max_err']:.6f} tol={TOL:.6f}"
        f" | mean_signed=({stats['signed_sum'][0]/max(stats['n_err'],1):+.3e},"
        f"{stats['signed_sum'][1]/max(stats['n_err'],1):+.3e},"
        f"{stats['signed_sum'][2]/max(stats['n_err'],1):+.3e})"
        f" mean_abs=({stats['abs_sum'][0]/max(stats['n_err'],1):.3e},"
        f"{stats['abs_sum'][1]/max(stats['n_err'],1):.3e},"
        f"{stats['abs_sum'][2]/max(stats['n_err'],1):.3e})"
    )


def ggx_control_runner():
    sim = os.getenv("SIM", "icarus")
    sys.path.append(str(proj_path / "hdl"))
    sys.path.append(str(proj_path / "sim"))

    ensure_inv_sqrt_rom(proj_path / "sim" / "sim_build" / "inv_sqrt_rom.mem", addr_bits=14)
    ensure_trig_rom(proj_path / "sim" / "sim_build" / "ggx_trig_rom.mem")

    sources = sources_for("axis_ggx_control")

    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel="axis_ggx_control",
        always=True,
        build_args=["-Wall", "-I", str(proj_path / "hdl")],
        timescale=("1ns", "1ps"),
        waves=True,
    )
    runner.test(
        hdl_toplevel="axis_ggx_control",
        test_module=test_file,
        waves=True,
    )


if __name__ == "__main__":
    ggx_control_runner()
