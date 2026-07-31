#!/usr/bin/env python3
import cocotb
import os
import random
import sys
import logging
import numpy as np
from math import log
from pathlib import Path
from cocotb.clock import Clock
from cocotb.triggers import Timer, ClockCycles, RisingEdge, FallingEdge, ReadOnly, with_timeout
from cocotb.runner import get_runner
from cocotb.utils import get_sim_time as gst
from cocotb_bus.bus import Bus
from cocotb_bus.drivers import BusDriver
from cocotb_bus.monitors import BusMonitor
from cocotb_bus.scoreboard import Scoreboard
from cocotb.binary import BinaryValue
from sources import sources_for

test_file = os.path.basename(__file__).replace(".py","")
proj_path = Path(__file__).resolve().parent.parent

class AXISMonitor(BusMonitor):
    """
    monitors axi streaming bus
    """
    transactions = 0 #use this variable to track good ready/valid handshakes
    def __init__(self, dut, name, clk, callback=None):
        self._signals = ['axis_tvalid','axis_tready','axis_tlast','axis_tdata','axis_tstrb']
        BusMonitor.__init__(self, dut, name, clk, callback=callback)
        self.clock = clk
        self.transactions = 0
    async def _monitor_recv(self):
        """
        Monitor receiver
        """
        rising_edge = RisingEdge(self.clock) # make these coroutines once and reuse
        while True:
            await rising_edge
            valid = self.bus.axis_tvalid.value
            ready = self.bus.axis_tready.value
            last = self.bus.axis_tlast.value
            data = self.bus.axis_tdata.value #.signed_integer
            if valid and ready:
                self.transactions+=1
                self._recv(data)

class AXISOutWithSidebandMonitor(BusMonitor):
    """
    Monitors M00 AXIS handshake and captures:
      - m00_axis_tdata (Vh)
      - dut.T1
      - dut.T2
    on the SAME accepted cycle.
    """
    transactions = 0

    def __init__(self, dut, name, clk, callback=None):
        self._signals = ['axis_tvalid','axis_tready','axis_tlast','axis_tdata','axis_tstrb']
        super().__init__(dut, name, clk, callback=callback)
        self.dut = dut
        self.clock = clk
        self.transactions = 0

    async def _monitor_recv(self):
        rising_edge = RisingEdge(self.clock)
        MASK96 = (1<<96) - 1

        while True:
            await rising_edge
            if self.bus.axis_tvalid.value and self.bus.axis_tready.value:
                self.transactions += 1
                sample = {
                    "m00":  int(self.bus.axis_tdata.value.signed_integer) & MASK96,
                    "T1":   int(self.dut.T1.value.signed_integer) & MASK96,
                    "T2":   int(self.dut.T2.value.signed_integer) & MASK96,
                    "time": gst(),
                    "count": self.transactions,
                }
                self._recv(sample)

class AXISDriver(BusDriver):
    def __init__(self, dut, name, clk, role="M"):
        self._signals = ['axis_tvalid', 'axis_tready', 'axis_tlast', 'axis_tdata','axis_tstrb']
        BusDriver.__init__(self, dut, name, clk)
        self.clock = clk
        if role=='M':
            self.role = role
            self.bus.axis_tdata.value = 0
            self.bus.axis_tstrb.value = 0
            self.bus.axis_tlast.value = 0
            self.bus.axis_tvalid.value = 0
        elif role == 'S':
            self.role = role
            self.bus.axis_tready.value = 0
        else:
            raise ValueError("role can only be 'M' or 'S'")

    async def _driver_send(self, value, sync=True):
        rising_edge = RisingEdge(self.clock) # make these coroutines once and reuse
        falling_edge = FallingEdge(self.clock)
        read_only = ReadOnly() #This is
        if self.role == 'M':
            if value.get("type") == "write_single":
                await falling_edge #wait until after a rising edge has passed.
                self.bus.axis_tdata.value = value.get('contents').get('data')
                self.bus.axis_tstrb.value = 0xF
                self.bus.axis_tlast.value = value.get('contents').get('last')
                self.bus.axis_tvalid.value = 1 #set valid to be 1
                await read_only
                if self.bus.axis_tready.value == 0: #ifnot there...
                    await RisingEdge(self.bus.axis_tready) #wait until it does go high
                await rising_edge
                #self.bus.axis_tvalid.value = 0 #set to 0 and be done.
            elif value.get("type") == "pause":
                await falling_edge
                self.bus.axis_tvalid.value = 0 #set to 0 and be done.
                await ClockCycles(self.clock,value.get("duration",1))
            elif value.get("type") == "write_burst":
                data = value.get("contents").get("data")
                for i in range(len(data)):
                    await falling_edge
                    self.bus.axis_tdata.value = int(data[i])
                    if i == len(data)-1:
                        self.bus.axis_tlast.value = 1
                    else:
                        self.bus.axis_tlast.value = 0
                    self.bus.axis_tvalid.value = 1
                    if self.bus.axis_tready.value == 0:
                        await RisingEdge(self.bus.axis_tready)
                    await rising_edge
                await falling_edge
                self.bus.axis_tvalid.value = 0
                self.bus.axis_tlast.value = 0
            else:
                pass
        elif self.role == 'S':
            if value.get("type") == "pause":
                await falling_edge
                self.bus.axis_tready.value = 0 #set to 0 and be done.
                await ClockCycles(self.clock,value.get("duration",1))
            elif value.get("type") == "read_single":
                await falling_edge #wait until after a rising edge has passed.
                self.bus.axis_tready.value = 1 #set valid to be 1
                await read_only
                if self.bus.axis_tvalid.value == 0: #ifnot there...
                    await RisingEdge(self.bus.axis_tvalid) #wait until it does go high
                await rising_edge
                self.bus.axis_tready.value = 0 #set to 0 and be done.
            elif value.get("type") == "read_burst":
                for i in range(value.get("duration",1)):
                    await falling_edge #wait until after a rising edge has passed.
                    self.bus.axis_tready.value = 1 #set valid to be 1
                    await read_only
                    if self.bus.axis_tvalid.value == 0: #ifnot there...
                        await RisingEdge(self.bus.axis_tvalid) #wait until it does go high
                    await rising_edge
                self.bus.axis_tready.value = 0 #set to 0 and be done.

async def reset(clk,rst, cycles_held = 3,polarity=1):
    rst.value = polarity
    await ClockCycles(clk, cycles_held)
    rst.value = not polarity
    
# FRAC_Q131 = 31
# FRAC_UQ032 = 32
# UQ0_32_MIN = 2.0**-15  # matches RTL 0x0002_0000
Q31 = 2**31

def u32(x): return x & 0xFFFF_FFFF

def q131_to_float(x: int) -> float:
    """Interpret x as signed Q1.31 int32 -> float."""
    x &= 0xFFFFFFFF
    if x & 0x80000000:
        x -= 0x100000000
    return float(x) / Q31

def float_to_q131(f: float) -> int:
    """Quantize float -> signed Q1.31 int32 (saturating)."""
    # clamp to [-1, 1)
    if f >= 1.0:
        f = np.nextafter(1.0, 0.0)
    if f < -1.0:
        f = -1.0
    v = int(np.round(f * Q31))
    # saturate to int32 range
    if v > 0x7FFFFFFF:
        v = 0x7FFFFFFF
    if v < -0x80000000:
        v = -0x80000000
    return v & 0xFFFFFFFF

def uq032_to_float(u: int) -> float:
    """UQ0.32 -> float in [0,1)."""
    u &= 0xFFFFFFFF
    return float(u) / 2**32

def float_to_uq032(f: float) -> int:
    """float -> UQ0.32."""
    if f <= 0.0:
        return 0
    if f >= 1.0:
        f = np.nextafter(1.0, 0.0)
    return int(np.floor(f * (2**32))) & 0xFFFFFFFF

def unpack96_vec_q131(packed96: int):
    """Return (z,y,x) as signed int32 raw (masked). Layout assumed: {z[95:64], y[63:32], x[31:0]}"""
    packed96 &= (1<<96) - 1
    x = (packed96 >> 0) & 0xFFFFFFFF
    y = (packed96 >> 32) & 0xFFFFFFFF
    z = (packed96 >> 64) & 0xFFFFFFFF
    return z, y, x

def pack128_alpha_view(alpha_uq032: int, vz_q131: int, vy_q131: int, vx_q131: int) -> int:
    """{alpha[127:96], view_z[95:64], view_y[63:32], view_x[31:0]}"""
    return ((alpha_uq032 & 0xFFFFFFFF) << 96) | ((vz_q131 & 0xFFFFFFFF) << 64) | ((vy_q131 & 0xFFFFFFFF) << 32) | (vx_q131 & 0xFFFFFFFF)

def normalize3(v):
    v = np.asarray(v, dtype=np.float64)
    n = np.linalg.norm(v)
    if n < 1e-30:
        return np.array([0.0,0.0,0.0], dtype=np.float64)
    return (v / n).astype(np.float64)

def inv_sqrt_ref(x):
    """
    Golden reference for inv_sqrt used in GGX.
    x is float in [0, 1)
    """
    X_MIN = 2**-15
    S = 0.25
    if x < X_MIN:
        x = X_MIN
    return S / np.sqrt(x)


def event_basis_ref(alpha_uq032, vx_q131, vy_q131, vz_q131):
    """
    Returns expected floats:
      Vh (3,), T1 (3,), T2 (3,)
    following your reference math.
    """
    alpha = uq032_to_float(alpha_uq032)

    view = normalize3([q131_to_float(vx_q131), q131_to_float(vy_q131), q131_to_float(vz_q131)])
    Vh = normalize3([alpha * view[0], alpha * view[1], view[2]])

    lensq = Vh[0]*Vh[0] + Vh[1]*Vh[1]
    if lensq > 1e-20:
        # inv_len = 1.0 / np.sqrt(lensq)
        inv_len = inv_sqrt_ref(max(lensq,2**-15)) * 4.0
        T1 = np.array([-Vh[1]*inv_len, Vh[0]*inv_len, 0.0], dtype=np.float64)
        T2 = np.cross(Vh, T1)
    else:
        T1 = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        T2 = np.array([0.0, 1.0, 0.0], dtype=np.float64)

    return Vh, T1, T2


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


def vec96_to_floats(packed96):
    z_i, y_i, x_i = unpack96_vec_q131(packed96)
    return np.array([q131_to_float(x_i), q131_to_float(y_i), q131_to_float(z_i)], dtype=np.float64)

def assert_vec_close(name, got_v, exp_v, tol=2e-4, dut=None):
    got_v = np.asarray(got_v, dtype=np.float64)
    exp_v = np.asarray(exp_v, dtype=np.float64)
    diff = np.max(np.abs(got_v - exp_v))
    if diff > tol:
        if dut is not None:
            dut._log.error(
                f"{name} mismatch:\n"
                f"  got={got_v}\n"
                f"  exp={exp_v}\n"
                f"  absdiff={np.abs(got_v-exp_v)} maxdiff={diff} tol={tol}"
            )
        raise AssertionError(f"{name} maxdiff {diff} > tol {tol}")

def build_model_and_checker(dut, tol_vh=2e-4, tol_t=2e-4, print_first=5):
    exp_q = []  # FIFO of expected dicts
    seen = {"out": 0}

    def model_cb(transaction):
        # transaction is 128-bit packed input from s00 monitor: {alpha, vz, vy, vx}
        packed = int(transaction) & ((1<<128)-1)  # your AXISMonitor passes .signed_integer; treat as int
        alpha = (packed >> 96) & 0xFFFFFFFF
        vz    = (packed >> 64) & 0xFFFFFFFF
        vy    = (packed >> 32) & 0xFFFFFFFF
        vx    = (packed >>  0) & 0xFFFFFFFF

        # compute floats
        Vh_f, T1_f, T2_f = event_basis_ref(alpha, vx, vy, vz)

        exp_q.append({
            "Vh_f": Vh_f,   # float xyz
            "T1_f": T1_f,
            "T2_f": T2_f,
            "in": (alpha, vx, vy, vz),
        })


    def check_cb(sample):
        if not exp_q:
            raise AssertionError("Got output but expected queue empty!")

        exp = exp_q.pop(0)

        in_alpha = exp['in'][0]
        in_vx = exp['in'][1]
        in_vy = exp['in'][2]
        in_vz = exp['in'][3]

        got_vh = vec96_to_floats(sample["m00"])
        got_t1 = vec96_to_floats(sample["T1"])
        got_t2 = vec96_to_floats(sample["T2"])

        # Note: unpack gives xyz; our ref uses xyz already.
        exp_vh = np.array([exp["Vh_f"][0], exp["Vh_f"][1], exp["Vh_f"][2]], dtype=np.float64)
        exp_t1 = np.array([exp["T1_f"][0], exp["T1_f"][1], exp["T1_f"][2]], dtype=np.float64)
        exp_t2 = np.array([exp["T2_f"][0], exp["T2_f"][1], exp["T2_f"][2]], dtype=np.float64)

        seen["out"] += 1
        if seen["out"] <= print_first:
            dut._log.info(
                f"IN#{seen['out']}:\n"
                f"  alpha={uq032_to_float(in_alpha)}, vx={q131_to_float(in_vx)}, vy={q131_to_float(in_vy)}, vz={q131_to_float(in_vz)}"
            )
            dut._log.info(
                f"OUT#{seen['out']} time={sample['time']}:\n"
                f"  Vh got={got_vh} exp={exp_vh}\n"
                f"  T1 got={got_t1} exp={exp_t1}\n"
                f"  T2 got={got_t2} exp={exp_t2}"
            )

        assert_vec_close("Vh", got_vh, exp_vh, tol=tol_vh, dut=dut)
        assert_vec_close("T1", got_t1, exp_t1, tol=tol_t, dut=dut)
        assert_vec_close("T2", got_t2, exp_t2, tol=tol_t, dut=dut)

    return exp_q, model_cb, check_cb


@cocotb.test()
async def test_event_basis(dut):
    """Test axis_ggx_event_basis using Scoreboard and Model Callback"""
    
    # 1. Setup Signal Queues
    sig_out_exp = [] # Expected transactions (Scoreboard queue)
    sig_out_act = [] # Actual transactions (for debug logging only)
    
    sig_out_exp, model_cb, check_cb, = build_model_and_checker(
        dut,
        tol_vh = 4e-2,
        tol_t = 1.2e-1,
        print_first = 20
    )

    # 3. Initialize Monitors & Drivers
    inm = AXISMonitor(dut, 's00', dut.s00_axis_aclk, callback=model_cb)
    outm = AXISOutWithSidebandMonitor(dut, 'm00', dut.s00_axis_aclk, callback=check_cb)
    
    ind = AXISDriver(dut, 's00', dut.s00_axis_aclk, "M")
    outd = AXISDriver(dut, 'm00', dut.s00_axis_aclk, "S")

    # 4. Start Simulation
    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    await reset(dut.s00_axis_aclk, dut.s00_axis_aresetn, cycles_held=5, polarity=0)

    # 5. Generate Data
    np.random.seed(42)
    N = 256
    input_data = []
    for _ in range(N):
        # random view direction in [-1,1], avoid near-zero vector
        v = np.random.randn(3)
        v = v / np.linalg.norm(v)
        vx, vy, vz = v.tolist()

        # alpha in (0,1): choose roughness^2-ish range
        alpha = float(np.random.uniform(0.02, 1.0))

        packed = pack128_alpha_view(
            float_to_uq032(alpha),
            float_to_q131(vz),
            float_to_q131(vy),
            float_to_q131(vx)
        )
        input_data.append(packed)
    
    # 6. Drive Sequence
    # Send all inputs in a burst
    ind.append({"type": "write_burst", "contents": {"data": input_data}})

   # Repeated stall pattern
    for _ in range(50):
        outd.append({"type": "read_burst", "duration": 4})
        outd.append({"type": "pause", "duration": 3})

    outd.append({"type": "read_burst", "duration": len(input_data) + 50}) # Read remainder

    # 7. Wait for completion
    # The folded (per-burst) inv_sqrt at stage 3 is sequential (~85 cyc/op), so
    # this back-to-back microbenchmark drains at ~1 basis / 85 cyc. (In the real
    # system event_basis runs once per N-sample burst, hiding this latency.)
    await ClockCycles(dut.s00_axis_aclk, 250*N)

    # 8. Assertions
    # If sig_out_exp is not empty, it means we missed outputs.
    assert len(sig_out_exp) == 0, f"Scoreboard mismatch! {len(sig_out_exp)} expected items remaining."
    
    # Ensure no data was lost (Input count == Output count)
    assert inm.transactions == outm.transactions, \
        f"Transaction count mismatch! In: {inm.transactions}, Out: {outm.transactions}"

    dut._log.info("Test Passed!")    
    
def event_basis_runner():
    """Simulate the Inv Sqrt Module"""
    hdl_toplevel_lang = os.getenv("HDL_TOPLEVEL_LANG", "verilog")
    sim = os.getenv("SIM", "icarus")
    sys.path.append(str(proj_path / "sim" / "model"))
    sys.path.append(str(proj_path / "hdl" ))
    ensure_inv_sqrt_rom(proj_path / "sim_build" / "inv_sqrt_rom.mem", addr_bits=14)
    sources = sources_for("axis_ggx_event_basis")
    
    build_test_args = ["-Wall", "-I", str(proj_path / "hdl")]
    sys.path.append(str(proj_path / "sim"))
    runner = get_runner(sim)
    hdl_toplevel = "axis_ggx_event_basis"
    
    parameters = {}
    runner.build(
        sources=sources,
        hdl_toplevel=hdl_toplevel,
        always=True,
        build_args=build_test_args,
        parameters=parameters,
        timescale = ('1ns','1ps'),
        waves=True
    )
    run_test_args = []
    runner.test(
        hdl_toplevel=hdl_toplevel,
        test_module=test_file,
        test_args=run_test_args,
        waves=True
    )
if __name__ == "__main__":
    event_basis_runner()
