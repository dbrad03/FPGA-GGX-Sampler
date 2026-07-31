#!/usr/bin/env python3
# Standalone test for axis_fixed_inv_sqrt_folded: same reference model and
# tolerance as test_fixed_inv_sqrt_nodsp, but the folded engine processes one
# input at a time (s00_axis_tready low while busy ~85 cycles), so the driver's
# backpressure handling paces it and the drain wait is sized accordingly.
import cocotb
import os
import sys
import numpy as np
from pathlib import Path
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge, ReadOnly
try:
    from cocotb.runner import get_runner
except ImportError:
    from cocotb_tools.runner import get_runner
from cocotb_bus.drivers import BusDriver
from cocotb_bus.monitors import BusMonitor
import rtl_sources

test_file = os.path.basename(__file__).replace(".py", "")
proj_path = Path(__file__).resolve().parent.parent


class AXISMonitor(BusMonitor):
    transactions = 0
    def __init__(self, dut, name, clk, callback=None):
        self._signals = ['axis_tvalid', 'axis_tready', 'axis_tlast', 'axis_tdata', 'axis_tstrb']
        BusMonitor.__init__(self, dut, name, clk, callback=callback)
        self.clock = clk
        self.transactions = 0
    async def _monitor_recv(self):
        rising_edge = RisingEdge(self.clock)
        read_only = ReadOnly()
        while True:
            await rising_edge
            await read_only
            valid = self.bus.axis_tvalid.value
            ready = self.bus.axis_tready.value
            data = self.bus.axis_tdata.value
            if valid and ready:
                self.transactions += 1
                self._recv(data)


class AXISDriver(BusDriver):
    def __init__(self, dut, name, clk, role="M"):
        self._signals = ['axis_tvalid', 'axis_tready', 'axis_tlast', 'axis_tdata', 'axis_tstrb']
        BusDriver.__init__(self, dut, name, clk)
        self.clock = clk
        if role == 'M':
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
        rising_edge = RisingEdge(self.clock)
        falling_edge = FallingEdge(self.clock)
        read_only = ReadOnly()
        if self.role == 'M':
            if value.get("type") == "write_burst":
                data = value.get("contents").get("data")
                for i in range(len(data)):
                    await falling_edge
                    self.bus.axis_tdata.value = int(data[i])
                    self.bus.axis_tlast.value = 1 if i == len(data) - 1 else 0
                    self.bus.axis_tvalid.value = 1
                    self.bus.axis_tstrb.value = 0xF
                    if self.bus.axis_tready.value == 0:
                        await RisingEdge(self.bus.axis_tready)
                    await rising_edge
                await falling_edge
                self.bus.axis_tvalid.value = 0
                self.bus.axis_tlast.value = 0
        elif self.role == 'S':
            if value.get("type") == "read_burst":
                for i in range(value.get("duration", 1)):
                    await falling_edge
                    self.bus.axis_tready.value = 1
                    await read_only
                    if self.bus.axis_tvalid.value == 0:
                        await RisingEdge(self.bus.axis_tvalid)
                    await rising_edge
                self.bus.axis_tready.value = 0


async def reset(clk, rst, cycles_held=5, polarity=0):
    rst.value = polarity
    await ClockCycles(clk, cycles_held)
    rst.value = not polarity


X_MIN = 2**-15
Q = 25
S = 0.25
SCALE = 1 << Q


def to32(x): return x & 0xFFFFFFFF
def float_to_q(x): return to32(int(np.round(np.clip(x, 0.0, 63.99999997) * SCALE)))
def q_to_float(x):
    x = to32(x)
    if x & (1 << 31):
        x -= 1 << 32
    return x / SCALE
def float_to_u32(x):
    x = float(np.clip(x, 0.0, np.nextafter(1.0, 0.0)))
    return to32(int(np.floor(x * (1 << 32))))
def inv_sqrt_ref(x):
    if x < X_MIN:
        x = X_MIN
    return float(S / np.sqrt(x))


sig_out_exp = []


@cocotb.test()
async def test_inv_sqrt_folded(dut):
    """Test axis_fixed_inv_sqrt_folded (bit-identical to the pipelined _nodsp)."""
    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    await reset(dut.s00_axis_aclk, dut.s00_axis_aresetn, cycles_held=5, polarity=0)

    def checker_callback(transaction):
        # Expectations are precomputed from the known input vector (below), so the
        # folded engine's tready-drop-on-accept can't cause a sampling race.
        assert sig_out_exp, "Unexpected output transaction"
        in_x, exp_fixed = sig_out_exp.pop(0)
        exp_float = q_to_float(exp_fixed)
        act_float = q_to_float(to32(int(transaction)))
        diff = abs(act_float - exp_float)
        TOLERANCE = 0.0005
        if diff > TOLERANCE:
            dut._log.error(f"Mismatch! In {in_x:.6f} Act {act_float:.7f} Exp {exp_float:.7f} Diff {diff:.7f}")
            assert False, "Value Mismatch"
        else:
            dut._log.info(f"OK! In {in_x:.6f} Act {act_float:.4f} Exp {exp_float:.4f}")

    outm = AXISMonitor(dut, 'm00', dut.s00_axis_aclk, callback=checker_callback)
    ind = AXISDriver(dut, 's00', dut.s00_axis_aclk, "M")
    outd = AXISDriver(dut, 'm00', dut.s00_axis_aclk, "S")

    np.random.seed(42)
    test_points = [0.0, 2**-20, 2**-16, 2**-15, 2**-15 + 2**-32, 2**-15 * 1.01]
    test_points += [0.001, 0.01, 1/64, 1/16, 0.1, 0.25, 0.5, 0.75]
    test_points += [0.9, 0.99, 0.999, np.nextafter(1.0, 0.0)]
    test_points += [float(np.random.uniform(0, 1.0)) for _ in range(50)]
    input_data = [float_to_u32(v) for v in test_points]

    # Precompute expected outputs (order-preserving) from the driven inputs.
    for xu in input_data:
        x = to32(xu) / (1 << 32)
        sig_out_exp.append((x, float_to_q(inv_sqrt_ref(x))))
    n_exp = len(sig_out_exp)

    ind.append({"type": "write_burst", "contents": {"data": input_data}})
    outd.append({"type": "read_burst", "duration": len(input_data)})

    # Folded engine: ~85 cycles/input, processed serially.
    await ClockCycles(dut.s00_axis_aclk, len(input_data) * 95 + 500)

    assert len(sig_out_exp) == 0, f"Scoreboard mismatch! {len(sig_out_exp)} of {n_exp} expected remaining."
    assert outm.transactions == n_exp, \
        f"Count mismatch! Out: {outm.transactions}, expected: {n_exp}"
    dut._log.info("Inv Sqrt FOLDED Test Passed!")


def inv_sqrt_runner():
    sim = os.getenv("SIM", "icarus")
    sys.path.append(str(proj_path / "sim"))
    sys.path.append(str(proj_path / "hdl"))
    sources = rtl_sources.sources_for("axis_fixed_inv_sqrt_folded")
    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel="axis_fixed_inv_sqrt_folded",
        always=True,
        build_args=["-Wall", "-I", str(proj_path / "hdl")],
        timescale=('1ns', '1ps'),
        waves=True,
    )
    runner.test(hdl_toplevel="axis_fixed_inv_sqrt_folded", test_module=test_file, waves=True)


if __name__ == "__main__":
    inv_sqrt_runner()
