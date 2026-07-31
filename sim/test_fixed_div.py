#!/usr/bin/env python3
import cocotb
import os
import random
import sys
import logging
import numpy as np
from pathlib import Path
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge, ReadOnly
try:
    from cocotb.runner import get_runner
except ImportError:
    from cocotb_tools.runner import get_runner
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
    """ Monitors AXI streaming bus """
    transactions = 0
    def __init__(self, dut, name, clk, callback=None):
        self._signals = ['axis_tvalid','axis_tready','axis_tlast','axis_tdata','axis_tstrb']
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
            valid = self.bus.axis_tvalid.value
            ready = self.bus.axis_tready.value
            data = self.bus.axis_tdata.value
            if valid and ready:
                self.transactions += 1
                self._recv(data)

class AXISDriver(BusDriver):
    """ Drives AXI streaming bus """
    def __init__(self, dut, name, clk, role="M"):
        self._signals = ['axis_tvalid', 'axis_tready', 'axis_tlast', 'axis_tdata','axis_tstrb']
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
                    self.bus.axis_tlast.value = 1 if i == len(data)-1 else 0
                    self.bus.axis_tvalid.value = 1
                    self.bus.axis_tstrb.value = 0xFF
                    if self.bus.axis_tready.value == 0:
                        await RisingEdge(self.bus.axis_tready)
                    await rising_edge
                self.bus.axis_tvalid.value = 0
                self.bus.axis_tlast.value = 0
            elif value.get("type") == "pause":
                await falling_edge
                self.bus.axis_tvalid.value = 0
                await ClockCycles(self.clock, value.get("duration", 1))
        elif self.role == 'S':
            if value.get("type") == "pause":
                await falling_edge
                self.bus.axis_tready.value = 0
                await ClockCycles(self.clock, value.get("duration", 1))
            elif value.get("type") == "read_burst":
                for i in range(value.get("duration", 1)):
                    await falling_edge
                    self.bus.axis_tready.value = 1
                    await read_only
                    if self.bus.axis_tvalid.value == 0:
                        await RisingEdge(self.bus.axis_tvalid)
                    await rising_edge
                self.bus.axis_tready.value = 0

sig_out_act = []
sig_out_exp = []

@cocotb.test()
async def test_div_axis(dut):
    """ Test pipelined restoring divider """
    # 1. Setup
    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    dut.s00_axis_aresetn.value = 0
    await ClockCycles(dut.s00_axis_aclk, 10)
    dut.s00_axis_aresetn.value = 1
    await ClockCycles(dut.s00_axis_aclk, 5)

    # 2. Scoreboard and callback
    def scoreboard_callback(transaction):
        sig_out_act.append(transaction)

    inm = AXISMonitor(dut, 's00', dut.s00_axis_aclk)
    outm = AXISMonitor(dut, 'm00', dut.s00_axis_aclk, callback=scoreboard_callback)
    ind = AXISDriver(dut, 's00', dut.s00_axis_aclk, "M")
    outd = AXISDriver(dut, 'm00', dut.s00_axis_aclk, "S")

    # 3. Reference Model
    # A = dividend (upper 32b), B = divisor (lower 32b)
    # Output = (A << 25) / B
    def run_ref_model(a_val, b_val):
        numerator = a_val << 25
        quotient = numerator // b_val
        # Clamp to 32-bit output
        return quotient & 0xFFFFFFFF

    # 4. Generate test vectors
    # Let's create pairs (A, B)
    test_pairs = []
    # Regular divisions
    for _ in range(100):
        # A in [1, 2**31], B in [1, 2**31]
        a = random.randint(1, 2**30)
        b = random.randint(1, 2**30)
        test_pairs.append((a, b))

    test_pairs.append((2**30, 1))
    test_pairs.append((1, 2**30))
    test_pairs.append((2**31-1, 2**31-1))
    test_pairs.append((0, 2**30))
    test_pairs.append((1073741824, 3037000500))

    input_data = []
    for a, b in test_pairs:
        # Pack to 64-bit: {A[31:0], B[31:0]}
        packed = (a << 32) | b
        input_data.append(packed)
        
        expected = run_ref_model(a, b)
        sig_out_exp.append(BinaryValue(value=expected, n_bits=32, bigEndian=False))

    # 5. Drive burst
    ind.append({"type": "write_burst", "contents": {"data": input_data}})

    # Read burst with backpressure
    outd.append({"type": "read_burst", "duration": 5})
    outd.append({"type": "pause", "duration": 10})
    outd.append({"type": "read_burst", "duration": len(input_data) + 100})

    # Wait for completion
    await ClockCycles(dut.s00_axis_aclk, len(input_data) + 150)

    # 6. Verify results
    assert len(sig_out_act) == len(input_data), f"Got only {len(sig_out_act)} of {len(input_data)} outputs!"
    
    mismatches = 0
    for idx, (a, b) in enumerate(test_pairs):
        act = sig_out_act[idx].integer
        exp = sig_out_exp[idx].integer
        if act != exp:
            mismatches += 1
            dut._log.error(f"Mismatch [{idx}]: A={a}, B={b} | Act: {act} Exp: {exp}")
        else:
            dut._log.info(f"Match [{idx}]: A={a}, B={b} | Act: {act} Exp: {exp}")

    assert mismatches == 0, f"Found {mismatches} test mismatches!"
    dut._log.info("Divider Test Passed!")

def div_runner():
    hdl_toplevel_lang = os.getenv("HDL_TOPLEVEL_LANG", "verilog")
    sim = os.getenv("SIM", "icarus")
    sys.path.append(str(proj_path / "sim"))
    sys.path.append(str(proj_path / "hdl"))
    
    sources = sources_for("axis_fixed_div")
    
    build_test_args = ["-Wall", "-I", str(proj_path / "hdl")]
    runner = get_runner(sim)
    hdl_toplevel = "axis_fixed_div"
    
    runner.build(
        sources=sources,
        hdl_toplevel=hdl_toplevel,
        always=True,
        build_args=build_test_args,
        timescale=('1ns','1ps'),
        waves=True
    )
    runner.test(
        hdl_toplevel=hdl_toplevel,
        test_module=test_file,
        waves=True
    )

if __name__ == "__main__":
    div_runner()
