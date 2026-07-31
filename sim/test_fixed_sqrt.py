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
import rtl_sources

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
        falling_edge = FallingEdge(self.clock)
        read_only = ReadOnly() #This is
        while True:
            await rising_edge
            await falling_edge #sometimes see in AXI shit
            await read_only  #readonly (the postline)
            valid = self.bus.axis_tvalid.value
            ready = self.bus.axis_tready.value
            last = self.bus.axis_tlast.value
            data = self.bus.axis_tdata.value #.signed_integer
            if valid and ready:
                self.transactions+=1
                thing = dict(data=data.signed_integer,last=last,name=self.name,count=self.transactions,time=gst())
                # print(f"{self.name}: {thing}")
                self._recv(data)

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

'''
{"type":"write_single", "contents": {"data":5, "last":0}}
{"type":"pause","duration":10}
{"type":"write_burst", "contents": {"data": np.array(9*[0]+[1]+30*[0]+[-2]+59*[0])}}
{"type":"read_single"}
{"type":"read_burst", "duration":10}
'''

Q = 32
SCALE = 1 << Q

def to32(x): return x & 0xFFFFFFFF

def float_to_u32(x):
    x = float(np.clip(x, 0.0, np.nextafter(1.0, 0.0)))
    return to32(int(np.floor(x * SCALE)))  # floor is natural for UQ0.32

def q32_to_float(x):
    u = to32(x)
    return u / SCALE

def sqrt_ref(x):
    """
    Golden reference for sqrt used in GGX.
    x is float in [0, 1)
    """
    return float(np.sqrt(np.clip(x, 0.0, np.nextafter(1.0, 0.0))))
            
@cocotb.test()
async def test_sqrt_axis(dut):
    """Test axis_fixed_sqrt Model Callback"""
    
    # 1. Setup Signal Queues
    sig_out_exp = [] # Expected transactions (Scoreboard queue)
    sig_out_act = [] # Actual transactions (for debug logging only)

    # 2. Define Model (Callback)
    # This runs every time the input monitor sees a valid transaction
    def sqrt_model(transaction):
        # Convert input
        xu = to32(int(transaction))
        x = xu / (1<<32)
        # dut._log.info(f"IN  raw=0x{xu:08x} x={x:.10f}")
        # dut._log.info(f"Model Input: {x:.6f}")
        
        y = sqrt_ref(x)
        exp_fixed = float_to_u32(y)
        
        sig_out_exp.append(exp_fixed)

        
    def checker_callback(transaction):
        """Pops expected result and compares with tolerance"""
        if not sig_out_exp:
            dut._log.error("Received transaction but no expected value available!")
            assert False, "Queue Empty"
            
        

        exp_fixed = sig_out_exp.pop(0)
        exp_float = q32_to_float(exp_fixed)
        
        act_fixed = to32(int(transaction))
        act_float = q32_to_float(act_fixed)
        
        if outm.transactions < 10:
            dut._log.info(
                f"RAW act=0x{act_fixed:08x} exp=0x{exp_fixed:08x} "
                f"act_f={act_float:.10f} exp_f={exp_float:.10f}"
            )
        diff = abs(act_float - exp_float)
        TOLERANCE = 1e-3

        if diff > TOLERANCE:
            dut._log.error(f"Mismatch! Act: {act_float:.8f} Exp: {exp_float:.8f} Diff: {diff:.8f}")
            assert False, "Value Mismatch"
        else:
            dut._log.info(f"OK! Act: {act_float:.8f} Exp: {exp_float:.8f}")

    # 3. Initialize Monitors & Drivers
    inm = AXISMonitor(dut, 's00', dut.s00_axis_aclk, callback=sqrt_model)
    outm = AXISMonitor(dut, 'm00', dut.s00_axis_aclk, callback=checker_callback)
    
    ind = AXISDriver(dut, 's00', dut.s00_axis_aclk, "M")
    outd = AXISDriver(dut, 'm00', dut.s00_axis_aclk, "S")

    # 4. Start Simulation
    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    await reset(dut.s00_axis_aclk, dut.s00_axis_aresetn, cycles_held=5, polarity=0)

    # 5. Generate Data
    N = 64
    test_points = []
    np.random.seed(42)
    
    endpoints = [0.0, np.nextafter(1.0,0.0)]
    perfect_sqs = [((2**k-1)**2)/2**32 for k in range(0, 16)]
    mid_range = [1/3, 0.1, 0.25, 0.5, 0.75, 0.99, 1/2+2**-32]
    awkward = [0.5 + 2**-32, 0.5 - 2**-32, 
               0.25 + 2**-32, 0.25 - 2**-32, 
               0.125 + 2**-32, 0.125 - 2**-32]
    randoms = []
    for _ in range(25):
        randoms.append(np.random.uniform(low=0.0, high=2**-16))
        randoms.append(np.random.uniform(low=2**-16, high=2**-8))
        randoms.append(np.random.uniform(low=2**-8, high=2**0.5))
        randoms.append(np.random.uniform(low=0.5, high=1.0))
        
    test_points.extend(endpoints)
    test_points.extend(perfect_sqs)
    test_points.extend(mid_range)
    test_points.extend(awkward)
    test_points.extend(randoms)
    input_data = [float_to_u32(v) for v in test_points]
    
    # 6. Drive Sequence
    # Send all inputs in a burst
    ind.append({"type": "write_burst", "contents": {"data": input_data}})

    # Read outputs with some backpressure ("pause") to stress the pipeline stall logic
    outd.append({"type": "read_burst", "duration": 3}) 
    outd.append({"type": "pause", "duration": 5})     # Backpressure for 5 clocks
    outd.append({"type": "read_burst", "duration": len(input_data)}) # Read remainder

    # 7. Wait for completion
    # Wait enough cycles for pipeline latency + pauses
    await ClockCycles(dut.s00_axis_aclk, len(input_data)+100)

    # 8. Assertions
    # If sig_out_exp is not empty, it means we missed outputs.
    assert len(sig_out_exp) == 0, f"Scoreboard mismatch! {len(sig_out_exp)} expected items remaining."
    
    # Ensure no data was lost (Input count == Output count)
    assert inm.transactions == outm.transactions, \
        f"Transaction count mismatch! In: {inm.transactions}, Out: {outm.transactions}"

    dut._log.info("Test Passed with Approximation Tolerance!")

def sqrt_runner():
    """Simulate the Inv Sqrt Module"""
    hdl_toplevel_lang = os.getenv("HDL_TOPLEVEL_LANG", "verilog")
    sim = os.getenv("SIM", "icarus")
    sys.path.append(str(proj_path / "sim" / "model"))
    sys.path.append(str(proj_path / "hdl" ))
    sources = rtl_sources.sources_for("axis_fixed_sqrt")
    
    build_test_args = ["-Wall", "-I", str(proj_path / "hdl")]
    sys.path.append(str(proj_path / "sim"))
    runner = get_runner(sim)
    hdl_toplevel = "axis_fixed_sqrt"
    
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
    sqrt_runner()