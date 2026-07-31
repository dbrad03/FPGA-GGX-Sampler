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

ADDR_BITS = 14
N = 1 << ADDR_BITS
X_MIN = 2**-15
Q = 25
S = 0.25
SCALE = 1 << Q

def to32(x): return x & 0xFFFFFFFF

def float_to_q(x):
    return to32(int(np.round(np.clip(x,0.0, 63.99999997) * SCALE)))

def q_to_float(x):
    x = to32(x)
    if x & (1<<31):
        x -= 1<<32
    return x / SCALE

def float_to_u32(x):
    x = float(np.clip(x,0.0, np.nextafter(1.0,0.0)))
    return to32(int(np.floor(x * (1<<32))))

def inv_sqrt_ref(x):
    """
    Golden reference for inv_sqrt used in GGX.
    x is float in [0, 1)
    """
    if x < X_MIN:
        x = X_MIN
        
    idx = min(max(0,int(x*N)), N-1)
    m = max(X_MIN,((idx+0.5)/N))
    
        
    y0 = S / np.sqrt(m)
    y1 = y0 * (1.5 - x * y0 * y0 * 8.0)
    return float(np.clip(y1,0.0,S/np.sqrt(X_MIN)))


def ensure_rom_exists():
    print("Generating inv_sqrt_rom.mem...")
    
    with open("inv_sqrt_rom.mem", "w") as f:
        for i in range(N):
            m = max((i + 0.5)/N, X_MIN)

            y = S / np.sqrt(m)
            y = min(y, (S/np.sqrt(X_MIN)))

            val = to32(int(np.round(y * SCALE)))
            f.write(f"{val:08x}\n")
            
@cocotb.test()
async def test_inv_sqrt_axis(dut):
    """Test axis_fixed_inv_sqrt using Scoreboard and Model Callback"""
    ensure_rom_exists()
    
    # 1. Setup Signal Queues
    sig_out_exp = [] # Expected transactions (Scoreboard queue)
    sig_out_act = [] # Actual transactions (for debug logging only)

    # 2. Define Model (Callback)
    # This runs every time the input monitor sees a valid transaction
    def inv_sqrt_model(transaction):
        # Convert input
        xu = to32(int(transaction))
        x = xu / (1<<32)
        # dut._log.info(f"IN  raw=0x{xu:08x} x={x:.10f}")
        # dut._log.info(f"Model Input: {x:.6f}")
        
        y = inv_sqrt_ref(x)
        exp_fixed = float_to_q(y)
        
        sig_out_exp.append((x,exp_fixed))

        
    def checker_callback(transaction):
        """Pops expected result and compares with tolerance"""
        if not sig_out_exp:
            dut._log.error("Received transaction but no expected value available!")
            assert False, "Queue Empty"

        in_x, exp_fixed = sig_out_exp.pop(0)
        exp_float = q_to_float(exp_fixed)
        
        act_fixed = to32(int(transaction))
        act_float = q_to_float(act_fixed)
        
        diff = abs(act_float - exp_float)
        TOLERANCE = 0.01

        if diff > TOLERANCE:
            dut._log.error(f"Mismatch! Input: {in_x:.6f} | Act: {act_float:.5f} Exp: {exp_float:.5f} Diff: {diff:.5f}")
            assert False, "Value Mismatch"
        else:
            dut._log.info(f"OK! Input: {in_x:.6f} | Act: {act_float:.4f} Exp: {exp_float:.4f}")

    # 3. Initialize Monitors & Drivers
    inm = AXISMonitor(dut, 's00', dut.s00_axis_aclk, callback=inv_sqrt_model)
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
    
    clamp_vectors = [0.0, 2**-20, 2**-16, 2**-15, 2**-15+2**-32, 2**-15*1.01]
    lut_bin_vectors = []
    for k in [0,1,2,3, 17, 123, 1024, (1<<ADDR_BITS)/2-1, (1<<ADDR_BITS)/2, (1<<ADDR_BITS)-2, (1<<ADDR_BITS)-1]:
        lut_bin_vectors.append(k/(1<<ADDR_BITS))
        lut_bin_vectors.append((k+0.5)/(1<<ADDR_BITS))
        lut_bin_vectors.append((k+1)/(1<<ADDR_BITS))
    midrange_vectors = [0.001, 0.01, 1/64, 1/16, 0.1, 0.25, 0.5, 0.75]
    near1_vectors = [0.9, 0.99, 0.999, np.nextafter(1.0,0.0), 0xFFFFFFFF]
    random_vectors = []
    for _ in range(10):
        random_vectors.append(np.random.uniform(low=0, high=2**-15))
        random_vectors.append(np.random.uniform(low=2**-15, high=0.01))
        random_vectors.append(np.random.uniform(low=0.01, high=0.5))
        random_vectors.append(np.random.uniform(low=0.5, high=1.0))
        
    test_points.extend(clamp_vectors)
    test_points.extend(lut_bin_vectors)
    test_points.extend(midrange_vectors)
    test_points.extend(near1_vectors)
    test_points.extend(random_vectors)
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
    await ClockCycles(dut.s00_axis_aclk, N+50)

    # 8. Assertions
    # If sig_out_exp is not empty, it means we missed outputs.
    assert len(sig_out_exp) == 0, f"Scoreboard mismatch! {len(sig_out_exp)} expected items remaining."
    
    # Ensure no data was lost (Input count == Output count)
    assert inm.transactions == outm.transactions, \
        f"Transaction count mismatch! In: {inm.transactions}, Out: {outm.transactions}"

    dut._log.info("Test Passed with Approximation Tolerance!")

def inv_sqrt_runner():
    """Simulate the Inv Sqrt Module"""
    hdl_toplevel_lang = os.getenv("HDL_TOPLEVEL_LANG", "verilog")
    sim = os.getenv("SIM", "icarus")
    sys.path.append(str(proj_path / "sim" / "model"))
    sys.path.append(str(proj_path / "hdl" ))
    sources = sources_for("axis_fixed_inv_sqrt")
    
    build_test_args = ["-Wall", "-I", str(proj_path / "hdl")]
    sys.path.append(str(proj_path / "sim"))
    runner = get_runner(sim)
    hdl_toplevel = "axis_fixed_inv_sqrt"
    
    parameters = {"ADDR_BITS": 14}
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
    inv_sqrt_runner()