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

FRAC_BITS = 32
ADDR_BITS = 10
SCALE = 2.0**(FRAC_BITS - 1)

def u32(x): return x & 0xFFFFFFFF

def to_fixed(f):
    f = np.clip(f, -0.999999, 0.999999)
    return int(f * SCALE)

def from_fixed(i):
    if isinstance(i, BinaryValue):
        i = i.signed_integer
        
    if i >= (1 << (FRAC_BITS-1)):
        i -= (1 << FRAC_BITS)
    return i / SCALE

def ensure_rom_exists():
    filename = "ggx_trig_rom.mem"
    # if os.path.exists(filename): return
    print(f"Generating {filename}...")
    N = 1 << ADDR_BITS
    with open(filename, "w") as f:
        for i in range(N):
            # Phase u from [0, 1) maps to angle [0, 2pi)
            # u = i / N
            # angle = u * 2 * pi
            angle = (i / N) * 2.0 * np.pi
            c = to_fixed(np.cos(angle))
            s = to_fixed(np.sin(angle))
            # Pack {Cos, Sin}
            val = (u32(s) << 32) | u32(c)
            f.write(f"{val:016x}\n")
            
@cocotb.test()
async def test_trig_lut(dut):
    """Test axis_trig_lut using Model Callback"""
    ensure_rom_exists()
    
    sig_out_exp = []
    sig_out_act = []
    
    def monitor_cb(transaction):
        val = transaction.integer
        cos_int = val & 0xFFFFFFFF
        sin_int = (val >> 32) & 0xFFFFFFFF
        sig_out_act.append((from_fixed(cos_int), from_fixed(sin_int)))
    
    inm = AXISMonitor(dut, 's00', dut.s00_axis_aclk, callback=None)
    outm = AXISMonitor(dut, 'm00', dut.s00_axis_aclk, callback=monitor_cb)
    ind = AXISDriver(dut, 's00', dut.s00_axis_aclk, "M")
    outd = AXISDriver(dut, 'm00', dut.s00_axis_aclk, "S")
    
    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    await reset(dut.s00_axis_aclk, dut.s00_axis_aresetn, cycles_held=5, polarity=0)
    
    # 1. Generate Inputs
    # 0.0, 0.25 (90 deg), 0.5 (180 deg), 0.75 (270 deg)
    input_data = [0.0, 0.25, 0.5, 0.75, 0.125, 1.0/(1<<10), ((1<<10)-1.0)/(1<<10)]
    for _ in range(100):
        input_data.append(np.random.uniform(0.0,0.999))
    dut_inputs = []
    for u in input_data:
        angle = u * 2.0 * np.pi
        dut_inputs.append(int(u*2**32)&0xFFFFFFFF)
        sig_out_exp.append((np.cos(angle), np.sin(angle)))
        
    ind.append({"type": "write_burst", "contents": {"data": dut_inputs}})
    outd.append({"type": "read_burst", "duration": 3}) 
    outd.append({"type": "pause", "duration": 5})     # Backpressure for 5 clocks
    outd.append({"type": "read_burst", "duration": len(input_data)}) # Read remainder
    
    await ClockCycles(dut.s00_axis_aclk, len(input_data)+50)
    assert len(sig_out_act) == len(sig_out_exp)
    for i, (act_c, act_s) in enumerate(sig_out_act):
        exp_c, exp_s = sig_out_exp[i]
        dut._log.info(f"In: {input_data[i]:.3f} | Exp: ({exp_c:.3f}, {exp_s:.3f}) | Act: ({act_c:.3f}, {act_s:.3f})")
        assert abs(act_c - exp_c) <= 0.03
        assert abs(act_s - exp_s) <= 0.03
    
def trig_lut_runner():
    """Simulate the Inv Sqrt Module"""
    hdl_toplevel_lang = os.getenv("HDL_TOPLEVEL_LANG", "verilog")
    sim = os.getenv("SIM", "icarus")
    sys.path.append(str(proj_path / "sim" / "model"))
    sys.path.append(str(proj_path / "hdl" ))
    sources = rtl_sources.sources_for("axis_trig_lut")
    
    build_test_args = ["-Wall", "-I", str(proj_path / "hdl")]
    sys.path.append(str(proj_path / "sim"))
    runner = get_runner(sim)
    hdl_toplevel = "axis_trig_lut"
    
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
    trig_lut_runner()