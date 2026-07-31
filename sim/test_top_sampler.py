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

def u32(x): return x & 0xFFFFFFFF

def pack64(w0=0, w1=0): return ((u32(w1)<<32) | u32(w0))
    
def unpack64(x): return u32(x), u32(x>>32)

sig_out_act = []
sig_out_exp = []

@cocotb.test
async def test_top_level_sampler(dut):
    """cocotb test for top level sampler"""
    
    def sampler_ctrl_model(transaction):
        seed, upper_word = unpack64(transaction)
        burst_len = upper_word & 0xFFFF
        for i in range(burst_len+1):
            packed_exp = pack64(w0=seed, w1=i)
            sig_out_exp.append(BinaryValue(value=packed_exp, n_bits=64, bigEndian=False))
        
        
    inm = AXISMonitor(dut,'s00',dut.s00_axis_aclk, callback=sampler_ctrl_model)
    outm = AXISMonitor(dut,'m00',dut.s00_axis_aclk, callback=lambda x: sig_out_act.append(x))
    ind = AXISDriver(dut,'s00',dut.s00_axis_aclk,"M") #M driver for S port
    outd = AXISDriver(dut,'m00',dut.s00_axis_aclk,"S") #S driver for M port
    
    scoreboard = Scoreboard(dut, fail_immediately=True)
    scoreboard.add_interface(outm, sig_out_exp)
    
    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    await reset(dut.s00_axis_aclk, dut.s00_axis_aresetn, cycles_held=5, polarity=0)
    
    # --- TEST GENERATION ---
    
    # We want 65536 samples.
    # Logic: indices 0..N -> N+1 samples.
    # To get 65536 samples, we need indices 0..65535.
    # So we send burst_length = 65535.
    
    N = 65536
    burst_length = N-1
    seed_val = 0x12345578
    in_beats = [pack64(w0=seed_val,w1=burst_length)]
    
    ind.append({"type":"write_burst", "contents":{"data":in_beats}})
    outd.append({"type":"read_burst", "duration":50})
    outd.append({"type":"pause", "duration": 10})
    outd.append({"type":"read_burst", "duration":N})
    
    await ClockCycles(dut.s00_axis_aclk, N+100)
    assert len(sig_out_exp)==0
    assert inm.transactions==1, f"Expected 1 command, got {inm.transactions}"
    assert N==outm.transactions, \
        f"Transaction count mismatch! Expected {N}, got {outm.transactions}"

    dut._log.info(f"Test Passed: Top Level Sampler counts index and handles tlast")
    
def top_sampler_runner():
    """Simulate the Top Level Sampler Controller"""
    hdl_toplevel_lang = os.getenv("HDL_TOPLEVEL_LANG", "verilog")
    sim = os.getenv("SIM", "icarus")
    sys.path.append(str(proj_path / "sim" / "model"))
    sys.path.append(str(proj_path / "hdl" ))
    sources = rtl_sources.sources_for("axis_top_lvl_sampler")
    
    build_test_args = ["-Wall", "-I", str(proj_path / "hdl")]
    sys.path.append(str(proj_path / "sim"))
    runner = get_runner(sim)
    hdl_toplevel = "axis_top_lvl_sampler"
    
    parameters = {"C_S00_AXIS_TDATA_WIDTH": 64, "C_M00_AXIS_TDATA_WIDTH": 64}
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
    top_sampler_runner()