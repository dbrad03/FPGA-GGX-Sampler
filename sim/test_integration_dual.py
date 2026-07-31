#!/usr/bin/env python3
import cocotb
import os
import random
import sys
import logging
import numpy as np
from math import log
from pathlib import Path
from functools import reduce
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

def u32(x): return x & 0xFFFFFFFF

def pack64(w0=0, w1=0): return ((u32(w1)<<32) | u32(w0))
    
def unpack64(x): return u32(x), u32(x>>32)

def reverse_bits(x: int, nbits = 32):
    # assumes x is already 32 bit
    out = 0
    for _ in range(nbits):
        out <<= 1
        out  |= (x & 1)
        x   >>= 1
    return u32(out)

def sobol_direction_numbers_2d(frac_bits = 32):
    """
    Returns a (2, frac_bits) array:
    - dim 0: standard v[k] = 1 << (31-k)
    - dim 1: based on primitive polynomial x^2 + x + 1
    """
    V = np.zeros((2, frac_bits), dtype=np.uint32)

    # --- dimension 0 ---
    for k in range(frac_bits):
        V[0, k] = np.uint32(1 << (frac_bits - 1 - k))

    # --- dimension 1 ---
    # primitive polynomial: x^2 + x + 1  → degree s=2
    s = 2
    a = 1        # coefficients for x^1 and x^0 terms
    m = [1, 3]   # initial direction numbers in integer form

    # load initial m's
    for i in range(s):
        V[1, i] = np.uint32(u32(m[i] << (frac_bits - 1 - i)))

    # recurrence
    for i in range(s, frac_bits):
        # start: v[i] = v[i-s] ^ (v[i-s] >> s)
        val = u32(int(V[1, i - s])) ^ u32(int(V[1, i - s]) >> s)

        # then conditionally xor terms based on bits of a_mask
        # for j=1..s-1, test bit (s-1-j)
        for j in range(1, s):
            if (a >> (s - 1 - j)) & 1:
                val ^= u32(int(V[1, i - j]) >> j)

        V[1, i] = np.uint32(u32(val))

    return V

def sobol_u32_stateless(dim: int, index: int, dir_table: np.ndarray) -> int:
    index = u32(index)
    g = index ^ (index >> 1)
    x = 0
    bit = 0
    num_bits = dir_table.shape[1] # should be
    
    while g and bit < num_bits:
        if g & 1:
            x ^= int(dir_table[dim][bit])
        g  >>= 1
        bit += 1
    return u32(x)

def rotl32(x: int, r: int) -> int:
    x = u32(x)
    return u32((x<<r) | (x>>(32-r)))

def fmix32(h: int) -> int:
    h  = u32(h)
    h ^= (h >> 16)
    h  = u32(h * 0x85ebca6b)
    h ^= (h >> 13)
    h  = u32(h * 0xc2b2ae35)
    h ^= (h >> 16)
    return u32(h)

def mix(h: int, k: int) -> int:
    # MurmurHash3 32-bit block mix
    h = u32(h)
    k = u32(k)

    k = u32(k * 0xcc9e2d51)
    k = rotl32(k, 15)
    k = u32(k * 0x1b873593)

    h ^= k
    h = rotl32(h, 13)
    h = u32(h * 5 + 0xe6546b64)
    return u32(h)
    
def hash_combine(*vals: int) -> int:
    if len(vals) == 0: raise ValueError("hash_combine needs at least 1 value")
    h0 = 0x9747b28c
    h = reduce(mix, (u32(v) for v in vals), u32(h0))
    return fmix32(h)

def laine_karras_permutation(x: int, seed: int) -> int:
    x  = u32(x + seed)
    x ^= u32(x * 0x6c50b47c)
    x ^= u32(x * 0xb82f1e52)
    x ^= u32(x * 0xc7afe638)
    x ^= u32(x * 0x8d22f6e6)
    return x

def nested_uniform_scramble(x: int, seed: int):
    x = reverse_bits(x)
    x = laine_karras_permutation(x, seed)
    x = reverse_bits(x)
    return x

def shuffled_scrambled_sobol2d(index: int, seed: int):
    direction_table_2d = sobol_direction_numbers_2d()
    
    index = nested_uniform_scramble(index, seed)
    s0 = sobol_u32_stateless(0, index, direction_table_2d)
    s1 = sobol_u32_stateless(1, index, direction_table_2d)
    
    u0 = nested_uniform_scramble(s0, hash_combine(seed, 0))
    u1 = nested_uniform_scramble(s1, hash_combine(seed, 1))
    return u0, u1

sig_out_act = []
sig_out_exp = []

@cocotb.test
async def test_dual_pipeline(dut):
    """ Test Full 2D Pipeline with 64-bit Packed Output """
    
    DIR = sobol_direction_numbers_2d()
    
    def pipeline_model(transaction):
        # Input: {burst_len, seed}
        val_int = int(transaction)
        seed, upper = unpack64(val_int)
        burst_len = upper & 0xFFFF
        
        # Calculate Expected sequence
        for i in range(burst_len + 1):
            # Dim 0 Calculation
            s0 = sobol_u32_stateless(0, i, DIR)
            h0 = hash_combine(seed, 0)
            res0 = nested_uniform_scramble(s0, h0)
            
            # Dim 1 Calculation
            s1 = sobol_u32_stateless(1, i, DIR)
            h1 = hash_combine(seed, 1)
            res1 = nested_uniform_scramble(s1, h1)
            
            # Output Pack: {res1 (32), res0 (32)}
            packed_res = pack64(w0=res0, w1=res1)
            
            sig_out_exp.append(BinaryValue(value=packed_res, n_bits=64, bigEndian=False))

    inm = AXISMonitor(dut,'s00',dut.s00_axis_aclk, callback=pipeline_model)
    outm = AXISMonitor(dut,'m00',dut.s00_axis_aclk, callback=lambda x: sig_out_act.append(x))
    ind = AXISDriver(dut,'s00',dut.s00_axis_aclk,"M") 
    outd = AXISDriver(dut,'m00',dut.s00_axis_aclk,"S") 
    
    scoreboard = Scoreboard(dut, fail_immediately=True)
    scoreboard.add_interface(outm, sig_out_exp)
    
    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    await reset(dut.s00_axis_aclk, dut.s00_axis_aresetn, cycles_held=5, polarity=0)
    
    # Send Command
    N = 2048
    seed_val = 0x12345678
    cmd = pack64(w0=seed_val, w1=N-1)
    
    ind.append({"type":"write_burst", "contents":{"data":[cmd]}})
    
    # Drain
    outd.append({"type":"read_burst", "duration":N+50})
    
    await ClockCycles(dut.s00_axis_aclk, N + 200)
    
    assert len(sig_out_exp) == 0, f"Scoreboard Error: {len(sig_out_exp)} items pending"
    dut._log.info(f"Test Passed: Generated {outm.transactions} 2D points.")
    
def dual_sample_runner():
    """Simulate the Top Level Sampler Controller"""
    hdl_toplevel_lang = os.getenv("HDL_TOPLEVEL_LANG", "verilog")
    sim = os.getenv("SIM", "icarus")
    sys.path.append(str(proj_path / "sim" / "model"))
    sys.path.append(str(proj_path / "hdl" ))
    sources = sources_for("axis_pre_ggx_sampler")
    
    build_test_args = ["-Wall", "-I", str(proj_path / "hdl")]
    sys.path.append(str(proj_path / "sim"))
    runner = get_runner(sim)
    hdl_toplevel = "axis_pre_ggx_sampler"
    
    parameters = {}#{"C_S00_AXIS_TDATA_WIDTH": 64, "C_M00_AXIS_TDATA_WIDTH": 64}
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
    dual_sample_runner()