import cocotb
import os
import random
import sys
from math import log
import numpy
import logging
import matplotlib.pyplot as plt
from pathlib import Path
from cocotb.clock import Clock
from cocotb.triggers import Timer, ClockCycles, RisingEdge, FallingEdge, ReadOnly,with_timeout
from cocotb.utils import get_sim_time as gst
from cocotb.runner import get_runner
# from vicoco.vivado_runner import get_runner
#new!!!
from cocotb_bus.bus import Bus
from cocotb_bus.drivers import BusDriver
from cocotb_bus.monitors import Monitor
from cocotb_bus.monitors import BusMonitor
from cocotb_bus.scoreboard import Scoreboard
import numpy as np
test_file = os.path.basename(__file__).replace(".py","")

proj_path = Path(__file__).resolve().parent.parent

FRAC_BITS = 32
FRAC_ONE  = 1 << FRAC_BITS
DIMS = 2

def ctz(x: int) -> int:
    """Count trailing zeros, 0 <= result < INDEX_BITS."""
    if x == 0:
        return 32
    n = 0
    while (x & 1) == 0:
        x >>= 1
        n += 1
    return n

class SobolInc:
    """
    Incremental Sobol, same recurrence as hardware:

        x_{n+1}^{(d)} = x_n^{(d)} XOR V[d, ctz(n+1)]
    """
    def __init__(self, dir_table):
        # dir_table: np.ndarray [DIMS, NUM_BITS] of uint32
        self.dir_table = dir_table.astype(np.uint32)
        self.num_dims, self.num_bits = self.dir_table.shape
        self.index = 0
        self.state = [0] * self.num_dims

    def step_all(self):
        """Advance one index, update ALL dims, return list of floats [0,1)."""
        self.index += 1
        bit = ctz(self.index)
        if bit < self.num_bits:
            for d in range(self.num_dims):
                self.state[d] ^= int(self.dir_table[d, bit])
        # return fixed-point ints too so we can pack exactly as RTL
        return [s for s in self.state]

def pack_sobol_sample(fixed_state, frac_bits=FRAC_BITS):
    """
    fixed_state: list of DIMS uint32 samples (Q0.FRAC_BITS).
    Dimension 0 in lowest bits, etc. (same as << (d * FRAC_BITS)).
    """
    word = 0
    for d, v in enumerate(fixed_state):
        word |= (int(v) & ((1 << frac_bits) - 1)) << (d * frac_bits)
    return word

def unpack_sobol_sample(packed_val):
    """
    Unpacks a 64-bit word into two 32-bit integers.
    Assumes dim 0 is in the lower 32 bits and dim 1 is in the upper 32 bits.
    """
    # Mask for 32 bits: 0xFFFFFFFF
    mask32 = (1 << 32) - 1
    
    # Extract the lower 32 bits (Dimension 0)
    dim0_val = packed_val & mask32
    
    # Shift right by 32 bits and extract the next 32 bits (Dimension 1)
    dim1_val = (packed_val >> 32) & mask32
    
    return dim0_val, dim1_val
    
def sobol_direction_numbers_2d(frac_bits=32):
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
    a = [1, 1]   # coefficients for x^1 and x^0 terms
    m = [1, 3]   # initial direction numbers in integer form

    # Load initial m’s into V
    for k in range(s):
        V[1, k] = np.uint32(m[k] << (frac_bits - k - 1))

    # Generate remaining direction numbers
    for k in range(s, frac_bits):
        V[1, k] = np.uint32(V[1, k - s] ^ (V[1, k - s] >> s))
        for j in range(1, s):
            if a[j]:
                V[1, k] ^= (V[1, k - j] >> j)

    return V


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
                #print(f"{self.name}: {thing}")
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
                #self.bus.axis_tvalid.value = 0
                #self.bus.axis_tlast.value = 0
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
    
sig_out_exp = []
sig_out_act = []

def sobol_scatter(u, v, title):
    plt.figure(figsize=(6,6))
    plt.scatter(u, v, s=3, alpha=0.7, edgecolor="none")
    plt.xlim(0,1)
    plt.ylim(0,1)
    plt.gca().set_aspect("equal")
    plt.title(title)
    plt.xlabel("u")
    plt.ylabel("v")
    plt.savefig("rtl_sobol_2d.png")

def normalize_32bit_sobol(values):
    """
    Converts 32-bit integer values to floats in the range [0, 1].
    """
    # Convert input to a numpy array
    arr = np.array(values)
    
    # If the array is signed int32, view it as unsigned uint32
    # This ensures bit patterns like 0xFFFFFFFF are treated as ~4 billion, not -1
    if arr.dtype == np.int32:
        arr = arr.view(np.uint32)
        
    # Divide by 2^32 (4294967296)
    return arr / 4294967296.0


@cocotb.test()
async def test_a(dut):
    """cocotb test for AXIS FIR15"""
    inm = AXISMonitor(dut,'s00',dut.s00_axis_aclk)
    outm = AXISMonitor(dut,'m00',dut.s00_axis_aclk, callback=lambda x: sig_out_act.append(x))
    ind = AXISDriver(dut,'s00',dut.s00_axis_aclk,"M") #M driver for S port
    outd = AXISDriver(dut,'m00',dut.s00_axis_aclk,"S") #S driver for M port
    
    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start()) 
    await reset(dut.s00_axis_aclk,dut.s00_axis_aresetn,cycles_held=5, polarity=0)
    
    DIR_TABLE = sobol_direction_numbers_2d()
    sobol = SobolInc(DIR_TABLE)
    N = 2**11
    
    for _ in range(N):
        fixed_state = sobol.step_all()
        sig_out_exp.append(pack_sobol_sample(fixed_state))

    pause = {"type": "pause","duration": 1}
    outd.append({'type':'read_burst', 'duration':N})
    outd.append(pause)
    await ClockCycles(dut.s00_axis_aclk, N + 20)
    assert len(sig_out_act) == len(sig_out_exp)
     
    u_sobol = []
    v_sobol = []
    for i, expected in enumerate(sig_out_exp):
        actual = sig_out_act[i]
        act0, act1 = unpack_sobol_sample(actual)
        exp0, exp1 = unpack_sobol_sample(expected)
        u_sobol.append(act0)
        v_sobol.append(act1)
        
        try:
            assert actual == expected
        except AssertionError:
            print(f"For dim 0.. expected {hex(exp0)}, got {hex(act0)}")
            print(f"For dim 1.. expected {hex(exp1)}, got {hex(act1)}")

    u_sobol = normalize_32bit_sobol(u_sobol)
    v_sobol = normalize_32bit_sobol(v_sobol)
    sobol_scatter(u_sobol, v_sobol, "Sobol 2D")
            
    
    
def sobol_runner():
    """Simulate the ADSB decoder using the Python runner."""
    hdl_toplevel_lang = os.getenv("HDL_TOPLEVEL_LANG", "verilog")
    sim = os.getenv("SIM", "icarus")
    # sim = os.getenv("SIM", "vivado")
    sys.path.append(str(proj_path / "sim" / "model"))
    sys.path.append(str(proj_path / "hdl" ))
    sources = [
               proj_path / "hdl" / "axis_sobol.sv", 
            ] 
    
    build_test_args = ["-Wall", "-I", str(proj_path / "hdl")]
    parameters = {} #!!!
    sys.path.append(str(proj_path / "sim"))
    runner = get_runner(sim)
    hdl_toplevel = "axis_sobol"
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
    sobol_runner()