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


######### SOBOL + SCRAMBLING FUNCTIONS & DEFINITIONS #############
FRAC_BITS = 32
FRAC_ONE  = 1 << FRAC_BITS
DIMS = 2

def sobol_value_stateless(dim: int, index: int, dir_table: np.ndarray) -> float:
    g = index ^ (index >> 1)
    x = 0
    bit = 0
    num_bits = dir_table.shape[1]
    while g and bit < num_bits:
        if g & 1:
            x ^= int(dir_table[dim, bit])
        g >>= 1
        bit += 1
    return x / float(FRAC_ONE)

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

def pack_sample(dim0, dim1):
    return ((dim1 & 0xFFFFFFFF) << 32) | (dim0 & 0xFFFFFFFF)

def unpack_sample(packed_val):
    """
    Unpacks a 64-bit word into two 32-bit integers.
    Assumes dim 0 is in the lower 32 bits and dim 1 is in the upper 32 bits.
    """
    dim0 = packed_val & 0xFFFFFFFF
    dim1 = (packed_val>>32) & 0xFFFFFFFF
    return dim0, dim1
    
def normalize_32bit_sobol(values):
    """
    Converts 32-bit integer values to floats in the range [0, 1].
    """
    arr = np.array(values)
    # If the array is signed int32, view it as unsigned uint32
    # This ensures bit patterns like 0xFFFFFFFF are treated as ~4 billion, not -1
    if arr.dtype == np.int32:
        arr = arr.view(np.uint32)
    # Divide by 2^32 (4294967296)
    return arr / 4294967296.0

def sobol_scatter(u, v, title, savefile="rtl_sobol_2d.png"):
    plt.figure(figsize=(6,6))
    plt.scatter(u, v, s=3, alpha=0.7, edgecolor="none")
    plt.xlim(0,1)
    plt.ylim(0,1)
    plt.gca().set_aspect("equal")
    plt.title(title)
    plt.xlabel("u")
    plt.ylabel("v")
    plt.savefig(savefile)

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

def reverse_bits(x, bits=32):
    # Quick python bit reversal
    b = '{:0{width}b}'.format(x, width=bits)
    return int(b[::-1], 2)

def laine_karras_permutation(x: int, seed: int) -> int:
    """
    Laine-Karras 32-bit permutation (nested uniform permutation).

    For a fixed seed, this is a bijection on 32-bit integers with good
    "nested" properties suitable for Owen-style scrambling.
    """
    x = reverse_bits(x)
    x = (x + seed) & 0xFFFFFFFF
    x ^= (x * 0x6c50b47C) & 0xFFFFFFFF
    x ^= (x * 0xB82F1E52) & 0xFFFFFFFF
    x ^= (x * 0xC7AFE638) & 0xFFFFFFFF
    x ^= (x * 0x8D22F6E6) & 0xFFFFFFFF
    return reverse_bits(x & 0xFFFFFFFF)

def make_lk_seed(pixel_id: int, sample_id: int, dim: int) -> int:
    """
    Combine pixel_id, sample_id, dim into a 32-bit seed.

    This mirrors what you'll do in RTL: some multiplications by odd
    constants and xors. Exact constants aren't sacred; they're just
    good mixers.
    """
    x = (pixel_id * 0x9e3779b1) & 0xFFFFFFFF
    x ^= (sample_id * 0x85ebca6b) & 0xFFFFFFFF
    x ^= (dim       * 0xc2b2ae35) & 0xFFFFFFFF
    return x & 0xFFFFFFFF # dont need this last one

def owen_scramble_lk(u: float,
                     pixel_id: int,
                     sample_id: int,
                     dim: int) -> float:
    """
    Digital Owen scrambling using Laine-Karras permutation:

      u_int      = floor(u * 2^32)
      seed       = f(pixel_id, sample_id, dim)
      u_scr_int  = LK(u_int, seed)
      u_scr      = u_scr_int / 2^32
    """
    # clamp + convert to 32-bit fixed-point
    u_clamped = min(max(u, 0.0), np.nextafter(1.0, 0.0))
    u_int = int(u_clamped * FRAC_ONE) & 0xFFFFFFFF

    seed = make_lk_seed(pixel_id, sample_id, dim)
    u_scr_int = laine_karras_permutation(u_int, seed)
    return (u_scr_int & 0xFFFFFFFF) / float(FRAC_ONE)
################################################

############ AXIS FUNCTIONS ####################
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
    
#################################################

sig_out_exp = []
sig_out_act = []

@cocotb.test()
async def test_a(dut):
    """cocotb test for AXIS FIR15"""
    rising_edge = RisingEdge(dut.s00_axis_aclk)
    falling_edge = FallingEdge(dut.s00_axis_aclk)
    readonly = ReadOnly()
    
    inm = AXISMonitor(dut,'s00',dut.s00_axis_aclk)
    outm = AXISMonitor(dut,'m00',dut.s00_axis_aclk, callback=lambda x: sig_out_act.append(x))
    ind = AXISDriver(dut,'s00',dut.s00_axis_aclk,"M") #M driver for S port
    outd = AXISDriver(dut,'m00',dut.s00_axis_aclk,"S") #S driver for M port
    
    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start()) 
    await reset(dut.s00_axis_aclk,dut.s00_axis_aresetn,cycles_held=5, polarity=0)
    
    DIR_TABLE = sobol_direction_numbers_2d()
    sobol = SobolInc(DIR_TABLE)
    N = 2048
    
    vectors = []
    PIXEL_ID = 0
    SAMPLE_ID = 0
    
    for _ in range(N):
        fixed_state = sobol.step_all()
        
        idx = sobol.index
        u_raw_int = fixed_state[0]
        v_raw_int = fixed_state[1]
        
        seed0 = make_lk_seed(PIXEL_ID, SAMPLE_ID, 0)
        u_scr_int = laine_karras_permutation(u_raw_int, seed0)
        
        seed1 = make_lk_seed(PIXEL_ID, SAMPLE_ID, 1)
        v_scr_int = laine_karras_permutation(v_raw_int, seed1)
        
        sig_out_exp.append((u_scr_int,v_scr_int))
        
        packed_data = pack_sample(u_raw_int,v_raw_int)
        vectors.append({
            'data': packed_data,
            'pixel_id': PIXEL_ID,
            'sample_id': SAMPLE_ID,
            'sample_index': idx
        })

    # pause = {"type": "pause","duration": 1}
    
    
    outd.append({'type':'read_burst', 'duration':N})
    
    for vec in vectors:
        await rising_edge
        await readonly
        while dut.s00_axis_tready.value == 0:
            await rising_edge
            await readonly
        
        await falling_edge
        
        dut.pixel_id.value = vec['pixel_id']
        dut.sample_id.value = vec['sample_id']
        dut.sample_index.value = vec['sample_index']
        
        dut.s00_axis_tdata.value = vec['data']
        dut.s00_axis_tvalid.value = 1
        # ind.append({'type': 'write_single', 'contents': {'data': vec['data'], 'last':0}})
        
    # ind.append(pause)
    await rising_edge
    await falling_edge
    dut.s00_axis_tvalid.value = 0
    
    
    await ClockCycles(dut.s00_axis_aclk, 3*N)
    
    assert len(sig_out_act) == len(sig_out_exp), \
        f"Transaction count mismatch.. expected {len(sig_out_exp)} transactions, saw {len(sig_out_act)}"
        
    u_plot_rtl = []
    v_plot_rtl = []
    
    u_sobol_py = np.array([sobol_value_stateless(0, n, DIR_TABLE) for n in range(1, N+1)])
    u_scr_py = [owen_scramble_lk(u_sobol_py[n],pixel_id=0,sample_id=0,dim=0)
                        for n in range(N)]
    v_sobol_py = np.array([sobol_value_stateless(1, n, DIR_TABLE) for n in range(1, N+1)])
    v_scr_py = [owen_scramble_lk(v_sobol_py[n],pixel_id=0,sample_id=0,dim=1)
                        for n in range(N)]
    
    for i, (exp_u, exp_v) in enumerate(sig_out_exp):
        act_u, act_v = unpack_sample(int(sig_out_act[i]))
        
        assert act_u == exp_u, f"Mismatch on dim 0.. {act_u} vs {exp_u}... sample index: {i}"
        assert act_v == exp_v, f"Mismatch on dim 1.. {act_v} vs {exp_v}"
        
        u_plot_rtl.append(act_u)
        v_plot_rtl.append(act_v)
    
   # ... (assertions passed) ...

    print(f"Verified {N} samples successfully")

    # 1. Normalize RTL output (Integers -> Floats 0..1)
    u_plot_rtl_norm = [x / (2**32) for x in u_plot_rtl]
    v_plot_rtl_norm = [x / (2**32) for x in v_plot_rtl]

    # 2. Normalize Python Model output (Integers -> Floats 0..1)
    # Note: u_scr_py / v_scr_py are already floats in [0, 1)
    u_plot_py_norm = [float(x) for x in u_scr_py]
    v_plot_py_norm = [float(x) for x in v_scr_py]

    # 3. Plot
    # Ensure your sobol_scatter function accepts lists of floats
    sobol_scatter(u_plot_rtl_norm, v_plot_rtl_norm, 
                  "Owen Scrambled Sobol (RTL Output)", 
                  "owen_scramble_rtl.png")
                  
    sobol_scatter(u_plot_py_norm, v_plot_py_norm, 
                  "Owen Scrambled Sobol (RTL Output)", 
                  "owen_scramble_py.png")
                  
    print("Plots saved to owen_scramble_rtl.png and owen_scramble_py.png")
            
    
    
def owen_runner():
    """Simulate the Owen-Scrambling Streamer using the Python runner."""
    hdl_toplevel_lang = os.getenv("HDL_TOPLEVEL_LANG", "verilog")
    sim = os.getenv("SIM", "icarus")
    # sim = os.getenv("SIM", "vivado")
    sys.path.append(str(proj_path / "sim" / "model"))
    sys.path.append(str(proj_path / "hdl" ))
    sources = [
               proj_path / "hdl" / "axis_owen_scrambler.sv", 
            ] 
    
    build_test_args = ["-Wall"], #"-I", str(proj_path / "hdl")]
    parameters = {} #!!!
    sys.path.append(str(proj_path / "sim"))
    runner = get_runner(sim)
    hdl_toplevel = "axis_owen_scrambler"
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
    owen_runner()
