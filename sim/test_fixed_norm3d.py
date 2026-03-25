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

FRAC_BITS = 32
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

def ensure_rom_exists(filename: Path, addr_bits=14):
    print(f"Generating {filename}...")
    N = 1 << addr_bits
    Q = 25
    SCALE = 1 << Q
    X_MIN = 2**-15
    S = 0.25

    filename.parent.mkdir(parents=True, exist_ok=True)
    with open(filename, "w", encoding="ascii") as f:
        for i in range(N):
            m = max((i + 0.5)/N, X_MIN)
            y = S / np.sqrt(m)
            # clip to max representable in Q7.25 range if you want:
            y = min(y, 63.99999997)   # generous; Q7.25 can represent big values
            val = int(np.round(y * SCALE)) & 0xFFFFFFFF
            f.write(f"{val:08x}\n")

           
def pack128(x=0,y=0,z=0,w=0):
    return u32(w)<<96 | u32(to_fixed(z))<<64 | u32(to_fixed(y))<<32 | u32(to_fixed(x))

def unpack128(val):
    if hasattr(val, 'integer'): val = val.integer
    x = u32(val)
    y = u32(val>>32)
    z = u32(val>>64)
    w = u32(val>>96)
    if x >= 0x80000000: x -= 0x100000000
    if y >= 0x80000000: y -= 0x100000000
    if z >= 0x80000000: z -= 0x100000000
    return from_fixed(x), from_fixed(y), from_fixed(z), w
            
def inv_sqrt_ref(x):
    """
    Golden reference for inv_sqrt used in GGX.
    x is float in [0, 1)
    """
    X_MIN = 2**-15
    N = 1 << 14
    S = 0.25
    if x < X_MIN:
        x = X_MIN
        
    idx = min(max(0,int(x*N)), N-1)
    m = max(X_MIN,((idx+0.5)/N))
    
        
    y0 = S / np.sqrt(m)
    y1 = y0 * (1.5 - x * y0 * y0 * 8.0)
    return float(np.clip(y1,0.0,S/np.sqrt(X_MIN)))

@cocotb.test()
async def test_fixed_norm(dut):
    """Test axis_fixed_norm3 using Scoreboard and Model Callback"""
    # ensure_rom_exists()
    
    # 1. Setup Signal Queues
    sig_out_exp = [] # Expected transactions (Scoreboard queue)
    sig_out_act = [] # Actual transactions (for debug logging only)

    # 2. Define Model (Callback)
    # This runs every time the input monitor sees a valid transaction
    # MAG_CLAMP = 1.0/128.0
    # MAG_CLAMP = np.sqrt(2**(-15))
    def fixed_norm_model(transaction):
        def q131_quantize(f):
            return from_fixed(to_fixed(f))
        
        x, y, z, w = unpack128(transaction)
        lensq = x*x + y*y + z*z
        if lensq < 1e-30:
            nx, ny, nz = 0.0, 0.0, 0.0
            exp_mag = 0.0
        else:
            k = 0
            lensq_scaled = lensq
            while lensq_scaled >= 1.0:
                lensq_scaled *= 0.25
                k += 1
            X_MIN = 2**-15
            lensq_scaled = max(lensq_scaled, X_MIN)
            inv_sqrt_scaled = inv_sqrt_ref(lensq_scaled)
            inv_len = (inv_sqrt_scaled*4.0) / (2.0**k)
            nx, ny, nz = x * inv_len, y * inv_len, z * inv_len
            exp_mag = float(np.sqrt(nx*nx+ny*ny+nz*nz))

        nx, ny, nz = q131_quantize(nx), q131_quantize(ny), q131_quantize(nz)
        sig_out_exp.append(((x,y,z),nx,ny,nz,exp_mag))
        
    def checker_callback(transaction):
        """Pops expected result and compares with tolerance"""
        if not sig_out_exp:
            dut._log.error("Received transaction but no expected value available!")
            assert False, "Queue Empty"

        vec_in, exp_x, exp_y, exp_z, exp_mag = sig_out_exp.pop(0)
        act_x, act_y, act_z, act_w = unpack128(transaction)
        
        # if exp_mag > 0:
        #     assert exp_mag <= 1.0001
        #     if np.sqrt(vec_in[0]**2 + vec_in[1]**2 + vec_in[2]**2) >= MAG_CLAMP:
        #         assert abs(exp_mag - 1.0) < 1e-6

        # Compare Magnitude of result (Should be ~1.0)
        # We also check individual components for direction accuracy
        
        # Calc errors
        err_x = abs(act_x - exp_x)
        err_y = abs(act_y - exp_y)
        err_z = abs(act_z - exp_z)
        
        # Calculate resulting magnitude (should be 1.0)
        act_mag = np.sqrt(act_x**2 + act_y**2 + act_z**2)
        err_mag = abs(act_mag - exp_mag)
        
        # Debug Log
        dut._log.info(f"In: {vec_in} | Exp: ({exp_x:.3f},{exp_y:.3f},{exp_z:.3f}) | Act: ({act_x:.3f},{act_y:.3f},{act_z:.3f}) | Mag: {act_mag:.4f}")
        
        # Debug: peek internal RTL signals at output time
        # lensq_u32 = int(dut.lensq_u32.value)
        # lensq_u32_clamped = int(dut.lensq_u32_clamped.value)
        # inv_len_q7_25 = int(dut.inv_len_q7_25.value)
        # dut._log.info(f"DBG lensq_u32=0x{lensq_u32:08x} clamped=0x{lensq_u32_clamped:08x} inv=0x{inv_len_q7_25:08x}")
        
        # Tolerances
        # Note: Fixed point errors accumulate. 0.05 is safe for 3-stage approx.
        if abs(exp_x) + abs(exp_y) + abs(exp_z) > 0.01: # If not zero input
            # Python-expected lensq in UQ0.32 (roughly matching your RTL)
            # mag2 = vec_in[0]**2 + vec_in[1]**2 + vec_in[2]**2
            # lensq_uq0_32_exp = int(np.floor(mag2 * (2**32))) & 0xFFFFFFFF
            # dut._log.error(f"PY lensq_uq0_32_exp=0x{lensq_uq0_32_exp:08x} (mag2={mag2:.10e})")

            assert err_mag < 0.05, f"Magnitude Mismatch! Expected: {exp_mag}, Got: {act_mag}"
            assert err_x < 0.05 and err_y < 0.05 and err_z < 0.05, "Component Mismatch"

    # 3. Initialize Monitors & Drivers
    inm = AXISMonitor(dut, 's00', dut.s00_axis_aclk, callback=fixed_norm_model)
    outm = AXISMonitor(dut, 'm00', dut.s00_axis_aclk, callback=checker_callback)
    
    ind = AXISDriver(dut, 's00', dut.s00_axis_aclk, "M")
    outd = AXISDriver(dut, 'm00', dut.s00_axis_aclk, "S")

    # 4. Start Simulation
    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    await reset(dut.s00_axis_aclk, dut.s00_axis_aresetn, cycles_held=5, polarity=0)

    # 5. Generate Data
    N=128
    # c = 1.0/128.0
    c = np.sqrt(2**(-15))
    np.random.seed(42)
    
    directed = [(k*c/8, 0, 0) for k in range(0, 40)]  # 0..~5c
    directed += [
        ( 0.999, 0.0,   0.0),
        (-0.999, 0.0,   0.0),
        ( 0.0,   0.999, 0.0),
        ( 0.0,  -0.999, 0.0),
        ( 0.0,   0.0,   0.999),
        ( 0.0,   0.0,  -0.999),
    ]
    directed += [
        (0.999, 0.999, 0.999),
        (-0.999, 0.999, 0.999),
        (0.999, -0.999, 0.999),
        (0.999, 0.999, -0.999),
    ]
    directed += [
        (0.0, 0.0, 0.0),
        (1e-9, 0.0, 0.0),
        (0.0, -1e-9, 0.0),
    ]
    vectors = [(0.006659924518316984, 0.002261493820697069, -0.003101613372564316),
               (0.5,0.0,0.0),(0.0,-0.8,0.0),(0.999,0.999,0.999)]
    for _ in range(N):
        v = np.random.uniform(-1,1,3)
        v = v / np.linalg.norm(v)
        scale = min(2.0 ** np.random.uniform(-7.5, -0.001), 0.999)
        vectors.append(tuple(v*scale))
    vectors = directed + vectors
    input_data = [pack128(v[0],v[1],v[2],0) for v in vectors]
    
    # 6. Drive Sequence
    # Send all inputs in a burst
    ind.append({"type": "write_burst", "contents": {"data": input_data}})

   # Repeated stall pattern
    for _ in range(20):
        outd.append({"type": "read_burst", "duration": 5})
        outd.append({"type": "pause", "duration": 3})

    outd.append({"type": "read_burst", "duration": len(vectors)}) # Read remainder

    # 7. Wait for completion
    # Wait enough cycles for pipeline latency + pauses
    await ClockCycles(dut.s00_axis_aclk, 4 * len(vectors))

    # 8. Assertions
    # If sig_out_exp is not empty, it means we missed outputs.
    assert len(sig_out_exp) == 0, f"Scoreboard mismatch! {len(sig_out_exp)} expected items remaining."
    
    # Ensure no data was lost (Input count == Output count)
    assert inm.transactions == outm.transactions, \
        f"Transaction count mismatch! In: {inm.transactions}, Out: {outm.transactions}"

    dut._log.info("Test Passed with Approximation Tolerance!")

def norm3d_runner():
    """Simulate the Inv Sqrt Module"""
    hdl_toplevel_lang = os.getenv("HDL_TOPLEVEL_LANG", "verilog")
    sim = os.getenv("SIM", "icarus")
    sys.path.append(str(proj_path / "sim" / "model"))
    sys.path.append(str(proj_path / "hdl" ))
    ensure_rom_exists(proj_path / "sim_build" / "inv_sqrt_rom.mem", addr_bits=14)
    sources = [
               proj_path / "hdl" / "axis_fixed_norm3.sv",
               proj_path / "hdl" / "axis_fixed_inv_sqrt.sv", 
            ] 
    
    build_test_args = ["-Wall", "-I", str(proj_path / "hdl")]
    sys.path.append(str(proj_path / "sim"))
    runner = get_runner(sim)
    hdl_toplevel = "axis_fixed_norm3"
    
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
    norm3d_runner()
