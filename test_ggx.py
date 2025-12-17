import matplotlib.pyplot as plt
import cocotb
from cocotb.triggers import Timer, ClockCycles, RisingEdge, FallingEdge, ReadOnly,with_timeout
from cocotb.clock import Clock
import numpy as np
import os
import sys
from pathlib import Path
from cocotb.binary import BinaryValue
from cocotb.utils import get_sim_time as gst
from cocotb.runner import get_runner
from cocotb_bus.bus import Bus
from cocotb_bus.drivers import BusDriver
from cocotb_bus.monitors import Monitor
from cocotb_bus.monitors import BusMonitor
from cocotb_bus.scoreboard import Scoreboard

test_file = os.path.basename(__file__).replace(".py","")
proj_path = Path(__file__).resolve().parent.parent

FRAC_BITS = 32
SCALE = 2.0**(FRAC_BITS - 1)

def to_fixed(f):
    f = np.clip(f, -0.999999, 0.999999)
    return int(f * SCALE)

def to_fixed_u32(f):
    """ Converts float [0, 1) to Unsigned Q0.32 """
    # Scale by 2^32 to use the full 32-bit integer range
    f = np.clip(f, 0.0, 0.999999999)
    return int(f * 4294967296.0) # 2^32
    # return int(f * 2**32)

def from_fixed(i):
    if i >= (1 << (FRAC_BITS-1)): i -= (1 << FRAC_BITS)
    return i / SCALE

# --- GOLDEN MODEL ---
def normalize(v):
    v = np.asarray(v, dtype=np.float64)
    n = np.linalg.norm(v)
    if n == 0.0: return v
    return v / n
def ggx_vndf_spherical_caps(u1, u2, view, alpha):
    view = normalize(np.asarray(view, dtype=np.float64))
    alpha = float(alpha)
    wi_std = normalize(np.array([view[0]*alpha, view[1], view[2]*alpha]))
    phi = 2.0 * np.pi * u1
    z = (1.0 - u2) * (1.0 + wi_std[2]) - wi_std[2]
    sin_theta = np.sqrt(max(0.0, 1.0 - z*z))
    x = sin_theta * np.cos(phi)
    y = sin_theta * np.sin(phi)
    c = np.array([x, y, z])
    h_std = c + wi_std
    h = normalize(np.array([h_std[0]*alpha, h_std[1], h_std[2]*alpha]))
    return h

# --- ROM GEN ---
def ensure_roms_exist():
    if not os.path.exists("inv_sqrt_rom.mem"):
        print("Generating inv_sqrt_rom.mem...")
        N = 1 << 8
        with open("inv_sqrt_rom.mem", "w") as f:
            for i in range(N):
                u = (i + 0.5) / N
                x = max(u, 1e-6)
                y = (1.0 / np.sqrt(x)) * 0.25 
                val = int(min(y, 0.9999999) * (1 << 31)) & 0xFFFFFFFF
                f.write(f"{val:08x}\n")

    if not os.path.exists("ggx_trig_rom.mem"):
        print("Generating ggx_trig_rom.mem...")
        N_TRIG = 1 << 10
        with open("ggx_trig_rom.mem", "w") as f:
            for i in range(N_TRIG):
                u = i / N_TRIG
                phi = 2.0 * np.pi * u
                c = int(to_fixed(np.cos(phi))) & 0xFFFFFFFF
                s = int(to_fixed(np.sin(phi))) & 0xFFFFFFFF
                f.write(f"{c:08x}{s:08x}\n") # Cos High, Sin Low
                
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
    
@cocotb.test()
async def test_ggx_distribution(dut):
    """Test GGX with many samples and plot distribution."""
    rising_edge = RisingEdge(dut.s00_axis_aclk)
    falling_edge = FallingEdge(dut.s00_axis_aclk)
    
    def to_float(raw):
        if raw >= 0x80000000: raw -= 0x100000000
        return raw / 2147483648.0 # Divide by 2^31
    
    received_packed_data = []
    
    inm = AXISMonitor(dut,'s00',dut.s00_axis_aclk)#, callback=debug_input)
    outm = AXISMonitor(dut,'m00',dut.s00_axis_aclk, 
                       callback=lambda x: received_packed_data.append(x.integer))
    ind = AXISDriver(dut,'s00',dut.s00_axis_aclk,"M") #M driver for S port
    # outd = AXISDriver(dut,'m00',dut.s00_axis_aclk,"S") #S driver for M port
    
    ensure_roms_exist()
    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    await reset(dut.s00_axis_aclk,dut.s00_axis_aresetn,cycles_held=5,polarity=0)

    # Config
    alpha_val = 0.4
    view_vec = [0.0, 1.0, 0.0] # Straight up view
    
    dut.alpha.value = to_fixed(alpha_val)
    dut.view_x.value = to_fixed(view_vec[0])
    dut.view_y.value = to_fixed(view_vec[1])
    dut.view_z.value = to_fixed(view_vec[2])
    dut.m00_axis_tready.value = 1
    
    dut._log.info("\n--- RUNNING PHASE SWEEP ---")
    
    # Test 4 cardinal directions: 0.0, 0.25, 0.5, 0.75
    points = [0, .25, .5, .6, .75, 0.9, 0.99]
    
    # We keep u1 constant to isolate rotation (u1)
    u1_val = 0.5 
    u1_fix = to_fixed_u32(u1_val) & 0xFFFFFFFF
    
    for u0_val in points:
        u0_fix = to_fixed_u32(u0_val) & 0xFFFFFFFF
        packed = (u1_fix << 32) | u0_fix
        val = BinaryValue(value=packed, n_bits=64, bigEndian=False)
        ind.append({'type': 'write_single', 'contents': {'data': val, 'last': 0}})
        
    # for _ in points:
    #     outd.append({'type': "read_single"})
    
    while len(received_packed_data) < len(points):
        await RisingEdge(dut.s00_axis_aclk)
        await ReadOnly()
        
    for i, u0_val in enumerate(points):
        received_raw = received_packed_data[i]
        hx_raw = received_raw & 0xFFFFFFFF
        hy_raw = (received_raw >> 32) & 0xFFFFFFFF
        hz_raw = (received_raw >> 64) & 0xFFFFFFFF
        rtl_x = to_float(hx_raw)
        rtl_y = to_float(hy_raw)
        rtl_z = to_float(hz_raw)
        
        # Calculate Reference (Z-Up)
        ref_zup = ggx_vndf_spherical_caps(u0_val, 0.5, [0,1,0], alpha_val)
        
        # FIX B: Swizzle Reference to match RTL (Y-Up)
        # Ref Z (Up)    -> RTL Y
        # Ref Y (Depth) -> RTL Z
        # Ref X (Side)  -> RTL X
        ref_yup = [ref_zup[0], ref_zup[1], ref_zup[2]]
        
        dut._log.info(f"u0={u0_val:<4} | RTL: {rtl_x:>7.4f} {rtl_y:>7.4f} {rtl_z:>7.4f} | REF: {ref_yup[0]:>7.4f} {ref_yup[1]:>7.4f} {ref_yup[2]:>7.4f}")
        
        # Verify all three
        # if not (np.isclose(rtl_x, ref_yup[0], atol=0.01) and 
        #         np.isclose(rtl_y, ref_yup[1], atol=0.01) and 
        #         np.isclose(rtl_z, ref_yup[2], atol=0.01)):
        #      dut._log.error(f"MISMATCH AT u0={u0_val}")
        
   
def ggx_runner():
    """Simulate the GGX VNDF Streamer using the Python runner."""
    hdl_toplevel_lang = os.getenv("HDL_TOPLEVEL_LANG", "verilog")
    sim = os.getenv("SIM", "icarus")
    # sim = os.getenv("SIM", "vivado")
    sys.path.append(str(proj_path / "sim" / "model"))
    sys.path.append(str(proj_path / "hdl" ))
    sources = [
            proj_path / "hdl" / "axis_ggx_vndf.sv",
            proj_path / "hdl" / "ggx_trig_lut.sv", 
            proj_path / "hdl" / "fixed_inv_sqrt_newton.sv",
            proj_path / "hdl" / "fixed_norm3.sv" 
            ] 
    
    build_test_args = ["-Wall"], #"-I", str(proj_path / "hdl")]
    parameters = {} #!!!
    sys.path.append(str(proj_path / "sim"))
    runner = get_runner(sim)
    hdl_toplevel = "axis_ggx_vndf"
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
    ggx_runner()
