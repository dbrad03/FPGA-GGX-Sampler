import cocotb
from cocotb.triggers import RisingEdge, FallingEdge, ReadOnly
from cocotb.clock import Clock
import numpy as np
import os
import sys
from pathlib import Path
from cocotb.utils import get_sim_time as gst
from cocotb.runner import get_runner

test_file = os.path.basename(__file__).replace(".py","")

proj_path = Path(__file__).resolve().parent.parent

# --- 1. CONFIGURATION ---
FRAC_BITS = 32
# Scale for Q1.31
SCALE = 2.0**(FRAC_BITS - 1)

def to_fixed(f):
    """Float -> Q1.31 signed integer"""
    f = np.clip(f, -0.999999, 0.999999)
    return int(f * SCALE)

def from_fixed(i):
    """Q1.31 signed integer -> Float"""
    if i >= (1 << (FRAC_BITS-1)):
        i -= (1 << FRAC_BITS)
    return i / SCALE

def ensure_rom_exists():
    """Generates inv_sqrt_rom.mem if missing."""
    # if os.path.exists("inv_sqrt_rom.mem"): return
    
    print("Generating inv_sqrt_rom.mem...")
    N = 1 << 8 # ADDR_BITS = 8
    with open("inv_sqrt_rom.mem", "w") as f:
        for i in range(N):
            u = (i + 0.5) / N
            x = max(u, 1e-6)
            # Scaling by 0.5 to fit in Q1.31 range
            y = (1.0 / np.sqrt(x)) * 0.25
            y = min(y,0.9999999)
            val = int(y * (1<<31)) & 0xFFFFFFFF
            f.write(f"{val:08x}\n")

# --- 2. TESTS ---

@cocotb.test()
async def test_inv_sqrt(dut):
    """Test fixed_inv_sqrt_newton"""
    rising_edge = RisingEdge(dut.clk)
    falling_edge = FallingEdge(dut.clk)
    ensure_rom_exists()
    
    # Setup Clock and Reset
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst_n.value = 0
    await rising_edge
    dut.rst_n.value = 1
    dut.in_valid.value = 0
    await rising_edge

    vectors = [
        # 0.001,   # Very small -> Large result (Check 0.25x scaling limits)
        0.05,    # Small perfect square (sqrt=0.2, inv=5.0, scaled=1.25)
        0.1,     # Original test case
        0.25,    # Perfect square (sqrt=0.5, inv=2.0, scaled=0.5)
        0.3333,  # Repeating decimal
        0.5,     # Mid-range
        0.64,    # Perfect square (sqrt=0.8, inv=1.25, scaled=0.3125)
        0.8,     # High range
        0.9,     # Very high range
        0.99,    # Near 1.0 boundary
        0.9999   # Edge case max
    ]
    
    for val in vectors:
        await falling_edge
        dut.in_x.value = to_fixed(val)
        dut.in_valid.value = 1
        await rising_edge
        dut.in_valid.value = 0
        
        # Wait for output
        for _ in range(10):
            await ReadOnly()
            if dut.out_valid.value == 1: break
            await rising_edge
            
        res = from_fixed(dut.out_y.value.signed_integer)
        exp = (1.0 / np.sqrt(val)) * 0.25 # Expect scaled output
        
        dut._log.info(f"Input: {val} | Out: {res:.4f} | Exp: {exp:.4f}")
        assert abs(res - exp) < 0.05, "Inverse Sqrt logic failed!"

def inv_sqrt_runner():
    """Simulate the Owen-Scrambling Streamer using the Python runner."""
    hdl_toplevel_lang = os.getenv("HDL_TOPLEVEL_LANG", "verilog")
    sim = os.getenv("SIM", "icarus")
    # sim = os.getenv("SIM", "vivado")
    sys.path.append(str(proj_path / "sim" / "model"))
    sys.path.append(str(proj_path / "hdl" ))
    sources = [
               proj_path / "hdl" / "fixed_inv_sqrt_newton.sv",
            ] 
    
    build_test_args = ["-Wall"], #"-I", str(proj_path / "hdl")]
    parameters = {} #!!!
    sys.path.append(str(proj_path / "sim"))
    runner = get_runner(sim)
    hdl_toplevel = "fixed_inv_sqrt_newton"
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