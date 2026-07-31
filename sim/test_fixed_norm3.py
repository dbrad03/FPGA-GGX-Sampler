import cocotb
from cocotb.triggers import RisingEdge, FallingEdge, ReadOnly
from cocotb.clock import Clock
import numpy as np
import os
import sys
from pathlib import Path
from cocotb.utils import get_sim_time as gst
from cocotb.runner import get_runner
from sources import sources_for

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

def ensure_rom_exists(filename, addr_bits=14):
    """Generates inv_sqrt_rom.mem (16384-entry Q7.25) for axis_fixed_inv_sqrt."""
    print(f"Generating {filename}...")
    N = 1 << addr_bits
    Q = 25
    SCALE_ROM = 1 << Q
    X_MIN = 2**-15
    S = 0.25
    from pathlib import Path as _Path
    _Path(filename).parent.mkdir(parents=True, exist_ok=True)
    with open(filename, "w") as f:
        for i in range(N):
            m = max((i + 0.5) / N, X_MIN)
            y = S / np.sqrt(m)
            y = min(y, 63.99999997)
            val = int(np.round(y * SCALE_ROM)) & 0xFFFFFFFF
            f.write(f"{val:08x}\n")

# --- 2. TESTS ---

@cocotb.test()
async def test_norm3(dut):
    """Test fixed_norm3"""
    rising_edge = RisingEdge(dut.clk)
    falling_edge = FallingEdge(dut.clk)
    ensure_rom_exists()
    
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst_n.value = 0
    await rising_edge
    dut.rst_n.value = 1
    dut.in_valid.value = 0
    await rising_edge

    vectors = [
        # 1. Axis-aligned (Safe)
        (0.5, 0.0, 0.0),       
        (0.0, -0.8, 0.0),      
        (0.0, 0.0, 0.25),

        # 2. Diagonal / Uniform
        # 0.577^2 * 3 ~= 0.999. Close to limit.
        (0.577, 0.577, 0.577), 
        (-0.5, -0.5, -0.5),    # Negative inputs

        # 3. Random-ish valid vectors
        (0.1, 0.2, 0.3),       # Mag ~0.37
        (0.4, 0.4, 0.4),       # Mag ~0.69
        (0.6, 0.0, 0.6),       # Mag ~0.85
        
        (0.15, 0.15, 0.15)     
    ]

    for vx, vy, vz in vectors:
        await falling_edge
        dut.in_x.value = to_fixed(vx)
        dut.in_y.value = to_fixed(vy)
        dut.in_z.value = to_fixed(vz)
        dut.in_valid.value = 1
        await rising_edge
        await falling_edge
        dut.in_valid.value = 0
    
        for _ in range(20):
            await ReadOnly()
            if dut.out_valid.value == 1: break
            await rising_edge
        
        await ReadOnly()
        out_x = from_fixed(dut.out_x.value.signed_integer)
        out_y = from_fixed(dut.out_y.value.signed_integer)
        out_z = from_fixed(dut.out_z.value.signed_integer)
        mag = np.sqrt(out_x**2 + out_y**2 + out_z**2)
        in_mag = np.sqrt(vx**2 + vy**2 + vz**2)
        dot = (vx*out_x + vy*out_y + vz*out_z) / in_mag
        
        dut._log.info(f"In:({vx:.2f},{vy:.2f},{vz:.2f}) -> Out:({out_x:.2f},{out_y:.2f},{out_z:.2f}) | Mag:{mag:.4f} | Dot:{dot:.4f}")

        assert abs(mag - 1.0) < 0.05, f"Magnitude Mismatch! Got {mag}"
        assert abs(dot - 1.0) < 0.05, f"Direction Mismatch! Dot product {dot} != 1.0"
    
def norm_runner():
    """Simulate the Owen-Scrambling Streamer using the Python runner."""
    hdl_toplevel_lang = os.getenv("HDL_TOPLEVEL_LANG", "verilog")
    sim = os.getenv("SIM", "icarus")
    sys.path.append(str(proj_path / "sim" / "model"))
    sys.path.append(str(proj_path / "hdl" ))
    ensure_rom_exists(proj_path / "sim" / "sim_build" / "inv_sqrt_rom.mem", addr_bits=14)
    sources = sources_for("axis_fixed_norm3")

    build_test_args = ["-Wall", "-I", str(proj_path / "hdl")]
    parameters = {}
    sys.path.append(str(proj_path / "sim"))
    runner = get_runner(sim)
    hdl_toplevel = "axis_fixed_norm3"
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
    norm_runner()