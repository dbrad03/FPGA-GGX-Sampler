#!/usr/bin/env python3
"""
FPGA vs CPU speedup estimate for the GGX VNDF sampling pipeline.

CPU side: measures pure-Python reference (same math as the RTL)
         + NumPy vectorized reference (fair upper bound for optimized CPU)
FPGA side: analytical — 100 MHz, 1 sample/cycle steady state.

Run from final/sim/ with the project venv active:
    python benchmark_speedup.py
"""

import time
import numpy as np
from functools import reduce
from pathlib import Path

# ─── fixed-point helpers (same as test_ggx_control.py) ──────────────────────
Q31 = 2**31
ADDR_BITS = 10   # trig LUT address bits
NORM_X_MIN = 2**-15

def u32(x):
    return int(x) & 0xFFFF_FFFF

def uq032_to_float(u):
    return float(u32(u)) / (2**32)

def q131_to_float(x):
    x = u32(x)
    if x & 0x8000_0000:
        x -= 0x1_0000_0000
    return float(x) / Q31

def float_to_q131(f):
    f = float(f)
    if f >= 1.0:   f = np.nextafter(1.0, 0.0)
    if f < -1.0:   f = -1.0
    v = int(np.round(f * Q31))
    v = max(min(v, 0x7FFF_FFFF), -0x8000_0000)
    return v & 0xFFFF_FFFF

def normalize3(v):
    v = np.asarray(v, dtype=np.float64)
    n = np.linalg.norm(v)
    return (v / n) if n > 1e-30 else np.zeros(3)

# ─── LUT-based inv_sqrt (mirrors RTL Newton step) ────────────────────────────
def inv_sqrt_ref(x):
    x_min = 2**-15
    n = 1 << 14
    s = 0.25
    x = max(x, x_min)
    idx = min(max(0, int(x * n)), n - 1)
    m   = max(x_min, (idx + 0.5) / n)
    y0  = s / np.sqrt(m)
    return float(np.clip(y0 * (1.5 - x * y0 * y0 * 8.0), 0.0, s / np.sqrt(x_min)))

# ─── norm3 (mirrors RTL fixed-point path) ────────────────────────────────────
def norm3_ref(v):
    v = np.asarray(v, dtype=np.float64)
    lensq = float(np.dot(v, v))
    if lensq < 1e-30:
        return np.zeros(3)
    k, ls = 0, lensq
    while ls >= 1.0:
        ls *= 0.25; k += 1
    ls = max(ls, NORM_X_MIN)
    inv_len = inv_sqrt_ref(ls) * 4.0 / (2.0**k)
    n = v * inv_len
    return np.array([q131_to_float(float_to_q131(c)) for c in n])

# ─── event basis ─────────────────────────────────────────────────────────────
def event_basis_ref(alpha, view_q):
    vx, vy, vz = [q131_to_float(x) for x in view_q]
    view = normalize3([vx, vy, vz])
    vh   = normalize3([alpha * view[0], alpha * view[1], view[2]])
    lensq = vh[0]**2 + vh[1]**2
    if lensq > 1e-20:
        inv_len = inv_sqrt_ref(max(lensq, NORM_X_MIN)) * 4.0
        t1 = np.array([-vh[1]*inv_len,  vh[0]*inv_len, 0.0])
        t2 = np.cross(vh, t1)
    else:
        t1 = np.array([1., 0., 0.])
        t2 = np.array([0., 1., 0.])
    return vh, t1, t2

# ─── projected area ──────────────────────────────────────────────────────────
def projected_ref(vhz_q, u2_uq, u1_uq):
    vhz = q131_to_float(vhz_q)
    u1  = uq032_to_float(u1_uq)
    idx = (u32(u2_uq) >> (32 - ADDR_BITS)) & ((1 << ADDR_BITS) - 1)
    ang = (idx / float(1 << ADDR_BITS)) * 2.0 * np.pi
    c, s = np.cos(ang), np.sin(ang)
    r    = np.sqrt(max(0.0, u1))
    t1, t2 = r * c, r * s
    blend = 0.5 * (1.0 + vhz)
    t2 = (1.0 - blend) * np.sqrt(max(0.0, 1.0 - t1**2)) + blend * t2
    return t1, t2

# ─── Sobol + Owen scramble + hash (same as test_ggx_control.py) ──────────────
def rotl32(x, r):
    x = u32(x); return u32((x << r) | (x >> (32 - r)))

def fmix32(h):
    h = u32(h)
    h ^= h >> 16;    h = u32(h * 0x85EBCA6B)
    h ^= h >> 13;    h = u32(h * 0xC2B2AE35)
    h ^= h >> 16
    return u32(h)

def mix(h, k):
    h, k = u32(h), u32(k)
    k = u32(k * 0xCC9E2D51)
    k = rotl32(k, 15)
    k = u32(k * 0x1B873593)
    h ^= k
    h = rotl32(h, 13)
    return u32(h * 5 + 0xE6546B64)

def hash_combine(*vals):
    return fmix32(reduce(mix, (u32(v) for v in vals), u32(0x9747B28C)))

def reverse_bits(x, nbits=32):
    out = 0
    for _ in range(nbits):
        out = (out << 1) | (x & 1); x >>= 1
    return u32(out)

def laine_karras(x, seed):
    x = u32(x + seed)
    for k in [0x6C50B47C, 0xB82F1E52, 0xC7AFE638, 0x8D22F6E6]:
        x ^= u32(x * k)
    return x

def nus(x, seed):
    return reverse_bits(laine_karras(reverse_bits(x), seed))

def sobol_dir_table():
    frac = 32
    v = np.zeros((2, frac), dtype=np.uint64)
    for k in range(frac):
        v[0, k] = np.uint64(1 << (frac - 1 - k))
    m = [1, 3]
    for i in range(2):
        v[1, i] = np.uint64(u32(m[i] << (frac - 1 - i)))
    for i in range(2, frac):
        val = int(v[1, i-2]) ^ (int(v[1, i-2]) >> 2)
        if (1 >> 0) & 1:
            val ^= int(v[1, i-1]) >> 1
        v[1, i] = np.uint64(u32(val))
    return v

def sobol_u32(dim, idx, dtable):
    g = u32(idx) ^ (u32(idx) >> 1)
    x, bit = 0, 0
    while g and bit < dtable.shape[1]:
        if g & 1: x ^= int(dtable[dim][bit])
        g >>= 1; bit += 1
    return u32(x)

def sample_sobol_scrambled(i, seed, dtable):
    idx = nus(i, seed)
    s0  = sobol_u32(0, idx, dtable)
    s1  = sobol_u32(1, idx, dtable)
    u0  = nus(s0, hash_combine(seed, 0))
    u1  = nus(s1, hash_combine(seed, 1))
    return u0, u1

# ─── full per-sample computation (matches control_ref_sequence) ──────────────
def ggx_vndf_sample_python(i, seed, vhz_q, vh, t1_vec, t2_vec, dtable):
    u0, u1  = sample_sobol_scrambled(i, seed, dtable)
    pt1, pt2 = projected_ref(vhz_q, u1, u0)
    t3      = np.sqrt(max(0.0, 1.0 - pt1**2 - pt2**2))
    h_unnorm = pt1 * t1_vec + pt2 * t2_vec + t3 * vh
    return norm3_ref(h_unnorm)

# ─── NumPy vectorized reference (fair upper-bound for optimized CPU) ─────────
def ggx_vndf_batch_numpy(n_samples, seed, vhz, vh, t1_vec, t2_vec, dtable):
    """Vectorized over samples; still pure Python for Sobol/scramble."""
    u0s = np.empty(n_samples, dtype=np.uint32)
    u1s = np.empty(n_samples, dtype=np.uint32)
    for i in range(n_samples):
        u0s[i], u1s[i] = sample_sobol_scrambled(i, seed, dtable)

    # projected area — vectorized
    idxs  = (u1s >> (32 - ADDR_BITS)) & ((1 << ADDR_BITS) - 1)
    angs  = (idxs / float(1 << ADDR_BITS)) * 2.0 * np.pi
    c, s  = np.cos(angs), np.sin(angs)
    r     = np.sqrt(np.maximum(0.0, u0s.astype(np.float64) / 2**32))
    pt1   = r * c
    pt2   = r * s
    blend = 0.5 * (1.0 + vhz)
    pt2   = (1.0 - blend) * np.sqrt(np.maximum(0.0, 1.0 - pt1**2)) + blend * pt2

    # reproject — vectorized
    t3    = np.sqrt(np.maximum(0.0, 1.0 - pt1**2 - pt2**2))
    hx    = pt1 * t1_vec[0] + pt2 * t2_vec[0] + t3 * vh[0]
    hy    = pt1 * t1_vec[1] + pt2 * t2_vec[1] + t3 * vh[1]
    hz    = pt1 * t1_vec[2] + pt2 * t2_vec[2] + t3 * vh[2]
    inv_n = 1.0 / np.sqrt(hx**2 + hy**2 + hz**2 + 1e-30)
    return np.stack([hx*inv_n, hy*inv_n, hz*inv_n], axis=1)

# ─── FPGA throughput model ───────────────────────────────────────────────────
FPGA_CLK_HZ     = 100e6          # 100 MHz
PIPELINE_LATENCY = 80            # ~80 cycles from first input to first output (conservative)
BEATS_PER_CMD    = 3             # 3 AXI beats to launch a burst
CLK_PERIOD_NS    = 1e9 / FPGA_CLK_HZ

def fpga_time_ns(n_samples):
    """Steady-state cycles: latency fill + 1 cycle/sample."""
    cycles = PIPELINE_LATENCY + n_samples
    return cycles * CLK_PERIOD_NS

# ─── benchmark harness ───────────────────────────────────────────────────────
def run_benchmark():
    print("=" * 70)
    print("  GGX VNDF Sampling — FPGA vs CPU Speedup Estimate")
    print("=" * 70)

    dtable = sobol_dir_table()
    seed   = 0xDEADBEEF
    alpha  = 0.4
    view   = normalize3([0.1, 0.3, 0.9])

    vx_q = float_to_q131(view[0])
    vy_q = float_to_q131(view[1])
    vz_q = float_to_q131(view[2])
    alpha_f = alpha

    vh, t1_vec, t2_vec = event_basis_ref(alpha_f, [vx_q, vy_q, vz_q])
    vhz_q = float_to_q131(vh[2])
    vhz   = q131_to_float(vhz_q)

    sample_sizes = [64, 256, 1024, 4096, 16384]

    print(f"\nFPGA: {FPGA_CLK_HZ/1e6:.0f} MHz, 1 sample/cycle (fully pipelined)")
    print(f"      pipeline latency ≈ {PIPELINE_LATENCY} cycles, {BEATS_PER_CMD} AXI beats/burst\n")

    print(f"{'N':>8} | {'Python (ms)':>11} | {'NumPy (ms)':>10} | {'FPGA (µs)':>9} | {'Speedup vs Py':>13} | {'Speedup vs NP':>13}")
    print("-" * 83)

    for N in sample_sizes:
        # ── Python reference ──────────────────────────────────────────────
        warmup = ggx_vndf_sample_python(0, seed, vhz_q, vh, t1_vec, t2_vec, dtable)
        REPS = max(1, min(5, 20_000 // N))
        t0 = time.perf_counter()
        for _ in range(REPS):
            for i in range(N):
                ggx_vndf_sample_python(i, seed, vhz_q, vh, t1_vec, t2_vec, dtable)
        py_ms = (time.perf_counter() - t0) / REPS * 1e3

        # ── NumPy vectorized ──────────────────────────────────────────────
        t0 = time.perf_counter()
        for _ in range(REPS):
            ggx_vndf_batch_numpy(N, seed, vhz, vh, t1_vec, t2_vec, dtable)
        np_ms = (time.perf_counter() - t0) / REPS * 1e3

        # ── FPGA analytical ───────────────────────────────────────────────
        fpga_us = fpga_time_ns(N) / 1e3

        py_speedup = py_ms / (fpga_us / 1e3)
        np_speedup = np_ms / (fpga_us / 1e3)

        print(f"{N:>8} | {py_ms:>11.2f} | {np_ms:>10.2f} | {fpga_us:>9.2f} | {py_speedup:>12.0f}x | {np_speedup:>12.0f}x")

    # ── throughput summary ────────────────────────────────────────────────
    print("\n── Steady-state throughput ─────────────────────────────────────────")
    fpga_msps = FPGA_CLK_HZ / 1e6
    print(f"  FPGA  : {fpga_msps:.0f} Msamples/sec  (100 MHz × 1 sample/cycle)")

    # large-N Python rate
    N_big = 4096
    t0 = time.perf_counter()
    for i in range(N_big):
        ggx_vndf_sample_python(i, seed, vhz_q, vh, t1_vec, t2_vec, dtable)
    py_rate = N_big / (time.perf_counter() - t0) / 1e3
    print(f"  Python: {py_rate:.2f} Ksamples/sec")

    t0 = time.perf_counter()
    ggx_vndf_batch_numpy(N_big, seed, vhz, vh, t1_vec, t2_vec, dtable)
    np_rate = N_big / (time.perf_counter() - t0) / 1e3
    print(f"  NumPy : {np_rate:.2f} Ksamples/sec")

    print(f"\n  FPGA speedup over Python : {fpga_msps*1e3 / py_rate:.0f}×")
    print(f"  FPGA speedup over NumPy  : {fpga_msps*1e3 / np_rate:.0f}×")

    print("\n── Latency to first sample ─────────────────────────────────────────")
    print(f"  FPGA  : ~{PIPELINE_LATENCY * CLK_PERIOD_NS:.0f} ns  ({PIPELINE_LATENCY} cycles × 10 ns)")
    print(f"  Python: ~{1.0/py_rate*1e3:.0f} µs per sample  (sequential, single-core)")
    print(f"  NumPy : ~{1.0/np_rate*1e3:.0f} µs per sample  (first sample of batch)")

    print("\nNote: FPGA latency includes ~80-cycle pipeline fill.")
    print("      Once filled, output rate is 1 sample per 10 ns (100 Msamples/sec).")
    print("      DMA transfer overhead not included (adds ~µs for PCIe/AXI setup).")


if __name__ == "__main__":
    run_benchmark()
