#!/usr/bin/env python3
"""Fixed-point CORDIC normalize: sweep N (iterations/plane) x W (internal frac
bits) to pick the RTL precision. Models exactly what the RTL will do: integer
shift-adds, and QUANTIZED gain-compensation constants (1/G between planes, xG on
z) -- the terms the user flagged as the compounded-error risk.

Internal format: signed, W fractional bits, a few integer bits of headroom (x
grows by ~G^2*sqrt(3) ~= 4.7 during two-plane vectoring, so >=3 int bits).
Input/output compared as real numbers.
"""
import numpy as np


def build(N):
    g = 1.0
    for i in range(N):
        g *= np.sqrt(1.0 + 2.0 ** (-2 * i))
    return g


def q(val, W):
    return int(np.floor(val * (1 << W) + 0.5))  # round-to-nearest fixed-point


def fx_normalize(vx, vy, vz, N, W, invg_q, g_q):
    """All ints scaled by 2^W. invg_q = round(1/G * 2^W), g_q = round(G * 2^W)."""
    ONE = 1 << W
    # to fixed
    x = q(vx, W); y = q(vy, W); z = q(vz, W)
    fx = 1
    if x < 0:
        x, y, z = -x, -y, -z
        fx = -1

    def vec(a, b):
        sig = []
        for i in range(N):
            s = -1 if b >= 0 else 1
            na = a - s * (b >> i)
            nb = b + s * (a >> i)
            a, b = na, nb
            sig.append(s)
        return a, b, sig

    def rot(a, b, sig):
        for i in range(N):
            s = sig[i]
            na = a - s * (b >> i)
            nb = b + s * (a >> i)
            a, b = na, nb
        return a, b

    # PASS 1 vectoring
    x, y, sig1 = vec(x, y)
    # rescale x by 1/G (quantized constant multiply): x*invg_q >> W
    x = (x * invg_q) >> W
    x, zc, sig2 = vec(x, z)
    # PASS 2 reconstruct: seed (1/G^2, 0, 0) = q(1/G^2)
    seed = q(1.0 / (build(N) ** 2), W)
    ax, ay, az = seed, 0, 0
    ax, az = rot(ax, az, [-s for s in sig2])
    ax, ay = rot(ax, ay, [-s for s in sig1])
    az = (az * g_q) >> W                      # z: one gain short -> xG (quantized)
    return np.array([fx * ax, fx * ay, fx * az], dtype=float) / ONE


rng = np.random.default_rng(3)
vecs = []
for _ in range(20000):
    v = rng.uniform(-1, 1, 3)
    if np.linalg.norm(v) > 1e-2:
        vecs.append(v)
# add hard edge cases: axis-aligned, tiny, near-diagonal
for e in ([1,0,0],[0,1,0],[0,0,1],[-1,0,0],[0,0,-1],[1,1,1],[-1e-2,1e-2,1],[1,-1e-3,1e-3]):
    vecs.append(np.array(e, float))

print(f"{'N':>3} {'W':>3} {'max_dir_err':>12} {'max_mag_dev':>12}")
for N in (12, 14, 16, 18, 20):
    G = build(N)
    for W in (20, 24, 27):
        invg_q = q(1.0 / G, W)
        g_q = q(G, W)
        md = mm = 0.0
        for v in vecs:
            ref = v / np.linalg.norm(v)
            got = fx_normalize(*v, N, W, invg_q, g_q)
            md = max(md, np.linalg.norm(got - ref))
            mm = max(mm, abs(np.linalg.norm(got) - 1.0))
        print(f"{N:>3} {W:>3} {md:>12.3e} {mm:>12.3e}")
