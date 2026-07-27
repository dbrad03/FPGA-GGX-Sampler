#!/usr/bin/env python3
"""Debug the 3D CORDIC normalize core: gain correction + convergence range."""
import numpy as np

N = 20
GAIN = 1.0
for i in range(N):
    GAIN *= np.sqrt(1.0 + 2.0 ** (-2 * i))
INV_GAIN = 1.0 / GAIN
INV_GAIN2 = 1.0 / (GAIN * GAIN)


def vectoring(a, b):
    sig = []
    for i in range(N):
        s = -1.0 if b >= 0 else 1.0
        a, b = a - s * b * 2.0 ** (-i), b + s * a * 2.0 ** (-i)
        sig.append(s)
    return a, b, sig


def rotate(a, b, sig):
    for i in range(N):
        s = sig[i]
        a, b = a - s * b * 2.0 ** (-i), b + s * a * 2.0 ** (-i)
    return a, b


def normalize3(vx, vy, vz):
    # Range reduction: circular vectoring needs the 'a' input >= 0.
    fx = 1.0
    if vx < 0:            # reflect the whole vector; undo at the end
        vx, vy, vz = -vx, -vy, -vz
        fx = -1.0
    # PASS 1: plane 1 (xy) -> fold y into x
    x, y, sig1 = vectoring(vx, vy)
    x *= INV_GAIN         # remove plane-1 gain so plane-2 inputs are consistent
    # plane 2 (xz): x>=0 always now (x = |(vx,vy)| >= 0)
    x, z, sig2 = vectoring(x, vz)
    # PASS 2: reconstruct v/|v| = R1^-1 R2^-1 e_x
    ax, ay, az = INV_GAIN2, 0.0, 0.0
    ax, az = rotate(ax, az, [-s for s in sig2])
    ax, ay = rotate(ax, ay, [-s for s in sig1])
    return np.array([fx * ax, fx * ay, fx * az * GAIN])  # z saw one fewer rotation -> one gain short


def magnitude3(vx, vy, vz):
    if vx < 0:
        vx, vy, vz = -vx, -vy, -vz
    x, _, _ = vectoring(vx, vy)
    x *= INV_GAIN
    x, _, _ = vectoring(x, vz)
    return x * INV_GAIN


rng = np.random.default_rng(2)
worst = []
md = mm = 0.0
for _ in range(50000):
    v = rng.uniform(-1, 1, 3)
    if np.linalg.norm(v) < 1e-3:
        continue
    ref = v / np.linalg.norm(v)
    got = normalize3(*v)
    de = np.linalg.norm(got - ref)
    me = abs(magnitude3(*v) - np.linalg.norm(v))
    if de > md:
        md = de; worst = (v.copy(), got.copy(), ref.copy())
    mm = max(mm, me)
print(f"N={N} max dir err={md:.3e} max mag err={mm:.3e}")
if worst:
    print("worst v   =", np.round(worst[0], 4))
    print("     got  =", np.round(worst[1], 4))
    print("     ref  =", np.round(worst[2], 4))
