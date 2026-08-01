#!/usr/bin/env python3
"""Octahedral output encoding: Python model + the measurement that picks the field width.

Issue #13 / ADR-0001. The ADR asserts that 8-bit fields quantize direction to
roughly 0.5-1 degrees -- a large fraction of the GGX lobe at the smallest
roughness the tests exercise -- and that 16-bit fields are around 0.002 degrees.
That was an argument. Running this module turns it into a measurement.

    python sim/oct32_model.py

The encode/decode here is the oracle the encoder RTL unit test is written
against, so keep it dependency-free and obvious rather than clever.

Encoding
--------
p = v / (|x| + |y| + |z|)          -- L1 projection onto the octahedron
z >= 0 :  (u, w) = (p.x, p.y)
z <  0 :  fold the lower hemisphere outward across the |u|+|w| = 1 diamond

This is exactly scale-invariant: oct(c*v) == oct(v) for any c > 0, which is
what lets the per-sample L2 normalize be deleted (ADR-0001).
"""
import numpy as np

OCT32_BITS = 16  # per field; Oct32 = 2 x 16b


# ---------------------------------------------------------------------------
# Encode / decode
# ---------------------------------------------------------------------------

def _sign_nonzero(x):
    """sign() that returns +1 at zero, matching the usual octahedral fold."""
    return np.where(x >= 0.0, 1.0, -1.0)


def oct_project(v):
    """Direction -> octahedral (u, w) in [-1, 1]^2. Scale-invariant."""
    v = np.asarray(v, dtype=np.float64)
    l1 = np.sum(np.abs(v), axis=-1, keepdims=True)
    p = v / l1
    px, py, pz = p[..., 0], p[..., 1], p[..., 2]
    u = np.where(pz >= 0.0, px, (1.0 - np.abs(py)) * _sign_nonzero(px))
    w = np.where(pz >= 0.0, py, (1.0 - np.abs(px)) * _sign_nonzero(py))
    return np.stack([u, w], axis=-1)


def oct_unproject(uw):
    """Octahedral (u, w) -> unit direction."""
    uw = np.asarray(uw, dtype=np.float64)
    u, w = uw[..., 0], uw[..., 1]
    z = 1.0 - np.abs(u) - np.abs(w)
    x = np.where(z >= 0.0, u, (1.0 - np.abs(w)) * _sign_nonzero(u))
    y = np.where(z >= 0.0, w, (1.0 - np.abs(u)) * _sign_nonzero(w))
    v = np.stack([x, y, z], axis=-1)
    return v / np.linalg.norm(v, axis=-1, keepdims=True)


def quantize(f, bits):
    """[-1, 1] -> unsigned `bits`-wide field.

    Mid-tread with a 2^bits scale, NOT 2^bits - 1. The RTL encoder computes
    this as (F + 2^k) >> s on a fixed-point F, so a 2^bits scale is a pure
    shift while 2^bits - 1 would need a multiplier -- and the encoder is
    required to use no DSPs. The distinction is one LSB of scale and does not
    change the field-width conclusion (see the measurement below).
    """
    n = 1 << bits
    return np.clip(np.floor((np.asarray(f) * 0.5 + 0.5) * n), 0, n - 1).astype(np.int64)


def dequantize(q, bits):
    """Unsigned `bits`-wide field -> [-1, 1], reconstructing at the bin centre."""
    n = 1 << bits
    return (np.asarray(q, dtype=np.float64) + 0.5) / n * 2.0 - 1.0


def oct_encode(v, bits=OCT32_BITS):
    """Direction (any scale) -> (qu, qw) integer fields."""
    uw = oct_project(v)
    return quantize(uw[..., 0], bits), quantize(uw[..., 1], bits)


def oct_decode(qu, qw, bits=OCT32_BITS):
    """Integer fields -> unit direction."""
    return oct_unproject(np.stack([dequantize(qu, bits), dequantize(qw, bits)], axis=-1))


def oct_roundtrip(v, bits=OCT32_BITS):
    qu, qw = oct_encode(v, bits)
    return oct_decode(qu, qw, bits)


# ---------------------------------------------------------------------------
# GGX VNDF reference sampler (float, unquantized) -- Heitz 2018
# ---------------------------------------------------------------------------

def ggx_vndf_sample(view, alpha, u1, u2):
    """Sample visible normals of a GGX distribution. Returns unit half-vectors."""
    vh = np.stack([np.full_like(u1, alpha * view[0]),
                   np.full_like(u1, alpha * view[1]),
                   np.full_like(u1, view[2])], axis=-1)
    vh = vh / np.linalg.norm(vh, axis=-1, keepdims=True)

    lensq = vh[..., 0] ** 2 + vh[..., 1] ** 2
    t1 = np.where(
        lensq[..., None] > 1e-14,
        np.stack([-vh[..., 1], vh[..., 0], np.zeros_like(lensq)], axis=-1)
        / np.sqrt(np.maximum(lensq, 1e-30))[..., None],
        np.array([1.0, 0.0, 0.0]),
    )
    t2 = np.cross(vh, t1)

    r = np.sqrt(u1)
    phi = 2.0 * np.pi * u2
    p1 = r * np.cos(phi)
    p2 = r * np.sin(phi)
    s = 0.5 * (1.0 + vh[..., 2])
    p2 = (1.0 - s) * np.sqrt(np.maximum(0.0, 1.0 - p1 * p1)) + s * p2

    nh = (p1[..., None] * t1 + p2[..., None] * t2
          + np.sqrt(np.maximum(0.0, 1.0 - p1 * p1 - p2 * p2))[..., None] * vh)
    h = np.stack([alpha * nh[..., 0], alpha * nh[..., 1],
                  np.maximum(nh[..., 2], 0.0)], axis=-1)
    return h / np.linalg.norm(h, axis=-1, keepdims=True)


# ---------------------------------------------------------------------------
# Measurement
# ---------------------------------------------------------------------------

def angle_deg(a, b):
    d = np.sum(a * b, axis=-1)
    return np.degrees(np.arccos(np.clip(d, -1.0, 1.0)))


def _tv_hist(a, b, bins, rng_lim):
    ha, _ = np.histogram(a, bins=bins, range=rng_lim)
    hb, _ = np.histogram(b, bins=bins, range=rng_lim)
    ha = ha / max(ha.sum(), 1)
    hb = hb / max(hb.sum(), 1)
    return 0.5 * np.abs(ha - hb).sum()


def measure(n=200_000, seed=1234):
    rng = np.random.default_rng(seed)
    print("=" * 78)
    print("Oct32 field-width validation (issue #13, ADR-0001)")
    print("=" * 78)

    # --- 1. Round-trip + angular error over the whole sphere -------------
    v = rng.normal(size=(n, 3))
    v /= np.linalg.norm(v, axis=-1, keepdims=True)
    lower = v[:, 2] < 0.0
    print(f"\nRound-trip over {n} uniform directions "
          f"({lower.sum()} of them in the lower hemisphere, exercising the fold):")
    print(f"  {'bits':>5} {'mean':>10} {'median':>10} {'p99':>10} {'max':>10}   (degrees)")
    results = {}
    for bits in (8, 16):
        err = angle_deg(v, oct_roundtrip(v, bits))
        results[bits] = err
        print(f"  {bits:>5} {err.mean():>10.5f} {np.median(err):>10.5f} "
              f"{np.percentile(err, 99):>10.5f} {err.max():>10.5f}")
        for name, mask in (("upper", ~lower), ("lower", lower)):
            e = err[mask]
            print(f"        {name} hemisphere: mean {e.mean():.5f}  max {e.max():.5f}")

    # Scale invariance: the encoder must not care about input magnitude.
    scales = rng.uniform(1e-3, 1e3, size=(n, 1))
    same = np.array_equal(np.stack(oct_encode(v, 16), -1),
                          np.stack(oct_encode(v * scales, 16), -1))
    print(f"\nScale invariance oct(c*v) == oct(v) over {n} random c in [1e-3, 1e3]: "
          f"{'HOLDS' if same else 'FAILS'}")

    # --- 2. Distribution distortion at each roughness in the test set ----
    # Roughnesses and view directions are the ones test_ggx_control drives.
    cases = [(0.35, np.array([0.0, 0.0, 1.0])),
             (0.62, np.array([0.0, 0.0, 1.0])),
             (0.12, np.array([0.0, 0.0, 1.0])),
             (0.12, np.array([0.6, 0.0, 0.8]))]  # grazing, narrowest lobe
    print("\nDistribution distortion vs the unquantized reference, per roughness:")
    print(f"  {'alpha':>6} {'view.z':>7} {'lobe':>9} {'8b p99':>9} {'8b/lobe':>9} "
          f"{'16b p99':>9} {'16b/lobe':>9} {'TV 8b':>8} {'TV 16b':>8} {'TV ctl':>8}")
    verdict_ok = True
    for alpha, view in cases:
        view = view / np.linalg.norm(view)
        u1 = rng.random(n)
        u2 = rng.random(n)
        h = ggx_vndf_sample(view, alpha, u1, u2)
        # Lobe angular scale: median opening angle of the sampled half-vectors
        # about the macrosurface normal. This is what the error must be small
        # against -- an absolute error in degrees means nothing on its own.
        lobe = np.median(angle_deg(h, np.array([0.0, 0.0, 1.0])))

        row = [f"  {alpha:>6.2f} {view[2]:>7.2f} {lobe:>9.4f}"]
        tv = {}
        for bits in (8, 16):
            hq = oct_roundtrip(h, bits)
            err = angle_deg(h, hq)
            p99 = np.percentile(err, 99)
            row.append(f" {p99:>9.5f} {p99 / lobe:>9.5f}")
            tv[bits] = _tv_hist(angle_deg(h, np.array([0.0, 0.0, 1.0])),
                                angle_deg(hq, np.array([0.0, 0.0, 1.0])),
                                bins=200, rng_lim=(0.0, 90.0))
            if bits == 16 and p99 / lobe > 0.01:
                verdict_ok = False
        # Control: TV between two independent halves of the SAME unquantized
        # set, i.e. the sampling-noise floor this metric cannot see below.
        half = n // 2
        ctl = _tv_hist(angle_deg(h[:half], np.array([0.0, 0.0, 1.0])),
                       angle_deg(h[half:], np.array([0.0, 0.0, 1.0])),
                       bins=200, rng_lim=(0.0, 90.0))
        row.append(f" {tv[8]:>8.5f} {tv[16]:>8.5f} {ctl:>8.5f}")
        print("".join(row))

    # --- 3. Verdict on ADR-0001 ------------------------------------------
    e8, e16 = results[8], results[16]
    print("\n" + "-" * 78)
    print("ADR-0001 claims: 8-bit ~ 0.5-1 deg, 16-bit ~ 0.002 deg.")
    print(f"Measured (p99 over the sphere): 8-bit {np.percentile(e8, 99):.5f} deg, "
          f"16-bit {np.percentile(e16, 99):.5f} deg")
    c8 = 0.5 <= np.percentile(e8, 99) <= 1.0
    c16 = abs(np.percentile(e16, 99) - 0.002) < 0.002
    print(f"  8-bit  claim: {'CONFIRMED' if c8 else 'NOT as stated'}")
    print(f"  16-bit claim: {'CONFIRMED' if c16 else 'NOT as stated'}")
    print(f"  Oct32 usable (16-bit p99 error < 1% of the narrowest lobe): "
          f"{'YES' if verdict_ok else 'NO'}")
    print("-" * 78)
    return results


if __name__ == "__main__":
    measure()
