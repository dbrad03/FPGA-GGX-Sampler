"""
GGX VNDF Sampler — Pipeline Block Diagram
Run:  python block_diagram.py
Out:  block_diagram.png
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch

# ── Palette ────────────────────────────────────────────────────────────────
C_BG      = "#F4F6F7"
C_DMA     = "#5B2C8D"
C_CTRL    = "#1A4F72"
C_BASIS   = "#784212"
C_SAMPLER = "#1D6A39"
C_PROJ    = "#0A5744"
C_REPR    = "#7B241C"
C_PRIM    = "#3D3D50"
WHITE     = "#FFFFFF"
DARK      = "#1C2833"
MID       = "#4A4A5A"

# Tinted fills for group boxes (very light tint so dark text reads well)
C_SAMPLER_FILL = "#D5F5E3"
C_PROJ_FILL    = "#D0EDE7"
C_REPR_FILL    = "#FADBD8"

A_MAIN  = "#17202A"
A_BURST = "#1A5276"
A_DMA   = "#5B2C8D"

# ── Canvas ─────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(22, 12))
fig.patch.set_facecolor(C_BG)
ax.set_facecolor(C_BG)
ax.set_xlim(0, 22)
ax.set_ylim(0, 12)
ax.axis("off")

# ── Helpers ────────────────────────────────────────────────────────────────
def box(ax, x, y, w, h, fc, ec=DARK, lw=1.4, zorder=3, alpha=1.0):
    ax.add_patch(FancyBboxPatch((x, y), w, h,
        boxstyle="round,pad=0.1", linewidth=lw,
        edgecolor=ec, facecolor=fc, alpha=alpha, zorder=zorder))

def txt(ax, x, y, s, size=9, color=WHITE, weight="bold",
        ha="center", va="center", style="normal", zorder=4):
    ax.text(x, y, s, ha=ha, va=va, fontsize=size, fontweight=weight,
            color=color, zorder=zorder, style=style, linespacing=1.4)

def group(ax, x, y, w, h, title, ec, fc, lw=1.6):
    """Dashed group box with tinted fill so text is readable."""
    ax.add_patch(FancyBboxPatch((x, y), w, h,
        boxstyle="round,pad=0.14", linewidth=lw,
        edgecolor=ec, facecolor=fc, linestyle="--", zorder=2))
    ax.text(x+0.18, y+h, f"  {title}  ", ha="left", va="top",
            fontsize=8.5, fontweight="bold", color=ec, zorder=5,
            bbox=dict(boxstyle="round,pad=0.18", facecolor=C_BG,
                      edgecolor=ec, lw=0.9))

def pill(ax, x, y, w, h, label, sub="", fc=C_PRIM, ec="#888"):
    box(ax, x, y, w, h, fc, ec=ec, lw=0.9, zorder=4)
    cy = y + h*0.65 if sub else y + h/2
    txt(ax, x+w/2, cy, label, size=7, color=WHITE, zorder=5)
    if sub:
        txt(ax, x+w/2, y+h*0.27, sub, size=6, color="#CCCCCC",
            weight="normal", style="italic", zorder=5)

def arr(ax, x0, y0, x1, y1, color=A_MAIN, lw=2.2, lbl="",
        ldx=0, ldy=0.22, head=15, cs="arc3,rad=0"):
    ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
        arrowprops=dict(arrowstyle="-|>", color=color, lw=lw,
                        mutation_scale=head, connectionstyle=cs), zorder=6)
    if lbl:
        ax.text((x0+x1)/2+ldx, (y0+y1)/2+ldy, lbl,
                ha="center", va="center", fontsize=7.5, color=color,
                fontweight="bold", zorder=7,
                bbox=dict(boxstyle="round,pad=0.22", facecolor=C_BG,
                          edgecolor=color, lw=0.9, alpha=0.95))

def divider(ax, y, label="", color="#BDC3C7"):
    ax.plot([0.5, 21.5], [y, y], color=color, lw=1.0, linestyle=":")
    if label:
        ax.text(0.55, y-0.06, label, fontsize=7.8, color=MID,
                fontweight="bold", va="top")

# ══════════════════════════════════════════════════════════════════════════════
# TITLE
# ══════════════════════════════════════════════════════════════════════════════
txt(ax, 11, 11.65, "FPGA GGX VNDF Sampler — RTL Pipeline Architecture",
    size=15, color=DARK)
txt(ax, 11, 11.25,
    "Zynq-7020  ·  100 MHz  ·  1 GGX VNDF sample per clock cycle  ·  "
    "WNS = +0.060 ns  ·  0 failing timing endpoints",
    size=9, color=MID, weight="normal")
ax.plot([0.5, 21.5], [11.0, 11.0], color="#BDC3C7", lw=1.0)

# ══════════════════════════════════════════════════════════════════════════════
# ZONE A  y = 8.85 .. 10.85   "Once per burst"
# ══════════════════════════════════════════════════════════════════════════════
divider(ax, 11.0)
txt(ax, 1.8, 10.88, "ONCE PER BURST", size=8, color=A_BURST,
    weight="bold", ha="center", va="bottom")

# A1 — AXI DMA MM2S  (beat format lives inside the box)
box(ax, 0.4, 8.9, 2.2, 1.85, C_DMA)
txt(ax, 1.5, 10.35, "AXI DMA MM2S", size=9)
txt(ax, 1.5, 10.05, "Zynq PS → PL", size=7.5, weight="normal", style="italic")
ax.plot([0.55, 2.45], [9.88, 9.88], color="#FFFFFF55", lw=0.7)
txt(ax, 1.5, 9.67, "Beat 0:  {seed_base, burst_len}", size=6.5, weight="normal")
txt(ax, 1.5, 9.38, "Beat 1:  {view_y,    view_x   }", size=6.5, weight="normal")
txt(ax, 1.5, 9.09, "Beat 2:  {alpha,     view_z   }", size=6.5, weight="normal")

# A2 — axis_ggx_control
box(ax, 2.9, 8.9, 3.0, 1.85, C_CTRL)
txt(ax, 4.4, 10.30, "axis_ggx_control", size=9)
txt(ax, 4.4, 9.90,  "Command FSM", size=8, weight="normal")
txt(ax, 4.4, 9.48,
    "WAIT_B0 → WAIT_B1 → WAIT_B2\n"
    "→ BASIS_REQ → WAIT_BASIS\n"
    "→ SAMPLER_CMD → RUN_BURST",
    size=7, weight="normal", style="italic")

arr(ax, 2.6, 9.82, 2.9, 9.82, color=A_DMA, lw=2.0, lbl="S_AXIS\n64b")

# A3 — axis_ggx_event_basis
box(ax, 6.2, 8.9, 6.0, 1.85, C_BASIS)
txt(ax, 9.2, 10.30, "axis_ggx_event_basis", size=9)
txt(ax, 9.2, 9.90,
    "Stretch:  Vhx = α·vx,  Vhy = α·vy,  Vhz = vz", size=8, weight="normal")
txt(ax, 9.2, 9.52,
    "Normalize Vh  →  build ONB:  T1 ⊥ Vh,   T2 = Vh × T1", size=8, weight="normal")
pill(ax, 6.35, 8.98, 2.2, 0.72, "axis_fixed_inv_sqrt", "3 instances · 9-stage", ec="#F0B27A")
pill(ax, 8.7,  8.98, 2.0, 0.72, "axis_fixed_norm3",    "10-stage pipeline",      ec="#F0B27A")
pill(ax, 10.85,8.98, 1.2, 0.72, "axis_skid\n_buffer",                             ec="#F0B27A")

arr(ax, 5.9, 9.82, 6.2, 9.82, color=A_BURST, lw=2.0,
    lbl="{ α, Vz, Vy, Vx }\n128b  Q1.31+UQ0.32")

# A4 — Post-implementation results (right of event_basis, same row)
rx, ry, rw, rh = 12.6, 8.9, 4.15, 1.85
box(ax, rx, ry, rw, rh, WHITE, ec="#5D6D7E", lw=1.2)
txt(ax, rx+rw/2, ry+rh-0.28, "Post-Implementation  (Zynq-7020)",
    size=8.5, color=DARK)
ax.plot([rx+0.18, rx+rw-0.18], [ry+rh-0.52, ry+rh-0.52], color="#CCC", lw=0.7)

res_rows = [
    ("Slice LUTs  ", "23,373 / 53,200", "44%"),
    ("Registers   ", "24,052 / 106,400","23%"),
    ("DSP48E1     ", "   196 / 220",    "89%"),
    ("BRAM Tiles  ", "  40.5 / 140",    "29%"),
]
for j, (k, v, pct) in enumerate(res_rows):
    yy = ry+rh-0.82-j*0.27
    txt(ax, rx+0.22, yy, k, size=7.5, color=DARK, ha="left",
        va="center", weight="normal")
    txt(ax, rx+rw-0.55, yy, v, size=7.5, color=DARK, ha="right",
        va="center", weight="normal", style="normal")
    txt(ax, rx+rw-0.1, yy, pct, size=7.5, color=C_CTRL, ha="right", va="center")

ax.plot([rx+0.18, rx+rw-0.18], [ry+0.62, ry+0.62], color="#CCC", lw=0.7)
txt(ax, rx+rw/2, ry+0.44, "100 MHz  ·  WNS = +0.060 ns ✓", size=8, color=C_PROJ)
txt(ax, rx+rw/2, ry+0.20, "100 M samples/sec  ·  ~5,000× vs Python",
    size=7.5, color=DARK, weight="normal")

# A5 — AXI DMA S2MM  (top right)
box(ax, 17.15, 8.9, 2.2, 1.85, C_DMA)
txt(ax, 18.25, 10.30, "AXI DMA S2MM", size=9)
txt(ax, 18.25, 9.93, "PL → Zynq PS", size=8, weight="normal", style="italic")
txt(ax, 18.25, 9.55, "128-bit TDATA", size=8, weight="normal")
txt(ax, 18.25, 9.18, "{0, hz, hy, hx}", size=8, weight="normal")

# ══════════════════════════════════════════════════════════════════════════════
# ZONE B  y = 4.4 .. 8.65   "Per-sample pipeline"
# ══════════════════════════════════════════════════════════════════════════════
divider(ax, 8.65, label="PER SAMPLE  (every clock cycle)")

# B1 — axis_pre_ggx_sampler
PS_X, PS_Y, PS_W, PS_H = 0.4, 4.5, 5.6, 3.9
group(ax, PS_X, PS_Y, PS_W, PS_H,
      "axis_pre_ggx_sampler", C_SAMPLER, C_SAMPLER_FILL)

# delay shim banner
box(ax, PS_X+0.18, PS_Y+3.28, PS_W-0.36, 0.38, "#F9E79F", ec="#D4AC0D", lw=0.9, zorder=4)
txt(ax, PS_X+PS_W/2, PS_Y+3.48,
    "8-cycle delay shim  (hash pipeline = 10 cy,  sobol = 2 cy  →  Δ = 8)",
    size=7.2, color="#7D6608", zorder=5)

# Dim labels
txt(ax, PS_X+PS_W/2, PS_Y+2.92, "Dimension 0  →  u₁",
    size=8, color=C_SAMPLER, weight="bold", zorder=4)
txt(ax, PS_X+PS_W/2, PS_Y+1.87, "Dimension 1  →  u₂",
    size=8, color=C_SAMPLER, weight="bold", zorder=4)

for row, (label_suf, dim, bot_y) in enumerate([("0", "0", PS_Y+2.15), ("1", "1", PS_Y+1.12)]):
    for i, (n, d) in enumerate([
        (f"hash_combine\n(dim {dim})", f"seed_d{dim} = H(seed, {dim})"),
        (f"sobol2d_stateless\n(dim {dim})", "gray(i) ⊕ vᵢ"),
        (f"nested_uniform\n_scramble", "rev(LK(rev(x), s))"),
    ]):
        bx = PS_X+0.22 + i*1.76
        pill(ax, bx, bot_y, 1.64, 0.68, n, d, fc=C_SAMPLER, ec="#1E8449")
        if i < 2:
            arr(ax, bx+1.64, bot_y+0.34, bx+1.76, bot_y+0.34,
                color=C_SAMPLER, lw=1.2, head=8)

# once-per-burst cmd arrow: control → sampler
arr(ax, 4.4, 8.9, 4.4, 8.65, color=A_BURST, lw=1.8, head=11)
arr(ax, 4.4, 8.65, PS_X+PS_W/2, 8.65, color=A_BURST, lw=1.8, head=11)
arr(ax, PS_X+PS_W/2, 8.65, PS_X+PS_W/2, PS_Y+PS_H+0.05,
    color=A_BURST, lw=1.8, head=11,
    lbl="{ burst_len, seed_base }  64b\n(once / burst)", ldy=0, ldx=1.3)

# B2 — axis_ggx_projected_area
PA_X, PA_Y, PA_W, PA_H = 6.3, 4.5, 5.6, 3.9
group(ax, PA_X, PA_Y, PA_W, PA_H,
      "axis_ggx_projected_area", C_PROJ, C_PROJ_FILL)

txt(ax, PA_X+PA_W/2, PA_Y+3.47,
    "Cap height:   z = (1 − u₁)(1 + Vhz) − Vhz", size=8.5, color=C_PROJ)
txt(ax, PA_X+PA_W/2, PA_Y+3.05,
    "Disk radius:  r = √(1 − z²)", size=8.5, color=C_PROJ)
txt(ax, PA_X+PA_W/2, PA_Y+2.63,
    "Azimuth:      φ = 2π · u₂", size=8.5, color=C_PROJ)
txt(ax, PA_X+PA_W/2, PA_Y+2.21,
    "Output:       t₁ = r·cos φ,   t₂ = r·sin φ", size=8.5, color=C_PROJ)
pill(ax, PA_X+0.2,  PA_Y+1.35, 1.65, 0.68,
     "axis_fixed_sqrt",  "r = √(1−z²)", fc=C_PRIM, ec="#5DADE2")
pill(ax, PA_X+2.05, PA_Y+1.35, 1.65, 0.68,
     "axis_trig_lut",    "sin/cos · 1K BRAM", fc=C_PRIM, ec="#5DADE2")
pill(ax, PA_X+3.9,  PA_Y+1.35, 1.52, 0.68,
     "BRAM meta2_vhz", "blend factor LUT", fc=C_PRIM, ec="#5DADE2")
pill(ax, PA_X+0.2,  PA_Y+0.5,  5.22, 0.68,
     "axis_skid_buffer  (backpressure isolation)", fc=C_PRIM, ec="#5DADE2")

# sampler → projected_area
arr(ax, PS_X+PS_W+0.05, PS_Y+1.8, PA_X, PS_Y+1.8,
    color=A_MAIN, lw=2.8, lbl="{ u₂, u₁ }  ·  64b  UQ0.32\nevery cycle", ldy=0.3, head=17)

# event_basis → projected_area: Vh_z (down then across)
arr(ax, 9.2, 8.9, 9.2, 8.65, color=A_BURST, lw=1.8, head=11)
arr(ax, 9.2, 8.65, PA_X+PA_W/2, 8.65, color=A_BURST, lw=1.8, head=11,
    lbl="Vhz (Q1.31)  latched once/burst", ldy=0.22)
arr(ax, PA_X+PA_W/2, 8.65, PA_X+PA_W/2, PA_Y+PA_H+0.05,
    color=A_BURST, lw=1.8, head=11)

# B3 — axis_ggx_reproject_normalize
RN_X, RN_Y, RN_W, RN_H = 12.2, 4.5, 5.6, 3.9
group(ax, RN_X, RN_Y, RN_W, RN_H,
      "axis_ggx_reproject_normalize", C_REPR, C_REPR_FILL)

txt(ax, RN_X+RN_W/2, RN_Y+3.47,
    "Reconstruct:  ωₕ_s = t₁·T1 + t₂·T2 + z·Vh", size=8.5, color=C_REPR)
txt(ax, RN_X+RN_W/2, RN_Y+3.02,
    "(dot products in stretched space, 352b input bus)", size=7.5, color=C_REPR, style="italic")
txt(ax, RN_X+RN_W/2, RN_Y+2.58,
    "Un-stretch:   ωₕ_x /= α,   ωₕ_y /= α", size=8.5, color=C_REPR)
txt(ax, RN_X+RN_W/2, RN_Y+2.16,
    "Normalize:    ωₕ  =  ωₕ / ‖ωₕ‖   →   unit half-vector", size=8.5, color=C_REPR)
pill(ax, RN_X+0.2,  RN_Y+1.35, 2.55, 0.68,
     "axis_fixed_norm3", "10-stage · Q1.31", fc=C_PRIM, ec="#E74C3C")
pill(ax, RN_X+2.92, RN_Y+1.35, 2.55, 0.68,
     "Q1.31 saturated arith", "2-bit overflow check", fc=C_PRIM, ec="#E74C3C")
pill(ax, RN_X+0.2,  RN_Y+0.5,  5.22, 0.68,
     "axis_skid_buffer  (output isolation)", fc=C_PRIM, ec="#E74C3C")

# projected_area → reproject_normalize
arr(ax, PA_X+PA_W+0.05, RN_Y+1.8, RN_X, RN_Y+1.8,
    color=A_MAIN, lw=2.8, lbl="{ t₂, t₁ }  ·  64b  Q1.31", ldy=0.28, head=17)

# event_basis → reproject_normalize: Vh, T1, T2
arr(ax, 12.0, 8.9, 12.0, 8.65, color=A_BURST, lw=1.8, head=11)
arr(ax, 12.0, 8.65, RN_X+RN_W/2, 8.65, color=A_BURST, lw=1.8, head=11,
    lbl="Vh, T1, T2  ·  3×96b Q1.31  (latched once/burst)", ldy=0.22)
arr(ax, RN_X+RN_W/2, 8.65, RN_X+RN_W/2, RN_Y+RN_H+0.05,
    color=A_BURST, lw=1.8, head=11)

# reproject_normalize → AXI DMA S2MM  (right then up)
arr(ax, RN_X+RN_W+0.05, RN_Y+2.6, 19.6, RN_Y+2.6,
    color=A_MAIN, lw=2.8,
    lbl="{ 0, hz, hy, hx }  ·  128b  Q1.31 + pad", ldy=0.3, head=17)
arr(ax, 19.6, RN_Y+2.6, 19.6, 8.9,
    color=A_MAIN, lw=2.0, head=13)

# ══════════════════════════════════════════════════════════════════════════════
# ZONE C  y = 0.5 .. 4.15   "Shared Math Primitives"
# ══════════════════════════════════════════════════════════════════════════════
divider(ax, 4.3, label="SHARED MATH PRIMITIVES  (instantiated inside sub-modules above)")

prim_specs = [
    ("axis_fixed_inv_sqrt",
     [("Algorithm",  "Newton-Raphson  1/√x"),
      ("Iteration",  "y₁ = y₀ · (1.5 − 8·x·y₀²)"),
      ("Pipeline",   "9 stages"),
      ("ROM",        "16,384-entry BRAM (Q7.25)"),
      ("Scaling",    "output = 0.25 · x⁻⁰·⁵"),
      ("Instances",  "3 in full design")]),
    ("axis_fixed_norm3",
     [("Purpose",    "normalize 3D vector  ‖v‖→1"),
      ("Method",     "||v||⁻¹ via inv_sqrt"),
      ("Then",       "v · ||v||⁻¹  (3 multiplies)"),
      ("Pipeline",   "10 stages"),
      ("Format",     "Q1.31 in & out"),
      ("Used by",    "event_basis, reproject_norm")]),
    ("axis_trig_lut",
     [("Purpose",    "sin / cos lookup"),
      ("Address",    "10-bit  u₂[31:22]"),
      ("Entries",    "1,024 sin+cos pairs"),
      ("Storage",    "dual-port BRAM"),
      ("Precision",  "Q1.31"),
      ("Used by",    "projected_area")]),
    ("axis_fixed_sqrt",
     [("Purpose",    "pipelined square root"),
      ("Method",     "wraps axis_fixed_inv_sqrt"),
      ("Computes",   "√(1 − z²)  (disk radius)"),
      ("Used by",    "projected_area"),
      ("",           ""),
      ("",           "")]),
    ("axis_skid_buffer",
     [("Purpose",    "1-entry elastic FIFO"),
      ("Function",   "decouples valid/ready"),
      ("Prevents",   "combinatorial ready loops"),
      ("Used at",    "all module boundaries"),
      ("",           ""),
      ("",           "")]),
]

N = len(prim_specs)
pw, gap = 4.0, 0.3
total = N*pw + (N-1)*gap
px0 = (22 - total) / 2
PH = 3.55

for i, (name, rows) in enumerate(prim_specs):
    px = px0 + i*(pw+gap)
    box(ax, px, 0.5, pw, PH, C_PRIM, ec="#666", lw=1.0)
    txt(ax, px+pw/2, 0.5+PH-0.29, name, size=8.5, color=WHITE)
    ax.plot([px+0.2, px+pw-0.2],
            [0.5+PH-0.54, 0.5+PH-0.54], color="#666", lw=0.7)
    for j, (k, v) in enumerate(rows):
        yy = 0.5+PH-0.82 - j*0.47
        if k:
            txt(ax, px+0.22, yy, k+":", size=7, color="#AAAACC",
                ha="left", va="center", weight="bold")
            txt(ax, px+pw-0.12, yy, v, size=7, color="#DDDDEE",
                ha="right", va="center", weight="normal")

# dotted usage lines from Zone B modules to Zone C primitives
usage_lines = [
    # (from_x, from_module_bottom_y, prim_index)
    (7.2,  PA_Y, 0),   # projected_area → inv_sqrt (via sqrt)
    (8.8,  PA_Y, 2),   # projected_area → trig_lut
    (9.5,  PA_Y, 3),   # projected_area → fixed_sqrt
    (6.0,  PS_Y, 4),   # sampler skid → skid_buffer (implicit)
    (13.5, RN_Y, 1),   # reproject → norm3
    (14.8, RN_Y, 4),   # reproject → skid_buffer
    (2.5,  RN_Y, 0),   # event_basis → inv_sqrt (from far left — skip, too messy)
]
for (fx, fy, pidx) in usage_lines[:6]:
    tx = px0 + pidx*(pw+gap) + pw/2
    ty = 0.5 + PH
    # vertical stub down from module, then line to prim
    ax.annotate("", xy=(tx, ty), xytext=(fx, fy),
                arrowprops=dict(arrowstyle="-", color="#AAAAAA", lw=0.7,
                                linestyle=":", connectionstyle="arc3,rad=0"),
                zorder=1)

# ══════════════════════════════════════════════════════════════════════════════
# Legend
# ══════════════════════════════════════════════════════════════════════════════
legend_items = [
    mpatches.Patch(color=C_DMA,     label="AXI DMA / Zynq PS"),
    mpatches.Patch(color=C_CTRL,    label="axis_ggx_control (FSM)"),
    mpatches.Patch(color=C_BASIS,   label="axis_ggx_event_basis"),
    mpatches.Patch(color=C_SAMPLER, label="axis_pre_ggx_sampler"),
    mpatches.Patch(color=C_PROJ,    label="axis_ggx_projected_area"),
    mpatches.Patch(color=C_REPR,    label="axis_ggx_reproject_normalize"),
    mpatches.Patch(color=C_PRIM,    label="Shared Math Primitives"),
    mpatches.Patch(color=A_MAIN,    label="Per-sample flow (every cycle)"),
    mpatches.Patch(color=A_BURST,   label="Once-per-burst signals"),
]
ax.legend(handles=legend_items,
          loc="lower center", bbox_to_anchor=(0.5, -0.01),
          fontsize=7.5, framealpha=0.97,
          facecolor=WHITE, edgecolor="#AAAAAA",
          labelcolor=DARK, ncol=5,
          handlelength=1.2, handleheight=0.95)

plt.tight_layout(pad=0.2)
plt.savefig("block_diagram.png", dpi=160, bbox_inches="tight", facecolor=C_BG)
print("Saved block_diagram.png")
