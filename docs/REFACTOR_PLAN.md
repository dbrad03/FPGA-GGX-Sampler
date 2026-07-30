# FPGA GGX Sampler: Refactoring & BVH Integration Plan

This document outlines the step-by-step engineering plan to refactor the GGX VNDF Sampler pipeline, optimize its resource usage, scale it to a multi-lane architecture, port it to the Digilent Zybo Z7-20 board, and prepare it for integration with a hardware BVH ray tracing engine.

---

## 1. System Vision & Target Architecture

```mermaid
graph TD
    subgraph Host Memory (DDR3)
        SceneData[BVH & Triangle Database]
        CmdBuf[Command Buffer]
        OutBuf[Compressed Samples Buffer]
    end

    subgraph Zynq PS (Cortex-A9)
        Driver[Bare-Metal C DMA Driver]
    end

    subgraph Zynq PL (Programmable Logic @ 200 MHz)
        subgraph AXI Interface
            AXI_DMA[AXI DMA IP]
        end

        subgraph GGX Sampler (5-Lanes)
            Control[Shared FSM & Basis Gen]
            Lane0[Lane 0: quantized math]
            Lane1[Lane 1: quantized math]
            Lane2[Lane 2]
            Lane3[Lane 3]
            Lane4[Lane 4]
            OctComp[Octahedral Compression Oct16]
        end

        subgraph Ray Tracing Engine
            RayGen[Ray Generation]
            BVH_Trav[BVH Traversal Engine]
            Tri_Int[Möller-Trumbore Intersection]
            BRAM_Cache[BVH Node Cache (BRAM)]
        end
    end

    %% Connections
    Driver -->|Configures| AXI_DMA
    CmdBuf -->|AXI MM2S| AXI_DMA
    AXI_DMA -->|AXI-Stream| Control
    Control --> Lane0 & Lane1 & Lane2 & Lane3 & Lane4
    Lane0 & Lane1 & Lane2 & Lane3 & Lane4 --> OctComp
    OctComp -->|16-bit compressed| AXI_DMA
    AXI_DMA -->|AXI S2MM| OutBuf

    SceneData <-->|AXI HP Ports| BVH_Trav
    BVH_Trav <--> BRAM_Cache
    BVH_Trav --> Tri_Int
    OctComp -->|Direct stream| RayGen
    RayGen --> BVH_Trav
```

---

## 2. Phase 1: Single-Lane Optimization & DSP Reduction

Before scaling to multiple lanes, we must reduce the DSP footprint of a single sample lane from **104 DSPs** to **< 15 DSPs**. We will do this by lowering the fixed-point precision and replacing multiplier-heavy IP with adder-based alternatives.

### Step 1.1: Precision Quantization (Q1.31 → Q1.23 / UQ0.24)
*   **Action**: Change the datapath from 32-bit fixed point to 24-bit fixed point.
*   **Rationale**: A 32-bit multiplication takes 4 DSP slices. A 24-bit multiplication ($24 \le 25, 24 > 18$) takes only **2 DSP slices**, immediately halving math footprint.
*   **Verification**: Run `sim/test_ggx_control.py` to ensure sampling errors remain below the tolerance threshold ($6.5 \times 10^{-2}$).

### Step 1.2: Digit-Recurrence division & square root (0 DSP, 0 BRAM)
*   **Action**: Replace the BRAM-LUT and Newton-Raphson-based `axis_fixed_inv_sqrt` and `axis_fixed_sqrt` with pipelined **digit-by-digit (non-restoring) square root and division** cores.
*   **Rationale**: Digit-recurrence algorithms use a binary-search-like remainder subtraction scheme that requires **only shifts and additions/subtractions**. This drops BRAM and DSP usage for square roots/divisions to zero, trading them for a small number of slice LUTs.
*   **Verification**: Run `test_fixed_sqrt.py` and `test_fixed_inv_sqrt.py`.

### Step 1.3: CORDIC Conversion (0 DSP)
*   **Action**: Replace the trigonometric LUT (`axis_trig_lut`) and 3D vector normalizer (`axis_fixed_norm3`) with pipelined **CORDIC (Coordinate Rotation Digital Computer)** engines.
*   **Rationale**: CORDIC computes vector rotations, sines, cosines, and magnitudes using iterative shift-and-add operations. This eliminates the remaining DSP multipliers used in trig lookups and vector normalization.
*   **Verification**: Run `test_fixed_norm3.py` and `test_trig_lut.py`.

---

## 3. Phase 2: Octahedral Compression (Oct16)

Currently, the output is a 96-bit vector padded to 128-bit per sample. At 1 GHz streaming, this requires **16 GB/s** of memory bandwidth, which is impossible on the Zynq-7020's 4.26 GB/s DDR3 bus.

*   **Action**: Build an octahedral compression module (`axis_octahedral_encode.sv`) that projects the 3D unit vector onto a 2D octahedral surface, producing two 8-bit signed values (16-bit total, Oct16).
*   **Rationale**: Reduces memory bandwidth from 16 GB/s to **2 GB/s**, enabling the Zynq DDR controller to stream samples to DDR3 without choking.
*   **Math**:
    $$p = \frac{v}{\|v\|_1}$$
    If $z \ge 0$, then $(u,v) = (p_x, p_y)$.
    If $z < 0$, then $(u,v) = \text{sign}(p_{x,y}) \cdot (1 - |p_{y,x}|)$.
*   **Verification**: Write a Python verification model in `sim/` to encode/decode vectors and check for spherical reconstruction errors.

---

## 4. Phase 3: Multi-Lane Expansion

Once the single-lane is optimized and compressed, we scale to **5 parallel lanes running at 200 MHz** to achieve a **1.0 GHz streaming rate**.

*   **Step 3.1**: Widen the top-level FSM index counter to increment by 5.
*   **Step 3.2**: Instantiate 5 parallelized sample generator lanes.
*   **Step 3.3**: Combine outputs into a single wide bus, or run them into a tightly packed FIFO stream.

---

## 5. Phase 4: Porting to the Zybo Z7-20

*   **Step 4.1**: Create a Vivado project targeting the `xc7z020clg400-1` part and load the Zybo Z7-20 board presets (to set up the correct PS DDR3 timings and CPU peripherals).
*   **Step 4.2**: Set up the AXI DMA block design, connecting the PL fabric to the high-performance (AXI HP) ports of the Zynq PS.
*   **Step 4.3**: Port the host software. Write a bare-metal C application in Vitis using the Xilinx standalone DMA driver (`xaxidma.h`) to initiate transmissions, trigger interrupts, and benchmark throughput.

---

## 6. Phase 5: BVH Ray Tracing Engine Integration

To perform real-time rendering, the generated samples will directly feed a **hardware BVH traversal and intersection engine** on the PL.

```
       [ GGX Sampler ]
              │ (Oct16)
              ▼
    [ Ray Generation Core ] ──(Ray)──► [ BVH Traversal Engine ]
                                               │         ▲
                                    (DDR3 / AXI HP)   (BRAM Cache)
                                               ▼         │
                                      [ Node/Tri Fetch Core ]
                                               │
                                               ▼
                                  [ Möller-Trumbore Core ]
                                               │
                                               ▼
                                      [ Frame Buffer ]
```

### Traversal Pipeline
1.  **Scene Database**: Store the scene BVH tree and triangle vertices in DDR3 memory.
2.  **PL Traversal FSM**: Traverse the BVH tree by performing ray-box intersection tests on bounding box nodes.
3.  **BRAM Cache**: Use the remaining BRAM blocks (~500 KB) as a read-only cache for the top levels of the BVH tree to reduce DDR3 latency.
4.  **Intersection pipeline**: Once a leaf node is hit, stream the triangle data into a pipelined Möller-Trumbore ray-triangle intersection core.
5.  **Shading**: The hit distance and normals will be fed into a shading block to compute local reflection using the GGX VNDF sample.

---

## 7. Resource & Bandwidth Risk Assessment

| Risk | Impact | Mitigation Strategy |
| :--- | :--- | :--- |
| **Timing Closure @ 200 MHz** | High | Insert register slices at module boundaries and use carry-save adders. |
| **DDR3 Bandwidth Contention** | Medium | The BVH engine and GGX Sampler will contend for the DDR3 bus. Use Oct16 compression for samples and keep BVH traversal cached in BRAM. |
| **BRAM capacity for BVH** | Low | Store only the top 3-4 levels of the BVH tree in BRAM; keep the rest in DDR3. |
| **Math Precision loss** | Medium | High-precision operations (like basis calculations) will stay 32-bit since they run once per burst, while the high-frequency per-sample path uses 24-bit/16-bit. |

---

## Progress notes (as of 2026-07-26)

- **Step 1.1 (quantization)** — captured a stronger way than planned: multiply *operands* truncated to 18b×25b so each product fits **one** DSP (better than the plan's 2-DSP/24b target). Operands are round-half-up (removes a measured systematic bias) and registered so no combinational logic sits between a register and a DSP. The full-*datapath* narrowing (values/products between ops) is still open (see Phase C in the working plan).
- **Step 1.2 (digit-recurrence sqrt/div)** — `axis_fixed_sqrt` rewritten to non-restoring digit recurrence (0 DSP/0 BRAM); a no-DSP inverse-sqrt (`axis_fixed_inv_sqrt_nodsp`) wraps sqrt+div.
- **Step 1.3 (CORDIC)** — in progress: replacing `axis_fixed_norm3` and `event_basis`'s inverse-sqrt with a dividerless CORDIC normalize.
- **Timing**: real post-route (full P&R, not OOC synth) Fmax ~155 MHz; single-lane DSP 85. Remaining 200 MHz gap dominated by the iterative sqrt/div blocks and routing on the wide products.
