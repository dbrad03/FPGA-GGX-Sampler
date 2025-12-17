# FPGA Acceleration of Owen-Scrambled Sobol Sequences for GGX VNDF Sampling

## ⚡ Overview
This project presents a hardware-accelerated sampling pipeline for Physically Based Rendering (PBR). It generates high-quality "Blue Noise" samples using Owen-Scrambled Sobol sequences and maps them directly to the GGX microfacet distribution on-chip.

By offloading this heavy importance-sampling math to the FPGA, the architecture is designed to free up the main shading cores for ray traversal.

## 🏗️ Architecture
The system operates as a deep, feed-forward pipeline designed for streaming processing:
* **Stage 1 (Sequence):** Incremental Sobol generation using banked ROMs.
* **Stage 2 (Scramble):** Pipelined Laine-Karras integer hashing to decorrelate samples.
* **Stage 3 (VNDF Core):** Maps 2D points to 3D microfacet normals using "Spherical Cap" projection.

## 🔧 Technical Details
* **Platform:** Designed for PYNQ-Z2 (Zynq-7000) / UltraScale+ RFSOC
* **Interface:** AXI4-Stream (DMA-based)
* **Precision:** 32-bit Fixed-Point (Q1.31)
* **Math Kernels:**
    * Pipelined Newton-Raphson Inverse Square Root (Error < 1.5e-5).
    * Block RAM-based Trigonometric LUTs.

## 📈 Results
* **Quality:** Indistinguishable from floating-point software references; successfully eliminates grid aliasing found in raw Sobol sequences.
* **Verification:** Fully verified via cycle-accurate co-simulation (Cocotb) against a NumPy golden model.
* **Status:** RTL Logic Design & Behavioral Simulation complete.

## 📄 Reference
[**Read the Final Report (PDF)**](./6_S9S5_Final_Report.pdf)
