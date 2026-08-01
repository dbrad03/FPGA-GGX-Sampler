# FPGA GGX Sampler

Hardware generation of GGX VNDF (visible normal distribution function) samples on a Zynq-7020,
streaming microfacet half-vectors for a downstream ray tracer. This glossary fixes the vocabulary the
design and its tests are written in.

## Work units

**Command**:
One AXI-Stream request describing a view direction, a surface roughness, a seed, and how many samples
to produce.
_Avoid_: Job, request, packet

**Burst**:
The run of samples produced from a single Command. Terminated by TLAST.
_Avoid_: Batch, block

**Sample**:
One generated half-vector, the unit of output.
_Avoid_: Point, sample point, ray

**Lane**:
One complete instance of the sampling pipeline, producing one Sample per clock at full rate. The
design's scaling unit — throughput targets are expressed as a lane count times a clock frequency.
_Avoid_: Channel, pipe, core

## Pipeline roles

**Per-burst**:
Work computed once per Command and reused by every Sample in the Burst. Latency here is nearly free,
so per-burst work may be serialized or resource-shared.
_Avoid_: Per-command, setup, prologue

**Per-sample**:
Work performed for every Sample. Must sustain one result per clock; latency may grow but throughput
may not drop.
_Avoid_: Inner loop, hot path, streaming stage

**Event basis**:
The per-burst orthonormal frame derived from the warped view direction, in which each Sample's
half-vector is expressed.
_Avoid_: Tangent frame, local frame, TBN

**Fold**:
Converting fully-unrolled pipelined arithmetic into a sequential engine that reuses one small
datapath across iterations. Applicable only to per-burst work, and required to preserve results
exactly.
_Avoid_: Serialize, iterate, resource-share

**Oct32**:
The output encoding: a direction projected onto an octahedron and stored as two 16-bit values.
Scale-invariant, so it does not require its input to be unit length.
_Avoid_: Oct16, octahedral compression, packed normal

## Backpressure

**Ready chain**:
A `tready` that propagates backwards through several cores inside one clock cycle, because each core
derives its input ready combinationally from its output ready. Its cost is fanout and route, not
logic — each hop drives every clock enable in the core it crosses.
_Avoid_: Backpressure path, stall path, combinational loop

**Cut**:
A registered-ready elastic buffer placed in a Ready chain so that a stall stops at a flip-flop and
resumes from it next cycle. Costs one cycle of latency and holds two beats, so a ready that is one
cycle late is still lossless. See ADR-0003.
_Avoid_: Skid buffer (that is one specific primitive), pipeline register, break

## Verification

**Bias gate**:
The assertion that the mean *signed* error of the pipeline is below a threshold. Guards against
systematic error from fixed-point narrowing, which Monte Carlo integration does not average away.
The project's sharpest correctness instrument.
_Avoid_: Accuracy test, error check

**Accuracy tolerance**:
A loose bound on maximum absolute error. Structurally blind to systematic error and never evidence
of numerical margin.
_Avoid_: Tolerance (unqualified), error budget

**Stream-integrity gate**:
An end-to-end assertion that the Sample sequence is correct — right values, right order, right
Burst boundaries — computed independently of the design rather than from any signal inside it.
_Avoid_: Integration test, end-to-end test

**Skew**:
A data path and its accompanying valid/sideband path having different depths, so payload and
metadata are misaligned. Corrupts the Sample stream and leaks across Burst boundaries.
_Avoid_: Offset, characterised offset, latency mismatch

**Bit-identical**:
A change that provably leaves every output byte unchanged, verified by comparison against a stored
baseline. The standard of evidence for refactors that claim to be behaviour-preserving.
_Avoid_: Equivalent, no-op, safe refactor
