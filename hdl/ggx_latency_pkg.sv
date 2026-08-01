`timescale 1ns / 1ps
//=============================================================================
// ggx_latency_pkg -- the single source of the design's pipeline depths.
//
// See docs/adr/0002-latency-package-is-law.md. The rule is that PRODUCING
// modules derive their own depths from here too, not just consumers: a package
// only consumers read is documentation, and documentation is exactly what
// drifted (INVSQRT_LATENCY = 150, wrong and self-labelled "informational").
//
// Latencies are functions of each core's PARAMETERS, never literals, so that
// re-parameterizing a core updates every dependent delay line at elaboration.
//
// "Latency" here means: cycles from a beat being accepted on the slave port to
// that beat's result being presented on the master port. An exact-match
// sideband delay line must equal it; an elastic FIFO need only exceed it.
//
// DO NOT "simplify" this to consumers-only.
//
// It is tempting to look at axis_fixed_sqrt deriving STAGES from a function of
// its own LAT_STAGES, or axis_fixed_div deriving NST from its own WIDTH and
// FRAC_BITS, and conclude the round trip through this package is pointless --
// the core already knows its own depth, so why not let the package just
// describe it for consumers to read?
//
// Because that is precisely the arrangement that failed. A package only
// consumers read is documentation, and documentation drifted: INVSQRT_LATENCY
// sat at 150 (self-labelled "informational") while the real figure was 142 and
// the folded engine it named was a different engine entirely; projected_area's
// TRIG_LATENCY said 2 for a 3-stage block; reproject's comment put norm3 at
// "~165" when it is 151. Every one of those was written by someone who knew the
// right number at the time.
//
// When the producer derives from the package too, there is exactly one number
// and re-parameterizing a core moves its consumers with it. When only consumers
// derive, there are two numbers that merely happen to agree, and nothing fails
// when they stop. The elaboration checks in the modules are belt-and-braces on
// top of this, not a substitute for it.
//=============================================================================
package ggx_latency_pkg;

  //--------------------------------------------------------------------------
  // Digit-recurrence cores
  //--------------------------------------------------------------------------

  // axis_fixed_sqrt: LAT_STAGES recurrence stages (defaults to SIG_BITS) plus
  // an input register and an output register.
  function automatic int sqrt_latency(input int lat_stages);
    return lat_stages + 2;
  endfunction

  // axis_fixed_div: ITERS = WIDTH + FRAC_BITS division steps, each split into a
  // SUB and a SELECT stage (which is what took latency 59 -> 116), plus an
  // input and an output register.
  function automatic int div_latency(input int width, input int frac_bits);
    return 2 * (width + frac_bits) + 2;
  endfunction

  // axis_fixed_inv_sqrt_nodsp: sqrt feeding div directly, no glue register.
  function automatic int inv_sqrt_nodsp_latency(input int sqrt_sig_bits,
                                                input int div_width,
                                                input int div_frac_bits);
    return sqrt_latency(sqrt_sig_bits) + div_latency(div_width, div_frac_bits);
  endfunction

  //--------------------------------------------------------------------------
  // Sampler chain
  //--------------------------------------------------------------------------

  // axis_sobol2d_stateless: fixed 2-stage pipeline (valid_pipeline[1:0]).
  localparam int SOBOL_LATENCY = 2;

  // axis_hash_combine_2d: fixed 18-stage pipeline (valid_pipeline[17:0]).
  // The data path was grown to match after it was found running one register
  // short of its valid path -- see the skew note in docs/handoff.md.
  localparam int HASH_LATENCY = 18;

  // axis_nested_uniform_scramble: the sideband array is indexed 0..SIDEBAND_DEPTH,
  // so it holds SIDEBAND_DEPTH+1 stages and the data path must match exactly.
  // Perturbing this by +/-1 is precisely the historical Skew defect.
  function automatic int scramble_latency(input int sideband_depth);
    return sideband_depth + 1;
  endfunction

  // axis_pre_ggx_sampler aligns the sobol path to the hash path before the
  // scramble consumes both, so it delays the index by the difference.
  function automatic int sampler_index_align_delay();
    return HASH_LATENCY - SOBOL_LATENCY;
  endfunction

  // axis_trig_lut: three register stages (s1_valid -> s2_valid -> m00_tvalid).
  // NOTE 3, not the 2 that projected_area's dead TRIG_LATENCY claimed.
  localparam int TRIG_LUT_LATENCY = 3;

  //--------------------------------------------------------------------------
  // Composite blocks
  //--------------------------------------------------------------------------

  // axis_fixed_norm3 (FOLD_INVSQRT = 0) wraps the pipelined inverse-sqrt in
  // this many additional register stages (lensq, scaling, output rounding).
  localparam int NORM3_WRAPPER_STAGES = 9;

  // NOTE: norm3_pipelined_latency() was deleted along with its last reader when
  // the Oct32 encoder replaced the per-sample normalize (issue #16). norm3
  // itself is still live -- event_basis uses the FOLDED variant -- and
  // NORM3_WRAPPER_STAGES above is still checked inside axis_fixed_norm3.
  // An unread constant here is the exact thing this package exists to delete.

  // axis_oct32_encode: 4 stages in (magnitudes, L1 sum, shift amount, shift),
  // the divide, and 1 output register. Replaces norm3 on the per-sample path.
  function automatic int oct32_encode_latency(input int div_width, input int div_frac_bits);
    return 4 + div_latency(div_width, div_frac_bits) + 1;
  endfunction

  localparam int OCT32_DIV_WIDTH = 20;
  localparam int OCT32_DIV_FRAC  = 18;

  //--------------------------------------------------------------------------
  // Elastic buffers
  //--------------------------------------------------------------------------
  // An exact-match sideband delay line must EQUAL the latency it rides
  // alongside. An elastic FIFO only has to EXCEED it -- one entry per beat that
  // can be in flight, or its TLAST tracking overflows and emits a spurious
  // TLAST. Depths themselves stay an explicit design choice (power-of-two ring
  // buffers with headroom); the package supplies the bound they must clear.
  function automatic int elastic_min_depth(input int spanned_latency);
    return spanned_latency + 1;
  endfunction

  // axis_fifo_2deep, used throughout as a REGISTERED-READY cut in a
  // backpressure chain (see docs/adr/0003-ready-is-registered-at-core-inputs.md):
  // its s_axis_tready is a flip-flop output, so a downstream stall stops at the
  // FIFO instead of propagating combinationally into the block upstream of it.
  // Two facts about it are latency, so they live here:
  //   - a beat accepted on the slave port is presented on the master port one
  //     cycle later, so every cut costs exactly one cycle of latency;
  //   - it holds up to two beats, which is what lets a registered (one cycle
  //     late) ready still be lossless.
  localparam int FIFO_2DEEP_LATENCY = 1;
  localparam int FIFO_2DEEP_DEPTH   = 2;

  // Latency a sideband FIFO must span when one of these sits in FRONT of the
  // core it rides alongside: the beats inside the core, plus the beat in flight
  // through the cut, plus the beats parked in the cut. Feed the result to
  // elastic_min_depth(). Getting this wrong does not mis-pair anything by one --
  // it overflows the ring and emits a spurious TLAST.
  function automatic int fifo_2deep_span(input int core_latency);
    return core_latency + FIFO_2DEEP_LATENCY + FIFO_2DEEP_DEPTH;
  endfunction

  //--------------------------------------------------------------------------
  // Parameters the design instantiates its cores with today.
  //--------------------------------------------------------------------------
  localparam int SQRT_SIG_BITS   = 24;  // every axis_fixed_sqrt instance
  localparam int DIV_WIDTH       = 32;
  localparam int DIV_FRAC_BITS   = 25;
  localparam int SCRAMBLE_SIDEBAND_DEPTH = 18;

  // Concrete depths for the instantiations above, for modules that need a
  // constant rather than a call. Only add one here when a module reads it --
  // an unread constant in this package is the exact thing it exists to delete.
  localparam int SQRT_LATENCY_INST    = sqrt_latency(SQRT_SIG_BITS);                    // 26
  localparam int OCT32_ENCODE_LATENCY_INST =
      oct32_encode_latency(OCT32_DIV_WIDTH, OCT32_DIV_FRAC);                            // 82

endpackage
