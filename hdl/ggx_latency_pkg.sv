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
// NOTE: as of this commit NOTHING consumes this package -- that is deliberate
// (the expand step of an expand/migrate/contract). The values below reproduce
// the numbers hardcoded across the design today; the migration tickets replace
// those literals with these calls.
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

  function automatic int norm3_pipelined_latency(input int sqrt_sig_bits,
                                                 input int div_width,
                                                 input int div_frac_bits);
    return inv_sqrt_nodsp_latency(sqrt_sig_bits, div_width, div_frac_bits)
           + NORM3_WRAPPER_STAGES;
  endfunction

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

  //--------------------------------------------------------------------------
  // Parameters the design instantiates its cores with today.
  //--------------------------------------------------------------------------
  localparam int SQRT_SIG_BITS   = 24;  // every axis_fixed_sqrt instance
  localparam int DIV_WIDTH       = 32;
  localparam int DIV_FRAC_BITS   = 25;
  localparam int SCRAMBLE_SIDEBAND_DEPTH = 18;

  // Concrete depths for the instantiations above, for modules that need a
  // constant rather than a call.
  localparam int SQRT_LATENCY_INST    = sqrt_latency(SQRT_SIG_BITS);                    // 26
  localparam int DIV_LATENCY_INST     = div_latency(DIV_WIDTH, DIV_FRAC_BITS);          // 116
  localparam int INV_SQRT_NODSP_LATENCY_INST =
      inv_sqrt_nodsp_latency(SQRT_SIG_BITS, DIV_WIDTH, DIV_FRAC_BITS);                  // 142
  localparam int SCRAMBLE_LATENCY_INST = scramble_latency(SCRAMBLE_SIDEBAND_DEPTH);     // 19
  localparam int NORM3_PIPELINED_LATENCY_INST =
      norm3_pipelined_latency(SQRT_SIG_BITS, DIV_WIDTH, DIV_FRAC_BITS);                 // 151

endpackage
