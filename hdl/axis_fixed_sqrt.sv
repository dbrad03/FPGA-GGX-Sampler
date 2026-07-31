`timescale 1ns / 1ps
`default_nettype none

module axis_fixed_sqrt #
  (
		parameter integer FRAC_BITS = 32,
		// Number of SIGNIFICANT sqrt bits to compute (== active digit-recurrence
		// stages). The root is built MSB-first, one bit per stage, so SIG_BITS
		// directly trades sqrt precision for the per-iteration remainder add width
		// (the timing binder) and the active-stage area. SIG_BITS = 32 reproduces
		// the original full-precision behaviour bit-for-bit.
		//
		// LATENCY = LAT_STAGES + 2 cycles. LAT_STAGES defaults to SIG_BITS, so a
		// narrowed sqrt runs FEWER stages (lower latency, no idle registers). The
		// reproject/projected_area sideband FIFOs are depth-sized to merely EXCEED
		// this latency, so they tolerate a shorter sqrt automatically; inv_sqrt_nodsp
		// uses an exact-match delay line and sizes it from SIG_BITS (see there).
		// LAT_STAGES can be set > SIG_BITS to pad latency (tail just carries the
		// finished root forward) if a fixed latency is ever wanted.
		parameter integer SIG_BITS   = 32,
		parameter integer LAT_STAGES = SIG_BITS
	)
	(
		// Ports of Axi Slave Bus Interface S00_AXIS
		input wire  s00_axis_aclk, s00_axis_aresetn,
		input wire  s00_axis_tlast, s00_axis_tvalid,
		input wire [FRAC_BITS-1 : 0] s00_axis_tdata,
		input wire [(FRAC_BITS/8)-1: 0] s00_axis_tstrb,
		output logic  s00_axis_tready,

		// Ports of Axi Master Bus Interface M00_AXIS
		input wire  m00_axis_aclk, m00_axis_aresetn,
		input wire  m00_axis_tready,
		output logic  m00_axis_tvalid, m00_axis_tlast,
		output logic [FRAC_BITS-1 : 0] m00_axis_tdata,
		output logic [(FRAC_BITS/8)-1: 0] m00_axis_tstrb
		);

  // This core runs single-clock in this project; keep other AXIS sideband/clock
  // ports referenced so synthesis does not emit no-load warnings.
  wire _unused_s00_tlast = s00_axis_tlast;
  wire _unused_s00_tstrb = ^s00_axis_tstrb;
  wire _unused_m00_aclk = m00_axis_aclk;
  wire _unused_m00_aresetn = m00_axis_aresetn;

  // ============================================================
  // Fixed-point contract (PINNED empirically 2026-07-27, see below):
  //   Input  x_in : u32 as UQ0.32 in [0,1)  (x = x_in / 2^32)
  //   Output y_out: u32 as UQ0.32 in [0,1)  (y = y_out / 2^32 ~= sqrt(x))
  //
  // We integer-sqrt the radicand
  //   N = x_in << 32   (64-bit)   => sqrt(N) = sqrt(x_in) * 2^16 = sqrt(x) * 2^32
  // and emit floor(sqrt(N)) DIRECTLY as UQ0.32 (NO extra shift -- the earlier
  // "root<<15 / Q1.31" comments were stale; verified against perfect squares:
  // x_in = k^2 -> out = k << 16).
  //
  // NARROWING (SIG_BITS < 32): the root is produced MSB-first, so computing only
  // the top SIG_BITS bits yields floor(sqrt(x)*2^SIG_BITS); we left-align it back
  // into the UQ0.32 field (low FRAC_BITS-SIG_BITS bits zero). Only the top 2*SIG_BITS
  // radicand bits are consumed, so RAD_W and the remainder add shrink with SIG_BITS.
  // ============================================================

  /// PIPELINE CONTROL LOGIC
  logic pipe_en;
  assign pipe_en = m00_axis_tready || !m00_axis_tvalid;
  assign s00_axis_tready = pipe_en;
  assign m00_axis_tlast = 1'b0;
  assign m00_axis_tstrb = '1;

  // The package defines this core's latency; the core sizes itself to match,
  // rather than the package documenting a number the core happens to have.
  // See docs/adr/0002-latency-package-is-law.md.
  localparam int LATENCY = ggx_latency_pkg::sqrt_latency(LAT_STAGES);
  localparam int STAGES = LATENCY - 2;   // pipeline depth (latency = STAGES+2)
  localparam int ACTIVE = SIG_BITS;      // number of stages that run the recurrence
  localparam int ROOT_W = SIG_BITS;      // root width == significant bits
  localparam int RAD_W  = 2*SIG_BITS;    // radicand width (2 bits consumed / stage)
  // Signed remainder for the NON-RESTORING recurrence. With an N-bit root the
  // trial magnitude is < 2^(N+2) and |rem| < 2^(N+1), so SIG_BITS+3 signed bits
  // (sign + N+2) cover it with margin. The shift/add-sub is done in a wider
  // temporary (REM_W+2) to hold the transient 4*rem term.
  localparam int REM_W  = SIG_BITS + 3;

  logic [STAGES:0]        valid;
  logic [RAD_W-1:0]       rad   [0:STAGES]; // shifting radicand (MSB pair consumed each stage)
  logic [ROOT_W-1:0]      root  [0:STAGES]; // partial root (built MSB-first, 0/1 bits)
  logic signed [REM_W-1:0] rem  [0:STAGES]; // signed partial remainder

  // combinational helpers
  logic [1:0]              bits;
  logic signed [REM_W+1:0] rem_shift; // 4*rem | bits (wider to hold the 4x term)
  logic signed [REM_W+1:0] trial;     // (root<<2) | 1 (subtract) or | 3 (add)
  logic signed [REM_W+1:0] addend;    // +trial when rem<0, -trial when rem>=0
  logic signed [REM_W+1:0] rem_next;

  // Radicand load: top RAD_W bits of {x_in, 32'b0}. For SIG_BITS >= 16 this is
  // {x_in, zero-pad}; for SIG_BITS < 16 it would drop low input bits (unused range).
  wire [RAD_W-1:0] rad_load;
  generate
    if (RAD_W >= FRAC_BITS)
      assign rad_load = {s00_axis_tdata, {(RAD_W-FRAC_BITS){1'b0}}};
    else
      assign rad_load = s00_axis_tdata[FRAC_BITS-1 -: RAD_W];
  endgenerate

  // Output: left-align the SIG_BITS-wide root back into the UQ0.32 field.
  wire [FRAC_BITS-1:0] root_out;
  generate
    if (SIG_BITS >= FRAC_BITS)
      assign root_out = root[STAGES][FRAC_BITS-1:0];
    else
      assign root_out = {root[STAGES], {(FRAC_BITS-SIG_BITS){1'b0}}};
  endgenerate

  integer i;
  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      for (i = 0; i <= STAGES; i=i+1) begin
        valid[i] <= 1'b0;
        rad[i]   <= '0;
        root[i]  <= '0;
        rem[i]   <= '0;
      end
      m00_axis_tvalid <= 1'b0;
      m00_axis_tdata  <= '0;
    end else if (pipe_en) begin

      // STAGE 0 LOAD
      valid[0] <= s00_axis_tvalid;
      if (s00_axis_tvalid) begin
        rad[0] <= rad_load; // top RAD_W bits of x_in << 32
        root[0] <= '0;
        rem[0]  <= '0;
      end else begin
        rad[0] <= '0;
        root[0] <= '0;
        rem[0]  <= '0;
      end

      // STAGES 0..STAGES-1 transform -> 1..STAGES. The first ACTIVE stages run the
      // non-restoring recurrence (no compare, no restore mux -- the add/subtract
      // mode is just the registered sign of rem[i], and the new root bit is the
      // sign of the result). Stages ACTIVE..STAGES-1 carry the finished root
      // forward to hold latency fixed (i < ACTIVE constant-folds per unrolled i).
      for (i = 0; i < STAGES; i=i+1) begin
        valid[i+1] <= valid[i];

        if (i < ACTIVE) begin
          // consume next 2 MSBs of radicand: rem_shift = 4*rem[i] + bits
          bits = rad[i][RAD_W-1 -: 2];
          rem_shift = ($signed({{2{rem[i][REM_W-1]}}, rem[i]}) <<< 2) | bits;

          // trial magnitude: (root<<2)|1 if subtracting, (root<<2)|3 if adding
          trial  = rem[i][REM_W-1] ? (($signed({1'b0, root[i]}) <<< 2) | 'sd3)
                                   : (($signed({1'b0, root[i]}) <<< 2) | 'sd1);
          // rem>=0 -> subtract trial ; rem<0 -> add trial
          addend = rem[i][REM_W-1] ? trial : -trial;
          rem_next = rem_shift + addend;

          rem[i+1]  <= rem_next[REM_W-1:0];
          // new root bit = 1 when result non-negative, else 0
          root[i+1] <= (root[i] << 1) | (rem_next[REM_W+1] ? 1'b0 : 1'b1);

          // shift radicand left by 2 to expose next pair of bits next stage
          rad[i+1] <= rad[i] << 2;
        end else begin
          // propagate-only tail: root is complete, just pass it along
          root[i+1] <= root[i];
          rem[i+1]  <= rem[i];
          rad[i+1]  <= rad[i];
        end
      end

      // OUTPUT REGISTER
      m00_axis_tvalid <= valid[STAGES];
      if (valid[STAGES]) begin
        m00_axis_tdata  <= root_out;
      end

    end
  end

endmodule
`default_nettype wire
