`timescale 1ns / 1ps
`default_nettype none
//
// FOLDED (sequential, resource-shared) inverse square root: S / sqrt(x).
//
// Drop-in replacement for axis_fixed_inv_sqrt_nodsp with an IDENTICAL port list,
// intended for PER-BURST use (event_basis) where latency is nearly free and
// throughput is irrelevant. The pipelined _nodsp variant fully UNROLLS the sqrt
// (26 stages) and the split divider (116 stages) and carries the x/y/z/shift
// sideband through two 100-bit delay lines -- ~28k FFs and cells smeared across
// the whole device, which is the design's dominant route-bound congestion.
//
// This variant runs the SAME two digit recurrences one iteration per cycle over
// a single small datapath (an FSM), so:
//   * the sideband just waits in one register -> the delay lines vanish,
//   * each loop's critical path is a single subtract (trivially meets 200 MHz),
//   * ~75x fewer FFs per instance and it places compactly.
// The per-iteration arithmetic is copied bit-for-bit from axis_fixed_sqrt
// (SIG_BITS=24) and axis_fixed_div (WIDTH=32, FRAC_BITS=25), so the result is
// bit-identical to the pipelined variant (the div sub/select split was purely a
// timing move -- "result bit-identical" per axis_fixed_div.sv). Throughput is
// 1 result / (~24 + ~57 + overhead) cycles; s00_axis_tready is low while busy.
//
module axis_fixed_inv_sqrt_folded #
  (
    parameter integer FRAC_BITS = 32,
    parameter integer ADDR_BITS = 14 // Unused; kept for interface compatibility
  )
  (
    input wire  s00_axis_aclk, s00_axis_aresetn,
    input wire  s00_axis_tlast, s00_axis_tvalid,
    input wire [FRAC_BITS-1 : 0] s00_axis_tdata,
    input wire [(FRAC_BITS/8)-1: 0] s00_axis_tstrb,
    output logic  s00_axis_tready,

    input wire [31:0] s00_axis_user_x,
    input wire [31:0] s00_axis_user_y,
    input wire [31:0] s00_axis_user_z,
    input wire [3:0]  s00_axis_user_shift,

    input wire  m00_axis_aclk, m00_axis_aresetn,
    input wire  m00_axis_tready,
    output logic  m00_axis_tvalid, m00_axis_tlast,
    output logic [FRAC_BITS-1 : 0] m00_axis_tdata,
    output logic [(FRAC_BITS/8)-1: 0] m00_axis_tstrb,

    output logic [31:0] m00_axis_user_x,
    output logic [31:0] m00_axis_user_y,
    output logic [31:0] m00_axis_user_z,
    output logic [3:0]  m00_axis_user_shift
  );

  // ---- Fixed-point constants (identical to axis_fixed_inv_sqrt_nodsp) --------
  localparam logic [31:0] X_MIN_UQ0_32     = 32'd131072;      // 2^-15 in UQ0.32
  localparam logic [31:0] DIVIDEND_S_UQ0_32 = 32'h4000_0000;  // S = 0.25 in UQ0.32

  // ---- sqrt recurrence params (mirror axis_fixed_sqrt with SIG_BITS=24) ------
  localparam int SIG_BITS = 24;
  localparam int RAD_W    = 2*SIG_BITS;   // 48
  localparam int ROOT_W   = SIG_BITS;     // 24
  localparam int REM_W    = SIG_BITS + 3; // 27
  localparam int SQRT_ITERS = SIG_BITS;   // 24 active iterations

  // ---- divider recurrence params (mirror axis_fixed_div W=32, FRAC=25) -------
  localparam int DWIDTH     = 32;
  localparam int DFRAC      = 25;
  localparam int DIVIDEND_W = DWIDTH + DFRAC; // 57
  localparam int DIV_ITERS  = DWIDTH + DFRAC; // 57

  // Unused AXIS sideband/clock ports referenced to avoid no-load warnings.
  wire _unused_s00_tlast   = s00_axis_tlast;
  wire _unused_s00_tstrb   = ^s00_axis_tstrb;
  wire _unused_m00_aclk    = m00_axis_aclk;
  wire _unused_m00_aresetn = m00_axis_aresetn;

  // ---- FSM ------------------------------------------------------------------
  typedef enum logic [1:0] { S_IDLE, S_SQRT, S_DIV, S_EMIT } state_t;
  state_t state;

  // sqrt datapath registers
  logic [RAD_W-1:0]        rad;
  logic [ROOT_W-1:0]       root;
  logic signed [REM_W-1:0] rem;
  logic [$clog2(SQRT_ITERS+1)-1:0] sqrt_i;

  // divider datapath registers
  logic [DIVIDEND_W-1:0] dividend;
  logic [DWIDTH-1:0]     divisor;
  logic [DWIDTH-1:0]     drem;
  logic [DIVIDEND_W-1:0] q;
  logic [$clog2(DIV_ITERS+1)-1:0] div_i;

  // sideband holding register
  logic [31:0] sb_x, sb_y, sb_z;
  logic [3:0]  sb_shift;

  // ---- sqrt iteration (combinational, identical to axis_fixed_sqrt) ---------
  logic [1:0]              s_bits;
  logic signed [REM_W+1:0] s_rem_shift, s_trial, s_addend, s_rem_next;
  always_comb begin
    s_bits      = rad[RAD_W-1 -: 2];
    s_rem_shift = ($signed({{2{rem[REM_W-1]}}, rem}) <<< 2) | s_bits;
    s_trial     = rem[REM_W-1] ? (($signed({1'b0, root}) <<< 2) | 'sd3)
                               : (($signed({1'b0, root}) <<< 2) | 'sd1);
    s_addend    = rem[REM_W-1] ? s_trial : -s_trial;
    s_rem_next  = s_rem_shift + s_addend;
  end
  // root value AFTER this iteration's update (the new MSB-first bit appended),
  // and its left-aligned UQ0.32 form used as the divider's divisor.
  wire [ROOT_W-1:0] root_next = (root << 1) | (s_rem_next[REM_W+1] ? 1'b0 : 1'b1);
  wire [DWIDTH-1:0] sqrt_out_next = {root_next, {(DWIDTH-SIG_BITS){1'b0}}};

  // ---- divider iteration (combinational, identical to axis_fixed_div) -------
  logic [DWIDTH:0] d_shifted, d_trial;
  logic            d_borrow;
  always_comb begin
    d_shifted = {drem, dividend[DIVIDEND_W-1]};
    d_trial   = d_shifted - {1'b0, divisor};
    d_borrow  = d_trial[DWIDTH];
  end

  // input clamp (identical to _nodsp)
  wire [31:0] x_clamped = (s00_axis_tdata < X_MIN_UQ0_32) ? X_MIN_UQ0_32
                                                          : s00_axis_tdata[31:0];

  always_ff @(posedge s00_axis_aclk) begin
    if (!s00_axis_aresetn) begin
      state           <= S_IDLE;
      s00_axis_tready <= 1'b1;
      m00_axis_tvalid <= 1'b0;
      m00_axis_tdata  <= '0;
      rad <= '0; root <= '0; rem <= '0; sqrt_i <= '0;
      dividend <= '0; divisor <= '0; drem <= '0; q <= '0; div_i <= '0;
      sb_x <= '0; sb_y <= '0; sb_z <= '0; sb_shift <= '0;
      m00_axis_user_x <= '0; m00_axis_user_y <= '0;
      m00_axis_user_z <= '0; m00_axis_user_shift <= '0;
    end else begin
      case (state)
        S_IDLE: begin
          s00_axis_tready <= 1'b1;
          if (s00_axis_tvalid && s00_axis_tready) begin
            // load sqrt: rad = top RAD_W bits of {x,32'b0} = {x, 16'b0}
            rad    <= {x_clamped, {(RAD_W-32){1'b0}}};
            root   <= '0;
            rem    <= '0;
            sqrt_i <= '0;
            sb_x   <= s00_axis_user_x;
            sb_y   <= s00_axis_user_y;
            sb_z   <= s00_axis_user_z;
            sb_shift <= s00_axis_user_shift;
            s00_axis_tready <= 1'b0;
            state  <= S_SQRT;
          end
        end

        S_SQRT: begin
          rem  <= s_rem_next[REM_W-1:0];
          root <= root_next;
          rad  <= rad << 2;
          if (sqrt_i == SQRT_ITERS-1) begin
            // final (24th) iteration: hand the just-computed root to the divider
            divisor  <= sqrt_out_next;
            dividend <= {DIVIDEND_S_UQ0_32, {DFRAC{1'b0}}};
            drem     <= '0;
            q        <= '0;
            div_i    <= '0;
            state    <= S_DIV;
          end else begin
            sqrt_i <= sqrt_i + 1'b1;
          end
        end

        S_DIV: begin
          drem     <= d_borrow ? d_shifted[DWIDTH-1:0] : d_trial[DWIDTH-1:0];
          q        <= (q << 1) | (d_borrow ? 1'b0 : 1'b1);
          dividend <= dividend << 1;
          if (div_i == DIV_ITERS-1) begin
            state <= S_EMIT;
          end else begin
            div_i <= div_i + 1'b1;
          end
        end

        S_EMIT: begin
          m00_axis_tvalid     <= 1'b1;
          m00_axis_tdata      <= q[DWIDTH-1:0];
          m00_axis_user_x     <= sb_x;
          m00_axis_user_y     <= sb_y;
          m00_axis_user_z     <= sb_z;
          m00_axis_user_shift <= sb_shift;
          if (m00_axis_tvalid && m00_axis_tready) begin
            m00_axis_tvalid <= 1'b0;
            s00_axis_tready <= 1'b1;
            state           <= S_IDLE;
          end
        end

        default: state <= S_IDLE;
      endcase
    end
  end

  assign m00_axis_tlast = 1'b0;
  assign m00_axis_tstrb = '1;

endmodule
`default_nettype wire
