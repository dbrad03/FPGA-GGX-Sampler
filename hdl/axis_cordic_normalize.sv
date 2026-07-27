`timescale 1ns / 1ps
`default_nettype none
//=============================================================================
// axis_cordic_normalize
//
// Dividerless, multiplier-free 3D vector normalize (Step 1.3 of the refactor
// plan). Input  {z,y,x} each Q1.31 signed; output {z,y,x} = v/|v| Q1.31 signed.
// Uses ONLY shifts and adds -> 0 DSP, 0 BRAM. (The two CORDIC gain-compensation
// constants are `use_dsp="no"` constant multiplies = shift-add in LUTs.)
//
// METHOD (bit-accurate reference: sim/test_cordic_normalize.py / cordic_dev.py):
//   PASS 1 - VECTORING (measure): rotate v onto the +x axis in two planes,
//     recording per-step sign decisions. Plane 1 folds y into x (xy-plane),
//     plane 2 folds z into x (xz-plane); R = R2.R1 with R.v ~ +x. Between planes
//     x is rescaled by 1/G so plane 2 sees a consistently-scaled pair.
//   PASS 2 - ROTATION (reconstruct): v/|v| = R^-1.e_x = R1^-1 . R2^-1 . e_x.
//     Seed unit +x (scaled 1/G^2) and replay INVERSE rotations in REVERSE plane
//     order: R2^-1 (xz, -sigma2) then R1^-1 (xy, -sigma1). z passes through one
//     rotation vs two for x,y -> one gain short -> single xG correction.
//   RANGE: circular vectoring needs x-input>=0; if vx<0 negate v (flag), undo.
//=============================================================================
module axis_cordic_normalize #
  (
    parameter integer NROT = 18,   // CORDIC iterations per plane (precision)
    parameter integer FRAC = 24,   // internal fractional bits
    parameter integer IW   = 28    // internal signed word width
  )
  (
    input  wire s00_axis_aclk, s00_axis_aresetn,
    input  wire s00_axis_tlast, s00_axis_tvalid,
    input  wire [127:0] s00_axis_tdata,  // {pad, z, y, x} Q1.31 signed (pad ignored)
    input  wire [15:0] s00_axis_tstrb,
    output wire s00_axis_tready,

    input  wire m00_axis_aclk, m00_axis_aresetn,
    input  wire m00_axis_tready,
    output logic m00_axis_tvalid, m00_axis_tlast,
    output logic [127:0] m00_axis_tdata, // {0, z, y, x} unit vector Q1.31 signed
    output logic [15:0] m00_axis_tstrb
  );

  wire _unused = ^{s00_axis_tstrb, s00_axis_tdata[127:96], m00_axis_aclk, m00_axis_aresetn};

  // CORDIC gain and Q(FRAC) compensation constants (elaboration-time).
  function automatic real cordic_gain(input integer n);
    real g; integer i;
    begin
      g = 1.0;
      for (i = 0; i < n; i = i + 1) g = g * $sqrt(1.0 + 2.0**(-2*i));
      cordic_gain = g;
    end
  endfunction
  localparam real    GAIN    = cordic_gain(NROT);
  localparam integer INVG_Q  = integer'((1.0/GAIN)        * (2.0**FRAC) + 0.5);
  localparam integer INVG2_Q = integer'((1.0/(GAIN*GAIN)) * (2.0**FRAC) + 0.5);
  localparam integer GAIN_Q  = integer'(GAIN              * (2.0**FRAC) + 0.5);

  // Stage-base indices (module scope for Icarus).
  localparam integer B_SCALE = NROT;       // px[NROT] -> rescale by 1/G
  localparam integer B_P2    = NROT + 1;   // plane-2 vectoring base
  localparam integer B_SEED  = 2*NROT + 1; // seed reconstruct
  localparam integer B_R2    = 2*NROT + 2; // R2^-1 base
  localparam integer B_R1    = 3*NROT + 2; // R1^-1 base
  localparam integer B_FIN   = 4*NROT + 2; // z*G correction + un-flip
  localparam integer B_SAT   = 4*NROT + 3; // saturate to Q1.31 (own stage)
  localparam integer NST     = 4*NROT + 4; // pipeline depth

  // Constant multiply by a compile-time Q(FRAC) constant C, returning
  // (a*C) >> FRAC, built as a SHIFT-ADD sum over C's set bits -> adder tree, 0
  // DSP (no `*`). Bit-identical to the multiply since all FRAC+ bits are summed.
  function automatic logic signed [IW-1:0] cmul(input logic signed [IW-1:0] a,
                                                input integer C);
    logic signed [IW+FRAC+4:0] acc;
    integer b;
    begin
      acc = '0;
      for (b = 0; b <= FRAC + 3; b = b + 1)
        if (C[b]) acc = acc + ($signed(a) <<< b);
      cmul = IW'(acc >>> FRAC);
    end
  endfunction

  wire pipe_en = m00_axis_tready || !m00_axis_tvalid;
  assign s00_axis_tready = pipe_en;
  assign m00_axis_tstrb  = '1;

  localparam logic signed [31:0] ONE_Q1 = 32'sh7FFF_FFFF;
  localparam logic signed [31:0] NEG_Q1 = 32'sh8000_0000;

  // Q1.31 -> internal Q(.FRAC)
  function automatic logic signed [IW-1:0] to_int(input logic signed [31:0] q131);
    to_int = IW'(q131 >>> (31 - FRAC));
  endfunction
  // internal Q(.FRAC) -> saturated Q1.31
  function automatic logic signed [31:0] to_q131(input logic signed [IW-1:0] v);
    logic signed [IW+8:0] up;
    begin
      up = $signed(v) <<< (31 - FRAC);
      if (up > ONE_Q1)      to_q131 = ONE_Q1;
      else if (up < NEG_Q1) to_q131 = NEG_Q1;
      else                  to_q131 = up[31:0];
    end
  endfunction

  // Pipeline registers.
  logic signed [IW-1:0] px [0:NST];
  logic signed [IW-1:0] py [0:NST];
  logic signed [IW-1:0] pz [0:NST];
  logic [NROT-1:0] sig1 [0:NST];
  logic [NROT-1:0] sig2 [0:NST];
  logic            flip [0:NST];
  logic [NST:0]    vld;
  logic [NST:0]    lst;

  // Module-scope temporaries (blocking, like axis_fixed_sqrt).
  integer s;
  logic signed [IW-1:0] x0, y0, z0;
  logic sneg;
  logic signed [IW-1:0] fnx, fny, fnz;

  always_ff @(posedge s00_axis_aclk) begin
    if (!s00_axis_aresetn) begin
      for (s = 0; s <= NST; s = s + 1) begin
        px[s] <= '0; py[s] <= '0; pz[s] <= '0;
        sig1[s] <= '0; sig2[s] <= '0; flip[s] <= 1'b0;
      end
      vld <= '0; lst <= '0;
      m00_axis_tvalid <= 1'b0; m00_axis_tlast <= 1'b0; m00_axis_tdata <= '0;
    end else if (pipe_en) begin
      // ---- Stage 0: load + range reduce ------------------------------------
      x0 = to_int($signed(s00_axis_tdata[31:0]));
      y0 = to_int($signed(s00_axis_tdata[63:32]));
      z0 = to_int($signed(s00_axis_tdata[95:64]));
      vld[0] <= s00_axis_tvalid;
      lst[0] <= s00_axis_tlast;
      flip[0] <= x0[IW-1];               // vx<0 -> negate whole vector
      px[0] <= x0[IW-1] ? -x0 : x0;
      py[0] <= x0[IW-1] ? -y0 : y0;
      pz[0] <= x0[IW-1] ? -z0 : z0;
      sig1[0] <= '0; sig2[0] <= '0;

      // ---- Stages 1..NROT: plane-1 vectoring (fold y into x) ---------------
      for (s = 0; s < NROT; s = s + 1) begin
        sneg = ~py[s][IW-1];             // y>=0 -> ref sigma=-1 (sneg=1)
        px[s+1] <= sneg ? (px[s] + (py[s] >>> s)) : (px[s] - (py[s] >>> s));
        py[s+1] <= sneg ? (py[s] - (px[s] >>> s)) : (py[s] + (px[s] >>> s));
        pz[s+1] <= pz[s];
        sig1[s+1] <= sig1[s] | (sneg ? (NROT'(1) << s) : NROT'(0));
        sig2[s+1] <= sig2[s];
        flip[s+1] <= flip[s]; vld[s+1] <= vld[s]; lst[s+1] <= lst[s];
      end

      // ---- Stage NROT+1: rescale x by 1/G (shift-add, 0 DSP) ---------------
      px[B_SCALE+1] <= cmul(px[B_SCALE], INVG_Q);
      py[B_SCALE+1] <= py[B_SCALE];
      pz[B_SCALE+1] <= pz[B_SCALE];
      sig1[B_SCALE+1] <= sig1[B_SCALE]; sig2[B_SCALE+1] <= sig2[B_SCALE];
      flip[B_SCALE+1] <= flip[B_SCALE]; vld[B_SCALE+1] <= vld[B_SCALE]; lst[B_SCALE+1] <= lst[B_SCALE];

      // ---- Stages: plane-2 vectoring (fold z into x) -----------------------
      for (s = 0; s < NROT; s = s + 1) begin
        sneg = ~pz[B_P2+s][IW-1];        // z>=0 -> ref sigma=-1
        px[B_P2+s+1] <= sneg ? (px[B_P2+s] + (pz[B_P2+s] >>> s)) : (px[B_P2+s] - (pz[B_P2+s] >>> s));
        pz[B_P2+s+1] <= sneg ? (pz[B_P2+s] - (px[B_P2+s] >>> s)) : (pz[B_P2+s] + (px[B_P2+s] >>> s));
        py[B_P2+s+1] <= py[B_P2+s];
        sig2[B_P2+s+1] <= sig2[B_P2+s] | (sneg ? (NROT'(1) << s) : NROT'(0));
        sig1[B_P2+s+1] <= sig1[B_P2+s];
        flip[B_P2+s+1] <= flip[B_P2+s]; vld[B_P2+s+1] <= vld[B_P2+s]; lst[B_P2+s+1] <= lst[B_P2+s];
      end

      // ---- Stage: seed reconstruct e_x * (1/G^2) ---------------------------
      px[B_SEED+1] <= IW'(INVG2_Q);
      py[B_SEED+1] <= '0;
      pz[B_SEED+1] <= '0;
      sig1[B_SEED+1] <= sig1[B_SEED]; sig2[B_SEED+1] <= sig2[B_SEED];
      flip[B_SEED+1] <= flip[B_SEED]; vld[B_SEED+1] <= vld[B_SEED]; lst[B_SEED+1] <= lst[B_SEED];

      // ---- Stages: R2^-1 (xz-plane, inverse of sigma2) ---------------------
      for (s = 0; s < NROT; s = s + 1) begin
        sneg = ~sig2[B_R2+s][s];         // inverse: negate stored sign
        px[B_R2+s+1] <= sneg ? (px[B_R2+s] + (pz[B_R2+s] >>> s)) : (px[B_R2+s] - (pz[B_R2+s] >>> s));
        pz[B_R2+s+1] <= sneg ? (pz[B_R2+s] - (px[B_R2+s] >>> s)) : (pz[B_R2+s] + (px[B_R2+s] >>> s));
        py[B_R2+s+1] <= py[B_R2+s];
        sig1[B_R2+s+1] <= sig1[B_R2+s]; sig2[B_R2+s+1] <= sig2[B_R2+s];
        flip[B_R2+s+1] <= flip[B_R2+s]; vld[B_R2+s+1] <= vld[B_R2+s]; lst[B_R2+s+1] <= lst[B_R2+s];
      end

      // ---- Stages: R1^-1 (xy-plane, inverse of sigma1) ---------------------
      for (s = 0; s < NROT; s = s + 1) begin
        sneg = ~sig1[B_R1+s][s];
        px[B_R1+s+1] <= sneg ? (px[B_R1+s] + (py[B_R1+s] >>> s)) : (px[B_R1+s] - (py[B_R1+s] >>> s));
        py[B_R1+s+1] <= sneg ? (py[B_R1+s] - (px[B_R1+s] >>> s)) : (py[B_R1+s] + (px[B_R1+s] >>> s));
        pz[B_R1+s+1] <= pz[B_R1+s];
        sig1[B_R1+s+1] <= sig1[B_R1+s]; sig2[B_R1+s+1] <= sig2[B_R1+s];
        flip[B_R1+s+1] <= flip[B_R1+s]; vld[B_R1+s+1] <= vld[B_R1+s]; lst[B_R1+s+1] <= lst[B_R1+s];
      end

      // ---- Stage B_FIN: z*G correction (shift-add, 0 DSP) + un-flip --------
      fnx = px[B_FIN];
      fny = py[B_FIN];
      fnz = cmul(pz[B_FIN], GAIN_Q);   // z saw one fewer rotation -> one gain short
      px[B_FIN+1] <= flip[B_FIN] ? -fnx : fnx;
      py[B_FIN+1] <= flip[B_FIN] ? -fny : fny;
      pz[B_FIN+1] <= flip[B_FIN] ? -fnz : fnz;
      vld[B_FIN+1] <= vld[B_FIN];
      lst[B_FIN+1] <= lst[B_FIN];

      // ---- Stage B_SAT: saturate to Q1.31, drive output --------------------
      m00_axis_tvalid <= vld[B_SAT];
      m00_axis_tlast  <= lst[B_SAT];
      m00_axis_tdata  <= {32'b0, to_q131(pz[B_SAT]), to_q131(py[B_SAT]), to_q131(px[B_SAT])};
    end
  end

endmodule
`default_nettype wire
