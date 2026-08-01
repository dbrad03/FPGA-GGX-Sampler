`timescale 1ns / 1ps
`default_nettype none
//=============================================================================
// axis_oct32_encode -- octahedral output encoder (ADR-0001, issue #14)
//
// Takes an UN-NORMALIZED half-vector and produces two 16-bit fields. It does
// not normalize, and nothing downstream needs to: octahedral encoding is
// exactly scale-invariant, oct(c*v) == oct(v) for c > 0, because the L1
// projection divides the scale out. That is the whole point -- it is what
// deletes the per-sample L2 normalize.
//
//   p = v / (|x| + |y| + |z|)
//   z >= 0 : (u, w) = (p.x, p.y)
//   z <  0 : fold outward across the |u| + |w| = 1 diamond
//
// NO DSPs and NO BRAM, both required:
//  - the divide reuses axis_fixed_div, a non-restoring digit recurrence built
//    from adds and muxes. A reciprocal-plus-multiply was rejected because it
//    puts DSPs back into a path being emptied; a table was rejected because it
//    spends BRAM, one of the three axes the multi-Lane bin-pack balances.
//  - the field mapping is (F + 2^18) >> 3, a pure add and shift. This is why
//    the model quantizes with a 2^bits scale rather than 2^bits - 1; the
//    latter would need a multiplier. See sim/oct32_model.py.
//
// Fully pipelined: one Sample per clock.
//=============================================================================
module axis_oct32_encode #
  (
    parameter integer C_S00_AXIS_TDATA_WIDTH = 96,   // {hz, hy, hx} Q1.31 signed
    parameter integer C_M00_AXIS_TDATA_WIDTH = 32    // {w_field, u_field} 16b each
  )
  (
    input  wire s00_axis_aclk, s00_axis_aresetn,
    input  wire s00_axis_tlast, s00_axis_tvalid,
    input  wire [C_S00_AXIS_TDATA_WIDTH-1:0] s00_axis_tdata,
    input  wire [(C_S00_AXIS_TDATA_WIDTH/8)-1:0] s00_axis_tstrb,
    output logic s00_axis_tready,

    input  wire m00_axis_aclk, m00_axis_aresetn,
    input  wire m00_axis_tready,
    output logic m00_axis_tvalid, m00_axis_tlast,
    output logic [C_M00_AXIS_TDATA_WIDTH-1:0] m00_axis_tdata,
    output logic [(C_M00_AXIS_TDATA_WIDTH/8)-1:0] m00_axis_tstrb
  );

  wire _unused_m00_aclk = m00_axis_aclk;
  wire _unused_m00_aresetn = m00_axis_aresetn;
  wire _unused_strb = ^s00_axis_tstrb;

  // Field width, and the fixed-point format the octahedral coordinates are
  // carried in between the divide and the field mapping.
  localparam int FIELD_BITS = 16;
  localparam int OCT_FRAC   = 18;              // u,w carried as Q1.18
  localparam int OCT_ONE    = 1 << OCT_FRAC;   // 1.0 in that format

  // Divider geometry. 20x18 is the "narrowly parameterized" reuse the ticket
  // asks for: the quotient only has to resolve a 16-bit field, and carrying 18
  // fractional bits leaves two bits of headroom below the field LSB.
  localparam int DIV_W = 20;
  localparam int DIV_F = OCT_FRAC;
  // Latency comes from the package, so this block's sideband delay line cannot
  // drift from the divider it rides alongside. See ADR-0002.
  localparam int DIV_LAT = ggx_latency_pkg::div_latency(DIV_W, DIV_F);

  // L1 can reach 3*2^31, so it needs 34 bits.
  localparam int L1_W = 34;
  // The dividend and divisor are shifted right by the SAME amount, which leaves
  // their ratio -- and so the encoded direction -- unchanged. The amount is
  // derived from L1's leading bit rather than fixed: a fixed shift silently
  // assumes |h| ~ 1, and a half-vector scaled down by 1e-3 would keep only ~8
  // significant bits of divisor. The encoder's contract is that it accepts an
  // UN-normalized vector at any scale, so it has to normalize the shift itself.
  localparam int SHIFT_W = 6;

  wire pipe_en = m00_axis_tready || !m00_axis_tvalid;

  wire signed [31:0] hx = $signed(s00_axis_tdata[31:0]);
  wire signed [31:0] hy = $signed(s00_axis_tdata[63:32]);
  wire signed [31:0] hz = $signed(s00_axis_tdata[95:64]);

  // -------------------------------------------------------------------------
  // Stage 0: magnitudes, signs, L1
  // -------------------------------------------------------------------------
  // |x| of a Q1.31 value needs 32 unsigned bits (|-2^31| = 2^31).
  wire [31:0] mag_x_c = hx[31] ? (~hx + 1'b1) : hx;
  wire [31:0] mag_y_c = hy[31] ? (~hy + 1'b1) : hy;
  wire [31:0] mag_z_c = hz[31] ? (~hz + 1'b1) : hz;

  logic        s0_valid;
  logic [31:0] s0_mag_x, s0_mag_y;
  logic [33:0] s0_l1;
  logic        s0_sx, s0_sy, s0_nz, s0_last;

  always_ff @(posedge s00_axis_aclk) begin
    if (!s00_axis_aresetn) begin
      s0_valid <= 1'b0; s0_mag_x <= '0; s0_mag_y <= '0; s0_l1 <= '0;
      s0_sx <= 1'b0; s0_sy <= 1'b0; s0_nz <= 1'b0; s0_last <= 1'b0;
    end else if (pipe_en) begin
      s0_valid <= s00_axis_tvalid;
      if (s00_axis_tvalid) begin
        s0_mag_x <= mag_x_c;
        s0_mag_y <= mag_y_c;
        s0_l1    <= {2'b0, mag_x_c} + {2'b0, mag_y_c} + {2'b0, mag_z_c};
        s0_sx    <= hx[31];
        s0_sy    <= hy[31];
        s0_nz    <= hz[31];   // the lower-hemisphere fold
        s0_last  <= s00_axis_tlast;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Stage 1: how far must L1 shift to fit the divider?
  // -------------------------------------------------------------------------
  // Priority-encode L1's leading one. Kept in its own stage so the encoder and
  // the barrel shift it drives do not share a cycle.
  logic [SHIFT_W-1:0] shamt_c;
  integer bi;
  always_comb begin
    shamt_c = '0;
    for (bi = DIV_W; bi < L1_W; bi = bi + 1) begin
      if (s0_l1[bi]) shamt_c = (bi - DIV_W + 1);
    end
  end
  // Detect the degenerate all-zero vector HERE, on the unshifted L1, not on the
  // shifted divisor in the next stage. Because shamt is derived from L1's
  // leading one, the shifted divisor is zero if and only if L1 is -- so testing
  // it downstream would put a 20-bit zero-compare directly behind the barrel
  // shift, which is exactly what held this block's worst path.
  wire l1_zero_c = (s0_l1 == '0);

  logic s1_valid;
  logic [L1_W-1:0] s1_l1;
  logic [31:0] s1_mag_x, s1_mag_y;
  logic [SHIFT_W-1:0] s1_shamt;
  logic s1_l1_zero;
  logic s1_sx, s1_sy, s1_nz, s1_last;

  always_ff @(posedge s00_axis_aclk) begin
    if (!s00_axis_aresetn) begin
      s1_valid <= 1'b0; s1_l1 <= '0; s1_mag_x <= '0; s1_mag_y <= '0;
      s1_shamt <= '0; s1_l1_zero <= 1'b0;
      s1_sx <= 1'b0; s1_sy <= 1'b0; s1_nz <= 1'b0; s1_last <= 1'b0;
    end else if (pipe_en) begin
      s1_valid <= s0_valid;
      if (s0_valid) begin
        s1_l1 <= s0_l1; s1_mag_x <= s0_mag_x; s1_mag_y <= s0_mag_y;
        s1_shamt <= shamt_c;
        s1_l1_zero <= l1_zero_c;
        s1_sx <= s0_sx; s1_sy <= s0_sy; s1_nz <= s0_nz; s1_last <= s0_last;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Stage 2: apply the shift. mag <= L1 always, so once L1 fits in DIV_W bits
  // the shifted magnitudes do too -- no separate clamp is needed for them.
  // -------------------------------------------------------------------------
  wire [L1_W-1:0] d_shifted  = s1_l1 >> s1_shamt;
  wire [L1_W-1:0] ax_shifted = {2'b0, s1_mag_x} >> s1_shamt;
  wire [L1_W-1:0] ay_shifted = {2'b0, s1_mag_y} >> s1_shamt;
  // An all-zero input has no direction to encode; clamp so the recurrence still
  // terminates cleanly. The flag comes from stage 1, off this path.
  wire [DIV_W-1:0] div_d_clamped =
      s1_l1_zero ? {{(DIV_W-1){1'b0}}, 1'b1} : d_shifted[DIV_W-1:0];

  logic s2_valid;
  logic [DIV_W-1:0] s2_d, s2_ax, s2_ay;
  logic s2_sx, s2_sy, s2_nz, s2_last;

  always_ff @(posedge s00_axis_aclk) begin
    if (!s00_axis_aresetn) begin
      s2_valid <= 1'b0; s2_d <= '0; s2_ax <= '0; s2_ay <= '0;
      s2_sx <= 1'b0; s2_sy <= 1'b0; s2_nz <= 1'b0; s2_last <= 1'b0;
    end else if (pipe_en) begin
      s2_valid <= s1_valid;
      if (s1_valid) begin
        s2_d  <= div_d_clamped;
        s2_ax <= ax_shifted[DIV_W-1:0];
        s2_ay <= ay_shifted[DIV_W-1:0];
        s2_sx <= s1_sx; s2_sy <= s1_sy; s2_nz <= s1_nz; s2_last <= s1_last;
      end
    end
  end

  // -------------------------------------------------------------------------
  // The two divides. Both see the same valid and the same ready, so their
  // results stay aligned with each other and with the sideband line below.
  // -------------------------------------------------------------------------
  wire [DIV_W-1:0] qx_raw, qy_raw;
  wire div_x_out_valid, div_y_out_valid;
  wire div_x_in_ready;

  axis_fixed_div #(.WIDTH(DIV_W), .FRAC_BITS(DIV_F)) u_div_x (
    .s00_axis_aclk(s00_axis_aclk), .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(1'b0), .s00_axis_tvalid(s2_valid),
    .s00_axis_tdata({s2_ax, s2_d}), .s00_axis_tstrb('1),
    .s00_axis_tready(div_x_in_ready),
    .m00_axis_aclk(s00_axis_aclk), .m00_axis_aresetn(s00_axis_aresetn),
    .m00_axis_tready(pipe_en), .m00_axis_tvalid(div_x_out_valid),
    .m00_axis_tlast(), .m00_axis_tdata(qx_raw), .m00_axis_tstrb()
  );

  axis_fixed_div #(.WIDTH(DIV_W), .FRAC_BITS(DIV_F)) u_div_y (
    .s00_axis_aclk(s00_axis_aclk), .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(1'b0), .s00_axis_tvalid(s2_valid),
    .s00_axis_tdata({s2_ay, s2_d}), .s00_axis_tstrb('1),
    .s00_axis_tready(), .m00_axis_aclk(s00_axis_aclk),
    .m00_axis_aresetn(s00_axis_aresetn),
    .m00_axis_tready(pipe_en), .m00_axis_tvalid(div_y_out_valid),
    .m00_axis_tlast(), .m00_axis_tdata(qy_raw), .m00_axis_tstrb()
  );

  // Sideband rides an exact-match delay line: DIV_LAT deep, taken from the
  // package so it cannot drift from the divider. Skew here would pair a
  // quotient with another Sample's signs -- silent, and wrong by a reflection.
  logic [2:0] sb [0:DIV_LAT-1];
  logic       sb_last [0:DIV_LAT-1];
  integer k;
  always_ff @(posedge s00_axis_aclk) begin
    if (!s00_axis_aresetn) begin
      for (k = 0; k < DIV_LAT; k = k + 1) begin
        sb[k] <= 3'b0; sb_last[k] <= 1'b0;
      end
    end else if (pipe_en) begin
      sb[0] <= {s2_nz, s2_sy, s2_sx};
      sb_last[0] <= s2_last;
      for (k = 1; k < DIV_LAT; k = k + 1) begin
        sb[k] <= sb[k-1];
        sb_last[k] <= sb_last[k-1];
      end
    end
  end

  wire d_sx = sb[DIV_LAT-1][0];
  wire d_sy = sb[DIV_LAT-1][1];
  wire d_nz = sb[DIV_LAT-1][2];
  wire d_last = sb_last[DIV_LAT-1];

  initial begin
    if (DIV_LAT != ggx_latency_pkg::div_latency(DIV_W, DIV_F))
      $fatal(1, "axis_oct32_encode: sideband depth %0d != divider latency %0d",
             DIV_LAT, ggx_latency_pkg::div_latency(DIV_W, DIV_F));
  end

  // -------------------------------------------------------------------------
  // Stage 2: fold, then map to fields
  // -------------------------------------------------------------------------
  // |u| and |w| are the quotients themselves. Under the fold the roles swap:
  // |u'| = 1 - |w| and |w'| = 1 - |u|. Since |x| + |y| <= L1, both stay in
  // [0, 1] and no saturation is needed.
  wire [OCT_FRAC:0] abs_u = d_nz ? (OCT_ONE[OCT_FRAC:0] - qy_raw[OCT_FRAC:0])
                                 : qx_raw[OCT_FRAC:0];
  wire [OCT_FRAC:0] abs_w = d_nz ? (OCT_ONE[OCT_FRAC:0] - qx_raw[OCT_FRAC:0])
                                 : qy_raw[OCT_FRAC:0];

  wire signed [OCT_FRAC+1:0] su = d_sx ? -$signed({1'b0, abs_u}) : $signed({1'b0, abs_u});
  wire signed [OCT_FRAC+1:0] sw = d_sy ? -$signed({1'b0, abs_w}) : $signed({1'b0, abs_w});

  // Field mapping: q = clamp((F + 1.0) >> (OCT_FRAC + 1 - FIELD_BITS)).
  // Pure add and shift -- no multiplier, hence no DSP.
  localparam int MAP_SHIFT = OCT_FRAC + 1 - FIELD_BITS;   // 3
  wire [OCT_FRAC+1:0] biased_u = su + $signed({2'b0, OCT_ONE[OCT_FRAC:0]});
  wire [OCT_FRAC+1:0] biased_w = sw + $signed({2'b0, OCT_ONE[OCT_FRAC:0]});
  wire [OCT_FRAC+1:0] mapped_u = biased_u >> MAP_SHIFT;
  wire [OCT_FRAC+1:0] mapped_w = biased_w >> MAP_SHIFT;
  wire [FIELD_BITS-1:0] field_u =
      (mapped_u >= (1 << FIELD_BITS)) ? {FIELD_BITS{1'b1}} : mapped_u[FIELD_BITS-1:0];
  wire [FIELD_BITS-1:0] field_w =
      (mapped_w >= (1 << FIELD_BITS)) ? {FIELD_BITS{1'b1}} : mapped_w[FIELD_BITS-1:0];

  always_ff @(posedge s00_axis_aclk) begin
    if (!s00_axis_aresetn) begin
      m00_axis_tvalid <= 1'b0;
      m00_axis_tdata  <= '0;
      m00_axis_tlast  <= 1'b0;
    end else if (pipe_en) begin
      m00_axis_tvalid <= div_x_out_valid && div_y_out_valid;
      if (div_x_out_valid && div_y_out_valid) begin
        m00_axis_tdata <= {field_w, field_u};
        m00_axis_tlast <= d_last;
      end
    end
  end

  assign s00_axis_tready = pipe_en && div_x_in_ready;
  assign m00_axis_tstrb  = '1;

endmodule
`default_nettype wire
