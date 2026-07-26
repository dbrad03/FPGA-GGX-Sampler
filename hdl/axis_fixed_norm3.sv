`timescale 1ns / 1ps
`default_nettype none

// ============================================================
// axis_fixed_norm3
//
// Normalize a 3D vector (x,y,z) given as signed Q1.31 each.
// Uses axis_fixed_inv_sqrt (input UQ0.32 lensq in [0,1), output Q7.25 inv_len_scaled)
// to compute inv_len = 1/sqrt(lensq), then multiplies components by inv_len.
//
// Input payload  (128b): {pad[31:0], in_z[31:0], in_y[31:0], in_x[31:0]}
// Output payload (128b): {pad[31:0], out_z[31:0], out_y[31:0], out_x[31:0]}
//
// Formats:
//   in_x/in_y/in_z  : signed Q1.31
//   lensq_uq0_32    : unsigned UQ0.32  (clamped to <1.0)
//   inv_len_q7_25   : unsigned Q7.25 (from axis_fixed_inv_sqrt; scaled by S internally)
//   out_*           : signed Q1.31  (normalized vector)
//
// Handshake:
//   Fully backpressure-safe. We only accept an input when BOTH:
//     - downstream can accept (pipe_en)
//     - inv_sqrt input can accept (inv_in_ready)
//   All internal delay lines advance with the same enable, so alignment is preserved.
//
// NOTE: Set INV_SQRT_LATENCY to your measured inv_sqrt latency (you said ~8 cycles).
// ============================================================

module axis_fixed_norm3#
  (
    parameter integer C_S00_AXIS_TDATA_WIDTH	= 128,
		parameter integer C_M00_AXIS_TDATA_WIDTH	= 128,
		parameter integer FRAC_BITS = 32
	)
	(
		// Ports of Axi Slave Bus Interface S00_AXIS
		input wire  s00_axis_aclk, s00_axis_aresetn,
		input wire  s00_axis_tlast, s00_axis_tvalid,
		input wire [C_S00_AXIS_TDATA_WIDTH-1 : 0] s00_axis_tdata, // {pass, in_z, in_y, in_x}
		input wire [(C_S00_AXIS_TDATA_WIDTH/8)-1: 0] s00_axis_tstrb,
		output logic  s00_axis_tready,

		// Ports of Axi Master Bus Interface M00_AXIS
		input wire  m00_axis_aclk, m00_axis_aresetn,
		input wire  m00_axis_tready,
		output logic  m00_axis_tvalid, m00_axis_tlast,
		output logic [C_M00_AXIS_TDATA_WIDTH-1 : 0] m00_axis_tdata, // (pass, out_z, out_y, out_x)
		output logic [(C_S00_AXIS_TDATA_WIDTH/8)-1: 0] m00_axis_tstrb
	);

  // localparam logic [31:0] LENSQ_MIN_UQ0_32 = 32'h0004_0000; // 1/128^2
  localparam logic [31:0] LENSQ_MIN_UQ0_32 = 32'h0002_0000; // 2**-15

  logic pipe_en;
  assign pipe_en = m00_axis_tready || !m00_axis_tvalid;

  wire inv_in_ready;
  wire inv_out_valid;
  wire norm_en = pipe_en && inv_in_ready;

  assign s00_axis_tready = norm_en;
  assign m00_axis_tlast = 1'b0;
  assign m00_axis_tstrb = '1;

  // Unpack input
  wire signed [FRAC_BITS-1:0] in_x = s00_axis_tdata[31:0];
  wire signed [FRAC_BITS-1:0] in_y = s00_axis_tdata[63:32];
  wire signed [FRAC_BITS-1:0] in_z = s00_axis_tdata[95:64];
  wire        [31:0]          in_pad = s00_axis_tdata[127:96];


  /// STAGE 00a: register inputs (breaks long comb path into module; allows DSP PREG in 00b)
  logic signed [31:0] x_in_r, y_in_r, z_in_r;
  // 18-bit rounded operands for the squares, captured here alongside x_in_r so the
  // round-half-up carry rides the input-capture path, not the reg -> DSP path.
  logic signed [17:0] x18_r, y18_r, z18_r;
  logic              s00a_valid;

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      x_in_r <= '0; y_in_r <= '0; z_in_r <= '0;
      x18_r <= '0; y18_r <= '0; z18_r <= '0;
      s00a_valid <= 1'b0;
    end else if (norm_en) begin
      s00a_valid <= s00_axis_tvalid;
      if (s00_axis_tvalid) begin
        x_in_r <= in_x;
        y_in_r <= in_y;
        z_in_r <= in_z;
        x18_r <= rnd_s18(in_x);
        y18_r <= rnd_s18(in_y);
        z18_r <= rnd_s18(in_z);
      end
    end
  end

  /// STAGE 00b: compute x^2, y^2, z^2 from registered inputs.
  /// Driving from a registered source lets Vivado infer DSP PREG, cutting the
  /// external-input → y2_reg carry chain that was causing the -4.3 ns violation.
  /// Squares of real numbers are always non-negative in Q2.62, so no sign check needed.
  logic        [63:0] x2, y2, z2;        // magnitude squared Q2.62 (unsigned)
  logic        [31:0] x00, y00, z00;     // registered input copy for downstream alignment
  logic              dims_sq_valid;
  // Quantize the square operands to 18-bit (Q1.17 = top 18 bits of the Q1.31
  // input). An 18x18 multiply fits a SINGLE DSP48E1 (25x18), which Vivado fully
  // pipelines internally (AREG/MREG/PREG) -- no fabric cascade -- so it closes
  // timing AND drops from 4 DSPs to 1 per square. The Q2.34 product is shifted
  // back up to Q2.62 so the downstream lensq/shift logic is unchanged. 17-bit
  // magnitude precision is far inside the 0.065 sampling tolerance.
  // ---------------------------------------------------------------------------
  // Round-half-up operand narrowing.
  //
  // Plain bit-slicing truncates toward -inf, so the quantisation error is not
  // zero-mean: measured on the full pipeline it showed up as a systematic bias
  // (mean_signed z = -1.4e-05, ~97% of total error) that Monte Carlo averaging
  // will NOT remove, unlike random noise. Adding the MSB of the discarded field
  // makes the error zero-mean. The cost is an increment on the narrowed value,
  // not a full-width add, which keeps the DSP input path cheap. The equality
  // guard stops the increment overflowing the narrowed width at top of range.
  // ---------------------------------------------------------------------------
  function automatic logic signed [17:0] rnd_s18(input logic signed [31:0] a);
    logic signed [17:0] t;
    begin
      t = $signed(a[31:14]);
      rnd_s18 = (a[13] && t != 18'sh1FFFF) ? t + 18'sh00001 : t;
    end
  endfunction

  function automatic logic signed [24:0] rnd_u25(input logic [31:0] u);
    logic [23:0] t;
    begin
      t = u[31:8];
      rnd_u25 = $signed({1'b0, ((u[7] && t != 24'hFFFFFF) ? t + 24'h000001 : t)});
    end
  endfunction

  wire signed [35:0] x18_sq = x18_r * x18_r; // Q2.34, single DSP (registered operand)
  wire signed [35:0] y18_sq = y18_r * y18_r;
  wire signed [35:0] z18_sq = z18_r * z18_r;
  wire signed [63:0] x2_w = {x18_sq, 28'b0}; // Q2.34 -> Q2.62
  wire signed [63:0] y2_w = {y18_sq, 28'b0};
  wire signed [63:0] z2_w = {z18_sq, 28'b0};

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      x00 <= '0; y00 <= '0; z00 <= '0;
      x2 <= '0; y2 <= '0; z2 <= '0;
      dims_sq_valid <= 1'b0;
    end else if (norm_en) begin
      dims_sq_valid <= s00a_valid;
      if (s00a_valid) begin
        x00 <= x_in_r; y00 <= y_in_r; z00 <= z_in_r;
        x2 <= $unsigned(x2_w);
        y2 <= $unsigned(y2_w);
        z2 <= $unsigned(z2_w);
      end
    end
  end

  /// STAGE 01a0: partial sum x^2 + y^2. Split from the 3-way sum so the two
  /// 64-bit carry chains no longer chain within one cycle (this was the -0.827
  /// ns lensq critical path). z^2 and the x/y/z passthrough are delayed one
  /// stage to stay aligned.
  logic [64:0]        xy2_sum;
  logic [63:0]        z2_d;
  logic signed [31:0] x000, y000, z000;
  logic               sq_valid_0;

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      sq_valid_0 <= 1'b0;
      xy2_sum <= '0; z2_d <= '0;
      x000 <= '0; y000 <= '0; z000 <= '0;
    end else if (norm_en) begin
      sq_valid_0 <= dims_sq_valid;
      if (dims_sq_valid) begin
        xy2_sum <= {1'b0, x2} + {1'b0, y2};
        z2_d    <= z2;
        x000 <= x00; y000 <= y00; z000 <= z00;
      end
    end
  end

  /// STAGE 01a: final sum (+ z^2)
  logic [65:0] lensq_q2_62;
  logic signed [31:0] x01a, y01a, z01a;
  logic              lensq_valid_a;

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      lensq_valid_a <= 1'b0;
      lensq_q2_62   <= '0;
      x01a <= 0;
      y01a <= 0;
      z01a <= 0;
    end else if (norm_en) begin
      lensq_valid_a <= sq_valid_0;
      if (sq_valid_0) begin
        lensq_q2_62 <= {1'b0, xy2_sum} + {2'b0, z2_d};
        x01a <= x000;
        y01a <= y000;
        z01a <= z000;
      end
    end
  end

  /// MSB Index and Shift logic on registered sum lensq_q2_62
  logic [3:0] lensq_shift;
  logic [6:0] msb_idx;
  always_comb begin
    msb_idx = '0;
    for (int i = 0; i <= 65; i=i+1) begin
      if (lensq_q2_62[i])
        msb_idx = i[6:0];
    end
    if (msb_idx <= 61) begin
      lensq_shift = 0;
    end else begin
      logic [6:0] d;
      d = msb_idx - 7'd61;
      lensq_shift = (d[0]) ? ( (d>>1) + 1 ) : (d>>1);
    end
  end

  logic [65:0] lensq_q2_62_shifted;
  logic [31:0] lensq_q0_32_shifted;
  assign lensq_q2_62_shifted = lensq_q2_62 >> (lensq_shift << 1);
  assign lensq_q0_32_shifted = lensq_q2_62_shifted[61:30];

  /// STAGE 01b: register shift and shifted lensq
  logic signed [31:0] x01, y01, z01;
  logic [3:0] lensq_shift_reg;
  logic [31:0] lensq_u32;
  logic lensq_valid;

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      lensq_valid <= 1'b0;
      lensq_shift_reg <= '0;
      lensq_u32 <= '0;
      x01 <= 0;
      y01 <= 0;
      z01 <= 0;
    end else if (norm_en) begin
      lensq_valid <= lensq_valid_a;
      if (lensq_valid_a) begin
        lensq_shift_reg <= lensq_shift;
        lensq_u32 <= lensq_q0_32_shifted;
        x01 <= x01a;
        y01 <= y01a;
        z01 <= z01a;
      end
    end
  end

  /// INSTANTIATE INV_SQRT MODULE
  /// input: UQ0.32 output: Q7.25
  
  logic [31:0] lensq_u32_clamped;
  assign lensq_u32_clamped = (lensq_u32 < LENSQ_MIN_UQ0_32) ? LENSQ_MIN_UQ0_32 : lensq_u32;
  logic [31:0] inv_len_q7_25;
  wire         [3:0] delayed_shift;
  wire signed [31:0] delayed_x;
  wire signed [31:0] delayed_y;
  wire signed [31:0] delayed_z;

  axis_fixed_inv_sqrt_nodsp # (
    .FRAC_BITS(FRAC_BITS),
    .ADDR_BITS(14)
  ) u_inv_sqrt (
    .s00_axis_aclk(s00_axis_aclk),
    .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(1'b0),
    .s00_axis_tvalid(lensq_valid),
    .s00_axis_tdata(lensq_u32_clamped),
    .s00_axis_tstrb('1),
    .s00_axis_tready(inv_in_ready),

    .s00_axis_user_x(x01),
    .s00_axis_user_y(y01),
    .s00_axis_user_z(z01),
    .s00_axis_user_shift(lensq_shift_reg),

    .m00_axis_aclk(s00_axis_aclk),
    .m00_axis_aresetn(s00_axis_aresetn),
    .m00_axis_tlast(),
    .m00_axis_tvalid(inv_out_valid),
    .m00_axis_tdata(inv_len_q7_25),
    .m00_axis_tstrb(),
    .m00_axis_tready(pipe_en),

    .m00_axis_user_x(delayed_x),
    .m00_axis_user_y(delayed_y),
    .m00_axis_user_z(delayed_z),
    .m00_axis_user_shift(delayed_shift)
  );

  /// FINAL MULTIPLY
  /// (Q1.32 signed * Q7.25 unsigned) -> Q8.56 signed
  /// Shift right by 25 to get Q1.31, normalized vec should stay within [-1,1]
  /// Saturate if necessary
  localparam logic signed [31:0] MAX_Q1_31 = 32'sh7FFF_FFFF;
  localparam logic signed [31:0] MIN_Q1_31 = 32'sh8000_0000;

  function automatic logic signed [31:0] saturate_q1_31(input logic signed [63:0] val);
    begin
      if (val > $signed(64'sh0000_0000_7FFF_FFFF)) saturate_q1_31 = MAX_Q1_31;
      else if (val < $signed(64'shFFFF_FFFF_8000_0000)) saturate_q1_31 = MIN_Q1_31;
      else saturate_q1_31 = val[31:0];
    end
  endfunction

  function automatic logic signed [63:0] mul_q131_q725_to_q856(
    input logic signed [31:0] q131,  // Q1.31
    input logic        [31:0] q725   // Q7.25
  );
    logic signed [17:0] q131_18;
    logic signed [24:0] q725_25;
    logic signed [42:0] prod_q8_34;
    begin
      q131_18 = rnd_s18(q131);
      q725_25 = rnd_u25(q725);
      prod_q8_34 = q131_18 * q725_25;
      mul_q131_q725_to_q856 = 64'(prod_q8_34) <<< 22;
    end
  endfunction

  function automatic logic signed [31:0] scale_q856_to_q131(
    input logic signed [63:0] prod_q8_56,
    input logic         [3:0] shift
  );
    begin
      scale_q856_to_q131 = saturate_q1_31(prod_q8_56 >>> (25-2+shift));
    end
  endfunction

  logic s3a_valid, s3b_valid;
  logic [3:0] s3a_shift;
  logic signed [63:0] s3a_x_q856, s3a_y_q856, s3a_z_q856;
  logic signed [31:0] s3b_x_q131, s3b_y_q131, s3b_z_q131;

  wire signed [63:0] s3a_x_q856_w = mul_q131_q725_to_q856(delayed_x, inv_len_q7_25);
  wire signed [63:0] s3a_y_q856_w = mul_q131_q725_to_q856(delayed_y, inv_len_q7_25);
  wire signed [63:0] s3a_z_q856_w = mul_q131_q725_to_q856(delayed_z, inv_len_q7_25);

  wire signed [31:0] s3b_x_q131_w = scale_q856_to_q131(s3a_x_q856, s3a_shift);
  wire signed [31:0] s3b_y_q131_w = scale_q856_to_q131(s3a_y_q856, s3a_shift);
  wire signed [31:0] s3b_z_q131_w = scale_q856_to_q131(s3a_z_q856, s3a_shift);

  /// OUTPUT REGISTERS
  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      s3a_valid <= 1'b0;
      s3a_shift <= '0;
      s3a_x_q856 <= '0;
      s3a_y_q856 <= '0;
      s3a_z_q856 <= '0;
      s3b_valid <= 1'b0;
      s3b_x_q131 <= '0;
      s3b_y_q131 <= '0;
      s3b_z_q131 <= '0;
      m00_axis_tvalid <= 1'b0;
      m00_axis_tdata <= '0;
      // m00_axis_tlast <= 1'b0;
      // m00_axis_tstrb <= '1;
    end else if (pipe_en) begin
      s3a_valid <= inv_out_valid;
      if (inv_out_valid) begin
        s3a_shift <= delayed_shift;
        s3a_x_q856 <= s3a_x_q856_w;
        s3a_y_q856 <= s3a_y_q856_w;
        s3a_z_q856 <= s3a_z_q856_w;
      end

      s3b_valid <= s3a_valid;
      if (s3a_valid) begin
        s3b_x_q131 <= s3b_x_q131_w;
        s3b_y_q131 <= s3b_y_q131_w;
        s3b_z_q131 <= s3b_z_q131_w;
      end

      m00_axis_tvalid <= s3b_valid;
      if (s3b_valid) begin
        m00_axis_tdata <= {32'b0, s3b_z_q131, s3b_y_q131, s3b_x_q131};
        // m00_axis_tlast <= 1'b0;
        // m00_axis_tstrb <= '1;
      end
    end
  end

endmodule

`default_nettype wire
