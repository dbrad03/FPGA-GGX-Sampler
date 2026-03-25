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

  localparam integer INV_SQRT_LATENCY = 9; // +1 for axis_fixed_inv_sqrt input register stage
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
  logic              s00a_valid;

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      x_in_r <= '0; y_in_r <= '0; z_in_r <= '0;
      s00a_valid <= 1'b0;
    end else if (norm_en) begin
      s00a_valid <= s00_axis_tvalid;
      if (s00_axis_tvalid) begin
        x_in_r <= in_x;
        y_in_r <= in_y;
        z_in_r <= in_z;
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
  wire signed [63:0] x_in_r_wide = {{32{x_in_r[31]}}, x_in_r};
  wire signed [63:0] y_in_r_wide = {{32{y_in_r[31]}}, y_in_r};
  wire signed [63:0] z_in_r_wide = {{32{z_in_r[31]}}, z_in_r};
  wire signed [63:0] x2_w = x_in_r_wide * x_in_r_wide;
  wire signed [63:0] y2_w = y_in_r_wide * y_in_r_wide;
  wire signed [63:0] z2_w = z_in_r_wide * z_in_r_wide;

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

  /// STAGE 01: conpute x^2 + y^2 + z^2-> UQ0.32
  logic [65:0] lensq_q2_62; // sum with headroom
  // logic [31:0] lensq_q0_32;
  assign lensq_q2_62 = {2'b0, x2} + {2'b0, y2} + {2'b0, z2};
  
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
      // ceil( (msb_idx - 61) / 2 )
      logic [6:0] d;
      d = msb_idx - 7'd61;
      lensq_shift = (d[0]) ? ( (d>>1) + 1 ) : (d>>1);
    end
  end
  logic [65:0] lensq_q2_62_shifted;
  logic [31:0] lensq_q0_32_shifted;
  assign lensq_q2_62_shifted = lensq_q2_62 >> (lensq_shift << 1);
  assign lensq_q0_32_shifted = lensq_q2_62_shifted[61:30]; // should be < 1.0


  logic signed [31:0] x01, y01, z01;
  logic [3:0] lensq_shift_reg;
  logic [31:0] lensq_u32;
  logic lensq_valid;

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      lensq_valid <= 1'b0;
      x01 <= 0;
      y01 <= 0;
      z01 <= 0;
    end else if (norm_en) begin
      lensq_valid <= dims_sq_valid;
      if (dims_sq_valid) begin
        lensq_shift_reg <= lensq_shift;
        lensq_u32 <= lensq_q0_32_shifted;
        x01 <= x00;
        y01 <= y00;
        z01 <= z00;
      end
    end
  end

  /// INSTANTIATE INV_SQRT MODULE
  /// input: UQ0.32 output: Q7.25
  
  logic [31:0] lensq_u32_clamped;
  assign lensq_u32_clamped = (lensq_u32 < LENSQ_MIN_UQ0_32) ? LENSQ_MIN_UQ0_32 : lensq_u32;
  logic [31:0] inv_len_q7_25;

  axis_fixed_inv_sqrt # (
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

    .m00_axis_aclk(s00_axis_aclk),
    .m00_axis_aresetn(s00_axis_aresetn),
    .m00_axis_tlast(),
    .m00_axis_tvalid(inv_out_valid),
    .m00_axis_tdata(inv_len_q7_25),
    .m00_axis_tstrb(),
    .m00_axis_tready(pipe_en)
  );

  /// DELAY INPUTS TO MATCH INV_SQRT LATENCY
  
  logic         [3:0] delay_shift [0:INV_SQRT_LATENCY-1];
  logic signed [31:0] delay_x     [0:INV_SQRT_LATENCY-1];
  logic signed [31:0] delay_y     [0:INV_SQRT_LATENCY-1];
  logic signed [31:0] delay_z     [0:INV_SQRT_LATENCY-1];

  integer k;
  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      for (k=0; k<INV_SQRT_LATENCY; k=k+1) begin
        delay_shift[k] <= 0;
        delay_x[k] <= 0;
        delay_y[k] <= 0;
        delay_z[k] <= 0;
      end
    end else if (pipe_en) begin
      if (lensq_valid && inv_in_ready) begin
        delay_shift[0] <= lensq_shift_reg;
        delay_x[0] <= x01;
        delay_y[0] <= y01;
        delay_z[0] <= z01;
      end else begin
        delay_shift[0] <= '0;
        delay_x[0] <= '0;
        delay_y[0] <= '0;
        delay_z[0] <= '0;
      end

      for (k=1; k<INV_SQRT_LATENCY; k=k+1) begin
          delay_shift[k] <= delay_shift[k-1];
          delay_x[k] <= delay_x[k-1];
          delay_y[k] <= delay_y[k-1];
          delay_z[k] <= delay_z[k-1];
      end
    end
  end

  wire         [3:0] delayed_shift = delay_shift[INV_SQRT_LATENCY-1];
  wire signed [31:0] delayed_x     = delay_x[INV_SQRT_LATENCY-1];
  wire signed [31:0] delayed_y     = delay_y[INV_SQRT_LATENCY-1];
  wire signed [31:0] delayed_z     = delay_z[INV_SQRT_LATENCY-1];

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
    logic signed [32:0] inv_scaled_q725;
    logic signed [63:0] prod_q8_56;
    begin
      inv_scaled_q725 = $signed({1'b0, q725});
      prod_q8_56 = $signed(q131) * inv_scaled_q725; // Q8.56 + extra bits
      mul_q131_q725_to_q856 = prod_q8_56;
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
