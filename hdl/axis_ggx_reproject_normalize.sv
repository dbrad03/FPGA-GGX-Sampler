`timescale 1ns / 1ps
`default_nettype none

module axis_ggx_reproject_normalize #
  (
    parameter integer C_S00_AXIS_TDATA_WIDTH = 352,
    parameter integer C_M00_AXIS_TDATA_WIDTH = 96,
    parameter integer FRAC_BITS              = 32
  )
  (
    // Ports of Axi Slave Bus Interface S00_AXIS
    input  wire s00_axis_aclk,
    input  wire s00_axis_aresetn,
    input  wire s00_axis_tlast,
    input  wire s00_axis_tvalid,
    // {Vh(96), T2(96), T1(96), {t2,t1}(64)}
    input  wire [C_S00_AXIS_TDATA_WIDTH-1:0] s00_axis_tdata,
    input  wire [(C_S00_AXIS_TDATA_WIDTH/8)-1:0] s00_axis_tstrb,
    output logic s00_axis_tready,

    // Ports of Axi Master Bus Interface M00_AXIS
    input  wire m00_axis_aclk,
    input  wire m00_axis_aresetn,
    input  wire m00_axis_tready,
    output logic m00_axis_tvalid,
    output logic m00_axis_tlast,
    // {hz, hy, hx}
    output logic signed [C_M00_AXIS_TDATA_WIDTH-1:0] m00_axis_tdata,
    output logic [(C_M00_AXIS_TDATA_WIDTH/8)-1:0] m00_axis_tstrb
  );

  localparam int META0_DEPTH = 128;
  localparam int META0_AW    = $clog2(META0_DEPTH);
  localparam int META1_DEPTH = 32;
  localparam int META1_AW    = $clog2(META1_DEPTH);

  localparam logic [31:0] ONE_UQ0_32 = 32'hFFFF_FFFF;
  localparam logic signed [31:0] ONE_Q1 = 32'sh7FFF_FFFF;
  localparam logic signed [31:0] NEG_ONE_Q1 = 32'sh8000_0000;

  function automatic logic signed [31:0] satq131(input logic signed [63:0] val);
    begin
      if (val > $signed(64'sh0000_0000_7FFF_FFFF)) satq131 = ONE_Q1;
      else if (val < $signed(64'shFFFF_FFFF_8000_0000)) satq131 = NEG_ONE_Q1;
      else satq131 = val[31:0];
    end
  endfunction

  function automatic logic signed [63:0] mul_q131_q131_to_q262(
    input logic signed [31:0] q131_a,
    input logic signed [31:0] q131_b
  );
    begin
      mul_q131_q131_to_q262 = $signed(q131_a) * $signed(q131_b);
    end
  endfunction

  function automatic logic signed [31:0] scale_q262_to_q131(
    input logic signed [63:0] prod_q262
  );
    begin
      // Q2.62 product: bits[63:62] are both sign bits; they must agree for no overflow.
      // Only overflow: [63:62]==2'b01 (INT_MIN*INT_MIN → positive clamp to ONE_Q1).
      // Negative overflow cannot occur since both inputs are in [-1, 1).
      if (prod_q262[63:62] == 2'b01)
        scale_q262_to_q131 = ONE_Q1;
      else
        scale_q262_to_q131 = $signed(prod_q262[62:31]);
    end
  endfunction

  function automatic logic signed [31:0] uq032_to_q131(input logic [31:0] uq032);
    logic [32:0] uq_plus_one;
    logic [31:0] q131_u;
    begin
      uq_plus_one = {1'b0, uq032} + 33'd1;
      q131_u = uq_plus_one[32:1];
      uq032_to_q131 = q131_u[31] ? ONE_Q1 : $signed(q131_u);
    end
  endfunction

  // --------------------------------------------------------------------------
  // Stage A0: Capture input sample
  // Stage A1A: Register raw signed t1^2 and t2^2
  // Stage A1: Register unsigned t1^2 and t2^2
  // Stage A2: Register sqrt argument t3 = max(0, 1 - t1^2 - t2^2)
  // --------------------------------------------------------------------------
  logic a0_valid, a0_last;
  logic [C_S00_AXIS_TDATA_WIDTH-1:0] a0_data;
  logic a1a_valid, a1a_last;
  logic [C_S00_AXIS_TDATA_WIDTH-1:0] a1a_data;
  logic signed [63:0] a1a_t1_sq_q262_s, a1a_t2_sq_q262_s;
  logic a1_valid, a1_last;
  logic [C_S00_AXIS_TDATA_WIDTH-1:0] a1_data;
  logic [63:0] a1_t1_sq_q262_u, a1_t2_sq_q262_u;
  logic a2_valid, a2_last;
  logic [C_S00_AXIS_TDATA_WIDTH-1:0] a2_data;
  logic [31:0] a2_t3_arg_uq032;

  wire a0_to_a1;
  wire a0_ready;
  wire a0_to_a1a;
  wire a1a_ready;
  wire a1a_to_a1;
  wire a1_ready;
  wire a1_to_a2;
  wire a2_to_sqrt;
  wire a2_ready;
  wire s00_axis_fire = s00_axis_tvalid && s00_axis_tready;

  wire signed [31:0] a0_t1_q131 = $signed(a0_data[31:0]);
  wire signed [31:0] a0_t2_q131 = $signed(a0_data[63:32]);
  wire signed [63:0] a1a_t1_sq_q262_s_w = $signed(a0_t1_q131) * $signed(a0_t1_q131);
  wire signed [63:0] a1a_t2_sq_q262_s_w = $signed(a0_t2_q131) * $signed(a0_t2_q131);
  wire [63:0] a1_t1_sq_q262_u_w = a1a_t1_sq_q262_s[63] ? 64'd0 : $unsigned(a1a_t1_sq_q262_s);
  wire [63:0] a1_t2_sq_q262_u_w = a1a_t2_sq_q262_s[63] ? 64'd0 : $unsigned(a1a_t2_sq_q262_s);

  wire [64:0] a2_t12_sq_sum_q262_w = {1'b0, a1_t1_sq_q262_u} + {1'b0, a1_t2_sq_q262_u};
  wire a2_sum_ge_one_w = |a2_t12_sq_sum_q262_w[64:62];
  wire [31:0] a2_t12_sq_sum_uq032_w = a2_t12_sq_sum_q262_w[61:30];
  wire [31:0] a2_t3_arg_uq032_w = a2_sum_ge_one_w ? 32'd0 : (ONE_UQ0_32 - a2_t12_sq_sum_uq032_w);

  wire sqrt_in_ready;
  wire sqrt_in_fire;
  wire sqrt_out_valid;
  wire [31:0] sqrt_out_uq032;
  logic sqrt_out_ready;

  assign a0_to_a1a = a0_valid && a1a_ready;
  assign a0_ready = !a0_valid || a0_to_a1a;
  assign a1a_to_a1 = a1a_valid && a1_ready;
  assign a1a_ready = !a1a_valid || a1a_to_a1;
  assign a1_to_a2 = a1_valid && a2_ready;
  assign a1_ready = !a1_valid || a1_to_a2;
  assign a2_to_sqrt = a2_valid && sqrt_in_ready;
  assign a2_ready = !a2_valid || a2_to_sqrt;
  assign s00_axis_tready = a0_ready;
  assign sqrt_in_fire = a2_to_sqrt;

  axis_fixed_sqrt #(
    .FRAC_BITS(FRAC_BITS)
  ) u_sqrt_t3 (
    .s00_axis_aclk(s00_axis_aclk),
    .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(1'b0),
    .s00_axis_tvalid(a2_valid),
    .s00_axis_tdata(a2_t3_arg_uq032),
    .s00_axis_tstrb('1),
    .s00_axis_tready(sqrt_in_ready),

    .m00_axis_aclk(s00_axis_aclk),
    .m00_axis_aresetn(s00_axis_aresetn),
    .m00_axis_tready(sqrt_out_ready),
    .m00_axis_tvalid(sqrt_out_valid),
    .m00_axis_tlast(),
    .m00_axis_tdata(sqrt_out_uq032), // here
    .m00_axis_tstrb()
  );

  wire sqrt_out_fire;

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      a0_valid <= 1'b0;
      a0_last <= 1'b0;
      a0_data <= '0;
    end else begin
      if (s00_axis_fire) begin
        a0_valid <= 1'b1;
        a0_last <= s00_axis_tlast;
        a0_data <= s00_axis_tdata;
      end else if (a0_to_a1a) begin
        a0_valid <= 1'b0;
      end
    end
  end

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      a1a_valid <= 1'b0;
      a1a_last <= 1'b0;
      a1a_data <= '0;
      a1a_t1_sq_q262_s <= '0;
      a1a_t2_sq_q262_s <= '0;
    end else begin
      if (a0_to_a1a) begin
        a1a_valid <= 1'b1;
        a1a_last <= a0_last;
        a1a_data <= a0_data;
        a1a_t1_sq_q262_s <= a1a_t1_sq_q262_s_w;
        a1a_t2_sq_q262_s <= a1a_t2_sq_q262_s_w;
      end else if (a1a_to_a1) begin
        a1a_valid <= 1'b0;
      end
    end
  end

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      a1_valid <= 1'b0;
      a1_last <= 1'b0;
      a1_data <= '0;
      a1_t1_sq_q262_u <= '0;
      a1_t2_sq_q262_u <= '0;
    end else begin
      if (a1a_to_a1) begin
        a1_valid <= 1'b1;
        a1_last <= a1a_last;
        a1_data <= a1a_data;
        a1_t1_sq_q262_u <= a1_t1_sq_q262_u_w;
        a1_t2_sq_q262_u <= a1_t2_sq_q262_u_w;
      end else if (a1_to_a2) begin
        a1_valid <= 1'b0;
      end
    end
  end

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      a2_valid <= 1'b0;
      a2_last <= 1'b0;
      a2_data <= '0;
      a2_t3_arg_uq032 <= '0;
    end else begin
      if (a1_to_a2) begin
        a2_valid <= 1'b1;
        a2_last <= a1_last;
        a2_data <= a1_data;
        a2_t3_arg_uq032 <= a2_t3_arg_uq032_w;
      end else if (a2_to_sqrt) begin
        a2_valid <= 1'b0;
      end
    end
  end

  // Metadata alignment for basis/scalars and TLAST through sqrt stage.
  logic [C_S00_AXIS_TDATA_WIDTH-1:0] meta0_data [0:META0_DEPTH-1];
  logic meta0_last [0:META0_DEPTH-1];
  logic [META0_AW-1:0] meta0_wr_ptr, meta0_rd_ptr;
  logic [META0_AW:0] meta0_count;

  wire [C_S00_AXIS_TDATA_WIDTH-1:0] sqrt_aligned_data = meta0_data[meta0_rd_ptr];
  wire sqrt_aligned_last = meta0_last[meta0_rd_ptr];

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      meta0_wr_ptr <= '0;
      meta0_rd_ptr <= '0;
      meta0_count <= '0;
    end else begin
      if (sqrt_in_fire) begin
        meta0_data[meta0_wr_ptr] <= a2_data;
        meta0_last[meta0_wr_ptr] <= a2_last;
        meta0_wr_ptr <= meta0_wr_ptr + 1'b1;
      end
      if (sqrt_out_fire) begin
        meta0_rd_ptr <= meta0_rd_ptr + 1'b1;
      end
      case ({sqrt_in_fire, sqrt_out_fire})
        2'b10: meta0_count <= meta0_count + 1'b1;
        2'b01: meta0_count <= meta0_count - 1'b1;
        default: begin end
      endcase
    end
  end

  // --------------------------------------------------------------------------
  // Stage B0: Register sqrt-aligned inputs to cut the sqrt->multiply path
  // Stage B1A: Raw Q2.62 reprojection products
  // Stage B1: Clamp/scale products back to Q1.31
  // Stage B2: Sum + saturate to form unnormalized H
  // --------------------------------------------------------------------------
  logic b0_valid, b0_last;
  logic signed [95:0] b0_t1, b0_t2, b0_vh;
  logic signed [31:0] b0_t1_s, b0_t2_s, b0_t3_s;

  logic b1a_valid, b1a_last;
  logic signed [63:0] b1a_hx_t1_q262, b1a_hx_t2_q262, b1a_hx_vh_q262;
  logic signed [63:0] b1a_hy_t1_q262, b1a_hy_t2_q262, b1a_hy_vh_q262;
  logic signed [63:0] b1a_hz_t1_q262, b1a_hz_t2_q262, b1a_hz_vh_q262;

  logic b1_valid, b1_last;
  logic signed [31:0] b1_hx_t1, b1_hx_t2, b1_hx_vh;
  logic signed [31:0] b1_hy_t1, b1_hy_t2, b1_hy_vh;
  logic signed [31:0] b1_hz_t1, b1_hz_t2, b1_hz_vh;

  logic b2_valid, b2_last;
  logic signed [31:0] b2_hx, b2_hy, b2_hz;

  wire norm_in_ready;
  wire b2_to_norm = b2_valid && norm_in_ready;
  wire b2_ready = !b2_valid || b2_to_norm;
  wire b1_to_b2 = b1_valid && b2_ready;
  wire b1_ready = !b1_valid || b1_to_b2;
  wire b1a_to_b1 = b1a_valid && b1_ready;
  wire b1a_ready = !b1a_valid || b1a_to_b1;
  wire b0_to_b1a = b0_valid && b1a_ready;
  wire b0_ready = !b0_valid || b0_to_b1a;
  assign sqrt_out_fire = sqrt_out_valid && b0_ready;
  assign sqrt_out_ready = b0_ready;

  wire signed [63:0] b1a_hx_t1_q262_w = mul_q131_q131_to_q262($signed(b0_t1[31:0]), b0_t1_s);
  wire signed [63:0] b1a_hx_t2_q262_w = mul_q131_q131_to_q262($signed(b0_t2[31:0]), b0_t2_s);
  wire signed [63:0] b1a_hx_vh_q262_w = mul_q131_q131_to_q262($signed(b0_vh[31:0]), b0_t3_s);
  wire signed [63:0] b1a_hy_t1_q262_w = mul_q131_q131_to_q262($signed(b0_t1[63:32]), b0_t1_s);
  wire signed [63:0] b1a_hy_t2_q262_w = mul_q131_q131_to_q262($signed(b0_t2[63:32]), b0_t2_s);
  wire signed [63:0] b1a_hy_vh_q262_w = mul_q131_q131_to_q262($signed(b0_vh[63:32]), b0_t3_s);
  wire signed [63:0] b1a_hz_t1_q262_w = mul_q131_q131_to_q262($signed(b0_t1[95:64]), b0_t1_s);
  wire signed [63:0] b1a_hz_t2_q262_w = mul_q131_q131_to_q262($signed(b0_t2[95:64]), b0_t2_s);
  wire signed [63:0] b1a_hz_vh_q262_w = mul_q131_q131_to_q262($signed(b0_vh[95:64]), b0_t3_s);

  wire signed [31:0] b1_hx_t1_w = scale_q262_to_q131(b1a_hx_t1_q262);
  wire signed [31:0] b1_hx_t2_w = scale_q262_to_q131(b1a_hx_t2_q262);
  wire signed [31:0] b1_hx_vh_w = scale_q262_to_q131(b1a_hx_vh_q262);
  wire signed [31:0] b1_hy_t1_w = scale_q262_to_q131(b1a_hy_t1_q262);
  wire signed [31:0] b1_hy_t2_w = scale_q262_to_q131(b1a_hy_t2_q262);
  wire signed [31:0] b1_hy_vh_w = scale_q262_to_q131(b1a_hy_vh_q262);
  wire signed [31:0] b1_hz_t1_w = scale_q262_to_q131(b1a_hz_t1_q262);
  wire signed [31:0] b1_hz_t2_w = scale_q262_to_q131(b1a_hz_t2_q262);
  wire signed [31:0] b1_hz_vh_w = scale_q262_to_q131(b1a_hz_vh_q262);

  wire signed [63:0] b2_hx_sum_w = $signed({{32{b1_hx_t1[31]}}, b1_hx_t1}) +
                                   $signed({{32{b1_hx_t2[31]}}, b1_hx_t2}) +
                                   $signed({{32{b1_hx_vh[31]}}, b1_hx_vh});
  wire signed [63:0] b2_hy_sum_w = $signed({{32{b1_hy_t1[31]}}, b1_hy_t1}) +
                                   $signed({{32{b1_hy_t2[31]}}, b1_hy_t2}) +
                                   $signed({{32{b1_hy_vh[31]}}, b1_hy_vh});
  wire signed [63:0] b2_hz_sum_w = $signed({{32{b1_hz_t1[31]}}, b1_hz_t1}) +
                                   $signed({{32{b1_hz_t2[31]}}, b1_hz_t2}) +
                                   $signed({{32{b1_hz_vh[31]}}, b1_hz_vh});

  wire signed [31:0] b2_hx_w = satq131(b2_hx_sum_w);
  wire signed [31:0] b2_hy_w = satq131(b2_hy_sum_w);
  wire signed [31:0] b2_hz_w = satq131(b2_hz_sum_w);

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      b0_valid <= 1'b0;
      b0_last <= 1'b0;
      b0_t1 <= '0;
      b0_t2 <= '0;
      b0_vh <= '0;
      b0_t1_s <= '0;
      b0_t2_s <= '0;
      b0_t3_s <= '0;
    end else begin
      if (sqrt_out_fire) begin
        b0_valid <= 1'b1;
        b0_last <= sqrt_aligned_last;
        b0_t1 <= $signed(sqrt_aligned_data[159:64]);
        b0_t2 <= $signed(sqrt_aligned_data[255:160]);
        b0_vh <= $signed(sqrt_aligned_data[351:256]);
        b0_t1_s <= $signed(sqrt_aligned_data[31:0]);
        b0_t2_s <= $signed(sqrt_aligned_data[63:32]);
        b0_t3_s <= uq032_to_q131(sqrt_out_uq032);
      end else if (b0_to_b1a) begin
        b0_valid <= 1'b0;
      end
    end
  end

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      b1a_valid <= 1'b0;
      b1a_last <= 1'b0;
      b1a_hx_t1_q262 <= '0;
      b1a_hx_t2_q262 <= '0;
      b1a_hx_vh_q262 <= '0;
      b1a_hy_t1_q262 <= '0;
      b1a_hy_t2_q262 <= '0;
      b1a_hy_vh_q262 <= '0;
      b1a_hz_t1_q262 <= '0;
      b1a_hz_t2_q262 <= '0;
      b1a_hz_vh_q262 <= '0;
    end else begin
      if (b0_to_b1a) begin
        b1a_valid <= 1'b1;
        b1a_last <= b0_last;
        b1a_hx_t1_q262 <= b1a_hx_t1_q262_w;
        b1a_hx_t2_q262 <= b1a_hx_t2_q262_w;
        b1a_hx_vh_q262 <= b1a_hx_vh_q262_w;
        b1a_hy_t1_q262 <= b1a_hy_t1_q262_w;
        b1a_hy_t2_q262 <= b1a_hy_t2_q262_w;
        b1a_hy_vh_q262 <= b1a_hy_vh_q262_w;
        b1a_hz_t1_q262 <= b1a_hz_t1_q262_w;
        b1a_hz_t2_q262 <= b1a_hz_t2_q262_w;
        b1a_hz_vh_q262 <= b1a_hz_vh_q262_w;
      end else if (b1a_to_b1) begin
        b1a_valid <= 1'b0;
      end
    end
  end

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      b1_valid <= 1'b0;
      b1_last <= 1'b0;
      b1_hx_t1 <= '0;
      b1_hx_t2 <= '0;
      b1_hx_vh <= '0;
      b1_hy_t1 <= '0;
      b1_hy_t2 <= '0;
      b1_hy_vh <= '0;
      b1_hz_t1 <= '0;
      b1_hz_t2 <= '0;
      b1_hz_vh <= '0;
    end else begin
      if (b1a_to_b1) begin
        b1_valid <= 1'b1;
        b1_last <= b1a_last;
        b1_hx_t1 <= b1_hx_t1_w;
        b1_hx_t2 <= b1_hx_t2_w;
        b1_hx_vh <= b1_hx_vh_w;
        b1_hy_t1 <= b1_hy_t1_w;
        b1_hy_t2 <= b1_hy_t2_w;
        b1_hy_vh <= b1_hy_vh_w;
        b1_hz_t1 <= b1_hz_t1_w;
        b1_hz_t2 <= b1_hz_t2_w;
        b1_hz_vh <= b1_hz_vh_w;
      end else if (b1_to_b2) begin
        b1_valid <= 1'b0;
      end
    end
  end

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      b2_valid <= 1'b0;
      b2_last <= 1'b0;
      b2_hx <= '0;
      b2_hy <= '0;
      b2_hz <= '0;
    end else begin
      if (b1_to_b2) begin
        b2_valid <= 1'b1;
        b2_last <= b1_last;
        b2_hx <= b2_hx_w;
        b2_hy <= b2_hy_w;
        b2_hz <= b2_hz_w;
      end else if (b2_to_norm) begin
        b2_valid <= 1'b0;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Stage C: Normalize H
  // --------------------------------------------------------------------------
  wire norm_out_valid;
  wire [127:0] norm_out_data;
  wire out_pipe_en = m00_axis_tready || !m00_axis_tvalid;
  wire norm_out_fire = norm_out_valid && out_pipe_en;

  axis_fixed_norm3 u_norm_h (
    .s00_axis_aclk(s00_axis_aclk),
    .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(1'b0),
    .s00_axis_tvalid(b2_valid),
    .s00_axis_tdata({32'b0, b2_hz, b2_hy, b2_hx}),
    .s00_axis_tstrb('1),
    .s00_axis_tready(norm_in_ready),

    .m00_axis_aclk(s00_axis_aclk),
    .m00_axis_aresetn(s00_axis_aresetn),
    .m00_axis_tready(out_pipe_en),
    .m00_axis_tvalid(norm_out_valid),
    .m00_axis_tlast(),
    .m00_axis_tdata(norm_out_data),
    .m00_axis_tstrb()
  );

  // TLAST alignment through normalization stage.
  logic meta1_last [0:META1_DEPTH-1];
  logic [META1_AW-1:0] meta1_wr_ptr, meta1_rd_ptr;
  logic [META1_AW:0] meta1_count;
  wire norm_aligned_last = meta1_last[meta1_rd_ptr];

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      meta1_wr_ptr <= '0;
      meta1_rd_ptr <= '0;
      meta1_count <= '0;
    end else begin
      if (b2_to_norm) begin
        meta1_last[meta1_wr_ptr] <= b2_last;
        meta1_wr_ptr <= meta1_wr_ptr + 1'b1;
      end
      if (norm_out_fire) begin
        meta1_rd_ptr <= meta1_rd_ptr + 1'b1;
      end
      case ({b2_to_norm, norm_out_fire})
        2'b10: meta1_count <= meta1_count + 1'b1;
        2'b01: meta1_count <= meta1_count - 1'b1;
        default: begin end
      endcase
    end
  end

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      m00_axis_tvalid <= 1'b0;
      m00_axis_tlast <= 1'b0;
      m00_axis_tdata <= '0;
    end else if (out_pipe_en) begin
      m00_axis_tvalid <= norm_out_valid;
      if (norm_out_valid) begin
        m00_axis_tlast <= norm_aligned_last;
        m00_axis_tdata <= $signed(norm_out_data[95:0]);
      end
    end
  end

  assign m00_axis_tstrb = '1;

endmodule

`default_nettype wire
