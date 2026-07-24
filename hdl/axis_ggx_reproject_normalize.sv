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
  // norm3 wraps the ~93-cycle no-DSP inverse-sqrt, so its worst-case in-flight
  // occupancy far exceeds 32; META1 must be deeper than that latency or the
  // TLAST-tracking FIFO overflows (spurious TLAST). Sized to 128 with margin.
  localparam int META1_DEPTH = 128;
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

  // Narrow saturation on a 34-bit Q1.31 accumulator (the wide 64-bit satq131 is
  // unnecessary once the sum is width-reduced to 34 bits).
  function automatic logic signed [31:0] satq131_34(input logic signed [33:0] val);
    begin
      if (val > $signed(34'sh0_7FFF_FFFF)) satq131_34 = ONE_Q1;
      else if (val < $signed(34'sh3_8000_0000)) satq131_34 = NEG_ONE_Q1;
      else satq131_34 = val[31:0];
    end
  endfunction

  // Quantized to a single DSP48E1 (18b x 25b), same as sq_q131_to_q262 below.
  // A full 32x32 product needs a 4-DSP cascade whose A->PCOUT delay is 4.21 ns
  // with zero logic levels, so it cannot be pipelined below the 5 ns target --
  // it has to be narrowed instead. Q1.31 x Q1.31 -> Q2.62, low bits zero-filled.
  function automatic logic signed [63:0] mul_q131_q131_to_q262(
    input logic signed [31:0] q131_a,
    input logic signed [31:0] q131_b
  );
    logic signed [17:0] a_18;       // Q1.17
    logic signed [24:0] b_25;       // Q1.24
    logic signed [42:0] prod_q2_41; // Q2.41
    begin
      a_18       = q131_a[31:14];
      b_25       = q131_b[31:7];
      prod_q2_41 = a_18 * b_25;                       // Q1.17 * Q1.24 -> Q2.41
      mul_q131_q131_to_q262 = 64'(prod_q2_41) <<< 21; // Q2.41 -> Q2.62
    end
  endfunction

  // Quantized square: truncate the operand to 18b and 25b so the product fits
  // ONE DSP48E1 (auto AREG/MREG/PREG, no fabric cascade). Q1.31 -> Q2.62 with
  // the low bits zero-filled. Both truncations keep the operand's sign, so the
  // result stays non-negative exactly as a true square does.
  function automatic logic signed [63:0] sq_q131_to_q262(
    input logic signed [31:0] q131
  );
    logic signed [17:0] a_18;       // Q1.17
    logic signed [24:0] b_25;       // Q1.24
    logic signed [42:0] prod_q2_41; // Q2.41
    begin
      a_18       = q131[31:14];
      b_25       = q131[31:7];
      prod_q2_41 = a_18 * b_25;                 // Q1.17 * Q1.24 -> Q2.41
      sq_q131_to_q262 = 64'(prod_q2_41) <<< 21; // Q2.41 -> Q2.62
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
  // Stage A1B: register the t1^2+t2^2 sum so the wide add and the (1-sum)
  // subtract in A2 no longer chain in a single cycle.
  logic a1b_valid, a1b_last;
  logic [C_S00_AXIS_TDATA_WIDTH-1:0] a1b_data;
  logic        a1b_sum_ge_one;
  logic [31:0] a1b_sum_uq032;
  logic a2_valid, a2_last;
  logic [C_S00_AXIS_TDATA_WIDTH-1:0] a2_data;
  logic [31:0] a2_t3_arg_uq032;

  wire a0_to_a1;
  wire a0_ready;
  wire a0_to_a1a;
  wire a1a_ready;
  wire a1a_to_a1;
  wire a1_ready;
  wire a1_to_a1b;
  wire a1b_ready;
  wire a1b_to_a2;
  wire a2_to_sqrt;
  wire a2_ready;
  wire s00_axis_fire = s00_axis_tvalid && s00_axis_tready;

  wire signed [31:0] a0_t1_q131 = $signed(a0_data[31:0]);
  wire signed [31:0] a0_t2_q131 = $signed(a0_data[63:32]);
  wire signed [63:0] a1a_t1_sq_q262_s_w = sq_q131_to_q262(a0_t1_q131);
  wire signed [63:0] a1a_t2_sq_q262_s_w = sq_q131_to_q262(a0_t2_q131);
  // t1^2 and t2^2 are non-negative for any input (even (-1.0)^2 = +1.0), so the
  // old sign-bit clamp to zero was dead logic on a register reset path; drop it.
  wire [63:0] a1_t1_sq_q262_u_w = $unsigned(a1a_t1_sq_q262_s);
  wire [63:0] a1_t2_sq_q262_u_w = $unsigned(a1a_t2_sq_q262_s);

  // A1B combinational: sum of the two squares.
  wire [64:0] a1b_sum_q262_w   = {1'b0, a1_t1_sq_q262_u} + {1'b0, a1_t2_sq_q262_u};
  wire        a1b_sum_ge_one_w = |a1b_sum_q262_w[64:62];
  wire [31:0] a1b_sum_uq032_w  = a1b_sum_q262_w[61:30];

  // A2 combinational: 1 - sum (clamped to 0 when sum >= 1).
  wire [31:0] a2_t3_arg_uq032_w = a1b_sum_ge_one ? 32'd0 : (ONE_UQ0_32 - a1b_sum_uq032);

  wire sqrt_in_ready;
  wire sqrt_in_fire;
  wire sqrt_out_valid;
  wire [31:0] sqrt_out_uq032;
  logic sqrt_out_ready;

  // sqrt input skid FIFO: registers the ready path so the upstream A-stage
  // backpressure no longer depends combinationally on the sqrt's ready. The
  // FIFO carries the {metadata, t3_arg} pair, so the metadata written into
  // meta0 stays aligned with the t3_arg entering the sqrt.
  wire         a2_fifo_ready;
  wire         fifo_sqrt_valid;
  wire         fifo_sqrt_last;
  wire [383:0] fifo_sqrt_data;

  assign a0_to_a1a = a0_valid && a1a_ready;
  assign a0_ready = !a0_valid || a0_to_a1a;
  assign a1a_to_a1 = a1a_valid && a1_ready;
  assign a1a_ready = !a1a_valid || a1a_to_a1;
  assign a1_to_a1b = a1_valid && a1b_ready;
  assign a1_ready = !a1_valid || a1_to_a1b;
  assign a1b_to_a2 = a1b_valid && a2_ready;
  assign a1b_ready = !a1b_valid || a1b_to_a2;
  assign a2_to_sqrt = a2_valid && a2_fifo_ready;
  assign a2_ready = !a2_valid || a2_to_sqrt;
  assign s00_axis_tready = a0_ready;
  assign sqrt_in_fire = fifo_sqrt_valid && sqrt_in_ready;

  axis_fifo_2deep #(
    .DATA_WIDTH(384)
  ) u_sqrt_fifo (
    .clk(s00_axis_aclk),
    .resetn(s00_axis_aresetn),
    .s_axis_tvalid(a2_valid),
    .s_axis_tready(a2_fifo_ready),
    .s_axis_tdata({a2_data, a2_t3_arg_uq032}),
    .s_axis_tlast(a2_last),
    .m_axis_tvalid(fifo_sqrt_valid),
    .m_axis_tready(sqrt_in_ready),
    .m_axis_tdata(fifo_sqrt_data),
    .m_axis_tlast(fifo_sqrt_last)
  );

  axis_fixed_sqrt #(
    .FRAC_BITS(FRAC_BITS)
  ) u_sqrt_t3 (
    .s00_axis_aclk(s00_axis_aclk),
    .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(1'b0),
    .s00_axis_tvalid(fifo_sqrt_valid),
    .s00_axis_tdata(fifo_sqrt_data[31:0]),
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
      end else if (a1_to_a1b) begin
        a1_valid <= 1'b0;
      end
    end
  end

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      a1b_valid <= 1'b0;
      a1b_last <= 1'b0;
      a1b_data <= '0;
      a1b_sum_ge_one <= 1'b0;
      a1b_sum_uq032 <= '0;
    end else begin
      if (a1_to_a1b) begin
        a1b_valid <= 1'b1;
        a1b_last <= a1_last;
        a1b_data <= a1_data;
        a1b_sum_ge_one <= a1b_sum_ge_one_w;
        a1b_sum_uq032 <= a1b_sum_uq032_w;
      end else if (a1b_to_a2) begin
        a1b_valid <= 1'b0;
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
      if (a1b_to_a2) begin
        a2_valid <= 1'b1;
        a2_last <= a1b_last;
        a2_data <= a1b_data;
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
        meta0_data[meta0_wr_ptr] <= fifo_sqrt_data[383:32];
        meta0_last[meta0_wr_ptr] <= fifo_sqrt_last;
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
  // Stage B0:  Register sqrt-aligned inputs to cut the sqrt->multiply path
  // Stage B0P: Per-multiply operand registers (fanout-1) -> DSP AREG/BREG
  // Stage B1A: Raw Q2.62 reprojection products                -> DSP MREG
  // Stage B1P: Product pipeline copy                          -> DSP PREG
  // Stage B1:  Clamp/scale products back to Q1.31
  // Stage B1B: Partial sum (t1+t2), carry vh forward
  // Stage B2:  Final sum (+vh) + saturate to form unnormalized H
  //
  // The 9 reprojection products are signed 32x32 multiplies, each mapping to a
  // 2-slice DSP48E1 cascade. To close 200 MHz the cascade must be fully
  // pipelined (AREG/BREG -> MREG -> PREG). b0 stays in fabric (it registers the
  // combinational BRAM read); B0P then gives every multiply its OWN fanout-1
  // operand pair so Vivado can pack them as AREG/BREG, and B1A/B1P provide the
  // two product-register levels (MREG/PREG) the cascade needs.
  // --------------------------------------------------------------------------
  logic b0_valid, b0_last;
  (* keep = "true" *) logic signed [95:0] b0_t1, b0_t2, b0_vh;
  (* keep = "true" *) logic signed [31:0] b0_t1_s, b0_t2_s, b0_t3_s;

  logic b0p_valid, b0p_last;
  logic signed [31:0] b0p_hx_t1_a, b0p_hx_t2_a, b0p_hx_vh_a;
  logic signed [31:0] b0p_hy_t1_a, b0p_hy_t2_a, b0p_hy_vh_a;
  logic signed [31:0] b0p_hz_t1_a, b0p_hz_t2_a, b0p_hz_vh_a;
  logic signed [31:0] b0p_hx_t1_b, b0p_hx_t2_b, b0p_hx_vh_b;
  logic signed [31:0] b0p_hy_t1_b, b0p_hy_t2_b, b0p_hy_vh_b;
  logic signed [31:0] b0p_hz_t1_b, b0p_hz_t2_b, b0p_hz_vh_b;

  logic b1a_valid, b1a_last;
  logic signed [63:0] b1a_hx_t1_q262, b1a_hx_t2_q262, b1a_hx_vh_q262;
  logic signed [63:0] b1a_hy_t1_q262, b1a_hy_t2_q262, b1a_hy_vh_q262;
  logic signed [63:0] b1a_hz_t1_q262, b1a_hz_t2_q262, b1a_hz_vh_q262;

  logic b1p_valid, b1p_last;
  logic signed [63:0] b1p_hx_t1_q262, b1p_hx_t2_q262, b1p_hx_vh_q262;
  logic signed [63:0] b1p_hy_t1_q262, b1p_hy_t2_q262, b1p_hy_vh_q262;
  logic signed [63:0] b1p_hz_t1_q262, b1p_hz_t2_q262, b1p_hz_vh_q262;

  logic b1_valid, b1_last;
  (* keep = "true" *) logic signed [31:0] b1_hx_t1, b1_hx_t2, b1_hx_vh;
  (* keep = "true" *) logic signed [31:0] b1_hy_t1, b1_hy_t2, b1_hy_vh;
  (* keep = "true" *) logic signed [31:0] b1_hz_t1, b1_hz_t2, b1_hz_vh;

  logic b1b_valid, b1b_last;
  logic signed [32:0] b1b_hx_sum, b1b_hy_sum, b1b_hz_sum;
  logic signed [31:0] b1b_hx_vh, b1b_hy_vh, b1b_hz_vh;

  // Stage B1C: final 34-bit sum (+vh), registered before saturation so the add
  // carry chain and the satq131_34 comparators no longer chain in one cycle.
  logic b1c_valid, b1c_last;
  logic signed [33:0] b1c_hx_sum, b1c_hy_sum, b1c_hz_sum;

  logic b2_valid, b2_last;
  logic signed [31:0] b2_hx, b2_hy, b2_hz;

  wire norm_in_ready;
  wire b2_to_norm = b2_valid && norm_in_ready;
  wire b2_ready = !b2_valid || b2_to_norm;

  wire b1c_to_b2 = b1c_valid && b2_ready;
  wire b1c_ready = !b1c_valid || b1c_to_b2;

  wire b1b_to_b1c = b1b_valid && b1c_ready;
  wire b1b_ready = !b1b_valid || b1b_to_b1c;

  wire b1_to_b1b = b1_valid && b1b_ready;
  wire b1_ready = !b1_valid || b1_to_b1b;

  wire b1p_to_b1 = b1p_valid && b1_ready;
  wire b1p_ready = !b1p_valid || b1p_to_b1;

  wire b1a_to_b1p = b1a_valid && b1p_ready;
  wire b1a_ready = !b1a_valid || b1a_to_b1p;

  wire b0p_to_b1a = b0p_valid && b1a_ready;
  wire b0p_ready = !b0p_valid || b0p_to_b1a;

  wire b0_to_b0p = b0_valid && b0p_ready;
  wire b0_ready = !b0_valid || b0_to_b0p;
  assign sqrt_out_fire = sqrt_out_valid && b0_ready;
  assign sqrt_out_ready = b0_ready;

  wire signed [63:0] b1a_hx_t1_q262_w = mul_q131_q131_to_q262(b0p_hx_t1_a, b0p_hx_t1_b);
  wire signed [63:0] b1a_hx_t2_q262_w = mul_q131_q131_to_q262(b0p_hx_t2_a, b0p_hx_t2_b);
  wire signed [63:0] b1a_hx_vh_q262_w = mul_q131_q131_to_q262(b0p_hx_vh_a, b0p_hx_vh_b);
  wire signed [63:0] b1a_hy_t1_q262_w = mul_q131_q131_to_q262(b0p_hy_t1_a, b0p_hy_t1_b);
  wire signed [63:0] b1a_hy_t2_q262_w = mul_q131_q131_to_q262(b0p_hy_t2_a, b0p_hy_t2_b);
  wire signed [63:0] b1a_hy_vh_q262_w = mul_q131_q131_to_q262(b0p_hy_vh_a, b0p_hy_vh_b);
  wire signed [63:0] b1a_hz_t1_q262_w = mul_q131_q131_to_q262(b0p_hz_t1_a, b0p_hz_t1_b);
  wire signed [63:0] b1a_hz_t2_q262_w = mul_q131_q131_to_q262(b0p_hz_t2_a, b0p_hz_t2_b);
  wire signed [63:0] b1a_hz_vh_q262_w = mul_q131_q131_to_q262(b0p_hz_vh_a, b0p_hz_vh_b);

  wire signed [31:0] b1_hx_t1_w = scale_q262_to_q131(b1p_hx_t1_q262);
  wire signed [31:0] b1_hx_t2_w = scale_q262_to_q131(b1p_hx_t2_q262);
  wire signed [31:0] b1_hx_vh_w = scale_q262_to_q131(b1p_hx_vh_q262);
  wire signed [31:0] b1_hy_t1_w = scale_q262_to_q131(b1p_hy_t1_q262);
  wire signed [31:0] b1_hy_t2_w = scale_q262_to_q131(b1p_hy_t2_q262);
  wire signed [31:0] b1_hy_vh_w = scale_q262_to_q131(b1p_hy_vh_q262);
  wire signed [31:0] b1_hz_t1_w = scale_q262_to_q131(b1p_hz_t1_q262);
  wire signed [31:0] b1_hz_t2_w = scale_q262_to_q131(b1p_hz_t2_q262);
  wire signed [31:0] b1_hz_vh_w = scale_q262_to_q131(b1p_hz_vh_q262);

  wire signed [33:0] b2_hx_sum_w = $signed({b1b_hx_sum[32], b1b_hx_sum}) + $signed({{2{b1b_hx_vh[31]}}, b1b_hx_vh});
  wire signed [33:0] b2_hy_sum_w = $signed({b1b_hy_sum[32], b1b_hy_sum}) + $signed({{2{b1b_hy_vh[31]}}, b1b_hy_vh});
  wire signed [33:0] b2_hz_sum_w = $signed({b1b_hz_sum[32], b1b_hz_sum}) + $signed({{2{b1b_hz_vh[31]}}, b1b_hz_vh});

  wire signed [31:0] b2_hx_w = satq131_34(b1c_hx_sum);
  wire signed [31:0] b2_hy_w = satq131_34(b1c_hy_sum);
  wire signed [31:0] b2_hz_w = satq131_34(b1c_hz_sum);

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      b0_valid <= 1'b0;
      b0_last <= 1'b0;
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
      end else if (b0_to_b0p) begin
        b0_valid <= 1'b0;
      end
    end
  end

  // Stage B0P: fan the b0 operands out into one dedicated register pair per
  // multiply. Each _a/_b register drives exactly one DSP, so it packs cleanly
  // into that DSP's AREG/BREG.
  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      b0p_valid <= 1'b0;
      b0p_last <= 1'b0;
    end else begin
      if (b0_to_b0p) begin
        b0p_valid <= 1'b1;
        b0p_last <= b0_last;
        b0p_hx_t1_a <= $signed(b0_t1[31:0]);  b0p_hx_t1_b <= b0_t1_s;
        b0p_hx_t2_a <= $signed(b0_t2[31:0]);  b0p_hx_t2_b <= b0_t2_s;
        b0p_hx_vh_a <= $signed(b0_vh[31:0]);  b0p_hx_vh_b <= b0_t3_s;
        b0p_hy_t1_a <= $signed(b0_t1[63:32]); b0p_hy_t1_b <= b0_t1_s;
        b0p_hy_t2_a <= $signed(b0_t2[63:32]); b0p_hy_t2_b <= b0_t2_s;
        b0p_hy_vh_a <= $signed(b0_vh[63:32]); b0p_hy_vh_b <= b0_t3_s;
        b0p_hz_t1_a <= $signed(b0_t1[95:64]); b0p_hz_t1_b <= b0_t1_s;
        b0p_hz_t2_a <= $signed(b0_t2[95:64]); b0p_hz_t2_b <= b0_t2_s;
        b0p_hz_vh_a <= $signed(b0_vh[95:64]); b0p_hz_vh_b <= b0_t3_s;
      end else if (b0p_to_b1a) begin
        b0p_valid <= 1'b0;
      end
    end
  end

  // Stage B1A: registered products (DSP MREG).
  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      b1a_valid <= 1'b0;
      b1a_last <= 1'b0;
    end else begin
      if (b0p_to_b1a) begin
        b1a_valid <= 1'b1;
        b1a_last <= b0p_last;
        b1a_hx_t1_q262 <= b1a_hx_t1_q262_w;
        b1a_hx_t2_q262 <= b1a_hx_t2_q262_w;
        b1a_hx_vh_q262 <= b1a_hx_vh_q262_w;
        b1a_hy_t1_q262 <= b1a_hy_t1_q262_w;
        b1a_hy_t2_q262 <= b1a_hy_t2_q262_w;
        b1a_hy_vh_q262 <= b1a_hy_vh_q262_w;
        b1a_hz_t1_q262 <= b1a_hz_t1_q262_w;
        b1a_hz_t2_q262 <= b1a_hz_t2_q262_w;
        b1a_hz_vh_q262 <= b1a_hz_vh_q262_w;
      end else if (b1a_to_b1p) begin
        b1a_valid <= 1'b0;
      end
    end
  end

  // Stage B1P: product pipeline copy (DSP PREG).
  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      b1p_valid <= 1'b0;
      b1p_last <= 1'b0;
    end else begin
      if (b1a_to_b1p) begin
        b1p_valid <= 1'b1;
        b1p_last <= b1a_last;
        b1p_hx_t1_q262 <= b1a_hx_t1_q262;
        b1p_hx_t2_q262 <= b1a_hx_t2_q262;
        b1p_hx_vh_q262 <= b1a_hx_vh_q262;
        b1p_hy_t1_q262 <= b1a_hy_t1_q262;
        b1p_hy_t2_q262 <= b1a_hy_t2_q262;
        b1p_hy_vh_q262 <= b1a_hy_vh_q262;
        b1p_hz_t1_q262 <= b1a_hz_t1_q262;
        b1p_hz_t2_q262 <= b1a_hz_t2_q262;
        b1p_hz_vh_q262 <= b1a_hz_vh_q262;
      end else if (b1p_to_b1) begin
        b1p_valid <= 1'b0;
      end
    end
  end

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      b1_valid <= 1'b0;
      b1_last <= 1'b0;
    end else begin
      if (b1p_to_b1) begin
        b1_valid <= 1'b1;
        b1_last <= b1p_last;
        b1_hx_t1 <= b1_hx_t1_w;
        b1_hx_t2 <= b1_hx_t2_w;
        b1_hx_vh <= b1_hx_vh_w;
        b1_hy_t1 <= b1_hy_t1_w;
        b1_hy_t2 <= b1_hy_t2_w;
        b1_hy_vh <= b1_hy_vh_w;
        b1_hz_t1 <= b1_hz_t1_w;
        b1_hz_t2 <= b1_hz_t2_w;
        b1_hz_vh <= b1_hz_vh_w;
      end else if (b1_to_b1b) begin
        b1_valid <= 1'b0;
      end
    end
  end

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      b1b_valid  <= 1'b0;
      b1b_last   <= 1'b0;
    end else begin
      if (b1_to_b1b) begin
        b1b_valid  <= 1'b1;
        b1b_last   <= b1_last;
        b1b_hx_sum <= $signed({b1_hx_t1[31], b1_hx_t1}) + $signed({b1_hx_t2[31], b1_hx_t2});
        b1b_hx_vh  <= b1_hx_vh;
        b1b_hy_sum <= $signed({b1_hy_t1[31], b1_hy_t1}) + $signed({b1_hy_t2[31], b1_hy_t2});
        b1b_hy_vh  <= b1_hy_vh;
        b1b_hz_sum <= $signed({b1_hz_t1[31], b1_hz_t1}) + $signed({b1_hz_t2[31], b1_hz_t2});
        b1b_hz_vh  <= b1_hz_vh;
      end else if (b1b_to_b1c) begin
        b1b_valid  <= 1'b0;
      end
    end
  end

  // Stage B1C: register the final (+vh) sum ahead of saturation.
  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      b1c_valid <= 1'b0;
      b1c_last  <= 1'b0;
    end else begin
      if (b1b_to_b1c) begin
        b1c_valid  <= 1'b1;
        b1c_last   <= b1b_last;
        b1c_hx_sum <= b2_hx_sum_w;
        b1c_hy_sum <= b2_hy_sum_w;
        b1c_hz_sum <= b2_hz_sum_w;
      end else if (b1c_to_b2) begin
        b1c_valid  <= 1'b0;
      end
    end
  end

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      b2_valid <= 1'b0;
      b2_last <= 1'b0;
    end else begin
      if (b1c_to_b2) begin
        b2_valid <= 1'b1;
        b2_last <= b1c_last;
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

  // norm input skid FIFO: registers the ready path so the B-stage backpressure
  // does not depend combinationally on norm3's (deep) ready. TLAST is still
  // tracked by meta1 keyed on b2_to_norm (in-order, depth covers norm3 latency).
  wire         fifo_norm_valid;
  wire         fifo_norm_ready;
  wire [127:0] fifo_norm_data;
  wire         fifo_norm_last;

  axis_fifo_2deep #(
    .DATA_WIDTH(128)
  ) u_norm_fifo (
    .clk(s00_axis_aclk),
    .resetn(s00_axis_aresetn),
    .s_axis_tvalid(b2_valid),
    .s_axis_tready(norm_in_ready),
    .s_axis_tdata({32'b0, b2_hz, b2_hy, b2_hx}),
    .s_axis_tlast(b2_last),
    .m_axis_tvalid(fifo_norm_valid),
    .m_axis_tready(fifo_norm_ready),
    .m_axis_tdata(fifo_norm_data),
    .m_axis_tlast(fifo_norm_last)
  );

  axis_fixed_norm3 u_norm_h (
    .s00_axis_aclk(s00_axis_aclk),
    .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(1'b0),
    .s00_axis_tvalid(fifo_norm_valid),
    .s00_axis_tdata(fifo_norm_data),
    .s00_axis_tstrb('1),
    .s00_axis_tready(fifo_norm_ready),

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
