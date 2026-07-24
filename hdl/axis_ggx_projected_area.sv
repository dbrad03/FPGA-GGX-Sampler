`timescale 1ns / 1ps
`default_nettype none

module axis_ggx_projected_area #
  (
    parameter integer C_S00_AXIS_TDATA_WIDTH = 96,
    parameter integer C_M00_AXIS_TDATA_WIDTH = 64,
    parameter integer FRAC_BITS              = 32,
    parameter integer TRIG_ADDR_BITS         = 10
  )
  (
    // Ports of Axi Slave Bus Interface S00_AXIS
    input  wire s00_axis_aclk,
    input  wire s00_axis_aresetn,
    input  wire s00_axis_tlast,
    input  wire s00_axis_tvalid,
    // {Vh.z (Q1.31 signed), u2 (UQ0.32), u1 (UQ0.32)}
    input  wire [C_S00_AXIS_TDATA_WIDTH-1:0] s00_axis_tdata,
    input  wire [(C_S00_AXIS_TDATA_WIDTH/8)-1:0] s00_axis_tstrb,
    output logic s00_axis_tready,

    // Ports of Axi Master Bus Interface M00_AXIS
    input  wire m00_axis_aclk,
    input  wire m00_axis_aresetn,
    input  wire m00_axis_tready,
    output logic m00_axis_tvalid,
    output logic m00_axis_tlast,
    // {t2 (Q1.31 signed), t1 (Q1.31 signed)}
    output logic signed [C_M00_AXIS_TDATA_WIDTH-1:0] m00_axis_tdata,
    output logic [(C_M00_AXIS_TDATA_WIDTH/8)-1:0] m00_axis_tstrb
  );

  localparam int SQRT_LATENCY = 33;
  localparam int TRIG_LATENCY = 2;
  localparam int SQRT_DELAY   = SQRT_LATENCY + 1;
  localparam int TRIG_DELAY   = TRIG_LATENCY + 1;
  localparam int SQRT_META_DEPTH = 256;
  localparam int TRIG_META_DEPTH = 256;
  localparam int SQRT_META_AW = $clog2(SQRT_META_DEPTH);
  localparam int TRIG_META_AW = $clog2(TRIG_META_DEPTH);

  localparam logic [31:0] ONE_UQ0_32 = 32'hFFFF_FFFF;
  localparam logic signed [31:0] ONE_Q1     = 32'sh7FFF_FFFF;
  localparam logic signed [31:0] NEG_ONE_Q1 = 32'sh8000_0000;

  wire [31:0] in_u1_uq032 = s00_axis_tdata[31:0];
  wire [31:0] in_u2_uq032 = s00_axis_tdata[63:32];
  wire signed [31:0] in_vhz_q131 = $signed(s00_axis_tdata[95:64]);

  function automatic logic signed [31:0] satq131(input logic signed [63:0] val);
    begin
      if (val > $signed(64'sh0000_0000_7FFF_FFFF)) satq131 = ONE_Q1;
      else if (val < $signed(64'shFFFF_FFFF_8000_0000)) satq131 = NEG_ONE_Q1;
      else satq131 = val[31:0];
    end
  endfunction

  function automatic logic signed [31:0] mul_q131_uq032_to_q131(
    input logic signed [31:0] q131,
    input logic [31:0] uq032
  );
    logic signed [23:0] q131_24;
    logic signed [17:0] uq032_18;
    logic signed [41:0] prod;
    begin
      q131_24 = q131[31:8];
      uq032_18 = $signed({1'b0, uq032[31:15]});
      prod = q131_24 * uq032_18;     // Q1.23 * Q0.17 -> Q1.40
      mul_q131_uq032_to_q131 = prod >>> 9; // Q1.40 -> Q1.31
    end
  endfunction

  function automatic logic [31:0] mul_uq032_uq032_to_uq032(
    input logic [31:0] uq032_a,
    input logic [31:0] uq032_b
  );
    logic signed [24:0] a_25;
    logic signed [17:0] b_18;
    logic signed [42:0] prod;
    begin
      a_25 = $signed({1'b0, uq032_a[31:9]}); // 24-bit unsigned -> 25-bit signed
      b_18 = $signed({1'b0, uq032_b[31:15]}); // 17-bit unsigned -> 18-bit signed
      prod = a_25 * b_18;                    // Q0.23 * Q0.17 -> Q0.40
      mul_uq032_uq032_to_uq032 = prod >>> 8; // Q0.40 -> Q0.32
    end
  endfunction

  function automatic logic signed [63:0] mul_q131_q131_to_q262(
    input logic signed [31:0] q131_a,
    input logic signed [31:0] q131_b
  );
    logic signed [23:0] a_24;
    logic signed [17:0] b_18;
    logic signed [41:0] prod;
    begin
      a_24 = q131_a[31:8];
      b_18 = q131_b[31:14];
      prod = a_24 * b_18; // Q1.23 * Q1.17 -> Q2.40
      mul_q131_q131_to_q262 = 64'(prod) <<< 22; // Q2.40 -> Q2.62
    end
  endfunction

  // --------------------------------------------------------------------------
  // Stage A: r = sqrt(u1)
  // --------------------------------------------------------------------------
  wire sqrt_r_in_ready;
  wire sqrt_r_out_valid;
  wire [31:0] sqrt_r_out_uq032;
  logic sqrt_r_out_ready;

  axis_fixed_sqrt #(
    .FRAC_BITS(FRAC_BITS)
  ) u_sqrt_r (
    .s00_axis_aclk(s00_axis_aclk),
    .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(1'b0),
    .s00_axis_tvalid(s00_axis_tvalid),
    .s00_axis_tdata(in_u1_uq032),
    .s00_axis_tstrb('1),
    .s00_axis_tready(sqrt_r_in_ready),

    .m00_axis_aclk(s00_axis_aclk),
    .m00_axis_aresetn(s00_axis_aresetn),
    .m00_axis_tready(sqrt_r_out_ready),
    .m00_axis_tvalid(sqrt_r_out_valid),
    .m00_axis_tlast(),
    .m00_axis_tdata(sqrt_r_out_uq032),
    .m00_axis_tstrb()
  );

  assign s00_axis_tready = sqrt_r_in_ready;
  wire sqrt_r_in_fire = s00_axis_tvalid && sqrt_r_in_ready;
  wire sqrt_r_out_fire;

  // Align u2/Vh.z/TLAST with sqrt(u1).
  logic [31:0] meta0_u2 [0:SQRT_META_DEPTH-1];
  logic signed [31:0] meta0_vhz [0:SQRT_META_DEPTH-1];
  logic meta0_last [0:SQRT_META_DEPTH-1];
  logic [SQRT_META_AW-1:0] meta0_wr_ptr, meta0_rd_ptr;
  logic [SQRT_META_AW:0] meta0_count;
  wire [31:0] delayed_u2_0 = meta0_u2[meta0_rd_ptr];
  wire signed [31:0] delayed_vhz_0 = meta0_vhz[meta0_rd_ptr];
  wire delayed_last_0_tap = meta0_last[meta0_rd_ptr];

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      meta0_wr_ptr <= '0;
      meta0_rd_ptr <= '0;
      meta0_count <= '0;
    end else begin
      if (sqrt_r_in_fire) begin
        meta0_u2[meta0_wr_ptr] <= in_u2_uq032;
        meta0_vhz[meta0_wr_ptr] <= in_vhz_q131;
        meta0_last[meta0_wr_ptr] <= s00_axis_tlast;
        meta0_wr_ptr <= meta0_wr_ptr + 1'b1;
      end
      if (sqrt_r_out_fire) begin
        meta0_rd_ptr <= meta0_rd_ptr + 1'b1;
      end
      case ({sqrt_r_in_fire, sqrt_r_out_fire})
        2'b10: meta0_count <= meta0_count + 1'b1;
        2'b01: meta0_count <= meta0_count - 1'b1;
        default: begin end
      endcase
    end
  end

  // --------------------------------------------------------------------------
  // Stage B: phi = 2*pi*u2, compute r*cos(phi), r*sin(phi)
  // --------------------------------------------------------------------------
  logic s1_valid, s1_last;
  logic [31:0] s1_r_uq032, s1_u2_uq032;
  logic signed [31:0] s1_vhz_q131;

  wire trig_in_ready;
  wire trig_out_valid;
  wire [63:0] trig_out_data; // {sin, cos}
  wire s1_to_trig = s1_valid && trig_in_ready;
  wire s1_ready = !s1_valid || s1_to_trig;
  assign sqrt_r_out_fire = sqrt_r_out_valid && s1_ready;
  assign sqrt_r_out_ready = s1_ready;

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      s1_valid <= 1'b0;
      s1_last <= 1'b0;
      s1_r_uq032 <= '0;
      s1_u2_uq032 <= '0;
      s1_vhz_q131 <= '0;
    end else begin
      if (sqrt_r_out_fire) begin
        s1_valid <= 1'b1;
        s1_last <= delayed_last_0_tap;
        s1_r_uq032 <= sqrt_r_out_uq032;
        s1_u2_uq032 <= delayed_u2_0;
        s1_vhz_q131 <= delayed_vhz_0;
      end else if (s1_to_trig) begin
        s1_valid <= 1'b0;
      end
    end
  end

  // Stage B1: Register trig outputs before the DSP multiplies.
  logic s1b_valid, s1b_last;
  logic [31:0] s1b_r_uq032;
  logic signed [31:0] s1b_trig_sin_q131, s1b_trig_cos_q131;
  logic signed [31:0] s1b_vhz_q131;

  // Stage C0: Register t1/t2 partials after the DSP multiplies.
  logic s2_valid, s2_last;
  logic signed [31:0] s2_t1_q131, s2_t2_q131;
  logic signed [31:0] s2_vhz_q131;

  // Stage C1: Register t1^2 before the second sqrt.
  logic c0_valid, c0_last;
  logic signed [31:0] c0_t1_q131, c0_t2_q131, c0_vhz_q131;
  logic signed [63:0] c0_t1_sq_q262;

  wire sqrt_t_in_ready;
  wire c0_to_sqrt2 = c0_valid && sqrt_t_in_ready;
  wire c0_ready = !c0_valid || c0_to_sqrt2;
  wire s2_to_c0 = s2_valid && c0_ready;
  wire s2_ready = !s2_valid || s2_to_c0;
  wire s1b_to_s2 = s1b_valid && s2_ready;
  wire s1b_ready = !s1b_valid || s1b_to_s2;

  axis_trig_lut #(
    .C_S00_AXIS_TDATA_WIDTH(64),
    .C_M00_AXIS_TDATA_WIDTH(64),
    .FRAC_BITS(FRAC_BITS),
    .ADDR_BITS(TRIG_ADDR_BITS)
  ) u_trig (
    .s00_axis_aclk(s00_axis_aclk),
    .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(1'b0),
    .s00_axis_tvalid(s1_valid),
    .s00_axis_tdata({32'b0, s1_u2_uq032}),
    .s00_axis_tstrb('1),
    .s00_axis_tready(trig_in_ready),

    .m00_axis_aclk(s00_axis_aclk),
    .m00_axis_aresetn(s00_axis_aresetn),
    .m00_axis_tready(s1b_ready),
    .m00_axis_tvalid(trig_out_valid),
    .m00_axis_tlast(),
    .m00_axis_tdata(trig_out_data),
    .m00_axis_tstrb()
  );

  wire trig_out_fire = trig_out_valid && s1b_ready;

  // Align r/Vh.z/TLAST with trig output.
  logic [31:0] meta1_r [0:TRIG_META_DEPTH-1];
  logic signed [31:0] meta1_vhz [0:TRIG_META_DEPTH-1];
  logic meta1_last [0:TRIG_META_DEPTH-1];
  logic [TRIG_META_AW-1:0] meta1_wr_ptr, meta1_rd_ptr;
  logic [TRIG_META_AW:0] meta1_count;
  wire [31:0] delayed_r_1 = meta1_r[meta1_rd_ptr];
  wire signed [31:0] delayed_vhz_1 = meta1_vhz[meta1_rd_ptr];
  wire delayed_last_1_tap = meta1_last[meta1_rd_ptr];

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      meta1_wr_ptr <= '0;
      meta1_rd_ptr <= '0;
      meta1_count <= '0;
    end else begin
      if (s1_to_trig) begin
        meta1_r[meta1_wr_ptr] <= s1_r_uq032;
        meta1_vhz[meta1_wr_ptr] <= s1_vhz_q131;
        meta1_last[meta1_wr_ptr] <= s1_last;
        meta1_wr_ptr <= meta1_wr_ptr + 1'b1;
      end
      if (trig_out_fire) begin
        meta1_rd_ptr <= meta1_rd_ptr + 1'b1;
      end
      case ({s1_to_trig, trig_out_fire})
        2'b10: meta1_count <= meta1_count + 1'b1;
        2'b01: meta1_count <= meta1_count - 1'b1;
        default: begin end
      endcase
    end
  end

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      s1b_valid <= 1'b0;
      s1b_last <= 1'b0;
      s1b_r_uq032 <= '0;
      s1b_trig_sin_q131 <= '0;
      s1b_trig_cos_q131 <= '0;
      s1b_vhz_q131 <= '0;
    end else begin
      if (trig_out_fire) begin
        s1b_valid <= 1'b1;
        s1b_last <= delayed_last_1_tap;
        s1b_r_uq032 <= delayed_r_1;
        s1b_trig_sin_q131 <= $signed(trig_out_data[63:32]);
        s1b_trig_cos_q131 <= $signed(trig_out_data[31:0]);
        s1b_vhz_q131 <= delayed_vhz_1;
      end else if (s1b_to_s2) begin
        s1b_valid <= 1'b0;
      end
    end
  end

  wire signed [31:0] t1_q131_pre = mul_q131_uq032_to_q131(s1b_trig_cos_q131, s1b_r_uq032);
  wire signed [31:0] t2_q131_pre = mul_q131_uq032_to_q131(s1b_trig_sin_q131, s1b_r_uq032);

  // --------------------------------------------------------------------------
  // Stage C: sqrt(max(0, 1 - t1^2))
  // --------------------------------------------------------------------------
  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      s2_valid <= 1'b0;
      s2_last <= 1'b0;
      s2_t1_q131 <= '0;
      s2_t2_q131 <= '0;
      s2_vhz_q131 <= '0;
    end else begin
      if (s1b_to_s2) begin
        s2_valid <= 1'b1;
        s2_last <= s1b_last;
        s2_t1_q131 <= t1_q131_pre;
        s2_t2_q131 <= t2_q131_pre;
        s2_vhz_q131 <= s1b_vhz_q131;
      end else if (s2_to_c0) begin
        s2_valid <= 1'b0;
      end
    end
  end

  wire signed [63:0] c0_t1_sq_q262_w = mul_q131_q131_to_q262(s2_t1_q131, s2_t1_q131);
  wire [31:0] t1_sq_uq032 = c0_t1_sq_q262[61:30];
  wire [31:0] sqrt_t_arg_uq032 = c0_t1_sq_q262[62] ? 32'd0 : (ONE_UQ0_32 - t1_sq_uq032);

  wire sqrt_t_out_valid;
  wire [31:0] sqrt_t_out_uq032;
  logic sqrt_t_out_ready;

  wire out_pipe_en = m00_axis_tready || !m00_axis_tvalid;

  axis_fixed_sqrt #(
    .FRAC_BITS(FRAC_BITS)
  ) u_sqrt_t (
    .s00_axis_aclk(s00_axis_aclk),
    .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(1'b0),
    .s00_axis_tvalid(c0_valid),
    .s00_axis_tdata(sqrt_t_arg_uq032),
    .s00_axis_tstrb('1),
    .s00_axis_tready(sqrt_t_in_ready),

    .m00_axis_aclk(s00_axis_aclk),
    .m00_axis_aresetn(s00_axis_aresetn),
    .m00_axis_tready(sqrt_t_out_ready),
    .m00_axis_tvalid(sqrt_t_out_valid),
    .m00_axis_tlast(),
    .m00_axis_tdata(sqrt_t_out_uq032),
    .m00_axis_tstrb()
  );

  wire sqrt_t_out_fire;

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      c0_valid <= 1'b0;
      c0_last <= 1'b0;
      c0_t1_q131 <= '0;
      c0_t2_q131 <= '0;
      c0_vhz_q131 <= '0;
      c0_t1_sq_q262 <= '0;
    end else begin
      if (s2_to_c0) begin
        c0_valid <= 1'b1;
        c0_last <= s2_last;
        c0_t1_q131 <= s2_t1_q131;
        c0_t2_q131 <= s2_t2_q131;
        c0_vhz_q131 <= s2_vhz_q131;
        c0_t1_sq_q262 <= c0_t1_sq_q262_w;
      end else if (c0_to_sqrt2) begin
        c0_valid <= 1'b0;
      end
    end
  end

  // Align t1/t2_pre/Vh.z/TLAST with sqrt(max(...)) output.
  logic signed [31:0] meta2_t1 [0:SQRT_META_DEPTH-1];
  logic signed [31:0] meta2_t2 [0:SQRT_META_DEPTH-1];
  logic signed [31:0] meta2_vhz [0:SQRT_META_DEPTH-1];
  logic meta2_last [0:SQRT_META_DEPTH-1];
  logic [SQRT_META_AW-1:0] meta2_wr_ptr, meta2_rd_ptr;
  logic [SQRT_META_AW:0] meta2_count;
  wire signed [31:0] delayed_t1_2 = meta2_t1[meta2_rd_ptr];
  wire signed [31:0] delayed_t2_2 = meta2_t2[meta2_rd_ptr];
  wire signed [31:0] delayed_vhz_2 = meta2_vhz[meta2_rd_ptr];
  wire delayed_last_2_tap = meta2_last[meta2_rd_ptr];

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      meta2_wr_ptr <= '0;
      meta2_rd_ptr <= '0;
      meta2_count <= '0;
    end else begin
      if (c0_to_sqrt2) begin
        meta2_t1[meta2_wr_ptr] <= c0_t1_q131;
        meta2_t2[meta2_wr_ptr] <= c0_t2_q131;
        meta2_vhz[meta2_wr_ptr] <= c0_vhz_q131;
        meta2_last[meta2_wr_ptr] <= c0_last;
        meta2_wr_ptr <= meta2_wr_ptr + 1'b1;
      end
      if (sqrt_t_out_fire) begin
        meta2_rd_ptr <= meta2_rd_ptr + 1'b1;
      end
      case ({c0_to_sqrt2, sqrt_t_out_fire})
        2'b10: meta2_count <= meta2_count + 1'b1;
        2'b01: meta2_count <= meta2_count - 1'b1;
        default: begin end
      endcase
    end
  end

  // --------------------------------------------------------------------------
  // Stage D0: Register metadata + sqrt output aligned by the BRAM-backed
  //           meta2 arrays.  Vivado may absorb d0 FFs into the BRAM output
  //           registers; D1a below provides a guaranteed non-BRAM cut point.
  // --------------------------------------------------------------------------
  logic d0_valid, d0_last;
  logic signed [31:0] d0_t1_q131, d0_t2_q131, d0_vhz_q131;
  logic [31:0] d0_sqrt_uq032;

  // --------------------------------------------------------------------------
  // Stage D1a: Compute s = 0.5*(1+Vh.z) and (1-s) then register.
  //            Splits the BRAM-fanout adder from the downstream DSP multiplies.
  // --------------------------------------------------------------------------
  logic d1a_valid, d1a_last;
  logic [31:0] d1a_s_uq032, d1a_one_minus_s_uq032;
  logic [31:0] d1a_sqrt_uq032;
  logic signed [31:0] d1a_t1_q131, d1a_t2_q131;

  // --------------------------------------------------------------------------
  // Stage D1b: DSP multiplies (term_a, term_b) registered.
  //            The DSP48E1 PREG adds one pipeline stage internally; this
  //            explicit FF stage ensures the post-multiply 33-bit adder in D1c
  //            starts from a clean FF rather than from the DSP PREG output.
  // --------------------------------------------------------------------------
  logic d1b_valid, d1b_last;
  logic [31:0] d1b_term_a_uq032;
  logic signed [31:0] d1b_term_b_q131, d1b_t1_q131;

  // Handshake chain: D0 -> D1a -> D1b -> output
  wire d1b_ready  = !d1b_valid || out_pipe_en;
  wire d1a_to_d1b = d1a_valid && d1b_ready;
  wire d1a_ready  = !d1a_valid || d1a_to_d1b;
  wire d0_to_d1a  = d0_valid && d1a_ready;
  wire d0_ready   = !d0_valid || d0_to_d1a;
  assign sqrt_t_out_fire  = sqrt_t_out_valid && d0_ready;
  assign sqrt_t_out_ready = d0_ready;

  // D0 register (loaded from BRAM-backed meta2 + sqrt output)
  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      d0_valid <= 1'b0;
      d0_last <= 1'b0;
      d0_t1_q131 <= '0;
      d0_t2_q131 <= '0;
      d0_vhz_q131 <= '0;
      d0_sqrt_uq032 <= '0;
    end else begin
      if (sqrt_t_out_fire) begin
        d0_valid <= 1'b1;
        d0_last <= delayed_last_2_tap;
        d0_t1_q131 <= delayed_t1_2;
        d0_t2_q131 <= delayed_t2_2;
        d0_vhz_q131 <= delayed_vhz_2;
        d0_sqrt_uq032 <= sqrt_t_out_uq032;
      end else if (d0_to_d1a) begin
        d0_valid <= 1'b0;
      end
    end
  end

  // D1a combinational: s = clamp(0.5 + vhz/2), one_minus_s = 1 - s
  logic signed [32:0] d1a_vhz_ext, d1a_s_ext;
  logic [31:0] d1a_s_uq032_w, d1a_one_minus_s_w;

  always_comb begin
    d1a_vhz_ext = $signed({d0_vhz_q131[31], d0_vhz_q131});
    d1a_s_ext   = 33'sh0_8000_0000 + d1a_vhz_ext;
    if (d1a_s_ext < 0)                         d1a_s_uq032_w = 32'd0;
    else if (d1a_s_ext > 33'sh0_FFFF_FFFF)    d1a_s_uq032_w = 32'hFFFF_FFFF;
    else                                        d1a_s_uq032_w = d1a_s_ext[31:0];
    d1a_one_minus_s_w = ONE_UQ0_32 - d1a_s_uq032_w;
  end

  // D1a register
  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      d1a_valid             <= 1'b0;
      d1a_last              <= 1'b0;
      d1a_s_uq032           <= '0;
      d1a_one_minus_s_uq032 <= '0;
      d1a_sqrt_uq032        <= '0;
      d1a_t1_q131           <= '0;
      d1a_t2_q131           <= '0;
    end else begin
      if (d0_to_d1a) begin
        d1a_valid             <= 1'b1;
        d1a_last              <= d0_last;
        d1a_s_uq032           <= d1a_s_uq032_w;
        d1a_one_minus_s_uq032 <= d1a_one_minus_s_w;
        d1a_sqrt_uq032        <= d0_sqrt_uq032;
        d1a_t1_q131           <= d0_t1_q131;
        d1a_t2_q131           <= d0_t2_q131;
      end else if (d1a_to_d1b) begin
        d1a_valid <= 1'b0;
      end
    end
  end

  // D1b combinational: two DSP multiplies (run in parallel, independent inputs)
  //   term_a_uq032 = (1-s) * sqrt   -- product of two UQ0.32 values, result in [0,1]
  //   term_b_q131  = t2_pre * s     -- Q1.31 * UQ0.32, result in Q1.31
  wire [31:0] d1b_term_a_uq032_w = mul_uq032_uq032_to_uq032(d1a_one_minus_s_uq032,
                                                              d1a_sqrt_uq032);
  wire signed [31:0] d1b_term_b_q131_w = mul_q131_uq032_to_q131(d1a_t2_q131,
                                                                  d1a_s_uq032);

  // D1b register: captures DSP outputs (+ DSP PREG if Vivado enables it).
  // Having this FF stage means D1c only needs one 33-bit adder + saturate.
  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      d1b_valid       <= 1'b0;
      d1b_last        <= 1'b0;
      d1b_term_a_uq032 <= '0;
      d1b_term_b_q131  <= '0;
      d1b_t1_q131      <= '0;
    end else begin
      if (d1a_to_d1b) begin
        d1b_valid        <= 1'b1;
        d1b_last         <= d1a_last;
        d1b_term_a_uq032 <= d1b_term_a_uq032_w;
        d1b_term_b_q131  <= d1b_term_b_q131_w;
        d1b_t1_q131      <= d1a_t1_q131;
      end else if (out_pipe_en) begin
        d1b_valid <= 1'b0;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Stage D1c (combinational into output register):
  //   t2 = term_a + term_b   saturated to Q1.31
  //
  // term_a_uq032 is the product of two values in [0,1], so it is always in
  // [0, 0xFFFF_FFFF] (UQ0.32, always non-negative).  Converting to Q1.31 is
  // a right-shift-by-1; the +1 rounding step from the previous version
  // contributed < 2^-32 error (<<< 1.2e-2 tolerance) and is omitted here to
  // eliminate 9 CARRY4 stages from the critical path.
  //
  // The saturating add uses 33-bit arithmetic rather than 64-bit because:
  //   term_a_q131 in [0, ONE_Q1]           (non-negative)
  //   term_b_q131 in [NEG_ONE_Q1, ONE_Q1]
  //   sum in [NEG_ONE_Q1, ~2*ONE_Q1] which fits in 33 signed bits
  // Overflow is detected on bits [32:31]: 2'b01 = +overflow, 2'b10 = -overflow.
  // This replaces a 64-bit sign-extended add (16 CARRY4) with a 33-bit add
  // (9 CARRY4), reducing the D1c logic depth by ~5 ns.
  // --------------------------------------------------------------------------
  logic signed [31:0] t2_q131_out;
  logic signed [32:0] t2_sum33;

  always_comb begin
    // term_a: shift UQ0.32 → Q1.31 (truncate; max error < 2^-32)
    // Since term_a_uq032 is always in [0,1], bit[31] of {1'b0, term_a[31:1]}
    // is always 0, so the result is always non-negative -- no clamp needed.
    t2_sum33 = $signed({1'b0, d1b_term_a_uq032[31:1]}) +
               $signed({d1b_term_b_q131[31], d1b_term_b_q131});
    // Saturate: bits[32:31]: 00/11 = in-range, 01 = +overflow, 10 = -overflow
    case (t2_sum33[32:31])
      2'b01:   t2_q131_out = ONE_Q1;
      2'b10:   t2_q131_out = NEG_ONE_Q1;
      default: t2_q131_out = t2_sum33[31:0];
    endcase
  end

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      m00_axis_tvalid <= 1'b0;
      m00_axis_tlast <= 1'b0;
      m00_axis_tdata <= '0;
    end else if (out_pipe_en) begin
      m00_axis_tvalid <= d1b_valid;
      if (d1b_valid) begin
        m00_axis_tlast <= d1b_last;
        m00_axis_tdata <= {t2_q131_out, d1b_t1_q131};
      end
    end
  end

  assign m00_axis_tstrb = '1;

endmodule

`default_nettype wire
