`timescale 1ns / 1ps
`default_nettype none

module axis_nested_uniform_scramble #
	(
		parameter integer C_S00_AXIS_TDATA_WIDTH	= 64,
		parameter integer C_M00_AXIS_TDATA_WIDTH	= 64
	)
	(
		// Ports of Axi Slave Bus Interface S00_AXIS
		input wire  s00_axis_aclk, s00_axis_aresetn,
		input wire  s00_axis_tlast, s00_axis_tvalid,
		input wire [C_S00_AXIS_TDATA_WIDTH-1 : 0] s00_axis_tdata, // {seed, x (u32)}
		input wire [(C_S00_AXIS_TDATA_WIDTH/8)-1: 0] s00_axis_tstrb,
		output logic  s00_axis_tready,

		// Ports of Axi Master Bus Interface M00_AXIS
		input wire  m00_axis_aclk, m00_axis_aresetn,
		input wire  m00_axis_tready,
		output logic  m00_axis_tvalid, m00_axis_tlast,
		output logic [C_M00_AXIS_TDATA_WIDTH-1 : 0] m00_axis_tdata, // {seed , scrambled x (u32)}
		output logic [(C_M00_AXIS_TDATA_WIDTH/8)-1: 0] m00_axis_tstrb
	);
  localparam int DATA_WIDTH = 32;
  localparam int SEED_WIDTH = 32;

  // Laine-Karras Permutation Constants
  localparam logic [31:0] LK_CONST_1 = 32'h6c50b47c;
  localparam logic [31:0] LK_CONST_2 = 32'hb82f1e52;
  localparam logic [31:0] LK_CONST_3 = 32'hc7afe638;
  localparam logic [31:0] LK_CONST_4 = 32'h8d22f6e6;

  function automatic [DATA_WIDTH-1:0] reverse_bits(input [DATA_WIDTH-1:0] in);
    integer i;
    begin
      for (i = 0; i < DATA_WIDTH; i++) begin
        reverse_bits[i] = in[DATA_WIDTH-1-i];
      end
    end
  endfunction

  logic stall, advance;
  always_comb begin
    stall  = m00_axis_tvalid && !m00_axis_tready;
    advance    = !stall;
    s00_axis_tready = !stall;
  end

  // ---------------------------------------------------------------------------
  // Sideband (valid / last / seed) delay line.
  //
  // TVALID/TLAST/seed are ALIGNED with TDATA at the output: the data path below
  // is 19 registers deep (rev_in, x_add, 4 rounds x {lk, sum, r}, y_rev, yt1-4),
  // so the sideband is 19 deep too (indices 0..SIDEBAND_DEPTH). Main's original
  // implementation was aligned as well; an earlier version of this branch
  // shipped a -4 skew here by mistake (caught by test_integration_dual).
  // From the package, so the depth and every consumer's view of this block's
  // latency are one number. See docs/adr/0002-latency-package-is-law.md.
  localparam int SIDEBAND_DEPTH = ggx_latency_pkg::SCRAMBLE_SIDEBAND_DEPTH;
  logic [SIDEBAND_DEPTH:0]           valid_pipeline, last_pipeline;
  logic [SEED_WIDTH-1:0]             seed_pipeline [0:SIDEBAND_DEPTH];

  // Bit-exact 32x32 constant multiply split into 16-bit halves, so each partial
  // product is a single 16x16 DSP with no cascade. See axis_hash_combine_2d for
  // the derivation. Truncating operands is not valid here either: the Laine-Karras
  // scramble relies on the full multiply to propagate entropy across all bits.
  //   x*C = x_lo*C_lo + ((x_lo*C_hi + x_hi*C_lo) << 16)   (mod 2^32)
  function automatic [63:0] cmul_pp(input [31:0] x, input [31:0] c);
    logic [31:0] p_ll, p_lh, p_hl;
    begin
      p_ll = x[15:0]  * c[15:0];
      p_lh = x[15:0]  * c[31:16];
      p_hl = x[31:16] * c[15:0];
      cmul_pp = {p_hl[15:0], p_lh[15:0], p_ll};
    end
  endfunction

  // The two middle partial products only ever affect bits [31:16], so this is a
  // single ternary 16-bit add (one carry chain), not a 16-bit add feeding a
  // 32-bit add. Bits [15:0] pass straight through from p_ll.
  function automatic [31:0] cmul_sum(input [63:0] pp);
    begin
      cmul_sum = {pp[31:16] + pp[47:32] + pp[63:48], pp[15:0]};
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Data path. Each Laine-Karras round used to be two register stages:
  //   (A) lk_mul = cmul_pp(x)                         -- DSP
  //   (B) x_next = x_delayed ^ cmul_sum(lk_mul)       -- ternary add + XOR
  // Stage B was the sampler's WNS holder: Vivado retimed the round-boundary
  // register into the next DSP, exposing the cmul_sum ternary add + XOR + the
  // DSP operand routing as one path. The split below gives cmul_sum its own
  // register so the ternary add and the XOR sit in separate cycles, and the XOR
  // (not the add) feeds the next DSP:
  //   (A) lk_mul = cmul_pp(x)                         -- DSP
  //   (B) sum    = cmul_sum(lk_mul)                   -- ternary add only
  //   (C) x_next = x_delayed ^ sum                    -- XOR only, feeds DSP
  // +1 cycle per round (4 total); absorbed by growing SIDEBAND_DEPTH above.

  // Front: input reverse + seed add.
  logic [DATA_WIDTH-1:0] rev_in;      // reverse_bits(x)
  logic [DATA_WIDTH-1:0] x_add;       // rev_in + seed  (= old data_pipeline[1])

  // Per-round registers. lkN = partial products, sumN = cmul_sum, rN = post-XOR.
  // xN_da / xN_db carry the round input forward to meet its XOR two cycles later.
  logic [63:0] lk_1, lk_2, lk_3, lk_4;
  logic [DATA_WIDTH-1:0] sum_1, sum_2, sum_3, sum_4;
  logic [DATA_WIDTH-1:0] r1, r2, r3, r4;
  logic [DATA_WIDTH-1:0] x0_da, x0_db;   // x_add delayed toward round-1 XOR
  logic [DATA_WIDTH-1:0] r1_da, r1_db;   // r1    delayed toward round-2 XOR
  logic [DATA_WIDTH-1:0] r2_da, r2_db;   // r2    delayed toward round-3 XOR
  logic [DATA_WIDTH-1:0] r3_da, r3_db;   // r3    delayed toward round-4 XOR

  // Back: reverse + tail carry (retiming slack for the placer).
  logic [DATA_WIDTH-1:0] y_rev, yt1, yt2, yt3, yt4;

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      valid_pipeline <= '0;
      last_pipeline  <= '0;
      for (integer i = 0; i <= SIDEBAND_DEPTH; i = i + 1) seed_pipeline[i] <= '0;
      rev_in <= '0; x_add <= '0;
      lk_1 <= '0; lk_2 <= '0; lk_3 <= '0; lk_4 <= '0;
      sum_1 <= '0; sum_2 <= '0; sum_3 <= '0; sum_4 <= '0;
      r1 <= '0; r2 <= '0; r3 <= '0; r4 <= '0;
      x0_da <= '0; x0_db <= '0;
      r1_da <= '0; r1_db <= '0;
      r2_da <= '0; r2_db <= '0;
      r3_da <= '0; r3_db <= '0;
      y_rev <= '0; yt1 <= '0; yt2 <= '0; yt3 <= '0; yt4 <= '0;
    end else begin
      if (advance) begin
        // --- Sideband delay line ---
        valid_pipeline[0] <= s00_axis_tvalid;
        last_pipeline[0]  <= s00_axis_tlast;
        seed_pipeline[0]  <= s00_axis_tdata[63:32];
        for (integer i = 0; i < SIDEBAND_DEPTH; i = i + 1) begin
          valid_pipeline[i+1] <= valid_pipeline[i];
          last_pipeline[i+1]  <= last_pipeline[i];
          seed_pipeline[i+1]  <= seed_pipeline[i];
        end

        // --- Front ---
        rev_in <= reverse_bits(s00_axis_tdata[31:0]);
        x_add  <= rev_in + seed_pipeline[0];

        // --- Round 1 (input x_add) ---
        lk_1  <= cmul_pp(x_add, LK_CONST_1);  x0_da <= x_add;
        sum_1 <= cmul_sum(lk_1);              x0_db <= x0_da;
        r1    <= x0_db ^ sum_1;

        // --- Round 2 (input r1) ---
        lk_2  <= cmul_pp(r1, LK_CONST_2);     r1_da <= r1;
        sum_2 <= cmul_sum(lk_2);              r1_db <= r1_da;
        r2    <= r1_db ^ sum_2;

        // --- Round 3 (input r2) ---
        lk_3  <= cmul_pp(r2, LK_CONST_3);     r2_da <= r2;
        sum_3 <= cmul_sum(lk_3);              r2_db <= r2_da;
        r3    <= r2_db ^ sum_3;

        // --- Round 4 (input r3) ---
        lk_4  <= cmul_pp(r3, LK_CONST_4);     r3_da <= r3;
        sum_4 <= cmul_sum(lk_4);              r3_db <= r3_da;
        r4    <= r3_db ^ sum_4;

        // --- Back: reverse + tail carry ---
        y_rev <= reverse_bits(r4);
        yt1   <= y_rev;
        yt2   <= yt1;
        yt3   <= yt2;
        yt4   <= yt3;
      end
    end
  end

  always_comb begin
    m00_axis_tdata  = {seed_pipeline[SIDEBAND_DEPTH], yt4};
    m00_axis_tvalid = valid_pipeline[SIDEBAND_DEPTH];
    m00_axis_tlast  = last_pipeline[SIDEBAND_DEPTH];
    m00_axis_tstrb  = '1;
  end

endmodule

`default_nettype wire
