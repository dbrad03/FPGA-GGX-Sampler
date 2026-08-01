`timescale 1ns / 1ps
`default_nettype none

module axis_ggx_event_basis #
  (
    parameter integer C_S00_AXIS_TDATA_WIDTH	= 128,
		parameter integer C_M00_AXIS_TDATA_WIDTH	= 96,
		parameter integer FRAC_BITS = 32
	)
	(
		// Ports of Axi Slave Bus Interface S00_AXIS
		input wire  s00_axis_aclk, s00_axis_aresetn,
		input wire  s00_axis_tlast, s00_axis_tvalid,
		input wire [C_S00_AXIS_TDATA_WIDTH-1 : 0] s00_axis_tdata, // {alpha, view_z, view_y, view_x}
		input wire [(C_S00_AXIS_TDATA_WIDTH/8)-1: 0] s00_axis_tstrb,
		output logic  s00_axis_tready,

		// Ports of Axi Master Bus Interface M00_AXIS
		input wire  m00_axis_aclk, m00_axis_aresetn,
		input wire  m00_axis_tready,
		output logic  m00_axis_tvalid, m00_axis_tlast,
		output logic signed [C_M00_AXIS_TDATA_WIDTH-1 : 0] m00_axis_tdata, // (vhz, vhy, vhx) (Q1.31 signed)
		output logic [(C_S00_AXIS_TDATA_WIDTH/8)-1: 0] m00_axis_tstrb,

    /// ADDITIONAL OUTPUTS
    output logic signed [C_M00_AXIS_TDATA_WIDTH-1:0] T1, // {t1z, t1y, t1x}
    output logic signed [C_M00_AXIS_TDATA_WIDTH-1:0] T2  // {t2z, t2y, t2x}
	);

	localparam logic signed [FRAC_BITS-1:0] ONE_Q1 		 = 32'h7FFF_FFFF; // 1 in Q1.31
	localparam logic signed [FRAC_BITS-1:0] NEG_ONE_Q1 = 32'h8000_0000; // -1
	localparam logic 		[FRAC_BITS-1:0] ONE_Q0 		 = 32'hFFFF_FFFF; // 1 in UQ0.32
	localparam logic 		[FRAC_BITS-1:0] UQ0_32_MIN = 32'h0002_0000; // XMIN = 2^-15

	wire 				[31:0] alpha_in  = s00_axis_tdata[127:96];
	wire signed [31:0] view_z_in = s00_axis_tdata[95:64];
	wire signed [31:0] view_y_in = s00_axis_tdata[63:32];
	wire signed [31:0] view_x_in = s00_axis_tdata[31:0];

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

  function automatic logic signed [24:0] rnd_s25(input logic signed [31:0] a);
    logic signed [24:0] t;
    begin
      t = $signed(a[31:7]);
      rnd_s25 = (a[6] && t != 25'sh0FFFFFF) ? t + 25'sh0000001 : t;
    end
  endfunction

  function automatic logic signed [24:0] rnd_u25(input logic [31:0] u);
    logic [23:0] t;
    begin
      t = u[31:8];
      rnd_u25 = $signed({1'b0, ((u[7] && t != 24'hFFFFFF) ? t + 24'h000001 : t)});
    end
  endfunction

	// Quantized to a single DSP48E1 (18b x 25b) with round-half-up operands, so
	// the product needs no fabric cascade. Q1.31 x UQ0.32 -> Q1.31.
	function automatic logic signed [31:0] mul_q131_uq032_to_q131(
		input logic signed [31:0] q131,
		input logic 			 [31:0] uq032
	);
		logic signed [17:0] q131_18;    // Q1.17
		logic signed [24:0] uq032_25;   // UQ0.24 (positive)
		logic signed [42:0] prod_q1_41; // Q1.41
		begin
				q131_18    = rnd_s18(q131);
				uq032_25   = rnd_u25(uq032);
				prod_q1_41 = q131_18 * uq032_25;                 // Q1.17 * UQ0.24 -> Q1.41
				// Round-half-up on the product too: a bare >>> truncates toward -inf and
			// reintroduces the same systematic bias the operand rounding removes.
			// The constant add maps to the DSP48E1 C port, so it costs no fabric.
			mul_q131_uq032_to_q131 = 32'((prod_q1_41 + 43'sh200) >>> 10); // Q1.41 -> Q1.31
		end
	endfunction

	// Operands ALREADY narrowed+rounded upstream (registered), clean reg -> DSP.
	// The `>>>10` (with round constant) is a constant shift = free bit-select.
	function automatic logic signed [31:0] mul_pre_q131_uq032_to_q131(
		input logic signed [17:0] q131_18,
		input logic signed [24:0] uq032_25
	);
		logic signed [42:0] prod_q1_41;
		begin
			prod_q1_41 = q131_18 * uq032_25;
			mul_pre_q131_uq032_to_q131 = 32'((prod_q1_41 + 43'sh200) >>> 10);
		end
	endfunction

	// Product carried at its natural 43-bit Q2.41 (was zero-inflated to Q2.62; the
	// low 21 bits were always zero). Operands are ALREADY narrowed+rounded upstream
	// (registered), so this is a clean reg -> DSP. Consumers (sat_shift31_q241, the
	// z2 bit-slices) re-index by -21. Shrinks the t2a / z2 registers + their nets.
	function automatic logic signed [42:0] mul_pre_q131_q131_to_q241(
		input logic signed [17:0] a_18,
		input logic signed [24:0] b_25
	);
		begin
			mul_pre_q131_q131_to_q241 = a_18 * b_25; // Q1.17 * Q1.24 -> Q2.41
		end
	endfunction

	// Operands ALREADY narrowed+rounded upstream (registered), clean reg -> DSP.
	function automatic logic signed [63:0] mul_pre_q131_q725_to_q856(
		input logic signed [17:0] a_18,
		input logic signed [24:0] b_25
	);
		logic signed [42:0] prod_q8_34;
		begin
			prod_q8_34 = a_18 * b_25;
			mul_pre_q131_q725_to_q856 = 64'(prod_q8_34) <<< 22;
		end
	endfunction

	function automatic logic signed [63:0] mul_q131_q725_to_q856(
		input logic signed [31:0] q131,
		input logic 			 [31:0] q725
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
  
	function automatic logic signed [63:0] round_shift_right_signed(
		input logic signed [63:0] val_wide,
		input int unsigned sh
	);
		logic signed [63:0] add;
		begin
			if (sh==0) round_shift_right_signed = val_wide;
			else begin
				add = (val_wide >>> (sh-1)) & 64'sd1; // "round half up"
				round_shift_right_signed = (val_wide >>> sh) + add;
			end
		end
	endfunction

	function automatic logic signed [31:0] satq131(input logic signed [63:0] val);
    begin
      if (val > $signed(64'sh0000_0000_7FFF_FFFF)) satq131 = ONE_Q1;
      else if (val < $signed(64'shFFFF_FFFF_8000_0000)) satq131 = NEG_ONE_Q1;
      else satq131 = val[31:0];
    end
  endfunction

  // Fast round-shift-right-by-31 + saturate for Q2.62 -> Q1.31.
  // Uses val64[63:62] 2-bit overflow decode instead of 64-bit comparison.
  // val64[63:62]==01 -> positive overflow -> ONE_Q1
  // val64[63:62]==10 -> negative overflow -> NEG_ONE_Q1
  // else: 33-bit add for rounding, overflow check on sign bits.
  function automatic logic signed [31:0] sat_shift31(input logic signed [63:0] val64);
    logic [32:0] r;
    begin
      r = {val64[63], val64[62:31]} + 33'(val64[30]);
      if (r[32] != r[31])
        sat_shift31 = r[32] ? NEG_ONE_Q1 : ONE_Q1;
      else
        sat_shift31 = r[31:0];
    end
  endfunction

  // Q2.41 (44-bit, one guard bit above Q2.41 so the t2a subtraction can't wrap)
  // -> Q1.31, saturating. Equivalent to sat_shift31 with indices -21: v>>10 is
  // the old val64>>31, v[9] the old round bit [30]. Bit-identical to sat_shift31
  // for every non-overflow value, and saturates correctly at the boundary.
  localparam logic signed [33:0] Q131_MAX_34 =  34'sd2147483647; //  2^31 - 1
  localparam logic signed [33:0] Q131_MIN_34 = -34'sd2147483648; // -2^31
  // Split out of the old sat_shift31_q241 (issue #17): the 34-bit rounding add
  // and the two 34-bit saturating compares used to share a cycle, ~9 CARRY4
  // back to back. Stage 5c now does the add, stage 5d the saturate.
  function automatic logic signed [33:0] rnd_shift31_q241(input logic signed [43:0] v);
    begin
      rnd_shift31_q241 = $signed(v[43:10]) + $signed({33'b0, v[9]});
    end
  endfunction

  function automatic logic signed [31:0] sat34_to_q131(input logic signed [33:0] s);
    begin
      if      (s > Q131_MAX_34) sat34_to_q131 = ONE_Q1;
      else if (s < Q131_MIN_34) sat34_to_q131 = NEG_ONE_Q1;
      else                      sat34_to_q131 = s[31:0];
    end
  endfunction

	function automatic logic [31:0] clamp_uq032_min(input logic [31:0] x);
		clamp_uq032_min = (x > UQ0_32_MIN) ? x : UQ0_32_MIN;
	endfunction

	logic pipe_en;
	assign pipe_en = m00_axis_tready || !m00_axis_tvalid;

	wire norm_in_ready;
	wire norm_out_valid;
	wire norm_en = pipe_en && norm_in_ready;

	assign s00_axis_tready = norm_en;

	wire inv_in_ready;
  wire inv_out_valid;
	logic [31:0] inv_len_q7_25;

	/// STAGE 0R: pre-round the warp multiply operands into registers so stage 0's
	/// DSP multiply is a clean reg -> DSP path. Shares the norm_en enable, so the
	/// whole front (s0r, s0) stalls together with no data loss; adds one cycle.
	logic s0r_valid;
	logic signed [17:0] vx_r18, vy_r18;
	logic signed [24:0] alpha_r25;
	logic signed [31:0] vz_0r;
	always_ff @(posedge s00_axis_aclk) begin
		if (s00_axis_aresetn==0) begin
			s0r_valid <= 1'b0;
			vx_r18 <= '0; vy_r18 <= '0; alpha_r25 <= '0; vz_0r <= '0;
		end else if (norm_en) begin
			s0r_valid <= s00_axis_tvalid;
			if (s00_axis_tvalid) begin
				vx_r18    <= rnd_s18(view_x_in);
				vy_r18    <= rnd_s18(view_y_in);
				alpha_r25 <= rnd_u25(alpha_in);
				vz_0r     <= view_z_in;
			end
		end
	end

	/// STAGE 0: WARP VIEW VECTOR
	logic s0_valid;
	logic signed [31:0] vx_scaled, vy_scaled, vz_00; // Q1.31
	always_ff @(posedge s00_axis_aclk) begin
		if (s00_axis_aresetn==0) begin
			s0_valid <= 1'b0;
			vx_scaled <= '0; vy_scaled <= '0; vz_00 <= '0;
		end else if (norm_en) begin
			s0_valid <= s0r_valid;
			if (s0r_valid) begin
				vx_scaled <= mul_pre_q131_uq032_to_q131(vx_r18, alpha_r25);
				vy_scaled <= mul_pre_q131_uq032_to_q131(vy_r18, alpha_r25);
				vz_00 		<= vz_0r;
			end
		end
	end

	/// STAGE 1: NORMALIZE WARPED VIEW VECTOR w/ axis_fixed_norm3
	logic [127:0] norm_out;
	wire signed [31:0] vh_z = $signed(norm_out[95:64]);
	wire signed [31:0] vh_y = $signed(norm_out[63:32]);
	wire signed [31:0] vh_x = $signed(norm_out[31:0]);
	// ---------------------------------------------------------------------
	// HANDSHAKE CHAIN (hs_*)
	//
	// Named hs_* on purpose. The T2 arithmetic band further down this same
	// file uses t2a_*/t2b_*, and the handshake stages used to be s2a_*/s2b_*
	// -- one character apart, in a 630-line file, which has already caused
	// real confusion about which band holds the critical path.
	//
	// Four uniform elastic register stages: zop -> sq -> sub -> clamp. A stage
	// may accept when it is empty, or when its successor is accepting this
	// cycle. Nothing here observes pipe_en. Backpressure arrives through
	// inv_in_ready, which the folded inverse-sqrt drops while busy and which
	// goes low when that engine's own output stalls against pipe_en. One
	// discipline, propagating one way.
	//
	// The old chain read:
	//     stage_2a_ready = stage_2b_ready || !s2a_valid_reg
	// With pipe_en low, stage_2b_ready was low, so this reduced to "stage 2a
	// is empty" and asserted anyway. Stage 2 -- not gated by pipe_en -- then
	// cleared its valid bit, while stage 2a -- which was gated -- never
	// latched the payload. The beat vanished with no error and no stall.
	// Issue #11 has the test that reaches it: 6 items in, 2 out.
	//
	// Do NOT reintroduce that bypass term to save a cycle. With one basis in
	// flight it never saved one; it was pure risk.
	// ---------------------------------------------------------------------
	logic hs_zop_valid, hs_sq_valid, hs_sub_valid, hs_clamp_valid;

	wire hs_clamp_ready = !hs_clamp_valid || inv_in_ready;
	wire hs_sub_ready   = !hs_sub_valid   || hs_clamp_ready;
	wire hs_sq_ready    = !hs_sq_valid    || hs_sub_ready;
	wire hs_zop_ready   = !hs_zop_valid   || hs_sq_ready;

	// A load is also exactly the condition under which the PRECEDING stage's
	// payload leaves, so each stage clears on its successor's load.
	wire hs_zop_load   = norm_out_valid && hs_zop_ready;
	wire hs_sq_load    = hs_zop_valid   && hs_sq_ready;
	wire hs_sub_load   = hs_sq_valid    && hs_sub_ready;
	wire hs_clamp_load = hs_sub_valid   && hs_clamp_ready;

	// event_basis is once-per-burst: fold this norm3's inv_sqrt (bit-identical,
	// removes its scattered sideband delay lines = the biggest route-bound bucket).
	axis_fixed_norm3 #(.FOLD_INVSQRT(1)) normalize_warped_view (
		.s00_axis_aclk(s00_axis_aclk),
    .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(1'b0),
    .s00_axis_tvalid(s0_valid),
    .s00_axis_tdata({32'b0, vz_00, vy_scaled, vx_scaled}),
    .s00_axis_tstrb('1),
    .s00_axis_tready(norm_in_ready),

    .m00_axis_aclk(s00_axis_aclk),
    .m00_axis_aresetn(s00_axis_aresetn),
    .m00_axis_tlast(),
    .m00_axis_tvalid(norm_out_valid),
    .m00_axis_tdata(norm_out),
    .m00_axis_tstrb(),
    .m00_axis_tready(hs_zop_ready)
	);

	/// STAGE ZOP payload: the ROUNDED z operands, registered ahead of the z^2
	/// square so the square is a clean reg -> DSP -> reg multiply instead of
	/// norm3_out -> rnd_s25 (5x CARRY4) -> DSP. Costs one cycle. Its handshake
	/// is the hs_* chain above, the same one every stage here uses.
	logic signed [17:0] vhz_r18;
	logic signed [24:0] vhz_r25;
	logic [95:0] Vh_2r;

	/// STAGE 2: COMPUTE LENGTH SQAURED (vhx^2 + vhy^2)
	logic signed [42:0] z2_q241;   // vhz^2, Q2.41 (was Q2.62 with 21 dead low bits)
	logic [95:0] Vh_20;

	wire [31:0] z2_uq032 = z2_q241[40:9];   // old [61:30], -21
	// wire [31:0] lensq = (z2_uq032 >= ONE_Q0) ? 32'b0 : (ONE_Q0 - z2_uq032);
	wire [31:0] lensq = z2_q241[41] ? 32'b0 : (ONE_Q0 - z2_uq032);
	wire lensq_condition = lensq < UQ0_32_MIN || z2_q241[41];

	// Payload registers for the clamp stage (was "stage 2b")
	logic [31:0] lensq_xy_reg;
	logic        lensq_branch_reg;
	logic [95:0] Vh_21_reg;

	// Payload registers for the sub stage (was "stage 2a")
	logic [31:0] lensq_sub_reg;
	logic        lensq_condition_reg;
	logic [95:0] Vh_21a_reg;

	// STAGE ZOP -- register the rounded z operands (the rounding rides
	// norm3_out -> reg, keeping it off the DSP path).
	always_ff @(posedge s00_axis_aclk) begin
		if (s00_axis_aresetn==0) begin
			hs_zop_valid <= 1'b0;
			vhz_r18 <= '0; vhz_r25 <= '0;
			Vh_2r <= '0;
		end else begin
			if (hs_zop_load) begin
				hs_zop_valid <= 1'b1;
				vhz_r18 <= rnd_s18(vh_z);
				vhz_r25 <= rnd_s25(vh_z);
				Vh_2r   <= norm_out[95:0];
			end else if (hs_sq_load) begin
				hs_zop_valid <= 1'b0;
			end
		end
	end

	// STAGE SQ -- square the pre-rounded operand (clean reg -> DSP -> reg)
	always_ff @(posedge s00_axis_aclk) begin
		if (s00_axis_aresetn==0) begin
			hs_sq_valid <= 1'b0;
			Vh_20			<= '0;
			z2_q241 	<= '0;
		end else begin
			if (hs_sq_load) begin
				hs_sq_valid <= 1'b1;
				z2_q241 <= mul_pre_q131_q131_to_q241(vhz_r18, vhz_r25);
				Vh_20		<= Vh_2r;
			end else if (hs_sub_load) begin
				hs_sq_valid <= 1'b0;
			end
		end
	end

	// STAGE SUB -- 1 - z^2
	always_ff @(posedge s00_axis_aclk) begin
		if (s00_axis_aresetn == 0) begin
			hs_sub_valid        <= 1'b0;
			lensq_sub_reg       <= '0;
			lensq_condition_reg <= 1'b0;
			Vh_21a_reg          <= '0;
		end else begin
			if (hs_sub_load) begin
				hs_sub_valid        <= 1'b1;
				lensq_sub_reg       <= z2_q241[41] ? 32'b0 : (ONE_Q0 - z2_uq032);
				lensq_condition_reg <= lensq_condition;
				Vh_21a_reg          <= Vh_20;
			end else if (hs_clamp_load) begin
				hs_sub_valid <= 1'b0;
			end
		end
	end

	// STAGE CLAMP -- clamp to the inverse-sqrt's minimum argument
	always_ff @(posedge s00_axis_aclk) begin
		if (s00_axis_aresetn == 0) begin
			hs_clamp_valid   <= 1'b0;
			lensq_xy_reg     <= '0;
			lensq_branch_reg <= 1'b0;
			Vh_21_reg        <= '0;
		end else begin
			if (hs_clamp_load) begin
				hs_clamp_valid   <= 1'b1;
				lensq_xy_reg     <= clamp_uq032_min(lensq_sub_reg);
				lensq_branch_reg <= lensq_condition_reg;
				Vh_21_reg        <= Vh_21a_reg;
			end else if (hs_clamp_valid && inv_in_ready) begin
				hs_clamp_valid <= 1'b0;
			end
		end
	end

	wire lensq_branch = lensq_branch_reg;
	wire [31:0] lensq_xy = lensq_xy_reg;
	wire [95:0] Vh_21 = Vh_21_reg;
 
	wire [31:0] delayed_Vh_x, delayed_Vh_y, delayed_Vh_z;
	wire [3:0] delayed_shift;
	wire [95:0] delayed_Vh = {delayed_Vh_z, delayed_Vh_y, delayed_Vh_x};
	wire use_inv_sqrt = ~delayed_shift[0];
 
	/// STAGE 3: 1 / SQRT(LENSQ). event_basis is once-per-burst, so use the FOLDED
	/// (sequential) inv_sqrt: bit-identical to the pipelined _nodsp but ~75x fewer
	/// FFs and no scattered sideband delay lines -- the dominant route-bound
	/// congestion in this block. inv_in_ready is the head of the hs_* elastic
	/// chain above, so the folded engine's busy-stall backpressures cleanly all
	/// the way to norm3's output handshake.
	axis_fixed_inv_sqrt_folded # (
    .FRAC_BITS(FRAC_BITS),
    .ADDR_BITS(14)
  ) u_inv_sqrt (
    .s00_axis_aclk(s00_axis_aclk),
    .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(1'b0),
    .s00_axis_tvalid(hs_clamp_valid),
    .s00_axis_tdata(lensq_xy),
    .s00_axis_tstrb('1),
    .s00_axis_tready(inv_in_ready),
    
    .s00_axis_user_x(Vh_21[31:0]),
    .s00_axis_user_y(Vh_21[63:32]),
    .s00_axis_user_z(Vh_21[95:64]),
    .s00_axis_user_shift({3'b0, lensq_branch}),

    .m00_axis_aclk(s00_axis_aclk),
    .m00_axis_aresetn(s00_axis_aresetn),
    .m00_axis_tlast(),
    .m00_axis_tvalid(inv_out_valid),
    .m00_axis_tdata(inv_len_q7_25),
    .m00_axis_tstrb(),
    .m00_axis_tready(pipe_en),
    
    .m00_axis_user_x(delayed_Vh_x),
    .m00_axis_user_y(delayed_Vh_y),
    .m00_axis_user_z(delayed_Vh_z),
    .m00_axis_user_shift(delayed_shift)
  );

	/// STAGE 3R: pre-round the stage-4a multiply operands (from the inv_sqrt
	/// output) into registers, so 4a's DSP multiply is a clean reg -> DSP path.
	/// Adds one pipeline cycle; Vh and use_inv_sqrt ride through it so everything
	/// downstream stays aligned.
	logic s3r_valid;
	logic use_inv_sqrt_3r;
	logic [95:0] Vh_3r;
	logic signed [17:0] vhy_r18_3r, vhx_r18_3r;
	logic signed [24:0] invlen_r25_3r;
	always_ff @(posedge s00_axis_aclk) begin
		if (s00_axis_aresetn==0) begin
			s3r_valid <= 1'b0;
			use_inv_sqrt_3r <= 1'b0;
			Vh_3r <= '0;
			vhy_r18_3r <= '0; vhx_r18_3r <= '0; invlen_r25_3r <= '0;
		end else if (pipe_en) begin
			s3r_valid <= inv_out_valid;
			if (inv_out_valid) begin
				use_inv_sqrt_3r <= use_inv_sqrt;
				Vh_3r <= delayed_Vh;
				vhy_r18_3r    <= rnd_s18($signed(delayed_Vh[63:32]));
				vhx_r18_3r    <= rnd_s18($signed(delayed_Vh[31:0]));
				invlen_r25_3r <= rnd_u25(inv_len_q7_25);
			end
		end
	end

	/// STAGE 4: FORM T1
	logic s4a_valid, s4b_valid, s4c_valid;
	logic use_inv_sqrt_40, use_inv_sqrt_41, use_inv_sqrt_42;
	logic signed [63:0] t1a_x, t1a_y;
	logic signed [63:0] t1b_x_shift, t1b_y_shift;
	logic signed [31:0] t1c_x, t1c_y;
	// Pre-rounded, pre-narrowed operands for the stage-5a DSP multiplies, computed
	// here in 4c (where t1c / Vh_42 are already being registered) so the rounding
	// carry chain does not sit on the reg -> DSP path in 5a. No added latency.
	logic signed [24:0] t1c_x_r25, t1c_y_r25;
	logic signed [17:0] vh42_z_r18, vh42_x_r18, vh42_y_r18;
	logic [95:0] Vh_40, Vh_41, Vh_42;
	// STAGE 4D (issue #17): splits satq131 from rnd_s25. Both used to run in 4c
	// on the same cycle -- a 64-bit saturating compare feeding a 25-bit rounding
	// increment, then straight into the DSP A port (Vivado absorbs t1c_*_r25 as
	// the DSP's AREG). That is two carry chains and the DSP setup in one cycle,
	// and it was the block's WNS holder at -2.139. Now 4c registers the
	// saturated value and 4d rounds it from a 32-bit register.
	// event_basis is PER-BURST, so the extra cycle costs nothing.
	logic s4d_valid;
	logic use_inv_sqrt_43;
	logic [95:0] Vh_43;
	logic signed [31:0] t1c_x_d, t1c_y_d;
	logic signed [17:0] vh43_z_r18, vh43_x_r18, vh43_y_r18;
	always_ff @(posedge s00_axis_aclk) begin
		if (s00_axis_aresetn==0) begin
			s4a_valid <= 1'b0;
			s4b_valid <= 1'b0;
			s4c_valid <= 1'b0;
			use_inv_sqrt_40 <= 1'b0;
			use_inv_sqrt_41 <= 1'b0;
			use_inv_sqrt_42 <= 1'b0;
			t1a_x <= '0; t1a_y <= '0;
			t1b_x_shift <= '0; t1b_y_shift <= '0;
			t1c_x <= '0; t1c_y <= '0;
			t1c_x_r25 <= '0; t1c_y_r25 <= '0;
			vh42_z_r18 <= '0; vh42_x_r18 <= '0; vh42_y_r18 <= '0;
			Vh_40 <= '0; Vh_41 <= '0; Vh_42 <= '0;
			s4d_valid <= 1'b0;
			use_inv_sqrt_43 <= 1'b0;
			Vh_43 <= '0;
			t1c_x_d <= '0; t1c_y_d <= '0;
			vh43_z_r18 <= '0; vh43_x_r18 <= '0; vh43_y_r18 <= '0;
		end else if (pipe_en) begin
			s4a_valid <= s3r_valid;
			if (s3r_valid) begin
				use_inv_sqrt_40 <= use_inv_sqrt_3r;
				Vh_40 <= Vh_3r;
				if (use_inv_sqrt_3r) begin
					t1a_x <= mul_pre_q131_q725_to_q856(vhy_r18_3r, invlen_r25_3r);
					t1a_y <= mul_pre_q131_q725_to_q856(vhx_r18_3r, invlen_r25_3r);
				end
			end

			s4b_valid <= s4a_valid;
			if (s4a_valid) begin
				use_inv_sqrt_41 <= use_inv_sqrt_40;
				Vh_41 <= Vh_40;
				if (use_inv_sqrt_40) begin
					t1b_x_shift <= round_shift_right_signed(-t1a_x,23);
					t1b_y_shift <= round_shift_right_signed(t1a_y,23);
				end
			end

			s4c_valid <= s4b_valid;
			if (s4b_valid) begin
				use_inv_sqrt_42 <= use_inv_sqrt_41;
				Vh_42 <= Vh_41;
				// Round Vh_41's three components into the 18-bit DSP operands here,
				// off the 5a reg -> DSP path (Vh_42 <= Vh_41, so round Vh_41).
				vh42_z_r18 <= rnd_s18($signed(Vh_41[95:64]));
				vh42_x_r18 <= rnd_s18($signed(Vh_41[31:0]));
				vh42_y_r18 <= rnd_s18($signed(Vh_41[63:32]));
				if (use_inv_sqrt_41) begin
					// Saturate only. The rounding that used to sit here, on the same
					// cycle and on the same 64-bit value, is now stage 4d.
					t1c_x <= satq131(t1b_x_shift);
					t1c_y <= satq131(t1b_y_shift);
				end
			end

			// STAGE 4D: round the SATURATED, registered 32-bit value into the
			// 25-bit DSP operand. Everything 5a consumes is carried one more
			// stage so it stays aligned.
			s4d_valid <= s4c_valid;
			if (s4c_valid) begin
				use_inv_sqrt_43 <= use_inv_sqrt_42;
				Vh_43 <= Vh_42;
				vh43_z_r18 <= vh42_z_r18;
				vh43_x_r18 <= vh42_x_r18;
				vh43_y_r18 <= vh42_y_r18;
				if (use_inv_sqrt_42) begin
					t1c_x_d <= t1c_x;
					t1c_y_d <= t1c_y;
					t1c_x_r25 <= rnd_s25(t1c_x);
					t1c_y_r25 <= rnd_s25(t1c_y);
				end
			end
		end
	end

	/// STAGE 5: FORM T2
	// 5a: DSP multiplies (t2a_*)
	// 5b: register negation/subtraction (breaks 16-CARRY4 chain from DSP PREG)
	// 5c: sat_shift31 (fast 2-bit decode, ~9 CARRY4)
	logic s5a_valid, s5b_valid, s5c_valid;
	logic use_inv_sqrt_50, use_inv_sqrt_5b;
	logic [95:0] T1_out0, T1_out_5b, T1_out_final;
	logic signed [42:0] t2a_x, t2a_y, t2a_z0, t2a_z1;          // Q2.41 (was Q2.62)
	logic signed [43:0] t2b_x_r, t2b_y_r, t2b_z_r; // Q2.41 + 1 guard bit (subtraction can't wrap)
	logic signed [31:0] t2b_x, t2b_y, t2b_z;
	logic [95:0] Vh_50, Vh_5b, Vh_51;
	// STAGE 5D (issue #17): the saturate, split off the rounding add above.
	logic s5d_valid;
	logic use_inv_sqrt_5c;
	logic signed [33:0] t2b_x_s, t2b_y_s, t2b_z_s;
	logic [95:0] Vh_52;
	logic [95:0] T1_out_final2;
	always_ff @(posedge s00_axis_aclk) begin
		if (s00_axis_aresetn==0) begin
			s5a_valid <= 1'b0;
			s5b_valid <= 1'b0;
			s5c_valid <= 1'b0;
			use_inv_sqrt_50 <= 1'b0;
			use_inv_sqrt_5b <= 1'b0;
			T1_out0 <= '0; T1_out_5b <= '0; T1_out_final <= '0;
			t2a_x <= '0; t2a_y <= '0;
			t2a_z0 <= '0; t2a_z1 <= '0;
			t2b_x_r <= '0; t2b_y_r <= '0; t2b_z_r <= '0;
			t2b_x <= '0; t2b_y <= '0; t2b_z <= '0;
			Vh_50 <= '0; Vh_5b <= '0; Vh_51 <= '0;
			s5d_valid <= 1'b0; use_inv_sqrt_5c <= 1'b0;
			t2b_x_s <= '0; t2b_y_s <= '0; t2b_z_s <= '0;
			Vh_52 <= '0; T1_out_final2 <= '0;
		end else if (pipe_en) begin
			// Stage 5a: DSP multiplies
			s5a_valid <= s4d_valid;
			if (s4d_valid) begin
				Vh_50 <= Vh_43;
				use_inv_sqrt_50 <= use_inv_sqrt_43;
				if (use_inv_sqrt_43) begin
					t2a_x  <= mul_pre_q131_q131_to_q241(vh43_z_r18, t1c_y_r25);
					t2a_y  <= mul_pre_q131_q131_to_q241(vh43_z_r18, t1c_x_r25);
					t2a_z0 <= mul_pre_q131_q131_to_q241(vh43_x_r18, t1c_y_r25);
					t2a_z1 <= mul_pre_q131_q131_to_q241(vh43_y_r18, t1c_x_r25);
					T1_out0 <= {32'b0, t1c_y_d, t1c_x_d};
				end else begin
					T1_out0 <= {32'b0, 32'b0, ONE_Q1};
				end
			end

			// Stage 5b: register negation/subtraction (breaks CARRY4 chain from DSP PREG)
			s5b_valid <= s5a_valid;
			if (s5a_valid) begin
				Vh_5b <= Vh_50;
				T1_out_5b <= T1_out0;
				use_inv_sqrt_5b <= use_inv_sqrt_50;
				if (use_inv_sqrt_50) begin
					// Sign-extend the Q2.41 products to 44 bits before negate/subtract so
					// the results cannot overflow (t2a_z0 - t2a_z1 can reach the Q2.41 edge).
					t2b_x_r <= -$signed({t2a_x[42], t2a_x});
					t2b_y_r <=  $signed({t2a_y[42], t2a_y});
					t2b_z_r <=  $signed({t2a_z0[42], t2a_z0}) - $signed({t2a_z1[42], t2a_z1});
				end
			end

			// Stage 5c: the 34-bit rounding add only.
			s5c_valid <= s5b_valid;
			if (s5b_valid) begin
				Vh_51 <= Vh_5b;
				T1_out_final <= T1_out_5b;
				use_inv_sqrt_5c <= use_inv_sqrt_5b;
				t2b_x_s <= rnd_shift31_q241(t2b_x_r);
				t2b_y_s <= rnd_shift31_q241(t2b_y_r);
				t2b_z_s <= rnd_shift31_q241(t2b_z_r);
			end

			// Stage 5d: the saturating compares, off the adder's carry chain.
			s5d_valid <= s5c_valid;
			if (s5c_valid) begin
				Vh_52 <= Vh_51;
				T1_out_final2 <= T1_out_final;
				if (use_inv_sqrt_5c) begin
					t2b_x <= sat34_to_q131(t2b_x_s);
					t2b_y <= sat34_to_q131(t2b_y_s);
					t2b_z <= sat34_to_q131(t2b_z_s);
				end else begin
					t2b_x <= '0;
					t2b_y <= ONE_Q1;
					t2b_z <= '0;
				end
			end
		end
	end

	assign m00_axis_tvalid = s5d_valid;
	assign m00_axis_tdata  = Vh_52;
	assign T1							 = T1_out_final2;
	assign T2							 = {t2b_z, t2b_y, t2b_x};
	assign m00_axis_tstrb  = '1;
	assign m00_axis_tlast  = 1'b0;


endmodule

`default_nettype wire
