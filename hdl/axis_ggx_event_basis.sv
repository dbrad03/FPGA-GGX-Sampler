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

	localparam int NORM_LATENCY = 10;
	localparam int INVSQRT_LATENCY = 9; // +1 for axis_fixed_inv_sqrt input register stage
	localparam logic signed [FRAC_BITS-1:0] ONE_Q1 		 = 32'h7FFF_FFFF; // 1 in Q1.31
	localparam logic signed [FRAC_BITS-1:0] NEG_ONE_Q1 = 32'h8000_0000; // -1
	localparam logic 		[FRAC_BITS-1:0] ONE_Q0 		 = 32'hFFFF_FFFF; // 1 in UQ0.32
	localparam logic 		[FRAC_BITS-1:0] UQ0_32_MIN = 32'h0002_0000; // XMIN = 2^-15

	wire 				[31:0] alpha_in  = s00_axis_tdata[127:96];
	wire signed [31:0] view_z_in = s00_axis_tdata[95:64];
	wire signed [31:0] view_y_in = s00_axis_tdata[63:32];
	wire signed [31:0] view_x_in = s00_axis_tdata[31:0];

	function automatic logic signed [31:0] mul_q131_uq032_to_q131(
		input logic signed [31:0] q131,
		input logic 			 [31:0] uq032
	);
		logic signed [63:0] prod_q163;
		logic signed [32:0] uq032_ext;
		begin
				uq032_ext = $signed({1'b0, uq032}); // unsigned -> signed positive
				prod_q163 = $signed(q131) * uq032_ext; // Q1.31 * Q0.32 -> Q1.63
				mul_q131_uq032_to_q131 = prod_q163 >>> 32; // Q1.63 -> Q1.31
		end
	endfunction

	function automatic logic signed [63:0] mul_q131_q131_to_q262(
		input logic signed [31:0] q131_a,
		input logic signed [31:0] q131_b
	);
		begin
				mul_q131_q131_to_q262 = (q131_a * q131_b);
		end
	endfunction

	function automatic logic signed [63:0] mul_q131_q725_to_q856(
		input logic signed [31:0] q131,
		input logic 			 [31:0] q725
	);
		logic signed [32:0] q725_ext;
		begin
				q725_ext = $signed({1'b0, q725});
				mul_q131_q725_to_q856 = $signed(q131) * q725_ext;
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

	/// STAGE 0: WARP VIEW VECTOR
	logic s0_valid;
	logic signed [31:0] vx_scaled, vy_scaled, vz_00; // Q1.31
	always_ff @(posedge s00_axis_aclk) begin
		if (s00_axis_aresetn==0) begin
			s0_valid <= 1'b0;
			vx_scaled <= '0; vy_scaled <= '0; vz_00 <= '0;
		end else if (norm_en) begin
			s0_valid <= s00_axis_tvalid;
			if (s00_axis_tvalid) begin
				vx_scaled <= mul_q131_uq032_to_q131(view_x_in, alpha_in);
				vy_scaled <= mul_q131_uq032_to_q131(view_y_in, alpha_in);
				vz_00 		<= view_z_in;
			end
		end
	end

	/// STAGE 1: NORMALIZE WARPED VIEW VECTOR w/ axis_fixed_norm3
	logic [127:0] norm_out;
	wire signed [31:0] vh_z = $signed(norm_out[95:64]);
	wire signed [31:0] vh_y = $signed(norm_out[63:32]);
	wire signed [31:0] vh_x = $signed(norm_out[31:0]);
	axis_fixed_norm3 normalize_warped_view (
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
    .m00_axis_tready(s2a_ready)
	);

	/// STAGE 2: COMPUTE LENGTH SQAURED (vhx^2 + vhy^2)
	logic s2a_valid;
	logic signed [63:0] z2_q262;
	logic [95:0] Vh_20;

	wire [31:0] z2_uq032 = z2_q262[61:30];
	// wire [31:0] lensq = (z2_uq032 >= ONE_Q0) ? 32'b0 : (ONE_Q0 - z2_uq032);
	wire [31:0] lensq = z2_q262[62] ? 32'b0 : (ONE_Q0 - z2_uq032);
	wire lensq_condition = lensq < UQ0_32_MIN || z2_q262[62];

	wire s2b_advance = pipe_en && s2a_valid && inv_in_ready;
	wire s2a_ready = pipe_en && (!s2a_valid || s2b_advance);
	wire s2a_load  = norm_out_valid && s2a_ready;
	
	always_ff @(posedge s00_axis_aclk) begin
		if (s00_axis_aresetn==0) begin
			s2a_valid <= 1'b0;
			Vh_20			<= '0;
			z2_q262 	<= '0;
		end else begin
			// STAGE 2 -- capture normalized warped vector when we can
			if (s2a_load) begin
				s2a_valid <= 1'b1;
				z2_q262 <= mul_q131_q131_to_q262(vh_z, vh_z);
				Vh_20		<= norm_out[95:0];
			end else if (s2b_advance) begin
				s2a_valid <= 1'b0; // don't need per se
			end
		end
	end

	wire lensq_branch = lensq_condition;
	wire [31:0] lensq_xy = clamp_uq032_min(lensq);
	wire [95:0] Vh_21 = Vh_20;

	/// STAGE 3: 1 / SQRT(LENSQ) w/ axis_fixed_inv_sqrt
	axis_fixed_inv_sqrt # (
    .FRAC_BITS(FRAC_BITS),
    .ADDR_BITS(14)
  ) u_inv_sqrt (
    .s00_axis_aclk(s00_axis_aclk),
    .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(1'b0),
    .s00_axis_tvalid(s2b_advance),
    .s00_axis_tdata(lensq_xy),
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

	// Delay Hemisphere Warped View to Match INV_SQRT LATENCY
	logic [95:0] delay_Vh [0:INVSQRT_LATENCY-1];
	logic [INVSQRT_LATENCY-1:0] delay_lensq_branch;
	always_ff @(posedge s00_axis_aclk) begin
		if (s00_axis_aresetn==0) begin
			delay_lensq_branch <= '0;
			for (integer i = 0; i < INVSQRT_LATENCY; i = i + 1) begin
				delay_Vh[i] <= '0;
			end
		end else if (pipe_en) begin
			delay_lensq_branch <= {delay_lensq_branch[INVSQRT_LATENCY-2:0], (s2b_advance ? lensq_branch : 1'b0)};
			delay_Vh[0] <=  s2b_advance ? Vh_21 : '0;
			for (integer i = 1; i < INVSQRT_LATENCY; i = i + 1) begin
				delay_Vh[i] <= delay_Vh[i-1];
			end
		end
	end
	wire use_inv_sqrt = ~delay_lensq_branch[INVSQRT_LATENCY-1];
	wire [95:0] delayed_Vh 	= delay_Vh[INVSQRT_LATENCY-1];

	/// STAGE 4: FORM T1
	logic s4a_valid, s4b_valid, s4c_valid;
	logic use_inv_sqrt_40, use_inv_sqrt_41, use_inv_sqrt_42;
	logic signed [63:0] t1a_x, t1a_y;
	logic signed [63:0] t1b_x_shift, t1b_y_shift;
	logic signed [31:0] t1c_x, t1c_y;
	logic [95:0] Vh_40, Vh_41, Vh_42;
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
			Vh_40 <= '0; Vh_41 <= '0; Vh_42 <= '0;
		end else if (pipe_en) begin
			s4a_valid <= inv_out_valid;
			if (inv_out_valid) begin
				use_inv_sqrt_40 <= use_inv_sqrt;
				Vh_40 <= delayed_Vh;
				if (use_inv_sqrt) begin
					t1a_x <= mul_q131_q725_to_q856($signed(delayed_Vh[63:32]), inv_len_q7_25);
					t1a_y <= mul_q131_q725_to_q856($signed(delayed_Vh[31:0]), inv_len_q7_25);
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
				if (use_inv_sqrt_41) begin
					t1c_x <= satq131(t1b_x_shift);
					t1c_y <= satq131(t1b_y_shift);
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
	logic signed [63:0] t2a_x, t2a_y, t2a_z0, t2a_z1;
	logic signed [63:0] t2b_x_r, t2b_y_r, t2b_z_r; // registered pre-sat values
	logic signed [31:0] t2b_x, t2b_y, t2b_z;
	logic [95:0] Vh_50, Vh_5b, Vh_51;
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
		end else if (pipe_en) begin
			// Stage 5a: DSP multiplies
			s5a_valid <= s4c_valid;
			if (s4c_valid) begin
				Vh_50 <= Vh_42;
				use_inv_sqrt_50 <= use_inv_sqrt_42;
				if (use_inv_sqrt_42) begin
					t2a_x  <= mul_q131_q131_to_q262($signed(Vh_42[95:64]), t1c_y);
					t2a_y  <= mul_q131_q131_to_q262($signed(Vh_42[95:64]), t1c_x);
					t2a_z0 <= mul_q131_q131_to_q262($signed(Vh_42[31:0]),  t1c_y);
					t2a_z1 <= mul_q131_q131_to_q262($signed(Vh_42[63:32]), t1c_x);
					T1_out0 <= {32'b0, t1c_y, t1c_x};
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
					t2b_x_r <= -t2a_x;
					t2b_y_r <=  t2a_y;
					t2b_z_r <= t2a_z0 - t2a_z1;
				end
			end

			// Stage 5c: sat_shift31 (fast 2-bit decode + 33-bit add, ~9 CARRY4)
			s5c_valid <= s5b_valid;
			if (s5b_valid) begin
				Vh_51 <= Vh_5b;
				T1_out_final <= T1_out_5b;
				if (use_inv_sqrt_5b) begin
					t2b_x <= sat_shift31(t2b_x_r);
					t2b_y <= sat_shift31(t2b_y_r);
					t2b_z <= sat_shift31(t2b_z_r);
				end else begin
					t2b_x <= '0;
					t2b_y <= ONE_Q1;
					t2b_z <= '0;
				end
			end
		end
	end

	assign m00_axis_tvalid = s5c_valid;
	assign m00_axis_tdata  = Vh_51;
	assign T1							 = T1_out_final;
	assign T2							 = {t2b_z, t2b_y, t2b_x};
	assign m00_axis_tstrb  = '1;
	assign m00_axis_tlast  = 1'b0;


endmodule

`default_nettype wire
