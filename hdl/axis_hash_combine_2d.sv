`timescale 1ns / 1ps
`default_nettype none

module axis_hash_combine_2d #
  (
		parameter integer C_S00_AXIS_TDATA_WIDTH	= 64,
		parameter integer C_M00_AXIS_TDATA_WIDTH	= 64,
		parameter integer DIMENSION								= 0
	)
	(
		// Ports of Axi Slave Bus Interface S00_AXIS
		input wire  s00_axis_aclk, s00_axis_aresetn,
		input wire  s00_axis_tlast, s00_axis_tvalid,
		input wire [C_S00_AXIS_TDATA_WIDTH-1 : 0] s00_axis_tdata, // {pass, seed_base}
		input wire [(C_S00_AXIS_TDATA_WIDTH/8)-1: 0] s00_axis_tstrb,
		output logic  s00_axis_tready,

		// Ports of Axi Master Bus Interface M00_AXIS
		input wire  m00_axis_aclk, m00_axis_aresetn,
		input wire  m00_axis_tready,
		output logic  m00_axis_tvalid, m00_axis_tlast,
		output logic [C_M00_AXIS_TDATA_WIDTH-1 : 0] m00_axis_tdata, // {pass, seed}
		output logic [(C_M00_AXIS_TDATA_WIDTH/8)-1: 0] m00_axis_tstrb
	);

	localparam H0 	= 32'h9747b28c;
	localparam MIX0 = 32'hcc9e2d51;
	localparam MIX1 = 32'h1b873593;
	localparam MIX2 = 32'he6546b64;
	localparam F0		= 32'h85ebca6b;
	localparam F1		= 32'hc2b2ae35;
	localparam [31:0] SECOND_MIX0 = DIMENSION * MIX0;
	localparam [31:0] SECOND_MIX1 = rotl32(SECOND_MIX0, 32'd15);
	localparam [31:0] SECOND_MIX2 = SECOND_MIX1 * MIX1;

	function automatic [31:0] rotl32(
		input [31:0] x,
		input [31:0] r
	);
		begin
			rotl32 = (x<<r) | (x>>(32'd32 - r));
		end
	endfunction

	// Bit-exact 32x32 constant multiply, split into 16-bit halves so that every
	// partial product is a single 16x16 DSP with no cascade. A full 32x32 needs a
	// 4-DSP cascade whose A->PCOUT delay is 4.21 ns at zero logic levels, which no
	// amount of pipelining can shorten. Truncating operands is NOT an option here:
	// a hash depends on every input bit reaching every output bit.
	//
	//   x*C = x_lo*C_lo + ((x_lo*C_hi + x_hi*C_lo) << 16)   (mod 2^32)
	//
	// MurmurHash is mod 2^32, so the x_hi*C_hi term is dropped -- it only affects
	// bits >= 32 -- and the middle terms need just their low 16 bits.
	// cmul_pp feeds an existing multiplier register slot and cmul_sum the stage
	// after it, so the 2-cycle multiply latency is unchanged.
	function automatic [63:0] cmul_pp(input [31:0] x, input [31:0] c);
		logic [31:0] p_ll, p_lh, p_hl;
		begin
			p_ll = x[15:0]  * c[15:0];   // full 32 bits used
			p_lh = x[15:0]  * c[31:16];  // low 16 bits used
			p_hl = x[31:16] * c[15:0];   // low 16 bits used
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

	logic stall, advance;
  always_comb begin
    stall  = m00_axis_tvalid && !m00_axis_tready;
    advance    = !stall;
    s00_axis_tready = !stall;
  end

	logic [17:0] valid_pipeline;
	logic [17:0] last_pipeline;

	logic [31:0] first_mix_pipe  [0:4];
	logic [31:0] second_mix_pipe [0:4];
	(* keep = "true" *) logic [31:0] final_mix_pipe  [0:5];
	logic [31:0] final_mix_pipe_xor2;
	logic [31:0] final_mix_pipe_xor3;

	// Intermediate multiplier registers for 2-cycle pipelined multipliers
	logic [31:0] tdata_reg;
	logic [63:0] first_mix_pipe_m0;  // packed partial products {p_hl[15:0], p_lh[15:0], p_ll[31:0]}
	logic [63:0] first_mix_pipe_m2;
	logic [63:0] final_mix_pipe_m2;
	logic [63:0] final_mix_pipe_m3;

	always_ff @(posedge s00_axis_aclk) begin
		if (s00_axis_aresetn==0) begin
			tdata_reg <= '0;
			for (integer i = 0; i < 5; i = i + 1) begin
				first_mix_pipe[i]  <= '0;
				second_mix_pipe[i] <= '0;
			end
			for (integer i = 0; i < 6; i = i + 1) final_mix_pipe[i] <= '0;
			final_mix_pipe_xor2 <= '0;
			final_mix_pipe_xor3 <= '0;
			first_mix_pipe_m0   <= '0;
			first_mix_pipe_m2   <= '0;
			final_mix_pipe_m2   <= '0;
			final_mix_pipe_m3   <= '0;
			for (integer i = 0; i <= 17; i = i + 1) begin
				valid_pipeline[i] <= 1'b0;
				last_pipeline[i]  <= 1'b0;
			end
		end else begin
			if (advance) begin
				valid_pipeline[0] <= s00_axis_tvalid;
				last_pipeline[0]  <= s00_axis_tlast;

				for (integer i = 0; i < 17; i = i + 1) begin
					valid_pipeline[i+1] <= valid_pipeline[i];
					last_pipeline[i+1]  <= last_pipeline[i];
				end

				// MurmurHash3 32-bit block mix
				tdata_reg          <= s00_axis_tdata[31:0];
				first_mix_pipe_m0  <= cmul_pp(tdata_reg, MIX0);
				first_mix_pipe[0]  <= cmul_sum(first_mix_pipe_m0);
				second_mix_pipe[0] <= SECOND_MIX0;

				first_mix_pipe[1]  <= rotl32(first_mix_pipe[0], 32'd15);
				second_mix_pipe[1] <= SECOND_MIX1;

				first_mix_pipe_m2  <= cmul_pp(first_mix_pipe[1], MIX1);
				first_mix_pipe[2]  <= cmul_sum(first_mix_pipe_m2);
				second_mix_pipe[2] <= SECOND_MIX2;

				first_mix_pipe[3]  <= rotl32(first_mix_pipe[2] ^ H0, 32'd13);
				second_mix_pipe[3] <= second_mix_pipe[2];

				first_mix_pipe[4]  <= (first_mix_pipe[3] + (first_mix_pipe[3] << 2)) + MIX2; // x*5 = x + x<<2
				second_mix_pipe[4] <= second_mix_pipe[3];

				final_mix_pipe[0] <= rotl32(second_mix_pipe[4] ^ first_mix_pipe[4], 32'd13);
				final_mix_pipe[1] <= (final_mix_pipe[0] + (final_mix_pipe[0] << 2)) + MIX2; // x*5 = x + x<<2

				// FMIX32 Pipelined
				final_mix_pipe_xor2 <= final_mix_pipe[1] ^ (final_mix_pipe[1] >> 16);
				final_mix_pipe_m2   <= cmul_pp(final_mix_pipe_xor2, F0);
				final_mix_pipe[2]   <= cmul_sum(final_mix_pipe_m2);
				
				final_mix_pipe_xor3 <= final_mix_pipe[2] ^ (final_mix_pipe[2] >> 13);
				final_mix_pipe_m3   <= cmul_pp(final_mix_pipe_xor3, F1);
				final_mix_pipe[3]   <= cmul_sum(final_mix_pipe_m3);
				
				final_mix_pipe[4]   <= final_mix_pipe[3] ^ (final_mix_pipe[3] >> 16);
				// Alignment reg: brings the data path to 18 regs, matching the
				// 18-deep valid/last pipeline (TDATA and TVALID aligned, as on main).
				final_mix_pipe[5]   <= final_mix_pipe[4];
			end
		end
	end

	always_comb begin
		m00_axis_tdata 	= {32'b0, final_mix_pipe[5]};
		m00_axis_tvalid = valid_pipeline[17];
		m00_axis_tlast 	= last_pipeline[17];
		m00_axis_tstrb 	= '1;
	end

endmodule

`default_nettype wire
