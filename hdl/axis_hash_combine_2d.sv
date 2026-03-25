`timescale 1ns / 1ps
`default_nettype none

(* use_dsp = "no" *)
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

	logic stall, advance;
  always_comb begin
    stall  = m00_axis_tvalid && !m00_axis_tready;
    advance    = !stall;
    s00_axis_tready = !stall;
  end

	logic [9:0] valid_pipeline;
	logic [9:0] last_pipeline;

	logic [31:0] first_mix_pipe  [0:4];
	logic [31:0] second_mix_pipe [0:4];
	logic [31:0] final_mix_pipe  [0:4];

	always_ff @(posedge s00_axis_aclk) begin
		if (s00_axis_aresetn==0) begin
			for (integer i = 0; i < 5; i = i + 1) begin
				first_mix_pipe[i]  <= '0;
				second_mix_pipe[i] <= '0;
				if (i < 4) begin
					final_mix_pipe[i]  <= '0;
				end
			end
			final_mix_pipe[4] <= '0;
			for (integer i = 0; i < 10; i = i + 1) begin
				valid_pipeline[i] <= 1'b0;
				last_pipeline[i]  <= 1'b0;
			end
		end else begin
			if (advance) begin
				valid_pipeline[0] <= s00_axis_tvalid;
				last_pipeline[0]  <= s00_axis_tlast;

				for (integer i = 0; i < 9; i = i + 1) begin
					valid_pipeline[i+1] <= valid_pipeline[i];
					last_pipeline[i+1]  <= last_pipeline[i];
				end

				// MurmurHash3 32-bit block mix
				first_mix_pipe[0]  <= s00_axis_tdata[31:0] * MIX0;
				second_mix_pipe[0] <= SECOND_MIX0;

				first_mix_pipe[1]  <= rotl32(first_mix_pipe[0], 32'd15);
				second_mix_pipe[1] <= SECOND_MIX1;

				first_mix_pipe[2]  <= first_mix_pipe[1] * MIX1;
				second_mix_pipe[2] <= SECOND_MIX2;

				first_mix_pipe[3]  <= rotl32(first_mix_pipe[2] ^ H0, 32'd13);
				second_mix_pipe[3] <= second_mix_pipe[2];

				first_mix_pipe[4]  <= (first_mix_pipe[3] * 32'd5) + MIX2;
				second_mix_pipe[4] <= second_mix_pipe[3];

				final_mix_pipe[0] <= rotl32(second_mix_pipe[4] ^ first_mix_pipe[4], 32'd13);
				final_mix_pipe[1] <= (final_mix_pipe[0] * 32'd5) + MIX2;

				// FMIX32
				final_mix_pipe[2] <= (final_mix_pipe[1] ^ (final_mix_pipe[1] >> 16)) * F0;
				final_mix_pipe[3] <= (final_mix_pipe[2] ^ (final_mix_pipe[2] >> 13)) * F1;
				final_mix_pipe[4] <= final_mix_pipe[3] ^ (final_mix_pipe[3] >> 16);
			end
		end
	end

	always_comb begin
		m00_axis_tdata 	= {32'b0, final_mix_pipe[4]};
		m00_axis_tvalid = valid_pipeline[9];
		m00_axis_tlast 	= last_pipeline[9];
		m00_axis_tstrb 	= '1;
	end

endmodule

`default_nettype wire
