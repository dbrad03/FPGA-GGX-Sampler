`timescale 1ns / 1ps
`default_nettype none

module axis_sobol2d_stateless #
  (
		parameter integer C_S00_AXIS_TDATA_WIDTH	= 64,
		parameter integer C_M00_AXIS_TDATA_WIDTH	= 64,
		parameter integer DIMENSION								= 0
	)
	(
		// Ports of Axi Slave Bus Interface S00_AXIS
		input wire  s00_axis_aclk, s00_axis_aresetn,
		input wire  s00_axis_tlast, s00_axis_tvalid,
		input wire [C_S00_AXIS_TDATA_WIDTH-1 : 0] s00_axis_tdata, // {pass, index (u32)}
		input wire [(C_S00_AXIS_TDATA_WIDTH/8)-1: 0] s00_axis_tstrb,
		output logic  s00_axis_tready,

		// Ports of Axi Master Bus Interface M00_AXIS
		input wire  m00_axis_aclk, m00_axis_aresetn,
		input wire  m00_axis_tready,
		output logic  m00_axis_tvalid, m00_axis_tlast,
		output logic [C_M00_AXIS_TDATA_WIDTH-1 : 0] m00_axis_tdata, // {pass, x}
		output logic [(C_M00_AXIS_TDATA_WIDTH/8)-1: 0] m00_axis_tstrb
	);
	localparam int DATA_WIDTH = 32;
  localparam int SEED_WIDTH = 32;

	// Direction Numbers
	logic [31:0] V [0:31];

	initial begin
		if (DIMENSION==0) begin
			V[0]=32'h80000000; 	V[1]=32'h40000000; 	V[2]=32'h20000000; 	V[3]=32'h10000000;
			V[4]=32'h08000000; 	V[5]=32'h04000000; 	V[6]=32'h02000000; 	V[7]=32'h01000000;
			V[8]=32'h00800000; 	V[9]=32'h00400000; 	V[10]=32'h00200000; V[11]=32'h00100000;
			V[12]=32'h00080000; V[13]=32'h00040000; V[14]=32'h00020000; V[15]=32'h00010000;
			V[16]=32'h00008000; V[17]=32'h00004000; V[18]=32'h00002000; V[19]=32'h00001000;
			V[20]=32'h00000800; V[21]=32'h00000400; V[22]=32'h00000200; V[23]=32'h00000100;
			V[24]=32'h00000080; V[25]=32'h00000040; V[26]=32'h00000020; V[27]=32'h00000010;
			V[28]=32'h00000008; V[29]=32'h00000004; V[30]=32'h00000002; V[31]=32'h00000001;
		end else begin
			V[0]=32'h80000000; 	V[1]=32'hc0000000; 	V[2]=32'hc0000000; 	V[3]=32'h90000000;
			V[4]=32'hb8000000; 	V[5]=32'he8000000; 	V[6]=32'he2000000; 	V[7]=32'ha3000000;
			V[8]=32'h8b000000; 	V[9]=32'hce400000; 	V[10]=32'hcee00000; V[11]=32'h9aa00000;
			V[12]=32'hb0080000; V[13]=32'he40c0000; V[14]=32'hee0c0000; V[15]=32'haa090000;
			V[16]=32'h808b8000; V[17]=32'hc0ce8000; V[18]=32'hc0ce2000; V[19]=32'h909a3000;
			V[20]=32'hb8b0b000; V[21]=32'he8e4e400; V[22]=32'he2eeee00; V[23]=32'ha3aaaa00;
			V[24]=32'h8b800080; V[25]=32'hce8000c0; V[26]=32'hce2000c0; V[27]=32'h9a300090;
			V[28]=32'hb0b000b8; V[29]=32'he4e400e8; V[30]=32'heeee00e2; V[31]=32'haaaa00a3;
		end
	end

	// Control Logic
	logic stall, advance;
  always_comb begin
    stall  = m00_axis_tvalid && !m00_axis_tready;
    advance    = !stall;
    s00_axis_tready = !stall;
  end

	// Pipeline Registers. Depth comes from the package so that this block's
	// latency and axis_pre_ggx_sampler's alignment delay are one number --
	// producers derive too, not just consumers.
	// See docs/adr/0002-latency-package-is-law.md.
	localparam int PIPE_DEPTH = ggx_latency_pkg::SOBOL_LATENCY;
	logic [PIPE_DEPTH-1:0] valid_pipeline;
	logic [PIPE_DEPTH-1:0] last_pipeline;
	logic [SEED_WIDTH-1:0] seed_pipeline [0:PIPE_DEPTH-1];

	logic [31:0] partial_sum0, partial_sum1, partial_sum2, partial_sum3;
	logic [31:0] final_sobol;

	// Combinatorial signals
	logic [31:0] index_raw;
	logic [31:0] g; // gray code value
	logic [31:0] sum0, sum1, sum2, sum3;

	always_comb begin
		index_raw = s00_axis_tdata[31:0];
		g = index_raw ^ (index_raw >> 1);

		sum0 = 32'b0;
		sum1 = 32'b0;
		sum2 = 32'b0;
		sum3 = 32'b0;

		for (integer i = 0; i < 8; i = i + 1) begin
			if (g[i]) sum0 = sum0 ^ V[i];
		end
		for (integer i = 8; i < 16; i = i + 1) begin
			if (g[i]) sum1 = sum1 ^ V[i];
		end
		for (integer i = 16; i < 24; i = i + 1) begin
			if (g[i]) sum2 = sum2 ^ V[i];
		end
		for (integer i = 24; i < 32; i = i + 1) begin
			if (g[i]) sum3 = sum3 ^ V[i];
		end
	end

	always_ff @(posedge s00_axis_aclk) begin
		if (s00_axis_aresetn==0) begin
			valid_pipeline <= '0;
			last_pipeline  <= '0;
			partial_sum0 	 <= '0; partial_sum1 <= '0; 
			partial_sum2 	 <= '0; partial_sum3 <= '0;
			final_sobol 	 <= '0;
			seed_pipeline[0] <= '0; seed_pipeline[1] <= '0;
		end else begin
			if (advance) begin
				valid_pipeline[0] <= s00_axis_tvalid;
				last_pipeline[0] 	<= s00_axis_tlast;
				seed_pipeline[0] 	<= s00_axis_tdata[63:32];

				partial_sum0 <= sum0;
				partial_sum1 <= sum1;
				partial_sum2 <= sum2;
				partial_sum3 <= sum3;
			
				valid_pipeline[1] <= valid_pipeline[0];
				last_pipeline[1] 	<= last_pipeline[0];
				seed_pipeline[1] 	<= seed_pipeline[0];
				final_sobol <= partial_sum0 ^ partial_sum1 ^ partial_sum2 ^ partial_sum3;
			end
		end
	end

	always_comb begin
		m00_axis_tdata 	= {seed_pipeline[PIPE_DEPTH-1], final_sobol};
		m00_axis_tvalid = valid_pipeline[PIPE_DEPTH-1];
		m00_axis_tlast 	= last_pipeline[PIPE_DEPTH-1];
		m00_axis_tstrb 	= '1;
	end

endmodule
`default_nettype wire
