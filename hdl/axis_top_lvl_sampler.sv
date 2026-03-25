`timescale 1ns / 1ps
`default_nettype none

module axis_top_lvl_sampler #
	(
		parameter integer C_S00_AXIS_TDATA_WIDTH	= 64,
		parameter integer C_M00_AXIS_TDATA_WIDTH	= 64
	)
	(
		// Ports of Axi Slave Bus Interface S00_AXIS
		input wire  s00_axis_aclk, s00_axis_aresetn,
		input wire  s00_axis_tlast, s00_axis_tvalid,
		input wire [C_S00_AXIS_TDATA_WIDTH-1 : 0] s00_axis_tdata, // {burst_len, seed_base}
		input wire [(C_S00_AXIS_TDATA_WIDTH/8)-1: 0] s00_axis_tstrb,
		output logic  s00_axis_tready,

		// Ports of Axi Master Bus Interface M00_AXIS
		input wire  m00_axis_aclk, m00_axis_aresetn,
		input wire  m00_axis_tready,
		output logic  m00_axis_tvalid, m00_axis_tlast,
		output logic [C_M00_AXIS_TDATA_WIDTH-1 : 0] m00_axis_tdata, // {sample_idx, seed_base}
		output logic [(C_M00_AXIS_TDATA_WIDTH/8)-1: 0] m00_axis_tstrb
	);

	logic transmitting;
	logic in_beat, out_beat;
	
	always_comb begin
		in_beat  = s00_axis_tvalid && s00_axis_tready;
		out_beat = m00_axis_tvalid && m00_axis_tready;
		s00_axis_tready = !transmitting;
	end


	logic [31:0] seed_base;
	logic [15:0] sample_idx;
	logic [15:0] burst_length;

	always_ff @(posedge s00_axis_aclk) begin
		if (s00_axis_aresetn == 0) begin
			transmitting 	<= 1'b0;
			seed_base 		<= '0;
			burst_length 	<= 16'hFFFF;
			sample_idx 		<= '0;
		end else begin
			if (!transmitting && in_beat) begin
				transmitting <= 1'b1; // could add check that burst_length is greather than 0
				seed_base		 <= s00_axis_tdata[31:0];
				burst_length <= s00_axis_tdata[47:32];
				sample_idx	 <= 16'b0;
			end else if (transmitting) begin
				if (out_beat) begin
					if (sample_idx == burst_length) begin
						transmitting <= 1'b0;
					end else begin
						sample_idx <= sample_idx + 1;
					end
				end
			end // end of transmitting loop
		end
	end // end always ff

	always_comb begin
		m00_axis_tvalid = transmitting;
		m00_axis_tdata 	= {16'b0, sample_idx, seed_base};
		m00_axis_tlast 	= transmitting && (sample_idx == burst_length);
		m00_axis_tstrb = '1;
	end

endmodule

`default_nettype wire