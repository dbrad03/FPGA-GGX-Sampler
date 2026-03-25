`timescale 1ns / 1ps
`default_nettype none

// AXI4-Stream skid buffer (1-entry elastic stage)
//
// Notes:
// - This module is intended for single-clock AXIS paths.
// - It preserves full throughput (1 beat/cycle) when downstream is ready.
// - It captures exactly one beat when downstream stalls to break combinational
//   ready paths and absorb backpressure cleanly.

module axis_skid_buffer #
	(
		parameter integer C_S00_AXIS_TDATA_WIDTH	= 128,
		parameter integer C_M00_AXIS_TDATA_WIDTH	= 128
	)
	(
		// Ports of Axi Slave Bus Interface S00_AXIS
		input wire  s00_axis_aclk, s00_axis_aresetn,
		input wire  s00_axis_tlast, s00_axis_tvalid,
		input wire [C_S00_AXIS_TDATA_WIDTH-1 : 0] s00_axis_tdata,
		input wire [(C_S00_AXIS_TDATA_WIDTH/8)-1: 0] s00_axis_tstrb,
		output logic  s00_axis_tready,

		// Ports of Axi Master Bus Interface M00_AXIS
		input wire  m00_axis_aclk, m00_axis_aresetn,
		input wire  m00_axis_tready,
		output logic  m00_axis_tvalid, m00_axis_tlast,
		output logic [C_M00_AXIS_TDATA_WIDTH-1 : 0] m00_axis_tdata,
		output logic [(C_M00_AXIS_TDATA_WIDTH/8)-1: 0] m00_axis_tstrb
	);

  // This core is used in single-clock mode in this project; keep secondary
  // clock/reset ports referenced to avoid no-load warnings in some flows.
  wire _unused_m00_aclk = m00_axis_aclk;
  wire _unused_m00_aresetn = m00_axis_aresetn;

  localparam integer S_STRB_W = (C_S00_AXIS_TDATA_WIDTH/8);
  localparam integer M_STRB_W = (C_M00_AXIS_TDATA_WIDTH/8);

  // Skid storage holds one full beat when downstream stalls.
  logic [C_S00_AXIS_TDATA_WIDTH-1:0] skid_data;
  logic [S_STRB_W-1:0]               skid_strb;
  logic                               skid_last;
  logic                               skid_full;

  // Output mux: bypass input when empty, serve skid entry when full.
  always_comb begin
    if (skid_full) begin
      m00_axis_tdata  = skid_data;
      m00_axis_tstrb  = skid_strb;
      m00_axis_tlast  = skid_last;
      m00_axis_tvalid = 1'b1;
    end else begin
      m00_axis_tdata  = s00_axis_tdata;
      m00_axis_tstrb  = s00_axis_tstrb;
      m00_axis_tlast  = s00_axis_tlast;
      m00_axis_tvalid = s00_axis_tvalid;
    end
  end

  // Upstream can send when skid is empty, or when downstream is consuming the
  // buffered beat this cycle.
  always_comb begin
    s00_axis_tready = (!skid_full) || m00_axis_tready;
  end

  wire s_fire = s00_axis_tvalid && s00_axis_tready;

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn == 0) begin
      skid_data <= '0;
      skid_strb <= {S_STRB_W{1'b1}};
      skid_last <= 1'b0;
      skid_full <= 1'b0;
    end else begin
      if (!skid_full) begin
        // Empty -> capture one beat only when downstream stalls this cycle.
        if (s_fire && !m00_axis_tready) begin
          skid_data <= s00_axis_tdata;
          skid_strb <= s00_axis_tstrb;
          skid_last <= s00_axis_tlast;
          skid_full <= 1'b1;
        end
      end else begin
        // Full -> output is buffered beat.
        if (m00_axis_tready) begin
          if (s00_axis_tvalid) begin
            // Simultaneous pop/push: keep full and replace with new beat.
            skid_data <= s00_axis_tdata;
            skid_strb <= s00_axis_tstrb;
            skid_last <= s00_axis_tlast;
            skid_full <= 1'b1;
          end else begin
            // Pop only: buffer drains.
            skid_full <= 1'b0;
          end
        end
      end
    end
  end

  // Widths are expected to match for this skid stage.
  // Keep a benign reference to M-side width so both params are used.
  wire _unused_width_mismatch_flag = (C_S00_AXIS_TDATA_WIDTH != C_M00_AXIS_TDATA_WIDTH) ||
                                     (S_STRB_W != M_STRB_W);

endmodule

`default_nettype wire
