`timescale 1ns / 1ps
`default_nettype none

module axis_fixed_sqrt #
  (
		parameter integer FRAC_BITS = 32
	)
	(
		// Ports of Axi Slave Bus Interface S00_AXIS
		input wire  s00_axis_aclk, s00_axis_aresetn,
		input wire  s00_axis_tlast, s00_axis_tvalid,
		input wire [FRAC_BITS-1 : 0] s00_axis_tdata,
		input wire [(FRAC_BITS/8)-1: 0] s00_axis_tstrb,
		output logic  s00_axis_tready,

		// Ports of Axi Master Bus Interface M00_AXIS
		input wire  m00_axis_aclk, m00_axis_aresetn,
		input wire  m00_axis_tready,
		output logic  m00_axis_tvalid, m00_axis_tlast,
		output logic [FRAC_BITS-1 : 0] m00_axis_tdata,
		output logic [(FRAC_BITS/8)-1: 0] m00_axis_tstrb
		);

  // This core runs single-clock in this project; keep other AXIS sideband/clock
  // ports referenced so synthesis does not emit no-load warnings.
  wire _unused_s00_tlast = s00_axis_tlast;
  wire _unused_s00_tstrb = ^s00_axis_tstrb;
  wire _unused_m00_aclk = m00_axis_aclk;
  wire _unused_m00_aresetn = m00_axis_aresetn;

  // ============================================================
  // Fixed-point contract:
  //   Input  x_in : u32 as Q0.32 in [0,1)  (x = x_in / 2^32)
  //   Output y_out: u32 as Q1.31 in [0,1)  (y = y_out / 2^31)
  //
  // We compute integer sqrt of:
  //   N = x_in << 32  (64-bit)
  // Then:
  //   root = floor(sqrt(N)) ~= sqrt(x) * 2^16
  // Finally:
  //   y_out = root << 15  ~= sqrt(x) * 2^31   (Q0.32)
  // ============================================================

  /// PIPELINE CONTROL LOGIC
  logic pipe_en;
  assign pipe_en = m00_axis_tready || !m00_axis_tvalid;
  assign s00_axis_tready = pipe_en;
  assign m00_axis_tlast = 1'b0;
  assign m00_axis_tstrb = '1;

  localparam int STAGES = 32;
  localparam int RAD_W  = 64; // radicand width
  localparam int ROOT_W = 32; // root width
  // Signed remainder for NON-RESTORING recurrence. |rem| < 2^33, so 35 signed
  // bits (sign + 34) cover it with margin. The shift/add-sub is done in a wider
  // temporary to avoid transient overflow of the 4*rem term.
  localparam int REM_W  = 35;

  logic [STAGES:0]        valid;
  logic [RAD_W-1:0]       rad   [0:STAGES]; // shifting radicand (MSB pair consumed each stage)
  logic [ROOT_W-1:0]      root  [0:STAGES]; // partial root (built MSB-first, 0/1 bits)
  logic signed [REM_W-1:0] rem  [0:STAGES]; // signed partial remainder

  // combinational helpers
  logic [1:0]              bits;
  logic signed [REM_W+1:0] rem_shift; // 4*rem | bits (wider to hold the 4x term)
  logic signed [REM_W+1:0] trial;     // (root<<2) | 1 (subtract) or | 3 (add)
  logic signed [REM_W+1:0] addend;    // +trial when rem<0, -trial when rem>=0
  logic signed [REM_W+1:0] rem_next;

  integer i;
  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      for (i = 0; i <= STAGES; i=i+1) begin
        valid[i] <= 1'b0;
        rad[i]   <= '0;
        root[i]  <= '0;
        rem[i]   <= '0;
      end
      m00_axis_tvalid <= 1'b0;
      m00_axis_tdata  <= '0;
    end else if (pipe_en) begin

      // STAGE 0 LOAD
      valid[0] <= s00_axis_tvalid;
      if (s00_axis_tvalid) begin
        rad[0] <= {s00_axis_tdata, 32'b0}; // x_in << 32
        root[0] <= '0;
        rem[0]  <= '0;
      end else begin
        rad[0] <= '0;
        root[0] <= '0;
        rem[0]  <= '0;
      end

      // STAGES 0..31 tranform -> 1..32 (non-restoring: no compare, no restore
      // mux -- the add/subtract mode is just the registered sign of rem[i], and
      // the new root bit is the sign of the result).
      for (i = 0; i < STAGES; i=i+1) begin
        valid[i+1] <= valid[i];

        // consume next 2 MSBs of radicand: rem_shift = 4*rem[i] + bits
        bits = rad[i][RAD_W-1 -: 2]; // rad[i][63:62]
        rem_shift = ($signed({{2{rem[i][REM_W-1]}}, rem[i]}) <<< 2) | bits;

        // trial magnitude: (root<<2)|1 if subtracting, (root<<2)|3 if adding
        trial  = rem[i][REM_W-1] ? (($signed({1'b0, root[i]}) <<< 2) | 'sd3)
                                 : (($signed({1'b0, root[i]}) <<< 2) | 'sd1);
        // rem>=0 -> subtract trial ; rem<0 -> add trial
        addend = rem[i][REM_W-1] ? trial : -trial;
        rem_next = rem_shift + addend;

        rem[i+1]  <= rem_next[REM_W-1:0];
        // new root bit = 1 when result non-negative, else 0
        root[i+1] <= (root[i] << 1) | (rem_next[REM_W+1] ? 1'b0 : 1'b1);

        // shift radicand left by 2 to expose next pair of bits next stage
        rad[i+1] <= rad[i] << 2;
      end

      // OUTPUT REGISTER
      m00_axis_tvalid <= valid[STAGES];
      if (valid[STAGES]) begin
        m00_axis_tdata  <= root[STAGES];
      end

    end
  end

endmodule
`default_nettype wire
