`timescale 1ns / 1ps
`default_nettype none

module axis_fixed_inv_sqrt #
  (
		parameter integer FRAC_BITS = 32,
    parameter integer ADDR_BITS = 14
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
  // -----------------------------
  // Fixed-point conventions used
  // -----------------------------
  // Input:  s00_axis_tdata is u32 representing x in [0,1):
  //         x = x_in / 2^32
  // Internal x: convert to Q7.25 by >>7 (since 32 frac -> 25 frac)
  //         x_q7_25 = floor(x_in / 2^7)  (Q7.25)
  //  
  // ROM: inv_rom entries are y0 in Q7.25, approximating y0 ≈ S/sqrt(m) .. using S=0.25 currently
  // Newton step uses factor in Q2.30 and multiplies y0 (Q7.25) * factor (Q2.30)
  // Output: m00_axis_tdata is Q7.25 (not Q1.31)

  localparam int ROM_SIZE = 1 << ADDR_BITS;
  localparam int QFRAC    = 25;           // fractional bits for Q7.25

  // 1.5 in Q2.30 format (2 integer bits incl sign, 30 fractional bits)
  localparam signed [31:0] THREE_HALVES_Q2_30 = 32'sh6000_0000;

  // X_MIN = 2^-15 in Q7.25 => 2^(-15) * 2^25 = 2^10 = 1024
  localparam logic [31:0] X_MIN = 32'd1024;

  //---------------------------------
  // 2. ROM LOADING
  //---------------------------------
  wire [FRAC_BITS-1:0] x_in = s00_axis_tdata;
  // Combinatorial address decode directly from x_in; registered into s0_addr_reg in stage -1.
  wire [FRAC_BITS-1:0] x_q7_25 = x_in >> 7; // Q0.32 -> Q7.25
  wire [FRAC_BITS-1:0] x_clamped = (x_q7_25 < X_MIN) ? X_MIN : x_q7_25;
  wire [ADDR_BITS-1:0] lut_idx = x_clamped >> (QFRAC - ADDR_BITS); // top ADDR_BITS of fractional part

  (* rom_style = "block" *) logic [FRAC_BITS-1:0] inv_rom [0:ROM_SIZE-1];
  initial begin
`ifndef SYNTHESIS
    if ($test$plusargs("dump_roms")) $display("Loading inv_sqrt_rom.mem...");
`endif
    $readmemh("inv_sqrt_rom.mem", inv_rom);
  end

  //---------------------------------
  // 3. PIPELINE CONTROL LOGIC
  //---------------------------------
  logic pipe_en;
  assign pipe_en = m00_axis_tready || !m00_axis_tvalid;
  assign s00_axis_tready = pipe_en;
  assign m00_axis_tlast = 1'b0;
  assign m00_axis_tstrb = '1;

  //---------------------------------
  // 4. PIPELINE STAGES
  //---------------------------------

  // -- Stage -1 -> 0: input register (breaks long comb path from caller into LUT ROM) --
  // Without this stage, the async LUT ROM lookup is combinatorially chained with the
  // caller's arithmetic (e.g. ONE_Q0 - z2_uq032 in event_basis), creating a path
  // spanning >13 ns.  Registering the input here caps the caller→inv_sqrt path at
  // ~3-4 ns and lets all subsequent stages start from a local registered source.
  logic        s0_valid;
  logic [FRAC_BITS-1:0] s0_x;
  logic [ADDR_BITS-1:0] s0_addr_reg; // registered ROM address (enables BRAM inference)

  // -- Stage 0 -> 1 Signals --
  logic        s1_valid;
  logic [FRAC_BITS-1:0] s1_x, s1_y0;

  // -- Stage 1 -> 2 Signals --
  logic        s2_valid;
  logic [FRAC_BITS-1:0] s2_x, s2_y0;
  logic [2*FRAC_BITS-1:0] s2_y0_sq_calc; // Q14.50

  logic        s3a_valid, s3b_valid;
  logic [3*FRAC_BITS-1:0] s3_t_wide; //Q21.75
  logic [FRAC_BITS-1:0] s3_y00, s3_y01, s3_t;

  // -- Stage 2 -> 3 Signals --
  logic signed [31:0] eight_t_q2_30, factor_q2_30;
  assign eight_t_q2_30 = $signed({1'b0, s3_t[FRAC_BITS-2:0]}) <<< 3 ; // Q7.25 -> Q2.30 + multiply by 8
  assign factor_q2_30 = THREE_HALVES_Q2_30 - eight_t_q2_30;

  // -- Stage 3 -> 4 Signals --
  logic        s4a_valid, s4b_valid;
  logic [FRAC_BITS-1:0] s4_y0;
  logic signed [FRAC_BITS-1:0] s4_factor;

  // -- Stage 4 -> Output Signals --
  logic        s5_valid;
  logic [FRAC_BITS-1:0] s5_prod;
  logic [2*FRAC_BITS-1:0] s5_prod_calc;

  // s0_x_clamped feeds s1_x (Newton's method input). s0_lut_idx is dead (address
  // now comes from the registered s0_addr_reg). Vivado trims unused logic.
  wire [FRAC_BITS-1:0] s0_x_q7_25   = s0_x >> 7;
  wire [FRAC_BITS-1:0] s0_x_clamped = (s0_x_q7_25 < X_MIN) ? X_MIN : s0_x_q7_25;

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      s0_valid <= 0; s0_x <= '0; s0_addr_reg <= '0;
      s1_valid <= 0; s2_valid <= 0;
      s3a_valid <= 0; s3b_valid <= 0;
      s4a_valid <= 0; s4b_valid <= 0;
      s1_x <= '0; s1_y0 <= '0;
      s2_x <= '0; s2_y0 <= '0; s2_y0_sq_calc <= '0;
      s3_y00 <= '0; s3_y01 <= '0; s3_t <= '0; s3_t_wide <= '0;
      s4_y0 <= '0; s4_factor <= '0;
      s5_prod_calc <= '0;

      m00_axis_tvalid <= 0;
      m00_axis_tdata  <= 0;
    end else if (pipe_en) begin
      // --- STAGE -1 (Input register) ---
      // Register both the raw input and the ROM address from x_in combinatorially.
      // s0_addr_reg is a proper FF (not a wire), which matches Vivado's BRAM inference
      // pattern (registered address → BRAM → registered output), identical to axis_trig_lut.
      s0_valid    <= s00_axis_tvalid;
      s0_x        <= x_in;
      s0_addr_reg <= lut_idx;   // lut_idx is combinatorial from x_in (registered here)

      // --- STAGE 0 -> STAGE 1 (ROM lookup from REGISTERED address) ---
      // s0_addr_reg is a registered FF → Vivado infers BRAM (not LUT ROM).
      // s1_x/s1_y0 update every pipe_en cycle; Stage 1→2 only consumes them
      // when s1_valid=1. Stale values when s0_valid=0 are harmless.
      s1_valid <= s0_valid;
      s1_x     <= s0_x_clamped;
      s1_y0    <= inv_rom[s0_addr_reg];

      // --- STAGE 1 -> STAGE 2 ---
      s2_valid <= s1_valid;
      if (s1_valid) begin
        s2_x     <= s1_x;
        s2_y0    <= s1_y0;
        s2_y0_sq_calc <= s1_y0 * s1_y0; // keep as 64-bit intermediate
      end

      // --- STAGE 2 -> STAGE 3 ---
      s3a_valid <= s2_valid;
      if (s2_valid) begin
        s3_y00 <= s2_y0;
        s3_t_wide  <= s2_x * s2_y0_sq_calc;
      end

      s3b_valid <= s3a_valid;
      if (s3a_valid) begin
        s3_y01 <= s3_y00;
        s3_t <= s3_t_wide >> 45; // Q21.75 -> Q2.30
      end

      // --- STAGE 3 -> STAGE 4 ---
      s4a_valid <= s3b_valid;
      if (s3b_valid) begin
        s4_y0     <= s3_y01;
        s4_factor <= factor_q2_30;
      end

      // --- STAGE 4 -> OUTPUT (M_AXIS) ---
      s4b_valid <= s4a_valid;
      if (s4a_valid) begin
        s5_prod_calc <= $signed(s4_y0) * $signed(s4_factor);
      end

      s5_valid <= s4b_valid;
      if (s4b_valid) begin
        // round then shift right 30 to get Q7.25 from Q9.55
        s5_prod <= $signed( (s5_prod_calc + (s5_prod_calc[63] ? -(64'sd1<<<29) : (64'sd1<<<29))) >>> 30);
      end

      m00_axis_tvalid <= s5_valid;
      if (s5_valid) begin
        m00_axis_tdata <= s5_prod; //<<< 6; // convert to Q1.31 from Q7.25
      end
    end
  end

endmodule

`default_nettype wire
