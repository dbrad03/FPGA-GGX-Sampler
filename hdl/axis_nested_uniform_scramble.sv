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

  logic [DATA_WIDTH-1:0] data_pipeline [0:10];
  logic [10:0]           valid_pipeline, last_pipeline;
  logic [SEED_WIDTH-1:0] seed_pipeline [0:10];

  logic [DATA_WIDTH-1:0] data_pipeline_d [1:4];

  // Bit-exact 32x32 constant multiply split into 16-bit halves, so each partial
  // product is a single 16x16 DSP with no cascade. See axis_hash_combine_2d for
  // the derivation. Truncating operands is not valid here either: the Laine-Karras
  // scramble relies on the full multiply to propagate entropy across all bits.
  //   x*C = x_lo*C_lo + ((x_lo*C_hi + x_hi*C_lo) << 16)   (mod 2^32)
  // cmul_pp lands in the existing lk_mul_* register, cmul_sum in the XOR stage
  // that already follows it, so pipeline latency is unchanged.
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

  logic [63:0] lk_mul_1, lk_mul_2, lk_mul_3, lk_mul_4; // packed partial products

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      for (integer i = 0; i <= 10; i = i + 1) begin
        last_pipeline[i]  <= 1'b0;
        valid_pipeline[i] <= 1'b0;
        data_pipeline[i]  <= '0;
        seed_pipeline[i]  <= '0;
      end
      for (integer i = 1; i <= 4; i = i + 1) begin
        data_pipeline_d[i] <= '0;
      end
      lk_mul_1 <= '0;
      lk_mul_2 <= '0;
      lk_mul_3 <= '0;
      lk_mul_4 <= '0;
    end else begin
      if (advance) begin
        last_pipeline[0]  <= s00_axis_tlast;
        valid_pipeline[0] <= s00_axis_tvalid;
        seed_pipeline[0]  <= s00_axis_tdata[63:32];
        data_pipeline[0]  <= reverse_bits(s00_axis_tdata[31:0]);

        for (integer i = 0; i < 10; i = i + 1) begin
          last_pipeline[i+1]  <= last_pipeline[i];
          valid_pipeline[i+1] <= valid_pipeline[i];
          seed_pipeline[i+1]  <= seed_pipeline[i];
        end
        
        // Cycle 1
        data_pipeline[1]  <= data_pipeline[0] + seed_pipeline[0];

        // Cycle 2
        lk_mul_1           <= cmul_pp(data_pipeline[1], LK_CONST_1);
        data_pipeline_d[1] <= data_pipeline[1];

        // Cycle 3
        data_pipeline[2]   <= data_pipeline_d[1] ^ cmul_sum(lk_mul_1);

        // Cycle 4
        lk_mul_2           <= cmul_pp(data_pipeline[2], LK_CONST_2);
        data_pipeline_d[2] <= data_pipeline[2];

        // Cycle 5
        data_pipeline[3]   <= data_pipeline_d[2] ^ cmul_sum(lk_mul_2);

        // Cycle 6
        lk_mul_3           <= cmul_pp(data_pipeline[3], LK_CONST_3);
        data_pipeline_d[3] <= data_pipeline[3];

        // Cycle 7
        data_pipeline[4]   <= data_pipeline_d[3] ^ cmul_sum(lk_mul_3);

        // Cycle 8
        lk_mul_4           <= cmul_pp(data_pipeline[4], LK_CONST_4);
        data_pipeline_d[4] <= data_pipeline[4];

        // Cycle 9
        data_pipeline[5]   <= data_pipeline_d[4] ^ cmul_sum(lk_mul_4);

        // Cycle 10
        data_pipeline[6]   <= reverse_bits(data_pipeline[5]);

        data_pipeline[7]   <= data_pipeline[6];
        data_pipeline[8]   <= data_pipeline[7];
        data_pipeline[9]   <= data_pipeline[8];
        data_pipeline[10]  <= data_pipeline[9];
      end
    end
  end

  always_comb begin
    m00_axis_tdata  = {seed_pipeline[10], data_pipeline[10]};
    m00_axis_tvalid = valid_pipeline[10];
    m00_axis_tlast  = last_pipeline[10];
    m00_axis_tstrb  = '1;
  end

endmodule

`default_nettype wire
