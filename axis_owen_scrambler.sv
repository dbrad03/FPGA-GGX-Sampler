`timescale 1ns / 1ps
`default_nettype none

module axis_owen_scrambler #
	(
	  parameter int DIMS       = 2,
    parameter int FRAC_BITS  = 32,
    parameter int INDEX_BITS = 32,
    parameter C_S00_AXIS_TDATA_WIDTH = DIMS*FRAC_BITS,
    parameter C_M00_AXIS_TDATA_WIDTH = DIMS*FRAC_BITS
) (
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
    output logic [(C_M00_AXIS_TDATA_WIDTH/8)-1: 0] m00_axis_tstrb,

    // Additional inputs for seed construction
    input wire [FRAC_BITS-1:0] pixel_id,
    input wire [FRAC_BITS-1:0] sample_id,
    input wire [INDEX_BITS-1:0] sample_index
);
  function automatic [31:0] bit_reverse_32(input [31:0] in);
    integer i;
    begin
      for (i = 0; i < 32; i++) begin
        bit_reverse_32[i] = in[31-i];
      end
    end
  endfunction

  // FOR CONSTRUCTING LAINE-KARRAS SEED
  localparam logic [FRAC_BITS-1:0] K_PIXEL  = 32'h9e3779b1;
  localparam logic [FRAC_BITS-1:0] K_SAMPLE = 32'h85ebca6b;
  localparam logic [FRAC_BITS-1:0] K_DIM    = 32'hc2b2ae35;

  // FOR PERFORMING LAINE-KARRAS PERMUTATION
  localparam logic [FRAC_BITS-1:0] H1 = 32'h6c50b47c;
  localparam logic [FRAC_BITS-1:0] H2 = 32'hb82f1e52;
  localparam logic [FRAC_BITS-1:0] H3 = 32'hc7afe638;
  localparam logic [FRAC_BITS-1:0] H4 = 32'h8d22f6e6;

  logic [FRAC_BITS-1:0] u0_pipe [3:0];
  logic [FRAC_BITS-1:0] u1_pipe [3:0];
  wire [FRAC_BITS-1:0]  u0_in, u1_in, u0_rev, u1_rev, u0_final, u1_final; 
  assign u0_in = s00_axis_tdata[FRAC_BITS-1:0];
  assign u1_in = s00_axis_tdata[(FRAC_BITS<<1)-1:FRAC_BITS];
  assign u0_rev = bit_reverse_32(u0_in);
  assign u1_rev = bit_reverse_32(u1_in);

  logic [FRAC_BITS-1:0] seed  [1:0];
  logic [FRAC_BITS-1:0] seed0 [1:0];
  logic [FRAC_BITS-1:0] seed1 [1:0];

  logic [FRAC_BITS-1:0] lk0 [4:0];
  logic [FRAC_BITS-1:0] lk1 [4:0];
  assign u0_final = bit_reverse_32(lk0[4]);
  assign u1_final = bit_reverse_32(lk1[4]);

  logic [FRAC_BITS-1:0] sample_id_reg;

  logic [C_M00_AXIS_TDATA_WIDTH-1:0] m00_axis_tdata_reg;
  logic                              m00_axis_tvalid_reg;
  logic [8:0]                        valid_pipe;

  assign s00_axis_tready = 1'b1;
  assign m00_axis_tlast = 1'b0;
  assign m00_axis_tstrb = {C_M00_AXIS_TDATA_WIDTH/8{1'b1}};
  assign m00_axis_tvalid = m00_axis_tvalid_reg;
  assign m00_axis_tdata = m00_axis_tdata_reg;

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn == 0) begin
      // reset everything
      m00_axis_tdata_reg <= '0;
      m00_axis_tvalid_reg <= 0;
      
      valid_pipe <= '0;

      for (int i = 0; i < 2; i++) begin
        seed[i] <= '0;
        seed0[i] <= '0;
        seed1[i] <= '0;
      end
      for (int i = 0; i < 4; i++) begin
        u0_pipe[i] <= '0;
        u1_pipe[i] <= '0;
      end
      for (int i = 0; i < 5; i++) begin
        lk0[i] <= '0;
        lk1[i] <= '0;
      end

      sample_id_reg <= '0;
    end else begin
      valid_pipe <= {valid_pipe[7:0], s00_axis_tvalid};

      if (s00_axis_tvalid) begin
        seed[0] <= pixel_id * K_PIXEL;
        u0_pipe[0] <= u0_rev;
        u1_pipe[0] <= u1_rev;

        sample_id_reg <= sample_id;
      end

      seed[1] <= seed[0] ^ (sample_id_reg * K_SAMPLE);
      u0_pipe[1] <= u0_pipe[0];
      u1_pipe[1] <= u1_pipe[0];

      seed0[0] <= seed[1] ^ (32'd0 * K_DIM);
      seed1[0] <= seed[1] ^ (32'd1 * K_DIM);
      u0_pipe[2] <= u0_pipe[1];
      u1_pipe[2] <= u1_pipe[1];

      seed0[1] <= seed0[0];
      seed1[1] <= seed1[0];
      u0_pipe[3] <= u0_pipe[2];
      u1_pipe[3] <= u1_pipe[2];

      lk0[0] <= u0_pipe[3] + seed0[1];
      lk1[0] <= u1_pipe[3] + seed1[1];

      lk0[1] <= lk0[0] ^ (lk0[0] * H1);
      lk1[1] <= lk1[0] ^ (lk1[0] * H1);

      lk0[2] <= lk0[1] ^ (lk0[1] * H2);
      lk1[2] <= lk1[1] ^ (lk1[1] * H2);

      lk0[3] <= lk0[2] ^ (lk0[2] * H3);
      lk1[3] <= lk1[2] ^ (lk1[2] * H3);

      lk0[4] <= lk0[3] ^ (lk0[3] * H4);
      lk1[4] <= lk1[3] ^ (lk1[3] * H4);

      m00_axis_tvalid_reg <= valid_pipe[8];
      if (valid_pipe[8]) m00_axis_tdata_reg <= {u1_final,u0_final};
    end
  end
   
endmodule

`default_nettype wire
