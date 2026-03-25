`timescale 1ns / 1ps
`default_nettype none

module axis_trig_lut #
  (
    parameter integer C_S00_AXIS_TDATA_WIDTH	= 64,
		parameter integer C_M00_AXIS_TDATA_WIDTH	= 64,
		parameter integer FRAC_BITS = 32,
    parameter integer ADDR_BITS = 10
	)
	(
		// Ports of Axi Slave Bus Interface S00_AXIS
		input wire  s00_axis_aclk, s00_axis_aresetn,
		input wire  s00_axis_tlast, s00_axis_tvalid,
		input wire [C_S00_AXIS_TDATA_WIDTH-1 : 0] s00_axis_tdata, // {pass, u0_phase}
		input wire [(C_S00_AXIS_TDATA_WIDTH/8)-1: 0] s00_axis_tstrb,
		output logic  s00_axis_tready,

		// Ports of Axi Master Bus Interface M00_AXIS
		input wire  m00_axis_aclk, m00_axis_aresetn,
		input wire  m00_axis_tready,
		output logic  m00_axis_tvalid, m00_axis_tlast,
		output logic [C_M00_AXIS_TDATA_WIDTH-1 : 0] m00_axis_tdata, // {sin_phi, cos_phi}
		output logic [(C_S00_AXIS_TDATA_WIDTH/8)-1: 0] m00_axis_tstrb
		);

  // This core runs single-clock in this project; keep secondary AXIS clock/reset
  // and unused sideband referenced so synthesis does not emit no-load warnings.
  wire _unused_s00_tlast = s00_axis_tlast;
  wire _unused_s00_tstrb = ^s00_axis_tstrb;
  wire _unused_m00_aclk = m00_axis_aclk;
  wire _unused_m00_aresetn = m00_axis_aresetn;

  localparam integer ROM_SIZE = 1 << ADDR_BITS;

  logic [C_M00_AXIS_TDATA_WIDTH-1:0] trig_rom [0:ROM_SIZE-1];

  initial begin
`ifndef SYNTHESIS
    if ($test$plusargs("dump_roms")) 
      $display("Loading ggx_trig_rom.mem...");
`endif
    $readmemh("ggx_trig_rom.mem", trig_rom);
  end

  logic pipe_en;
  assign pipe_en = m00_axis_tready || !m00_axis_tvalid;
  assign s00_axis_tready = pipe_en;

  wire [FRAC_BITS-1:0] u0_phase = s00_axis_tdata[FRAC_BITS-1:0];
  wire [ADDR_BITS-1:0] addr_comb = u0_phase[FRAC_BITS-1 : FRAC_BITS-ADDR_BITS];

  logic s1_valid;
  logic [ADDR_BITS-1:0] s1_addr;
  logic s2_valid;
  logic [C_M00_AXIS_TDATA_WIDTH-1:0] s2_data; // {cos, sin}

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      s1_valid <= 0;
      s1_addr  <= 0;
      s2_valid <= 0;
      s2_data  <= 0;
      
      m00_axis_tvalid <= 0;
      m00_axis_tdata  <= 0;
    end else begin
      if (pipe_en) begin
        s1_valid <= s00_axis_tvalid;
        if (s00_axis_tvalid) begin
          s1_addr <= addr_comb;
        end

        s2_valid <= s1_valid;
        if (s1_valid) begin
          s2_data <= trig_rom[s1_addr];
        end

        m00_axis_tvalid <= s2_valid;
        if (s2_valid) begin
          m00_axis_tdata <= s2_data;
        end
      end
    end
  end

  assign m00_axis_tlast = 1'b0;
  assign m00_axis_tstrb = {(C_S00_AXIS_TDATA_WIDTH/8){1'b1}};

endmodule

`default_nettype wire
