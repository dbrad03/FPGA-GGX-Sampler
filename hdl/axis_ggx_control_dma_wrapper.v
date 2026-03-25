`timescale 1ns / 1ps
`default_nettype none

// -----------------------------------------------------------------------------
// DMA-friendly wrapper for axis_ggx_control.
//
// Command stream format on S_AXIS (3 beats, 64-bit each):
//   Beat0: {unused[15:0], burst_len[15:0], seed_base[31:0]}   TLAST=0
//   Beat1: {view_y[31:0], view_x[31:0]}                       TLAST=0
//   Beat2: {alpha[31:0], view_z[31:0]}                        TLAST=1
//
// Output stream on M_AXIS (DMA-facing):
//   Beat : {{pad}, hz[31:0], hy[31:0], hx[31:0]}
//   Default width is 128-bit. Low 96 bits carry GGX output (Q1.31 each).
//   TLAST asserted on final sample of command burst.
// -----------------------------------------------------------------------------
module axis_ggx_control_dma_wrapper #
(
  parameter S_AXIS_TDATA_WIDTH = 64,
  parameter M_AXIS_TDATA_WIDTH = 128
)
(
  input  wire                         aclk,
  input  wire                         aresetn,

  // AXI4-Stream Slave (from DMA MM2S)
  input  wire [S_AXIS_TDATA_WIDTH-1:0] s_axis_tdata,
  input  wire [(S_AXIS_TDATA_WIDTH/8)-1:0] s_axis_tkeep,
  input  wire                         s_axis_tvalid,
  output wire                         s_axis_tready,
  input  wire                         s_axis_tlast,

  // AXI4-Stream Master (to DMA S2MM)
  output wire [M_AXIS_TDATA_WIDTH-1:0] m_axis_tdata,
  output wire [(M_AXIS_TDATA_WIDTH/8)-1:0] m_axis_tkeep,
  output wire                         m_axis_tvalid,
  input  wire                         m_axis_tready,
  output wire                         m_axis_tlast
);

  // ---------------------------------------------------------------------------
  // Input skid (DMA MM2S -> GGX control)
  // ---------------------------------------------------------------------------
  wire [S_AXIS_TDATA_WIDTH-1:0]           cmd_tdata;
  wire [(S_AXIS_TDATA_WIDTH/8)-1:0]       cmd_tkeep;
  wire                                     cmd_tvalid;
  wire                                     cmd_tready;
  wire                                     cmd_tlast;

  axis_skid_buffer #(
    .C_S00_AXIS_TDATA_WIDTH(S_AXIS_TDATA_WIDTH),
    .C_M00_AXIS_TDATA_WIDTH(S_AXIS_TDATA_WIDTH)
  ) u_cmd_skid (
    .s00_axis_aclk(aclk),
    .s00_axis_aresetn(aresetn),
    .s00_axis_tlast(s_axis_tlast),
    .s00_axis_tvalid(s_axis_tvalid),
    .s00_axis_tdata(s_axis_tdata),
    .s00_axis_tstrb(s_axis_tkeep),
    .s00_axis_tready(s_axis_tready),

    .m00_axis_aclk(aclk),
    .m00_axis_aresetn(aresetn),
    .m00_axis_tready(cmd_tready),
    .m00_axis_tvalid(cmd_tvalid),
    .m00_axis_tlast(cmd_tlast),
    .m00_axis_tdata(cmd_tdata),
    .m00_axis_tstrb(cmd_tkeep)
  );

  // Internal GGX output is fixed 96-bit {hz, hy, hx}.
  wire [95:0] ggx_m_axis_tdata;
  wire ggx_m_axis_tvalid;
  wire ggx_m_axis_tready;
  wire ggx_m_axis_tlast;
  wire [11:0] ggx_m_axis_tstrb;

  axis_ggx_control #(
    .C_S00_AXIS_TDATA_WIDTH(S_AXIS_TDATA_WIDTH),
    .C_M00_AXIS_TDATA_WIDTH(96),
    .FRAC_BITS(32)
  ) u_axis_ggx_control (
    .s00_axis_aclk(aclk),
    .s00_axis_aresetn(aresetn),
    .s00_axis_tlast(cmd_tlast),
    .s00_axis_tvalid(cmd_tvalid),
    .s00_axis_tdata(cmd_tdata),
    .s00_axis_tstrb(cmd_tkeep),
    .s00_axis_tready(cmd_tready),

    .m00_axis_aclk(aclk),
    .m00_axis_aresetn(aresetn),
    .m00_axis_tready(ggx_m_axis_tready),
    .m00_axis_tvalid(ggx_m_axis_tvalid),
    .m00_axis_tlast(ggx_m_axis_tlast),
    .m00_axis_tdata(ggx_m_axis_tdata),
    .m00_axis_tstrb(ggx_m_axis_tstrb)
  );

  // ---------------------------------------------------------------------------
  // Output padding + skid (GGX control -> DMA S2MM)
  // ---------------------------------------------------------------------------

  // DMA-facing stream is padded to full width so S2MM stream width can match
  // its memory map width. Payload stays in the low 96 bits:
  //   [31:0]=hx, [63:32]=hy, [95:64]=hz, [127:96]=0
  wire [M_AXIS_TDATA_WIDTH-1:0]           out_pad_tdata;
  wire [(M_AXIS_TDATA_WIDTH/8)-1:0]       out_pad_tkeep;

  assign out_pad_tdata = {{(M_AXIS_TDATA_WIDTH-96){1'b0}}, ggx_m_axis_tdata};
  assign out_pad_tkeep = {(M_AXIS_TDATA_WIDTH/8){1'b1}};

  axis_skid_buffer #(
    .C_S00_AXIS_TDATA_WIDTH(M_AXIS_TDATA_WIDTH),
    .C_M00_AXIS_TDATA_WIDTH(M_AXIS_TDATA_WIDTH)
  ) u_out_skid (
    .s00_axis_aclk(aclk),
    .s00_axis_aresetn(aresetn),
    .s00_axis_tlast(ggx_m_axis_tlast),
    .s00_axis_tvalid(ggx_m_axis_tvalid),
    .s00_axis_tdata(out_pad_tdata),
    .s00_axis_tstrb(out_pad_tkeep),
    .s00_axis_tready(ggx_m_axis_tready),

    .m00_axis_aclk(aclk),
    .m00_axis_aresetn(aresetn),
    .m00_axis_tready(m_axis_tready),
    .m00_axis_tvalid(m_axis_tvalid),
    .m00_axis_tlast(m_axis_tlast),
    .m00_axis_tdata(m_axis_tdata),
    .m00_axis_tstrb(m_axis_tkeep)
  );

endmodule

`default_nettype wire
