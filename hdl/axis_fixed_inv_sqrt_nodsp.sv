`timescale 1ns / 1ps
`default_nettype none

module axis_fixed_inv_sqrt_nodsp #
  (
    parameter integer FRAC_BITS = 32,
    parameter integer ADDR_BITS = 14 // Unused but kept for interface compatibility
  )
  (
    input wire  s00_axis_aclk, s00_axis_aresetn,
    input wire  s00_axis_tlast, s00_axis_tvalid,
    input wire [FRAC_BITS-1 : 0] s00_axis_tdata,
    input wire [(FRAC_BITS/8)-1: 0] s00_axis_tstrb,
    output logic  s00_axis_tready,

    input wire [31:0] s00_axis_user_x,
    input wire [31:0] s00_axis_user_y,
    input wire [31:0] s00_axis_user_z,
    input wire [3:0]  s00_axis_user_shift,

    input wire  m00_axis_aclk, m00_axis_aresetn,
    input wire  m00_axis_tready,
    output logic  m00_axis_tvalid, m00_axis_tlast,
    output logic [FRAC_BITS-1 : 0] m00_axis_tdata,
    output logic [(FRAC_BITS/8)-1: 0] m00_axis_tstrb,

    output logic [31:0] m00_axis_user_x,
    output logic [31:0] m00_axis_user_y,
    output logic [31:0] m00_axis_user_z,
    output logic [3:0]  m00_axis_user_shift
  );

  // Min input to avoid division by zero or large quotient overflow
  // X_MIN = 2^-15 in UQ0.32 => 2^-15 * 2^32 = 2^17 = 131072
  localparam logic [31:0] X_MIN_UQ0_32 = 32'd131072;
  
  // S = 0.25 (dividend) in UQ0.32 => 0.25 * 2^32 = 2^30 = 32'h4000_0000
  localparam logic [31:0] DIVIDEND_S_UQ0_32 = 32'h4000_0000;

  // Unused AXIS ports mapping
  wire _unused_s00_tlast = s00_axis_tlast;
  wire _unused_s00_tstrb = ^s00_axis_tstrb;
  wire _unused_m00_aclk = m00_axis_aclk;
  wire _unused_m00_aresetn = m00_axis_aresetn;

  // 1. Clamp input x
  wire [31:0] x_in = s00_axis_tdata;
  wire [31:0] x_clamped = (x_in < X_MIN_UQ0_32) ? X_MIN_UQ0_32 : x_in;

  // 3. Declarations for pipelined divider
  wire div_in_ready;
  wire div_out_valid;
  wire [31:0] div_out_data;

  // 2. Instantiate pipelined square root
  // Inputs: svalid, x_clamped (UQ0.32)
  // Output: sqrt_out (UQ0.32), valid after 32 cycles
  wire sqrt_out_valid;
  wire [31:0] sqrt_out_data;
  wire sqrt_in_ready;

  axis_fixed_sqrt #(
    .FRAC_BITS(32)
  ) u_sqrt (
    .s00_axis_aclk(s00_axis_aclk),
    .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(1'b0),
    .s00_axis_tvalid(s00_axis_tvalid),
    .s00_axis_tdata(x_clamped),
    .s00_axis_tstrb('1),
    .s00_axis_tready(sqrt_in_ready),

    .m00_axis_aclk(s00_axis_aclk),
    .m00_axis_aresetn(s00_axis_aresetn),
    .m00_axis_tready(div_in_ready),
    .m00_axis_tvalid(sqrt_out_valid),
    .m00_axis_tlast(),
    .m00_axis_tdata(sqrt_out_data),
    .m00_axis_tstrb()
  );

  // 3. Instantiate pipelined divider
  // Dividend: DIVIDEND_S_UQ0_32
  // Divisor: sqrt_out_data
  // Quotient format: Q7.25 (since FRAC_BITS = 25)
  // Output valid after 57 cycles

  axis_fixed_div #(
    .WIDTH(32),
    .FRAC_BITS(25)
  ) u_div (
    .s00_axis_aclk(s00_axis_aclk),
    .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(1'b0),
    .s00_axis_tvalid(sqrt_out_valid),
    .s00_axis_tdata({DIVIDEND_S_UQ0_32, sqrt_out_data}),
    .s00_axis_tstrb('1),
    .s00_axis_tready(div_in_ready),

    .m00_axis_aclk(s00_axis_aclk),
    .m00_axis_aresetn(s00_axis_aresetn),
    .m00_axis_tready(m00_axis_tready),
    .m00_axis_tvalid(div_out_valid),
    .m00_axis_tlast(),
    .m00_axis_tdata(div_out_data),
    .m00_axis_tstrb()
  );

  // Sideband propagation logic
  wire sqrt_pipe_en = div_in_ready || !sqrt_out_valid;
  wire div_pipe_en = m00_axis_tready || !div_out_valid;

  // delay_div matches axis_fixed_div's latency: the 2-phase (sub/select) split
  // doubled div's step count, so its latency went 59 -> 116 cycles (index 0:115).
  localparam int DIV_DLY = 115;
  logic [99:0] delay_sqrt [0:33];
  logic [99:0] delay_div  [0:DIV_DLY];

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn == 0) begin
      for (int j = 0; j <= 33; j = j + 1) begin
        delay_sqrt[j] <= '0;
      end
    end else if (sqrt_pipe_en) begin
      delay_sqrt[0] <= {s00_axis_user_x, s00_axis_user_y, s00_axis_user_z, s00_axis_user_shift};
      for (int j = 1; j <= 33; j = j + 1) begin
        delay_sqrt[j] <= delay_sqrt[j-1];
      end
    end
  end

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn == 0) begin
      for (int j = 0; j <= DIV_DLY; j = j + 1) begin
        delay_div[j] <= '0;
      end
    end else if (div_pipe_en) begin
      delay_div[0] <= delay_sqrt[33];
      for (int j = 1; j <= DIV_DLY; j = j + 1) begin
        delay_div[j] <= delay_div[j-1];
      end
    end
  end

  assign m00_axis_user_x     = delay_div[DIV_DLY][99:68];
  assign m00_axis_user_y     = delay_div[DIV_DLY][67:36];
  assign m00_axis_user_z     = delay_div[DIV_DLY][35:4];
  assign m00_axis_user_shift = delay_div[DIV_DLY][3:0];

  // Map to top-level outputs
  assign s00_axis_tready = sqrt_in_ready;
  assign m00_axis_tvalid = div_out_valid;
  assign m00_axis_tdata  = div_out_data;
  assign m00_axis_tlast  = 1'b0;
  assign m00_axis_tstrb  = '1;

endmodule
`default_nettype wire
