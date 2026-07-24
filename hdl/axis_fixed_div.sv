`timescale 1ns / 1ps
`default_nettype none

module axis_fixed_div #
  (
    parameter integer WIDTH     = 32,
    parameter integer FRAC_BITS = 25
  )
  (
    input wire  s00_axis_aclk, s00_axis_aresetn,
    input wire  s00_axis_tlast, s00_axis_tvalid,
    // s00_axis_tdata: {A[31:0] (dividend), B[31:0] (divisor)}
    input wire [2*WIDTH-1 : 0] s00_axis_tdata,
    input wire [(2*WIDTH/8)-1: 0] s00_axis_tstrb,
    output logic  s00_axis_tready,

    input wire  m00_axis_aclk, m00_axis_aresetn,
    input wire  m00_axis_tready,
    output logic  m00_axis_tvalid, m00_axis_tlast,
    output logic [WIDTH-1 : 0] m00_axis_tdata,
    output logic [(WIDTH/8)-1: 0] m00_axis_tstrb
  );

  // Unused AXIS signals
  wire _unused_s00_tlast = s00_axis_tlast;
  wire _unused_s00_tstrb = ^s00_axis_tstrb;
  wire _unused_m00_aclk = m00_axis_aclk;
  wire _unused_m00_aresetn = m00_axis_aresetn;

  logic pipe_en;
  assign pipe_en = m00_axis_tready || !m00_axis_tvalid;
  assign s00_axis_tready = pipe_en;
  assign m00_axis_tlast = 1'b0;
  assign m00_axis_tstrb = '1;

  localparam int STAGES = WIDTH + FRAC_BITS; // 32 + 25 = 57 stages
  localparam int DIVIDEND_W = WIDTH + FRAC_BITS;

  logic [STAGES:0] valid;
  logic [DIVIDEND_W-1:0] dividend [0:STAGES];
  logic [WIDTH-1:0]      divisor  [0:STAGES];
  logic [WIDTH-1:0]      rem      [0:STAGES];
  logic [DIVIDEND_W-1:0] q        [0:STAGES];

  logic [WIDTH:0] trial_sub; // WIDTH+1 bits to detect borrow
  logic [WIDTH:0] shifted_rem; // WIDTH+1 bits to prevent MSB loss on shift
  logic next_bit;

  integer i;
  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn == 0) begin
      for (i = 0; i <= STAGES; i = i + 1) begin
        valid[i]    <= 1'b0;
        dividend[i] <= '0;
        divisor[i]  <= '0;
        rem[i]      <= '0;
        q[i]        <= '0;
      end
      m00_axis_tvalid <= 1'b0;
      m00_axis_tdata  <= '0;
    end else if (pipe_en) begin
      // STAGE 0 LOAD
      valid[0] <= s00_axis_tvalid;
      if (s00_axis_tvalid) begin
        // dividend is A shifted left by FRAC_BITS
        dividend[0] <= {s00_axis_tdata[2*WIDTH-1 : WIDTH], {FRAC_BITS{1'b0}}};
        divisor[0]  <= s00_axis_tdata[WIDTH-1 : 0];
        rem[0]      <= '0;
        q[0]        <= '0;
      end else begin
        dividend[0] <= '0;
        divisor[0]  <= '0;
        rem[0]      <= '0;
        q[0]        <= '0;
      end

      // STAGES 0 to STAGES-1
      for (i = 0; i < STAGES; i = i + 1) begin
        valid[i+1]   <= valid[i];
        divisor[i+1] <= divisor[i];

        // Shift remainder left by 1, and bring in the MSB of the dividend
        shifted_rem = {rem[i], dividend[i][DIVIDEND_W-1]};
        
        // Subtract divisor
        trial_sub = shifted_rem - {1'b0, divisor[i]};

        if (trial_sub[WIDTH] == 1'b0) begin
          // No borrow, trial_sub is non-negative
          rem[i+1] <= trial_sub[WIDTH-1:0];
          q[i+1]   <= (q[i] << 1) | 1'b1;
        end else begin
          // Borrow occurred, restore remainder
          rem[i+1] <= shifted_rem[WIDTH-1:0];
          q[i+1]   <= (q[i] << 1);
        end

        // Shift dividend left
        dividend[i+1] <= dividend[i] << 1;
      end

      // OUTPUT REGISTER
      m00_axis_tvalid <= valid[STAGES];
      if (valid[STAGES]) begin
        // Since we did WIDTH + FRAC_BITS stages, the quotient is in q[STAGES]
        m00_axis_tdata <= q[STAGES][WIDTH-1:0];
      end
    end
  end

endmodule
`default_nettype wire
