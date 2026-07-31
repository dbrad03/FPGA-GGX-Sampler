`timescale 1ns / 1ps
`default_nettype none
// Non-restoring binary long division, digit-recurrence, fully pipelined.
//
// Each iteration is SPLIT across two pipeline stages so the wide subtract no
// longer shares a cycle with the borrow-check remainder mux + quotient update:
//   SUB stage:    trial_sub = shifted_rem - divisor        (the WIDTH+1 subtract)
//   SELECT stage: borrow = trial_sub[WIDTH];               (mux + quotient bit)
//                 rem  = borrow ? shifted_rem : trial_sub
//                 q    = (q<<1) | ~borrow
// This isolates the subtract's carry chain from the select logic (was one fused
// stage). Doubles the stage count (latency, not throughput); result bit-identical.
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

  wire _unused_s00_tlast = s00_axis_tlast;
  wire _unused_s00_tstrb = ^s00_axis_tstrb;
  wire _unused_m00_aclk = m00_axis_aclk;
  wire _unused_m00_aresetn = m00_axis_aresetn;

  logic pipe_en;
  assign pipe_en = m00_axis_tready || !m00_axis_tvalid;
  assign s00_axis_tready = pipe_en;
  assign m00_axis_tlast = 1'b0;
  assign m00_axis_tstrb = '1;

  localparam int ITERS      = WIDTH + FRAC_BITS; // 57 division steps
  // The package defines this core's latency; the core sizes itself to match.
  // See docs/adr/0002-latency-package-is-law.md.
  localparam int LATENCY    = ggx_latency_pkg::div_latency(WIDTH, FRAC_BITS);
  localparam int NST        = LATENCY - 2;       // two pipeline stages per step
  localparam int DIVIDEND_W = WIDTH + FRAC_BITS;

  logic [NST:0]          valid;
  logic [DIVIDEND_W-1:0] dividend [0:NST];
  logic [WIDTH-1:0]      divisor  [0:NST];
  logic [WIDTH-1:0]      rem      [0:NST];
  logic [DIVIDEND_W-1:0] q        [0:NST];
  logic [WIDTH:0]        trial    [0:NST]; // trial_sub, SUB stage -> SELECT stage
  logic [WIDTH:0]        shft     [0:NST]; // shifted_rem, SUB stage -> SELECT stage

  // combinational temporaries (module scope for Icarus)
  logic [WIDTH:0] shifted_c;
  logic           borrow_c;
  integer j;

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn == 0) begin
      for (j = 0; j <= NST; j = j + 1) begin
        valid[j]    <= 1'b0;
        dividend[j] <= '0;
        divisor[j]  <= '0;
        rem[j]      <= '0;
        q[j]        <= '0;
        trial[j]    <= '0;
        shft[j]     <= '0;
      end
      m00_axis_tvalid <= 1'b0;
      m00_axis_tdata  <= '0;
    end else if (pipe_en) begin
      // STAGE 0 LOAD
      valid[0] <= s00_axis_tvalid;
      if (s00_axis_tvalid) begin
        dividend[0] <= {s00_axis_tdata[2*WIDTH-1 : WIDTH], {FRAC_BITS{1'b0}}};
        divisor[0]  <= s00_axis_tdata[WIDTH-1 : 0];
      end else begin
        dividend[0] <= '0;
        divisor[0]  <= '0;
      end
      rem[0]   <= '0;
      q[0]     <= '0;
      trial[0] <= '0;
      shft[0]  <= '0;

      for (j = 0; j < NST; j = j + 1) begin
        valid[j+1]   <= valid[j];
        divisor[j+1] <= divisor[j];
        if (j % 2 == 0) begin
          // SUB stage: compute the WIDTH+1 subtract only
          shifted_c    = {rem[j], dividend[j][DIVIDEND_W-1]};
          shft[j+1]    <= shifted_c;
          trial[j+1]   <= shifted_c - {1'b0, divisor[j]};
          rem[j+1]     <= rem[j];
          dividend[j+1] <= dividend[j];
          q[j+1]       <= q[j];
        end else begin
          // SELECT stage: borrow check -> remainder mux + quotient bit + shift
          borrow_c      = trial[j][WIDTH];
          rem[j+1]      <= borrow_c ? shft[j][WIDTH-1:0] : trial[j][WIDTH-1:0];
          q[j+1]        <= (q[j] << 1) | (borrow_c ? 1'b0 : 1'b1);
          dividend[j+1] <= dividend[j] << 1;
          trial[j+1]    <= trial[j];
          shft[j+1]     <= shft[j];
        end
      end

      // OUTPUT REGISTER
      m00_axis_tvalid <= valid[NST];
      if (valid[NST]) m00_axis_tdata <= q[NST][WIDTH-1:0];
    end
  end

endmodule
`default_nettype wire
