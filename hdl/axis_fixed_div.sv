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
// BACKPRESSURE (parameter ELASTIC):
//
//   ELASTIC = 1 (default, STALLABLE) -- one pipe_en gates all 2*(W+F) stages.
//     Simple and correct, and the enable reaches every FF and every SRL in the
//     core. The carried divisor/dividend/trial/shft become SRL32E rather than
//     SRL32, and their CE pins are timing endpoints. In oct32's two dividers
//     that was 694 failing endpoints -- 56% of the design's total -- one logic
//     level deep and 88% route. Nothing on a path like that can be pipelined.
//
//   ELASTIC = 0 (RIGID) -- the recurrence free-runs and can never be told to
//     wait, so it carries no enable at all. Instead the core refuses beats at
//     its slave port unless a slot is already reserved in an output ring deep
//     enough to hold everything the pipeline can still deliver. Costs one cycle
//     of latency (ggx_latency_pkg::div_latency_rigid) and one ring per core.
//
// The recurrence below is shared by both: the arithmetic is not duplicated per
// mode, so a rigid divider is bit-identical to a stallable one by construction.
// See issue #33 and docs/adr/0003-ready-is-registered-at-core-inputs.md.
module axis_fixed_div #
  (
    parameter integer WIDTH     = 32,
    parameter integer FRAC_BITS = 25,
    parameter integer ELASTIC   = 1
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
  assign m00_axis_tlast = 1'b0;
  assign m00_axis_tstrb = '1;

  localparam int ITERS      = WIDTH + FRAC_BITS; // 57 division steps
  // The package defines this core's latency; the core sizes itself to match.
  // See docs/adr/0002-latency-package-is-law.md. The recurrence is the same
  // depth in both modes -- only the tail differs -- so NST comes from the
  // stallable figure either way.
  localparam int CORE_LAT   = ggx_latency_pkg::div_latency(WIDTH, FRAC_BITS);
  localparam int NST        = CORE_LAT - 2;      // two pipeline stages per step

  // A beat is taken when it is accepted, in both modes. Under ELASTIC=1 the
  // block below only runs while pipe_en is high and s00_axis_tready IS pipe_en,
  // so this is exactly the old `if (s00_axis_tvalid)` and the loaded values are
  // unchanged.
  wire in_fire = s00_axis_tvalid && s00_axis_tready;
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
    end else if (pipe_en) begin
      // STAGE 0 LOAD
      valid[0] <= in_fire;
      if (in_fire) begin
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
    end
  end

  // ---------------------------------------------------------------------------
  // The tail, and the only thing that differs between the two modes.
  // ---------------------------------------------------------------------------
  generate
    if (ELASTIC) begin : g_elastic
      // One enable for the whole core, derived from the consumer's ready.
      assign pipe_en = m00_axis_tready || !m00_axis_tvalid;
      assign s00_axis_tready = pipe_en;

      always_ff @(posedge s00_axis_aclk) begin
        if (s00_axis_aresetn == 0) begin
          m00_axis_tvalid <= 1'b0;
          m00_axis_tdata  <= '0;
        end else if (pipe_en) begin
          m00_axis_tvalid <= valid[NST];
          if (valid[NST]) m00_axis_tdata <= q[NST][WIDTH-1:0];
        end
      end
    end else begin : g_rigid
      // Nothing to enable. This is the whole point: no CE on any recurrence FF,
      // and the carried divisor/dividend map to plain SRLs.
      assign pipe_en = 1'b1;

      localparam int RING_DEPTH =
          ggx_latency_pkg::rigid_ring_depth(
              ggx_latency_pkg::div_latency_rigid(WIDTH, FRAC_BITS));
      localparam int RING_AW = $clog2(RING_DEPTH);

      logic [WIDTH-1:0]   ring [0:RING_DEPTH-1];
      logic [RING_AW-1:0] ring_wr_ptr, ring_rd_ptr;
      logic [RING_AW:0]   ring_count;
      // Credits are the reservations: one is spent when a beat is accepted and
      // returned when that beat leaves the ring. Since a credit exists for
      // every ring slot, `outstanding <= RING_DEPTH` holds, so the ring can
      // never be written when it is full and the pipeline never has to stop.
      logic [RING_AW:0]   credits;

      wire ring_valid   = (ring_count != 0);
      wire out_pipe_en  = m00_axis_tready || !m00_axis_tvalid;
      wire ring_pop     = ring_valid && out_pipe_en;

      // The core's ONLY reason to refuse a beat. Note what it does not depend
      // on: m00_axis_tready. Backpressure reaches this port through the credit
      // register, one cycle later, which makes this core a Cut (ADR-0003).
      assign s00_axis_tready = (credits != 0);

      always_ff @(posedge s00_axis_aclk) begin
        if (s00_axis_aresetn == 0) begin
          ring_wr_ptr <= '0;
          ring_rd_ptr <= '0;
          ring_count  <= '0;
          credits     <= RING_DEPTH[RING_AW:0];
          m00_axis_tvalid <= 1'b0;
          m00_axis_tdata  <= '0;
        end else begin
          if (valid[NST]) begin
            ring[ring_wr_ptr] <= q[NST][WIDTH-1:0];
            ring_wr_ptr <= ring_wr_ptr + 1'b1;
          end
          if (ring_pop) ring_rd_ptr <= ring_rd_ptr + 1'b1;

          case ({valid[NST], ring_pop})
            2'b10: ring_count <= ring_count + 1'b1;
            2'b01: ring_count <= ring_count - 1'b1;
            default: ;
          endcase

          case ({in_fire, ring_pop})
            2'b10: credits <= credits - 1'b1;
            2'b01: credits <= credits + 1'b1;
            default: ;
          endcase

          if (out_pipe_en) begin
            m00_axis_tvalid <= ring_valid;
            if (ring_valid) m00_axis_tdata <= ring[ring_rd_ptr];
          end
        end
      end
    end
  endgenerate

endmodule
`default_nettype wire
