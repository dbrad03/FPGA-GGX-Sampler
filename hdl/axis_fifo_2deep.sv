`timescale 1ns / 1ps
`default_nettype none

module axis_fifo_2deep #
  (
    parameter integer DATA_WIDTH = 64
  )
  (
    input wire                    clk,
    input wire                    resetn,

    // Slave Port
    input wire                    s_axis_tvalid,
    output logic                  s_axis_tready,
    input wire [DATA_WIDTH-1:0]   s_axis_tdata,
    input wire                    s_axis_tlast,

    // Master Port
    output logic                  m_axis_tvalid,
    input wire                    m_axis_tready,
    output logic [DATA_WIDTH-1:0] m_axis_tdata,
    output logic                  m_axis_tlast
  );

  // 2-deep FIFO storage
  logic [DATA_WIDTH-1:0] mem [0:1];
  logic                  mem_last [0:1];
  logic [1:0]            count;
  logic                  rd_ptr;
  logic                  wr_ptr;

  always_ff @(posedge clk) begin
    if (!resetn) begin
      count  <= 2'd0;
      rd_ptr <= 1'b0;
      wr_ptr <= 1'b0;
    end else begin
      case ({s_axis_tvalid && s_axis_tready, m_axis_tvalid && m_axis_tready})
        2'b10: count <= count + 1'b1;
        2'b01: count <= count - 1'b1;
        default: ;
      endcase

      if (s_axis_tvalid && s_axis_tready) begin
        mem[wr_ptr] <= s_axis_tdata;
        mem_last[wr_ptr] <= s_axis_tlast;
        wr_ptr <= wr_ptr + 1'b1;
      end

      if (m_axis_tvalid && m_axis_tready) begin
        rd_ptr <= rd_ptr + 1'b1;
      end
    end
  end

  assign m_axis_tdata  = mem[rd_ptr];
  assign m_axis_tlast  = mem_last[rd_ptr];
  assign m_axis_tvalid = (count > 0);

  always_ff @(posedge clk) begin
    if (!resetn) begin
      s_axis_tready <= 1'b1;
    end else begin
      case ({s_axis_tvalid && s_axis_tready, m_axis_tvalid && m_axis_tready})
        2'b10: s_axis_tready <= (count + 1'b1 < 2'd2);
        2'b01: s_axis_tready <= (count - 1'b1 < 2'd2);
        default: s_axis_tready <= (count < 2'd2);
      endcase
    end
  end

endmodule
`default_nettype wire
