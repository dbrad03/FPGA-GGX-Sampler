`timescale 1ns / 1ps
`default_nettype none

module axis_sobol #(
    parameter int DIMS       = 2,
    parameter int FRAC_BITS  = 32,
    parameter int INDEX_BITS = 32,
    parameter int NUM_BITS   = 32, // number of direction bits
    parameter C_S00_AXIS_TDATA_WIDTH = 32,
    parameter C_M00_AXIS_TDATA_WIDTH = 2*FRAC_BITS
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
    output logic [(C_M00_AXIS_TDATA_WIDTH/8)-1: 0] m00_axis_tstrb
);

    // `include "sobol_dir_table_pkg.sv"
    // import sobol_dir_table_pkg::*;
    `include "sobol_dir_table.svh"

    localparam SHIFT = $clog2(FRAC_BITS);

    // Internal state: x_n^{(d)} for each dimension
    logic [FRAC_BITS-1:0] state [DIMS];
    logic [INDEX_BITS-1:0] index_reg;

    logic [$clog2(NUM_BITS+1)-1:0] bit_idx_reg;

    // Output data
    logic                              m00_axis_tvalid_reg;
    logic [C_M00_AXIS_TDATA_WIDTH-1:0] m00_axis_tdata_reg;
    
    assign s00_axis_tready = 1'b1;
    assign m00_axis_tvalid = m00_axis_tvalid_reg;
    assign m00_axis_tdata = m00_axis_tdata_reg;
    assign m00_axis_tstrb = {C_M00_AXIS_TDATA_WIDTH/8{1'b1}};
    assign m00_axis_tlast = 1'b0;

    wire advance = m00_axis_tready || ~m00_axis_tvalid_reg;
    /// COUNT TRAILING ZEROS of INDEX
    function automatic int ctz(input logic [INDEX_BITS-1:0] x);
        int i;
        begin
            ctz = INDEX_BITS;
            for (i = 0; i < INDEX_BITS; i++) begin
                if (x[i]) begin
                    ctz = i;
                    return ctz;
                end
            end
        end
    endfunction

    always_ff @(posedge s00_axis_aclk) begin
        if (s00_axis_aresetn == 0) begin
            for (int d = 0; d < DIMS; d++) begin
                state[d] <= '0;
            end
            index_reg <= '0;
            bit_idx_reg <= '0;

            m00_axis_tvalid_reg <= 0;
            m00_axis_tdata_reg <= '0;
        end else begin
            if (advance) begin
                logic [INDEX_BITS-1:0] index_next;
                logic [C_M00_AXIS_TDATA_WIDTH-1:0] packed_data;
                packed_data = '0;


                // STAGE 1 : calculate ctz
                index_next = index_reg + 1;
                index_reg <= index_next;
                bit_idx_reg <= ctz(index_next + 1);

                // STAGE 2 : update state and pack output
                for (int d = 0; d < DIMS; d++) begin
                logic [FRAC_BITS-1:0] new_state;
                new_state = state[d];
                if (bit_idx_reg < NUM_BITS)
                    new_state = state[d] ^ sobol_dir(d,bit_idx_reg);
                state[d] <= new_state;

                packed_data = packed_data | (C_M00_AXIS_TDATA_WIDTH'(new_state) << (d << SHIFT));
                end 

                m00_axis_tdata_reg <= packed_data;
                m00_axis_tvalid_reg <= 1'b1;
                // end
            end
        end
    end


endmodule


`default_nettype wire

