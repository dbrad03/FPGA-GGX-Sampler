`timescale 1ns / 1ps
`default_nettype none

module axis_pre_ggx_sampler #(
    parameter integer C_S_AXIS_TDATA_WIDTH = 64,
    parameter integer C_M_AXIS_TDATA_WIDTH = 64 // 32-bit Dim1 + 32-bit Dim0
)
(
    input wire  s00_axis_aclk, s00_axis_aresetn,
    input wire  s00_axis_tlast, s00_axis_tvalid,
    input wire [C_S_AXIS_TDATA_WIDTH-1 : 0] s00_axis_tdata, //{burst_len, seed_base}
    input wire [(C_S_AXIS_TDATA_WIDTH/8)-1: 0] s00_axis_tstrb,
    output wire s00_axis_tready,

    input wire  m00_axis_aclk, m00_axis_aresetn,
    input wire  m00_axis_tready,
    output wire m00_axis_tvalid, m00_axis_tlast,
    output wire [C_M_AXIS_TDATA_WIDTH-1 : 0] m00_axis_tdata,
    output wire [(C_M_AXIS_TDATA_WIDTH/8)-1: 0] m00_axis_tstrb
);

    localparam [(C_S_AXIS_TDATA_WIDTH/8)-1:0] AXIS_STRB_ALL = {(C_S_AXIS_TDATA_WIDTH/8){1'b1}};

    // -------------------------------------------------------------------------
    // 1. Shared Sampler (Burst Generator)
    // -------------------------------------------------------------------------
    wire [63:0] samp_tdata;
    wire samp_tvalid, samp_tready, samp_tlast;

    axis_top_lvl_sampler #(
        .C_S00_AXIS_TDATA_WIDTH(64), 
        .C_M00_AXIS_TDATA_WIDTH(64)
    ) sampler_inst (
        .s00_axis_aclk(s00_axis_aclk), 
        .s00_axis_aresetn(s00_axis_aresetn),
        .s00_axis_tstrb(s00_axis_tstrb),
        .s00_axis_tdata(s00_axis_tdata), 
        .s00_axis_tvalid(s00_axis_tvalid), 
        .s00_axis_tlast(s00_axis_tlast),
        .s00_axis_tready(s00_axis_tready),

        .m00_axis_aclk(m00_axis_aclk), 
        .m00_axis_aresetn(m00_axis_aresetn),
        // .m00_axis_tstrb() - Left unconnected (internal signal)
        .m00_axis_tdata(samp_tdata), 
        .m00_axis_tvalid(samp_tvalid), 
        .m00_axis_tlast(samp_tlast),
        .m00_axis_tready(samp_tready),
        .m00_axis_tstrb() 
    );

    // -------------------------------------------------------------------------
    // 2. Shared Delay Compensation
    // The sobol path is shorter than the hash path, so the index waits for the
    // difference before the scramble consumes both. Both numbers now come from
    // ggx_latency_pkg, so re-pipelining either block updates this delay instead
    // of silently mis-pairing index with seed.
    // See docs/adr/0002-latency-package-is-law.md.
    // -------------------------------------------------------------------------
    localparam integer ALIGN_DELAY = ggx_latency_pkg::sampler_index_align_delay();
    reg [31:0] delay_index [0:ALIGN_DELAY-1];
    reg [ALIGN_DELAY-1:0] delay_valid;
    reg [ALIGN_DELAY-1:0] delay_last;
    integer i;

    always @(posedge s00_axis_aclk) begin
        if (!s00_axis_aresetn) begin
            delay_valid <= {ALIGN_DELAY{1'b0}};
            delay_last  <= {ALIGN_DELAY{1'b0}};
            for (i=0; i<ALIGN_DELAY; i=i+1) delay_index[i] <= 32'b0;
        end else if (samp_tready) begin
            delay_index[0] <= {16'b0, samp_tdata[47:32]};
            delay_valid[0] <= samp_tvalid;
            delay_last[0]  <= samp_tlast;
            for(i=0; i<ALIGN_DELAY-1; i=i+1) begin
                delay_index[i+1] <= delay_index[i];
                delay_valid[i+1] <= delay_valid[i];
                delay_last[i+1]  <= delay_last[i];
            end
        end
    end

    wire [63:0] hash_in_data  = samp_tdata;
    wire [63:0] sobol_in_data = {32'b0, delay_index[ALIGN_DELAY-1]};
    wire sobol_in_valid = delay_valid[ALIGN_DELAY-1];
    wire sobol_in_last  = delay_last[ALIGN_DELAY-1];

    // -------------------------------------------------------------------------
    // 3. Parallel Paths
    // -------------------------------------------------------------------------
    wire [63:0] hash0_out, sobol0_out, scram0_out;
    wire [63:0] hash1_out, sobol1_out, scram1_out;
    wire hash0_v, hash0_l, sobol0_v;
    wire hash1_v, hash1_l, sobol1_v;
    
    // Internal Backpressure Wires
    // These connect Scrambler INPUT READY to Hash/Sobol OUTPUT READY
    wire scram0_in_ready; 
    wire scram1_in_ready;

    // --- DIMENSION 0 ---
    axis_hash_combine_2d   #(.DIMENSION(0)) hash0  (
        .s00_axis_aclk(s00_axis_aclk), .s00_axis_aresetn(s00_axis_aresetn), 
        .s00_axis_tdata(hash_in_data), .s00_axis_tvalid(samp_tvalid), 
        .s00_axis_tlast(samp_tlast),   .s00_axis_tstrb(AXIS_STRB_ALL), .s00_axis_tready(samp_tready), // Controls upstream Sampler
        .m00_axis_aclk(m00_axis_aclk), .m00_axis_aresetn(m00_axis_aresetn), 
        .m00_axis_tdata(hash0_out),    .m00_axis_tvalid(hash0_v), 
        .m00_axis_tlast(hash0_l),      .m00_axis_tstrb(), .m00_axis_tready(scram0_in_ready)
    );

    axis_sobol2d_stateless #(.DIMENSION(0)) sobol0 (
        .s00_axis_aclk(s00_axis_aclk), .s00_axis_aresetn(s00_axis_aresetn), 
        .s00_axis_tdata(sobol_in_data), .s00_axis_tvalid(sobol_in_valid), 
        .s00_axis_tlast(sobol_in_last), .s00_axis_tstrb(AXIS_STRB_ALL), .s00_axis_tready(),
        .m00_axis_aclk(m00_axis_aclk),  .m00_axis_aresetn(m00_axis_aresetn), 
        .m00_axis_tdata(sobol0_out),    .m00_axis_tvalid(sobol0_v), 
        .m00_axis_tlast(),              .m00_axis_tstrb(), .m00_axis_tready(scram0_in_ready)
    );
    
    wire [63:0] scram0_in = {hash0_out[31:0], sobol0_out[31:0]};
    wire scram0_v = hash0_v && sobol0_v;
    wire scram0_out_v, scram0_out_l;

    axis_nested_uniform_scramble scram0 (
        .s00_axis_aclk(s00_axis_aclk), .s00_axis_aresetn(s00_axis_aresetn), 
        .s00_axis_tdata(scram0_in),    .s00_axis_tvalid(scram0_v), 
        .s00_axis_tlast(hash0_l),      .s00_axis_tstrb(AXIS_STRB_ALL), .s00_axis_tready(scram0_in_ready), // Driving upstream
        .m00_axis_aclk(m00_axis_aclk), .m00_axis_aresetn(m00_axis_aresetn), 
        .m00_axis_tdata(scram0_out),   .m00_axis_tvalid(scram0_out_v), 
        .m00_axis_tlast(scram0_out_l), .m00_axis_tready(m00_axis_tready),
        .m00_axis_tstrb()
    );

    // --- DIMENSION 1 ---
    axis_hash_combine_2d   #(.DIMENSION(1)) hash1  (
        .s00_axis_aclk(s00_axis_aclk), .s00_axis_aresetn(s00_axis_aresetn), 
        .s00_axis_tdata(hash_in_data), .s00_axis_tvalid(samp_tvalid), 
        .s00_axis_tlast(samp_tlast),   .s00_axis_tstrb(AXIS_STRB_ALL), .s00_axis_tready(),
        .m00_axis_aclk(m00_axis_aclk), .m00_axis_aresetn(m00_axis_aresetn), 
        .m00_axis_tdata(hash1_out),    .m00_axis_tvalid(hash1_v), 
        .m00_axis_tlast(hash1_l),      .m00_axis_tstrb(), .m00_axis_tready(scram1_in_ready)
    );

    axis_sobol2d_stateless #(.DIMENSION(1)) sobol1 (
        .s00_axis_aclk(s00_axis_aclk), .s00_axis_aresetn(s00_axis_aresetn), 
        .s00_axis_tdata(sobol_in_data), .s00_axis_tvalid(sobol_in_valid), 
        .s00_axis_tlast(sobol_in_last), .s00_axis_tstrb(AXIS_STRB_ALL), .s00_axis_tready(), 
        .m00_axis_aclk(m00_axis_aclk),  .m00_axis_aresetn(m00_axis_aresetn), 
        .m00_axis_tdata(sobol1_out),    .m00_axis_tvalid(sobol1_v), 
        .m00_axis_tlast(),              .m00_axis_tstrb(), .m00_axis_tready(scram1_in_ready)
    );

    wire [63:0] scram1_in = {hash1_out[31:0], sobol1_out[31:0]};
    wire scram1_v = hash1_v && sobol1_v;
    wire scram1_out_v;

    axis_nested_uniform_scramble scram1 (
        .s00_axis_aclk(s00_axis_aclk), .s00_axis_aresetn(s00_axis_aresetn), 
        .s00_axis_tdata(scram1_in),    .s00_axis_tvalid(scram1_v), 
        .s00_axis_tlast(hash1_l),      .s00_axis_tstrb(AXIS_STRB_ALL), .s00_axis_tready(scram1_in_ready), 
        .m00_axis_aclk(m00_axis_aclk), .m00_axis_aresetn(m00_axis_aresetn), 
        .m00_axis_tdata(scram1_out),   .m00_axis_tvalid(scram1_out_v), 
        .m00_axis_tlast(),             .m00_axis_tready(m00_axis_tready),
        .m00_axis_tstrb()
    );

    // -------------------------------------------------------------------------
    // 4. Output Packing
    // -------------------------------------------------------------------------
    assign m00_axis_tvalid = scram0_out_v && scram1_out_v;
    assign m00_axis_tlast  = scram0_out_l;
    
    // Pack: {Dim1_Val (32), Dim0_Val (32)}
    assign m00_axis_tdata = {scram1_out[31:0], scram0_out[31:0]};
    assign m00_axis_tstrb = 8'hFF;

endmodule
`default_nettype wire
