`timescale 1ns / 1ps
`default_nettype none

module axis_ggx_control #
  (
		parameter integer C_S00_AXIS_TDATA_WIDTH	= 64,
		parameter integer C_M00_AXIS_TDATA_WIDTH	= 32,   // Oct32: {w_field, u_field}
    parameter integer FRAC_BITS               = 32
	)
  (
  		// Ports of Axi Slave Bus Interface S00_AXIS
		input wire  s00_axis_aclk, s00_axis_aresetn,
		input wire  s00_axis_tlast, s00_axis_tvalid,
    // Input data comes in as 3-beat 64-bit packets
    // Beat0: {burst_len, seed_base}, Beat1: {view_y, view_x}, Beat2: {alpha, view_z}
    // TLAST=1 on the third beat (Beat2, 0-indexed)
		input wire [C_S00_AXIS_TDATA_WIDTH-1 : 0] s00_axis_tdata,
		input wire [(C_S00_AXIS_TDATA_WIDTH/8)-1: 0] s00_axis_tstrb,
		output logic  s00_axis_tready,

		// Ports of Axi Master Bus Interface M00_AXIS
		input wire  m00_axis_aclk, m00_axis_aresetn,
		input wire  m00_axis_tready,
		output logic  m00_axis_tvalid, m00_axis_tlast, // TLAST=1 on last sample of burst
		output logic [C_M00_AXIS_TDATA_WIDTH-1 : 0] m00_axis_tdata, // Oct32 {w_field, u_field}
		output logic [(C_M00_AXIS_TDATA_WIDTH/8)-1: 0] m00_axis_tstrb
	);

  // ---------------------------------------------------------------------------
  // Command parsing (3 beats):
  // Beat0: {burst_len[15:0], seed_base[31:0]}
  // Beat1: {view_y(Q1.31), view_x(Q1.31)}
  // Beat2: {alpha(UQ0.32), view_z(Q1.31)}
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    ST_WAIT_B0     = 3'd0,
    ST_WAIT_B1     = 3'd1,
    ST_WAIT_B2     = 3'd2,
    ST_BASIS_REQ   = 3'd3,
    ST_WAIT_BASIS  = 3'd4,
    ST_SAMPLER_CMD = 3'd5,
    ST_RUN_BURST   = 3'd6
  } state_t;

  state_t state;

  logic [15:0] burst_len_reg;
  logic [31:0] seed_base_reg;
  logic [31:0] alpha_reg;
  logic signed [31:0] view_x_reg, view_y_reg, view_z_reg;

  // Stored basis (per shading event)
  logic basis_valid_reg;
  logic signed [95:0] basis_vh_reg;
  logic signed [95:0] basis_t1_reg;
  logic signed [95:0] basis_t2_reg;

  // ---------------------------------------------------------------------------
  // Event-basis stage (run once per command)
  // ---------------------------------------------------------------------------
  wire basis_in_valid = (state == ST_BASIS_REQ);
  wire basis_in_ready;
  wire basis_out_valid;
  wire basis_out_ready = (state == ST_WAIT_BASIS);
  wire basis_out_fire = basis_out_valid && basis_out_ready;
  wire signed [95:0] basis_out_vh;
  wire signed [95:0] basis_out_t1;
  wire signed [95:0] basis_out_t2;

  wire [127:0] basis_in_data = {
    alpha_reg,
    view_z_reg,
    view_y_reg,
    view_x_reg
  };

  axis_ggx_event_basis u_basis (
    .s00_axis_aclk(s00_axis_aclk),
    .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(1'b0),
    .s00_axis_tvalid(basis_in_valid),
    .s00_axis_tdata(basis_in_data),
    .s00_axis_tstrb('1),
    .s00_axis_tready(basis_in_ready),

    .m00_axis_aclk(s00_axis_aclk),
    .m00_axis_aresetn(s00_axis_aresetn),
    .m00_axis_tready(basis_out_ready),
    .m00_axis_tvalid(basis_out_valid),
    .m00_axis_tlast(),
    .m00_axis_tdata(basis_out_vh),
    .m00_axis_tstrb(),
    .T1(basis_out_t1),
    .T2(basis_out_t2)
  );

  // ---------------------------------------------------------------------------
  // Per-sample u generation (u2,u1)
  // ---------------------------------------------------------------------------
  wire sampler_cmd_valid = (state == ST_SAMPLER_CMD);
  wire sampler_cmd_ready;
  wire [63:0] sampler_cmd_data = {16'b0, burst_len_reg, seed_base_reg};

  wire sampler_out_valid;
  wire sampler_out_ready;
  wire sampler_out_last;
  wire [63:0] sampler_out_data; // {u2, u1} = {dim1, dim0}

  axis_pre_ggx_sampler u_sampler (
    .s00_axis_aclk(s00_axis_aclk),
    .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(1'b1),
    .s00_axis_tvalid(sampler_cmd_valid),
    .s00_axis_tdata(sampler_cmd_data),
    .s00_axis_tstrb('1),
    .s00_axis_tready(sampler_cmd_ready),

    .m00_axis_aclk(s00_axis_aclk),
    .m00_axis_aresetn(s00_axis_aresetn),
    .m00_axis_tready(sampler_out_ready),
    .m00_axis_tvalid(sampler_out_valid),
    .m00_axis_tlast(sampler_out_last),
    .m00_axis_tdata(sampler_out_data),
    .m00_axis_tstrb()
  );

  // ---------------------------------------------------------------------------
  // Projected-area parameterization stage:
  // input  {Vh.z, u2, u1}
  // output {t2, t1}
  // ---------------------------------------------------------------------------
  wire proj_in_valid = (state == ST_RUN_BURST) && basis_valid_reg && sampler_out_valid;
  wire proj_in_ready;
  wire proj_out_valid;
  wire proj_out_last;
  wire signed [63:0] proj_out_t2_t1;

  wire reproj_in_valid;
  wire reproj_in_ready;
  wire reproj_out_valid;
  wire reproj_out_last;
  wire [31:0] reproj_out_h;   // Oct32 fields

  wire [95:0] proj_in_data = {
    basis_vh_reg[95:64],     // Vh.z (Q1.31)
    sampler_out_data[63:32], // u2 (UQ0.32)
    sampler_out_data[31:0]   // u1 (UQ0.32)
  };

  assign sampler_out_ready = basis_valid_reg && (state == ST_RUN_BURST) && proj_in_ready;

  wire proj_to_skid_ready;
  wire [63:0] skid_proj_out_t2_t1;
  wire skid_proj_out_valid;
  wire skid_proj_out_last;
  wire skid_to_reproj_ready;

  axis_fifo_2deep #(
    .DATA_WIDTH(64)
  ) u_skid_proj (
    .clk(s00_axis_aclk),
    .resetn(s00_axis_aresetn),
    .s_axis_tvalid(proj_out_valid),
    .s_axis_tready(proj_to_skid_ready),
    .s_axis_tdata(proj_out_t2_t1),
    .s_axis_tlast(proj_out_last),
    .m_axis_tvalid(skid_proj_out_valid),
    .m_axis_tready(skid_to_reproj_ready),
    .m_axis_tdata(skid_proj_out_t2_t1),
    .m_axis_tlast(skid_proj_out_last)
  );

  axis_ggx_projected_area u_projected_area (
    .s00_axis_aclk(s00_axis_aclk),
    .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(sampler_out_last),
    .s00_axis_tvalid(proj_in_valid),
    .s00_axis_tdata(proj_in_data),
    .s00_axis_tstrb('1),
    .s00_axis_tready(proj_in_ready),

    .m00_axis_aclk(s00_axis_aclk),
    .m00_axis_aresetn(s00_axis_aresetn),
    .m00_axis_tready(proj_to_skid_ready),
    .m00_axis_tvalid(proj_out_valid),
    .m00_axis_tlast(proj_out_last),
    .m00_axis_tdata(proj_out_t2_t1),
    .m00_axis_tstrb()
  );

  // ---------------------------------------------------------------------------
  // Reprojection + normalization stage:
  // input  {Vh, T2, T1, {t2,t1}}
  // output {hz, hy, hx}
  // ---------------------------------------------------------------------------
  wire [351:0] reproj_in_data = {
    basis_vh_reg,
    basis_t2_reg,
    basis_t1_reg,
    skid_proj_out_t2_t1
  };

  axis_ggx_reproject_normalize u_reproject_normalize (
    .s00_axis_aclk(s00_axis_aclk),
    .s00_axis_aresetn(s00_axis_aresetn),
    .s00_axis_tlast(skid_proj_out_last),
    .s00_axis_tvalid(skid_proj_out_valid),
    .s00_axis_tdata(reproj_in_data),
    .s00_axis_tstrb('1),
    .s00_axis_tready(skid_to_reproj_ready),

    .m00_axis_aclk(s00_axis_aclk),
    .m00_axis_aresetn(s00_axis_aresetn),
    .m00_axis_tready(m00_axis_tready),
    .m00_axis_tvalid(reproj_out_valid),
    .m00_axis_tlast(reproj_out_last),
    .m00_axis_tdata(reproj_out_h),
    .m00_axis_tstrb()
  );
  assign reproj_in_valid = skid_proj_out_valid;
  assign reproj_in_ready = skid_to_reproj_ready;

  // ---------------------------------------------------------------------------
  // Control FSM
  // ---------------------------------------------------------------------------
  wire cmd_fire = s00_axis_tvalid && s00_axis_tready;
  wire reproj_fire = reproj_out_valid && m00_axis_tready;

  always_ff @(posedge s00_axis_aclk) begin
    if (s00_axis_aresetn==0) begin
      state <= ST_WAIT_B0;
      burst_len_reg <= '0;
      seed_base_reg <= '0;
      alpha_reg <= '0;
      view_x_reg <= '0;
      view_y_reg <= '0;
      view_z_reg <= '0;
      basis_valid_reg <= 1'b0;
      basis_vh_reg <= '0;
      basis_t1_reg <= '0;
      basis_t2_reg <= '0;
    end else begin
      case (state)
        ST_WAIT_B0: begin
          if (cmd_fire) begin
            basis_valid_reg <= 1'b0;
            if (s00_axis_tlast) begin
              state <= ST_WAIT_B0;
            end else begin
              seed_base_reg <= s00_axis_tdata[31:0];
              burst_len_reg <= s00_axis_tdata[47:32];
              state <= ST_WAIT_B1;
            end
          end
        end

        ST_WAIT_B1: begin
          if (cmd_fire) begin
            if (s00_axis_tlast) begin
              state <= ST_WAIT_B0;
            end else begin
              view_x_reg <= $signed(s00_axis_tdata[31:0]);
              view_y_reg <= $signed(s00_axis_tdata[63:32]);
              state <= ST_WAIT_B2;
            end
          end
        end

        ST_WAIT_B2: begin
          if (cmd_fire) begin
            if (!s00_axis_tlast) begin
              state <= ST_WAIT_B0;
            end else begin
              view_z_reg <= $signed(s00_axis_tdata[31:0]);
              alpha_reg <= s00_axis_tdata[63:32];
              state <= ST_BASIS_REQ;
            end
          end
        end

        ST_BASIS_REQ: begin
          if (basis_in_valid && basis_in_ready) begin
            state <= ST_WAIT_BASIS;
          end
        end

        ST_WAIT_BASIS: begin
          if (basis_out_fire) begin
            basis_vh_reg <= basis_out_vh;
            basis_t1_reg <= basis_out_t1;
            basis_t2_reg <= basis_out_t2;
            basis_valid_reg <= 1'b1;
            state <= ST_SAMPLER_CMD;
          end
        end

        ST_SAMPLER_CMD: begin
          if (sampler_cmd_valid && sampler_cmd_ready) begin
            state <= ST_RUN_BURST;
          end
        end

        ST_RUN_BURST: begin
          if (reproj_fire && reproj_out_last) begin
            state <= ST_WAIT_B0;
          end
        end

        default: state <= ST_WAIT_B0;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // Top-level AXIS mapping
  // Current output emits reprojected+normalized hemisphere vector {hz, hy, hx}.
  // A later stage can still apply ellipsoid back-transform if needed.
  // ---------------------------------------------------------------------------
  wire cmd_parse_state = (state == ST_WAIT_B0) ||
                         (state == ST_WAIT_B1) ||
                         (state == ST_WAIT_B2);

  assign s00_axis_tready = cmd_parse_state;

  assign m00_axis_tvalid = reproj_out_valid;
  assign m00_axis_tlast  = reproj_out_last;
  assign m00_axis_tdata  = reproj_out_h;
  assign m00_axis_tstrb  = '1;

endmodule
`default_nettype wire
