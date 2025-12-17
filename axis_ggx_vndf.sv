`timescale 1ns / 1ps
`default_nettype none

module axis_ggx_vndf #
	(
    parameter int FRAC_BITS  = 32,
    parameter int DIMS       = 3,
    parameter C_S00_AXIS_TDATA_WIDTH = 2*FRAC_BITS, // u0,u1
    parameter C_M00_AXIS_TDATA_WIDTH = DIMS*FRAC_BITS // hx, hy, hz
) (
    // Ports of Axi Slave Bus Interface S00_AXIS
    input wire  s00_axis_aclk, s00_axis_aresetn,
    input wire  s00_axis_tlast, s00_axis_tvalid,
    input wire [C_S00_AXIS_TDATA_WIDTH-1 : 0] s00_axis_tdata, // pack randoms
    input wire [(C_S00_AXIS_TDATA_WIDTH/8)-1: 0] s00_axis_tstrb,
    output logic  s00_axis_tready,

    // Ports of Axi Master Bus Interface M00_AXIS
    input wire  m00_axis_aclk, m00_axis_aresetn,
    input wire  m00_axis_tready,
    output logic  m00_axis_tvalid, m00_axis_tlast,
    output logic [C_M00_AXIS_TDATA_WIDTH-1 : 0] m00_axis_tdata,
    output logic [(C_M00_AXIS_TDATA_WIDTH/8)-1: 0] m00_axis_tstrb,

    // Additional inputs for seed construction
    input wire signed [FRAC_BITS-1:0] view_x,
    input wire signed [FRAC_BITS-1:0] view_y,
    input wire signed [FRAC_BITS-1:0] view_z,
    input wire [FRAC_BITS-1:0]        alpha
);
    // Represents 1.0 in Q1.(FRAC_BITS-1) aka Q1.31 - uses 0x7FFF
    localparam signed [FRAC_BITS-1:0] ONE = {1'b0, {(FRAC_BITS-1){1'b1}}};

    localparam int TRIG_LATENCY     = 2;
    localparam int INV_SQRT_LATENCY = 6;
    localparam int NORM_3_LATENCY   = 9;

    // no backpressure for now.. can wrap in skid buffer
    assign s00_axis_tready = 1'b1;
    assign m00_axis_tlast = 1'b0;
    assign m00_axis_tstrb = '1;

    function automatic signed [FRAC_BITS-1:0] 
      fixed_mul(input signed [FRAC_BITS-1:0] a,
                input signed [FRAC_BITS-1:0] b);
        logic signed [2*FRAC_BITS - 1 : 0] product;
        begin
          product = a * b;
          fixed_mul = product[2*FRAC_BITS-2 : FRAC_BITS-1];
        end
    endfunction

    //////////////////////////////////////////////////////////
    //// STAGE 0.. ACCEPT u0,u1 random values when valid sample
    //////////////////////////////////////////////////////////
    logic [FRAC_BITS-1:0] u0_s0, u1_s0;
    wire [FRAC_BITS-1:0] u0_in, u1_in;
    logic s0_valid;
    wire s0_fire = s00_axis_tvalid & s00_axis_tready;


    assign u0_in = s00_axis_tdata[FRAC_BITS-1:0];
    assign u1_in = s00_axis_tdata[(FRAC_BITS<<1)-1:FRAC_BITS];
    always_ff @(posedge s00_axis_aclk) begin
      if (s00_axis_aresetn == 0) begin
        s0_valid <= 1'b0;
        u0_s0 <= '0;
        u1_s0 <= '0;
      end else begin
        s0_valid <= s00_axis_tvalid;
        if (s00_axis_tvalid) begin
          u0_s0 <= u0_in;
          u1_s0 <= u1_in;
        end
      end
    end
    //////////////////////////////////////////////////////////
    //// END STAGE 0
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //// STAGE 1..warp view and normalize
    //////////////////////////////////////////////////////////
    logic signed [FRAC_BITS-1:0] wi_tmp_x, wi_tmp_y, wi_tmp_z;

    always_ff @(posedge s00_axis_aclk) begin
      if (s00_axis_aresetn == 0) begin
        wi_tmp_x <= '0;
        wi_tmp_y <= '0;
        wi_tmp_z <= '0;
      end else begin
        if (s0_valid) begin
          wi_tmp_x <= fixed_mul(view_x, alpha);
          wi_tmp_y <= view_y;
          wi_tmp_z <= fixed_mul(view_z, alpha);
        end
      end
    end

    logic s1_norm_valid;
    logic signed [FRAC_BITS-1:0] wi_x, wi_y, wi_z;

    // Has approximately 7 cycle delay
    fixed_norm3 #(
      .FRAC_BITS(FRAC_BITS)
    ) u_norm_wi (
      .clk       (s00_axis_aclk),
      .rst_n     (s00_axis_aresetn),
      .in_valid  (s0_valid),
      .in_x      (wi_tmp_x),
      .in_y      (wi_tmp_y),
      .in_z      (wi_tmp_z),
      .out_valid (s1_norm_valid),
      .out_x     (wi_x),
      .out_y     (wi_y),
      .out_z     (wi_z)
    );

    logic [FRAC_BITS-1:0] u0_s1, u1_s1;
    logic [FRAC_BITS-1:0] u0_pipe [0:NORM_3_LATENCY-1];
    logic [FRAC_BITS-1:0] u1_pipe [0:NORM_3_LATENCY-1];
    always_ff @(posedge s00_axis_aclk) begin
      if (s00_axis_aresetn==0) begin
        u0_s1 <= '0;
        u1_s1 <= '0;
        for (int i = 0; i < NORM_3_LATENCY; i++) begin
          u0_pipe[i] <= '0;
          u1_pipe[i] <= '0;
        end
      end else begin
        u0_pipe[0] <= s0_valid ? u0_s0 : '0;
        u1_pipe[0] <= s0_valid ? u1_s0 : '0;

        for (int i = 1; i < NORM_3_LATENCY; i++) begin
          u0_pipe[i] <= u0_pipe[i-1];
          u1_pipe[i] <= u1_pipe[i-1];
        end
        u0_s1 <= u0_pipe[NORM_3_LATENCY-1];
        u1_s1 <= u1_pipe[NORM_3_LATENCY-1];
      end
    end
    //////////////////////////////////////////////////////////
    //// END STAGE 1
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    /////// STAGE 1.5
    //////////////////////////////////////////////////////////
    logic trig_valid;
    logic signed [FRAC_BITS-1:0] cos_phi, sin_phi;
    
    ggx_trig_lut #(
      .FRAC_BITS(FRAC_BITS),
      .ADDR_BITS(10)
    ) u_trig (
      .clk        (s00_axis_aclk),
      .rst_n      (s00_axis_aresetn),
      .in_valid   (s1_norm_valid),
      .u0_phase   (u0_s1),
      .out_valid  (trig_valid),
      .cos_phi    (cos_phi),
      .sin_phi    (sin_phi)
    );
    logic [FRAC_BITS-1:0] u1_d1, u1_d2;
    logic signed [FRAC_BITS-1:0] wx_d1, wx_d2, wy_d1, wy_d2, wz_d1, wz_d2;

    always_ff @(posedge s00_axis_aclk) begin
      if (s00_axis_aresetn == 0) begin
        u1_d1 <= '0;
        u1_d2 <= '0;
        wx_d1 <= '0; wy_d1 <= '0; wz_d1 <= '0;
        wx_d2 <= '0; wy_d2 <= '0; wz_d2 <= '0;
      end else begin
        u1_d1 <= s1_norm_valid ? u1_s1 : '0;
        wx_d1 <= s1_norm_valid ? wi_x : '0;
        wy_d1 <= s1_norm_valid ? wi_y : '0;
        wz_d1 <= s1_norm_valid ? wi_z : '0;

        u1_d2 <= u1_d1;
        wx_d2 <= wx_d1; wy_d2 <= wy_d1; wz_d2 <= wz_d1;
      end
    end
    //////////////////////////////////////////////////////////
    /////// END STAGE 1.5 
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    ///// STAGE 2A... trig + z / t = 1 -A^2
    //////////////////////////////////////////////////////////
    logic s2a_valid;
    logic signed [FRAC_BITS-1:0] cap_z_s2a;
    logic signed [FRAC_BITS-1:0] wi_x_s2a, wi_y_s2a, wi_z_s2a;
    logic signed [FRAC_BITS-1:0] cos_s2b, sin_s2b;

    always_ff @(posedge s00_axis_aclk) begin
      if (s00_axis_aresetn == 0) begin
        wi_x_s2a <= '0;
        wi_y_s2a <= '0;
        wi_z_s2a <= '0;
        cap_z_s2a <= '0;
        s2a_valid   <= '0;
        cos_s2b <= '0;
        sin_s2b <= '0;
      end else begin
        s2a_valid <= trig_valid;
        if (trig_valid) begin
          logic signed [FRAC_BITS-1:0] one_minus_u1; 
          logic signed [FRAC_BITS-1:0]   one_plus_wz;
          logic signed [2*FRAC_BITS-1:0] prod_intmd;

          cos_s2b <= cos_phi;
          sin_s2b <= sin_phi;

          one_minus_u1 = $signed(ONE) - $signed({1'b0, u1_d2[FRAC_BITS-1:1]});
          one_plus_wz = ONE + wz_d2;
          prod_intmd = one_minus_u1 * one_plus_wz;

          cap_z_s2a <= prod_intmd[2*FRAC_BITS-2 : FRAC_BITS-1] - wz_d2;

          cos_s2b <= cos_phi;
          sin_s2b <= sin_phi;

          // Pass through view vector
          wi_x_s2a <= wx_d2;
          wi_y_s2a <= wy_d2;
          wi_z_s2a <= wz_d2;
        end
      end
    end
    //////////////////////////////////////////////////////////
    ///// END STAGE 2A
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    ///// STAGE 2B
    //////////////////////////////////////////////////////////
    logic s2b_valid;
    logic signed [FRAC_BITS-1:0] t_s2b;
    logic signed [FRAC_BITS-1:0] cap_z_s2b;
    logic signed [FRAC_BITS-1:0] wi_x_s2b, wi_y_s2b, wi_z_s2b;

    always_ff @(posedge s00_axis_aclk) begin
      if (s00_axis_aresetn == 0) begin
        s2b_valid <= 1'b0;
        t_s2b     <= '0;
        cap_z_s2b <= '0;
        wi_x_s2b  <= '0;
        wi_y_s2b  <= '0;
        wi_z_s2b  <= '0;
      end else begin
        s2b_valid <= s2a_valid;
        if (s2a_valid) begin
          logic signed [FRAC_BITS-1:0] z_sq;
          z_sq = fixed_mul(cap_z_s2a, cap_z_s2a);

          if (ONE < z_sq) begin
            t_s2b <= 0;
          end else begin
            t_s2b <= ONE - z_sq;
          end

          cap_z_s2b <= cap_z_s2a;
          wi_x_s2b <= wi_x_s2a;
          wi_y_s2b <= wi_y_s2a;
          wi_z_s2b <= wi_z_s2a;
        end
      end
    end
    //////////////////////////////////////////////////////////
    ///// END STAGE 2B
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //// STAGE 3... inv_sqrt(t) and sin(theta) = t* inv_sqrt(t)
    //////////////////////////////////////////////////////////
    logic inv_valid;
    logic signed [FRAC_BITS-1:0] inv_t; // ~= 1/sqrt(t)
    fixed_inv_sqrt_newton #(
      .FRAC_BITS(FRAC_BITS),
      .ADDR_BITS(8)
    ) u_inv_sqrt_t (
      .clk (s00_axis_aclk),
      .rst_n (s00_axis_aresetn),
      .in_valid (s2b_valid),
      .in_x (t_s2b),
      .out_valid (inv_valid),
      .out_y (inv_t)
    );

    logic signed [FRAC_BITS-1:0] cos_d[0:INV_SQRT_LATENCY-1]; 
    logic signed [FRAC_BITS-1:0] sin_d[0:INV_SQRT_LATENCY-1];
    logic signed [FRAC_BITS-1:0] z_d[0:INV_SQRT_LATENCY-1];
    logic signed [FRAC_BITS-1:0] t_d[0:INV_SQRT_LATENCY-1];
    logic signed [FRAC_BITS-1:0] wx_d[0:INV_SQRT_LATENCY-1];
    logic signed [FRAC_BITS-1:0] wy_d[0:INV_SQRT_LATENCY-1]; 
    logic signed [FRAC_BITS-1:0] wz_d[0:INV_SQRT_LATENCY-1];

    always_ff @(posedge s00_axis_aclk) begin
      if (s00_axis_aresetn == 0) begin
        for (int i = 0; i < INV_SQRT_LATENCY; i++) begin
          cos_d[i] <= '0;
          sin_d[i] <= '0;
          z_d[i] <= '0;
          t_d[i] <= '0;
          wx_d[i] <= '0;
          wy_d[i] <= '0;
          wz_d[i] <= '0;
        end
      end else begin
        if (s2b_valid) begin
          cos_d[0] <= s2b_valid ? cos_s2b: '0;
          sin_d[0] <= s2b_valid ? sin_s2b: '0;
          z_d[0] <= s2b_valid ? cap_z_s2b: '0;
          t_d[0] <= s2b_valid ? t_s2b : '0;
          wx_d[0] <= s2b_valid ? wi_x_s2b: '0;
          wy_d[0] <= s2b_valid ? wi_y_s2b: '0;
          wz_d[0] <= s2b_valid ? wi_z_s2b: '0;
        end
        for (int i = 1; i < INV_SQRT_LATENCY; i++) begin
          cos_d[i] <= cos_d[i-1];
          sin_d[i] <= sin_d[i-1];
          z_d[i] <= z_d[i-1];
          t_d[i] <= t_d[i-1];
          wx_d[i] <= wx_d[i-1];
          wy_d[i] <= wy_d[i-1];
          wz_d[i] <= wz_d[i-1];
        end
      end
    end
    //////////////////////////////////////////////////////////
    //// END STAGE 3
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //// STAGE 4.. build c, h_std, h_ell, normalize h_ell -> h
    // c = (sin(theta)cos(phi), sin(theta)sin(phi), z)
    //////////////////////////////////////////////////////////
    logic s4_valid;
    logic signed [FRAC_BITS-1:0] sin_theta_scaled;
    logic signed [FRAC_BITS-1:0] c_x_0, c_y_0, c_z_0;
    logic signed [FRAC_BITS:0] sum_x, sum_y, sum_z;
    logic signed [FRAC_BITS-1:0] hstd_x, hstd_y, hstd_z;

    always_ff @(posedge s00_axis_aclk) begin
      if (s00_axis_aresetn==0) begin
        s4_valid <= 1'b0;
        hstd_x <= '0;
        hstd_y <= '0;
        hstd_z <= '0;
        sum
        
      end else begin
        s4_valid <= inv_valid;
        if (inv_valid) begin
          sin_theta_scaled  = fixed_mul(t_d[INV_SQRT_LATENCY-1], inv_t) <<< 2;

          c_x_0             = fixed_mul(sin_theta_scaled, cos_d[INV_SQRT_LATENCY-1]);
          c_y_0             = fixed_mul(sin_theta_scaled, sin_d[INV_SQRT_LATENCY-1]);
          c_z_0             = z_d[INV_SQRT_LATENCY-1];

          sum_x             = {c_x_0[FRAC_BITS-1], c_x_0} + {wx_d[INV_SQRT_LATENCY-1][FRAC_BITS-1], wx_d[INV_SQRT_LATENCY-1]};
          sum_y             = {c_y_0[FRAC_BITS-1], c_y_0} + {wy_d[INV_SQRT_LATENCY-1][FRAC_BITS-1], wy_d[INV_SQRT_LATENCY-1]};
          sum_z             = {c_z_0[FRAC_BITS-1], c_z_0} + {wz_d[INV_SQRT_LATENCY-1][FRAC_BITS-1], wz_d[INV_SQRT_LATENCY-1]};

          hstd_x <= sum_x >>> 2;
          hstd_y <= sum_y >>> 2;
          hstd_z <= sum_z >>> 2;
        end
      end
    end

    // h_ell = (hstd_x * alpha, hstd_y, hstd_z * alpha)
    logic s4b_valid;
    logic signed [FRAC_BITS-1:0] hell_x, hell_y, hell_z;
  
    always_ff @(posedge s00_axis_aclk) begin
      if (s00_axis_aresetn==0) begin
        s4b_valid <= 1'b0;
        hell_x <= '0;
        hell_y <= '0;
        hell_z <= '0;
      end else begin
        s4b_valid <= s4_valid;
        if (s4_valid) begin
          hell_x <= fixed_mul(hstd_x, alpha);
          hell_y <= hstd_y;
          hell_z <= fixed_mul(hstd_z, alpha);
        end
      end
    end

    // normalize h_ell with fixed_norm3
    logic h_valid;
    logic signed [FRAC_BITS-1:0] h_x, h_y, h_z;

    fixed_norm3 #(
      .FRAC_BITS(FRAC_BITS)
    ) u_norm_final (
      .clk       (s00_axis_aclk),
      .rst_n     (s00_axis_aresetn),
      .in_valid  (s4b_valid),
      .in_x      (hell_x),
      .in_y      (hell_y),
      .in_z      (hell_z),
      .out_valid (h_valid),
      .out_x     (h_x),
      .out_y     (h_y),
      .out_z     (h_z)
    );
    //////////////////////////////////////////////////////////
    //// END STAGE 4
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //// OUTPUT PACKING
    //////////////////////////////////////////////////////////
    logic [C_M00_AXIS_TDATA_WIDTH-1:0] m00_axis_tdata_reg;
    logic                              m00_axis_tvalid_reg;

    assign m00_axis_tdata = m00_axis_tdata_reg;
    assign m00_axis_tvalid = m00_axis_tvalid_reg;

    always_ff @(posedge s00_axis_aclk) begin
      if (s00_axis_aresetn == 0) begin
        m00_axis_tvalid_reg <= 1'b0;
        m00_axis_tdata_reg <= '0;
      end else begin
        m00_axis_tvalid_reg <= h_valid;
        if (h_valid) begin
          m00_axis_tdata_reg <= {h_z, h_y, h_x};
        end
      end
    end
    //// END
endmodule

`default_nettype wire
