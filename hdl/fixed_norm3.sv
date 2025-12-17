`timescale 1ns / 1ps
`default_nettype none

module fixed_norm3 #(
    parameter int FRAC_BITS = 32
) (
    input  wire                        clk,
    input  wire                        rst_n,

    input  wire                        in_valid,
    input  wire signed [FRAC_BITS-1:0] in_x,
    input  wire signed [FRAC_BITS-1:0] in_y,
    input  wire signed [FRAC_BITS-1:0] in_z,

    output logic                       out_valid,
    output logic signed [FRAC_BITS-1:0] out_x,
    output logic signed [FRAC_BITS-1:0] out_y,
    output logic signed [FRAC_BITS-1:0] out_z
);

    // ---------------------------------------------------
    // Helper: signed Q1.(F-1) multiply
    // ---------------------------------------------------
    function automatic signed [FRAC_BITS-1:0]
    fixed_mul_q1(input signed [FRAC_BITS-1:0] a,
                 input signed [FRAC_BITS-1:0] b);
        logic signed [2*FRAC_BITS-1:0] prod;
        begin
            prod = a * b;
            fixed_mul_q1 = prod[2*FRAC_BITS-2 : FRAC_BITS-1];
        end
    endfunction

    // ---------------------------------------------------
    // Stage 0: latch inputs
    // ---------------------------------------------------
    logic                        v0, v1;
    logic signed [FRAC_BITS-1:0] x0, y0, z0;
    logic signed [FRAC_BITS-1:0] xx, yy, zz; // x^2, y^2, z^2
    logic signed [FRAC_BITS:0] len2_s1; // Q2.30

    // Pipeline for x,y,z to align with inv_sqrt output (2 stages of inv_sqrt)
    logic signed [FRAC_BITS-1:0] x_pipe[0:7];
    logic signed [FRAC_BITS-1:0] y_pipe[0:7];
    logic signed [FRAC_BITS-1:0] z_pipe[0:7];
    
    // ---------------------------------------------------
    // Inv-sqrt instance
    // ---------------------------------------------------
    logic                        inv_valid;
    logic signed [FRAC_BITS-1:0] inv_len;

    fixed_inv_sqrt_newton #(
        .FRAC_BITS(FRAC_BITS),
        .ADDR_BITS(8)
    ) u_inv_len (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (v1),
        .in_x     (len2_s1[FRAC_BITS-1:0]),
        .out_valid(inv_valid),
        .out_y    (inv_len)
    );

    always_ff @(posedge clk or negedge rst_n) begin
      if (rst_n==0) begin
        v0 <= 1'b0; v1 <= 1'b0;
        x0 <= '0;  y0 <= '0;  z0 <= '0;
        xx <= '0;  yy <= '0;  zz <= '0;
        len2_s1 <= '0;
        for (int i = 0; i < 8; i++) begin
          x_pipe[i] <= '0;
          y_pipe[i] <= '0;
          z_pipe[i] <= '0;
        end
        out_valid <= 1'b0;
        out_x     <= '0; out_y <= '0; out_z <= '0;
      end else begin
        // Stage 0: latch raw inputs
        v0 <= in_valid;
        if (in_valid) begin
          x0 <= in_x;
          y0 <= in_y;
          z0 <= in_z;
          x_pipe[0] <= in_x;
          y_pipe[0] <= in_y;
          z_pipe[0] <= in_z;
        end

        // Stage 1: compute len2 = x^2 + y^2 + z^2
        v1 <= v0;
        if (v0) begin
          xx = fixed_mul_q1(x0, x0);
          yy = fixed_mul_q1(y0, y0);
          zz = fixed_mul_q1(z0, z0);
          len2_s1 <= xx + yy + zz;
        end

        for (int i = 1; i < 8; i++) begin
          x_pipe[i] <= x_pipe[i-1];
          y_pipe[i] <= y_pipe[i-1];
          z_pipe[i] <= z_pipe[i-1];
        end

        // Output normalization when inv_len is valid
        out_valid <= inv_valid;
        if (inv_valid) begin
            out_x <= (fixed_mul_q1(x_pipe[7], inv_len))<<<2;
            out_y <= (fixed_mul_q1(y_pipe[7], inv_len))<<<2;
            out_z <= (fixed_mul_q1(z_pipe[7], inv_len))<<<2;
        end
      end
    end

endmodule

`default_nettype wire
