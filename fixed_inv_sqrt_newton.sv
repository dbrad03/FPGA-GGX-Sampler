`timescale 1ns / 1ps
`default_nettype none

module fixed_inv_sqrt_newton #(
  parameter int FRAC_BITS = 32,
  parameter int ADDR_BITS = 8
) (
  input wire                          clk,
  input wire                          rst_n,
  
  input wire                          in_valid,
  input wire signed [FRAC_BITS-1:0]   in_x,

  output logic                        out_valid,
  output logic signed [FRAC_BITS-1:0] out_y
);
  localparam int ROM_SIZE = 1 << ADDR_BITS;

  // Constants
  // 1.5 in Q1.31 is 1.5 * 2^31 = 3 << 30 = 0xC0000000 (Wait, 1.5 is > 1.0)
  // PROBLEM: Q1.31 range is [-1.0, 1.0). 1.5 cannot be represented!
  // FIX: We must treat the intermediate 'factor' calc carefully.
  // 1.5 = 0011.000... (Q2.30). 
  // Let's stick to the math: factor = 1.5 - 0.5 * x * y^2.
  // If we assume our inputs are scaled (0.5x), we need to be very careful.
  // Let's implement the standard pipeline and handle 1.5 as a special wide add.
  
  // To perform 1.5 - t, we can use a wider wire.
  localparam signed [FRAC_BITS:0] THREE_HALVES = $signed({1'b0,32'd3} <<< (FRAC_BITS-2)); // 1.5 extended
  
  function automatic signed [FRAC_BITS-1:0] fixed_mul_q1(
    input signed [FRAC_BITS-1:0] a,
    input signed [FRAC_BITS-1:0] b
  );
    logic signed [2*FRAC_BITS-1:0] prod;
    begin
        prod = a * b;
        // Keep bits [62:31] for Q1.31
        fixed_mul_q1 = prod[2*FRAC_BITS-2 : FRAC_BITS-1];
    end
  endfunction

  // --- SIGNALS & PIPELINE REGS ---
  // We define them at the top to satisfy Icarus/Verilog parsers
  
  // ROM
  wire [FRAC_BITS-2:0] x_mag = in_x[FRAC_BITS-2:0];
  wire [ADDR_BITS-1:0] lut_idx = x_mag[FRAC_BITS-2 : FRAC_BITS-1-ADDR_BITS];
  logic signed [FRAC_BITS-1:0] inv_rom [0:ROM_SIZE-1];
  
  initial begin
    if ($test$plusargs("dump_roms")) 
       $display("Loading inv_sqrt_rom.mem...");
    $readmemh("inv_sqrt_rom.mem", inv_rom);
  end  

  // Pipeline Valid signals
  logic [4:0] v_pipe; 

  // Stage 1 signals
  logic signed [FRAC_BITS-1:0] s1_x;
  logic signed [FRAC_BITS-1:0] s1_y0;

  // Stage 2 signals
  logic signed [FRAC_BITS-1:0] s2_x;
  logic signed [FRAC_BITS-1:0] s2_y0;
  logic signed [FRAC_BITS-1:0] s2_y0_sq;

  // Stage 3 signals
  logic signed [FRAC_BITS-1:0] s3_y0;
  logic signed [FRAC_BITS-1:0] s3_t;

  logic signed [FRAC_BITS:0] wide_1_5;
  logic signed [FRAC_BITS:0] wide_8_t;
  logic signed [FRAC_BITS:0] wide_factor;

  assign wide_1_5 = THREE_HALVES;

  // Stage 4 signals
  logic signed [FRAC_BITS-1:0] s4_y0;
  logic signed [FRAC_BITS:0] s4_factor;
  
  logic signed [2*FRAC_BITS:0] s5_prod;
  logic signed [2*FRAC_BITS:0] final_prod;
  // --- LOGIC ---

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      v_pipe <= '0;
      s1_x <= '0; s1_y0 <= '0;
      s2_x <= '0; s2_y0 <= '0; s2_y0_sq <= '0;
      s3_y0 <= '0; s3_t <= '0;
      s4_y0 <= '0; s4_factor <= '0;
      wide_8_t <= '0; wide_factor <= '0;
      final_prod <= '0;  
      out_valid <= 0;
      out_y <= 0;
      // Clear data path optionally, or leave x logic undefined
    end else begin
      
      // --- STAGE 0 -> 1: ROM LOOKUP ---
      v_pipe[0] <= in_valid;
      if (in_valid) begin
         s1_x  <= in_x;
         s1_y0 <= inv_rom[lut_idx]; // Latency 1 (Clocked RAM behavior)
      end

      // --- STAGE 1 -> 2: Y0^2 ---
      v_pipe[1] <= v_pipe[0];
      if (v_pipe[0]) begin
         s2_x     <= s1_x;
         s2_y0    <= s1_y0;
         s2_y0_sq <= fixed_mul_q1(s1_y0, s1_y0); // 1 Mult
      end

      // --- STAGE 2 -> 3: T = X * Y0^2 ---
      v_pipe[2] <= v_pipe[1];
      if (v_pipe[1]) begin
         s3_y0 <= s2_y0;
         s3_t  <= fixed_mul_q1(s2_x, s2_y0_sq); // 1 Mult
      end

      // --- STAGE 3 -> 4: FACTOR = 1.5 - 8*T ---
      v_pipe[3] <= v_pipe[2];
      if (v_pipe[2]) begin
         s4_y0 <= s3_y0;
             
         wide_8_t = {s3_t[FRAC_BITS-1], s3_t} <<<3 ; // Sign extend and shift
         wide_factor = wide_1_5 - wide_8_t;

         // Clamp or truncate back to Q1.31
         // If math is right, factor should be around 1.0
         s4_factor <= wide_factor; 
      end

      // --- STAGE 4 -> 5: Y1 = Y0 * FACTOR ---
      v_pipe[4] <= v_pipe[3];
      if (v_pipe[3]) begin
        s5_prod <= s4_y0 * s4_factor;
      end

      out_valid <= v_pipe[4];
      if (v_pipe[4]) begin
        if (|s5_prod[2*FRAC_BITS : 2*FRAC_BITS-2]) out_y <= '0;
        else out_y <= s5_prod[2*FRAC_BITS-2+1:FRAC_BITS-1];
      end    
    end
  end

endmodule
`default_nettype wire