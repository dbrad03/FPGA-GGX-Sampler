`timescale 1ns / 1ps
`default_nettype none

module ggx_trig_lut #(
    parameter int FRAC_BITS = 32,
    parameter int ADDR_BITS = 10
) (
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        in_valid,
    input  wire [FRAC_BITS-1:0]        u0_phase,
    
    output logic                       out_valid,
    output logic signed [FRAC_BITS-1:0] cos_phi,
    output logic signed [FRAC_BITS-1:0] sin_phi
);

    localparam int ROM_SIZE = 1 << ADDR_BITS;
    
    // 64-bit wide ROM: {Cos[63:32], Sin[31:0]}
    logic [63:0] trig_rom [0:ROM_SIZE-1];

    initial begin
        if ($test$plusargs("dump_roms")) 
            $display("Loading ggx_trig_rom.mem...");
        $readmemh("ggx_trig_rom.mem", trig_rom);
    end

    initial begin
        #1;
        $display("ROM[0]=%h ROM[256]=%h ROM[768]=%h ROM[1023]=%h",
            trig_rom[0], trig_rom[256], trig_rom[768], trig_rom[1023]);
    end

    // Use top bits of u0 to index ROM
    // u0 is [0..1). ROM covers [0..1).
    logic [ADDR_BITS-1:0] addr_reg;
    wire [ADDR_BITS-1:0] addr = u0_phase[FRAC_BITS-1 : FRAC_BITS-ADDR_BITS];

    // Pipeline Stages
    logic v1, v2;
    logic [63:0] rom_data;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            v1 <= 0; v2 <= 0;
            addr_reg <= '0;
            rom_data <= '0;
        end else begin
            // Stage 1: Address Register (Implicit in Block RAM usually, or explicit)
            v1 <= in_valid;
            if (in_valid) begin
                addr_reg <= addr;
            end
            
            // Stage 2: Data Register
            v2 <= v1;
            if (v1) begin
                rom_data <= trig_rom[addr_reg]; // Synchronous Read
            end
        end
    end
    always_comb begin
        cos_phi = rom_data[63:32]; // High 32 bits
        sin_phi = rom_data[31:0];  // Low 32 bits
        out_valid = v2;
    end

endmodule
`default_nettype wire