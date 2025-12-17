// sobol_dir_table.svh
// 2-D Sobol direction numbers as a function, for Icarus

function automatic logic [31:0] sobol_dir (input int dim, input int bit_idx);
    logic [31:0] val;
    begin
        val = 32'h0;

        case (dim)
            0: begin
                case (bit_idx)
                    0:  val = 32'h80000000;
                    1:  val = 32'h40000000;
                    2:  val = 32'h20000000;
                    3:  val = 32'h10000000;
                    4:  val = 32'h08000000;
                    5:  val = 32'h04000000;
                    6:  val = 32'h02000000;
                    7:  val = 32'h01000000;
                    8:  val = 32'h00800000;
                    9:  val = 32'h00400000;
                    10: val = 32'h00200000;
                    11: val = 32'h00100000;
                    12: val = 32'h00080000;
                    13: val = 32'h00040000;
                    14: val = 32'h00020000;
                    15: val = 32'h00010000;
                    16: val = 32'h00008000;
                    17: val = 32'h00004000;
                    18: val = 32'h00002000;
                    19: val = 32'h00001000;
                    20: val = 32'h00000800;
                    21: val = 32'h00000400;
                    22: val = 32'h00000200;
                    23: val = 32'h00000100;
                    24: val = 32'h00000080;
                    25: val = 32'h00000040;
                    26: val = 32'h00000020;
                    27: val = 32'h00000010;
                    28: val = 32'h00000008;
                    29: val = 32'h00000004;
                    30: val = 32'h00000002;
                    31: val = 32'h00000001;
                    default: val = 32'h0;
                endcase
            end

            1: begin
                case (bit_idx)
                    0:  val = 32'h80000000;
                    1:  val = 32'hc0000000;
                    2:  val = 32'hc0000000;
                    3:  val = 32'h90000000;
                    4:  val = 32'hb8000000;
                    5:  val = 32'he8000000;
                    6:  val = 32'he2000000;
                    7:  val = 32'ha3000000;
                    8:  val = 32'h8b000000;
                    9:  val = 32'hce400000;
                    10: val = 32'hcee00000;
                    11: val = 32'h9aa00000;
                    12: val = 32'hb0080000;
                    13: val = 32'he40c0000;
                    14: val = 32'hee0c0000;
                    15: val = 32'haa090000;
                    16: val = 32'h808b8000;
                    17: val = 32'hc0ce8000;
                    18: val = 32'hc0ce2000;
                    19: val = 32'h909a3000;
                    20: val = 32'hb8b0b000;
                    21: val = 32'he8e4e400;
                    22: val = 32'he2eeee00;
                    23: val = 32'ha3aaaa00;
                    24: val = 32'h8b800080;
                    25: val = 32'hce8000c0;
                    26: val = 32'hce2000c0;
                    27: val = 32'h9a300090;
                    28: val = 32'hb0b000b8;
                    29: val = 32'he4e400e8;
                    30: val = 32'heeee00e2;
                    31: val = 32'haaaa00a3;
                    default: val = 32'h0;
                endcase
            end

            default: val = 32'h0;
        endcase

        sobol_dir = val;
    end
endfunction
