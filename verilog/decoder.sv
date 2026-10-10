// decoder: RV32IM 全量译码（纯组合）
// 覆盖：RV32I 37 条 + M 扩展 8 条 + Load/Store 8 条；CSR/FENCE/ECALL/EBREAK/未知编码 → illegal=1
// 编码表与立即数表见 notes/b-20261006-执行单元规格.md §1/§2
module decoder import pkg_cpu2026::*; (
    input  logic [31:0] instr,
    output dec_info_t   dec
);

    logic [6:0]  opcode;
    logic [2:0]  funct3;
    logic [6:0]  funct7;
    logic [31:0] imm_i, imm_s, imm_b, imm_u, imm_j;

    assign opcode = instr[6:0];
    assign funct3 = instr[14:12];
    assign funct7 = instr[31:25];

    // 五种立即数格式（B/J 型 imm[0] 恒 0，重点自查位）
    assign imm_i = {{21{instr[31]}}, instr[30:20]};
    assign imm_s = {{20{instr[31]}}, instr[31:25], instr[11:7]};
    assign imm_b = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
    assign imm_u = {instr[31:12], 12'b0};
    assign imm_j = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

    always_comb begin
        dec = '0;
        dec.rs1    = instr[19:15];
        dec.rs2    = instr[24:20];
        dec.rd     = instr[11:7];
        dec.funct3 = funct3;
        dec.funct7 = funct7;

        case (opcode)
        7'b0110111: begin                              // LUI
            dec.is_lui   = 1'b1;
            dec.imm      = imm_u;
            dec.alu_op   = ALU_PASSA;
            dec.rd_write = 1'b1;
        end
        7'b0010111: begin                              // AUIPC
            dec.is_auipc = 1'b1;
            dec.imm      = imm_u;
            dec.alu_op   = ALU_ADD;
            dec.rd_write = 1'b1;
        end
        7'b1101111: begin                              // JAL
            dec.is_jal   = 1'b1;
            dec.imm      = imm_j;
            dec.rd_write = 1'b1;
        end
        7'b1100111: begin                              // JALR
            if (funct3 == 3'b000) begin
                dec.is_jalr  = 1'b1;
                dec.imm      = imm_i;
                dec.rd_write = 1'b1;
            end else begin
                dec.illegal = 1'b1;
            end
        end
        7'b1100011: begin                              // BRANCH
            dec.is_branch = 1'b1;
            dec.uses_rs2  = 1'b1;
            dec.imm       = imm_b;
            case (funct3)
            3'b000: dec.br_type = BR_BEQ;
            3'b001: dec.br_type = BR_BNE;
            3'b100: dec.br_type = BR_BLT;
            3'b101: dec.br_type = BR_BGE;
            3'b110: dec.br_type = BR_BLTU;
            3'b111: dec.br_type = BR_BGEU;
            default: dec.illegal = 1'b1;
            endcase
        end
        7'b0000011: begin                              // LOAD
            dec.is_load  = 1'b1;
            dec.imm      = imm_i;
            dec.rd_write = 1'b1;
            case (funct3)
            3'b000: dec.ld_type = LD_LB;
            3'b001: dec.ld_type = LD_LH;
            3'b010: dec.ld_type = LD_LW;
            3'b100: dec.ld_type = LD_LBU;
            3'b101: dec.ld_type = LD_LHU;
            default: dec.illegal = 1'b1;
            endcase
        end
        7'b0100011: begin                              // STORE
            dec.is_store = 1'b1;
            dec.uses_rs2 = 1'b1;
            dec.imm      = imm_s;
            case (funct3)
            3'b000: dec.st_type = ST_SB;
            3'b001: dec.st_type = ST_SH;
            3'b010: dec.st_type = ST_SW;
            default: dec.illegal = 1'b1;
            endcase
        end
        7'b0010011: begin                              // OP-IMM
            dec.is_alu   = 1'b1;
            dec.imm      = imm_i;
            dec.rd_write = 1'b1;
            case (funct3)
            3'b000: dec.alu_op = ALU_ADD;
            3'b001: dec.alu_op = ALU_SLL;
            3'b010: dec.alu_op = ALU_SLT;
            3'b011: dec.alu_op = ALU_SLTU;
            3'b100: dec.alu_op = ALU_XOR;
            3'b101: dec.alu_op = (funct7 == 7'b0100000) ? ALU_SRA : ALU_SRL;
            3'b110: dec.alu_op = ALU_OR;
            3'b111: dec.alu_op = ALU_AND;
            default: dec.illegal = 1'b1;
            endcase
        end
        7'b0110011: begin                              // OP / M 扩展
            dec.uses_rs2 = 1'b1;
            dec.rd_write = 1'b1;
            if (funct7 == 7'b0000001) begin            // M 扩展
                case (funct3)
                3'b000: begin dec.is_mul = 1'b1; dec.mul_type = MUL_MUL;    end
                3'b001: begin dec.is_mul = 1'b1; dec.mul_type = MUL_MULH;   end
                3'b010: begin dec.is_mul = 1'b1; dec.mul_type = MUL_MULHSU; end
                3'b011: begin dec.is_mul = 1'b1; dec.mul_type = MUL_MULHU;  end
                3'b100: begin dec.is_div = 1'b1; dec.div_type = DIV_DIV;    end
                3'b101: begin dec.is_div = 1'b1; dec.div_type = DIV_DIVU;   end
                3'b110: begin dec.is_div = 1'b1; dec.div_type = DIV_REM;    end
                3'b111: begin dec.is_div = 1'b1; dec.div_type = DIV_REMU;   end
                default: dec.illegal = 1'b1;
                endcase
            end else begin
                dec.is_alu = 1'b1;
                case (funct3)
                3'b000: dec.alu_op = (funct7 == 7'b0100000) ? ALU_SUB : ALU_ADD;
                3'b001: dec.alu_op = ALU_SLL;
                3'b010: dec.alu_op = ALU_SLT;
                3'b011: dec.alu_op = ALU_SLTU;
                3'b100: dec.alu_op = ALU_XOR;
                3'b101: dec.alu_op = (funct7 == 7'b0100000) ? ALU_SRA : ALU_SRL;
                3'b110: dec.alu_op = ALU_OR;
                3'b111: dec.alu_op = ALU_AND;
                default: dec.illegal = 1'b1;
                endcase
            end
        end
        // 7'b0001111 FENCE、7'b1110011 SYSTEM(ECALL/EBREAK/CSR)、其余未知
        default: dec.illegal = 1'b1;
        endcase
    end

endmodule
