// pkg_cpu2026: 全局编码常量与译码结构（见 docs/iface.md v1.0）
package pkg_cpu2026;

    parameter logic [4:0] ALU_ADD = 5'd0,  ALU_SUB = 5'd1,  ALU_SLL = 5'd2, ALU_SLT = 5'd3,
                          ALU_SLTU = 5'd4, ALU_XOR = 5'd5,  ALU_SRL = 5'd6, ALU_SRA = 5'd7,
                          ALU_OR = 5'd8,   ALU_AND = 5'd9,  ALU_PASSA = 5'd10, ALU_PASSB = 5'd11;

    parameter logic [2:0] LD_LB = 3'd0, LD_LBU = 3'd1, LD_LH = 3'd2,
                          LD_LHU = 3'd3, LD_LW = 3'd4;

    parameter logic [1:0] ST_SB = 2'd0, ST_SH = 2'd1, ST_SW = 2'd2;

    parameter logic [1:0] MUL_MUL = 2'd0, MUL_MULH = 2'd1,
                          MUL_MULHSU = 2'd2, MUL_MULHU = 2'd3;

    parameter logic [1:0] DIV_DIV = 2'd0, DIV_DIVU = 2'd1,
                          DIV_REM = 2'd2, DIV_REMU = 2'd3;

    parameter logic [2:0] BR_BEQ = 3'd0, BR_BNE = 3'd1, BR_BLT = 3'd2,
                          BR_BGE = 3'd3, BR_BLTU = 3'd4, BR_BGEU = 3'd5;

    parameter int PRF_W    = 6;   // 阶段 4 启用：物理寄存器 64 项
    parameter int ROB_ID_W = 6;   // 阶段 4 启用：ROB 64 项

    typedef struct packed {
        logic        illegal;
        // 分类
        logic        is_alu;
        logic        is_lui;
        logic        is_auipc;
        logic        is_load;
        logic        is_store;
        logic        is_branch;
        logic        is_jal;
        logic        is_jalr;
        logic        is_mul;
        logic        is_div;
        // 操作数
        logic [4:0]  rs1;
        logic [4:0]  rs2;
        logic [4:0]  rd;
        logic [31:0] imm;
        logic [2:0]  funct3;
        logic [6:0]  funct7;
        // 控制
        logic [4:0]  alu_op;
        logic [2:0]  ld_type;
        logic [1:0]  st_type;
        logic [1:0]  mul_type;
        logic [1:0]  div_type;
        logic [2:0]  br_type;
        logic        uses_rs2;
        logic        rd_write;
    } dec_info_t;

endpackage
