// alu: 算术/逻辑运算 + 分支比较辅助输出（eq/lt/ltu）
// 移位量取 b[4:0]；SLT/SLTU 直接复用 lt/ltu
import pkg_cpu2026::*;

module alu (
    input  logic [4:0]  op,
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic [31:0] out,
    output logic        eq,    // a == b
    output logic        lt,    // 有符号 a < b
    output logic        ltu    // 无符号 a < b
);

    always_comb begin
        eq  = (a == b);
        lt  = ($signed(a) < $signed(b));
        ltu = (a < b);
        case (op)
        ALU_ADD:   out = a + b;
        ALU_SUB:   out = a - b;
        ALU_SLL:   out = a << b[4:0];
        ALU_SLT:   out = {{31{1'b0}}, lt};
        ALU_SLTU:  out = {{31{1'b0}}, ltu};
        ALU_XOR:   out = a ^ b;
        ALU_SRL:   out = a >> b[4:0];
        ALU_SRA:   out = $signed(a) >>> b[4:0];
        ALU_OR:    out = a | b;
        ALU_AND:   out = a & b;
        ALU_PASSA: out = a;
        ALU_PASSB: out = b;
        default:   out = 'x;
        endcase
    end

endmodule
