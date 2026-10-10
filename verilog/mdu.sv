// mdu: 乘除单元
//   MUL 系列：单周期组合（64 位积，MUL 取低 32 位，MULH/MULHSU/MULHU 取高 32 位）
//   DIV 系列：移位-减恢复法迭代，32 拍；除零/溢出语义按 notes/b-20261006 §4.2
//   握手：start 脉冲启动（busy 时忽略），valid 单拍脉冲携带结果
module mdu import pkg_cpu2026::*; #(
    parameter int   DIV_MAX_CYCLES = 34,
    parameter logic MUL_PIPELINE    = 0
) (
    input  logic        clk,
    input  logic        reset,
    input  logic        start,
    input  logic        is_mul,    // 1=乘法 0=除法
    input  logic [1:0]  mul_type,
    input  logic [1:0]  div_type,
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic        busy,
    output logic        valid,
    output logic [31:0] result
);

    logic        running;
    logic        is_mul_l;
    logic [1:0]  mul_type_l, div_type_l;
    logic [31:0] a_l, b_l;
    logic [31:0] mag_a, mag_b;      // 无符号绝对值
    logic        neg_a, neg_b;
    logic [5:0]  iter;
    logic [32:0] rem;               // 33 位余数
    logic [31:0] quot;

    // ---- 乘法（组合 64 位积，两补码位型与符号解释无关）----
    logic [63:0] mul_wide;
    logic [31:0] mul_res;
    always_comb begin
        unique case (mul_type_l)
        MUL_MUL, MUL_MULH:
            mul_wide = {{32{a_l[31]}}, a_l} * {{32{b_l[31]}}, b_l};
        MUL_MULHSU:
            mul_wide = {{32{a_l[31]}}, a_l} * {32'd0, b_l};
        MUL_MULHU:
            mul_wide = {32'd0, a_l} * {32'd0, b_l};
        default:
            mul_wide = '0;
        endcase
    end
    assign mul_res = (mul_type_l == MUL_MUL) ? mul_wide[31:0] : mul_wide[63:32];

    // ---- 除法迭代（恢复法：r/q 组成 65 位移位寄存器）----
    logic [32:0] rem_shifted, rem_sub;
    logic        sub;
    assign rem_shifted = {rem[31:0], quot[31]};
    assign rem_sub     = rem_shifted - {1'b0, mag_b};
    assign sub         = rem_shifted >= {1'b0, mag_b};

    // 符号调整（商符号 = 异或，余数符号随被除数；INT_MIN 溢出自洽：
    // -0x80000000 == 0x80000000，故 0x80000000/-1 自然得到正确商）
    logic [31:0] q_final, r_final;
    assign q_final = (neg_a ^ neg_b) ? (~quot + 32'd1) : quot;
    assign r_final = neg_a ? (~rem[31:0] + 32'd1) : rem[31:0];

    logic [31:0] div_res;
    always_comb begin
        unique case (div_type_l)
        DIV_DIV:  div_res = q_final;
        DIV_DIVU: div_res = quot;
        DIV_REM:  div_res = r_final;
        DIV_REMU: div_res = rem[31:0];
        default:  div_res = 'x;
        endcase
    end

    assign busy = running;

    always_ff @(posedge clk) begin
        if (reset) begin
            running <= 1'b0;
            valid   <= 1'b0;
            result  <= 32'd0;
        end else begin
            valid <= 1'b0;
            if (start && !running) begin
                running    <= 1'b1;
                is_mul_l   <= is_mul;
                mul_type_l <= mul_type;
                div_type_l <= div_type;
                a_l <= a;
                b_l <= b;
                neg_a <= a[31];
                neg_b <= b[31];
                mag_a <= a[31] ? (~a + 32'd1) : a;
                mag_b <= b[31] ? (~b + 32'd1) : b;
                quot <= a[31] ? (~a + 32'd1) : a;
                rem  <= 33'd0;
                iter <= 6'd0;
            end else if (running) begin
                if (is_mul_l) begin
                    valid   <= 1'b1;
                    result  <= mul_res;
                    running <= 1'b0;
                end else if (mag_b == 32'd0) begin
                    valid <= 1'b1;
                    if (div_type_l == DIV_REM || div_type_l == DIV_REMU)
                        result <= a_l;               // REM x/0 = x
                    else
                        result <= 32'hffffffff;      // DIV x/0 = 0xFFFFFFFF
                    running <= 1'b0;
                end else if (iter == 6'd32) begin
                    valid   <= 1'b1;
                    result  <= div_res;
                    running <= 1'b0;
                end else begin
                    rem  <= sub ? rem_sub : rem_shifted;
                    quot <= {quot[30:0], sub};
                    iter <= iter + 6'd1;
                end
            end
        end
    end

endmodule
