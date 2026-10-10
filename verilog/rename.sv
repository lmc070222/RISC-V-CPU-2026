// rename: 物理寄存器重命名（PRF + RAT + 空闲列表 + 分支 checkpoint 快照）
// 方案 A（已确认）：
//   - checkpoint 槽位 = 分支自己的 ROB 号，分支派发当拍写入"重命名后"的 RAT，
//     误预测单拍整份恢复，无需分配/释放管理
//   - 旧物理寄存器在提交时回收；flush 被冲掉的物理寄存器由 ROB 侧逐拍归还（flush_free_*）
//   - x0 恒映射物理 0，从不重命名、从不写回
//   - 派发宽度 = 1；PRF 读带 CDB 旁路（当拍写回可读）
// 时序约定：disp_* 在 cycle N 有效 → 边沿 N+1 处理（重命名/ROB/空闲列表/快照同拍），
//           alloc_* 与 ps*/s* 输出为寄存器输出，于边沿 N+1 锁存、cycle N+1 可见
module rename import pkg_cpu2026::*; #(
    parameter int ARCH_NUM   = 32,
    parameter int PRF_NUM    = 48,
    parameter int PRF_W      = $clog2(PRF_NUM),
    parameter int ROB_DEPTH  = 16,
    parameter int ROB_ID_W   = 4,
    parameter int WB_PORTS   = 3
) (
    input  logic clk,
    input  logic reset,

    // ---- 派发输入（cycle N 有效，边沿 N+1 处理）----
    input  logic                disp_valid,
    input  logic                ext_ready,       // ROB / 目标发射队列均有空位
    input  logic [ROB_ID_W-1:0] next_rob_id,     // ROB 尾部指针
    input  dec_info_t           disp_dec,
    output logic                disp_ready,      // 组合输出（反压）

    // ---- 分配结果（寄存器输出；alloc_pdst_c 为组合版，供 ROB 同沿分配）----
    output logic                alloc_valid,
    output logic [ROB_ID_W-1:0] alloc_rob_id,
    output logic [PRF_W-1:0]    alloc_pdst,
    output logic [PRF_W-1:0]    alloc_pdst_c,
    output logic [PRF_W-1:0]    alloc_old_pdst_c,   // rd 的旧映射（组合，ROB 存盘回收用）

    // ---- 重命名后的源（寄存器输出）----
    output logic [PRF_W-1:0]    ps1, ps2,
    output logic                s1_ready, s2_ready,
    output logic [31:0]         s1_data, s2_data,

    // ---- 提交回收（延迟释放：无未解析分支时才归还）----
    input  logic                commit_free_valid,
    input  logic [PRF_W-1:0]    commit_free_pdst,
    output logic                commit_free_taken,   // 本拍实际接收（未被 RAT 检查跳过）

    // ---- flush：单拍恢复 RAT + 逐拍归还物理寄存器 ----
    input  logic                flush_valid,
    input  logic [ROB_ID_W-1:0] flush_rob_id,
    input  logic                flush_free_valid,
    input  logic [PRF_W-1:0]    flush_free_pdst,

    // ---- CDB 写回（3 端口）----
    input  logic [WB_PORTS-1:0]             cdb_valid,
    input  logic [WB_PORTS-1:0][PRF_W-1:0] cdb_pdst,
    input  logic [WB_PORTS-1:0][31:0]      cdb_data
);

    logic has_rd;
    assign has_rd = disp_dec.rd_write && (disp_dec.rd != 5'd0);

    logic alloc_valid_c;
    assign alloc_valid_c = disp_valid && disp_ready;

    // ================= PRF 与就绪位 =================
    logic [31:0] prf [0:PRF_NUM-1];
    logic [PRF_NUM-1:0] prf_ready;

    always_ff @(posedge clk) begin
        for (int j = 0; j < WB_PORTS; j++) begin
            if (cdb_valid[j] && (cdb_pdst[j] != '0)) begin
                prf[cdb_pdst[j]] <= cdb_data[j];
            end
        end
    end

    always_ff @(posedge clk) begin
        if (reset) begin
            prf_ready <= '1;    // 初始全部就绪（物理 0..31 为架构初值）
        end else begin
            for (int j = 0; j < WB_PORTS; j++)
                if (cdb_valid[j] && (cdb_pdst[j] != '0)) prf_ready[cdb_pdst[j]] <= 1'b1;
            if (alloc_valid_c && has_rd)          prf_ready[fl_stack[fl_top - 1]] <= 1'b0;
            if (push_commit)                 prf_ready[commit_free_pdst]      <= 1'b1;
            if (flush_free_valid)                 prf_ready[flush_free_pdst]      <= 1'b1;
        end
    end

    // ================= RAT =================
    logic [PRF_W-1:0] rat [0:ARCH_NUM-1];

    always_ff @(posedge clk) begin
        if (reset) begin
            for (int i = 0; i < ARCH_NUM; i++) rat[i] <= i;
        end else if (flush_valid) begin
            for (int i = 0; i < ARCH_NUM; i++) rat[i] <= cp[flush_rob_id][i];
        end else if (alloc_valid_c && has_rd) begin
            rat[disp_dec.rd] <= fl_stack[fl_top - 1];
        end
    end

    // ================= checkpoint 快照（槽位 = ROB 号）=================
    logic [PRF_W-1:0] cp [0:ROB_DEPTH-1][0:ARCH_NUM-1];

    always_ff @(posedge clk) begin
        if (alloc_valid_c &&
            (disp_dec.is_branch || disp_dec.is_jal || disp_dec.is_jalr)) begin
            for (int i = 0; i < ARCH_NUM; i++)
                cp[next_rob_id][i] <= (has_rd && (i == disp_dec.rd)) ? fl_stack[fl_top - 1]
                                                                    : rat[i];
        end
    end

    // ================= 空闲列表（LIFO，双 push + 单 pop）=================
    logic [$clog2(PRF_NUM+1)-1:0] fl_top;
    logic [PRF_W-1:0] fl_stack [0:PRF_NUM-1];
    logic push_commit, push_flush, pop_alloc;

    // checkpoint 恢复可能复活旧映射：归还前检查 pdst 是否仍被 RAT 引用（若是则跳过）
    logic rat_contains_free;
    logic rat_contains_ffree;
    always_comb begin
        rat_contains_free = 1'b0;
        rat_contains_ffree = 1'b0;
        for (int i = 1; i < ARCH_NUM; i++) begin
            if (rat[i] == commit_free_pdst) rat_contains_free = 1'b1;
            if (rat[i] == flush_free_pdst) rat_contains_ffree = 1'b1;
        end
    end

    assign push_commit = commit_free_valid && !rat_contains_free;
    assign push_flush  = flush_free_valid && !rat_contains_ffree;
    assign commit_free_taken = push_commit;
    assign pop_alloc   = alloc_valid_c && has_rd;

    always_ff @(posedge clk) begin
        if (reset) begin
            for (int i = 0; i < PRF_NUM - ARCH_NUM; i++)
                fl_stack[i] <= PRF_W'((PRF_NUM - ARCH_NUM - 1) - i + ARCH_NUM);
            fl_top <= PRF_NUM - ARCH_NUM;
        end else begin
            // 同拍 push 复用被 pop 腾出的槽（fl_top - pop），避免旧值残留
            if (push_commit)
                fl_stack[fl_top - ($clog2(PRF_NUM+1))'(pop_alloc)] <= commit_free_pdst;
            if (push_flush)
                fl_stack[fl_top - ($clog2(PRF_NUM+1))'(pop_alloc)
                         + ($clog2(PRF_NUM+1))'(push_commit)] <= flush_free_pdst;
            fl_top <= fl_top
                    + ($clog2(PRF_NUM+1))'(push_commit)
                    + ($clog2(PRF_NUM+1))'(push_flush)
                    - ($clog2(PRF_NUM+1))'(pop_alloc);
        end
    end

    // ================= 派发与源重命名 =================
    assign disp_ready = ext_ready && (!has_rd || (fl_top != '0));

    // 组合版分配物理寄存器（cycle N 有效，ROB/SB 于同沿 N+1 分配使用）
    assign alloc_pdst_c = fl_stack[fl_top - 1];
    // rd 的旧映射（cycle N 组合，ROB 在提交时回收）
    assign alloc_old_pdst_c = rat[disp_dec.rd];

    // 组合源重命名（cycle N 有效）
    logic [PRF_W-1:0] ps1_c, ps2_c;
    logic             s1_ready_c, s2_ready_c;
    logic [31:0]      s1_data_c, s2_data_c;

    always_comb begin
        if (disp_dec.rs1 == 5'd0) begin
            ps1_c = '0; s1_ready_c = 1'b1; s1_data_c = 32'd0;
        end else begin
            ps1_c = rat[disp_dec.rs1];
            s1_ready_c = prf_ready[ps1_c];
            s1_data_c = prf[ps1_c];
            for (int j = 0; j < WB_PORTS; j++) begin
                if (cdb_valid[j] && (cdb_pdst[j] == ps1_c)) begin
                    s1_data_c  = cdb_data[j];
                    s1_ready_c = 1'b1;   // 当拍 CDB 视为就绪（数据经旁路进入 IQ）
                end
            end
        end
        if (disp_dec.rs2 == 5'd0) begin
            ps2_c = '0; s2_ready_c = 1'b1; s2_data_c = 32'd0;
        end else begin
            ps2_c = rat[disp_dec.rs2];
            s2_ready_c = prf_ready[ps2_c];
            s2_data_c = prf[ps2_c];
            for (int j = 0; j < WB_PORTS; j++) begin
                if (cdb_valid[j] && (cdb_pdst[j] == ps2_c)) begin
                    s2_data_c  = cdb_data[j];
                    s2_ready_c = 1'b1;
                end
            end
        end
    end

    // ================= 派发信息锁存（边沿 N+1，与内部处理同拍）=================
`ifndef SYNTHESIS
`endif
    always_ff @(posedge clk) begin
        if (reset) begin
            alloc_valid  <= 1'b0;
            alloc_rob_id <= '0;
            alloc_pdst   <= '0;
            ps1 <= '0; ps2 <= '0;
            s1_ready <= 1'b1; s2_ready <= 1'b1;
            s1_data <= 32'd0; s2_data <= 32'd0;
        end else begin
            alloc_valid  <= alloc_valid_c;
            alloc_rob_id <= next_rob_id;
            alloc_pdst   <= fl_stack[fl_top - 1];
            ps1 <= ps1_c; ps2 <= ps2_c;
            s1_ready <= s1_ready_c; s2_ready <= s2_ready_c;
            s1_data <= s1_data_c; s2_data <= s2_data_c;
        end
    end

endmodule
