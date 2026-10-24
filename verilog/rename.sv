// rename: 物理寄存器重命名（PRF + RAT + 空闲列表 + 分支 checkpoint 快照）
// 2 宽版本：每拍最多重命名 2 条指令（同拍指令间做寄存器转发）
//   - checkpoint 槽位 = 分支自己的 ROB 号，分支派发当拍写入"重命名后"的 RAT
//   - 旧物理寄存器在提交时回收（双通道）；flush 被冲掉的由 ROB 侧逐拍归还
//   - x0 恒映射物理 0
//   - PRF 读带 CDB 旁路（当拍写回可读）；pair 内 inst1 的源若命中 inst0 的 rd，则
//     映射到 inst0 的新 pdst 且置未就绪（等待 CDB 唤醒）
import pkg_cpu2026::*;

module rename #(
    parameter int ARCH_NUM   = 32,
    parameter int PRF_NUM    = 48,
    parameter int PRF_W      = $clog2(PRF_NUM),
    parameter int ROB_DEPTH  = 16,
    parameter int ROB_ID_W   = 4,
    parameter int WB_PORTS   = 3
) (
    input  logic clk,
    input  logic reset,

    // ---- 派发输入（cycle N 有效，边沿 N+1 处理）；disp_valid[1] 表示成对派发 ----
    input  logic [1:0]          disp_valid,
    input  pkg_cpu2026::dec_info_t     disp_dec0,
    input  pkg_cpu2026::dec_info_t     disp_dec1,
    input  logic                ext_ready,
    input  logic [ROB_ID_W-1:0] next_rob_id,
    output logic                disp_ready,

    // ---- 分配结果（寄存器输出；*_c 为组合版，供 ROB 同沿分配）----
    output logic [1:0]          alloc_valid,
    output logic [ROB_ID_W-1:0] alloc_rob_id0,
    output logic [ROB_ID_W-1:0] alloc_rob_id1,
    output logic [PRF_W-1:0]    alloc_pdst0,
    output logic [PRF_W-1:0]    alloc_pdst1,
    output logic [PRF_W-1:0]    alloc_pdst0_c,
    output logic [PRF_W-1:0]    alloc_pdst1_c,
    output logic [PRF_W-1:0]    alloc_old_pdst0_c,
    output logic [PRF_W-1:0]    alloc_old_pdst1_c,

    // ---- 重命名后的源（寄存器输出，两路）----
    output logic [PRF_W-1:0]    ps1_0, ps2_0, ps1_1, ps2_1,
    output logic                s1_ready_0, s2_ready_0, s1_ready_1, s2_ready_1,
    output logic [31:0]         s1_data_0, s2_data_0, s1_data_1, s2_data_1,

    // ---- 提交回收（双通道，延迟释放）----
    input  logic [1:0]              commit_free_valid,
    input  logic [1:0][PRF_W-1:0]   commit_free_pdst,
    output logic [1:0]              commit_free_taken,

    // ---- flush ----
    input  logic                flush_valid,
    input  logic [ROB_ID_W-1:0] flush_rob_id,
    input  logic                flush_free_valid,
    input  logic [PRF_W-1:0]    flush_free_pdst,

    // ---- CDB 写回 ----
    input  logic [WB_PORTS-1:0]             cdb_valid,
    input  logic [WB_PORTS-1:0][PRF_W-1:0] cdb_pdst,
    input  logic [WB_PORTS-1:0][31:0]      cdb_data
);

    // ================= 基础判定 =================
    logic has_rd0, has_rd1, alloc_fire;
    assign has_rd0 = disp_valid[0] && disp_dec0.rd_write && (disp_dec0.rd != 5'd0);
    assign has_rd1 = disp_valid[1] && disp_dec1.rd_write && (disp_dec1.rd != 5'd0);
    assign alloc_fire = disp_valid[0] && disp_ready;

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

    // ================= 空闲列表（LIFO，3 push + 2 pop）=================
    logic [$clog2(PRF_NUM+1)-1:0] fl_top;
    logic [PRF_W-1:0] fl_stack [0:PRF_NUM-1];
    logic push_commit0, push_commit1, push_flush;

    logic rat_contains_f0, rat_contains_f1, rat_contains_ffree;
    always_comb begin
        rat_contains_f0 = 1'b0;
        rat_contains_f1 = 1'b0;
        rat_contains_ffree = 1'b0;
        for (int i = 1; i < ARCH_NUM; i++) begin
            if (rat[i] == commit_free_pdst[0]) rat_contains_f0 = 1'b1;
            if (rat[i] == commit_free_pdst[1]) rat_contains_f1 = 1'b1;
            if (rat[i] == flush_free_pdst)     rat_contains_ffree = 1'b1;
        end
    end

    assign push_commit0 = commit_free_valid[0] && !rat_contains_f0;
    assign push_commit1 = commit_free_valid[1] && !rat_contains_f1;
    assign push_flush   = flush_free_valid && !rat_contains_ffree;
    assign commit_free_taken = {push_commit1, push_commit0};

    logic [$clog2(PRF_NUM+1)-1:0] push_cnt, pop_req, pop_fire, push_base;

    always_comb begin
        push_cnt = ($clog2(PRF_NUM+1))'(push_commit0)
                 + ($clog2(PRF_NUM+1))'(push_commit1)
                 + ($clog2(PRF_NUM+1))'(push_flush);
        // pop_req 用于就绪检查（不含 fire）；pop_fire 用于空闲列表实际弹出
        pop_req  = ($clog2(PRF_NUM+1))'(has_rd0) + ($clog2(PRF_NUM+1))'(has_rd1);
        pop_fire = ($clog2(PRF_NUM+1))'(has_rd0 && alloc_fire)
                 + ($clog2(PRF_NUM+1))'(has_rd1 && alloc_fire);
        push_base = fl_top - pop_fire;
    end

    // 组合分配物理寄存器（cycle N 有效）
    logic [PRF_W-1:0] pdst0_c, pdst1_c;
    assign pdst0_c = fl_stack[fl_top - 1];
    assign pdst1_c = fl_stack[fl_top - ($clog2(PRF_NUM+1))'(has_rd0 ? 2 : 1)];

    // ================= 派发与源重命名 =================
    assign disp_ready = ext_ready && (fl_top >= pop_req);

    assign alloc_old_pdst0_c = rat[disp_dec0.rd];
    // lane1 的旧映射：若与 lane0 写同一 rd，则 lane0 的新映射才是 lane1 的旧值
    assign alloc_old_pdst1_c = (has_rd0 && (disp_dec1.rd == disp_dec0.rd))
                               ? pdst0_c : rat[disp_dec1.rd];
    assign alloc_pdst0_c = pdst0_c;
    assign alloc_pdst1_c = pdst1_c;

    // ================= checkpoint 快照（槽位 = ROB 号）=================
    logic [PRF_W-1:0] cp [0:ROB_DEPTH-1][0:ARCH_NUM-1];
    logic [PRF_W-1:0] rat [0:ARCH_NUM-1];

    // ================= checkpoint 写入 =================
    always_ff @(posedge clk) begin
        if (alloc_fire && (disp_dec0.is_branch || disp_dec0.is_jal || disp_dec0.is_jalr)) begin
            for (int i = 0; i < ARCH_NUM; i++)
                cp[next_rob_id][i] <= (has_rd0 && (i == disp_dec0.rd)) ? pdst0_c : rat[i];
        end
        if (alloc_fire && disp_valid[1] &&
            (disp_dec1.is_branch || disp_dec1.is_jal || disp_dec1.is_jalr)) begin
            for (int i = 0; i < ARCH_NUM; i++)
                cp[next_rob_id + ROB_ID_W'(1)][i] <= (has_rd1 && (i == disp_dec1.rd)) ? pdst1_c
                                : (has_rd0 && (i == disp_dec0.rd)) ? pdst0_c
                                : rat[i];
        end
    end

    // ================= RAT =================
    always_ff @(posedge clk) begin
        if (reset) begin
            for (int i = 0; i < ARCH_NUM; i++) rat[i] <= i;
        end else if (flush_valid) begin
            for (int i = 0; i < ARCH_NUM; i++) rat[i] <= cp[flush_rob_id][i];
        end else if (alloc_fire) begin
            if (has_rd0) rat[disp_dec0.rd] <= pdst0_c;
            if (has_rd1) rat[disp_dec1.rd] <= pdst1_c;
        end
    end

    always_ff @(posedge clk) begin
        if (reset) begin
            prf_ready <= '1;
        end else begin
            for (int j = 0; j < WB_PORTS; j++)
                if (cdb_valid[j] && (cdb_pdst[j] != '0)) prf_ready[cdb_pdst[j]] <= 1'b1;
            if (alloc_fire && has_rd0)            prf_ready[pdst0_c] <= 1'b0;
            if (alloc_fire && has_rd1)            prf_ready[pdst1_c] <= 1'b0;
            if (push_commit0)                     prf_ready[commit_free_pdst[0]] <= 1'b1;
            if (push_commit1)                     prf_ready[commit_free_pdst[1]] <= 1'b1;
            if (flush_free_valid)                 prf_ready[flush_free_pdst]    <= 1'b1;
        end
    end

    always_ff @(posedge clk) begin
        if (reset) begin
            for (int i = 0; i < PRF_NUM - ARCH_NUM; i++)
                fl_stack[i] <= PRF_W'((PRF_NUM - ARCH_NUM - 1) - i + ARCH_NUM);
            fl_top <= PRF_NUM - ARCH_NUM;
        end else begin
            if (push_commit0)
                fl_stack[push_base] <= commit_free_pdst[0];
            if (push_commit1)
                fl_stack[push_base + ($clog2(PRF_NUM+1))'(push_commit0)] <= commit_free_pdst[1];
            if (push_flush)
                fl_stack[push_base + ($clog2(PRF_NUM+1))'(push_commit0)
                         + ($clog2(PRF_NUM+1))'(push_commit1)] <= flush_free_pdst;
            fl_top <= fl_top + push_cnt - pop_fire;
        end
    end

    // inst0 源重命名
    logic [PRF_W-1:0] ps1_0_c, ps2_0_c;
    logic             s1r_0_c, s2r_0_c;
    logic [31:0]      s1d_0_c, s2d_0_c;

    always_comb begin
        if (disp_dec0.rs1 == 5'd0) begin
            ps1_0_c = '0; s1r_0_c = 1'b1; s1d_0_c = 32'd0;
        end else begin
            ps1_0_c = rat[disp_dec0.rs1];
            s1r_0_c = prf_ready[ps1_0_c];
            s1d_0_c = prf[ps1_0_c];
            for (int j = 0; j < WB_PORTS; j++) begin
                if (cdb_valid[j] && (cdb_pdst[j] == ps1_0_c)) begin
                    s1d_0_c  = cdb_data[j];
                    s1r_0_c = 1'b1;
                end
            end
        end
        if (disp_dec0.rs2 == 5'd0) begin
            ps2_0_c = '0; s2r_0_c = 1'b1; s2d_0_c = 32'd0;
        end else begin
            ps2_0_c = rat[disp_dec0.rs2];
            s2r_0_c = prf_ready[ps2_0_c];
            s2d_0_c = prf[ps2_0_c];
            for (int j = 0; j < WB_PORTS; j++) begin
                if (cdb_valid[j] && (cdb_pdst[j] == ps2_0_c)) begin
                    s2d_0_c  = cdb_data[j];
                    s2r_0_c = 1'b1;
                end
            end
        end
    end

    // inst1 源重命名（含 pair 内转发）
    logic [PRF_W-1:0] ps1_1_c, ps2_1_c;
    logic             s1r_1_c, s2r_1_c;
    logic [31:0]      s1d_1_c, s2d_1_c;

    always_comb begin
        if (disp_dec1.rs1 == 5'd0) begin
            ps1_1_c = '0; s1r_1_c = 1'b1; s1d_1_c = 32'd0;
        end else if (has_rd0 && (disp_dec1.rs1 == disp_dec0.rd)) begin
            ps1_1_c = pdst0_c; s1r_1_c = 1'b0; s1d_1_c = 32'd0;
        end else begin
            ps1_1_c = rat[disp_dec1.rs1];
            s1r_1_c = prf_ready[ps1_1_c];
            s1d_1_c = prf[ps1_1_c];
            for (int j = 0; j < WB_PORTS; j++) begin
                if (cdb_valid[j] && (cdb_pdst[j] == ps1_1_c)) begin
                    s1d_1_c  = cdb_data[j];
                    s1r_1_c = 1'b1;
                end
            end
        end
        if (disp_dec1.rs2 == 5'd0) begin
            ps2_1_c = '0; s2r_1_c = 1'b1; s2d_1_c = 32'd0;
        end else if (has_rd0 && (disp_dec1.rs2 == disp_dec0.rd)) begin
            ps2_1_c = pdst0_c; s2r_1_c = 1'b0; s2d_1_c = 32'd0;
        end else begin
            ps2_1_c = rat[disp_dec1.rs2];
            s2r_1_c = prf_ready[ps2_1_c];
            s2d_1_c = prf[ps2_1_c];
            for (int j = 0; j < WB_PORTS; j++) begin
                if (cdb_valid[j] && (cdb_pdst[j] == ps2_1_c)) begin
                    s2d_1_c  = cdb_data[j];
                    s2r_1_c = 1'b1;
                end
            end
        end
    end

    // ================= 派发信息锁存（边沿 N+1，与内部处理同拍）=================
    always_ff @(posedge clk) begin
        if (reset) begin
            alloc_valid  <= '0;
            alloc_rob_id0 <= '0;
            alloc_rob_id1 <= '0;
            alloc_pdst0  <= '0;
            alloc_pdst1  <= '0;
            ps1_0 <= '0; ps2_0 <= '0; ps1_1 <= '0; ps2_1 <= '0;
            s1_ready_0 <= 1'b1; s2_ready_0 <= 1'b1;
            s1_ready_1 <= 1'b1; s2_ready_1 <= 1'b1;
            s1_data_0 <= 32'd0; s2_data_0 <= 32'd0;
            s1_data_1 <= 32'd0; s2_data_1 <= 32'd0;
        end else begin
            alloc_valid  <= {alloc_fire && disp_valid[1], alloc_fire};
            alloc_rob_id0 <= next_rob_id;
            alloc_rob_id1 <= next_rob_id + ROB_ID_W'(1);
            alloc_pdst0  <= pdst0_c;
            alloc_pdst1  <= pdst1_c;
            ps1_0 <= ps1_0_c; ps2_0 <= ps2_0_c;
            ps1_1 <= ps1_1_c; ps2_1 <= ps2_1_c;
            s1_ready_0 <= s1r_0_c; s2_ready_0 <= s2r_0_c;
            s1_ready_1 <= s1r_1_c; s2_ready_1 <= s2r_1_c;
            s1_data_0 <= s1d_0_c; s2_data_0 <= s2d_0_c;
            s1_data_1 <= s1d_1_c; s2_data_1 <= s2d_1_c;
        end
    end

endmodule
