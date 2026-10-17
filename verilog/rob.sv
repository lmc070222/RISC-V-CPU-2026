// rob: 重排序缓冲区（按序分配、乱序完成、按序提交）
// 与提示词的一处偏差（与方案 A 一致）：提交不写架构寄存器堆——纯 PRF + checkpoint 方案中
// 数据始终住在 PRF，架构状态由 RAT 映射表达，恢复走 checkpoint；提交只需：
//   1) 按序推进 head；2) 输出旧物理寄存器（commit_old_pdst）给 rename 回收
// flush：年轻条目立即清 valid、tail 回退（编号复用）；被冲条目的 pdst 由 FREE 状态机
//   逐拍归还（flush_free_*，直接接 rename 的 flush_free 端口）；归还期间暂停分配。
// 约定：ROB_DEPTH 为 2 的幂（环序取模）；条目索引 = 其 ROB 号。
// 时序：alloc/complete/flush 于 cycle N 有效 → 边沿 N+1 生效；commit_* 为组合输出。
module rob #(
    parameter int ROB_DEPTH    = 16,
    parameter int ROB_ID_W     = 4,
    parameter int COMMIT_WIDTH = 2,
    parameter int PRF_W        = 6,
    parameter int WB_PORTS     = 3
) (
    input  logic clk,
    input  logic reset,

    // ---- 分配（来自 rename，2 宽）----
    input  logic [1:0]              alloc_valid,
    output logic                    alloc_ready,     // 组合（满 / FREE 归还中则 0）
    output logic                    alloc_ready_2,   // 可接收成对分配
    input  logic [1:0][31:0]        alloc_pc,
    input  logic [1:0][4:0]         alloc_arch_rd,
    input  logic [1:0]              alloc_has_rd,
    input  logic [1:0][PRF_W-1:0]   alloc_pdst,
    input  logic [1:0][PRF_W-1:0]   alloc_old_pdst,
    input  logic [1:0]              alloc_is_store,
    input  logic [1:0]              alloc_is_branch,
    input  logic [1:0]              alloc_is_jal,
    input  logic [1:0]              alloc_is_jalr,
    input  logic [1:0]              alloc_illegal,
    input  logic [1:0][31:0]        alloc_pd_target,   // 预测目标（PC+4 或 BTB 目标）
    output logic [ROB_ID_W-1:0]     next_rob_id,     // 尾部指针（组合，给 rename）
    output logic [ROB_ID_W-1:0]     head_rob_id,     // 头部指针（组合，给 IQ 选择基准）

    // ---- 完成（来自 CDB，携带 ROB 号）----
    input  logic [WB_PORTS-1:0]               complete_valid,
    input  logic [WB_PORTS-1:0][ROB_ID_W-1:0] complete_rob_id,

    // ---- 提交（组合输出）----
    output logic                                  commit_valid,
    output logic [COMMIT_WIDTH-1:0]               commit_mask,
    output logic [COMMIT_WIDTH-1:0][ROB_ID_W-1:0] commit_rob_id,
    output logic [COMMIT_WIDTH-1:0]               commit_has_rd,
    output logic [COMMIT_WIDTH-1:0][PRF_W-1:0]    commit_old_pdst,
    output logic [COMMIT_WIDTH-1:0]               commit_is_store,
    output logic [COMMIT_WIDTH-1:0]               commit_illegal,
    output logic [COMMIT_WIDTH-1:0][31:0]         commit_pc,

    // 延迟释放：checkpoint 恢复可能复活旧物理寄存器，故提交的旧 pdst 只在
    // 没有未解析分支时才归还空闲列表（双通道）
    output logic [1:0]              commit_free_valid,
    output logic [1:0][PRF_W-1:0]   commit_free_pdst,
    input  logic [1:0]              commit_free_taken,   // rename 实际接收（未被跳过）

    // ---- flush ----
    input  logic                    flush_valid,
    input  logic [ROB_ID_W-1:0]     flush_rob_id,
    output logic                    flush_free_valid,   // 组合：本拍归还一个 pdst
    output logic [PRF_W-1:0]        flush_free_pdst,
    output logic                    flush_free_done,    // 归还结束（FREE→IDLE）

    // 预测目标查找（BRU 误预测判定用）
    input  logic [ROB_ID_W-1:0]     lookup_rob_id,
    output logic [31:0]             lookup_pd_target
);

    // ================= 条目（索引 = ROB 号）=================
    logic                    valid     [0:ROB_DEPTH-1];
    logic [31:0]             pc        [0:ROB_DEPTH-1];
    logic [4:0]              arch_rd   [0:ROB_DEPTH-1];
    logic                    has_rd    [0:ROB_DEPTH-1];
    logic [PRF_W-1:0]        pdst      [0:ROB_DEPTH-1];
    logic [PRF_W-1:0]        old_pdst  [0:ROB_DEPTH-1];
    logic                    done      [0:ROB_DEPTH-1];
    logic                    is_store  [0:ROB_DEPTH-1];
    logic                    is_branch [0:ROB_DEPTH-1];
    logic                    is_jal    [0:ROB_DEPTH-1];
    logic                    is_jalr   [0:ROB_DEPTH-1];
    logic                    illegal   [0:ROB_DEPTH-1];
    logic [31:0]             pd_target [0:ROB_DEPTH-1];

    logic [ROB_ID_W-1:0] head, tail;

    // ================= 延迟释放队列 =================
    logic [$clog2(ROB_DEPTH+1)-1:0] unresolved_cnt;   // 在飞未解析分支数
    logic [PRF_W-1:0]              pend_q [0:2*ROB_DEPTH-1];
    logic [$clog2(2*ROB_DEPTH)-1:0]   pend_wptr, pend_rptr;   // 回绕模 2*ROB_DEPTH
    logic [$clog2(2*ROB_DEPTH+1)-1:0] pend_cnt;

    logic pend_push0, pend_push1;
    assign pend_push0 = commit_mask[0] && commit_has_rd[0];
    assign pend_push1 = (COMMIT_WIDTH > 1) && commit_mask[1] && commit_has_rd[1];
    assign commit_free_valid[0] = !reset && (unresolved_cnt == '0) && (pend_cnt != '0);
    assign commit_free_valid[1] = !reset && (unresolved_cnt == '0) && (pend_cnt >= 2) &&
                                  commit_free_valid[0] && commit_free_taken[0];
    assign commit_free_pdst[0]  = pend_q[pend_rptr];
    assign commit_free_pdst[1]  = pend_q[pend_rptr + $clog2(2*ROB_DEPTH)'(1)];

    // ================= FREE 状态机（flush 逐拍归还）=================
    typedef enum logic {S_IDLE, S_FREE} fstate_t;
    fstate_t fstate;
    logic [ROB_ID_W-1:0] free_ptr, free_end;

    assign flush_free_valid = (fstate == S_FREE) && (free_ptr != free_end) && has_rd[free_ptr];
    assign flush_free_pdst  = pdst[free_ptr];
    assign flush_free_done  = (fstate == S_FREE) && (free_ptr == free_end);

    // ================= 环序与满 =================
    logic full1, full2;
    assign full1 = valid[(tail + ROB_ID_W'(1)) % ROB_DEPTH];
    assign full2 = valid[(tail + ROB_ID_W'(2)) % ROB_DEPTH];
    assign alloc_ready   = !reset && !full1 && (fstate == S_IDLE);
    assign alloc_ready_2 = !reset && !full2 && (fstate == S_IDLE);
    assign next_rob_id = tail;
    assign head_rob_id = head;
    assign lookup_pd_target = pd_target[lookup_rob_id];

    // ================= 环序 younger 判定 =================
    logic [ROB_ID_W-1:0] flush_diff [0:ROB_DEPTH-1];
    for (genvar i = 0; i < ROB_DEPTH; i++) begin : g_diff
        assign flush_diff[i] = ROB_ID_W'(i) - flush_rob_id;
    end

    // ================= 提交组合逻辑 =================
`ifndef SYNTHESIS
`endif
    logic [COMMIT_WIDTH-1:0] commit_mask_c;
    logic                    commit_valid_c;
    logic [ROB_ID_W-1:0]     commit_cnt_c;

    // pending 释放队列水位反压：环深 32，本拍最多 2 新推 + 2 轮转；
    // 余量不足时暂停提交，防止环溢出（会造成释放列表注入重复/垃圾项）
    wire commit_gate = (pend_cnt <= ($clog2(2*ROB_DEPTH+1))'(28));

    always_comb begin
        commit_mask_c = '0;
        commit_valid_c = 1'b0;
        commit_cnt_c = '0;
        for (int i = 0; i < COMMIT_WIDTH; i++) begin
            logic [ROB_ID_W-1:0] idx;
            logic ok;
            idx = (head + ROB_ID_W'(i)) % ROB_DEPTH;
            if (i == 0) ok = commit_gate && valid[idx] && done[idx];
            else        ok = commit_mask_c[i-1] && valid[idx] && done[idx];
            commit_mask_c[i] = ok;
            commit_valid_c |= ok;
            if (ok) commit_cnt_c = commit_cnt_c + ROB_ID_W'(1);
        end
    end

    assign commit_valid = commit_valid_c;
    assign commit_mask  = commit_mask_c;

    for (genvar i = 0; i < COMMIT_WIDTH; i++) begin : g_commit
        logic [ROB_ID_W-1:0] idx;
        assign idx = (head + ROB_ID_W'(i)) % ROB_DEPTH;
        assign commit_rob_id[i]   = idx;
        assign commit_has_rd[i]   = commit_mask[i] && has_rd[idx];
        assign commit_old_pdst[i] = old_pdst[idx];
        assign commit_is_store[i] = commit_mask[i] && is_store[idx];
        assign commit_illegal[i]  = commit_mask[i] && illegal[idx];
        assign commit_pc[i]       = pc[idx];
    end

    // ================= 条目更新（单块）=================
    logic [$clog2(ROB_DEPTH+1)-1:0] uc_alloc_inc;
    logic uc_complete_dec;
    logic [$clog2(ROB_DEPTH+1)-1:0] uc_flush_dec;
    logic [$clog2(ROB_DEPTH+1)-1:0] uc_complete_dec_w;

    always_comb begin
        uc_alloc_inc = 1'b0;
        if (alloc_valid[0] && alloc_ready &&
            (alloc_is_branch[0] || alloc_is_jal[0] || alloc_is_jalr[0])) uc_alloc_inc = 1'b1;
        if (alloc_valid[1] && alloc_valid[0] && alloc_ready_2 &&
            (alloc_is_branch[1] || alloc_is_jal[1] || alloc_is_jalr[1]))
            uc_alloc_inc = uc_alloc_inc + $clog2(ROB_DEPTH+1)'(1);
        uc_complete_dec = 1'b0;
        uc_complete_dec_w = '0;
        for (int j = 0; j < WB_PORTS; j++) begin
            if (complete_valid[j] && valid[complete_rob_id[j]] &&
                (is_branch[complete_rob_id[j]] || is_jal[complete_rob_id[j]] ||
                 is_jalr[complete_rob_id[j]])) begin
                uc_complete_dec = 1'b1;
                uc_complete_dec_w = uc_complete_dec_w + $clog2(ROB_DEPTH+1)'(1);
            end
        end
        // flush 清掉的分支数（组合）
        uc_flush_dec = '0;
        if (flush_valid) begin
            for (int i = 0; i < ROB_DEPTH; i++) begin
                if (valid[i] &&
                    ((ROB_ID_W'(i) - (flush_rob_id + ROB_ID_W'(1))) <
                     (tail - (flush_rob_id + ROB_ID_W'(1)))) &&
                    (is_branch[i] || is_jal[i] || is_jalr[i])) begin
                    uc_flush_dec = uc_flush_dec + $clog2(ROB_DEPTH+1)'(1);
                end
            end
        end
    end

    always_ff @(posedge clk) begin
        if (reset) begin
            for (int i = 0; i < ROB_DEPTH; i++) begin
                valid[i] <= 1'b0;
                done[i]  <= 1'b0;
            end
            head <= '0;
            tail <= '0;
            fstate <= S_IDLE;
            free_ptr <= '0;
            free_end <= '0;
            unresolved_cnt <= '0;
            pend_wptr <= '0; pend_rptr <= '0; pend_cnt <= '0;
        end else begin
            // 1) 完成（先于分配，避免同拍 done 双写歧义）
            for (int j = 0; j < WB_PORTS; j++) begin
                if (complete_valid[j]) begin
                    done[complete_rob_id[j]] <= 1'b1;
                end
            end
            // 2) 分配（可 1 或 2 条）
            if (alloc_valid[0] && alloc_ready) begin
                valid[tail]     <= 1'b1;
                done[tail]      <= 1'b0;
                pc[tail]        <= alloc_pc[0];
                arch_rd[tail]   <= alloc_arch_rd[0];
                has_rd[tail]    <= alloc_has_rd[0];
                pdst[tail]      <= alloc_pdst[0];
                old_pdst[tail]  <= alloc_old_pdst[0];
                is_store[tail]  <= alloc_is_store[0];
                is_branch[tail] <= alloc_is_branch[0];
                is_jal[tail]    <= alloc_is_jal[0];
                is_jalr[tail]   <= alloc_is_jalr[0];
                illegal[tail]   <= alloc_illegal[0];
                pd_target[tail] <= alloc_pd_target[0];
                if (alloc_valid[1] && alloc_ready_2) begin
                    valid[(tail + ROB_ID_W'(1)) % ROB_DEPTH]     <= 1'b1;
                    done[(tail + ROB_ID_W'(1)) % ROB_DEPTH]      <= 1'b0;
                    pc[(tail + ROB_ID_W'(1)) % ROB_DEPTH]        <= alloc_pc[1];
                    arch_rd[(tail + ROB_ID_W'(1)) % ROB_DEPTH]   <= alloc_arch_rd[1];
                    has_rd[(tail + ROB_ID_W'(1)) % ROB_DEPTH]    <= alloc_has_rd[1];
                    pdst[(tail + ROB_ID_W'(1)) % ROB_DEPTH]      <= alloc_pdst[1];
                    old_pdst[(tail + ROB_ID_W'(1)) % ROB_DEPTH]  <= alloc_old_pdst[1];
                    is_store[(tail + ROB_ID_W'(1)) % ROB_DEPTH]  <= alloc_is_store[1];
                    is_branch[(tail + ROB_ID_W'(1)) % ROB_DEPTH] <= alloc_is_branch[1];
                    is_jal[(tail + ROB_ID_W'(1)) % ROB_DEPTH]    <= alloc_is_jal[1];
                    is_jalr[(tail + ROB_ID_W'(1)) % ROB_DEPTH]   <= alloc_is_jalr[1];
                    illegal[(tail + ROB_ID_W'(1)) % ROB_DEPTH]   <= alloc_illegal[1];
                    pd_target[(tail + ROB_ID_W'(1)) % ROB_DEPTH] <= alloc_pd_target[1];
                    tail <= tail + ROB_ID_W'(2);
                end else begin
                    tail <= tail + ROB_ID_W'(1);
                end
            end
            // 3) 提交推进
            for (int i = 0; i < COMMIT_WIDTH; i++) begin
                if (commit_mask[i]) begin
                    valid[(head + ROB_ID_W'(i)) % ROB_DEPTH] <= 1'b0;
                end
            end
            if (commit_valid) begin
                head <= head + commit_cnt_c;
            end
            // 4) flush / FREE 推进（同块，避免多驱动）
            if (flush_valid) begin
                // 清空环区间 [flush_rob_id+1, tail)：长度 rng（< 16，无半环歧义）
                logic [ROB_ID_W-1:0] rng;
                rng = tail - (flush_rob_id + ROB_ID_W'(1));
                for (int i = 0; i < ROB_DEPTH; i++) begin
                    if ((ROB_ID_W'(i) - (flush_rob_id + ROB_ID_W'(1))) < rng) begin
                        if (valid[i]) begin
                            valid[i] <= 1'b0;
                        end
                    end
                end
                tail     <= flush_rob_id + ROB_ID_W'(1);
                free_ptr <= flush_rob_id + ROB_ID_W'(1);
                free_end <= tail;          // NBA 取旧 tail
                fstate   <= S_FREE;
            end else begin
                case (fstate)
                S_IDLE: begin end
                S_FREE: begin
                    if (free_ptr == free_end) fstate <= S_IDLE;
                    else free_ptr <= free_ptr + ROB_ID_W'(1);
                end
                default: ;
                endcase
            end
            // 5) 延迟释放队列（双通道）：提交时入队，无未解析分支时出队归还；
            //    被 rename 跳过（RAT 引用）的条目轮转到队尾，避免阻塞后续释放
            if (pend_push0)
                pend_q[pend_wptr] <= old_pdst[head];
            if (pend_push1)
                pend_q[pend_wptr + $clog2(ROB_DEPTH+1)'(pend_push0)] <=
                    old_pdst[(head + ROB_ID_W'(1)) % ROB_DEPTH];
            if (pend_push1)
                pend_wptr <= pend_wptr + $clog2(ROB_DEPTH+1)'(pend_push0) + $clog2(ROB_DEPTH+1)'(1);
            else if (pend_push0)
                pend_wptr <= pend_wptr + $clog2(ROB_DEPTH+1)'(1);

            if (commit_free_valid[0] && commit_free_taken[0]) begin
                if (commit_free_valid[1] && commit_free_taken[1]) begin
                    pend_rptr <= pend_rptr + $clog2(ROB_DEPTH+1)'(2);
                end else begin
                    // 第一通道归还、第二通道被跳过：轮转重排（弹出两槽：归还 + 轮转）
                    if (commit_free_valid[1]) begin
                        pend_q[pend_wptr + $clog2(ROB_DEPTH+1)'(pend_push0) +
                               $clog2(ROB_DEPTH+1)'(pend_push1)] <=
                            pend_q[pend_rptr + $clog2(ROB_DEPTH+1)'(1)];
                        pend_wptr <= pend_wptr + $clog2(ROB_DEPTH+1)'(pend_push0) +
                                     $clog2(ROB_DEPTH+1)'(pend_push1) + $clog2(ROB_DEPTH+1)'(1);
                        pend_rptr <= pend_rptr + $clog2(ROB_DEPTH+1)'(2);
                    end else begin
                        pend_rptr <= pend_rptr + $clog2(ROB_DEPTH+1)'(1);
                    end
                end
            end else if (commit_free_valid[0]) begin
                // 第一通道被跳过：轮转重排
                pend_q[pend_wptr + $clog2(ROB_DEPTH+1)'(pend_push0) +
                       $clog2(ROB_DEPTH+1)'(pend_push1)] <= pend_q[pend_rptr];
                pend_wptr <= pend_wptr + $clog2(ROB_DEPTH+1)'(pend_push0) +
                             $clog2(ROB_DEPTH+1)'(pend_push1) + $clog2(ROB_DEPTH+1)'(1);
                pend_rptr <= pend_rptr + $clog2(ROB_DEPTH+1)'(1);
            end
            // 轮转（requeue）是"弹出+重推"净零，不进入计数
            pend_cnt <= pend_cnt
                        + $clog2(ROB_DEPTH+1)'(pend_push0) + $clog2(ROB_DEPTH+1)'(pend_push1)
                        - $clog2(ROB_DEPTH+1)'(commit_free_valid[0] && commit_free_taken[0])
                        - $clog2(ROB_DEPTH+1)'(commit_free_valid[1] && commit_free_taken[1]);
            // 未解析分支数净更新（单一 NBA，饱和钳制防下溢回绕；
            // 提前释放的旧 pdst 由 rename 的 RAT 检查兜底跳过）
            unresolved_cnt <= ((unresolved_cnt + uc_alloc_inc)
                               >= (uc_complete_dec_w + uc_flush_dec))
                            ? (unresolved_cnt + uc_alloc_inc
                               - uc_complete_dec_w - uc_flush_dec)
                            : '0;
        end
    end

endmodule
