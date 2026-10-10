// issue_queue: 发射队列（唤醒 + 最老优先选择 + flush）
// 面积关键点（综合重点监控）：tag 比较器数量 ≈ IQ_DEPTH × 2 源 × WAKEUP_PORTS
//   + 入队当拍唤醒旁路 2 × WAKEUP_PORTS + 选择器 ISSUE_WIDTH × IQ_DEPTH 个 age 比较器，
//   均随 IQ_DEPTH / WAKEUP_PORTS 线性增长（位宽 PRF_W / ROB_ID_W）。
// 职责划分：flush 只清 valid；被冲掉条目的物理寄存器由 ROB 统一逐拍归还（单一归还源）。
// 时序：alloc/wakeup/flush 于 cycle N 有效 → 边沿 N+1 生效；select 为组合输出。
module issue_queue #(
    parameter int IQ_DEPTH      = 8,
    parameter int ISSUE_WIDTH   = 1,
    parameter int WAKEUP_PORTS  = 3,
    parameter int PRF_W         = 6,
    parameter int ROB_ID_W      = 4,
    parameter int OP_W          = 8,
    parameter logic STORE_ORDER = 0,
    parameter logic DEBUG       = 0
) (
    input  logic clk,
    input  logic reset,

    // ---- 入队 ----
    input  logic                    alloc_valid,
    output logic                    alloc_ready,
    output logic                    alloc_ready_2,   // 空闲槽 >= 2（覆盖在飞分配的第二拍竞态）
    input  logic [ROB_ID_W-1:0]     alloc_rob_id,
    input  logic [PRF_W-1:0]        alloc_pdst,
    input  logic                    alloc_has_rs1,
    input  logic                    alloc_has_rs2,
    input  logic [PRF_W-1:0]        alloc_p1,
    input  logic [PRF_W-1:0]        alloc_p2,
    input  logic                    alloc_s1_ready,
    input  logic                    alloc_s2_ready,
    input  logic [31:0]             alloc_s1_data,
    input  logic [31:0]             alloc_s2_data,
    input  logic [31:0]             alloc_imm,
    input  logic                    alloc_use_imm,
    input  logic [31:0]             alloc_pc,
    input  logic [OP_W-1:0]         alloc_op,

    // ---- 唤醒（tag + data 广播）----
    input  logic [WAKEUP_PORTS-1:0]             wakeup_valid,
    input  logic [WAKEUP_PORTS-1:0][PRF_W-1:0] wakeup_tag,
    input  logic [WAKEUP_PORTS-1:0][31:0]      wakeup_data,

    // ---- 选择基准（ROB head，环序最老优先）----
    input  logic [ROB_ID_W-1:0]     head_rob_id,

    // ---- 发射抑制（如 FU busy：本拍不发射任何条目）----
    input  logic                    issue_block,

    // ---- 发射 ----
    output logic [ISSUE_WIDTH-1:0]                issue_valid,
    output logic [ISSUE_WIDTH-1:0][ROB_ID_W-1:0] issue_rob_id,
    output logic [ISSUE_WIDTH-1:0][PRF_W-1:0]    issue_pdst,
    output logic [ISSUE_WIDTH-1:0][31:0]         issue_op1,
    output logic [ISSUE_WIDTH-1:0][31:0]         issue_op2,
    output logic [ISSUE_WIDTH-1:0][31:0]         issue_imm,
    output logic [ISSUE_WIDTH-1:0][31:0]         issue_pc,
    output logic [ISSUE_WIDTH-1:0][OP_W-1:0]     issue_op,

    // ---- flush（环序年轻于 flush_rob_id 的条目清空）----
    input  logic                    flush_valid,
    input  logic [ROB_ID_W-1:0]     flush_rob_id
);

    localparam int IDX_W = $clog2(IQ_DEPTH);

    // ================= 条目寄存器 =================
    logic                    valid     [0:IQ_DEPTH-1];
    logic [ROB_ID_W-1:0]     rob_id    [0:IQ_DEPTH-1];
    logic [PRF_W-1:0]        pdst      [0:IQ_DEPTH-1];
    logic                    has_rs1   [0:IQ_DEPTH-1];
    logic                    has_rs2   [0:IQ_DEPTH-1];
    logic [PRF_W-1:0]        p1        [0:IQ_DEPTH-1];
    logic [PRF_W-1:0]        p2        [0:IQ_DEPTH-1];
    logic                    s1_ready  [0:IQ_DEPTH-1];
    logic                    s2_ready  [0:IQ_DEPTH-1];
    logic [31:0]             s1_data   [0:IQ_DEPTH-1];
    logic [31:0]             s2_data   [0:IQ_DEPTH-1];
    logic [31:0]             imm       [0:IQ_DEPTH-1];
    logic                    use_imm   [0:IQ_DEPTH-1];
    logic [31:0]             pc        [0:IQ_DEPTH-1];
    logic [OP_W-1:0]         op        [0:IQ_DEPTH-1];

    // ================= 入队槽查找 =================
    logic [IDX_W-1:0] free_idx;
    logic             has_free;
    logic [IDX_W:0]   free_cnt;
    always_comb begin
        free_idx = '0;
        has_free = 1'b0;
        free_cnt = '0;
        for (int i = 0; i < IQ_DEPTH; i++) begin
            if (!valid[i]) begin
                if (!has_free) free_idx = IDX_W'(i);
                has_free = 1'b1;
                free_cnt = free_cnt + 1'b1;
            end
        end
    end
    assign alloc_ready = !reset && has_free;
    // 派发检查用：需容纳本拍在飞分配（rn 寄存器导致分配比派发检查晚一拍）
    assign alloc_ready_2 = !reset && (free_cnt >= (IDX_W+1)'(2));

    // 入队当拍唤醒旁路（rename 的 s1_ready 不含当拍 CDB，这里补齐）
    logic        alloc_w1_match, alloc_w2_match;
    logic [31:0] alloc_w1_data, alloc_w2_data;
    always_comb begin
        alloc_w1_match = 1'b0; alloc_w1_data = '0;
        alloc_w2_match = 1'b0; alloc_w2_data = '0;
        for (int j = 0; j < WAKEUP_PORTS; j++) begin
            if (wakeup_valid[j] && (wakeup_tag[j] == alloc_p1) &&
                alloc_has_rs1 && !alloc_s1_ready) begin
                alloc_w1_match = 1'b1;
                alloc_w1_data  = wakeup_data[j];
            end
            if (wakeup_valid[j] && (wakeup_tag[j] == alloc_p2) &&
                alloc_has_rs2 && !alloc_s2_ready) begin
                alloc_w2_match = 1'b1;
                alloc_w2_data  = wakeup_data[j];
            end
        end
    end

    // ================= flush 环序（基于 head 的年龄比较，无半环歧义）=================
    logic [ROB_ID_W-1:0] flush_age;
    assign flush_age = flush_rob_id - head_rob_id;

    // ================= 条目更新（单块，避免多驱动）=================
    always_ff @(posedge clk) begin
        if (reset) begin
            for (int i = 0; i < IQ_DEPTH; i++) valid[i] <= 1'b0;
        end else begin
            // 1) 发射弹出
            for (int r = 0; r < ISSUE_WIDTH; r++) begin
                if (sel_found[r]) valid[sel_idx[r]] <= 1'b0;
            end
            // 2) flush：年龄大于 flush 点的清空
            if (flush_valid) begin
                for (int i = 0; i < IQ_DEPTH; i++) begin
                    if (valid[i] && ((rob_id[i] - head_rob_id) > flush_age)) begin
                        valid[i] <= 1'b0;
                    end
                end
            end
            // 3) 唤醒（valid 且源未就绪的条目）
            for (int i = 0; i < IQ_DEPTH; i++) begin
                if (valid[i] && has_rs1[i] && !s1_ready[i]) begin
                    for (int j = 0; j < WAKEUP_PORTS; j++) begin
                        if (wakeup_valid[j] && (wakeup_tag[j] == p1[i])) begin
                            s1_ready[i] <= 1'b1;
                            s1_data[i]  <= wakeup_data[j];
                        end
                    end
                end
                if (valid[i] && has_rs2[i] && !s2_ready[i]) begin
                    for (int j = 0; j < WAKEUP_PORTS; j++) begin
                        if (wakeup_valid[j] && (wakeup_tag[j] == p2[i])) begin
                            s2_ready[i] <= 1'b1;
                            s2_data[i]  <= wakeup_data[j];
                        end
                    end
                end
            end
            // 4) 入队（最后写：free 槽 valid=0，与弹出/flush 槽位不同；
            //    flush 当拍禁止入队，防止新条目逃过清除）
            if (alloc_valid && alloc_ready && !flush_valid) begin
                valid[free_idx]     <= 1'b1;
                rob_id[free_idx]    <= alloc_rob_id;
                pdst[free_idx]      <= alloc_pdst;
                has_rs1[free_idx]   <= alloc_has_rs1;
                has_rs2[free_idx]   <= alloc_has_rs2;
                p1[free_idx]        <= alloc_p1;
                p2[free_idx]        <= alloc_p2;
                s1_ready[free_idx]  <= alloc_s1_ready || alloc_w1_match;
                s2_ready[free_idx]  <= alloc_s2_ready || alloc_w2_match;
                s1_data[free_idx]   <= alloc_w1_match ? alloc_w1_data : alloc_s1_data;
                s2_data[free_idx]   <= alloc_w2_match ? alloc_w2_data : alloc_s2_data;
                imm[free_idx]       <= alloc_imm;
                use_imm[free_idx]   <= alloc_use_imm;
                pc[free_idx]        <= alloc_pc;
                op[free_idx]        <= alloc_op;
            end
        end
    end

    // ================= 选择（ISSUE_WIDTH 轮，环序最老优先）=================
    logic [ROB_ID_W-1:0] age [0:IQ_DEPTH-1];
    logic                ready [0:IQ_DEPTH-1];
    logic                ld_blocked [0:IQ_DEPTH-1];
    for (genvar i = 0; i < IQ_DEPTH; i++) begin : g_age
        assign age[i]   = rob_id[i] - head_rob_id;   // 模 2^W 环距
        // flush 当拍禁止发射：防止被冲分支进入 FU 产生幽灵重定向
        assign ready[i] = !issue_block && !flush_valid && valid[i] &&
                          (!has_rs1[i] || s1_ready[i]) &&
                          (!has_rs2[i] || s2_ready[i]) &&
                          !ld_blocked[i];
    end

    // STORE_ORDER=1 时（LSU 队列）：load 不得越过未发射的旧 store——
    // 旧 store 地址未解析时 SB 查询会 stall load，而旧 store 又因 LSU
    // 被占无法发射，形成循环等待死锁。
    generate
        if (STORE_ORDER) begin : g_so
            for (genvar i = 0; i < IQ_DEPTH; i++) begin : g_so_i
                logic older_store;
                always_comb begin
                    older_store = 1'b0;
                    for (int j = 0; j < IQ_DEPTH; j++) begin
                        if (valid[j] && op[j][7] && (age[j] < age[i]))
                            older_store = 1'b1;
                    end
                    ld_blocked[i] = !op[i][7] && older_store;
                end
            end
        end else begin : g_nso
            for (genvar i = 0; i < IQ_DEPTH; i++) begin : g_nso_i
                assign ld_blocked[i] = 1'b0;
            end
        end
    endgenerate

    logic [ISSUE_WIDTH-1:0]            sel_found;
    logic [ISSUE_WIDTH-1:0][IDX_W-1:0] sel_idx;

    always_comb begin
        sel_found = '0;
        sel_idx   = '0;
        for (int r = 0; r < ISSUE_WIDTH; r++) begin
            logic [IDX_W-1:0] cur;
            logic             cf;
            cur = '0; cf = 1'b0;
            for (int i = 0; i < IQ_DEPTH; i++) begin
                if (ready[i]) begin
                    logic masked;
                    masked = 1'b0;
                    for (int r2 = 0; r2 < r; r2++) begin
                        if (sel_found[r2] && (sel_idx[r2] == i)) masked = 1'b1;
                    end
                    if (!masked) begin
                        if (!cf) begin
                            cur = IDX_W'(i);
                            cf  = 1'b1;
                        end else if (age[i] < age[cur]) begin
                            cur = IDX_W'(i);
                        end
                    end
                end
            end
            sel_found[r] = cf;
            sel_idx[r]   = cur;
        end
    end

    for (genvar r = 0; r < ISSUE_WIDTH; r++) begin : g_issue
        assign issue_valid[r]  = sel_found[r];
        assign issue_rob_id[r] = rob_id[sel_idx[r]];
        assign issue_pdst[r]   = pdst[sel_idx[r]];
        assign issue_op1[r]    = s1_data[sel_idx[r]];
        assign issue_op2[r]    = use_imm[sel_idx[r]] ? imm[sel_idx[r]] : s2_data[sel_idx[r]];
        assign issue_imm[r]    = imm[sel_idx[r]];
        assign issue_pc[r]     = pc[sel_idx[r]];
        assign issue_op[r]     = op[sel_idx[r]];
    end

`ifndef SYNTHESIS
    if (DEBUG) begin : g_dbg
        integer dbg_cyc = 0;
        always @(posedge clk) begin
            dbg_cyc <= dbg_cyc + 1;
            if (!reset) begin
                for (int i = 0; i < IQ_DEPTH; i++) begin
                    for (int j = 0; j < WAKEUP_PORTS; j++) begin
                        if (valid[i] && has_rs1[i] && !s1_ready[i] && wakeup_valid[j] &&
                            (wakeup_tag[j] == p1[i]))
                            $display("[iq-wake] c=%0d rob=%0d tag=%0d p1=%0d", dbg_cyc, rob_id[i], wakeup_tag[j], p1[i]);
                    end
                end
                if (alloc_valid && alloc_ready)
                    $display("[iq-alloc] c=%0d rob=%0d p1=%0d s1r=%b w1m=%b wv=%b",
                             dbg_cyc, alloc_rob_id, alloc_p1, alloc_s1_ready, alloc_w1_match,
                             {wakeup_valid[0], wakeup_valid[1], wakeup_valid[2], wakeup_valid[3]});
                if (alloc_valid && !alloc_ready)
                    $display("[iq-drop] c=%0d rob=%0d p1=%0d", dbg_cyc, alloc_rob_id, alloc_p1);
                if (sel_found[0])
                    $display("[iq-iss] c=%0d rob=%0d p1=%0d s1r=%b", dbg_cyc, rob_id[sel_idx[0]],
                             p1[sel_idx[0]], s1_ready[sel_idx[0]]);
            end
        end
    end
`endif

endmodule
