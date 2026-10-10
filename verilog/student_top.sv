// student_top: RV32IM 乱序执行 CPU（阶段 5：OoO 集成）
// 微架构（已确认方案）：
//   取指（FIFO+在飞+重定向排空，复用阶段 4）→ 派发（rename：PRF/RAT/freelist/checkpoint，
//   宽度 1）→ 4 个发射队列（ALU/MDU/LSU/BRU，唤醒+最老优先）→ 各 FU → CDB 三数据端口
//   （ALU/MDU/LSU）+ BRU 单独 complete → ROB 按序提交（宽度 1）→ store 经 SB 提交后写内存
//   分支在 BRU 解析：taken → 广播 flush（ROB/IQ/SB/rename checkpoint/取指重定向），
//   ROB 逐拍归还被冲 pdst；已发出 AXI 写不可撤回（只有提交后的 store 才发写，SB 保证）
// 时序链：派发 cycle N → rename/ROB/SB 于边沿 N+1 → IQ 于边沿 N+2 → FU 执行 → CDB
module student_top import pkg_cpu2026::*; #(
    parameter int FETCH_DEPTH = 4,
    parameter int FIFO_DEPTH  = 4,
    parameter int DBG         = 2
) (
    input  logic        clock,
    input  logic        reset,

    output logic [31:0] araddr,
    output logic        arvalid,
    input  logic        arready,
    input  logic [31:0] rdata,
    input  logic [1:0]  rresp,
    input  logic        rvalid,
    output logic        rready,
    output logic [31:0] awaddr,
    output logic        awvalid,
    input  logic        awready,
    output logic [31:0] wdata,
    output logic [3:0]  wstrb,
    output logic        wvalid,
    input  logic        wready,
    input  logic [1:0]  bresp,
    input  logic        bvalid,
    output logic        bready
);

    // ================= axi_master =================
    logic        rd_req_valid;
    logic        rd_req_ready;
    logic [31:0] rd_req_addr;
    logic        rd_resp_valid;
    logic        rd_resp_ready;
    logic [31:0] rd_resp_data;
    logic [1:0]  rd_resp_err;
    logic        wr_req_valid;
    logic        wr_req_ready;
    logic [31:0] wr_req_addr;
    logic [31:0] wr_req_data;
    logic [3:0]  wr_req_strb;
    logic        wr_resp_valid;
    logic        wr_resp_ready;
    logic [1:0]  wr_resp_err;
    logic        exit_ack;

    assign rd_resp_ready = !reset;
    assign wr_resp_ready = !reset;

    axi_master #(
        .RD_QUEUE_DEPTH (8),
        .WR_QUEUE_DEPTH (4),
        .DBG            (0)
    ) u_master (
        .clk          (clock),
        .reset        (reset),
        .rd_req_valid (rd_req_valid),
        .rd_req_ready (rd_req_ready),
        .rd_req_addr  (rd_req_addr),
        .rd_resp_valid(rd_resp_valid),
        .rd_resp_ready(rd_resp_ready),
        .rd_resp_data (rd_resp_data),
        .rd_resp_err  (rd_resp_err),
        .wr_req_valid (wr_req_valid),
        .wr_req_ready (wr_req_ready),
        .wr_req_addr  (wr_req_addr),
        .wr_req_data  (wr_req_data),
        .wr_req_strb  (wr_req_strb),
        .wr_resp_valid(wr_resp_valid),
        .wr_resp_ready(wr_resp_ready),
        .wr_resp_err  (wr_resp_err),
        .exit_ack     (exit_ack),
        .araddr  (araddr),  .arvalid (arvalid),  .arready (arready),
        .rdata   (rdata),   .rresp   (rresp),    .rvalid  (rvalid),   .rready (rready),
        .awaddr  (awaddr),  .awvalid (awvalid),  .awready (awready),
        .wdata   (wdata),   .wstrb   (wstrb),    .wvalid  (wvalid),   .wready (wready),
        .bresp   (bresp),   .bvalid  (bvalid),   .bready  (bready)
    );

    // ================= 参数 =================
    localparam int ROB_W = 4;
    localparam int OP_W  = 8;

    // ================= 取指单元 =================
    logic [31:0] fetch_pc;
    logic [95:0] fq [0:FIFO_DEPTH-1];   // {pc, instr, predicted_target}
    logic [1:0]  fq_wptr, fq_rptr;
    logic [2:0]  fq_cnt;
    logic [0:7]  ro_type;               // 在飞读类型：0=I-cache 重填，1=LSU
    logic [2:0]  ro_wptr, ro_rptr;
    logic [3:0]  ro_cnt;

    logic lsu_req_valid, lsu_fire, lsu_rd_resp;

    // I-cache fetch 查询流水（1 拍）
    logic [31:0] lf_pc, lf_pd;
    logic        lf_on;
    logic        ic_req_ready, ic_hit;
    logic [31:0] ic_data;
    logic        ic_rd_req, ic_rd_ready, ic_resp_valid;
    logic [31:0] ic_rd_addr, ic_resp_data;
    logic        fq_pop;

    // 取指重定向（来自 BRU）
    logic        redirect_fire;
    logic [31:0] redirect_target;
    logic [31:0] fetch_redirect_target;

    // ================= 分支预测（BTB + BHT）=================
    logic        f_pred_hit, f_pred_taken;
    logic [31:0] f_pred_target;
    logic [31:0] predicted_next;

    btb_bht u_btb_bht (
        .clk(clock), .reset(reset),
        .pc(launch_pc),
        .predict_hit(f_pred_hit),
        .predict_taken(f_pred_taken),
        .predict_target(f_pred_target),
        .upd_valid(br_r_valid),
        .upd_pc(br_r_pc),
        .upd_taken(br_taken),
        .upd_target(br_actual_target)
    );
    assign predicted_next = (f_pred_hit && f_pred_taken) ? f_pred_target : launch_pc + 32'd4;

    // ================= 自修改代码安全：store 未提交前阻塞取指 =================
    // 课程正确性测试含"数据与代码重叠"的程序（无 FENCE.I 要求，参考实现为顺序取指），
    // 故取指不得越过未提交的 store：store 派发时清 FIFO 并排空在飞请求，
    // 提交后从 store_pc+4 重新取指。
    logic        store_inflight;
    logic [31:0] store_pc_next;
    logic        store_disp_fire;
    logic        store_commit_fire;

    // store 屏障已移除：取指不再串行等待 store 提交，
    // load/store 顺序由 SB 前递（地址/数据就绪检查 + 已提交未释放 stall）保证；
    // 结构冒险由 LSU 单发射串行 + LSU IQ 的 STORE_ORDER 约束兜底。
    assign store_disp_fire   = 1'b0;
    assign store_commit_fire = 1'b0;

    always_ff @(posedge clock) begin
        if (reset) begin
            store_inflight <= 1'b0;
            store_pc_next <= 32'd0;
        end else begin
            if (flush_valid) begin
                store_inflight <= 1'b0;   // 投机 store 被冲掉，解除取指阻塞
            end else if (store_disp_fire) begin
                store_inflight <= 1'b1;
                store_pc_next <= fd_pc + 32'd4;
            end else if (store_commit_fire) begin
                store_inflight <= 1'b0;
            end
        end
    end

    // 读端口仲裁：LSU 优先，I-cache 重填让位；在飞类型记入 ro_q 供响应路由
    assign rd_req_valid = lsu_req_valid || ic_rd_req;
    assign rd_req_addr  = lsu_req_valid ? lsu_addr : ic_rd_addr;
    assign ic_rd_ready  = rd_req_ready && !lsu_req_valid;
    assign lsu_fire     = lsu_req_valid && rd_req_ready;
    assign ic_resp_valid = rd_resp_valid && (ro_type[ro_rptr] == 1'b0);
    assign ic_resp_data  = rd_resp_data;
    assign lsu_rd_resp   = rd_resp_valid && (ro_type[ro_rptr] == 1'b1);

    // ================= 派发 =================
    logic [31:0] fd_pc, fd_instr, fd_pd_target;
    dec_info_t   fd_dec;
    assign fd_pc    = fq[fq_rptr][63:32];
    assign fd_instr = fq[fq_rptr][31:0];
    assign fd_pd_target = fq[fq_rptr][95:64];
    decoder u_fdec (.instr(fd_instr), .dec(fd_dec));

    logic fd_alu, fd_mul, fd_lsu, fd_bru, fd_store, fd_has_rd;
    assign fd_alu   = fd_dec.is_alu || fd_dec.is_lui || fd_dec.is_auipc;
    assign fd_mul   = fd_dec.is_mul || fd_dec.is_div;
    assign fd_lsu   = fd_dec.is_load || fd_dec.is_store;
    assign fd_bru   = fd_dec.is_branch || fd_dec.is_jal || fd_dec.is_jalr;
    assign fd_store = fd_dec.is_store;
    assign fd_has_rd = fd_dec.rd_write && (fd_dec.rd != 5'd0);

    logic disp_valid;
    logic disp_ready;
    logic halted;

    // ================= rename =================
    logic        rn_alloc_valid;
    logic [ROB_W-1:0] rn_alloc_rob_id;
    logic [PRF_W-1:0] rn_alloc_pdst, rn_alloc_pdst_c, rn_alloc_old_pdst_c;
    logic [PRF_W-1:0] rn_ps1, rn_ps2;
    logic        rn_s1_ready, rn_s2_ready;
    logic [31:0] rn_s1_data, rn_s2_data;

    // ================= ROB =================
    logic        rob_alloc_valid;
    logic        rob_alloc_ready;
    logic [ROB_W-1:0] rob_next_id, rob_head_id;
    logic        rob_commit_valid;
    logic        rob_commit_mask;
    logic [ROB_W-1:0] rob_commit_rob_id;
    logic        rob_commit_has_rd;
    logic [PRF_W-1:0] rob_commit_old_pdst;
    logic        rob_commit_is_store;
    logic        rob_commit_illegal;
    logic [31:0] rob_commit_pc;
    logic        rob_commit_free_valid;
    logic [PRF_W-1:0] rob_commit_free_pdst;
    logic        rob_commit_free_taken;
    logic [31:0] rob_lookup_pd_target;
    logic        rob_flush_free_valid;
    logic [PRF_W-1:0] rob_flush_free_pdst;
    logic        rob_flush_free_done;

    // ================= SB =================
    logic        sb_alloc_ready;
    logic        sb_fill_valid;
    logic [ROB_W-1:0] sb_fill_rob_id;
    logic [31:0] sb_fill_addr, sb_fill_data;
    logic [3:0]  sb_fill_wstrb;
    logic        sb_commit_valid;
    logic [ROB_W-1:0] sb_commit_rob_id;
    logic        sb_query_valid;
    logic [ROB_W-1:0] sb_query_rob_id;
    logic [31:0] sb_query_addr;
    logic [1:0]  sb_query_size;
    logic        sb_query_forward, sb_query_stall;
    logic [31:0] sb_query_data;

    // ================= IQ×4 =================
    logic iq_alu_alloc_valid, iq_alu_alloc_ready, iq_alu_alloc_ready_2, iq_alu_issue;
    logic iq_mdu_alloc_valid, iq_mdu_alloc_ready, iq_mdu_alloc_ready_2, iq_mdu_issue;
    logic iq_lsu_alloc_valid, iq_lsu_alloc_ready, iq_lsu_alloc_ready_2, iq_lsu_issue;
    logic iq_bru_alloc_valid, iq_bru_alloc_ready, iq_bru_alloc_ready_2, iq_bru_issue;
    logic [ROB_W-1:0] iq_alu_issue_rob, iq_mdu_issue_rob, iq_lsu_issue_rob, iq_bru_issue_rob;
    logic [PRF_W-1:0] iq_alu_issue_pdst, iq_mdu_issue_pdst, iq_lsu_issue_pdst, iq_bru_issue_pdst;
    logic [31:0] iq_alu_issue_op1, iq_alu_issue_op2, iq_alu_issue_imm, iq_alu_issue_pc;
    logic [31:0] iq_mdu_issue_op1, iq_mdu_issue_op2;
    logic [31:0] iq_lsu_issue_op1, iq_lsu_issue_op2, iq_lsu_issue_imm;
    logic [31:0] iq_bru_issue_op1, iq_bru_issue_op2, iq_bru_issue_imm, iq_bru_issue_pc;
    logic [OP_W-1:0] iq_alu_issue_op, iq_mdu_issue_op, iq_lsu_issue_op, iq_bru_issue_op;

    // ================= CDB（4 数据端口：ALU/MDU/LSU/BRU）=================
    logic [3:0] cdb_valid;
    logic [3:0][PRF_W-1:0] cdb_pdst;
    logic [3:0][ROB_W-1:0] cdb_rob_id;
    logic [3:0][31:0]     cdb_data;
    logic [3:0]           complete_valid;
    logic [3:0][ROB_W-1:0] complete_rob_id;

    // ================= 派发控制 =================
    logic target_ready;
    // 在飞分配预留：rn 阶段指令的分配比派发检查晚一拍采样，
    // 若该指令与当前派发目标同一 IQ，需保证有 2 个空槽（弃置槽位=永久死锁）
    assign target_ready = fd_alu ? (iq_alu_alloc_ready && (!iq_alu_alloc_valid || iq_alu_alloc_ready_2)) :
                          fd_mul ? (iq_mdu_alloc_ready && (!iq_mdu_alloc_valid || iq_mdu_alloc_ready_2)) :
                          fd_lsu ? (iq_lsu_alloc_ready && (!iq_lsu_alloc_valid || iq_lsu_alloc_ready_2)) :
                          fd_bru ? (iq_bru_alloc_ready && (!iq_bru_alloc_valid || iq_bru_alloc_ready_2)) : 1'b1;

    logic ext_ready;
    assign ext_ready = rob_alloc_ready && target_ready && (!fd_store || sb_alloc_ready);

    assign disp_valid = !reset && !halted && (fq_cnt != 3'd0) && !flush_valid && !illegal_halt;

    rename #(.WB_PORTS(4)) u_rename (
        .clk(clock), .reset(reset),
        .disp_valid(disp_valid),
        .ext_ready(ext_ready),
        .next_rob_id(rob_next_id),
        .disp_dec(fd_dec),
        .disp_ready(disp_ready),
        .alloc_valid(rn_alloc_valid),
        .alloc_rob_id(rn_alloc_rob_id),
        .alloc_pdst(rn_alloc_pdst),
        .alloc_pdst_c(rn_alloc_pdst_c),
        .alloc_old_pdst_c(rn_alloc_old_pdst_c),
        .ps1(rn_ps1), .ps2(rn_ps2),
        .s1_ready(rn_s1_ready), .s2_ready(rn_s2_ready),
        .s1_data(rn_s1_data), .s2_data(rn_s2_data),
        .commit_free_valid(rob_commit_free_valid),
        .commit_free_pdst(rob_commit_free_pdst),
        .commit_free_taken(rob_commit_free_taken),
        .flush_valid(flush_valid),
        .flush_rob_id(flush_rob_id),
        .flush_free_valid(rob_flush_free_valid),
        .flush_free_pdst(rob_flush_free_pdst),
        .cdb_valid(cdb_valid),
        .cdb_pdst(cdb_pdst),
        .cdb_data(cdb_data)
    );

    // 派发锁存（与 rename 输出同沿）
    logic        rn_valid;
    dec_info_t   rn_dec;
    logic [31:0] rn_pc, rn_instr;

    always_ff @(posedge clock) begin
        if (reset) rn_valid <= 1'b0;
        else begin
            rn_valid <= disp_valid && disp_ready;
            rn_dec <= fd_dec;
            rn_pc  <= fd_pc;
            rn_instr <= fd_instr;
        end
    end

    // ================= ROB =================
    assign rob_alloc_valid = disp_valid && disp_ready;

    rob #(
        .ROB_DEPTH(16), .ROB_ID_W(ROB_W), .COMMIT_WIDTH(1),
        .PRF_W(PRF_W), .WB_PORTS(4)
    ) u_rob (
        .clk(clock), .reset(reset),
        .alloc_valid(rob_alloc_valid),
        .alloc_ready(rob_alloc_ready),
        .alloc_pc(fd_pc),
        .alloc_arch_rd(fd_dec.rd),
        .alloc_has_rd(fd_has_rd),
        .alloc_pdst(rn_alloc_pdst_c),
        .alloc_old_pdst(fd_has_rd ? rn_alloc_old_pdst_c : '0),
        .alloc_is_store(fd_store),
        .alloc_is_branch(fd_dec.is_branch),
        .alloc_is_jal(fd_dec.is_jal),
        .alloc_is_jalr(fd_dec.is_jalr),
        .alloc_illegal(fd_dec.illegal),
        .alloc_pd_target(fd_pd_target),
        .next_rob_id(rob_next_id),
        .head_rob_id(rob_head_id),
        .complete_valid(complete_valid),
        .complete_rob_id(complete_rob_id),
        .commit_valid(rob_commit_valid),
        .commit_mask(rob_commit_mask),
        .commit_rob_id(rob_commit_rob_id),
        .commit_has_rd(rob_commit_has_rd),
        .commit_old_pdst(rob_commit_old_pdst),
        .commit_is_store(rob_commit_is_store),
        .commit_illegal(rob_commit_illegal),
        .commit_pc(rob_commit_pc),
        .commit_free_valid(rob_commit_free_valid),
        .commit_free_pdst(rob_commit_free_pdst),
        .commit_free_taken(rob_commit_free_taken),
        .flush_valid(flush_valid),
        .flush_rob_id(flush_rob_id),
        .flush_free_valid(rob_flush_free_valid),
        .flush_free_pdst(rob_flush_free_pdst),
        .flush_free_done(rob_flush_free_done),
        .lookup_rob_id(br_r_rob),
        .lookup_pd_target(rob_lookup_pd_target)
    );

    // 被覆盖旧映射：rd 的当前 RAT 映射（组合读 rename 的 rat —— 通过 ps1 取巧不可靠）
    // 处理：若 rd == rs1，ps1 即旧映射；否则用专用逻辑——rename 提供 old 映射输出
    // （见 rename.alloc_old_pdst_c 说明）

    // ================= SB =================
    logic sb_alloc_valid_x;
    assign sb_alloc_valid_x = disp_valid && disp_ready && fd_store;

    store_buffer #(.SB_DEPTH(8), .ROB_ID_W(ROB_W)) u_sb (
        .clk(clock), .reset(reset),
        .alloc_valid(sb_alloc_valid_x),
        .alloc_ready(sb_alloc_ready),
        .alloc_rob_id(rob_next_id),
        .fill_valid(sb_fill_valid),
        .fill_rob_id(sb_fill_rob_id),
        .fill_addr(sb_fill_addr),
        .fill_data(sb_fill_data),
        .fill_wstrb(sb_fill_wstrb),
        .commit_valid(sb_commit_valid),
        .commit_rob_id(sb_commit_rob_id),
        .wr_req_valid(wr_req_valid),
        .wr_req_ready(wr_req_ready),
        .wr_req_addr(wr_req_addr),
        .wr_req_data(wr_req_data),
        .wr_req_strb(wr_req_strb),
        .wr_resp_valid(wr_resp_valid),
        .exit_ack(exit_ack),
        .query_valid(sb_query_valid),
        .query_rob_id(sb_query_rob_id),
        .query_addr(sb_query_addr),
        .query_size(sb_query_size),
        .query_forward(sb_query_forward),
        .query_data(sb_query_data),
        .query_stall(sb_query_stall),
        .flush_valid(flush_valid),
        .flush_rob_id(flush_rob_id),
        .head_rob_id(rob_head_id),
        .tail_rob_id(rob_next_id)
    );

    assign sb_commit_valid = rob_commit_valid && rob_commit_mask && rob_commit_is_store;
    assign sb_commit_rob_id = rob_commit_rob_id;

    // ================= 分派 op 编码与操作数（cycle N+1，基于 rn_dec）=================
    logic [OP_W-1:0] d_alu_op, d_mdu_op, d_lsu_op, d_bru_op;
    logic d_alu_has1, d_alu_has2, d_alu_useimm;
    logic d_mdu_has1, d_mdu_has2;
    logic d_lsu_has1, d_lsu_has2;
    logic d_bru_has1, d_bru_has2;

    always_comb begin
        d_alu_op = {3'b000, rn_dec.alu_op};
        if (rn_dec.is_lui)   d_alu_op = 8'h20;
        if (rn_dec.is_auipc) d_alu_op = 8'h40;
        d_mdu_op = {3'b000, rn_dec.is_mul, rn_dec.mul_type, rn_dec.div_type};
        d_lsu_op = rn_dec.is_store ? {1'b1, 2'b00, rn_dec.st_type, 3'b000}
                                   : {2'b00, rn_dec.ld_type, 3'b000};
        d_bru_op = {rn_dec.is_jalr, rn_dec.is_jal, rn_dec.is_branch,
                    (rn_dec.rd_write && (rn_dec.rd != 5'd0)), 1'b0, rn_dec.br_type};
        d_alu_has1 = rn_dec.is_alu;
        d_alu_has2 = rn_dec.uses_rs2;
        d_alu_useimm = rn_dec.is_alu && !rn_dec.uses_rs2;
        d_mdu_has1 = 1'b1;
        d_mdu_has2 = 1'b1;
        d_lsu_has1 = 1'b1;
        d_lsu_has2 = rn_dec.is_store;
        d_bru_has1 = rn_dec.is_branch || rn_dec.is_jalr;
        d_bru_has2 = rn_dec.is_branch;
    end

    logic rn_alu, rn_mul, rn_lsu, rn_bru, rn_store;
    assign rn_alu = rn_valid && (rn_dec.is_alu || rn_dec.is_lui || rn_dec.is_auipc);
    assign rn_mul = rn_valid && (rn_dec.is_mul || rn_dec.is_div);
    assign rn_lsu = rn_valid && (rn_dec.is_load || rn_dec.is_store);
    assign rn_bru = rn_valid && (rn_dec.is_branch || rn_dec.is_jal || rn_dec.is_jalr);

    assign iq_alu_alloc_valid = rn_alu;
    assign iq_mdu_alloc_valid = rn_mul;
    assign iq_lsu_alloc_valid = rn_lsu;
    assign iq_bru_alloc_valid = rn_bru;

    // ================= IQ×4 =================
    issue_queue #(.IQ_DEPTH(8), .ISSUE_WIDTH(1), .WAKEUP_PORTS(4), .PRF_W(PRF_W), .ROB_ID_W(ROB_W), .OP_W(OP_W))
    u_iq_alu (
        .clk(clock), .reset(reset),
        .alloc_valid(iq_alu_alloc_valid), .alloc_ready(iq_alu_alloc_ready), .alloc_ready_2(iq_alu_alloc_ready_2),
        .alloc_rob_id(rn_alloc_rob_id), .alloc_pdst(rn_alloc_pdst),
        .alloc_has_rs1(d_alu_has1), .alloc_has_rs2(d_alu_has2),
        .alloc_p1(rn_ps1), .alloc_p2(rn_ps2),
        .alloc_s1_ready(rn_s1_ready), .alloc_s2_ready(rn_s2_ready),
        .alloc_s1_data(rn_s1_data), .alloc_s2_data(rn_s2_data),
        .alloc_imm(rn_dec.imm), .alloc_use_imm(d_alu_useimm),
        .alloc_pc(rn_pc), .alloc_op(d_alu_op),
        .wakeup_valid(cdb_valid), .wakeup_tag(cdb_pdst), .wakeup_data(cdb_data),
        .head_rob_id(rob_head_id),
        .issue_block(1'b0),
        .issue_valid(iq_alu_issue), .issue_rob_id(iq_alu_issue_rob),
        .issue_pdst(iq_alu_issue_pdst),
        .issue_op1(iq_alu_issue_op1), .issue_op2(iq_alu_issue_op2),
        .issue_imm(iq_alu_issue_imm), .issue_pc(iq_alu_issue_pc),
        .issue_op(iq_alu_issue_op),
        .flush_valid(flush_valid), .flush_rob_id(flush_rob_id)
    );

    issue_queue #(.IQ_DEPTH(8), .ISSUE_WIDTH(1), .WAKEUP_PORTS(4), .PRF_W(PRF_W), .ROB_ID_W(ROB_W), .OP_W(OP_W))
    u_iq_mdu (
        .clk(clock), .reset(reset),
        .alloc_valid(iq_mdu_alloc_valid), .alloc_ready(iq_mdu_alloc_ready), .alloc_ready_2(iq_mdu_alloc_ready_2),
        .alloc_rob_id(rn_alloc_rob_id), .alloc_pdst(rn_alloc_pdst),
        .alloc_has_rs1(d_mdu_has1), .alloc_has_rs2(d_mdu_has2),
        .alloc_p1(rn_ps1), .alloc_p2(rn_ps2),
        .alloc_s1_ready(rn_s1_ready), .alloc_s2_ready(rn_s2_ready),
        .alloc_s1_data(rn_s1_data), .alloc_s2_data(rn_s2_data),
        .alloc_imm(32'd0), .alloc_use_imm(1'b0),
        .alloc_pc(32'd0), .alloc_op(d_mdu_op),
        .wakeup_valid(cdb_valid), .wakeup_tag(cdb_pdst), .wakeup_data(cdb_data),
        .head_rob_id(rob_head_id),
        .issue_block(mdu_busy),
        .issue_valid(iq_mdu_issue), .issue_rob_id(iq_mdu_issue_rob),
        .issue_pdst(iq_mdu_issue_pdst),
        .issue_op1(iq_mdu_issue_op1), .issue_op2(iq_mdu_issue_op2),
        .issue_imm(), .issue_pc(),
        .issue_op(iq_mdu_issue_op),
        .flush_valid(flush_valid), .flush_rob_id(flush_rob_id)
    );

    issue_queue #(.IQ_DEPTH(8), .ISSUE_WIDTH(1), .WAKEUP_PORTS(4), .PRF_W(PRF_W), .ROB_ID_W(ROB_W), .OP_W(OP_W), .STORE_ORDER(1))
    u_iq_lsu (
        .clk(clock), .reset(reset),
        .alloc_valid(iq_lsu_alloc_valid), .alloc_ready(iq_lsu_alloc_ready), .alloc_ready_2(iq_lsu_alloc_ready_2),
        .alloc_rob_id(rn_alloc_rob_id), .alloc_pdst(rn_alloc_pdst),
        .alloc_has_rs1(d_lsu_has1), .alloc_has_rs2(d_lsu_has2),
        .alloc_p1(rn_ps1), .alloc_p2(rn_ps2),
        .alloc_s1_ready(rn_s1_ready), .alloc_s2_ready(rn_s2_ready),
        .alloc_s1_data(rn_s1_data), .alloc_s2_data(rn_s2_data),
        .alloc_imm(rn_dec.imm), .alloc_use_imm(1'b0),
        .alloc_pc(32'd0), .alloc_op(d_lsu_op),
        .wakeup_valid(cdb_valid), .wakeup_tag(cdb_pdst), .wakeup_data(cdb_data),
        .head_rob_id(rob_head_id),
        .issue_block(lsu_busy),
        .issue_valid(iq_lsu_issue), .issue_rob_id(iq_lsu_issue_rob),
        .issue_pdst(iq_lsu_issue_pdst),
        .issue_op1(iq_lsu_issue_op1), .issue_op2(iq_lsu_issue_op2),
        .issue_imm(iq_lsu_issue_imm), .issue_pc(),
        .issue_op(iq_lsu_issue_op),
        .flush_valid(flush_valid), .flush_rob_id(flush_rob_id)
    );

    issue_queue #(.IQ_DEPTH(8), .ISSUE_WIDTH(1), .WAKEUP_PORTS(4), .PRF_W(PRF_W), .ROB_ID_W(ROB_W), .OP_W(OP_W))
    u_iq_bru (
        .clk(clock), .reset(reset),
        .alloc_valid(iq_bru_alloc_valid), .alloc_ready(iq_bru_alloc_ready), .alloc_ready_2(iq_bru_alloc_ready_2),
        .alloc_rob_id(rn_alloc_rob_id), .alloc_pdst(rn_alloc_pdst),
        .alloc_has_rs1(d_bru_has1), .alloc_has_rs2(d_bru_has2),
        .alloc_p1(rn_ps1), .alloc_p2(rn_ps2),
        .alloc_s1_ready(rn_s1_ready), .alloc_s2_ready(rn_s2_ready),
        .alloc_s1_data(rn_s1_data), .alloc_s2_data(rn_s2_data),
        .alloc_imm(rn_dec.imm), .alloc_use_imm(1'b0),
        .alloc_pc(rn_pc), .alloc_op(d_bru_op),
        .wakeup_valid(cdb_valid), .wakeup_tag(cdb_pdst), .wakeup_data(cdb_data),
        .head_rob_id(rob_head_id),
        .issue_block(1'b0),
        .issue_valid(iq_bru_issue), .issue_rob_id(iq_bru_issue_rob),
        .issue_pdst(iq_bru_issue_pdst),
        .issue_op1(iq_bru_issue_op1), .issue_op2(iq_bru_issue_op2),
        .issue_imm(iq_bru_issue_imm), .issue_pc(iq_bru_issue_pc),
        .issue_op(iq_bru_issue_op),
        .flush_valid(flush_valid), .flush_rob_id(flush_rob_id)
    );

    // ================= ALU FU（1 拍执行）=================
    logic        alu_r_valid;
    logic [ROB_W-1:0] alu_r_rob;
    logic [PRF_W-1:0] alu_r_pdst;
    logic [OP_W-1:0]  alu_r_op;
    logic [31:0] alu_r_op1, alu_r_op2, alu_r_imm, alu_r_pc;

    always_ff @(posedge clock) begin
        if (reset) begin
            alu_r_valid <= 1'b0;
        end else begin
            if (flush_valid && alu_r_valid &&
                is_younger(alu_r_rob, flush_rob_id)) begin
                alu_r_valid <= 1'b0;
            end else if (iq_alu_issue) begin
                alu_r_valid <= 1'b1;
                alu_r_rob <= iq_alu_issue_rob;
                alu_r_pdst <= iq_alu_issue_pdst;
                alu_r_op <= iq_alu_issue_op;
                alu_r_op1 <= iq_alu_issue_op1;
                alu_r_op2 <= iq_alu_issue_op2;
                alu_r_imm <= iq_alu_issue_imm;
                alu_r_pc <= iq_alu_issue_pc;
            end else begin
                alu_r_valid <= 1'b0;
            end
        end
    end

    logic [2:0]  alu_mode;
    logic [31:0] alu_a, alu_b;
    logic [4:0]  alu_op_sel;
    logic [31:0] alu_out;
    assign alu_mode = alu_r_op[7:5];
    always_comb begin
        unique case (alu_mode)
        3'd1: begin alu_a = alu_r_imm; alu_b = 32'd0; alu_op_sel = ALU_PASSA; end  // LUI
        3'd2: begin alu_a = alu_r_pc;  alu_b = alu_r_imm; alu_op_sel = ALU_ADD; end  // AUIPC
        3'd3, 3'd4: begin alu_a = alu_r_pc; alu_b = 32'd4; alu_op_sel = ALU_ADD; end // JAL/JALR link
        default: begin alu_a = alu_r_op1; alu_b = alu_r_op2; alu_op_sel = alu_r_op[4:0]; end
        endcase
    end
    alu u_alu (.op(alu_op_sel), .a(alu_a), .b(alu_b), .out(alu_out), .eq(), .lt(), .ltu());

    assign cdb_valid[0] = alu_r_valid;
    assign cdb_pdst[0]  = alu_r_pdst;
    assign cdb_rob_id[0] = alu_r_rob;
    assign cdb_data[0]  = alu_out;

    // ================= MDU FU =================
    logic        mdu_start_pulse, mdu_busy, mdu_valid;
    logic [31:0] mdu_result;
    logic        mdu_r_valid, mdu_discard;
    logic [ROB_W-1:0] mdu_r_rob;
    logic [PRF_W-1:0] mdu_r_pdst;
    logic [31:0] mdu_r_op1, mdu_r_op2;
    logic [OP_W-1:0] mdu_r_op;

    mdu u_mdu (
        .clk(clock), .reset(reset),
        .start(iq_mdu_issue),
        .is_mul(iq_mdu_issue_op[4]),
        .mul_type(iq_mdu_issue_op[3:2]),
        .div_type(iq_mdu_issue_op[1:0]),
        .a(iq_mdu_issue_op1), .b(iq_mdu_issue_op2),
        .busy(mdu_busy), .valid(mdu_valid), .result(mdu_result)
    );

    always_ff @(posedge clock) begin
        if (reset) begin
            mdu_r_valid <= 1'b0;
            mdu_discard <= 1'b0;
        end else begin
            if (iq_mdu_issue) begin
                mdu_r_valid <= 1'b1;
                mdu_discard <= 1'b0;
                mdu_r_rob <= iq_mdu_issue_rob;
                mdu_r_pdst <= iq_mdu_issue_pdst;
            end else if (mdu_valid) begin
                mdu_r_valid <= 1'b0;
            end
            if (flush_valid && mdu_r_valid && is_younger(mdu_r_rob, flush_rob_id)) begin
                mdu_discard <= 1'b1;   // 结果丢弃（mdu 继续空转完成）
            end
        end
    end

    assign cdb_valid[1]  = mdu_valid && mdu_r_valid && !mdu_discard;
    assign cdb_pdst[1]   = mdu_r_pdst;
    assign cdb_rob_id[1] = mdu_r_rob;
    assign cdb_data[1]   = mdu_result;

    // ================= LSU FU =================
    typedef enum logic [1:0] {LS_IDLE, LS_STORE, LS_WAIT_SB, LS_WAIT_MEM} ls_state_t;
    ls_state_t ls_state;
    logic [ROB_W-1:0] ls_rob;
    logic [PRF_W-1:0] ls_pdst;
    logic [OP_W-1:0]  ls_op;
    logic [31:0] ls_addr, ls_data, ls_result;
    logic        ls_done_valid;   // 单拍：load 结果/ store 完成
    logic        ls_is_load;

    logic [31:0] lsu_addr;   // 读请求地址（与 fetch 仲裁）
    assign lsu_addr = ls_addr;

    // load 读请求：WAIT_SB 且不 stall 且不 forward → 发请求（flush 当拍不发，避免无主在飞请求）
    assign lsu_req_valid = !reset && !halted && !flush_valid &&
                           (ls_state == LS_WAIT_SB) && ls_is_load &&
                           !sb_query_stall && !sb_query_forward;
    assign sb_query_valid = (ls_state == LS_WAIT_SB) && ls_is_load;
    assign sb_query_rob_id = ls_rob;
    assign sb_query_addr = ls_addr;
    assign sb_query_size = (ls_op[5:3] == LD_LW)  ? 2'd2 :
                           (ls_op[5:3] == LD_LH || ls_op[5:3] == LD_LHU) ? 2'd1 : 2'd0;

    // 响应路由：load 响应到达且 LSU 在等待（丢弃计数为 0 才认为是本 load 的响应）
    logic load_resp_route;
    assign load_resp_route = lsu_rd_resp && (lsu_drain == 4'd0) &&
                             (ls_state == LS_WAIT_MEM) && ls_is_load;

    // load 数据字节提取 + 符号扩展
    logic [31:0] ls_word;
    assign ls_word = rd_resp_data;
    logic [31:0] ls_extracted;
    always_comb begin
        unique case (ls_op[5:3])
        LD_LB:  ls_extracted = {{24{ls_word[7 + 8*ls_addr[1:0]]}}, ls_word[8*ls_addr[1:0] +: 8]};
        LD_LBU: ls_extracted = {24'd0, ls_word[8*ls_addr[1:0] +: 8]};
        LD_LH:  ls_extracted = {{16{ls_word[15 + 8*ls_addr[1]]}}, ls_word[8*ls_addr[1] +: 16]};
        LD_LHU: ls_extracted = {16'd0, ls_word[8*ls_addr[1] +: 16]};
        default: ls_extracted = ls_word;
        endcase
    end

    // store 掩码与数据移位（store 编码：op[7]=1, op[4:3]=st_type）
    logic [4:0] ls_shift;
    logic [3:0] ls_strb;
    always_comb begin
        if (ls_op[4:3] == ST_SB) begin
            ls_strb = 4'b0001 << ls_addr[1:0];
            ls_shift = {3'b0, ls_addr[1:0]};
        end else if (ls_op[4:3] == ST_SH) begin
            ls_strb = ls_addr[1] ? 4'b1100 : 4'b0011;
            ls_shift = {4'b0, ls_addr[1]};
        end else begin
            ls_strb = 4'hF;
            ls_shift = 5'd0;
        end
    end

    // LSU 在飞请求丢弃计数：被冲刷的投机 load 请求已在途，其响应先到；
// 吞掉这些响应，避免误配给下一笔真实 load（响应按序返回，drain 先于当前响应归零）
    logic [3:0] lsu_drain;
    always_ff @(posedge clock) begin
        if (reset) begin
            lsu_drain <= 4'd0;
        end else begin
            if (flush_valid && (ls_state == LS_WAIT_MEM) &&
                is_younger(ls_rob, flush_rob_id) && !lsu_rd_resp)
                lsu_drain <= lsu_drain + 4'd1;
            else if (lsu_rd_resp && (lsu_drain != 4'd0))
                lsu_drain <= lsu_drain - 4'd1;
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            ls_state <= LS_IDLE;
            ls_done_valid <= 1'b0;
        end else begin
            ls_done_valid <= 1'b0;
            unique case (ls_state)
            LS_IDLE: begin
                if (iq_lsu_issue && !flush_valid) begin
                    ls_rob <= iq_lsu_issue_rob;
                    ls_pdst <= iq_lsu_issue_pdst;
                    ls_op <= iq_lsu_issue_op;
                    ls_addr <= iq_lsu_issue_op1 + iq_lsu_issue_imm;
                    ls_data <= iq_lsu_issue_op2;
                    ls_is_load <= ~iq_lsu_issue_op[7];
                    if (iq_lsu_issue_op[7]) ls_state <= LS_STORE;
                    else                    ls_state <= LS_WAIT_SB;
                end
            end
            LS_STORE: begin
                ls_state <= LS_IDLE;       // 回填 SB + complete 于本拍组合完成
            end
            LS_WAIT_SB: begin
                if (sb_query_stall) begin
                    // 保持
                end else if (sb_query_forward) begin
                    ls_result <= sb_query_data;
                    ls_done_valid <= 1'b1;
                    ls_state <= LS_IDLE;
                end else if (lsu_fire) begin
                    ls_state <= LS_WAIT_MEM;   // 请求真正发出才等待响应
                end
            end
            LS_WAIT_MEM: begin
                if (load_resp_route) begin
                    ls_result <= ls_extracted;
                    ls_done_valid <= 1'b1;
                    ls_state <= LS_IDLE;
                end
            end
            default: ls_state <= LS_IDLE;
            endcase
            // flush 在飞 load/store（读请求已发出则计入 drain 由响应路径吞掉；
            // 同时清掉可能残留的 done 写回）
            if (flush_valid && (ls_state != LS_IDLE) && is_younger(ls_rob, flush_rob_id)) begin
                ls_state <= LS_IDLE;
                ls_done_valid <= 1'b0;
            end
        end
    end

    logic lsu_busy;
    assign lsu_busy = (ls_state != LS_IDLE);

    // SB 回填（LS_STORE 拍）
    assign sb_fill_valid = (ls_state == LS_STORE);
    assign sb_fill_rob_id = ls_rob;
    assign sb_fill_addr = ls_addr;
    assign sb_fill_data = ls_data << ls_shift;
    assign sb_fill_wstrb = ls_strb;

    assign cdb_valid[2]  = ls_done_valid && ls_is_load;      // load 写回
    assign cdb_pdst[2]   = ls_pdst;
    assign cdb_rob_id[2] = ls_rob;
    assign cdb_data[2]   = ls_result;

    // ================= BRU =================
    logic        br_r_valid;
    logic [ROB_W-1:0] br_r_rob;
    logic [PRF_W-1:0] br_r_pdst;
    logic [31:0] br_r_pc, br_r_op1, br_r_op2, br_r_imm;
    logic [OP_W-1:0] br_r_op;

    always_ff @(posedge clock) begin
        if (reset) begin
            br_r_valid <= 1'b0;
        end else begin
            if (flush_valid && iq_bru_issue && is_younger(iq_bru_issue_rob, flush_rob_id)) begin
                br_r_valid <= 1'b0;   // 同拍发射的被 flush 分支直接丢弃
            end else if (flush_valid && br_r_valid && is_younger(br_r_rob, flush_rob_id)) begin
                br_r_valid <= 1'b0;   // 已发射但被 flush 的分支：抑制重定向
            end else if (iq_bru_issue) begin
                br_r_valid <= 1'b1;
                br_r_rob <= iq_bru_issue_rob;
                br_r_pdst <= iq_bru_issue_pdst;
                br_r_pc <= iq_bru_issue_pc;
                br_r_op1 <= iq_bru_issue_op1;
                br_r_op2 <= iq_bru_issue_op2;
                br_r_imm <= iq_bru_issue_imm;
                br_r_op <= iq_bru_issue_op;
            end else begin
                br_r_valid <= 1'b0;
            end
        end
    end

    logic br_taken;
    always_comb begin
        br_taken = 1'b0;
        if (br_r_valid) begin
            if (br_r_op[6] || br_r_op[7]) begin          // jal / jalr
                br_taken = 1'b1;
            end else if (br_r_op[5]) begin               // branch
                unique case (br_r_op[2:0])
                BR_BEQ:  br_taken = (br_r_op1 == br_r_op2);
                BR_BNE:  br_taken = (br_r_op1 != br_r_op2);
                BR_BLT:  br_taken = ($signed(br_r_op1) <  $signed(br_r_op2));
                BR_BGE:  br_taken = ($signed(br_r_op1) >= $signed(br_r_op2));
                BR_BLTU: br_taken = (br_r_op1 <  br_r_op2);
                BR_BGEU: br_taken = (br_r_op1 >= br_r_op2);
                default: ;
                endcase
            end
        end
    end
    // 实际目标：taken 分支/jal → pc+imm；jalr → rs1+imm；not-taken → pc+4
    logic [31:0] br_actual_target;
    assign br_actual_target = br_r_op[7] ? ((br_r_op1 + br_r_imm) & ~32'h1)
                                          : (br_taken ? (br_r_pc + br_r_imm)
                                                      : (br_r_pc + 32'd4));
    assign redirect_target = br_actual_target;
    // 取指重定向 = 分支误预测 或 store 提交后的重取指
    assign redirect_fire = (br_r_valid && !halted &&
                            (br_actual_target != rob_lookup_pd_target)) || store_commit_fire;
    assign fetch_redirect_target = store_commit_fire ? store_pc_next : br_actual_target;

    // flush 广播（仅来自 BRU 误预测；store 提交重取指不触发 flush）
    logic        flush_valid;
    logic [ROB_W-1:0] flush_rob_id;
    assign flush_valid  = br_r_valid && !halted &&
                          (br_actual_target != rob_lookup_pd_target);
    assign flush_rob_id = br_r_rob;

    // BRU 写回链接值（jal/jalr 且有 rd：op[4]=1）→ CDB 端口 3
    assign cdb_valid[3]  = br_r_valid && (br_r_op[6] || br_r_op[7]) && br_r_op[4];
    assign cdb_pdst[3]   = br_r_pdst;
    assign cdb_rob_id[3] = br_r_rob;
    assign cdb_data[3]   = br_r_pc + 32'd4;

    // BRU complete（ROB 端口 3）；端口 2 覆盖 load 写回与 store 完成
    assign complete_valid = {br_r_valid,
                             cdb_valid[2] || (ls_state == LS_STORE),
                             cdb_valid[1],
                             cdb_valid[0]};
    assign complete_rob_id = {br_r_rob,
                              (ls_state == LS_STORE) ? ls_rob : cdb_rob_id[2],
                              cdb_rob_id[1],
                              cdb_rob_id[0]};

    // ================= 环序函数（基于 head 的年龄比较，无半环歧义）=================
    function automatic logic is_younger(input logic [ROB_W-1:0] a, b);
        return (a - rob_head_id) > (b - rob_head_id);
    endfunction

    // ================= 取指单元（I-cache 驱动）=================
    assign fq_pop             = disp_valid && disp_ready;

    // 查询发射条件：查询槽空闲（或本拍消费）且 I-cache 可接收且 FIFO 有空间
    // 消费同拍再发射时，新查询的 pc 是本拍命中的预测目标 lf_pd
    logic [31:0] launch_pc;
    logic ic_launch;
    assign launch_pc = (lf_on && ic_hit) ? lf_pd : fetch_pc;
    assign ic_launch = !reset && !halted && !redirect_fire && ic_req_ready &&
                       (!lf_on || ic_hit) &&
                       ((fq_cnt + (lf_on ? 3'd1 : 3'd0)) < FIFO_DEPTH);

    icache #(.LINES(64), .LINE_WORDS(8), .MAXOUT(4)) u_icache (
        .clk(clock), .reset(reset),
        .req_pc(launch_pc), .req_valid(ic_launch), .req_ready(ic_req_ready),
        .hit(ic_hit), .hit_data(ic_data),
        .rd_req(ic_rd_req), .rd_ready(ic_rd_ready), .rd_addr(ic_rd_addr),
        .resp_valid(ic_resp_valid), .resp_data(ic_resp_data)
    );

    always_ff @(posedge clock) begin
        if (reset) begin
            fetch_pc <= 32'd0;
            lf_on <= 1'b0;
            fq_wptr <= 2'd0; fq_rptr <= 2'd0; fq_cnt <= 3'd0;
            ro_wptr <= 3'd0; ro_rptr <= 3'd0; ro_cnt <= 4'd0;
        end else begin
            if (redirect_fire) begin
                fetch_pc <= fetch_redirect_target;
                lf_on <= 1'b0;
                fq_wptr <= 2'd0; fq_rptr <= 2'd0; fq_cnt <= 3'd0;
            end else begin
                // 命中：入队并前进（lf_on 未必清零——同拍可能再次发射）
                if (lf_on && ic_hit) begin
                    fq[fq_wptr] <= {lf_pd, lf_pc, ic_data};
                    if (DBG >= 2)
                        $fdisplay(dbg_fd, "[fqpush] c=%0d pc=%08h instr=%08h", cycle_cnt, lf_pc, ic_data);
                    fq_wptr <= fq_wptr + 2'd1;
                    fetch_pc <= lf_pd;
                    lf_on <= 1'b0;
                end else if (lf_on && !ic_hit) begin
                    // 缺失：等待重填完成后对同一 pc 重发
                    lf_on <= 1'b0;
                end
                if (ic_launch) begin
                    lf_on <= 1'b1;
                    lf_pc <= launch_pc;
                    lf_pd <= predicted_next;
                end
                fq_cnt <= fq_cnt + ((lf_on && ic_hit) ? 3'd1 : 3'd0) - (fq_pop ? 3'd1 : 3'd0);
                if (fq_pop) fq_rptr <= fq_rptr + 2'd1;
            end
            if (ic_rd_req && ic_rd_ready) ro_type[ro_wptr] <= 1'b0;
            if (lsu_fire)                ro_type[ro_wptr] <= 1'b1;
            if ((ic_rd_req && ic_rd_ready) || lsu_fire) ro_wptr <= ro_wptr + 3'd1;
            if (rd_resp_valid) ro_rptr <= ro_rptr + 3'd1;
            ro_cnt <= ro_cnt + (((ic_rd_req && ic_rd_ready) || lsu_fire) ? 4'd1 : 4'd0)
                              - (rd_resp_valid ? 4'd1 : 4'd0);
        end
    end

    // ================= 终止 =================
    logic illegal_halt;
    always_ff @(posedge clock) begin
        if (reset) begin
            halted <= 1'b0;
            illegal_halt <= 1'b0;
        end else begin
            if (exit_ack) halted <= 1'b1;
            if (rob_commit_valid && rob_commit_illegal && !illegal_halt) begin
                illegal_halt <= 1'b1;
                $display("[cpu] ILLEGAL committed instr pc=%08h", rob_commit_pc);
            end
        end
    end

    // ================= 调试 =================
`ifndef SYNTHESIS
    integer dbg_fd;
    integer cycle_cnt;
    initial begin
        dbg_fd = $fopen("cpu_dbg.log", "w");
    end
    always @(posedge clock) begin
        if (reset) cycle_cnt <= 0;
        else begin
            cycle_cnt <= cycle_cnt + 1;
            if (DBG) begin
                for (int j = 0; j < 3; j++)
                    if (cdb_valid[j])
                        $fdisplay(dbg_fd, "[cdb] c=%0d port=%0d rob=%0d pdst=%0d data=%08h",
                                  cycle_cnt, j, cdb_rob_id[j], cdb_pdst[j], cdb_data[j]);
                if (rob_commit_valid)
                    $fdisplay(dbg_fd, "[commit] c=%0d rob=%0d pc=%08h has_rd=%b old_pdst=%0d store=%b sbv=%b",
                              cycle_cnt, rob_commit_rob_id, rob_commit_pc,
                              rob_commit_has_rd, rob_commit_old_pdst, rob_commit_is_store,
                              sb_commit_valid);
                if (flush_valid)
                    $fdisplay(dbg_fd, "[flush] c=%0d rob=%0d target=%08h",
                              cycle_cnt, flush_rob_id, redirect_target);
                if (rob_flush_free_valid && DBG >= 2)
                    $fdisplay(dbg_fd, "[free] c=%0d pdst=%0d", cycle_cnt, rob_flush_free_pdst);
                if (redirect_fire)
                    $fdisplay(dbg_fd, "[fetch] c=%0d REDIRECT to %08h", cycle_cnt, redirect_target);
                if (exit_ack)
                    $fdisplay(dbg_fd, "[cpu] c=%0d EXIT_ACK", cycle_cnt);
                if (rn_valid)
                    $fdisplay(dbg_fd, "[disp] c=%0d rob=%0d pc=%08h pdst=%0d ps1=%0d s1r=%b ps2=%0d s2r=%b st=%b ld=%b",
                              cycle_cnt, rn_alloc_rob_id, rn_pc, rn_alloc_pdst,
                              rn_ps1, rn_s1_ready, rn_ps2, rn_s2_ready,
                              rn_dec.is_store, rn_dec.is_load);
                if (DBG >= 2) begin
                    $fdisplay(dbg_fd, "[fetch] c=%0d pc=%08h fq=%0d lf=%b ic_hit=%b ro=%0d lsu=%0d tgt=%08h",
                              cycle_cnt, fetch_pc, fq_cnt, lf_on, ic_hit, ro_cnt, ls_state,
                              fetch_redirect_target);
                    if (fq_cnt != 3'd0 && !disp_ready)
                        $fdisplay(dbg_fd, "[dispstall] c=%0d rob_rdy=%b iq_alu=%b iq_mdu=%b iq_lsu=%b iq_bru=%b sb=%b",
                                  cycle_cnt, rob_alloc_ready, iq_alu_alloc_ready, iq_mdu_alloc_ready,
                                  iq_lsu_alloc_ready, iq_bru_alloc_ready, sb_alloc_ready);
                    if (iq_alu_issue) $fdisplay(dbg_fd, "[iss-alu] c=%0d rob=%0d", cycle_cnt, iq_alu_issue_rob);
                    if (iq_mdu_issue) $fdisplay(dbg_fd, "[iss-mdu] c=%0d rob=%0d", cycle_cnt, iq_mdu_issue_rob);
                    if (iq_lsu_issue) $fdisplay(dbg_fd, "[iss-lsu] c=%0d rob=%0d op=%02h op1=%08h op2=%08h imm=%08h",
                                               cycle_cnt, iq_lsu_issue_rob, iq_lsu_issue_op,
                                               iq_lsu_issue_op1, iq_lsu_issue_op2, iq_lsu_issue_imm);
                    if (iq_bru_issue) $fdisplay(dbg_fd, "[iss-bru] c=%0d rob=%0d", cycle_cnt, iq_bru_issue_rob);
                    if ((ls_state == LS_WAIT_SB) && sb_query_forward)
                        $fdisplay(dbg_fd, "[ldf] c=%0d rob=%0d addr=%08h data=%08h", cycle_cnt, ls_rob, ls_addr, sb_query_data);
                    if ((ls_state == LS_WAIT_SB) && !sb_query_stall && !sb_query_forward && lsu_fire)
                        $fdisplay(dbg_fd, "[ldm] c=%0d rob=%0d addr=%08h", cycle_cnt, ls_rob, ls_addr);
                end
            end
        end
    end
`endif

endmodule
