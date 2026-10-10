// store_buffer: 存储缓冲（store 提交后才真正写内存 + load 转发）
// 设计（与设计备忘 §3 一致）：
//   - store 在派发时分配条目（携带 ROB 号），LSU 执行时回填 addr/data/wstrb（各带就绪标志）
//   - 提交（ROB 按序提交 store）→ committed=1 → 按序发 AXI 写（wr_req），等 B 响应后释放条目
//     （退出 store 的写完成判定用 exit_ack，axi_master 对退出写不回 wr_resp）
//   - load 查询（组合）：扫描所有"更老且仍占用条目"的 store（含已提交未写完 → 保守 stall）：
//       地址未知 → stall；同字完全覆盖且数据已知 → 转发（多候选取最年轻）；部分重叠 → stall
//   - flush：年轻于 flush_rob_id 的条目清空（物理寄存器由 ROB 统一归还，SB 无归还职责）
// 时序：alloc/fill/commit/flush/wr_resp 于 cycle N → 边沿 N+1；query 为纯组合。
module store_buffer #(
    parameter int SB_DEPTH = 8,
    parameter int ROB_ID_W = 4
) (
    input  logic clk,
    input  logic reset,

    // ---- 分配（store 派发时）----
    input  logic                alloc_valid,
    output logic                alloc_ready,
    input  logic [ROB_ID_W-1:0] alloc_rob_id,

    // ---- 执行回填（LSU 执行 store 时，addr/data/wstrb 同拍有效）----
    input  logic                fill_valid,
    input  logic [ROB_ID_W-1:0] fill_rob_id,
    input  logic [31:0]         fill_addr,
    input  logic [31:0]         fill_data,
    input  logic [3:0]          fill_wstrb,

    // ---- 提交（ROB 提交 store 时）----
    input  logic                commit_valid,
    input  logic [ROB_ID_W-1:0] commit_rob_id,

    // ---- 写内存（提交后发出，等响应释放）----
    output logic                wr_req_valid,
    input  logic                wr_req_ready,
    output logic [31:0]         wr_req_addr,
    output logic [31:0]         wr_req_data,
    output logic [3:0]          wr_req_strb,
    input  logic                wr_resp_valid,    // 普通 store 的 B 响应
    input  logic                exit_ack,         // 退出 store 的 B 握手（axi_master）

    // ---- load 查询（组合）----
    input  logic                query_valid,
    input  logic [ROB_ID_W-1:0] query_rob_id,
    input  logic [31:0]         query_addr,
    input  logic [1:0]          query_size,       // 0=byte 1=half 2=word
    output logic                query_forward,    // 完全覆盖：转发数据（右对齐）
    output logic [31:0]         query_data,
    output logic                query_stall,      // 地址未知 / 部分重叠 / 数据未知 / 更老已提交未写完

    // ---- flush ----
    input  logic                    flush_valid,
    input  logic [ROB_ID_W-1:0]     flush_rob_id,
    // ROB head / tail（环序年龄比较基准；tail 界定在飞窗口，排除已提交未写完成的条目）
    input  logic [ROB_ID_W-1:0]     head_rob_id,
    input  logic [ROB_ID_W-1:0]     tail_rob_id
);

    // ================= 条目 =================
    logic                    valid     [0:SB_DEPTH-1];
    logic [ROB_ID_W-1:0]     rob_id    [0:SB_DEPTH-1];
    logic                    committed [0:SB_DEPTH-1];
    logic                    addr_valid[0:SB_DEPTH-1];
    logic                    data_valid[0:SB_DEPTH-1];
    logic [31:0]             addr      [0:SB_DEPTH-1];
    logic [31:0]             data      [0:SB_DEPTH-1];
    logic [3:0]              wstrb     [0:SB_DEPTH-1];
    logic                    wr_sent   [0:SB_DEPTH-1];
    logic                    is_exit   [0:SB_DEPTH-1];

    logic [ROB_ID_W-1:0] head, tail;

    logic full;
    assign full = valid[(tail + ROB_ID_W'(1)) % SB_DEPTH];
    assign alloc_ready = !reset && !full;

    // ---- 写发出（head 条目 committed 且未发）----
    assign wr_req_valid = !reset && valid[head] && committed[head] && !wr_sent[head];
    assign wr_req_addr  = addr[head];
    assign wr_req_data  = data[head];
    assign wr_req_strb  = wstrb[head];

`ifndef SYNTHESIS
`endif

    // ================= 环序（基于 head 的年龄 + tail 窗口）=================
    logic [ROB_ID_W-1:0] sb_flush_age;
    logic [ROB_ID_W-1:0] sb_tail_age;
    assign sb_flush_age = flush_rob_id - head_rob_id;
    assign sb_tail_age  = tail_rob_id - head_rob_id;

    // ================= load 查询（组合）=================
    logic [3:0] ld_byte_mask;
    always_comb begin
        unique case (query_size)
        2'd0: ld_byte_mask = 4'h1 << query_addr[1:0];
        2'd1: ld_byte_mask = query_addr[1] ? 4'hC : 4'h3;
        default: ld_byte_mask = 4'hF;
        endcase
    end

    always_comb begin
        query_forward = 1'b0;
        query_data    = '0;
        query_stall   = 1'b0;
        if (query_valid) begin
            logic [31:0] best_data;
            logic        found_forward;
            best_data = '0;
            found_forward = 1'b0;
            // 从老（head）到新扫描；更年轻的完全覆盖会覆盖 best_data
            for (int i = 0; i < SB_DEPTH; i++) begin
                logic [ROB_ID_W-1:0] e_age, q_age;
                e_age = rob_id[i] - head_rob_id;
                q_age = query_rob_id - head_rob_id;
                if (valid[i] && committed[i]) begin
                    query_stall = 1'b1;      // 已提交未释放：必比 load 老且内存可能未更新
                end else if (valid[i] && (e_age < q_age)) begin
                    if (committed[i]) begin
                        query_stall = 1'b1;      // 已提交但写未完成（内存可能未更新）
                    end else if (!addr_valid[i]) begin
                        query_stall = 1'b1;      // 地址未知 → 保守 stall
                    end else if (((addr[i] & ~32'h3) == (query_addr & ~32'h3)) &&
                                 ((ld_byte_mask & wstrb[i]) == ld_byte_mask)) begin
                        if (data_valid[i]) begin
                            found_forward = 1'b1;   // 完全覆盖 → 转发（最年轻者胜）
                            unique case (query_size)
                            2'd0: best_data = {24'd0, data[i] >> (8 * query_addr[1:0])};
                            2'd1: best_data = {16'd0, query_addr[1] ? data[i][31:16]
                                                                   : data[i][15:0]};
                            default: best_data = data[i];
                            endcase
                        end else begin
                            query_stall = 1'b1;  // 覆盖但数据未知
                        end
                    end else if (((addr[i] & ~32'h3) == (query_addr & ~32'h3)) &&
                                 ((ld_byte_mask & wstrb[i]) != 4'd0)) begin
                        query_stall = 1'b1;      // 部分重叠 → 保守 stall（v1 不支持多 store 合并）
                    end
                end
            end
            if (found_forward && !query_stall) begin
                query_forward = 1'b1;
                query_data    = best_data;
            end
        end
    end

    // ================= 条目更新（单块）=================
    always_ff @(posedge clk) begin
        if (reset) begin
            for (int i = 0; i < SB_DEPTH; i++) begin
                valid[i]      <= 1'b0;
                committed[i]  <= 1'b0;
                addr_valid[i] <= 1'b0;
                data_valid[i] <= 1'b0;
                wr_sent[i]    <= 1'b0;
            end
            head <= '0;
            tail <= '0;
        end else begin
            // 1) 释放：head 条目写完成
            if (valid[head] && committed[head] && wr_sent[head] &&
                (is_exit[head] ? exit_ack : wr_resp_valid)) begin
                valid[head] <= 1'b0;
                head <= head + ROB_ID_W'(1);
            end
            // 2) 写发出：head committed 且未发 → 握手 fire
            if (wr_req_valid && wr_req_ready) begin
                wr_sent[head] <= 1'b1;
            end
            // 3) 分配
            if (alloc_valid && alloc_ready) begin
                valid[tail]      <= 1'b1;
                committed[tail]  <= 1'b0;
                addr_valid[tail] <= 1'b0;
                data_valid[tail] <= 1'b0;
                wr_sent[tail]    <= 1'b0;
                rob_id[tail]     <= alloc_rob_id;
                tail <= tail + ROB_ID_W'(1);
            end
            // 4) 执行回填
            if (fill_valid) begin
                for (int i = 0; i < SB_DEPTH; i++) begin
                    if (valid[i] && (rob_id[i] == fill_rob_id)) begin
                        addr[i]       <= fill_addr;
                        data[i]       <= fill_data;
                        wstrb[i]      <= fill_wstrb;
                        addr_valid[i] <= 1'b1;
                        data_valid[i] <= 1'b1;
                        is_exit[i]    <= (fill_addr == 32'h80000000);
                    end
                end
            end
            // 5) 提交
            if (commit_valid) begin
                for (int i = 0; i < SB_DEPTH; i++) begin
                    if (valid[i] && (rob_id[i] == commit_rob_id)) begin
                        committed[i] <= 1'b1;
                    end
                end
            end
            // 6) flush：清年轻条目（窗口内：age_f < entry_age < tail_age）+ tail 回退
            if (flush_valid) begin
                logic [$clog2(SB_DEPTH)-1:0] cnt;
                cnt = '0;
                for (int i = 0; i < SB_DEPTH; i++) begin
                    logic [ROB_ID_W-1:0] e_age;
                    e_age = rob_id[i] - head_rob_id;
                    if (valid[i] && (e_age > sb_flush_age) && (e_age < sb_tail_age)) begin
                        valid[i] <= 1'b0;
                        cnt = cnt + $clog2(SB_DEPTH)'(1);
                    end
                end
                tail <= tail - ROB_ID_W'(cnt);
                if (valid[head] && ((rob_id[head] - head_rob_id) > sb_flush_age) &&
                    ((rob_id[head] - head_rob_id) < sb_tail_age)) begin
                    head <= tail - ROB_ID_W'(cnt);
                end
            end
        end
    end

endmodule
