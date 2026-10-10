// 临时 test_top（模块名 student_top）：store_buffer 模块自检
// 断言：同字/字节/半字完全覆盖转发、部分重叠 stall、地址未知 stall、
//       更年轻 store 不挡更老 load、提交后才发写、committed 未写完阻塞 load、
//       写完成释放、flush 清年轻条目
// query 为纯组合：seq N 驱动 → cycle N 输出 → seq N+1 检查（与上一个 query 同 seq）
// 退出值 = 错误计数（EXPECTED=0 即全过）
module student_top import pkg_cpu2026::*; (
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

    // ================= axi_master（仅退出写）=================
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

    assign rd_req_valid  = 1'b0;
    assign rd_req_addr   = 32'd0;
    assign rd_resp_ready = 1'b1;
    assign wr_resp_ready = 1'b1;

    axi_master #(.DBG(0)) u_master (
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

    // ================= 被测模块 =================
    logic                sb_alloc_valid, sb_alloc_ready;
    logic [3:0]          sb_alloc_rob_id;
    logic                sb_fill_valid;
    logic [3:0]          sb_fill_rob_id;
    logic [31:0]         sb_fill_addr, sb_fill_data;
    logic [3:0]          sb_fill_wstrb;
    logic                sb_commit_valid;
    logic [3:0]          sb_commit_rob_id;
    logic                sb_wr_req_valid, sb_wr_req_ready;
    logic [31:0]         sb_wr_addr, sb_wr_data;
    logic [3:0]          sb_wr_strb;
    logic                sb_wr_resp_valid, sb_exit_ack;
    logic                q_valid;
    logic [3:0]          q_rob_id;
    logic [31:0]         q_addr;
    logic [1:0]          q_size;
    logic                q_forward, q_stall;
    logic [31:0]         q_data;
    logic                sb_flush_valid;
    logic [3:0]          sb_flush_rob_id;

    store_buffer #(.SB_DEPTH(8), .ROB_ID_W(4)) u_sb (
        .clk(clock), .reset(reset),
        .alloc_valid(sb_alloc_valid),
        .alloc_ready(sb_alloc_ready),
        .alloc_rob_id(sb_alloc_rob_id),
        .fill_valid(sb_fill_valid),
        .fill_rob_id(sb_fill_rob_id),
        .fill_addr(sb_fill_addr),
        .fill_data(sb_fill_data),
        .fill_wstrb(sb_fill_wstrb),
        .commit_valid(sb_commit_valid),
        .commit_rob_id(sb_commit_rob_id),
        .wr_req_valid(sb_wr_req_valid),
        .wr_req_ready(sb_wr_req_ready),
        .wr_req_addr(sb_wr_addr),
        .wr_req_data(sb_wr_data),
        .wr_req_strb(sb_wr_strb),
        .wr_resp_valid(sb_wr_resp_valid),
        .exit_ack(sb_exit_ack),
        .query_valid(q_valid),
        .query_rob_id(q_rob_id),
        .query_addr(q_addr),
        .query_size(q_size),
        .query_forward(q_forward),
        .query_data(q_data),
        .query_stall(q_stall),
        .flush_valid(sb_flush_valid),
        .flush_rob_id(sb_flush_rob_id)
    );

    assign sb_wr_req_ready = 1'b1;   // 模拟 axi_master 恒 ready
    assign sb_exit_ack     = 1'b0;   // 测试不覆盖退出 store

    // ================= 自检序列 =================
    integer err_cnt;

    task automatic check_ok(input string name, input logic cond);
        if (!cond) begin
            $display("FAIL [%s]", name);
            err_cnt = err_cnt + 1;
        end
    endtask

    logic [4:0] seq;

    always_ff @(posedge clock) begin
        if (reset) begin
            seq <= 5'd0;
        end else begin
            // 默认驱动
            sb_alloc_valid <= 1'b0;
            sb_alloc_rob_id <= 4'd0;
            sb_fill_valid <= 1'b0;
            sb_fill_rob_id <= 4'd0;
            sb_fill_addr <= 32'd0; sb_fill_data <= 32'd0; sb_fill_wstrb <= 4'd0;
            sb_commit_valid <= 1'b0;
            sb_commit_rob_id <= 4'd0;
            sb_wr_resp_valid <= 1'b0;
            sb_flush_valid <= 1'b0;
            sb_flush_rob_id <= 4'd0;
            q_valid <= 1'b0;
            q_rob_id <= 4'd0;
            q_addr <= 32'd0;
            q_size <= 2'd2;

            if (seq < 5'd30) begin
                case (seq)
                0: begin end
                1: begin end
                // ---- 同字转发基础 ----
                2:  begin sb_alloc_valid <= 1'b1; sb_alloc_rob_id <= 4'd0; end
                3:  begin sb_fill_valid <= 1'b1; sb_fill_rob_id <= 4'd0;
                         sb_fill_addr <= 32'h1000; sb_fill_data <= 32'h11223344; sb_fill_wstrb <= 4'hF; end
                4:  begin sb_alloc_valid <= 1'b1; sb_alloc_rob_id <= 4'd1; end
                5:  begin sb_fill_valid <= 1'b1; sb_fill_rob_id <= 4'd1;
                         sb_fill_addr <= 32'h2000; sb_fill_data <= 32'hDEADBEEF; sb_fill_wstrb <= 4'hF; end
                6:  begin q_valid <= 1'b1; q_rob_id <= 4'd2; q_addr <= 32'h1000; q_size <= 2'd2; end
                7:  begin check_ok("Q1 word forward", q_forward && !q_stall && q_data==32'h11223344);
                         sb_alloc_valid <= 1'b1; sb_alloc_rob_id <= 4'd3; end
                8:  begin sb_fill_valid <= 1'b1; sb_fill_rob_id <= 4'd3;
                         sb_fill_addr <= 32'h1004; sb_fill_data <= 32'h0000AA00; sb_fill_wstrb <= 4'b0010; end
                9:  begin q_valid <= 1'b1; q_rob_id <= 4'd4; q_addr <= 32'h1005; q_size <= 2'd0; end
                10: begin check_ok("Q2 byte forward", q_forward && !q_stall && q_data==32'hAA);
                         q_valid <= 1'b1; q_rob_id <= 4'd4; q_addr <= 32'h1006; q_size <= 2'd0; end
                11: begin check_ok("Q3 byte no overlap", !q_forward && !q_stall);
                         sb_alloc_valid <= 1'b1; sb_alloc_rob_id <= 4'd5; end
                12: begin sb_fill_valid <= 1'b1; sb_fill_rob_id <= 4'd5;
                         sb_fill_addr <= 32'h1008; sb_fill_data <= 32'hBBBB0000; sb_fill_wstrb <= 4'b1100; end
                13: begin q_valid <= 1'b1; q_rob_id <= 4'd6; q_addr <= 32'h100A; q_size <= 2'd1; end
                14: begin check_ok("Q4 half forward", q_forward && !q_stall && q_data==32'hBBBB);
                         sb_alloc_valid <= 1'b1; sb_alloc_rob_id <= 4'd7; end
                15: begin sb_fill_valid <= 1'b1; sb_fill_rob_id <= 4'd7;
                         sb_fill_addr <= 32'h1010; sb_fill_data <= 32'h11111111; sb_fill_wstrb <= 4'b0001; end
                16: begin q_valid <= 1'b1; q_rob_id <= 4'd8; q_addr <= 32'h1010; q_size <= 2'd2; end
                17: begin check_ok("Q5 partial stall", q_stall && !q_forward);
                         sb_alloc_valid <= 1'b1; sb_alloc_rob_id <= 4'd9; end   // 不 fill：地址未知
                18: begin q_valid <= 1'b1; q_rob_id <= 4'd10; q_addr <= 32'h2000; q_size <= 2'd2; end
                19: begin check_ok("Q6 addr unknown stall", q_stall && !q_forward);
                         q_valid <= 1'b1; q_rob_id <= 4'd2; q_addr <= 32'h1010; q_size <= 2'd2; end
                20: begin check_ok("Q7 younger not blocking", !q_stall && !q_forward);
                         sb_commit_valid <= 1'b1; sb_commit_rob_id <= 4'd0; end
                21: begin end
                22: begin check_ok("W1 store write issued", sb_wr_req_valid &&
                                   sb_wr_addr==32'h1000 && sb_wr_data==32'h11223344 && sb_wr_strb==4'hF);
                         q_valid <= 1'b1; q_rob_id <= 4'd2; q_addr <= 32'h1000; q_size <= 2'd2; end
                23: begin check_ok("Q8 committed blocks load", q_stall && !q_forward);
                         sb_wr_resp_valid <= 1'b1; end
                24: begin end   // 边沿 24 释放 rob0
                25: begin q_valid <= 1'b1; q_rob_id <= 4'd2; q_addr <= 32'h1000; q_size <= 2'd2; end
                26: begin check_ok("Q9 released no stall", !q_stall && !q_forward);
                         sb_flush_valid <= 1'b1; sb_flush_rob_id <= 4'd5; end  // 清 rob7、rob9
                27: begin q_valid <= 1'b1; q_rob_id <= 4'd10; q_addr <= 32'h1010; q_size <= 2'd2; end
                28: begin check_ok("Q10 flushed no stall", !q_stall && !q_forward); end
                29: begin end
                default: ;
                endcase
                seq <= seq + 5'd1;
            end
        end
    end

    // ---- 退出写 ----
    logic exit_go, exit_sent, summary_done;
    always_ff @(posedge clock) begin
        if (reset) begin
            exit_go <= 1'b0; exit_sent <= 1'b0; summary_done <= 1'b0;
        end else begin
            if (seq == 5'd30 && !exit_go) exit_go <= 1'b1;
            if (wr_req_valid && wr_req_ready) exit_sent <= 1'b1;
            if (exit_ack && !summary_done) begin
                summary_done <= 1'b1;
                if (err_cnt == 0) $display("=== SB SELF-TEST: ALL PASS ===");
                else              $display("=== SB SELF-TEST: %0d FAIL ===", err_cnt);
            end
        end
    end

    assign wr_req_valid = !reset && exit_go && !exit_sent;
    assign wr_req_addr  = 32'h80000000;
    assign wr_req_data  = err_cnt;
    assign wr_req_strb  = 4'hf;

endmodule
