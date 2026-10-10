// axi_master: CPU 内部请求 <-> AXI4-Lite 五通道转换
// 接口定义见 docs/iface.md v1.0（axi_master 一节）
// 设计要点：
//   1) 每通道内部 FIFO（深度=在飞上限），rd_req_ready/wr_req_ready 仅由内部计数决定
//   2) arvalid/awvalid/wvalid 只由队列非空决定，不组合依赖任何 ready（防死锁）
//   3) rready/bready 恒 1（非复位）：在飞数 <= 4 < 从机 16 项队列，响应必能收
//   4) AW 与 W 保持同拍拉高（锁步），避免"W 先于 AW"的等待窗口
//   5) 退出写（0x80000000 + wstrb=4'hf）在 B 握手当拍置 exit_ack
module axi_master #(
    parameter int RD_QUEUE_DEPTH = 4,   // 在飞读请求上限（< 从机 16 项队列）
    parameter int WR_QUEUE_DEPTH = 4,   // 在飞写请求上限
    parameter int DBG            = 1    // $display 开关（综合时被 SYNTHESIS 宏剔除）
) (
    input  logic clk,
    input  logic reset,

    // ---- CPU 侧：读 ----
    input  logic        rd_req_valid,
    output logic        rd_req_ready,
    input  logic [31:0] rd_req_addr,     // 必须 4 字节对齐
    output logic        rd_resp_valid,
    input  logic        rd_resp_ready,
    output logic [31:0] rd_resp_data,
    output logic [1:0]  rd_resp_err,     // 2'b00=OKAY 2'b10=SLVERR 2'b11=DECERR

    // ---- CPU 侧：写 ----
    input  logic        wr_req_valid,
    output logic        wr_req_ready,
    input  logic [31:0] wr_req_addr,     // 必须 4 字节对齐
    input  logic [31:0] wr_req_data,
    input  logic [3:0]  wr_req_strb,
    output logic        wr_resp_valid,
    input  logic        wr_resp_ready,
    output logic [1:0]  wr_resp_err,

    // ---- 退出协议 ----
    output logic        exit_ack,        // 退出 Store 的 B 握手完成脉冲

    // ---- AXI4-Lite 侧（与框架 student_top 模板逐位一致）----
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

    localparam int RD_PTR_W = RD_QUEUE_DEPTH > 1 ? $clog2(RD_QUEUE_DEPTH) : 1;
    localparam int RD_CNT_W = $clog2(RD_QUEUE_DEPTH + 1);
    localparam int WR_PTR_W = WR_QUEUE_DEPTH > 1 ? $clog2(WR_QUEUE_DEPTH) : 1;
    localparam int WR_CNT_W = $clog2(WR_QUEUE_DEPTH + 1);
    localparam logic [RD_CNT_W-1:0] RD_DEPTH_L = RD_CNT_W'(RD_QUEUE_DEPTH);
    localparam logic [WR_CNT_W-1:0] WR_DEPTH_L = WR_CNT_W'(WR_QUEUE_DEPTH);

    // ================= 读通路 =================
    // ar_q: 待发 AR 的地址队列
    logic [31:0]      ar_q  [0:RD_QUEUE_DEPTH-1];
    logic [RD_PTR_W-1:0] ar_wptr, ar_rptr;
    logic [RD_CNT_W-1:0] ar_cnt;
    // rr_q: 已收 R 响应待 CPU 消费
    logic [31:0]      rr_data [0:RD_QUEUE_DEPTH-1];
    logic [1:0]       rr_err  [0:RD_QUEUE_DEPTH-1];
    logic [RD_PTR_W-1:0] rr_wptr, rr_rptr;
    logic [RD_CNT_W-1:0] rr_cnt;

    assign rd_req_ready = !reset && (ar_cnt < RD_DEPTH_L);
    assign arvalid      = !reset && (ar_cnt != '0);
    assign araddr       = ar_q[ar_rptr];
    assign rready       = !reset;                       // 恒收：ar_cnt <= DEPTH < 16
    assign rd_resp_valid = !reset && (rr_cnt != '0);
    assign rd_resp_data  = rr_data[rr_rptr];
    assign rd_resp_err   = rr_err[rr_rptr];

    always_ff @(posedge clk) begin
        if (reset) begin
            ar_wptr <= '0; ar_rptr <= '0; ar_cnt <= '0;
        end else begin
            if (rd_req_valid && rd_req_ready) begin
                ar_q[ar_wptr] <= rd_req_addr;
                ar_wptr <= ar_wptr + 1'b1;
            end
            if (arvalid && arready) begin
                ar_rptr <= ar_rptr + 1'b1;
            end
            ar_cnt <= ar_cnt + (RD_CNT_W)'(rd_req_valid && rd_req_ready)
                              - (RD_CNT_W)'(arvalid && arready);
        end
    end

    always_ff @(posedge clk) begin
        if (reset) begin
            rr_wptr <= '0; rr_rptr <= '0; rr_cnt <= '0;
        end else begin
            if (rvalid && rready) begin
                rr_data[rr_wptr] <= rdata;
                rr_err[rr_wptr]  <= rresp;
                rr_wptr <= rr_wptr + 1'b1;
            end
            if (rd_resp_valid && rd_resp_ready) begin
                rr_rptr <= rr_rptr + 1'b1;
            end
            rr_cnt <= rr_cnt + (RD_CNT_W)'(rvalid && rready)
                              - (RD_CNT_W)'(rd_resp_valid && rd_resp_ready);
        end
    end

    // ================= 写通路 =================
    // wq: AW 与 W 锁步共用的单队列（地址+数据+掩码+退出标志）
    logic [31:0]      wq_addr [0:WR_QUEUE_DEPTH-1];
    logic [31:0]      wq_data [0:WR_QUEUE_DEPTH-1];
    logic [3:0]       wq_strb [0:WR_QUEUE_DEPTH-1];
    logic             wq_exit [0:WR_QUEUE_DEPTH-1];
    logic [WR_PTR_W-1:0] wq_wptr, wq_rptr;
    logic [WR_CNT_W-1:0] wq_cnt;
    // bq_exit: 与在飞写一一对应的退出标志队列（B 响应按序返回）
    logic             bq_exit [0:WR_QUEUE_DEPTH-1];
    logic [WR_PTR_W-1:0] bq_wptr, bq_rptr;
    logic [WR_CNT_W-1:0] bq_cnt;
    // br_q: 普通写响应待 CPU 消费
    logic [1:0]       br_err  [0:WR_QUEUE_DEPTH-1];
    logic [WR_PTR_W-1:0] br_wptr, br_rptr;
    logic [WR_CNT_W-1:0] br_cnt;

    assign wr_req_ready = !reset && (wq_cnt < WR_DEPTH_L);
    assign awvalid      = !reset && (wq_cnt != '0);
    assign wvalid       = awvalid;                      // AW/W 锁步
    assign awaddr       = wq_addr[wq_rptr];
    assign wdata        = wq_data[wq_rptr];
    assign wstrb        = wq_strb[wq_rptr];
    assign bready       = !reset;                       // 恒收
    assign wr_resp_valid = !reset && (br_cnt != '0);
    assign wr_resp_err   = br_err[br_rptr];
    // 退出写（0x80000000 + wstrb=4'hf）的 B 握手当拍置位（组合输出，与从机捕获结果同拍）
    assign exit_ack      = !reset && bvalid && bready && bq_exit[bq_rptr];

    always_ff @(posedge clk) begin
        if (reset) begin
            wq_wptr <= '0; wq_rptr <= '0; wq_cnt <= '0;
        end else begin
            if (wr_req_valid && wr_req_ready) begin
                wq_addr[wq_wptr] <= wr_req_addr;
                wq_data[wq_wptr] <= wr_req_data;
                wq_strb[wq_wptr] <= wr_req_strb;
                wq_exit[wq_wptr] <= (wr_req_addr == 32'h80000000) && (wr_req_strb == 4'hf);
                wq_wptr <= wq_wptr + 1'b1;
            end
            if (awvalid && awready && wvalid && wready) begin
                wq_rptr <= wq_rptr + 1'b1;
            end
            wq_cnt <= wq_cnt + (WR_CNT_W)'(wr_req_valid && wr_req_ready)
                              - (WR_CNT_W)'(awvalid && awready && wvalid && wready);
        end
    end

    always_ff @(posedge clk) begin
        if (reset) begin
            bq_wptr <= '0; bq_rptr <= '0; bq_cnt <= '0;
        end else begin
            if (awvalid && awready && wvalid && wready) begin
                bq_exit[bq_wptr] <= wq_exit[wq_rptr];
                bq_wptr <= bq_wptr + 1'b1;
            end
            if (bvalid && bready) begin
                bq_rptr <= bq_rptr + 1'b1;
            end
            bq_cnt <= bq_cnt + (WR_CNT_W)'(awvalid && awready && wvalid && wready)
                              - (WR_CNT_W)'(bvalid && bready);
        end
    end

    always_ff @(posedge clk) begin
        if (reset) begin
            br_wptr <= '0; br_rptr <= '0; br_cnt <= '0;
        end else begin
            if (bvalid && bready && !bq_exit[bq_rptr]) begin
                br_err[br_wptr] <= bresp;
                br_wptr <= br_wptr + 1'b1;
            end
            if (wr_resp_valid && wr_resp_ready) begin
                br_rptr <= br_rptr + 1'b1;
            end
            br_cnt <= br_cnt + (WR_CNT_W)'(bvalid && bready && !bq_exit[bq_rptr])
                              - (WR_CNT_W)'(wr_resp_valid && wr_resp_ready);
        end
    end

`ifndef SYNTHESIS
    always @(posedge clk) begin
        if (DBG != 0 && !reset) begin
            if (arvalid && arready)
                $display("[axi_master] AR fire addr=%08h", araddr);
            if (rvalid && rready)
                $display("[axi_master] R  fire data=%08h resp=%b", rdata, rresp);
            if (awvalid && awready)
                $display("[axi_master] AW fire addr=%08h", awaddr);
            if (wvalid && wready)
                $display("[axi_master] W  fire data=%08h strb=%b", wdata, wstrb);
            if (bvalid && bready)
                $display("[axi_master] B  fire resp=%b", bresp);
            if (exit_ack)
                $display("[axi_master] EXIT ACK");
        end
    end
`endif

endmodule
