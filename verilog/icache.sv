// icache: 只读 L1 指令缓存（直接映射、参数化行数/行长）
// 查询：请求在 cycle T 采样，命中/数据在 cycle T+1 组合给出（SRAM 1 周期读延迟）
// 缺失：单 MSHR 重填，按字顺序经统一读端口回填整行（最多 MAXOUT 个在飞读）
module icache #(
    parameter int LINES      = 64,
    parameter int LINE_WORDS = 8,
    parameter int MAXOUT     = 4
) (
    input  logic        clk,
    input  logic        reset,

    // ---- fetch 查询（组合请求，下一拍返回 hit/data）----
    input  logic [31:0] req_pc,
    input  logic        req_valid,
    output logic        req_ready,     // 当前可接收新查询（非重填中）
    output logic        hit,           // 上一拍查询命中
    output logic [31:0] hit_data,

    // ---- 重填读端口（统一走 top 的读仲裁 + ro_q 路由）----
    output logic        rd_req,
    input  logic        rd_ready,
    output logic [31:0] rd_addr,
    input  logic        resp_valid,    // 本 cache 的重填响应
    input  logic [31:0] resp_data
);

    localparam int OFF_W = $clog2(LINE_WORDS);         // 行内字偏移
    localparam int IDX_W = $clog2(LINES);
    localparam int TAG_W = 32 - OFF_W - IDX_W - 2;
    localparam int LINEW  = LINE_WORDS * 4;

    logic [LINES-1:0] valid;

    // ---- 查询流水 ----
    logic             lk_a;                  // 上一拍发生了查询
    logic [TAG_W-1:0] lk_tag;
    logic [IDX_W-1:0] lk_idx;

    // ---- 重填 ----
    logic             busy;
    logic [31:0]      r_base;
    logic [TAG_W-1:0] r_tag;
    logic [IDX_W-1:0] r_idx;
    logic [OFF_W:0]   r_issue;
    logic [OFF_W:0]   r_done;

    // 查询地址分解
    logic [IDX_W-1:0] req_idx;
    logic [OFF_W-1:0] req_word;
    logic [TAG_W-1:0] req_tag;
    assign req_idx  = req_pc[OFF_W+2+IDX_W-1 : OFF_W+2];
    assign req_word = req_pc[OFF_W+1 : 2];
    assign req_tag  = req_pc[31 : OFF_W+2+IDX_W];

    wire lk_req = req_valid && req_ready;
    assign req_ready = !reset && !busy;

    // 数组访问：空闲拍查询读；重填响应拍数据写
    logic arr_en, arr_we;
    logic [IDX_W+OFF_W-1:0] arr_addr;
    logic [31:0] arr_wdata;
    assign arr_en    = lk_req || resp_valid;
    assign arr_we    = resp_valid;
    assign arr_addr  = lk_req ? {req_idx, req_word} : {r_idx, r_done[OFF_W-1:0]};
    assign arr_wdata = resp_data;

    wire rd_fire   = busy && (r_issue < (OFF_W+1)'(LINE_WORDS)) &&
                     ((r_issue - r_done) < (OFF_W+1)'(MAXOUT));
    assign rd_req  = rd_fire;
    assign rd_addr = r_base + {r_issue, 2'b00};

    // tag 阵列（独立实例，与数据阵列并行读；重填最后响应拍写回）
    logic tag_en, tag_we;
    logic [IDX_W-1:0] tag_addr;
    logic [TAG_W-1:0] tag_wdata, tag_rdata;
    assign tag_en    = lk_req || resp_valid;
    assign tag_we    = resp_valid && (r_done == (OFF_W+1)'(LINE_WORDS-1));
    assign tag_addr  = lk_req ? req_idx : r_idx;
    assign tag_wdata = r_tag;

    logic [31:0] data_rdata;

    sram_fakeram #(.DEPTH(LINES*LINE_WORDS), .WIDTH(32)) u_data (
        .clk(clk), .en(arr_en), .we(arr_we),
        .wmask(1'b1), .addr(arr_addr), .wdata(arr_wdata), .rdata(data_rdata)
    );
    sram_fakeram #(.DEPTH(LINES), .WIDTH(TAG_W)) u_tag (
        .clk(clk), .en(tag_en), .we(tag_we),
        .wmask(1'b1), .addr(tag_addr), .wdata(tag_wdata), .rdata(tag_rdata)
    );

    // 命中判定（查询后一拍）
    assign hit      = lk_a && valid[lk_idx] && (tag_rdata == lk_tag);
    assign hit_data = data_rdata;

    always_ff @(posedge clk) begin
        if (reset) begin
            valid <= '0;
            lk_a  <= 1'b0;
            busy  <= 1'b0;
            r_issue <= '0;
            r_done  <= '0;
        end else begin
            lk_a <= lk_req;
            if (lk_req) begin
                lk_tag <= req_tag;
                lk_idx <= req_idx;
            end
            // 缺失 → 启动重填
            if (lk_a && !hit && !busy) begin
                busy    <= 1'b1;
                r_base  <= {req_pc_hold[31:OFF_W+2], {(OFF_W+2){1'b0}}};
                r_tag   <= lk_tag;
                r_idx   <= lk_idx;
                r_issue <= '0;
                r_done  <= '0;
            end
            // 重填请求握手
            if (rd_fire && rd_ready) r_issue <= r_issue + (OFF_W+1)'(1);
            // 重填响应：写数据阵列；最后字写 tag 并置 valid，结束重填
            if (resp_valid) begin
                if (r_done == (OFF_W+1)'(LINE_WORDS-1)) begin
                    valid[r_idx] <= 1'b1;
                    busy <= 1'b0;
                    r_done <= '0;
                end else begin
                    r_done <= r_done + (OFF_W+1)'(1);
                end
            end
        end
    end

    // 重填基准需要被查询地址的完整值：查询时锁存 pc
    logic [31:0] req_pc_hold;
    always_ff @(posedge clk) begin
        if (lk_req) req_pc_hold <= req_pc;
    end

endmodule
