// dcache: L1 数据缓存（直接映射、写回、no-write-allocate）
// - load：查询 1 拍（ld_ready 握手 → 下一拍 ld_hit/ld_data 组合有效）；缺失自动整行重填
// - store（来自 SB 提交）：命中 → 字节掩码写数组并立即完成；缺失 → 转 AXI 写（不分配）
// - 换出：重填占用有效行时，先整行写回（LINE_WORDS 字），等全部 B 响应后再重填
// - 写单元：store 缺失 / exit / 换出字 共用 AXI 写口；写标签 FIFO 区分 B 归属
module dcache #(
    parameter int LINES      = 64,
    parameter int LINE_WORDS = 8,
    parameter int MAXOUT     = 4
) (
    input  logic        clk,
    input  logic        reset,

    // ---- load 端口（LSU）----
    input  logic        ld_req,
    input  logic [31:0] ld_addr,
    output logic        ld_ready,
    output logic        ld_hit,
    output logic [31:0] ld_data,

    // ---- store 端口（SB 头）----
    input  logic        st_req,
    input  logic [31:0] st_addr, st_data,
    input  logic [3:0]  st_strb,
    input  logic        st_is_exit,
    output logic        st_accept,
    output logic        st_cache_done,
    output logic        st_b_done,

    // ---- 重填读端口 ----
    output logic        rd_req,
    input  logic        rd_ready,
    output logic [31:0] rd_addr,
    input  logic        resp_valid,
    input  logic [31:0] resp_data,

    // ---- AXI 写端口 ----
    output logic        wr_req_valid,
    input  logic        wr_req_ready,
    output logic [31:0] wr_req_addr, wr_req_data,
    output logic [3:0]  wr_req_strb,
    input  logic        wr_resp_valid
);

    localparam int OFF_W = $clog2(LINE_WORDS);
    localparam int IDX_W = $clog2(LINES);
    localparam int TAG_W = 32 - OFF_W - IDX_W - 2;
    localparam int WW    = OFF_W + 1;          // 字计数位宽

    logic [LINES-1:0] valid;

    typedef enum logic [2:0] {D_IDLE, D_LD, D_ST, D_EVRD, D_EVWR, D_REF} state_t;
    state_t state;

    function automatic logic [IDX_W-1:0] get_idx(input logic [31:0] a);
        return a[OFF_W+2+IDX_W-1 : OFF_W+2];
    endfunction
    function automatic logic [OFF_W-1:0] get_word(input logic [31:0] a);
        return a[OFF_W+1 : 2];
    endfunction
    function automatic logic [TAG_W-1:0] get_tag(input logic [31:0] a);
        return a[31 : OFF_W+2+IDX_W];
    endfunction

    // 被接收查询（D_LD/D_ST 判定用）
    logic [31:0]      q_addr;
    logic [IDX_W-1:0] q_idx;
    logic             q_is_st;

    // store 锁存 / 待转 AXI 项
    logic [31:0] st_q_addr, st_q_data;
    logic [3:0]  st_q_strb;
    logic        st_to_axi, st_axi_exit;
    logic [31:0] st_axi_addr, st_axi_data;
    logic [3:0]  st_axi_strb;

    // 重填
    logic             refill_busy;
    logic [31:0]      r_base;
    logic [TAG_W-1:0] r_tag;
    logic [IDX_W-1:0] r_idx;
    logic [WW-1:0]    r_issue, r_done;

    // 换出
    logic             ev_busy;
    logic [TAG_W-1:0] ev_tag;
    logic [IDX_W-1:0] ev_idx;
    logic [WW-1:0]    ev_i, ev_b;

    // 写单元
    typedef enum logic [1:0] {W_IDLE, W_REQ, W_WAIT} wstate_t;
    wstate_t     wstate;
    logic [31:0] w_addr, w_data;
    logic [3:0]  w_strb;
    logic        w_is_store, w_is_exit;

    // 写标签 FIFO（1=store，0=evict）
    logic [3:0] wtag [0:3];
    logic [1:0] wtag_w, wtag_r;
    logic [2:0] wtag_cnt;

    // ---- tag 阵列 ----
    logic tag_en, tag_we;
    logic [IDX_W-1:0] tag_addr;
    logic [TAG_W-1:0] tag_wdata, tag_rdata;
    wire refill_last = resp_valid && (r_done == WW'(LINE_WORDS-1));
    assign tag_en   = ((state == D_IDLE) && (ld_req || st_req)) || refill_last;
    assign tag_we   = refill_last;
    assign tag_addr = ((state == D_IDLE) && ld_req) ? get_idx(ld_addr) :
                      ((state == D_IDLE) && st_req) ? get_idx(st_addr) : r_idx;
    assign tag_wdata = r_tag;

    // ---- 数据阵列访问（每拍一次）----
    logic arr_en, arr_we;
    logic [IDX_W+OFF_W-1:0] arr_addr;
    logic [31:0] arr_wdata;
    logic [3:0]  arr_wmask;
    logic [31:0] data_rdata;

    wire st_hit_c = (state == D_ST) && valid[q_idx] && (tag_rdata == get_tag(q_addr));

    always_comb begin
        arr_en    = 1'b0;
        arr_we    = 1'b0;
        arr_addr  = '0;
        arr_wdata = '0;
        arr_wmask = '0;
        if ((state == D_IDLE) && ld_req) begin
            arr_en   = 1'b1;
            arr_addr = {get_idx(ld_addr), get_word(ld_addr)};
        end else if (st_hit_c) begin
            arr_en    = 1'b1;
            arr_we    = 1'b1;
            arr_addr  = {get_idx(st_q_addr), get_word(st_q_addr)};
            arr_wdata = st_q_data;
            arr_wmask = st_q_strb;
        end else if ((state == D_EVRD)) begin
            arr_en   = 1'b1;
            arr_addr = {ev_idx, ev_i[OFF_W-1:0]};
        end else if ((state == D_REF) && resp_valid) begin
            arr_en    = 1'b1;
            arr_we    = 1'b1;
            arr_addr  = {r_idx, r_done[OFF_W-1:0]};
            arr_wdata = resp_data;
            arr_wmask = 4'hF;
        end
    end

    sram_fakeram #(.DEPTH(LINES*LINE_WORDS), .WIDTH(32), .WRITE_GRANULARITY(8)) u_data (
        .clk(clk), .en(arr_en), .we(arr_we), .wmask(arr_wmask),
        .addr(arr_addr), .wdata(arr_wdata), .rdata(data_rdata)
    );
    sram_fakeram #(.DEPTH(LINES), .WIDTH(TAG_W)) u_tag (
        .clk(clk), .en(tag_en), .we(tag_we), .wmask(1'b1),
        .addr(tag_addr), .wdata(tag_wdata), .rdata(tag_rdata)
    );

    // ---- 对外输出 ----
    assign ld_ready  = !reset && (state == D_IDLE);
    assign ld_hit    = (state == D_LD) && valid[q_idx] && (tag_rdata == get_tag(q_addr));
    assign ld_data   = data_rdata;
    assign st_accept = !reset && (state == D_IDLE) && st_req && !ld_req && !st_to_axi;

    // ---- 重填读请求 ----
    wire rd_fire = (state == D_REF) && (r_issue < WW'(LINE_WORDS)) &&
                   ((r_issue - r_done) < WW'(MAXOUT));
    assign rd_req  = rd_fire;
    assign rd_addr = r_base + {r_issue, 2'b00};

    // ---- 写单元 ----
    assign wr_req_valid = (wstate == W_REQ);
    assign wr_req_addr  = w_addr;
    assign wr_req_data  = w_data;
    assign wr_req_strb  = w_strb;

    wire ev_item_req = (state == D_EVWR) && (ev_i < WW'(LINE_WORDS)) && !st_to_axi;
    wire ev_done     = ev_busy && (ev_b == WW'(LINE_WORDS));

    // ---- 主状态机 + 写单元 + 标签 FIFO ----
    always_ff @(posedge clk) begin
        if (reset) begin
            state <= D_IDLE;
            valid <= '0;
            refill_busy <= 1'b0;
            ev_busy <= 1'b0;
            r_issue <= '0; r_done <= '0;
            ev_i <= '0; ev_b <= '0;
            wstate <= W_IDLE;
            st_to_axi <= 1'b0; st_axi_exit <= 1'b0;
            wtag_w <= '0; wtag_r <= '0; wtag_cnt <= '0;
            st_cache_done <= 1'b0; st_b_done <= 1'b0;
        end else begin
            st_cache_done <= 1'b0;
            st_b_done <= 1'b0;

            // ============ 主状态 ============
            case (state)
            D_IDLE: begin
                if (ld_req) begin
                    q_addr <= ld_addr; q_idx <= get_idx(ld_addr); q_is_st <= 1'b0;
                    state <= D_LD;
                end else if (st_req) begin
                    if (st_is_exit) begin
                        st_to_axi <= 1'b1; st_axi_exit <= 1'b1;
                        st_axi_addr <= st_addr; st_axi_data <= st_data; st_axi_strb <= st_strb;
                    end else begin
                        q_addr <= st_addr; q_idx <= get_idx(st_addr); q_is_st <= 1'b1;
                        st_q_addr <= st_addr; st_q_data <= st_data; st_q_strb <= st_strb;
                        state <= D_ST;
                    end
                end
            end
            D_LD: begin
                if (ld_hit) begin
                    state <= D_IDLE;
                end else begin
                    refill_busy <= 1'b1;
                    r_base <= {q_addr[31:OFF_W+2], {(OFF_W+2){1'b0}}};
                    r_tag  <= get_tag(q_addr);
                    r_idx  <= q_idx;
                    r_issue <= '0; r_done <= '0;
                    if (valid[q_idx]) begin
                        ev_busy <= 1'b1;
                        ev_tag <= tag_rdata; ev_idx <= q_idx;
                        ev_i <= '0; ev_b <= '0;
                        state <= D_EVRD;
                    end else begin
                        state <= D_REF;
                    end
                end
            end
            D_ST: begin
                if (st_hit_c) begin
                    st_cache_done <= 1'b1;
                end else begin
                    st_to_axi <= 1'b1; st_axi_exit <= 1'b0;
                    st_axi_addr <= st_q_addr; st_axi_data <= st_q_data; st_axi_strb <= st_q_strb;
                end
                state <= D_IDLE;
            end
            D_EVRD: begin
                // 本拍发读；下一拍（D_EVWR）data_rdata 即被写回的字
                state <= D_EVWR;
            end
            D_EVWR: begin
                if (ev_done) begin
                    ev_busy <= 1'b0;
                    state <= D_REF;
                end
            end
            D_REF: begin
                if (rd_fire && rd_ready) r_issue <= r_issue + WW'(1);
                if (resp_valid) begin
                    if (r_done == WW'(LINE_WORDS-1)) begin
                        valid[r_idx] <= 1'b1;
                        refill_busy <= 1'b0;
                        state <= D_IDLE;
                    end else begin
                        r_done <= r_done + WW'(1);
                    end
                end
            end
            default: state <= D_IDLE;
            endcase

            // ============ 写单元 ============
            case (wstate)
            W_IDLE: begin
                if (st_to_axi) begin
                    w_addr <= st_axi_addr; w_data <= st_axi_data; w_strb <= st_axi_strb;
                    w_is_store <= 1'b1; w_is_exit <= st_axi_exit;
                    wstate <= W_REQ;
                end else if (ev_item_req) begin
                    w_addr <= {ev_tag, ev_idx, ev_i[OFF_W-1:0], 2'b00};
                    w_data <= data_rdata; w_strb <= 4'hF;
                    w_is_store <= 1'b0; w_is_exit <= 1'b0;
                    wstate <= W_REQ;
                end
            end
            W_REQ: begin
                if (wr_req_ready) begin
                    if (w_is_store) begin
                        st_to_axi <= 1'b0;
                        if (w_is_exit) begin
                            wstate <= W_IDLE;             // exit：B 走 exit_ack
                        end else begin
                            wtag[wtag_w] <= 1'b1;
                            wtag_w   <= wtag_w + 2'd1;
                            wtag_cnt <= wtag_cnt + 3'd1;
                            wstate   <= W_WAIT;
                        end
                    end else begin
                        // 换出字：推标签，不等待
                        wtag[wtag_w] <= 1'b0;
                        wtag_w   <= wtag_w + 2'd1;
                        wtag_cnt <= wtag_cnt + 3'd1;
                        wstate   <= W_IDLE;
                        if (ev_i < WW'(LINE_WORDS-1)) state <= D_EVRD;
                        ev_i <= ev_i + WW'(1);
                    end
                end
            end
            W_WAIT: begin
                // 由 B 弹出标签时回 W_IDLE（见下方 FIFO 段）
            end
            default: wstate <= W_IDLE;
            endcase

            // ============ 写标签 FIFO（B 弹出）============
            if (wr_resp_valid && (wtag_cnt != 3'd0)) begin
                wtag_r   <= wtag_r + 2'd1;
                wtag_cnt <= wtag_cnt - 3'd1;
                if (wtag[wtag_r] == 1'b1) begin
                    st_b_done <= 1'b1;
                    wstate <= W_IDLE;
                end else if (ev_busy) begin
                    ev_b <= ev_b + WW'(1);
                end
            end
        end
    end

endmodule
