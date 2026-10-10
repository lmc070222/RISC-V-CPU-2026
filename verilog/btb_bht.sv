// btb_bht: 分支目标缓冲 + 分支历史表（2 位饱和计数器）
// 预测在取指阶段查询（组合输出）；执行阶段（BRU 解析）更新。
// 表均为寄存器数组（面积超支再换 sram_fakeram）；直接映射。
// 正确性由恢复路径兜底：即使预测错误，BRU 解析后按"实际目标 vs 预测目标"比较
// 触发 flush + 重定向，故"预测命中但 BHT 状态错误"不影响程序正确性。
// 更新竞态：BRU 每拍至多解析一条，更新天然串行；取指查询为组合、更新在边沿，
// 同拍查询读到旧值（晚一拍生效），无结构冒险。
module btb_bht #(
    parameter int BTB_ENTRIES = 64,
    parameter int BHT_ENTRIES = 512,
    parameter int BTB_IDX_W   = 6,      // = $clog2(BTB_ENTRIES)，索引 pc[BTB_IDX_W+1:2]
    parameter int BHT_IDX_W   = 9,      // = $clog2(BHT_ENTRIES)，索引 pc[BHT_IDX_W+1:2]
    parameter int TAG_W       = 24      // tag = pc[31:8]
) (
    input  logic clk,
    input  logic reset,

    // ---- 取指阶段预测查询（组合）----
    input  logic [31:0] pc,
    output logic        predict_hit,      // BTB 命中且有效
    output logic        predict_taken,    // BHT 计数器高位（>=2 视为 taken）
    output logic [31:0] predict_target,

    // ---- 执行阶段更新（BRU 解析）----
    input  logic        upd_valid,
    input  logic [31:0] upd_pc,
    input  logic        upd_taken,
    input  logic [31:0] upd_target
);

    logic             btb_valid  [0:BTB_ENTRIES-1];
    logic [TAG_W-1:0] btb_tag    [0:BTB_ENTRIES-1];
    logic [31:0]      btb_target [0:BTB_ENTRIES-1];
    logic [1:0]       bht_cnt    [0:BHT_ENTRIES-1];

    logic [BTB_IDX_W-1:0] btb_idx, btb_idx_u;
    logic [BHT_IDX_W-1:0] bht_idx, bht_idx_u;
    assign btb_idx   = pc[BTB_IDX_W+1:2];
    assign bht_idx   = pc[BHT_IDX_W+1:2];
    assign btb_idx_u = upd_pc[BTB_IDX_W+1:2];
    assign bht_idx_u = upd_pc[BHT_IDX_W+1:2];

    assign predict_hit    = btb_valid[btb_idx] && (btb_tag[btb_idx] == pc[31:8]);
    assign predict_taken  = bht_cnt[bht_idx][1];
    assign predict_target = btb_target[btb_idx];

    // 复位初始化（generate 展开，避免 Verilator 对 for 循环内数组 NBA 的限制）
    generate
        for (genvar gi = 0; gi < BTB_ENTRIES; gi++) begin : g_btb_init
            always_ff @(posedge clk)
                if (reset) btb_valid[gi] <= 1'b0;
        end
        for (genvar gi = 0; gi < BHT_ENTRIES; gi++) begin : g_bht_init
            always_ff @(posedge clk)
                if (reset) bht_cnt[gi] <= 2'b01;   // 弱不跳
        end
    endgenerate

    always_ff @(posedge clk) begin
        if (upd_valid) begin
            // BHT 2 位饱和计数器
            if (upd_taken) begin
                if (bht_cnt[bht_idx_u] != 2'b11)
                    bht_cnt[bht_idx_u] <= bht_cnt[bht_idx_u] + 2'd1;
            end else begin
                if (bht_cnt[bht_idx_u] != 2'b00)
                    bht_cnt[bht_idx_u] <= bht_cnt[bht_idx_u] - 2'd1;
            end
            // BTB：taken 时写入
            if (upd_taken) begin
                btb_valid[btb_idx_u]  <= 1'b1;
                btb_tag[btb_idx_u]    <= upd_pc[31:8];
                btb_target[btb_idx_u] <= upd_target;
            end
        end
    end

endmodule
