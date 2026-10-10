# 接口契约文档（iface.md）

> 版本 v1.0 草案 | 2026-10-06 | 状态：**待双方互审后冻结**
> 规则：消费者提需求、所有者改定义、双方确认后 24 小时生效；先改本文件再改代码。

---

## 1. 通用约定

- **时钟/复位**：所有模块使用 `clk` + `reset`（**高电平有效**，与框架一致；启动时框架保持 5 拍高电平，复位期间所有 `valid`/`ready` 输出必须为 0）。
- **握手**：`valid` 拉高后必须保持到同周期 `ready` 为高（fire）；`valid` 不得组合依赖 `ready`；接收方 fire 当拍数据有效。
- **参数化规范**：参数全大写；模块必须能在参数默认值下 `make build` 通过。
- **命名**：`*_valid`/`*_ready` 成对；请求类信号以 `req_` 前缀、响应类以 `resp_` 前缀。
- **保留名**：不得声明 `sram_fakeram*`、`fakeram_asap7_*` 前缀的模块。

## 2. 模块所有权清单

| 模块 | 所有者 | 消费者 | 状态 |
| --- | --- | --- | --- |
| `axi_master` | A | mem_if / fetch_unit / store_unit | 草案 |
| `mem_if` | A | B 的 LSU / Store Buffer | 草案 |
| `fetch_unit` | A | B 的流水线 | 草案 |
| `store_unit` | A | B 的 Store Buffer / LSU | 草案 |
| `decoder` | B | B 的流水线 / 重命名 | **v1.0** |
| `regfile` | B | B 的流水线 / ROB 提交 | **v1.0** |
| `alu` | B | B 的流水线 / 发射队列 | **v1.0** |
| `mdu` | B | B 的流水线 / 发射队列 | **v1.0** |
| `rob` | B | B 的重命名 / 提交 | 草案（阶段 4 细化） |
| `issue_queue` | B | B 的重命名 | 草案（阶段 4 细化） |
| `student_top` | 共同 | 框架 | 框架模板固定 |

---

## 3. axi_master（A 所有，B 通过 mem_if/fetch_unit 间接消费）

功能：CPU 内部请求 ↔ AXI4-Lite 五通道转换；识别退出 Store。

```systemverilog
module axi_master #(
    parameter int RD_QUEUE_DEPTH = 4,   // 最多在飞读请求数（< 从机 16 项队列）
    parameter int WR_QUEUE_DEPTH = 4    // 最多在飞写请求数
) (
    input  logic clk,
    input  logic reset,

    // ---- CPU 侧：读 ----
    input  logic        rd_req_valid,
    output logic        rd_req_ready,       // 在飞读请求数 < RD_QUEUE_DEPTH
    input  logic [31:0] rd_req_addr,        // 必须 4 字节对齐
    output logic        rd_resp_valid,
    input  logic        rd_resp_ready,
    output logic [31:0] rd_resp_data,
    output logic [1:0]  rd_resp_err,        // 2'b00=OKAY 2'b10=SLVERR 2'b11=DECERR

    // ---- CPU 侧：写 ----
    input  logic        wr_req_valid,
    output logic        wr_req_ready,       // 在飞写请求数 < WR_QUEUE_DEPTH
    input  logic [31:0] wr_req_addr,        // 必须 4 字节对齐
    input  logic [31:0] wr_req_data,
    input  logic [3:0]  wr_req_strb,
    output logic        wr_resp_valid,
    input  logic        wr_resp_ready,
    output logic [1:0]  wr_resp_err,

    // ---- 退出协议 ----
    output logic        exit_ack,           // 退出 Store（0x80000000 且 wstrb=4'hf）的 B 握手完成脉冲

    // ---- AXI4-Lite 侧（与框架 student_top 模板一致）----
    output logic [31:0] araddr;  output logic arvalid; input logic arready;
    input  logic [31:0] rdata;   input  logic [1:0] rresp;  input logic rvalid; output logic rready;
    output logic [31:0] awaddr;  output logic awvalid; input logic awready;
    output logic [31:0] wdata;   output logic [3:0] wstrb;  output logic wvalid; input logic wready;
    input  logic [1:0]  bresp;   input  logic bvalid;       output logic bready;
);
```

**语义**：
- 每通道请求按序接受、响应按序返回；`rd_req_ready/wr_req_ready` 仅由在飞计数决定（不得组合依赖 AXI 侧 ready）。
- `exit_ack` 在退出 Store 的 `bvalid && bready` 当拍置 1，CPU 收到后即可停止一切活动。
- 错误响应仅透传；本课程程序不应触发（对齐 + 地址范围由软件保证）。

---

## 4. mem_if（A 所有，B 消费）

功能：核心访存子系统统一入口。v1 = 直通桥（内部直接接 axi_master）；v2 = D-cache + MSHR（**接口不变，无缝替换**）。

```systemverilog
module mem_if #(
    parameter int MSHR_DEPTH = 4,    // 最多在飞请求数
    parameter int ID_WIDTH    = 6    // 请求标签位宽（建议接 ROB 号）
) (
    input  logic clk,
    input  logic reset,

    // ---- 核心侧请求（来自 LSU / Store Buffer）----
    input  logic                  req_valid,
    output logic                  req_ready,
    input  logic                  req_is_write,   // 0=load 1=store
    input  logic [31:0]           req_addr,       // 字对齐地址（sub-word 选择在 LSU 完成）
    input  logic [31:0]           req_wdata,
    input  logic [3:0]            req_wstrb,      // SB/SH 由 LSU 生成掩码
    input  logic [ID_WIDTH-1:0]   req_id,

    // ---- 核心侧响应（允许乱序返回，按 req_id 匹配）----
    output logic                  resp_valid,
    input  logic                  resp_ready,
    output logic [ID_WIDTH-1:0]   resp_id,
    output logic [31:0]           resp_data,      // load 数据；store 响应时无意义
    output logic [1:0]            resp_err
);
```

**语义**：
- 所有传输均为 32 位字粒度；`LB/LBU/LH/LHU` 的字节选择与符号扩展在 **B 的 LSU** 完成。
- 同一 `req_id` 的在飞请求最多 1 个（提交方保证）。
- v1 直通桥按序响应即可；接口保留乱序能力供 v2 使用，LSU 必须按 id 匹配而非按序消费。
- 退出 Store 经此接口直通（LSU 发 `req_is_write=1, addr=0x80000000, wstrb=4'hf`）。

---

## 5. fetch_unit / store_unit（A 所有，B 消费）

### 5.1 fetch_unit（阶段 3 交付）

```systemverilog
module fetch_unit #(
    parameter int FETCH_QUEUE_DEPTH = 4
) (
    input  logic clk,
    input  logic reset,

    output logic [31:0] instr,        // 与 fetch_valid 同拍有效
    output logic        fetch_valid,
    input  logic        fetch_ready,  // 流水线本拍接收（stall 时置 0）

    input  logic        redirect_valid, // 分支/跳转重定向（优先级高于顺序取指）
    input  logic [31:0] redirect_pc
);
```

**语义**：PC 由 fetch_unit 内部持有；`redirect_valid` 当拍生效，下拍从 `redirect_pc` 取指。

### 5.2 store_unit（阶段 3 交付）

```systemverilog
module store_unit #(
    parameter int ST_QUEUE_DEPTH = 4
) (
    input  logic clk,
    input  logic reset,
    input  logic        st_valid,
    output logic        st_ready,
    input  logic [31:0] st_addr,      // 字对齐
    input  logic [31:0] st_data,
    input  logic [3:0]  st_strb,
    output logic        st_done,      // B 响应收到脉冲（含退出 Store 的 exit_ack 同步输出）
    output logic        exit_ack
);
```

**语义**：阶段 2/3 阻塞式；阶段 4 起改由 Store Buffer 直接接 mem_if，本模块退化为普通写通路。

---

## 6. decoder（B 所有，v1.0 冻结）

```systemverilog
package pkg_cpu2026;
    // ---- 编码定义（B 所有，双方共用）----
    localparam logic [4:0] ALU_ADD=5'd0, ALU_SUB=5'd1, ALU_SLL=5'd2, ALU_SLT=5'd3,
                           ALU_SLTU=5'd4, ALU_XOR=5'd5, ALU_SRL=5'd6, ALU_SRA=5'd7,
                           ALU_OR=5'd8, ALU_AND=5'd9, ALU_PASSA=5'd10, ALU_PASSB=5'd11;
    localparam logic [2:0] LD_LB=3'd0, LD_LBU=3'd1, LD_LH=3'd2, LD_LHU=3'd3, LD_LW=3'd4;
    localparam logic [1:0] ST_SB=2'd0, ST_SH=2'd1, ST_SW=2'd2;
    localparam logic [1:0] MUL_MUL=2'd0, MUL_MULH=2'd1, MUL_MULHSU=2'd2, MUL_MULHU=2'd3;
    localparam logic [1:0] DIV_DIV=2'd0, DIV_DIVU=2'd1, DIV_REM=2'd2, DIV_REMU=2'd3;
    localparam logic [2:0] BR_BEQ=3'd0, BR_BNE=3'd1, BR_BLT=3'd2, BR_BGE=3'd3,
                           BR_BLTU=3'd4, BR_BGEU=3'd5;
    // ---- 全局参数默认值（阶段 4 启用）----
    parameter int PRF_W    = 6;   // 物理寄存器 64 项
    parameter int ROB_ID_W = 6;   // ROB 64 项
endpackage

typedef struct packed {
    logic        illegal;     // CSR/FENCE/ECALL/EBREAK/未知编码 → 1
    // 分类
    logic        is_alu;      // R-type 或 I-type 算术逻辑
    logic        is_lui;
    logic        is_auipc;
    logic        is_load;
    logic        is_store;
    logic        is_branch;
    logic        is_jal;
    logic        is_jalr;
    logic        is_mul;
    logic        is_div;
    // 操作数
    logic [4:0]  rs1;
    logic [4:0]  rs2;
    logic [4:0]  rd;
    logic [31:0] imm;         // 已符号扩展；U-type 已左移 12 位
    logic [2:0]  funct3;
    logic [6:0]  funct7;
    // 控制
    logic [4:0]  alu_op;
    logic [2:0]  ld_type;     // 见 LD_*
    logic [1:0]  st_type;     // 见 ST_*
    logic [1:0]  mul_type;    // 见 MUL_*
    logic [1:0]  div_type;    // 见 DIV_*
    logic [2:0]  br_type;     // 见 BR_*
    logic        uses_rs2;    // R-type ALU / mul / div / store / branch 为 1
    logic        rd_write;    // 是否写 rd（branch/store 为 0）
} dec_info_t;

module decoder (
    input  logic [31:0] instr,
    output dec_info_t   dec
);
```

**语义**：
- 纯组合逻辑，1 周期内给出全部控制位（阶段 2 直接例化；阶段 4 重命名复用）。
- `illegal` 置位的指令在阶段 2 作为"停止提交"处理；课程测试不含这些指令。
- 立即数语义表（bit 位拼接，来自 RISC-V 规范）见 notes/b-20261006-执行单元规格.md。

---

## 7. regfile（B 所有，v1.0 冻结）

```systemverilog
module regfile #(
    parameter int NUM_REGS     = 32,
    parameter int READ_PORTS   = 2,
    parameter int WRITE_PORTS  = 1
) (
    input  logic clk,
    input  logic reset,
    // 读（组合输出，写穿：同拍写可读出新值）
    input  logic [4:0]  rs1_addr,
    input  logic [4:0]  rs2_addr,
    output logic [31:0] rs1_data,
    output logic [31:0] rs2_data,
    // 写（上升沿写入）
    input  logic        we,
    input  logic [4:0]  wa,
    input  logic [31:0] wd
);
```

**语义**：
- **x0 硬接线**：读 x0 恒 0；写 x0 被忽略（不产生物理写）。
- 阶段 2 按 v1.0 实现；阶段 4 的 PRF 是**另一个模块**，复用端口风格（读写端口数随发射宽度参数化），本模块在顺序阶段保留。

---

## 8. alu（B 所有，v1.0 冻结）

```systemverilog
module alu (
    input  logic [4:0]  op,       // pkg_cpu2026 的 ALU_* 编码
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic [31:0] out,
    output logic        eq,       // a == b（分支用）
    output logic        lt,       // $signed(a) < $signed(b)
    output logic        ltu       // a < b 无符号
);
```

**语义**：
- 纯组合；`SLL/SRL/SRA` 移位量取 `b[4:0]`。
- `ALU_PASSA`：`out = a`（LUI：a 接 imm）；`ALU_PASSB`：`out = b`（预留）。
- `JAL/JALR` 链接地址：用 `ALU_ADD`，a=PC，b=4；`AUIPC`：`ALU_ADD`，a=PC，b=imm。
- 分支由 BRU 使用 `eq/lt/ltu` + `br_type` 判定，不修改寄存器。

---

## 9. mdu（B 所有，v1.0 冻结）

```systemverilog
module mdu #(
    parameter int DIV_MAX_CYCLES = 34,     // 迭代上限（含提前退出）
    parameter logic MUL_PIPELINE  = 0      // 阶段 2 固定 0：单周期组合乘法
) (
    input  logic clk,
    input  logic reset,
    input  logic        start,     // 1 拍脉冲；busy 时忽略
    input  logic        is_mul,    // 1=乘法 0=除法
    input  logic [1:0]  mul_type,
    input  logic [1:0]  div_type,
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic        busy,
    output logic        valid,     // 本拍 result 有效
    output logic [31:0] result
);
```

**语义**：
- `start=1 且 !busy` 当拍锁存输入；乘法 `valid` 在下一拍；除法最多 `DIV_MAX_CYCLES` 拍（余数为 0 提前结束）。
- 除零/溢出语义按 RISC-V 规范（DIV 除以 0 → 0xFFFFFFFF；DIV 溢出 → 0x80000000；REM 除以 0 → 被除数；REM 溢出 → 0），详见规格文档。

---

## 10. rob（B 所有，草案——阶段 4 细化）

```systemverilog
module rob #(
    parameter int ROB_DEPTH    = 32,   // 建议 32/64/128 三档扫描
    parameter int COMMIT_WIDTH = 2,
    parameter int PRF_W        = 6,
    parameter int ROB_ID_W     = 6
) (
    input  logic clk,
    input  logic reset,
    // ---- 分配（来自重命名，程序顺序）----
    input  logic              alloc_valid,
    output logic              alloc_ready,
    input  logic [31:0]       alloc_pc,
    input  logic [4:0]        alloc_arch_rd,
    input  logic [PRF_W-1:0]  alloc_phys_rd,
    input  logic              alloc_has_rd,
    output logic [ROB_ID_W-1:0] alloc_rob_id,
    // ---- 完成（来自 CDB）----
    input  logic              complete_valid,
    input  logic [ROB_ID_W-1:0] complete_rob_id,
    input  logic              complete_exception,
    input  logic              complete_mispred,
    input  logic [31:0]       complete_target,
    // ---- 提交 ----
    output logic              commit_valid,
    output logic [COMMIT_WIDTH-1:0][ROB_ID_W-1:0] commit_rob_id,
    output logic [COMMIT_WIDTH-1:0][4:0]           commit_arch_rd,
    output logic [COMMIT_WIDTH-1:0][PRF_W-1:0]     commit_phys_rd,
    output logic [COMMIT_WIDTH-1:0]                commit_has_rd,
    // ---- flush：清除晚于 flush_rob_id 的一切 ----
    input  logic              flush_valid,
    input  logic [ROB_ID_W-1:0] flush_rob_id
);
```

**语义**：`flush_valid` 为 1 时，比 `flush_rob_id` 年轻的条目当拍作废；`alloc_rob_id` 复用被 flush 的编号。

---

## 11. issue_queue（B 所有，草案——阶段 4 细化）

```systemverilog
module issue_queue #(
    parameter int IQ_DEPTH     = 8,    // 每队列深度
    parameter int ISSUE_WIDTH  = 1,    // 每周期发射条数
    parameter int WAKEUP_PORTS = 2,    // CDB 写回端口数
    parameter int PRF_W        = 6,
    parameter int ROB_ID_W     = 6
) (
    input  logic clk,
    input  logic reset,
    // ---- 入队（来自重命名）----
    input  logic              alloc_valid,
    output logic              alloc_ready,
    input  logic [ROB_ID_W-1:0] alloc_rob_id,
    input  logic              alloc_has_rs1,
    input  logic              alloc_has_rs2,
    input  logic [PRF_W-1:0]  alloc_p1,       // rs1 物理寄存器号
    input  logic [PRF_W-1:0]  alloc_p2,
    input  logic [31:0]       alloc_imm,      // I-type 立即数或 store 数据
    input  logic              alloc_use_imm,  // op2 = imm 而非 rs2
    input  logic [4:0]        alloc_op,       // 直达 FU 的操作码位
    // ---- 唤醒（CDB tag 广播）----
    input  logic [WAKEUP_PORTS-1:0]            wakeup_valid,
    input  logic [WAKEUP_PORTS-1:0][PRF_W-1:0] wakeup_tag,
    // ---- 发射（最老就绪优先）----
    output logic              issue_valid,
    output logic [ROB_ID_W-1:0] issue_rob_id,
    output logic [31:0]       issue_op1,
    output logic [31:0]       issue_op2,
    output logic [4:0]        issue_op,
    // ---- flush ----
    input  logic              flush_valid,
    input  logic [ROB_ID_W-1:0] flush_rob_id
);
```

**语义**：被 flush 的条目立即归还其占用的物理寄存器（与空闲列表联动）。

---

## 12. student_top（共同，框架模板固定）

端口与 `README-ZH.md` 中的模板**逐位一致**，内部仅例化核心 + fetch_unit/store_unit/mem_if 等顶层互连。

---

## 13. 变更记录

| 日期 | 版本 | 变更 | 提议人 | 状态 |
| --- | --- | --- | --- | --- |
| 2026-10-06 | v1.0 | 初稿 | B（执行单元部分）/ A（总线部分草案） | 待互审 |
