# RISC-V CPU（CPU 2026）开发流程文档

> 本文档基于仓库内全部资料（README-ZH、docs/axi4-lite.md、docs/sram.md、Makefile、config.mk、scripts/ 仿真与综合框架）整理，给出从环境搭建到最终提交的完整开发路线：**每一阶段要做什么、以什么顺序做、怎么验证、验收标准是什么**。

---

## 0. 总体路线图

| 阶段 | 时间（14 周制） | 主题 | 验收标准 |
| --- | --- | --- | --- |
| 阶段 0 | 第 1 周 | 环境搭建与框架熟悉 | `make build` / `make run` 跑通最小设计 |
| 阶段 1 | 第 1–2 周 | ISA 学习与微架构设计 | 设计文档 + 模块框图 |
| 阶段 2 | 第 2–3 周 | 最小可运行 CPU（多周期/顺序流水） | 三个基础程序全过（**75 分档**） |
| 阶段 3 | 第 3–4 周 | 顺序 5 级流水 + 冒险处理 | 基础程序稳定通过，可跑 perf |
| 阶段 4 | 第 4–7 周 | 乱序核心（重命名 + ROB + 按序提交） | 正确性测试集基本全过；**第 7 周中期检查** |
| 阶段 5 | 第 7–9 周 | Cache 与 SRAM 存储子系统 | perf 有 IPC 提升，综合面积含 SRAM |
| 阶段 6 | 第 9–10 周 | 正确性收敛 | `make test` 全过（**85 分档**） |
| 阶段 7 | 第 10–12 周 | 性能优化（IPC 冲档） | GEOMEAN IPC ≥ 目标档位 |
| 阶段 8 | 第 12–13 周 | 综合与时序（面积/频率） | 面积 ≤ 档位上限，频率 ≥ 300 MHz |
| 阶段 9 | 第 13–14 周 | 报告与 OJ 提交 | **第 14 周周六 23:59 前提交** |

**评分背景**（决定每个阶段的优先级）：

- **正确性 85 分**：基础程序（向量乘、向量加、0–100 累加）75 分 → 仿真测试集其余程序 85 分。前一阶段不过，后一阶段无分；退出返回值必须与标准答案**完全一致**。
- **性能 15 分 + 频率加分最多 4 分**，三档要求：

  | 档位 | 最大面积 (μm²) | 最低 IPC（几何平均） | 最低频率 (MHz) | 累计分数 |
  |---|---|---|---|---|
  | 1 | 9,000 | 0.6000 | 300 | 90 |
  | 2 | 18,000 | 0.8450 | 300 | 95 |
  | 3 | 36,000 | 1.0985 | 300 | 100 |

  - 频率加分：400 MHz +2，500 MHz +4。
  - IPC = 动态指令数 / 运行周期数，对 `testcases/perf_*` 全部基准取**几何平均**。
  - 面积 = Yosys + ASAP7（7.5-track RVT TT）标准单元面积 + `sram_fakeram` 的 SRAM 估算面积。

---

## 阶段 0：环境搭建与框架熟悉（第 1 周）

**目标**：本地能编译、仿真、跑测试、综合，理解框架的"游戏规则"。

### 0.1 获取代码

```sh
# 强烈建议 Fork 而非下载 ZIP，便于同步官方测试用例与框架更新
git clone git@github.com:<your-name>/RISC-V-CPU-2026.git
cd RISC-V-CPU-2026
git submodule update --init --recursive   # 拉取 testcases 子模块（correctness_* 与 perf_*）
```

### 0.2 配置硬件工具链（三选一）

- **方案 A（推荐，Linux x86-64 / WSL2）**：下载 `cpu2026-tools-x86_64.AppImage` 放到项目根目录并 `chmod +x`。Make 会自动使用其中的 Verilator / Yosys / ABC / OpenSTA / ASAP7 库。无 FUSE 的系统可设 `export APPIMAGE_EXTRACT_AND_RUN = 1`。
- **方案 B（Docker）**：`docker build -t cpu2026 .`，然后 `docker run --rm -it -v "$PWD":/work cpu2026 bash`，在 `/work` 下执行 make。
- **方案 C（原生工具链）**：在 `config.mk` 里把 `APPIMAGE` 置空并填写 `VERILATOR/YOSYS/ABC/STA/ASAP7_LIB` 绝对路径。

### 0.3 熟读必读文档

1. `README-ZH.md`：指令集要求、微架构要求、终止约定、评分标准（本流程文档的依据）。
2. `docs/axi4-lite.md`：AXI4-Lite 五通道握手与内存从机行为（**必读，逐条理解**）。
3. `docs/sram.md`：`sram_fakeram` 的接口、时序与支持范围（Cache 阶段必用）。
4. `Makefile` / `config.mk`：所有构建入口与可覆盖参数。
5. `scripts/sim.cpp`：内存从机行为的事实标准（队列深度、延迟、退出判定）。

### 0.4 验证框架可用

写一个最小 `student_top`（只含规定端口，例如复位后直接向 `0x80000000` 写 `wstrb=4'hf` 的返回值为 0），加入 `verilog/filelist.f`，然后：

```sh
make help     # 查看所有命令
make build    # Verilator 编译（产物 build/sim）
make run PROGRAM=testcases/correctness_add_to_100/program.data EXPECTED=5050
make synth    # 综合流程走通
```

**验收标准**：上述四条命令全部正常执行。此时应已能回答：复位几拍、地址空间多大、延迟多少周期、程序如何退出、队列深度多少。

---

## 阶段 1：ISA 学习与微架构设计（第 1–2 周）

**目标**：在写代码前把"要做什么"定清楚，避免中期返工。

### 1.1 明确指令集边界（RV32IM）

- **必须完整实现**：RV32I 用户态全部指令 + M 扩展全部 8 条（`MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU`）+ 全部 Load/Store（`LB, LBU, LH, LHU, LW, SB, SH, SW`）。
- **不做**：`CSR*`、`FENCE/FENCE.I`、`ECALL/EBREAK`。
- **不要求**非对齐访存（访问按自然边界对齐）。
- **细节坑**：乘除法是带符号/无符号组合的（`MULHSU` 混合符号），除零与溢出语义要按 RISC-V 规范实现（如 DIV 除以 0 商为 0xFFFFFFFF、REM 余数为被除数）。

### 1.2 明确接口与平台约束

- 顶层必须叫 `student_top`，端口与 README 中的 AXI4-Lite 模板**逐位一致**（AR/R/AW/W/B 五通道 + `clock/reset`）。
- `reset` 高电平有效，启动时保持 **5 周期**。
- 内存：256 MiB 小端 RAM，地址 `0x00000000–0x0fffffff`，未加载字节为 0；读写地址必须 4 字节对齐。
- 每通道 16 项 FIFO；请求握手后**下一周期**才进入服务；服务到响应默认 **10 周期**延迟（`LATENCY` 可调）。
- 退出：向 `0x80000000` 执行 `wstrb=4'hf` 的 32 位 Store，`wdata` 为返回值；仿真器在 **B 通道握手成功瞬间**（`bvalid && bready`）捕获结果并结束。
- 复位/写响应期间 `ready` 为 0；`arvalid` 等不能在组合逻辑上依赖 `arready`（防死锁黄金法则）。

### 1.3 选定 HDL 与总体方案

- 语言：Verilog/SystemVerilog/Chisel 均可；用 Chisel 需在提交前产出 Verilog（参考 `chisel-example` 分支）。
- 微架构硬性要求：**乱序发射执行 + 按序提交（ROB）**，且发射宽度、ROB 深度、PRF 大小、保留站深度、Cache 容量/相联度**必须参数化**。
- 推荐基线方案（下文阶段 2–5 即按此展开）：
  - 顺序阶段作为跳板 → 改为 **PRF 物理寄存器重命名 + 各功能单元发射队列 + ROB 按序提交** 的 OoO；
  - I-cache + D-cache + Store Buffer；分支预测（BTB/BHT，可选 RAS）。

### 1.4 产出

模块划分图（取指、译码/重命名、发射队列、执行单元、ROB、LSU/Cache、AXI 适配器）、接口定义、参数列表、git 分支策略（建议每阶段一个分支 + tag，保持可回滚）。

**验收标准**：两人组内评审通过的设计文档；`config.mk` 已按本机配置好（`JOBS/MAX_CYCLES/LATENCY` 等）。

---

## 阶段 2：最小可运行 CPU（第 2–3 周）

**目标**：以最快路径跑通三个基础程序（向量乘、向量加、0–100 累加），拿到 75 分档的可行性证明。

### 2.1 实现内容（按依赖顺序）

1. **顶层 `student_top`**：按端口模板声明，内部例化 CPU 核心与 AXI 适配器。
2. **AXI4-Lite Master 适配器**（优先级最高，建议最先单测）：
   - 读：`arvalid/arready` 发地址 → 等 `rvalid`，`rready` 接数据（实现"请求即被接受后须有接收方"的握手状态机）。
   - 写：`aw` 与 `w` 两通道可同周期也可错开，但**两者都握手后**从机才处理；B 通道必须及时 `bready` 接收响应，否则会堵队列。
   - 记住：`VALID` 拉高后必须保持到握手成功；`VALID` 不得组合依赖 `READY`。
3. **取指**：PC + 通过 AXI 读指令字。内存延迟 10 周期，取指阶段必然要**阻塞等待**（多周期状态机或流水 stall 均可）。
4. **译码器**：先覆盖基础程序实际用到的指令子集（`LW/SW/ADD/ADDI/MUL/SUB/BEQ/BNE/JAL/JALR/BLT/SLTI` 等），**结构上按 RV32IM 全集预留** opcode/imm 译码，避免后期重写。
5. **寄存器堆**（x0 恒 0）。
6. **ALU + 乘除单元**：乘法可先单周期组合逻辑；除法用迭代法，注意多周期时如何阻塞。
7. **访存单元 LSU**：load 阻塞写回直到数据返回；store 由 AXI 适配器处理 `wstrb`（`SW` 全 1，`SH/SB` 按字节掩码——虽然基础程序可能只用 `SW`，掩码逻辑现在就做对）。
8. **退出协议**：识别向 `0x80000000` 的 Store，确保发出 `awaddr=0x80000000, wstrb=4'hf, wdata=返回值`，并继续应答 B 通道直到握手完成（否则不计分）。

### 2.2 形态建议

先用**多周期状态机**或**不转发的顺序流水**即可，本阶段重点是：译码正确、AXI 正确、退出正确。把总线当"黑盒慢内存"，不要过早做缓存。

### 2.3 调试手段

```sh
make run PROGRAM=testcases/correctness_add_to_100/program.data EXPECTED=5050
make run ... WAVE=trace.vcd   # 波形（Surfer/GTKWave 查看）
make run ... LOG=run.log      # 日志：RTL 里用 $display 打 PC/寄存器/总线握手
```

- 框架建议：**优先用 `$display` 与断言，不要死磕波形**。
- 善用 `MAX_CYCLES`、`LATENCY`（如先 `LATENCY=1` 排除时序耦合，再回到 10）。
- 对照 `scripts/sim.cpp` 理解从机行为（队列深度 16、服务节奏、响应延迟）。

**验收标准**：`make test Case=correctness_add_to_100`、向量乘、向量加三个用例全部 PASS（75 分档）。建议对每个里程碑 `git tag`。

---

## 阶段 3：顺序 5 级流水 + 冒险处理（第 3–4 周，可并入阶段 4）

**目标**：把阶段 2 的单发设计改成流水线，为 OoO 打基础（若你对 OoO 很有把握，可把本阶段压缩，但**冒险处理逻辑的语义**必须在 OoO 里被正确覆盖）。

### 3.1 实现内容

1. **流水划分**：F（取指）/ D（译码）/ E（执行，ALU 或乘除）/ M（访存）/ W（写回）。
2. **数据冒险**：旁路转发网络（EX/MEM → EX，MEM/WB → EX）+ 必要的 stall（如 load-use）。
3. **控制冒险**：分支先在译码/执行阶段解析、stall + flush（先保证正确，预测留到阶段 7）。
4. **访存阻塞**：AXI 响应未回时停流水；`aw` 与 `w` 通道同拍或分拍都要测到。
5. **参数化雏形**：流水级、寄存器堆端口数用 parameter。

### 3.2 关键点

- load-use 必须 stall：这是正确性陷阱（数据拿旧值）。
- store 数据被后续 load 读回的情形（SW 后紧跟 LW 同地址）要覆盖，写出自测小程序。
- 乘除单元若多周期，需在流水线里插入 busy/stall 逻辑——建议此时就把乘除做成**独立功能单元 + 发射队列**的雏形，阶段 4 直接复用。

**验收标准**：三个基础程序 + 自编的冒险/回写测试全部通过；`make perf` 能正常输出 IPC 表格（IPC 低没关系）。

---

## 阶段 4：乱序核心（第 4–7 周，含中期检查）

**目标**：满足课程核心要求——**操作数就绪即乱序发射执行 + ROB 按序提交**。

### 4.1 实现内容（严格按此依赖顺序推进）

1. **物理寄存器堆 PRF + 重命名**：
   - 译码段维护 RAT（架构寄存器 → 物理寄存器）与空闲列表；
   - 目的寄存器在译码时重命名，写回时释放旧物理寄存器（旧值在提交后才真正释放）；
   - PRF 大小参数化（如 32 架构 + 32 物理）。
2. **发射队列 / 保留站**：
   - 每条指令携带源操作数（就绪值或等待的物理寄存器 tag）；
   - **唤醒**（Wakeup）：写回 tag 广播与比较（CAM 或全比较器）；
   - **选择**（Select）：就绪指令中按年龄（如 ROB 号）仲裁发射；
   - 每个功能单元一个发射队列，深度参数化。
3. **功能单元**：ALU（整数/逻辑/分支地址）、MDU（乘除，多周期）、LSU（地址计算 + 访存队列）、BRU（分支判定）。
4. **写回总线/仲裁**：支持多写回端口（参数化），避免同周期写回冲突。
5. **ROB + 按序提交**：
   - ROB 记录目标架构寄存器、物理寄存器、异常信息、完成位、PC；
   - **提交宽度**参数化；只有 ROB 头且已完成才提交；
   - 异常/分支重定向在 ROB 中标记，提交（或分支解析）时统一处理，**精确**恢复。
6. **分支错误恢复**：分支在 BRU 解析后广播 flush，清空更年轻的指令；RAT 用 checkpoint 恢复或从 ROB 顺序回放（任选一种，写清楚权衡）。
7. **访存顺序**：
   - Store Buffer：store 在提交时才真正写入内存（或提交后立即发），load 可对 store buffer 做转发（地址命中 + 部分重叠处理）；
   - 保守方案：load 等待所有更老 store 发射后再执行，保证顺序（先正确后性能）。
8. **参数化**：发射宽度、ROB 深度、PRF 大小、保留站深度、写回端口数全部为 parameter，便于阶段 7 扫描。

### 4.2 中期检查（第 7 周）前必须完成

- 基础三程序在 OoO 核心上通过；
- 自编乱序性测试（如延迟不等的操作链）验证确实乱序执行；
- 代码 Review 会考察**架构细节与权衡**，AI 辅助编码需保留与 LLM 的交互记录。

**验收标准**：`make test` 正确性集通过大部分（至少全部基础程序）；性能测试不超 `MAX_CYCLES` 超时。

---

## 阶段 5：存储子系统：Cache 与 SRAM（第 7–9 周）

**目标**：用 `sram_fakeram` 构建缓存层次，缓解 10 周期内存延迟对 IPC 的压制。

### 5.1 实现内容

1. **I-cache**（可先做，收益最直接）：
   - 数据阵列 + tag 阵列 + valid 位均可用 `sram_fakeram`（valid 位注意：SRAM **无复位引脚**，可用寄存器存 valid 并复位清零）；
   - 取指 miss 时经 AXI 按行（如 32B/64B）重填，重填期间停取指（或先单 MSHR）；
   - 写回式不适用 I-cache（只读），失效靠直接覆盖。
2. **D-cache**：
   - 参数：容量、相联度（直接映射起步，后续可做组相联）、行大小、替换策略（LRU/PLRU）、写策略（write-back/write-allocate 或 write-through 二选一，**先简单**）；
   - `LB/LBU/LH/LHU/SB/SH` 在 32 位数据通路上处理：字节掩码写入用 `sram_fakeram` 的 `WRITE_GRANULARITY=8` + 4 位 `wmask`；
   - 多 MSHR（Miss Status Holding Register）支持多个 outstanding miss（与 16 深 AXI 队列匹配）；
   - Cache 与 Store Buffer 的一致性：load 必须能命中 store buffer 中未写入 cache 的数据。
3. **BTB / BHT**（可选，可放阶段 7）：数组用 `sram_fakeram`，其面积会计入 SRAM 估算。
4. **遵守 SRAM 规范**（`docs/sram.md`）：
   - 单端口 1RW：同一周期只能一读或一写（数据/指令两阵列分开实例化）；
   - 读延迟 1 周期（本周期发地址，下周期取数）；
   - 实例参数限制：`DEPTH ≤ 1,048,576`、`WIDTH ≤ 4,096`、`DEPTH × WIDTH ≤ 16,777,216` 位（2 MiB/实例）；
   - **禁止**把 `sram_fakeram.sv` 加进 `filelist.f`（框架自动引入）、禁止自写同名 stub。
   - 面积成本参考：SRAM 约 **0.0419904 μm²/位**，即每 KiB ≈ 344 μm²（含 tag 另行计算）。

**验收标准**：`make perf` 的 IPC 相比无缓存版本有量级提升；`make synth` 报告中出现 SRAM 面积分项。

---

## 阶段 6：正确性收敛（第 9–10 周）

**目标**：`make test` 全部通过（85 分档），并储备回归能力。

### 6.1 工作内容

1. 逐条过 `testcases/correctness_*` 所有用例，失败时用 `LOG=` + `$display` 定位（PC、ROB 号、访存地址）。
2. 边界与语义自查：
   - `MULH/MULHSU/MULHU` 符号组合；`DIV/DIVU/REM/REMU` 除零、符号、溢出语义；
   - `LB/LBU/LH/LHU` 符号扩展；`SLTI/SLTIU` 无符号比较；移位量低 5 位；
   - 分支在 ROB 中的精确恢复；异常/重定向时 ROB 与 RAT 一致性；
   - 退出 Store 前所有更老指令必须已提交（保证内存可见顺序）；
   - 退出时序：B 通道握手必须被应答（`bready` 保持，不能提前停摆）。
3. 建立**自测回归集**：冒险组合、load-store 重叠、分支链、乘除边界、循环嵌套等小汇编，作为每次改动的 CI（`make test` 一键）。
4. 用 `LATENCY=1` 与 `LATENCY=10` 双档跑，暴露时序耦合 bug。

**验收标准**：`make test` 全部 PASS；`make perf` 全部用例在 `MAX_CYCLES` 内正常退出（IPC 暂不要求）。

---

## 阶段 7：性能优化（第 10–12 周）

**目标**：把 GEOMEAN IPC 推上目标档位（0.6000 / 0.8450 / 1.0985），同时控制面积。

### 7.1 优化杠杆（按性价比排序）

1. **分支预测**：BTB（2-way 起步）+ BHT（2-bit 饱和计数器），可加 RAS 处理 `JAL/JALR` 配对；预测失败惩罚受 flush 深度影响。
2. **发射宽度**：1 → 2 → 4，同时扩大写回端口数与提交宽度（瓶颈常在这三处，需成对提升）。
3. **资源规模**：ROB 深度（32→64→128）、PRF 大小、各保留站深度——用参数扫描找拐点（**这也是报告素材**）。
4. **访存级并行**：多 MSHR、非阻塞 cache（miss 下继续服务 hit）、load 绕过 store 的保守约束放宽、write-back 脏行及时回写。
5. **Cache 调优**：容量/相联度/行大小/替换策略扫描；关注 hit 延迟路径（tag 比较与选择）。
6. **流水级细化**：重命名/发射关键路径拆分，避免某级成为频率瓶颈。

### 7.2 方法（与阶段 8 交替迭代）

```sh
make perf                       # 全部基准 IPC 表 + GEOMEAN
make perf Case=perf_median      # 单点快速迭代
make synth MODE=diagnose        # 看面积分布，防止性能优化把面积打爆
```

- 每次只改一个变量，记录（参数配置 → IPC → 面积 → 频率）三元组，直接用于报告的参数敏感度分析。
- 面积预算意识：档位上限 9k / 18k / 36k μm²，SRAM 每 KiB ≈ 344 μm²，逻辑面积以 `synth` 实测为准。

**验收标准**：确定最终参数组合，IPC/面积同时落入目标档位；敏感度数据表成型。

---

## 阶段 8：综合与时序（第 12–13 周）

**目标**：面积与频率达标（300 MHz 底线，400/500 MHz 有 +2/+4 加分）。

### 8.1 工作流

1. `make synth MODE=diagnose`：看模块树面积占比（Area/Direct/% total），锁定大头。
2. 重构热点（如发射队列比较器阵列、多端口寄存器堆、旁路网络），再 `make synth`（opt 模式，跨模块展平优化）。
3. 频率迭代：
   ```sh
   make synth CLOCK_PERIOD_NS=2.0   # 500 MHz 目标
   make synth CLOCK_PERIOD_NS=3.333 # ~300 MHz
   ```
   - `report.txt`：总面积（组合/时序/SRAM 分项）、`Estimated frequency`（1/Tmin）、`Worst setup slack`（正 = 达标）、关键路径起止点。
   - `timing.rpt`：关键路径逐级单元延迟；`constraints.sdc`：I/O 约束。
4. 若关键路径在 SRAM（`clock_to_q` 与深度/宽度有关），考虑拆小实例；在组合逻辑则拆流水级或重写热点。
5. 权衡：加流水级换频率可能增面积；降发射宽度省面积但降 IPC——用阶段 7 的数据表做决策。

**验收标准**：`Estimated frequency ≥ 300 MHz`（目标 400/500），总面积 + SRAM 估算 ≤ 档位上限，slack 为正。

---

## 阶段 9：报告与 OJ 提交（第 13–14 周）

**DDL：第 14 周周六 23:59。两人一组。**

### 9.1 实验报告（必含两部分）

1. **参数敏感度分析**：对发射宽度、PRF 大小、ROB 大小、Cache 容量/相联度等做单变量扫描，给出 IPC（与面积/频率）随参数变化的曲线与趋势结论。
2. **架构探索总结**：开发中尝试过的方案、踩过的坑、最终权衡（如 checkpoint 恢复 vs 顺序回放、write-back vs write-through、直接映射 vs 组相联）。

### 9.2 OJ 提交检查清单

- [ ] 顶层模块名为 `student_top`，AXI4-Lite 端口与模板逐位一致；
- [ ] 所有 Verilog/SystemVerilog 源文件以**相对路径**列入 `verilog/filelist.f`；
- [ ] `sram_fakeram.sv` **不在** filelist 中；无自写 `sram_fakeram*` stub；无遗留 `blackboxes.txt`；
- [ ] `config.mk` 中不含本机绝对路径（OJ 用自己的工具链），提交前用 `make`（或 `make code`）验证能产出 `./code`；
- [ ] `make test` 全过、`make perf` 与 `make synth` 复现报告的 IPC/面积/频率数据；
- [ ] 保留与 LLM 的交互记录（Code Review 会查）。

---

## 附录 A：常用命令速查

```sh
make help                                        # 命令帮助
make build [JOBS=8]                              # Verilator 编译 → build/sim
make test [Case=correctness_add_to_100]          # 正确性测试
make perf [Case=perf_median]                     # 性能基准 + GEOMEAN
make run PROGRAM=<program.data> EXPECTED=5050 \
     [WAVE=trace.vcd] [LOG=run.log]              # 单程序 + 波形/日志
make synth [MODE=opt|diagnose] \
     [CLOCK_PERIOD_NS=2.0]                       # 综合 + STA，产物在 build/synth/<mode>/
make clean                                       # 清理
make / make code                                 # OJ 用：产出 ./code
# 常用参数：MAX_CYCLES=1000000  LATENCY=10  SIM=<预编译仿真器>  TESTCASES=<路径>
```

## 附录 B：规范硬约束清单（贴墙上）

| 项 | 要求 |
| --- | --- |
| 指令集 | RV32IM：RV32I 全集 + M 扩展 8 条 + LB/LBU/LH/LHU/LW/SB/SH/SW；不做 CSR/FENCE/ECALL/EBREAK；不要求非对齐 |
| 微架构 | 乱序发射执行 + ROB 按序提交；发射宽度/ROB/PRF/保留站/Cache 参数化 |
| 顶层 | `student_top`，AXI4-Lite 五通道端口严格一致；`reset` 高电平 5 周期 |
| 内存 | 256 MiB 小端 RAM（0x00000000–0x0fffffff）；地址 4 字节对齐；每通道 16 项 FIFO；默认 10 周期延迟 |
| 退出 | Store 到 `0x80000000`，`wstrb=4'hf`，返回值在 `wdata`；B 通道握手成功即仿真结束 |
| SRAM | 用 `sram_fakeram`；单端口 1RW；读延迟 1 周期；`en=0` 或写周期 `rdata` 未定义；无复位；越界地址 `$fatal`；单实例 ≤ 2 MiB |

## 附录 C：常见坑（阶段 2/3/4 高频踩雷点）

1. **AXI 死锁**：`VALID` 依赖 `READY`（组合环）、`rready/bready` 挂死后队列堵满 16 项、AW 与 W 不同步导致写请求永不被服务。→ 状态机明确"请求发出后必须有能力接收响应"。
2. **退出判定**：必须等到 `bvalid && bready`；`wstrb != 4'hf` 的 0x80000000 写会得 SLVERR 且不退出。
3. **复位**：5 拍复位期间与复位后首个请求行为要干净（PC 从 0 取指）。
4. **SRAM 时序**：读数据 1 周期后才有效；写周期 rdata 未定义（别拿它当前值读改写）；无复位 → valid/tag 初始化用寄存器。
5. **load-use / store-load 重叠**：顺序阶段靠 stall，OoO 阶段靠 store buffer 转发，漏一处就是随机错。
6. **重命名释放时机**：旧物理寄存器只能在**提交**时释放，否则会被 flush/异常破坏。
7. **分支 flush 不彻底**：ROB、发射队列、取指队列、总线请求（已发出的 AXI 写不能撤回）都要有对应处理。
8. **超时**：死锁/死循环会吃满 `MAX_CYCLES`，注意 `make test MAX_CYCLES=5000000` 区分"慢"与"死"。
9. **filelist**：重复条目、相对路径错误、把 `sram_fakeram.sv` 列进去都会直接构建失败。

## 附录 D：迭代工程习惯建议

- **每阶段一个 git 分支 + 里程碑 tag**，通过一项测试即提交；阶段 2 的"可运行最小内核"永远保留，作为 A/B 对照基线。
- **回归优先**：任何优化合入前先 `make test` 全绿，性能改动必须附 IPC 对比数据。
- **调试分层**：先用 `LATENCY=1` 隔离总线时序 → 再单测点（`Case=`）→ 最后全量。波形只做最后手段，`$display` 打结构化的周期级日志（PC/ROB/发射/提交事件流）更高效。
- **报告同步写**：阶段 7/8 每轮实验数据直接填进报告的敏感度表格，避免最后两周补数据。
