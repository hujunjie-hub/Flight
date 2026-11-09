# T191-D-REVIEW — V1.9.1-GLOBAL-AUDIT-R3 静态复核记录

> **任务：** `T191-D-REVIEW`（SPEC，唯一Owner D，审核人 A/B/C）  
> **基线：** `V1.9.1-GLOBAL-AUDIT-R3` / Schema 6.0 / Product-Port `0x00010903`  
> **复核范围：** `DOCUMENT_STATIC` + 现有 SCons/Kconfig 接入 + 规范 ABI/职责/预算；不含生产头 ABI、ARM 目标构建、WCET、HIL、飞行  
> **执行时间：** 2026-11-04 22:10–22:33（Asia/Shanghai），工程根 `D:/STM32Project/FMT-Firmware-1.2.0/Flight`  
> **机器证据：** `review_checks.json`（本目录）与 `commands.log`  
> **结论：** 规范主体自洽、证据可复现；发现 **2 项正文缺陷** 与 **2 项工程记录/证据链缺口**。本记录**不是** Gate A 会签，也**不是**独立审核通过。

## 1. 结论摘要

| 项目 | 结果 |
|---|---|
| 复核项 | 59 项自动检查：PASS 46 / FAIL 2 / OBS 6，另 5 项记录为 DRIFT（工作区已按授权进入实现，非规范缺陷） |
| 22 项历史处置（R01–R22） | 22/22 在指定权威正文命中；`document_resolution` 与 09 验收ID、Owner 一致 |
| 78 项全量审查（A58/B8/C12） | 78/78 在 12 号声明的正文位置命中，无缺项 |
| 三轮证据 | round3 在冻结树上可逐字节复现（12/12 文件相同）；round1/2 保留为历史，输入已随修订前移 |
| 规范 ABI | Schema 6.0 / Port 0x00010903 / Header 88B / 55 条注册消息 / 07 slot 表与编译布局逐项相等 |
| 预算 | 栈 52.5KiB、AXI 232/320KiB、DTCM 92.5KiB、M5 62,016B、ICD 42,400B、CPU 54.5705%、900µs 链、DShot 位时序均可复算 |
| 现有 SCons 接入 | 根 `SConstruct`/`SConscript`/`applications/SConscript`/`Kconfig`/`rtconfig.py` 与 03 §0 描述一致；无顶层 CMake |
| 需处置 | **D-REV-01**（11号 Schema 5.0 残留）、**D-REV-02**（07号 pipeline 门限 1900/3000µs 冲突）、**D-REV-03**（证据链无法重建）、**D-REV-04**（任务记录与工作区不一致） |
| 独立签署 | **NOT_RUN**：本执行上下文非 A/B/C，不能替代非作者审核；用户未批准 Gate A / ABI 冻结 |

**判定建议：** 规范可按“有条件接受”继续后续实现，但 D-REV-01/02 必须由所属Owner先修正并重跑本记录检查；D-REV-03/04 属于 D 自身治理项，应在任务关闭前补齐。由于 A/B/C 独立意见缺失，本任务只能进入 `REVIEW`，不能进入 `ACCEPTED`。

**复核后处置（01:3x）：** A-ABI/B-UPSTREAM/C-FMS/A-PLATFORM/**B-ADAPTER** 已接受（A-PLATFORM 静态、B-ADAPTER Host/ARM；DRDY/目标/HIL NOT_RUN）；CR-C01/C02 CLOSED，C-M5 ASSIGNED；**B-REPLAY 解除阻塞**、B-FDIR 待 B-REPLAY；C-M6 待 C-M5/B-REPLAY；CR-C03 待批准。详见第 18–34 节。

## 2. 复核对象与方法

- **通读**：00、01、03 §0、07、09、10、11、12、AGENT、AGENT_HANDOFF、README；对 02/04/05/06/08 采用“类型/字段/门限/公式”定点检索并保留行号（见第 4、5 节）。
- **重跑**：`python spec/tools/audit_spec.py --round 3`；`compile_abi()`；`check_reference_models.run_reference_checks()`；`python spec/validation/T191-D-REVIEW/review_checks.py`。
- **独立实现**：复核脚本不复用 `audit_spec.py` 的断言，自行解析 Markdown/JSON/XML、重算算术、重算外部树哈希。
- **限制**：`check_dialect.py` 需要 pymavlink 2.4.50；当前环境未安装，本轮只核对 `dialect_report.json` 的输入哈希与结论，标为未复现。
- **版本标识**：工程无 Git 元数据（`git rev-parse` 失败），按 10 §3/00 §12.1 使用文件 SHA256 快照（`SHA256SUMS.txt`），不伪造 commit/dirty。

## 3. 证据可复现性

| 检查 | 结果 | 证据 |
|---|---|---|
| round3 全量重跑（冻结树，22:12） | PASS，12/12 报告逐字节相同 | 重跑输出 `{"round":3,"status":"PASS","types":143,"checks":9}`，耗时 2.9s |
| ABI 布局复现 | PASS | `abi_layout.json` SHA256 `b787e580…a73705` 前后一致；143 类型、Header 88B |
| 数学参考复现 | PASS | `mathematical_reference_report.json` SHA256 `87c85d1b…217d49` 前后一致；7 组 |
| 规范输入未变 | PASS | `change_summary.json` 21 个文件 Hash 全部匹配；round3 `inputs_sha256` 全部匹配 |
| 外部树快照 | DRIFT | R3 记录 10,059 文件；生成时快照 10,135 文件，65 个在 R3 后被修改（applications 51 / build 1 / packages 13）。计数为生成时快照，重跑会变 |
| round3 命令现状 | DRIFT | 现在执行报 `AssertionError: outside-spec tree changed`（exit 1） |
| dialect | 未复现 | 环境无 pymavlink；报告记录 2.4.50、13 消息、26 往返、ARM 头编译 PASS |

说明：22:12 的重跑发生在下游实现文件出现之前，因此“12/12 相同”是冻结树上的真实复现；22:16 之后工作区进入实现阶段，外部树快照必然失效。这恰好暴露 `TEST-OUTSIDE-SPEC-INTEGRITY` 无法按 09 要求“为新基线重建”的工具缺口（D-REV-03）。

## 4. 22 项历史处置复核

每项检查：11 号表格行、09 §4.7 验收行、Owner 一致性、正文命中位置。全部 22 项通过。

| 项 | 权威定义（11号） | 正文证据（doc:line） | 结论 |
|---|---|---|---|
| R01 | 02 ControllerInput/Permit；04 M6；06授权；07时效 | 02:1324；06:79；07:425 | 命中 |
| R02 | 02 Mission/FMS/Guidance Port；04任务边界；06 MissionManager | 02:772；06:68 | 命中 |
| R03 | 02定宽消息；07消息槽表；规划 `applications/tools/contract_layout.py`（待… | 02:374；07:318 | 命中 |
| R04 | 07 DShot DMA/Update时序；09 TEST-ACC-061/062 | 07:645；07:157 | 命中 |
| R05 | 05全观测shadow replay；07历史/预算 | 05:1358；05:1356 | 命中 |
| R06 | 02两类时间；07统一时间门限 | 07:421；07:422 | 命中 |
| R07 | 02 SimulatedMeasurementBatch；09仿真需求 | 02:2451；02:374 | 命中 |
| R08 | 07 NVIC/外设资源；09板级台账 | 07:693；07:637 | 命中 |
| R09 | 04 A*完整键/Closed；00容量摘要 | 04:436；04:436 | 命中 |
| R10 | 02 MotionReference/采样；04连续验证/MPC | 02:172；04:444 | 命中 |
| R11 | 02 raw/time/policy/obstacle/receipt Port | 02:144；02:123；02:809 | 命中 |
| R12 | 07唯一资源账；00摘要 | 07:136 | 命中 |
| R13 | 07调度/WCET/接纳；05 replay | 07:181；07:117 | 命中 |
| R14 | 04障碍适用矩阵；02任务flag；08 Profile | 04:424；04:356 | 命中 |
| R15 | 02 SensorConsistencyEvidence；05 probation | 02:646；05:790 | 命中 |
| R16 | 03参数服务；08 CORE/NonSafetySettings | 08:264 | 命中 |
| R17 | 10任务状态机/DoD；AGENT_HANDOFF | 10:89 | 命中 |
| R18 | 10文件Owner/交接协议；AGENT_HANDOFF | HANDOFF:18 | 命中 |
| R19 | 00发布/批准范围；09唯一Gate；10 CR | 09:288 | 命中 |
| R20 | 09需求/测试唯一台账 | 09:221 | 命中 |
| R21 | 00～10及AGENT/README命名 | 00:101；01:23 | 命中 |
| R22 | 04 MPC内近似域；06 PreArm；02原因码 | 04:274；02:848 | 命中 |

## 5. 78 项全量审查复核

12 号清单共 78 项（A01–A58、B01–B08、C01–C12），每项在声明文件中检索修复要点，78/78 命中。下表为逐项证据位置；匹配文本仅作定位，完整内容见正文。

| 项 | 证据位置 | 命中片段 |
|---|---|---|
| A01 | 02:2188 | } ProductFmsConfigPayload; |
| A02 | 02:745 | reference_id |
| A03 | 02:728 | update_permit |
| A04 | 02:558 | NavigationCapability 只描述姿态、航向、位置、速度、高度等估计能力、质量、数 |
| A05 | 02:386 | source_session_id |
| A06 | 02:374 | Header.sequence |
| A07 | 02:374 | Header.sequence |
| A08 | 02:376 | 比力 |
| A09 | 05:1107 | BURST32 |
| A10 | 03:503 | pin_count |
| A11 | 04:428 | time_bin |
| A12 | 04:432 | P00 |
| A13 | 04:444 | 480ms |
| A14 | 02:2289 | const LandedStateSnapshot *landed_state |
| A15 | 04:21 | Rate PID |
| A16 | 04:345 | ω_sp |
| A17 | 04:301 | MpcOutputSnapshot |
| A18 | 07:110 | 1053 |
| A19 | 07:114 | 2053 |
| A20 | 04:397 | BENCH_INHIBIT |
| A21 | 02:1078 | get_health |
| A22 | 02:333 | ClockInstant |
| A23 | 02:864 | FaultEvidenceEvent |
| A24 | 02:1575 | 非规范性设计草案 |
| A25 | 00:489 | 本地14.3.1 |
| A26 | 04:282 | 190行 |
| A27 | 07:394 | 293 |
| A28 | 05:1365 | N-1 |
| A29 | 05:1219 | Qc |
| A30 | 05:1518 | error retraction |
| A31 | 05:1209 | Phi-angle |
| A32 | 05:1156 | 天线速度杆臂 |
| A33 | 05:1262 | GNSS_BACKEND=DISABLED |
| A34 | 05:1277 | 压力高度 |
| A35 | 05:793 | 4倍 |
| A36 | 05:729 | DATA_CNTR递增不能单独判卡死 |
| A37 | 06:638 | PreArm ID |
| A38 | 06:482 | 绝对100ms |
| A39 | 06:451 | 只有引用相对Home高度 |
| A40 | 02:1196 | NavigationOriginSnapshot |
| A41 | 06:410 | LandDetector |
| A42 | 06:636 | 4个长操作tracker |
| A43 | 06:10 | FCU.xml |
| A44 | 06:593 | 轮换 |
| A45 | 08:266 | HEALTH_CONFIRMED |
| A46 | 08:268 | maintenance_epoch |
| A47 | 08:244 | SHA256 |
| A48 | 03:325 | DIRECT_BSP |
| A49 | 07:645 | active零 |
| A50 | 07:653 | 受控RCC读回 |
| A51 | 07:637 | HT/TC |
| A52 | 07:316 | 8项capture池 |
| A53 | 07:62 | 1152 |
| A54 | 10:96 | READY_FOR_ASSIGNMENT→CLOSED |
| A55 | 09:385 | 不继承 |
| A56 | 05:1133 | 8291a93 |
| A57 | 05:1454 | 不能预填 |
| A58 | 00:8 | V1.8 |
| B01 | 02:366 | ALTITUDE_ATTITUDE |
| B02 | 05:477 | 100ms..1s |
| B03 | 02:1062 | EstimatorHealth |
| B04 | 07:348 | 48KiB |
| B05 | 04:485 | 512B |
| B06 | 05:1354 | 2304 |
| B07 | 04:481 | 旋转向量 |
| B08 | 05:1255 | 1100 |
| C01 | 02:374 | 外层message_type固定SensorMeasurement(13) |
| C02 | 02:463 | quality_flags bit |
| C03 | 02:2604 | QuadSensorProcessingConfig |
| C04 | 02:2611 | 同一发布世代 |
| C05 | 04:304 | 25ms |
| C06 | 05:481 | mapping_epoch |
| C07 | 04:428 | time_bin |
| C08 | 02:2628 | extern "C" |
| C09 | 07:348 | 20B |
| C10 | 09:381 | project_structure_report |
| C11 | 02:2616 | 17个 |
| C12 | 02:2630 | 参考点固定为机体重心CG |

> 说明：`A06/A07` 共用 `Header.sequence` 冻结语句、`A20` 在 00/03/04/08 多处出现，均属同一修复的多处落点；命中不代表语义充分，语义判断见第 11 节。

## 6. 规范 ABI 与版本

| 检查 | 结果 |
|---|---|
| 02 宏 | `FC_CONTRACT_SCHEMA_MAJOR=6`、`MINOR=0`、`PRODUCT_CONTRACT_REVISION=0x00010903`、`PORT_API_REVISION=0x00010903` |
| Header | 88B；143 个规范类型；C11/C++14 一致；无重复 typedef |
| 消息注册 | 55 条 Header-bearing 消息，ID 唯一；FCU.xml 13 条私有消息 43000–43012 |
| 07 slot 表 | 29 类对象 slot 与编译 `sizeof` 全部相等 |
| 版本一致性 | 00/02/10/12/README/AGENT/HANDOFF 均为 6.0；**11 号仍写 5.0**（D-REV-01） |
| 旧版本号 | `0x00010901` 仅出现在 00 的升级说明与 11 的历史小节，均有历史限定 |

## 7. 职责、任务记录与治理

| 检查 | 结果 |
|---|---|
| 文件 Owner 表 | 10 号覆盖 00–12/AGENT/HANDOFF/README/tools/contracts/validation |
| Gate 唯一性 | Gate A–F 表只在 09 出现一次 |
| 任务记录 | 15 条，字段齐全；Owner 唯一且不在 reviewer 内；依赖为 DAG；baseline 一致；全部 READY/ack=null/NOT_RUN |
| 状态机 | 10 号七状态与 HANDOFF 枚举一致；无 CANCELLED/DONE |
| 记录与工作区 | **不一致**：A-ABI/B-UPSTREAM/B-REPLAY/C-FMS 的 source_paths 下已出现新文件，记录仍为 READY（D-REV-04） |

## 8. 预算、时序与资源算术

| 项 | 复算 | 结论 |
|---|---|---|
| 线程栈 | 25 项合计 57,856B；去 FinSH 53,760B=52.5KiB；再去 Timer 52,736B=51.5KiB | PASS |
| AXI | 10 项合计 232KiB，320−232=88KiB（27.5%） | PASS |
| DTCM | 52.5+4+14+14+8=92.5KiB，余 35.5KiB ≥32KiB | PASS |
| M5 planner | 62,016B（含 1KiB primitive 容量）<64KiB，余 3,520B；逐项相加为 61,952B（差 64B） | PASS（OBS-02） |
| ICD arena | 42,400B ≤48KiB（编译布局） | PASS |
| CPU | 450µs×1053 + 35µs×2053 = 54.5705%，余 0.4295% | PASS |
| 时延链 | 5+320+40+60+250+100+35=810µs，+90µs=900µs | PASS |
| 物理窗 | 3000+900+100=4000µs <5000µs；5000+2500=7500µs | PASS（但 07 表值见 D-REV-02） |
| DShot | 275MHz：300k→917/344/688、ARR916；600k→458/172/344、ARR457 | PASS |
| Burst 时间 | 22B=184.3µs、34B=284.9µs | PASS |

## 9. 现有 SCons/Kconfig 接入

| 检查 | 结果 |
|---|---|
| 根构建 | `SConstruct` 保留 `PrepareBuilding`/`DoBuilding` 并显式接入 `libraries/HAL_Drivers/SConscript`；产物 `Flight.elf/.axf/.out`、`Flight.bin/.map` |
| 递归规则 | 根/应用 `SConscript` 仅接入含 `SConscript` 的子目录，不做全树 Glob |
| Kconfig | 依次 source `rt-thread`、`packages`、`libraries`、`board` |
| 工具链 | `rtconfig.py` 默认 Keil，`RTT_CC`/`RTT_EXEC_PATH` 可覆盖；GCC 路径存在；实际 ARM GCC 14.3.1 与 12-A25 一致 |
| 配置 | `rtconfig.h`：优先级32、tick 1000、GPIO/UART1/FinSH 已启用，与 03 §0.1 一致 |
| CMake | 工程根与 `applications/` 无 CMake；仅 `doc/` 官方用例含 |
| 目录 | R3 基线时 `applications/` 仅 `main.c`/`SConscript`；当前已生成 contracts/core/plugins/products/tests（DRIFT） |

## 10. 工具与证据链

| 缺口 | 影响 |
|---|---|
| `audit_spec.py` 断言全部任务 READY/ack=null | 第一次合法状态迁移即失败；`TEST-V191-AGENT` 的“迁移合法性”无检查器 |
| 无 `protected_sources_report.json` / `change_summary.json` 生成器 | 09 要求“新基线重建”，但没有可执行命令；任何实现阶段文件都会使 round3 断言失败 |
| `--round N` 总是重写报告 | 重跑 round1/2 会覆盖历史证据，只能整体回滚 |
| dialect 依赖外部 pymavlink | 当前环境不可复现；报告未记录安装方式/路径 |

## 11. 发现（需处置）

### D-REV-01 · P2 · Owner D · 11 号仍声明 Schema 5.0

- 位置：`spec/11_审核问题闭环与变更追踪.md:9`。
- 原文：“公共字段/Port改变后Schema升为5.0，Product/Port revision为0x00010903”。
- 冲突：R3 基线为 Schema 6.0；`0x00010903` 属 R3；R2 才是 5.0/0x00010901（11:232 有历史说明）。
- 影响：按 11 号执行会构建 Major=5 消费者，与 02 的 `FC_CONTRACT_SCHEMA_MAJOR=6` 不兼容；ABI 冻结/实现任务可能取错版本。
- 建议：改为“Schema升为6.0，Product/Port revision为0x00010903”；或明确写“R2升5.0；R3再升6.0/0x00010903”。

### D-REV-02 · P2 · Owner A · 07 号 pipeline 门限自相矛盾

- 位置：`spec/07_实时调度与板级硬件资源设计.md:419` 表行 `MAX_IMU_PIPELINE_DELAY_US = 1900us`；同一节 `07:432` 写“pipeline≤3000us”。
- 外部一致性：00:385、05:1109、11:86 均为 3ms；A09 明确“1.8ms 管线预算不成立”并改为 3ms。
- 影响：实现按表取 1900µs 会比 00/05/11 声明的 3ms 更早拒绝配置（ADIS 默认 group delay 约1.29–1.57ms 尚可，但轴对齐/标定余量受限）；按 3ms 实现又与表冲突。同名硬门限只能有一个值。
- 建议：表值改 3000µs 并与 07:432 对齐；若确实需要“延迟+对齐”两个不同常量，则拆名并在 05/00 同步。修改后重跑本记录 BUD-12/BUD-08。

### D-REV-03 · P2 · Owner D · 证据链无法在新基线重建

- 事实：R3 冻结树上 round3 可逐字节复现；下游实现文件出现后同一命令报 `outside-spec tree changed`（exit 1）。
- 缺口：交付工具中没有 `protected_sources_report.json`/`change_summary.json` 的生成器；09 §5 要求“为新基线重建”，但无命令/脚本。
- 叠加：`audit_spec.py` 把“全部任务 READY”写成断言，合法迁移即断；`TEST-V191-AGENT` 无迁移合法性检查器。
- 建议：新增 `--baseline-write` 类显式生成模式（含文件清单与 tree hash），将“未派发”断言改为状态机合法性检查；在 09 记录重建命令。

### D-REV-04 · P3 · Owner D · HANDOFF 与工作区不一致

- 事实：复核期间（22:11–22:21）`applications/contracts`、`applications/products/QUAD_X4_H723`、`applications/tests/abi`、`applications/core/m4`、`packages/KF-GINS` 出现新文件。
- 记录：相关任务仍为 `READY_FOR_ASSIGNMENT`、`acknowledgement=null`，且 T191-A-ABI/B-UPSTREAM 声明依赖未接受的 T191-D-REVIEW。
- 影响：任务台账不能反映真实执行；D 合并状态时会把“已产出”误判为“未开工”，或把未审核产物当作已接单证据。
- 建议：按 10 §6 由 D 核对实际接单/基线后更新记录（或要求停止未授权工作）；本记录不代替该处置。

## 12. 观察项（建议，不阻塞）

| 编号 | 位置 | 内容 |
|---|---|---|
| OBS-01 | 07:832 | “ICD 参考占用41376B”是第二轮账面值；当前编译布局 42,400B（C09）。两者均<48KiB，建议标注“R2 值/当前值”。 |
| OBS-02 | 07:274–279 | M5 合计 62,016B 与逐项相加 61,952B 差 64B（primitive 容量 1KiB vs 960B）；建议在合计行注明按容量计。 |
| OBS-03 | 00:510 | KF-GINS 行写“完整commit待锁定”，但 05:1133 已固定 `8291a93…`；建议区分“设计输入已锁定/导入包待锁源码Hash”。 |
| OBS-04 | 09 §4.7 | R17–R21 是治理/文档控制，却标 `IMPLEMENTATION`；与 `TEST-V191-AGENT`（静态检查）重叠，建议改 `DOCUMENT_STATIC`。 |
| OBS-05 | 工具 | `--round N` 无 verify-only 模式，重跑会覆盖历史报告；建议增加只读校验。 |
| OBS-06 | 工具 | dialect 报告未记录 pymavlink 安装路径/方式；建议在 09 写明复现环境。 |

## 13. 复核期间的工作区漂移

| 时间 | 事件 |
|---|---|
| 22:12 | 冻结树上 round3 重跑，12/12 报告相同（本节所有 PASS 的前提） |
| 22:11–22:16 | `packages/KF-GINS/` 上游快照与 trim 源码出现 |
| 22:16–22:20 | `applications/contracts/`、`products/QUAD_X4_H723/`、`tests/abi/`、`core/m4/` 出现；`build/m4_abi/` 生成 `.o` |
| 22:21+ | 外部树持续增加（生成时 10,135 文件）；round3 命令失败 |

这些文件不是本任务交付物，也未被本记录审核或接受；仅用于说明证据基线已变化。

## 14. 未执行项与批准状态

| 项 | 状态 | 说明 |
|---|---|---|
| A/B/C 非作者独立意见 | NOT_RUN | 本执行上下文只有 D；不得代签 |
| 用户 Gate A / ABI 冻结批准 | NOT_RUN | 无批准记录 |
| 生产 Host/ARM ABI | NOT_RUN | 仅规范块 ARM 编译 |
| ARM 目标构建 / map / WCET / RTA / 栈水位 | NOT_RUN | 未构建固件 |
| HIL / 无桨台架 / 飞行 | NOT_RUN | 未执行 |
| TEST-SPEC-DIALECT 复现 | NOT_RUN | 缺 pymavlink |
| 09 台账登记 | 未执行 | 建议在任务接受时由 D 增加 `TEST-T191-D-REVIEW` 行；本记录不改 09 |

## 15. 回滚

- 本任务**未修改** 00–12、AGENT、HANDOFF 或任何固件/驱动/包；仅在 `spec/validation/T191-D-REVIEW/` 新增复核记录与脚本。
- 回滚：删除 `spec/validation/T191-D-REVIEW/` 即恢复 R3 证据原状（12 个 JSON 未被改写，Hash 与 22:12 快照相同）。
- 不得只回滚 11 号单行修复而保留新 ABI 消费者；若后续按 D-REV-01/02 修改正文，应整体重建 round3 证据。

## 16. 状态请求（供 D 合并，不直接改写 HANDOFF）

```json
{
  "task_id": "T191-D-REVIEW",
  "baseline": "V1.9.1-GLOBAL-AUDIT-R3",
  "update_sequence": 3,
  "requested_transition": "READY_FOR_ASSIGNMENT -> ASSIGNED -> IN_PROGRESS -> REVIEW (same session)",
  "assignee": "D",
  "acknowledgement": "用户于2026-11-04指示“执行Agent D任务”；D按唯一无前置任务T191-D-REVIEW接单",
  "deliverable_refs": [
    "spec/validation/T191-D-REVIEW/T191-D-REVIEW_report.md",
    "spec/validation/T191-D-REVIEW/review_checks.json",
    "spec/validation/T191-D-REVIEW/review_checks.py",
    "spec/validation/T191-D-REVIEW/commands.log",
    "spec/validation/T191-D-REVIEW/SHA256SUMS.txt"
  ],
  "actual_commands": [
    "python spec/tools/audit_spec.py --round 3",
    "python spec/validation/T191-D-REVIEW/review_checks.py"
  ],
  "results": {
    "checks": 59,
    "pass": 46,
    "fail": 2,
    "observations": 6,
    "drift": 5
  },
  "reviewer": [
    "A",
    "B",
    "C"
  ],
  "review_status": "NOT_RUN",
  "blocked_by": [],
  "resume_state": null,
  "next_owner": "D",
  "next_action": "A/B/C 非作者复核；Owner 处置 D-REV-01/02；D 修复 D-REV-03/04 并核对 HANDOFF 与工作区"
}
```

## 17. 复核后处置（22:5x，供 A/B/C 复核）

本节记录 D 对自身可处置项的闭环。它不改变第 11 节发现的原始结论，也不代表 A/B/C 复核或用户批准。

| 项 | 处置 | 证据 |
|---|---|---|
| D-REV-01 | **已更正**：11:9 “Schema升为5.0”→“6.0”，并写明旧Major=5消费者必须迁移 | `spec/11_审核问题闭环与变更追踪.md` §1 |
| D-REV-03 | **已修复工具/流程**：`audit_spec.py` 改为状态机合法性检查（不再断言全部 READY）；外部树快照改为可选 `--outside-baseline`；新增 `--write-outside-baseline` 生成器；新增 `--label` 保留历史报告 | `python spec/tools/audit_spec.py --round 3 --label t191-d-review` → PASS（9 checks，含 `handoff_task_records_legal`、`protected_sources_report:NOT_ENFORCED`）；`spec/validation/t191-d-review_report.json` SHA256 `ba3ccf1c…f9c0d81` |
| 任务记录 | **已合并**：T191-D-REVIEW `READY→ASSIGNED→IN_PROGRESS→REVIEW→ACCEPTED`、`verification_status=PASS`、`update_sequence=5`；A-ABI `REVIEW/seq4`；A-PLATFORM `ASSIGNED/seq3` | `spec/AGENT_HANDOFF.md` |
| 09 台账 | `TEST-T191-D-REVIEW` = PASS（DOCUMENT_STATIC，附接受记录）；不声称 Gate A | `spec/09_需求验证实施与发布管理.md` §4.5/§4.8 |
| D-REV-02 | **已关闭**：A 于 23:10 将 07:419 改为 3000us；D 复核 07:419/432 一致、无 1900us 残留，07 SHA256 `aacf4ca9…` | 第 18.3 节 |
| D-REV-04 | **部分闭环**：A-ABI 已合并、A-PLATFORM 已排；B-UPSTREAM 待 B 确认；C-FMS/M5 状态请求待合并 | 第 19 节 |
| R3 历史证据 | **已归档**：原 round3/change_summary/protected_sources 副本在 `r3_archive/`；当前 `round3_report.json`、`change_summary.json` 为重建后基线 | 第 19 节 |

说明：`change_summary.json` 仍是 R3 时点的变更摘要，对 09/11/HANDOFF/tools 的逐文件 Hash 已属历史；当前基线的可复核输入以 `spec/validation/t191-d-review-r2_report.json` 的 `inputs_sha256` 为准。`review_checks.json` 是 22:31 时点快照；`review_checks_post.json` 是处置后复跑结果。

## 18. 非作者复核意见与处置（23:2x，供 Gate A）

本节只登记 A/B/C 意见和 D 的处置，不构成会签。A/B 意见原文分别见 `applications/tests/abi/reviews/A_REVIEW_T191-D-REVIEW.md`、`build/validation/review_b/T191-D-REVIEW_B_REVIEW.md`；C 的书面意见与 C-M5/FMS 状态请求待补。

### 18.1 A 的意见（ACCEPT_WITH_CONDITIONS，14/14 PASS）

| 条件 | D 执行情况 |
|---|---|
| C1 复跑复核脚本并更新 ABI-02/BUD-12 | 已复跑：`review_checks_post.json`；ABI-02、BUD-12 均 PASS；EVID-01/02 因 R3 快照属历史而保持 FAIL（预期） |
| C2 07 更正后重建 spec 证据 | 已重建：`python spec/tools/audit_spec.py --round 3 --label t191-d-review-r2` → PASS（9 checks）；07 SHA256 `aacf4ca9…`，与 A 声明一致 |
| C3 合并 A-ABI resubmission | 已合并：HANDOFF `T191-A-ABI` = ASSIGNED / IN_PROGRESS / seq 3 / ack 已记录 |

### 18.2 B 的意见（8 项，2 项阻塞）

| 项 | 处置 | 状态 |
|---|---|---|
| B-R01 | 接受；05 §7 坐标系/矩阵次序由 B 增补 | 待 B |
| B-R02 | **关闭**（CR-V191-B02）：当前 H723 使用项目自带固定维数层、不链接 Eigen；00 §12.3 已改为条件式；05 §6.2 由 B 对齐 | B 改 05 |
| B-R03 | **关闭**（CR-V191-B03）：实测 ARM GCC 14.3.1 默认最小化枚举（`__ARM_SIZEOF_MINIMAL_ENUM=1`，断言失败；`-fno-short-enums` 通过）；00 §12.1 已列强制旗标 | A 改 rtconfig.py |
| B-R04 | 接受为实施约束（07/05 不保留每槽绝对时间） | 已闭环 |
| B-R05 | 打开：`applications/Kconfig` 未聚合 `core/*`、`plugins/*` | A |
| B-R06 | 打开：`applications/calibration/` 未创建 | B |
| B-R07 | 待用户决定是否新增 GNSS 原生/BMM350 验证任务 | 用户 |
| B-R08 | 打开：28KiB 预算余量 464B，需增列每实例开销 | A/B |

### 18.3 D-REV-02 独立复核（A 修正后）

- 07:419 表值与 07:432 正文均为 `3000us`，全文已无 `1900us` 残留；与 00:385、05、11 一致。
- 结论：D-REV-02 关闭；A 的修正不代替 Gate A 会签，也不改变预算仍需目标实测的事实。

### 18.4 当前状态

- `T191-D-REVIEW`：REVIEW / IN_PROGRESS / seq 4；A、B 意见已记录，C 书面意见与用户 Gate A 批准仍 NOT_RUN。
- `T191-A-ABI`：ASSIGNED / IN_PROGRESS / seq 3；待 D 独立复核 HOST_PRODUCTION_ABI 证据后进入 REVIEW。
- `T191-B-UPSTREAM`：READY / seq 3；D 已记录 B 复核并关闭 B-R02/B-R03，等待 B 确认 prompt 后 ASSIGNED。
- 证据：`t191-d-review-r2_report.json` SHA256 `1e41f0ea…8fbb12`；`review_checks_post.json` SHA256 `46c80a65…1da42a`。

## 19. 用户指示的接受与重建（23:4x）

| 步骤 | 执行结果 | 证据（SHA256 前 16 位） |
|---|---|---|
| ① 合并 A-ABI 补交（dedup `T191-A-ABI:2`）与 A 的 D-REVIEW 意见 | HANDOFF `T191-A-ABI` = `REVIEW` / IN_PROGRESS / seq 4；A 意见（ACCEPT_WITH_CONDITIONS，C1-C3）记录在 §18.1 | `applications/tests/abi/status_request_T191-A-ABI.json`、`A_REVIEW_T191-D-REVIEW.md` |
| ② 07 修正后重建证据 | `python spec/tools/audit_spec.py --round 3` → PASS（9 checks，21 inputs）；`review_checks.py --label review_checks_post_accept` → **0 FAIL** / 6 OBS / 2 DRIFT；`change_summary.json` 重建（21 文件，6 MODIFIED）；`protected_sources_report.json` 刷新（10,309 文件） | round3 `1b840879…`；change_summary `258e8dbb…`；protected `3b2f177f…`；post_accept `ff9c6821…` |
| ③ `T191-D-REVIEW` 记 ACCEPTED | HANDOFF = `ACCEPTED` / PASS / seq 5；09 `TEST-T191-D-REVIEW` = PASS；接受依据 A/B/C 三份非作者意见（C-REV-01/02/03、B-R05..R08 仍未关闭） | 09 SHA256 `ff7102d7…` |
| ④ A-ABI 记 REVIEW，排 A-PLATFORM | `T191-A-ABI` = `REVIEW` / seq 4；`T191-A-PLATFORM` = `ASSIGNED` / seq 3（阶段1证据已收，阶段2待 A-ABI ACCEPTED） | `status_request_T191-A-PLATFORM.json` |

**边界：** ACCEPTED 只表示复核任务交付被接受，不表示 C-REV-01/02/03 或 B-R05..R08 已关闭；Gate A 与 ABI 冻结仍需 A/B/C 会签和用户按范围真实批准。R3 原证据已归档在 `r3_archive/`，重建只针对当前基线。

## 20. B 任务合并与 D-REV-04 关闭（23:5x）

| 任务 | 合并结果 | seq | 前置（保留） |
|---|---|---:|---|
| T191-B-UPSTREAM | ASSIGNED，ack/证据已记录 | 4 | 无（D-REVIEW 已 ACCEPTED） |
| T191-B-ADAPTER | ASSIGNED，ack/证据已记录 | 3 | A-PLATFORM、B-UPSTREAM |
| T191-B-REPLAY | ASSIGNED，ack/证据已记录 | 3 | B-ADAPTER |
| T191-B-FDIR | ASSIGNED，ack/证据已记录 | 3 | B-REPLAY |
| T191-C-FMS | ASSIGNED，ack/证据已记录 | 3 | A-ABI、B-ADAPTER |
| T191-C-M5 | 记录 NO_CHANGE 准备交付，保持 READY | 3 | C-FMS |

- **D-REV-04 关闭**：A/B/C 已知提交均已进入 HANDOFF，不再存在“已交付但记录仍 READY（除 C-M5 的 NO_CHANGE 准备）”的缺口。
- **B-R02（Eigen）**：维持 CR-V191-B02 决策——H723 使用项目自带固定维数层、不链接 Eigen；Eigen 5.0.1 仅候选；05 §6.2 由 B 对齐。
- **B-R03（-fno-short-enums）**：维持 CR-V191-B03——ARM GCC 14.3.1 默认最小化枚举，00 §12.1 已列为强制旗标；`rtconfig.py` 仍待 A 落实，故“决策关闭、构建动作 OPEN”。
- 证据：round3 `6f7c52a3…`；change_summary `9708b639…`；protected `e90a4115…`；post-b `6397b9dc…`（59 项 0 FAIL / 6 OBS / 3 DRIFT，DRIFT 为既有实现目录与新写入）。

## 21. C 提交合并、C 复核 revision 2 与 CR 登记（00:0x）

| 事项 | 结果 |
|---|---|
| C 四份状态请求 | 按 `applications/core/status_requests/index.json` 合并：C-FMS rev3 → ASSIGNED/seq4；C-M5 rev2（NO_CHANGE）→ READY/seq4，登记 NONE-source 证据；C-M6/M7 NOT_STARTED → READY/seq3，登记阻塞项（MPC solver、ControlModelPort、C-M5/B-REPLAY、DShot 契约） |
| D-REV-04 | **C 部分关闭**：至此 A/B/C 已知提交均已进入 HANDOFF；M6/M7 为明确 NOT_STARTED，M5 为 NO_CHANGE 准备，不再视为“记录与工作区不一致” |
| C 复核 revision 2 | 已并入：`C_REVIEW_SUBMISSION.json`（22 项 = 15 PASS / 1 FAIL / 6 OBS）；D-REVIEW 记录 evidence 增加该提交，`update_sequence=6` |
| CR-V191-C01（mission_epoch） | 已在 11 §4.1 登记：`ProductOperationPayload` 增加 `mission_epoch`+valid_mask，M4 拒绝旧 epoch；ABI 变化须升级并重跑 A 锁；PROPOSED/NOT_RUN |
| CR-V191-C02（围栏入口） | 已在 11 §4.1 登记：首版采用编译期 Product 配置，不新增运行时命令；Companion 只能提交候选；PROPOSED/NOT_RUN |
| B-R05（Kconfig 聚合） | **已关闭**：`applications/Kconfig` 聚合 `core/Kconfig`，后者以 `rsource` 接入 m4/m5；移除重复 `FMS_QUAD_X4_ENABLE`。解析检查：10 文件/36 符号，`FMS_QUAD_X4_ENABLE`/`KF_GINS_QUAD_ENABLE` 可达，无缺失/无重复（RT-Thread 环境变量除外） |
| B-R03（-fno-short-enums） | 决策保持；`rtconfig.py` 仍待 A，构建动作 OPEN |

- 证据：round3 `9aa72643…`；change_summary `0b7b4b78…`；protected `b8dbccb5…`；post-c `3984365b…`（59 项 0 FAIL / 6 OBS；EVID-03 因工作区持续写入可能显示 DRIFT）；09 `b764697d…`、11 `712d874f…`。
- 边界：C-REV-01/02 的正文修改与 ABI 升级尚未实施，Gate A、ABI 冻结与任务 CLOSED 仍待真实会签/用户批准。

## 22. A-ABI 复核暂停与 A-PLATFORM 阶段2（00:0x）

| 步骤 | 结果 |
|---|---|
| A-PLATFORM rev2 | 已合并（NO_CHANGE / seq5）：B-R03 由 A 在 `rtconfig.py` 落实、B-R05 由 D 的 Kconfig 聚合关闭；但阶段2 **未放行** |
| A-ABI 独立复核（CR-C01 前布局） | `python applications/tests/abi/abi_report.py` 复跑逐项相同：142类型、ARM `f1e36c57…`、Host `e288b875…`、lock PASS；`-fno-short-enums` 缺失时 `fc_contract_version.h:45` 静态断言失败、加旗标通过（build-flag 检查成立） |
| CR-C01 后复核 | A 在 02/生产头加入 `mission_epoch` 后：spec 146类型、新hash ARM `180aaae8…`/Host `1d2019…`、`spec_mismatches=0`，但 `abi_lock` **FAIL**；02 已升 `0x00010904`，而 Schema 仍 6.0、00/HANDOFF 基线串仍 `0x00010903`、UGV/USV 预览块仍 `0x00010903` |
| D 处置 | 按 10 将 A-ABI `ACCEPTED→IN_PROGRESS`（验收撤回），09 `TEST-A-ABI-HOST`=IN_PROGRESS；A-PLATFORM 保持 `blocked_by=[T191-A-ABI]`，阶段2暂停 |
| 需 A 完成 | ①Schema Major/Minor 决策并记入 CR；②`abi_report.py --emit-lock` 重建锁；③同步 00/HANDOFF/AGENT/README 基线串；④重交状态请求。之后 D 复验并恢复 A-ABI ACCEPTED、放行阶段2 |

- 证据：round3 `614b70b4…`（146类型）；change_summary `2624d9f0…`；protected `0c25f6cb…`；post-d3 `9375f855…`。`review_checks` 的 ABI-01 FAIL 是期望的检测结果：02 的 `0x00010904` 与基线串/锁不一致。

## 23. 官方数据集复现登记与 A-PLATFORM 推进（00:1x）

| 事项 | 结果 |
|---|---|
| B 官方数据集复现 | `REQ-NAV-KFG-001`=PASS（Host）：commit `8291a93e…`；数据集三文件Hash与 `dataset_run.json` 一致（D 直接复核）；全量输出三文件Hash一致（Navresult `be3fddb7…`、STD `1dca8e1f…`、IMU_ERR `67d1afa5…`）；672,923 历元、truth RMS 0.039m |
| REQ-NAV-KFG-013 | IN_PROGRESS：Pristine vs Project Host 24k历元 dp≤1.14e-7m；D 重跑 `dataset_dual_compare.exe` 得到同一组数值；H723 与逐patch差分仍 NOT_RUN |
| 证据清单问题 | `build/validation/kf_gins_golden/SHA256SUMS.txt` 对 `DATASET_EVIDENCE.md`/`dataset_run.json` 已过期；已要求 B 刷新，不影响本次登记（D 按实际Hash复核） |
| A-PLATFORM | 阶段1证据登记 `TEST-A-PLATFORM-SVC`=IN_PROGRESS（raw 72/0、time 114/0、ARM C99 PASS）；A-ABI 0x00010904 接受后 `blocked_by` 清空，阶段2放行 |
| A-ABI 0x00010904 | 145类型、ARM `588707eb…`、Host `2e8c855d…`、spec `55ce3faf…`、lock `4781f28b…`、firmware flags PASS；D 独立复跑一致；Schema 仍 6.0（02 §1141 说明仅Product payload变化），00/HANDOFF/AGENT/README 基线串由 D 同步为 `0x00010904` |

- 证据：round3 `c2f94d89…`；change_summary `06a0b21e…`；protected `7dc0fb40…`；post-f `ba5d4b80…`（59项 0 FAIL）。
- 边界：Gate C/B 会签、H723 三构建差分、用户 ABI 冻结批准仍 NOT_RUN。

## 24. T191-B-UPSTREAM 审核接受与 B-ADAPTER 路由（00:2x）

| 事项 | 结果 |
|---|---|
| 上游锁定 | `UPSTREAM_LOCK.json`：commit `8291a93e…`、tree `6d255a9c…`、archive `183a86c0…`、GPL-3.0-or-later；10 个 pristine 文件本地Hash、锁内Hash、spec 记录三者全部一致 |
| Host 测试 | `build/validation/kf_gins_upstream/` 5 项：jacobian 220、core 2636、port 179、contract coexistence、upstream compare 426；合计 3,341 检查 0 失败 |
| 数据集证据 | `REQ-NAV-KFG-001`=PASS（Host，已独立复核数据集/输出Hash与dual比较）；`REQ-NAV-KFG-013`=IN_PROGRESS（H723 未跑）；golden `SHA256SUMS.txt` 对 2 文件过期，B 须在 Gate C 前刷新（D 按实际Hash复核，不阻塞接受） |
| B-R01 / B-R06 | **关闭**：05 §3.2 明确 `L=C·l`/`skew(L)` 次序；`applications/calibration/` 与 README 已建立 |
| 任务状态 | `T191-B-UPSTREAM`=ACCEPTED/PASS/seq6（scope IMPLEMENTATION；H723/WCET/SIL/Gate C NOT_RUN）；09 `TEST-B-UPSTREAM`=PASS |
| B-ADAPTER 路由 | `T191-B-ADAPTER` 保持 ASSIGNED/seq4，`blocked_by` 仅剩 `T191-A-PLATFORM`；A-PLATFORM 验收后 B 开始实际器件burst/TimeService绑定 |

- 证据：round3 `9ab77718…`；change_summary `117e7633…`；protected `fd0d2a2d…`；post-g `8651ca4c…`（59项 0 FAIL）。

## 25. C-FMS rev4 合并与 CR-C01/C02 处置（00:3x）

| 事项 | 结果 |
|---|---|
| C-FMS rev4 | 合并为 `REVIEW`/IN_PROGRESS/seq5：`T191-C-FMS` 不再有前置阻塞；Host 17/17、240 checks/0 failures，ARM 6 对象 + `ARM_COMPILE_AND_ABI_OK` |
| CR-V191-C01 | **IMPLEMENTED**：`ProductOperationPayload.mission_epoch` + M4 旧/缺 epoch 拒绝（m4_engine.c:497–506），测试 `mission_epoch_binding` PASS；ABI 已升 `0x00010904`、锁 `4781f28b…` |
| CR-V191-C02 | **IMPLEMENTED**：`ProductGeofenceConfig` 编译期加载（`quad_x4_fms_configure_product_geofence`），无运行时 `SET_GEOFENCE`；测试 `geofence_required` PASS |
| 09 登记 | 新增 `TEST-C-FMS-EXEC`=PASS（IMPLEMENTATION，Host/ARM）；目标构建/WCET/SIL/HIL、Gate D 与独立复核仍 NOT_RUN |
| 交付文档问题 | `T191-C-FMS_DELIVERY.md` 头部基线仍写 `0x00010903`（实际 0x00010904）；建议 C 更新文档，不影响证据Hash与代码 |

- 证据：round3 `1ebcb4a9…`；change_summary `fbae8f7a…`；protected `078f1312…`；post-h `f7a82b64…`（59项 0 FAIL）。

## 26. CR 关闭、B-R08 与版本串同步（00:4x）

| 事项 | 结果 |
|---|---|
| CR-V191-C01 | `design_disposition=SPECIFIED`，`closure_status=CLOSED`（02/生产头/M4 校验、0x00010904 锁、Host 17/240、ARM PASS）；独立复核与用户批准仍 NOT_RUN |
| CR-V191-C02 | `design_disposition=SPECIFIED`，待 Gate D 复核后关闭；编译期围栏入口已实现，无运行时命令 |
| 09 复核行 | C-REV-01=`CR-C01 CLOSED`；C-REV-02=`CR-C02 SPECIFIED（待复核）`；09:349 P0-01 版本改 `0x00010904`；11:9 同步 `0x00010904` |
| B-R08 | **关闭**：07:350/05:1488 增列 per-instance 1,256B，active/shadow 各15,360B；DTCM 94.5KiB、余33.5KiB；00:381 由92.5同步为94.5；复核脚本 BUD-03 同步 |
| 重建 | `audit_spec.py --round 3` PASS（146类型）；change_summary 重建；`review_checks --label review_checks_post_i` 59项 0 FAIL |

- 证据：round3 `f803b463…`；change_summary `0522ad49…`；protected `e256e14c…`；post-i `3fb63e6d…`。

## 27. B-UPSTREAM 复验与 golden 清单关闭（00:5x）

- `T191-B-UPSTREAM` 维持 ACCEPTED/PASS/seq6；B-R01/B-R06 维持关闭。
- `build/validation/kf_gins_golden/SHA256SUMS.txt` 已由 B 刷新：7/7 条目与实际文件Hash一致（此前 `DATASET_EVIDENCE.md`/`dataset_run.json` 过期项关闭）；09 与 HANDOFF 的条件文字已同步移除。
- `T191-B-ADAPTER` 维持 ASSIGNED/seq4，`blocked_by` 仅 `T191-A-PLATFORM`；A-PLATFORM 阶段2证据尚未交付，暂不可开工。
- 并发说明：重建期间 06/08 被 C 更新（CR-C02 正文），D 已在新树上重跑 `--round 3` 与复核脚本并通过。

- 证据：round3 `690938e1…`；change_summary `ff0d98de…`；protected `f37c6240…`；post-j `c6dc8de5…`（59项 0 FAIL）。

## 28. 契约diff复核合并、C02 入口文字与观察处置（00:6x）

| 事项 | 结果 |
|---|---|
| 合并 A 的契约diff复核 | `C_CONTRACT_DIFF_A_REVIEW.md` + `c_contract_diff_check.json`（20/20 PASS）登记为 09 `TEST-CONTRACT-DIFF`；A-ABI/C-FMS evidence 增补该复核，A-ABI seq8、C-FMS seq6 |
| ABI 锁定值 | 不变：145类型、Schema 6.0、Port/Product `0x00010904`、ARM `588707eb…`、Host `2e8c855d…`、lock `4781f28b…`；ABI 冻结仍待用户批准 |
| OBS-C-DIFF-01 | 已由 C 修正：`m4_mission.c` 头注释已改为 CR-V191-C01 epoch 校验说明 |
| OBS-C-DIFF-02 | 已由 C 修正：`status_request_T191-C-FMS.json` 基线改为 `0x00010904` 并注明 baseline_correction |
| OBS-C-DIFF-03 / C02 入口文字 | 02 CR-C02 段已补入 `quad_x4_fms_configure_product_geofence(const ProductGeofenceConfig*)`（Composition Root、非公共Port、禁运行时/Companion），与 06:433、08:201 一致；11 CR-C02 closure 注记同步 |

- 证据：round3 `b8759d7a…`；change_summary `9f9cf5e2…`；protected `b1ede039…`；post-k `4197c4fe…`（59项 0 FAIL）。

## 29. C-FMS 接受与 A-PLATFORM 阶段2 复核（00:7x）

| 任务 | 结果 |
|---|---|
| T191-C-FMS | **ACCEPTED/PASS/seq7**：CR-C01/C02 实现、Host 17/240、ARM PASS；目标构建/WCET/SIL/HIL 与 Gate D 保持 NOT_RUN |
| T191-A-PLATFORM | merged to **REVIEW/IN_PROGRESS/seq8**：Gate B static 16/16 PASS（TIM2/MPU/DShot/IRQ/DMA/位时序），但 **DRDY→预置SPI1 DMA NOT_RUN**（`drv_spi` 异步扩展缺失）、目标波形/map/WCET/HIL NOT_RUN，故不升 ACCEPTED |
| T191-B-ADAPTER | 继续 `blocked_by=[T191-A-PLATFORM]`：A 未提供 B 所需的原始快速通道 |
| 09 登记 | 新增 `TEST-A-PLATFORM-STAGE2`=IN_PROGRESS（16/16 static PASS + DRDY/Target NOT_RUN）；`TEST-C-FMS-EXEC`=PASS 保持 |

- 证据：round3 `87f4b0e8…`；change_summary `5b0384f3…`；protected `55134053…`；post-l `bd913f71…`（59项 0 FAIL）。

## 30. CR 定稿、MPC 决策与 C-M5/B-REPLAY 受理（00:8x）

| 事项 | 结果 |
|---|---|
| C-FMS | 按用户指示由 ACCEPTED 撤回为 **REVIEW/IN_PROGRESS/seq8**；CR 批准前不得升级，C-M5 不开工 |
| CR-C01/C02 | 独立复核=**PASS**（D：`TEST-A-ABI-HOST`、`TEST-CONTRACT-DIFF`；A：`C_CONTRACT_DIFF_A_REVIEW.md`）；用户批准=**PENDING**（未预填）；CR-C01/C02 均仍 SPECIFIED，未 CLOSED |
| CR-V191-C03 | 新增（11 §4.2）：**OSQP 1.0.0**（mode2/15变量/190行/静态workspace/无heap，包归 `packages/osqp/`）+ **ControlModelPort ABI v1 冻结**（02 §7 原样，不增删字段/原型）；SPECIFIED，用户批准 PENDING |
| B-REPLAY | 受理为 **REVIEW/IN_PROGRESS/seq4**：公共样本journal桥/M3 bridge Host+ARM 增量证据已登记（09 `TEST-B-REPLAY-INCR`）；真实burst/WCET NOT_RUN |
| C-M5 | 维持 READY/seq5（NO_CHANGE 准备）；待 C-FMS 与 B-REPLAY 接受后由 D 路由 |
| C-M6 | READY/seq4，阻塞缩减为 C-M5/B-REPLAY；MPC/ABI 决策已记录，待两前置接受后启动 |

- 证据：round3 `7d88f1b3…`；change_summary `52ce9c29…`；protected `823bbb57…`；post-m `accd117d…`（59项 0 FAIL）。

## 31. C-FMS Gate D 受理与 CR-C02 关闭（00:9x）

| 事项 | 结果 |
|---|---|
| T191-C-FMS | Gate D 复核完成 → **ACCEPTED/PASS/seq9**；依据 Host 17/240、ARM 编译、CR-C01/C02 实现与契约复核；目标构建/WCET/SIL/HIL 保持 **NOT_RUN** |
| CR-V191-C01 | **CLOSED**（用户 2026-11-05 指示；独立复核 PASS、ABI 0x00010904 锁） |
| CR-V191-C02 | **CLOSED**（Gate D 复核通过；编译期 `ProductGeofenceConfig` + `quad_x4_fms_configure_product_geofence` 入口；目标/SIL/HIL 仍 NOT_RUN） |
| T191-C-M5 | **ASSIGNED/seq6**：C-FMS 已接受、B-REPLAY 桥已 REVIEW；按 NONE-source 范围启动 |
| T191-C-M6 | 仍 READY，阻塞仅 C-M5/B-REPLAY；CR-C03 决策已记录 |

- 证据：round3 `8c63b412…`；change_summary `addb5fca…`；protected `823bbb57…`；post-n `accd117d…`（59项 0 FAIL）。

## 32. 三份 B 增量复核受理（01:1x）

| 任务 | 结果 |
|---|---|
| T191-B-ADAPTER rev2 | **REVIEW/IN_PROGRESS/seq5**：M2↔PLATFORM 桥42检查、test_m2 75、imu_sim 217、ARM 无heap符号 PASS；真实器件burst待A-PLATFORM；`blocked_by=[T191-A-PLATFORM]` |
| T191-B-REPLAY rev2 | **REVIEW/IN_PROGRESS/seq5**：公共样本journal桥/M3 bridge（Host 64检查）与 test_replay 531 检查 PASS；真实录制burst/WCET NOT_RUN；`blocked_by=[B-ADAPTER]` |
| T191-B-FDIR rev2 | **REVIEW/IN_PROGRESS/seq4**：Sensor Integrity Host 127检查、ARM PASS；无estimator句柄、不能改活动P；`blocked_by=[B-REPLAY]` |
| 09 登记 | 新增 `TEST-B-ADAPTER-INCR`、`TEST-B-FDIR-INCR`；`TEST-B-REPLAY-INCR` 保持 IN_PROGRESS |

- 证据：round3 `b7781d68…`；change_summary `0fb2b7de…`；protected `823bbb57…`；post-o `fecaeee3…`（59项 0 FAIL）。

## 33. T191-A-PLATFORM 验收（01:2x）

| 事项 | 结果 |
|---|---|
| 证据包 | `evidence/gate_b/acceptance_matrix.json`（12项：GB-01..08 PASS_STATIC、GB-09..12 NOT_RUN）+ `GATE_B_EVIDENCE_PACKAGE.md` + `package_sha256.json`（9文件Hash实算一致）+ symbols/sizes；ARM 编译 PASS |
| 任务状态 | **ACCEPTED/PASS/seq9**（静态范围）；DRDY→预置SPI DMA、目标波形/map/WCET/RTA/HIL 保持 NOT_RUN，归 Gate B/E 后续 |
| 后续链 | B-ADAPTER 解除 A-PLATFORM 阻塞（仍 REVIEW）；B-REPLAY←B-ADAPTER；B-FDIR←B-REPLAY |

- 证据：round3 `59c0af45…`；change_summary `bb178ee6…`；protected `823bbb57…`；post-p `1868a938…`（59项 0 FAIL）。

## 34. B-ADAPTER 接受与 B-REPLAY 解阻（01:3x）

| 事项 | 结果 |
|---|---|
| T191-B-ADAPTER | **ACCEPTED/PASS/seq6**：M2↔PLATFORM桥42、test_m2 75、imu_sim 217、ARM 无heap符号；真实器件burst/sample-time目标与HIL仍NOT_RUN（Gate C/E） |
| T191-B-REPLAY | `blocked_by=[]`（seq6）：开始真实录制burst回放、replay预算与M3 bridge集成证据；目标/HIL NOT_RUN |
| T191-B-FDIR | 仍被 B-REPLAY 阻塞（REVIEW）；待 REPLAY 接受后放开 |

- 证据：round3 `81db8b3a…`；change_summary `12e4572a…`；protected `823bbb57…`；post-q `dc294bc0…`（59项 0 FAIL）。

