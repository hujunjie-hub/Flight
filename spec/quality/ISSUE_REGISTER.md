# Flight 问题登记清单入口

本文件是当前本地正式问题登记入口。`GITHUB_PUBLICATION = PARTIAL`：GitHub Issues #8–#19 已用于当前部分问题在线跟踪；历史 FLIGHT-LOCAL 编号不保证与线上 Issue 一一映射，只有明确登记的映射有效。不得虚构 GitHub Issue 编号，也不得出现本地 CLOSED、GitHub OPEN 的状态冲突。

- [全部开放问题](https://github.com/hujunjie-hub/Flight/issues?q=is%3Aissue+is%3Aopen)
- [开放的安全问题](https://github.com/hujunjie-hub/Flight/issues?q=is%3Aissue+is%3Aopen+label%3Akind%3Asafety)
- [开放的 S0 问题](https://github.com/hujunjie-hub/Flight/issues?q=is%3Aissue+is%3Aopen+label%3Aseverity%3AS0)
- [开放的 M5 问题](https://github.com/hujunjie-hub/Flight/issues?q=is%3Aissue+is%3Aopen+label%3Aarea%3AM5)

## 新问题登记前检查

- [ ] 是否能用一个明确的根因描述问题？多个根因应拆成多个 Issue。
- [ ] 是否记录首次发现 commit、当前 baseline commit 和分支？
- [ ] 是否记录测试 ID、完整命令、环境、工具链和 RUN_ID？
- [ ] 是否保留完整日志、退出码、checks、failures 和 SHA256？
- [ ] 随机测试是否记录 seed、生成器版本、失败输入和最小反例？
- [ ] 是否判断安全影响、fail-closed 结果和阻断里程碑？
- [ ] 是否按 S0–S3 选择严重度并指定责任人？
- [ ] 是否使用 [Issue 模板](../../.github/ISSUE_TEMPLATE/flight-validation-issue.md)？

## 状态维护

状态必须按 [Issue 工作流](ISSUE_WORKFLOW.md) 更新；关闭前必须完成 [Issue 关闭清单](ISSUE_CLOSURE_CHECKLIST.md)。

GitHub Issues #8–#19 已在线跟踪；未映射的历史本地问题仍按本地状态管理。未完成适用的复验/审查不得 VERIFIED/CLOSED。MR-009/#8 的独立审查要求已由项目方于 2026-11-14 明确豁免，仅限该项，不得扩展到其他安全/数学问题。BLOCKER_BOARD.md 只是本文件的派生视图，不得维护独立状态。

## 历史本地正式登记（部分映射 GitHub Issues）

以下条目保留 R1–R16 draft 的
原编号与证据；线上 Issue 映射见本文件末尾。未完成适用的复验/审查前不得进入
VERIFIED/CLOSED；FLIGHT-LOCAL-04/MR-009 当前为 OWNER_WAIVED_INDEPENDENT_REVIEW / TECHNICAL_REVIEW_COMPLETED_WITH_CORRECTION。B01–B07 的详细阻塞视图见 [BLOCKER_BOARD.md](BLOCKER_BOARD.md)。

| ID | 严重度 | 根因 | 修复/基线 | 验证 | 状态 |
|---|---|---|---|---|---|
| FLIGHT-LOCAL-01 | S2 | Windows 检出把 *.sh 转 CRLF | R2 修复；`tools/check_line_endings.py` | line_endings PASS | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-02 | S2 | Cortex-M7 只编译 8/14 M5 源 | R2 补全 | M5 ARM 14 对象 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-03 | S2 | 历史 packages/SConscript 缺失导致链接阻塞 | R3 安装；2026-11-12/13 三包 commit 锁定并核对安装 HEAD | M5-06 接管复验：M4/M5 测试配置 Flight.elf 链接 PASS；完整 Release 产品链接/目标运行仍需复核 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-04 | S2 | MR-009 binary32 consumer/validator error envelope；同轴误差合成缺口已修正 | R31 rounded vf/af derivative、S06-107 oracle、S06-099 Fraction offsets、18 mutation FP audit；同轴误差先按轴合成再做 3-D outward norm，修复 commit 4f1b76d | 项目方 2026-11-14 明确豁免独立审查；S06-116 old-expression fail / corrected-expression pass；Host/ASan 复测通过 | OWNER_WAIVED_INDEPENDENT_REVIEW / TECHNICAL_REVIEW_COMPLETED_WITH_CORRECTION |
| FLIGHT-LOCAL-05 | S2 | spec/ABI 证据曾钉住 CRLF 哈希 | R3 efc28d7 LF 重基线 | 当前 CI：Spec Audit 145 types / 8 checks PASS；生产 ABI 144 types / 0 mismatch / 0 unexpected Host/ARM | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-06 | S2 | RT-Thread 三个包曾使用 latest，依赖不可复现 | OWNER 批准后固定三个 40-hex commit；跟踪源与锁文件一致 | `check_rt_packages_lock.py` PASS；14 个负向用例；当前 Host CI 未单独重跑此 job | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-07 | S1 | 验收脚本 assert 可被 -O 剥离 | R2 修复 | test_audit_optimized PASS | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-08 | S1 | 围栏净空未减消费曲线误差 | R5 修复 | S06-097 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-09 | S2 | FP 圆整/环境假设缺运行时证据 | R5 修复 | S06-098 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-10 | S1 | 时间量化不等式错误 | R6 修复 | S06-099 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-11 | S2 | FP 报告描述旧模型 | R7 修复 | consumer_fp MATCH | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-12 | S1 | R7 差分碰撞超包线、仅中点 | R8 撤回 | S06-100..102 NOT_RUN | REOPENED / NOT_RUN |
| FLIGHT-LOCAL-13 | S2 | S06-103 名义时间、S06-104 采样代连续 | R9/R10 修复 | S06-103/104 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-14 | S2 | 时间参考用舍入 h、固定系数结论 | R10 撤回/修复 | S06-103/105 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-15 | S2 | 时间/系数/包线方法缺口 | R11/R12 修复 | S06-105/107 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-16 | S2 | 测试 API 曾复制静态 SafetyEnvelope | R14b 595b1b7 | S06-111 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-17 | S0 | R14 初版 jerk 6/d³ 欠界 | R14b 595b1b7 | S06-110b/112 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-18 | S2 | S06-108 仅下一段；S06-109 float+epsilon | R15 a2fd81a | S06-108/109 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-19 | S0 | R14b 尾段 Bernstein 当幂基 | R16 fa1de8f | S06-112 1080 checks | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-20 | S1 | S06-108 漏 active 不变；C 注入不真实 | R16 fa1de8f | S06-108/113 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-21 | S2 | 非零错误 profile_hash 未拒绝 | R16 fa1de8f | S06-111 negative | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-22 | S1 | 非优化 `-O0` 专用镜像的 Validator 最坏链 4,248 B > 4,096 B | 按 Release `-O2` 复测专用镜像，3,864 B / 4,096 B；保留 `-O0` 失败 | B06，台账 2026-11-13 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-23 | S2 | R15 工具链 SHA/archive 字段不一致 | R16/R17 归档流程 | R16 manifest/bundle | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-24 | S1 | R16 选了 4,128 的短路径；R17 真实路径 4,232；R18 SP 修正 4,248 | R17 分析修正 | 本文件/analysis_p2 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-25 | S0 | H723 栈水位/WCET/RTA/HIL/实机避障 NOT_RUN | 无硬件；B07 | NOT_RUN | BLOCKED |
| FLIGHT-LOCAL-26 | S1 | `roll_map_to_position` 旧栈拷贝 32,840 B，Planner 产品化阻断 | R18/R19 逐轴原地滚动；R22 完整 40+64+24=128 B | 1000-case random + deterministic；legacy 32,840 B Host-only | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-27 | S2 | ICD+M4 静态余量在加入 GroundStopCertificate 双快照后估算仅 8 B | 不扩大 48 KiB 上限；当前表已计入 2×112 B，必须重跑预算与最终 map | 旧基线 232 B；扣除新增 224 B 后估算 8 B；本轮待重新执行 `check_memory_budget.py` | OPEN / RISK |
| FLIGHT-LOCAL-28 | S1 | S06-112 把所有 C API 拒绝都计为预期 fail-closed，可能掩盖合法样本意外拒绝 | R19 8b112f0279735f4f9977f4314b81dccf203b14c3 状态码区分；次正规仅接受 NUMERIC | S06-112 140 legal/20 subnormal status=2 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-29 | S2 | R18 地图原地滚动报告 104 B，漏算 map_cell_at 24 B；真实 128 B | R19 8b112f0279735f4f9977f4314b81dccf203b14c3 反汇编/.su 修正 | roll 40 + axis 64 + map_cell_at 24 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-30 | S2 | R18 Manifest 归档字段/外部 sidecar 未闭环 | R19 8b112f0279735f4f9977f4314b81dccf203b14c3 archive script + external report | r19 archive report/sidecar | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-31 | S2 | S06-112 历史结果文件未明确标记过期，存在证据混用风险 | R20 173360d49882dd98cb2a11ac26d189fc0e2f57a4 证据索引/状态 | S06_112_EVIDENCE_INDEX.md；r20 唯一 current | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-32 | S2 | R19 阻塞视图、Manifest、资源摘要历史基线描述滞后 | R20 173360d49882dd98cb2a11ac26d189fc0e2f57a4 | 文档/Manifest 基线统一 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-33 | S1 | S06-112 次正规拒绝及旧公式负向断言不足，单个拒绝即可 PASS | R21 0642bdaf9ba370c079b3b447a3d4acc4cd0bc514 严格矩阵/R14b 细节 | 160/140/20 status2/420；R14b 6/60 underbound | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-34 | S2 | S06-112 证据索引绑定错误代码 SHA | R21 0642bdaf9ba370c079b3b447a3d4acc4cd0bc514 索引 | 索引/library/Manifest 一致 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-35 | S2 | analysis_p2 资源摘要与当前基线不一致 | R29 fd8198d 文首 R29 摘要 + HISTORICAL 标记 | R29 blocker/analysis 检查 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-36 | S2 | R28 归档后元数据提交导致 Archive HEAD 漂移 | R29 两阶段 prepare/archive 协议 | R29 manifest/archive 检查 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-37 | S2 | Host/ARM 测试环境记录过期或缺少精确工具链绑定 | R29 r29_environment.json | environment 检查 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-38 | S2 | 回归证据缺少逐项 RUN_ID 绑定，历史 Manifest 未标记 | R29 r29_test_results.json + source manifest | test-results 检查 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-39 | S1 | S06-112 JSON 需按 R29 RUN_ID/code 重新绑定 | R29 real seed + JSON/log 分离 | S06-112 420/0 JSON_OK | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-40 | S1 | 测试结果收集器/证据检查器需 R29 严格复验 | R29 collector 28 + checker 33 负向 | collector/checker selftest normal/-O | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-41 | S2 | R22 原始工作区字节不可完整恢复 | R29 有范围搜索；保留 R27/R28 原始副本 | R27 worktree manifest SHA | BLOCKED |
| FLIGHT-LOCAL-51 | S1 | 回归入口接受空选择、错误 SHA 或以 0 退出掩盖门禁失败 | `36d7b7c` 校验选择和 HEAD，传播结果，按 RUN_ID 准备清单 | T24、T30，本地 8 项通过 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-52 | S1 | 静态栈分析会放过可达帧冲突或无界动态帧，递归会崩溃 | `36d7b7c` 对不完整帧证据返回 UNKNOWN | T26、T27、T29，8 项自测 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-53 | S1 | 固件验证链接前只核对包锁声明，安装源码可能偏离锁定 commit | `36d7b7c` 在链接前核对实际 HEAD 和跟踪源文件 | T14、T18、T22，HAL 头文件负向对照 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-54 | S2 | GNU ld 报告 32 位枚举对象与输出 enum-size 属性不一致 | 链接日志已保留，待 ABI 负责人核对边界和工具链库属性 | T18 链接 exit 0；当前生产 ABI CI 144 types / 0 spec mismatch，但 Tag_ABI_enum_size 警告仍待 ABI 负责人确认 | OPEN |
| FLIGHT-LOCAL-55 | S1 | Ubuntu CI 的 spec/ABI 检查硬编码 Windows 编译器路径 | 使用平台工具链查找，CI 安装 ARM/Newlib 和 MinGW | WSL、Windows 和 Host validation 复验 | FIXED_WAITING_RETEST |

历史 draft 01–20 保留在 `build/validation/issue_drafts/`；本表按根因去重，不删除
任何失败记录。


详细记录：`spec/quality/R29_ISSUE_DETAILS.md`；R28/R27/R25/R24 记录保留为历史。

FLIGHT-LOCAL-51..54 的复现与复测见 `M5-06-TAKEOVER-20261113` 归档内 `build/validation/regenerated/baseline-reproduction`、`import-reproduction` 和逐项台账。

FLIGHT-LOCAL-54 的 `readelf -A` 复查显示，CubeCLT `crtbegin.o`、`crtend.o` 和 `Flight.elf` 的 `Tag_ABI_enum_size` 为 `small`，项目 `board.o` 为 `int`。ABI 负责人待确认跨对象接口影响，现有 `-fno-short-enums` 和 ABI 锁继续生效。

## 2026-11-13 规范同步前 CI 复核（2a0237）

当前实现基线 `2a0237bf42963f5a4457af24da1b713cd6f957d1`（`opt/stage-02-m5-06-safety-validator`）的 Host validation、M6/M7 host validation、M7 host validation 均 SUCCESS，详见 `spec/09_需求验证实施与发布管理.md`。CI 对 Spec Audit、生产 ABI、M4/M5/M6/M7 Host、Sanitizer、runtime SIL 和 Cortex-M7 模块编译提供当前基线证据。

本轮 CI 不会自动关闭本地问题；`FIXED_WAITING_RETEST` 仍须独立复验。FLIGHT-LOCAL-27 因 GroundStopCertificate 双快照重新计入后只估算剩余 8 B，继续 OPEN / RISK。FLIGHT-LOCAL-54 的 enum-size 属性差异仍 OPEN。GitHub Issue 发布状态仍为 `GITHUB_PUBLICATION = PENDING`。本次 SPEC 同步 commit 的验证必须重新运行，不继承上述旧 SHA 的 PASS。

## 2026-11-13 规范同步后 CI 复核（b6b2c8）

验证基线 `b6b2c8d1fbe2483ac76164993e7798a18af1e697` 的三组工作流全部通过：[Host validation 6/6](https://github.com/hujunjie-hub/Flight/actions/runs/43410120128)、[M6/M7 host validation 5/5](https://github.com/hujunjie-hub/Flight/actions/runs/43410120115)、[M7 host validation 2/2](https://github.com/hujunjie-hub/Flight/actions/runs/43410120239)。生产 ABI 144 types / 0 mismatch / 0 unexpected Host/ARM；Spec Audit 145 types / 8 checks PASS；M4/M5/M6/M7 Cortex-M7 模块编译、M6/M7 Host/Sanitizer 和 M4→M7 runtime SIL 均 PASS。

本轮通过不关闭尚需独立复验的问题。FLIGHT-LOCAL-27 因 GroundStopCertificate 双快照重新计入后只估算剩余 8 B，继续 OPEN / RISK；`check_memory_budget.py` 和最终 map 仍待复验。FLIGHT-LOCAL-54 的 enum-size 属性差异仍 OPEN。GitHub Issue 发布状态仍为 `GITHUB_PUBLICATION = PENDING`。MR-009 R31 内部预审已完成但仍 WAITING_INDEPENDENT_REVIEW；M5-06 仍受 S06-100..102 与 B07 等独立发布门禁阻塞，M5-07 不得启动。


## 2026-11-13 MR-009 R31 三轮内部预审记录（独立审查仍待完成）

以下三轮是作者侧内部预审、代码修正与测试记录。它们不替代项目要求的独立数学审查，也不构成 APPROVED 或 CLOSED。

- 数学轮：修复 velocity/acceleration time-shift derivative bound，使用实际 binary32 rounded vf/af 系数导数；确认 position cf'、Horner 10/12/10、gamma 定向舍入、RN overflow midpoint 与次正规传播。
- 实现轮：对照 M6 consumer 与 M5 validator 的系数、局部时间和 Horner 运算顺序；双源 FP source audit 覆盖 consumer/validator time-domain、expanded s_eval_max、三组 Horner 与 cf'/vf'/af' 时间导数，18 个 mutation cases。
- 测试轮：S06-099 60,002 offsets 双时间模型 Fraction 检查；S06-107 oracle 覆盖 rounded derivative coefficients；Host #43446380866 6/6、M6/M7 #43446380836 5/5、M7 #43446380724 2/2 全部 SUCCESS。
- 代码修正 79f23d0200c31ebe934bfc8793e65d7af1e6bdd4；oracle a55071966209a1af615ff6137963eab4631e8bae；双源/expanded-time audit ad9df36687b68b43f9118ae6e26be8716d900c90；最终审计基线 8dcb60b3ac18e6a763c939d3e810984092cdd321。
- disposition：**INTERNAL_PRE_REVIEW_COMPLETE / WAITING_INDEPENDENT_REVIEW**。完整 Validator Horner 拒绝分支可达性不作声称；H723 实机、40 ms route preview、S06-100..102、WCET/RTA、HIL、飞行均按各自门禁处理。M5-06 整体仍 BLOCKED，M5-07 不启动。独立审查者尚未指定，必须逐项完成 R30 matrix 并签署最终结论。


## ENG-07 原生 FMT-SIM 本机复测登记

- 在线跟踪：GitHub Issues [#17 Land](https://github.com/hujunjie-hub/Flight/issues/17)、[#18 Return](https://github.com/hujunjie-hub/Flight/issues/18)、[#19 Mission](https://github.com/hujunjie-hub/Flight/issues/19)。
- 当前修复代码基线：ce00ff9f0a8eb4989d1a171c3322f3640de2965c；文档/CI 基线：dffba6fd9e2f25e03083439b37a4d557fe11e24d。
- 当前证据：Host #642 6/6、M6/M7 #130 5/5、M7 #91 2/2 SUCCESS；修复后的原生 FMT-SIM Land/Return/Mission 均 NOT_RUN。
- 执行责任：项目 Owner 在 Windows 本机执行；远程 Agent 不代跑原生 FMT-SIM。Agent 负责代码、Host/SIL/CI、分析和下一轮修复。
- 统一方案：spec/validation/FMT_SIM_LOCAL_HANDOFF.md；批量入口 `tests/simulation/run_fmt_sim_regression.ps1` 一次构建并启动三个独立引擎进程。验收固定为隔离 FMT-SIM、UDP 14653、350 ms、干净工作区且 HEAD 与 upstream tracking HEAD 一致；需回传 manifest、summary、完整日志、逐场景退出码、SHA256SUMS 和 ZIP sidecar。
- 状态规则：#17/#18/#19 继续 OPEN；收到修复后本机证据并逐项复核前，不标 VERIFIED/CLOSED。Host/CI PASS 不替代原生 FMT-SIM；FMT-SIM PASS 不替代 B07 H723 实机/HIL/WCET/栈水位/飞行证据。
