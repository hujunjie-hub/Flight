# Flight 问题登记清单入口

本文件是当前本地正式问题登记入口。`GITHUB_PUBLICATION = PARTIAL`：GitHub Issues #8–#19、#21–#22 已用于当前部分问题在线跟踪；历史 FLIGHT-LOCAL 编号不保证与线上 Issue 一一映射，只有明确登记的映射有效。不得虚构 GitHub Issue 编号，也不得出现本地 CLOSED、GitHub OPEN 的状态冲突。

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

GitHub Issues #8–#19、#21–#22 已在线跟踪；#8 已因项目方明确豁免独立审查且技术复核完成而关闭（不代表第三方独立 APPROVED）。未映射的历史 FLIGHT-LOCAL 编号仍按本地状态管理；未完成适用的复验/审查不得 VERIFIED/CLOSED。MR-009/#8 的豁免仅限该项，不得扩展到其他安全/数学问题。下方带日期的 R16–R31/CI 条目保留当时快照；其中 GITHUB_PUBLICATION=PENDING、MR-009 WAITING_INDEPENDENT_REVIEW 和旧资源数字均非当前状态，当前状态以本段、末尾 ENG-07 当前记录和 BLOCKER_BOARD 顶部摘要为准。BLOCKER_BOARD.md 只是本文件的派生视图，不得维护独立状态。

## 历史本地正式登记（部分映射 GitHub Issues）

以下条目保留 R1–R16 draft 的
原编号与证据；线上 Issue 映射见本文件末尾。未完成适用的复验/审查前不得进入
VERIFIED/CLOSED；FLIGHT-LOCAL-04/MR-009 当前为 OWNER_WAIVED_INDEPENDENT_REVIEW / TECHNICAL_REVIEW_COMPLETED_WITH_CORRECTION。B01–B07 的详细阻塞视图见 [BLOCKER_BOARD.md](BLOCKER_BOARD.md)。

| ID | 严重度 | 根因 | 修复/基线 | 验证 | 状态 |
|---|---|---|---|---|---|
| FLIGHT-LOCAL-01 | S2 | Windows 检出把 *.sh 转 CRLF | R2 修复；`tools/check_line_endings.py` | line_endings PASS | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-02 | S2 | Cortex-M7 只编译 8/14 M5 源 | R2 补全 | M5 ARM 14 对象 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-03 | S2 | 历史 packages/SConscript 缺失导致链接阻塞 | R3 安装；2026-11-12/13 三包 commit 锁定并核对安装 HEAD | M5-06 接管复验：M4/M5 测试配置 Flight.elf 链接 PASS；完整 Release 产品链接/目标运行仍需复核 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-04 | S2 | MR-009 binary32 consumer/validator error envelope；同轴误差合成缺口已修正 | R31 rounded vf/af derivative、S06-107 oracle、S06-099 Fraction offsets、18 mutation FP audit；同轴误差先按轴合成再做 3-D outward norm，修复 commit 4f1b76d | 项目方 2026-11-14 明确豁免独立审查；S06-116 old-expression fail / corrected-expression pass；Host/ASan 复测通过；GitHub #8 已关闭 | OWNER_WAIVED_INDEPENDENT_REVIEW / TECHNICAL_REVIEW_COMPLETED_WITH_CORRECTION |
| FLIGHT-LOCAL-05 | S2 | spec/ABI 证据曾钉住 CRLF 哈希 | R3 efc28d7 LF 重基线 | 当前 CI：Spec Audit 145 types / 8 checks PASS；生产 ABI 144 types / 0 mismatch / 0 unexpected Host/ARM | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-06 | S2 | RT-Thread 三个包曾使用 latest，依赖不可复现 | OWNER 批准后固定三个 40-hex commit；跟踪源与锁文件一致 | `check_rt_packages_lock.py` PASS；14 个负向用例；当前 Host CI 未单独重跑此 job | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-07 | S1 | 验收脚本 assert 可被 -O 剥离 | R2 修复 | test_audit_optimized PASS | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-08 | S1 | 围栏净空未减消费曲线误差 | R5 修复 | S06-097 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-09 | S2 | FP 圆整/环境假设缺运行时证据 | R5 修复 | S06-098 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-10 | S1 | 时间量化不等式错误 | R6 修复 | S06-099 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-11 | S2 | FP 报告描述旧模型 | R7 修复 | consumer_fp MATCH | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-12 | S1 | R7 旧差分碰撞夹具超出产品包线、仅中点采样不足 | R8 撤回旧夹具；改用产品包线内 S06-100..102 | S06-100..102 Host/ASan 各 322,337 checks / 0 failures；GitHub #15 仍 OPEN / RETESTING | RETESTING / OPEN |
| FLIGHT-LOCAL-13 | S2 | S06-103 名义时间、S06-104 采样代连续 | R9/R10 修复 | S06-103/104 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-14 | S2 | 时间参考用舍入 h、固定系数结论 | R10 撤回/修复 | S06-103/105 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-15 | S2 | 时间/系数/包线方法缺口 | R11/R12 修复 | S06-105/107 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-16 | S2 | 测试 API 曾复制静态 SafetyEnvelope | R14b 595b1b7 | S06-111 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-17 | S0 | R14 初版 jerk 6/d³ 欠界 | R14b 595b1b7 | S06-110b/112 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-18 | S2 | S06-108 仅下一段；S06-109 float+epsilon | R15 a2fd81a | S06-108/109 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-19 | S0 | R14b 尾段 Bernstein 当幂基 | R16 fa1de8f | S06-112 1080 checks | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-20 | S1 | S06-108 漏 active 不变；C 注入不真实 | R16 fa1de8f | S06-108/113 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-21 | S2 | 非零错误 profile_hash 未拒绝 | R16 fa1de8f | S06-111 negative | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-22 | S1 | 历史 `-O0` Validator 最坏链 4,248 B 超过 4,096 B | 2026-11-14 重建 `-O2` / `-O0` 专用镜像并完成调用图复核 | `-O2` 3,576/4,096 B；`-O0` 3,968/4,096 B；调用图 COMPLETE，`-O0` 余量 128 B | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-23 | S2 | R15 工具链 SHA/archive 字段不一致 | R16/R17 归档流程 | R16 manifest/bundle | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-24 | S1 | 历史 R16/R17/R18 栈路径漏计，曾报告 4,128/4,232/4,248 B | 2026-11-14 专用镜像重建并完成调用图分析 | `-O2` 3,576/4,096 B；`-O0` 3,968/4,096 B；调用图 COMPLETE | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-25 | S0 | H723 栈水位/WCET/RTA/HIL/实机避障 NOT_RUN | 无硬件；B07 | NOT_RUN | BLOCKED |
| FLIGHT-LOCAL-26 | S1 | `roll_map_to_position` 旧栈拷贝 32,840 B，Planner 产品化阻断 | R18/R19 逐轴原地滚动；R22 完整 40+64+24=128 B | 1000-case random + deterministic；legacy 32,840 B Host-only | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-27 | S2 | ICD+M4 静态 RAM 余量低于 1 KiB 警戒线 | 删除重复 mission staging 后重新核对 Release map 和预算 | 48,888/49,152 B，余量 264 B；仍须监控后续增长 | OPEN / RISK |
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
- disposition：**OWNER_WAIVED_INDEPENDENT_REVIEW / TECHNICAL_REVIEW_COMPLETED_WITH_CORRECTION**；项目方已明确豁免该项独立审查，技术复核发现已修正，GitHub Issue #8 已关闭；不声称第三方独立 APPROVED。


## ENG-07 原生 FMT-SIM 本机复测登记

- R33 RUN_ID：`FMT-SIM-LOCAL-20261114T084723472Z-5e3b74f4`；测试 HEAD：`5e3b74f40293a005de693eceb61af4c055f1efdb`。ZIP SHA256 `0d6b75c578e2ae71b501f9f26b827559fe8b0887598253af55d09d09f63cdcfb` 与 sidecar 匹配，manifest SHA256 `6604b49d3fe889c9c022b63dec313b762880d7aa8cc12e736975257e2490971d`，内部 102 项 SHA256 全部通过；branch/upstream 匹配、worktree clean/stable、loopback/UDP/hover/350 ms 门禁均有效。
- R33 结果：Land 39,388/0 PASS、Return 40,485/0 PASS、Mission 67,112/3 FAIL；总计 146,985/3，最长状态间隔 94.8 ms。Mission 末段最大下降速度 0.374 m/s，touchdown distance 0.171 m；失败仅为 terminal ACK、DISARM ACK、arm state 未 DISARMED。详细证据见 [R33 report](../validation/FMT_SIM_LOCAL_20261114_5E3B74F4_REVIEW.md)。
- R33 复现接地交接缺陷：t=34.280 s contact=1、landed=UNKNOWN、evidence=0x0；M4 严格稳定门槛未满足，旧代码没有给 M5 保留 advisory contact evidence。新生产修复 `419f56962f74fd9d89bc62cfc11ee0923910177c` / `15a33756d4e269800e6778c6180f7086179c9429` 已提交，M4 安全阈值和 2 s dwell 未放宽。
- 测试补强 `3270cff5aec26e3ae0717c62d2d78e9f30752020`：缺失导航时不发布接地候选证据；`contact_candidate_not_disarm_authority` 与 `contact_candidate_requires_valid_navigation` 均 PASS。
- Current-main CI on `3270cff5aec26e3ae0717c62d2d78e9f30752020`: Host [#43593701736](https://github.com/hujunjie-hub/Flight/actions/runs/43593701736) 6/6, M6/M7 [#43593701892](https://github.com/hujunjie-hub/Flight/actions/runs/43593701892) 5/5, M7 [#43593701803](https://github.com/hujunjie-hub/Flight/actions/runs/43593701803) 2/2 SUCCESS. M4 969/0; M5 327,030/0; M4/M5 integration 1,042/0; MR-009 focused 430/0; M7 274/0.
- R33 测试 HEAD 早于上述生产修复；修复后的原生 FMT-SIM 仍 NOT_RUN。#17/#18/#19/#21/#22 全部继续 OPEN；#17/#18 单场景 PASS 不能替代完整 manifest closure eligibility。
- Owner 在 Windows 本机运行；Agent 负责代码、Host/SIL/CI、归档分析和文档。统一方案：`spec/validation/FMT_SIM_LOCAL_HANDOFF.md`；批量入口 `tests/simulation/run_fmt_sim_regression.ps1`。严格门槛为 main、UDP 14653、350 ms、hover 0.62、显式 loopback 127.0.0.1、clean worktree 且 HEAD 与刚 fetch 的 origin/main 一致。
- FMT-SIM PASS 不替代 B07 H723 实机/HIL/WCET/栈水位/飞行证据。

## 2026-11-14 R30 归档复核登记

- 归档：`FMT-SIM-R30-20261114.zip`；SHA256 `eb52b0d6430662cc0a88869570dcf23c00c964add676d2ef31870c4ae9909063`；sidecar 匹配，内部 31 项哈希全部匹配。
- 测试 HEAD：`3299a4f0702083ca4374ccc978edccb5d0c900da`；工作区 DIRTY，归档 upstream `51e7e923...`。复核时 GitHub 当前目标分支 HEAD `9063f31053ac301b71c806858a67e84f69402414`，较测试 HEAD 前进 181 commits，包含后续 ENG-07 修复。
- 结果：Land 5、Return 6、Mission 9、Land single 5、Land Beach 4 failures；共 201,306 checks / 29 failures。最长状态间隔 67.1–208.4 ms，低于 350 ms。
- 分类：`DIAGNOSTIC_ONLY / STALE_HEAD / DIRTY_WORKTREE / FORMAL_MANIFEST_MISSING`。不构成修复后复测，不关闭 #17/#18/#19；不基于此旧 HEAD 重复修改生产代码。
- 下一步：新建干净克隆检出当前分支，使用 `tests/simulation/run_fmt_sim_regression.ps1`，显式 `-ConfirmLoopbackAddress`，并回传 manifest、summary、完整日志和 ZIP sidecar。详细记录见 [R30 归档复核报告](../validation/FMT_SIM_R30_20261114_ARCHIVE_REVIEW.md)。

## 2026-11-14 main 唯一分支与 ENG-07 R31 本机复测状态

- 当前唯一远程开发/测试/交付分支为 `main`。PR [#20](https://github.com/hujunjie-hub/Flight/pull/20) 已合并，合并 HEAD 为 `eacb966b502b7f9cb547617e866641504346a5fd`；此前 `opt/spec-sync-review` 的 83 个提交和 stage-02 后续 94 个提交均包含在完整 177 提交历史中。 两个旧 opt 分支已由清理 workflow run [#43567201449](https://github.com/hujunjie-hub/Flight/actions/runs/43567201449) 删除；远程分支列表仅剩 `main`。 Main HEAD `d7666c5aa093dbef36fef11b82909e91fdc61491` 的三组工作流全部 SUCCESS：Host [#43567633122](https://github.com/hujunjie-hub/Flight/actions/runs/43567633122) 6/6、M6/M7 [#43567633103](https://github.com/hujunjie-hub/Flight/actions/runs/43567633103) 5/5、M7 [#43567633072](https://github.com/hujunjie-hub/Flight/actions/runs/43567633072) 2/2；Host PowerShell runner parse step PASS。
- R31 本机 FMT-SIM RUN_ID `FMT-SIM-LOCAL-20261114T045144621Z-304fbd12` 测试的是旧 HEAD `304fbd12078679d46b80cb626aba9eb0727b6914`，比合并源 HEAD `bcbe4db5e665e5f678069afc1b206a00ae1cd36d` 落后 7 commits。Land 36,600/4、Return 36,601/3、Mission 43,692/6，合计 116,893 checks / 13 failures；构建 PASS，三场景 FAIL。
- ZIP SHA256 `af97c2ac451c2937fab186ccd5e1a9f553b4ee6d699470ab9963e0b105505d26` 与 sidecar 一致，内部 102 项哈希全部匹配。该结果有效绑定旧 HEAD，但不验证后续 7 个提交或当前 main。
- GitHub Issues #17 Land、#18 Return、#19 Mission 均保持 OPEN。等待 Owner 在干净的 `main` / `origin/main` 同步 checkout 上完成正式 350 ms 批量复测；不得用旧 HEAD 失败直接判定当前 main 失败，也不得用 Host/CI PASS 关闭原生 FMT-SIM Issue。
- 具体失败断言、工具链、FMT-SIM/settings 哈希及逐场景日志哈希见 [R31 归档复核报告](../validation/FMT_SIM_LOCAL_20261114_304FBD12_REVIEW.md)。


## 2026-11-14 ENG-07 R32 本机复测与修复状态

- 归档 RUN_ID `FMT-SIM-LOCAL-20261114T072713056Z-e470f00c`，测试 HEAD `e470f00cd539f438b1923b0faa844456ba2d3b59`；main/origin/main 精确匹配，worktree clean/stable，fetch 成功，正式 350 ms / UDP 14653 / hover 0.62 / loopback gate 全部满足。
- 归档完整性 PASS：ZIP SHA256 `93f9ad1289a0b30f0a92afa3dee039b37b1e8c03435db54354a2986cdf3eb76d` 与 sidecar 匹配；manifest SHA256 `50cd771b3b810bdf8d5dd08c761b201b4e5a19770ee4b9405d93799d0e039234` 匹配；内部 102 项哈希全部通过。
- R32：Land 48,043/0 PASS；Return 44,667/0 PASS；Mission 62,793/3 FAIL；合计 155,503 checks / 3 failures。最大状态间隔 145.3 ms。Mission 剩余：近地 0.50 m 最大下降速度 0.665 m/s、DISARMED ACK reason 11、arm state 未 DISARMED。
- 代码根因：M5 旧约束仅覆盖最后 0.20 m 的 0.12 m/s，缺少最后 0.50 m 的 0.40 m/s 包络；LANDED 后继续追 Home XY 导致 touchdown handoff 水平速度短暂越过 FMS 0.20 m/s 稳定门槛。FMS 安全门槛不应放宽。
- 修复 main commit `3d8d29b5dfda21237f8ed02db3c31223c656bfbe`：增加 0.50 m / 0.40 m/s 速度约束；LANDED 后锁存 touchdown XY 并在快照短暂丢失时保持；SIL 增加 landed stable/evidence/confidence 和 near-ground peak 日志。
- Fix commit `3d8d29b5dfda21237f8ed02db3c31223c656bfbe` passed Host validation [#43574875444](https://github.com/hujunjie-hub/Flight/actions/runs/43574875444) 6/6, M6/M7 [#43574875363](https://github.com/hujunjie-hub/Flight/actions/runs/43574875363) 5/5 and M7 [#43574875394](https://github.com/hujunjie-hub/Flight/actions/runs/43574875394) 2/2. M5 Host and ASan/UBSan each passed 326,004/0; M4 Host 945/0; M4/M5 integration and its sanitizer each 1,018/0; MR-009 focused 430/0. New tests `landing_alignment_and_speed` and `landing_contact_latches_xy` passed.
- #17 Land、#18 Return 在 R32 各自场景 PASS，但整份 manifest `issue_closure_eligible=false`，因为 #19 Mission 仍 FAIL；三个 Issue 继续 OPEN，等待修复后完整三场景复测。R32 完整证据见 [R32 report](../validation/FMT_SIM_LOCAL_20261114_E470F00C_REVIEW.md)。
