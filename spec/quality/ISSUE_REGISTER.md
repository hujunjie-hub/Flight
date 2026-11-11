# Flight 问题登记清单入口

本文件是当前本地正式问题登记入口。`GITHUB_PUBLICATION = PENDING`：GitHub 发布未获授权前，以本文件为正式登记；获得明确授权后，GitHub Issues 才作为在线同步入口。不得虚构 GitHub Issue 编号，也不得出现本地 CLOSED、GitHub OPEN 的状态冲突。

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

GitHub 发布前，本地状态最多到 FIXED_WAITING_RETEST；未经独立复验不得标记 VERIFIED/CLOSED。BLOCKER_BOARD.md 只是本文件的派生视图，不得维护独立状态。

## 本地正式临时登记（GitHub 发布待授权）

Agent 无 GitHub 写权限；以下条目是本轮唯一本地登记入口，保留 R1–R16 draft 的
原编号与证据，`GITHUB_PUBLICATION = PENDING`。未完成独立复验前不得进入
VERIFIED/CLOSED。B01–B07 的详细阻塞视图见 [BLOCKER_BOARD.md](BLOCKER_BOARD.md)。

| ID | 严重度 | 根因 | 修复/基线 | 验证 | 状态 |
|---|---|---|---|---|---|
| FLIGHT-LOCAL-01 | S2 | Windows 检出把 *.sh 转 CRLF | R2 修复；`tools/check_line_endings.py` | line_endings PASS | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-02 | S2 | Cortex-M7 只编译 8/14 M5 源 | R2 补全 | M5 ARM 14 对象 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-03 | S2 | packages/SConscript 缺失，链接 BLOCKED | 依赖 B03/B04 | check_target_build BLOCKED | BLOCKED |
| FLIGHT-LOCAL-04 | S2 | MR-009 消费曲线/验证曲线数值路径不一致 | 多轮修复；R16 三类 jerk | S06-085..093、112 | OPEN / WAITING_INDEPENDENT_REVIEW |
| FLIGHT-LOCAL-05 | S2 | spec/ABI 证据钉住 CRLF 哈希 | 未修改；B01/B02 | audit/abi FAIL | BLOCKED |
| FLIGHT-LOCAL-06 | S2 | RT-Thread 三个包均为 latest | 未修改；B03 | packages_lock FAIL | BLOCKED |
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
| FLIGHT-LOCAL-22 | S1 | Validator 最坏栈 4,248 B > 4,096 B | 未修复；B06 | analysis_p2 R17 | OPEN / BLOCKED |
| FLIGHT-LOCAL-23 | S2 | R15 工具链 SHA/archive 字段不一致 | R16/R17 归档流程 | R16 manifest/bundle | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-24 | S1 | R16 选了 4,128 的短路径；R17 真实路径 4,232；R18 SP 修正 4,248 | R17 分析修正 | 本文件/analysis_p2 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-25 | S0 | H723 栈水位/WCET/RTA/HIL/实机避障 NOT_RUN | 无硬件；B07 | NOT_RUN | BLOCKED |
| FLIGHT-LOCAL-26 | S1 | `roll_map_to_position` 旧栈拷贝 32,840 B，Planner 产品化阻断 | R18/R19 逐轴原地滚动；R22 完整 40+64+24=128 B | 1000-case random + deterministic；legacy 32,840 B Host-only | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-27 | S2 | ICD+M4 静态余量仅 232 B（48,920 / 49,152 B） | 未扩大预算；每次静态布局变更复算 | memory_budget PASS + WARN | OPEN / RISK |
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

历史 draft 01–20 保留在 `build/validation/issue_drafts/`；本表按根因去重，不删除
任何失败记录。


详细记录：`spec/quality/R29_ISSUE_DETAILS.md`；R28/R27/R25/R24 记录保留为历史。

