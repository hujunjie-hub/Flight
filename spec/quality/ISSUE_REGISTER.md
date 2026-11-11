# Flight 问题登记清单入口

本文件是 GitHub Issues 的使用入口，不复制维护一份静态 Issue 清单。**实时问题列表以 GitHub Issues 为准**。

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

GitHub label 尚未创建时，先在 Issue 正文填写 Quality Status 和 Severity，并在后续补齐标签；不能因标签缺失而省略问题登记。不得在本文件手工抄录开放问题，以免与 GitHub Issue 状态分叉。

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
| FLIGHT-LOCAL-26 | S1 | `roll_map_to_position` 栈拷贝 32,840 B，Planner 产品化阻断 | R18 7554b41 逐轴原地滚动 + 差分测试 | m5_map_roll_inplace_differential；roll 40 B + axis 64 B | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-27 | S2 | ICD+M4 静态余量仅 232 B（48,920 / 49,152 B） | 未扩大预算；每次静态布局变更复算 | memory_budget PASS + WARN | OPEN / RISK |

历史 draft 01–20 保留在 `build/validation/issue_drafts/`；本表按根因去重，不删除
任何失败记录。

