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

## 本地临时登记（GitHub 发布待授权）

以下条目在本地质量台账中登记，用于 R15/R16 复验追溯。Agent 没有 GitHub 写权限，
不冒充已发布 GitHub Issue；`GITHUB_PUBLICATION = PENDING`。未完成独立复验前不得
标记 VERIFIED/CLOSED。

| 本地编号 | 严重度 | 根因 | 修复/基线 | 验证 | 状态 |
|---|---|---|---|---|---|
| FLIGHT-LOCAL-16 | S2 | 测试 API 曾复制静态 SafetyEnvelope | R14b 595b1b7 起接入产品 builder；R16 保持 | S06-111，M5 Host 319,438/0 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-17 | S0 | R14 初版 jerk 6/d³ 欠界 | R14b 595b1b7 修正为 60/d³；R16 保持 | S06-110b，S06-112 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-18 | S2 | S06-108 只测下一段、S06-109 float+epsilon | R14b/R15 a2fd81a | S06-108 96 例，S06-109 96 例 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-19 | S0 | R14b jerk 尾段把 Bernstein 控制点当幂基 | R16 fa1de8f | S06-112 1080 component checks，旧公式 248/300 欠界 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-20 | S1 | S06-108 遗漏 active 不变但 global/local 不等；C 注入不真实 | R16 fa1de8f | S06-108 wrong_local=72；S06-113 76 checks | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-21 | S2 | 产品 config profile_hash 非零错误值未拒绝 | R16 fa1de8f | S06-111 负向 config 测试 | FIXED_WAITING_RETEST |
| FLIGHT-LOCAL-22 | S1 | Validator 最坏栈 4,128 B > 4,096 B | 未修复；产品化阻断 | analysis_p2 R16 | OPEN / BLOCKED |
| FLIGHT-LOCAL-23 | S2 | R15 工具链记录与代码 SHA 不一致、archive 字段不完整 | R16 归档流程 | R16 归档 manifest / r15_test_results | FIXED_WAITING_RETEST |

历史 R1–R14 的 draft 仍保留在 `build/validation/issue_drafts/`，不得删除或改写失败记录。
