# Flight 问题登记与质量闭环

## 唯一记录位置

**GitHub Issues 是问题登记、状态、责任人和关闭结论的唯一权威来源。**

- 不允许仅把问题写在聊天记录、临时 TXT、Excel 或本地 Agent memory 中。
- `spec/09_需求验证实施与发布管理.md` 只维护需求/验收状态；问题的完整生命周期放在 GitHub Issue。
- `work/validation/archives/<RUN_ID>/` 保存本次复现和验证证据，不作为问题状态台账。
- Issue 中必须引用测试 ID、commit SHA、RUN_ID 和证据路径/链接。
- Agent 没有 GitHub 写权限时，生成符合 `.github/ISSUE_TEMPLATE/flight-validation-issue.md` 的 Markdown 草稿，并将状态标为 `ISSUE_DRAFT_NOT_PUBLISHED`，不得声称已经登记。

## 编号

GitHub 自动生成的 Issue number 是唯一编号。项目内引用统一写作 `FLIGHT-<number>`，例如 `FLIGHT-42`，并附 GitHub URL。不得自行分配可能重复的编号。

## 严重级别

| 级别 | 定义 | 处理要求 |
|---|---|---|
| S0 | 安全关键：可能导致错误 Permit、危险输出、碰撞漏检、失控或绕过安全门禁 | 立即阻断相关发布/阶段；优先复现；未经独立审查不得关闭 |
| S1 | 高：核心功能错误、持续回归、关键测试/目标构建失败或安全边界退化 | 阻断当前里程碑验收，优先修复 |
| S2 | 中：局部功能/接口/性能/测试覆盖问题，有明确保守降级 | 纳入当前或下一修复批次 |
| S3 | 低：文档、可维护性、非阻塞诊断和体验问题 | 排期处理，不得影响安全声明 |

无法确定严重度时先按较高等级登记，再由 TRIAGED 阶段复核。涉及安全证明缺口、未知输入被当作安全、预算耗尽仍放行，一律至少 S0。

## 状态机

唯一状态集合：

`NEW → TRIAGED → NEEDS_INFO → REPRODUCED → IN_PROGRESS → FIXED_WAITING_RETEST → RETESTING → VERIFIED → CLOSED`

旁路状态：`BLOCKED`、`REOPENED`。

含义、进入条件和允许的下一状态见 [ISSUE_WORKFLOW.md](ISSUE_WORKFLOW.md)。GitHub Issue 的 open/closed 与质量状态分开：
- NEW 到 VERIFIED 通常保持 GitHub Open。
- CLOSED 才关闭 GitHub Issue。
- BLOCKED、NEEDS_INFO、FIXED_WAITING_RETEST 不能关闭。
- 关闭后发现回归，重新打开并标记 REOPENED。

## 必须使用的标签

状态标签：`status:new`、`status:triaged`、`status:needs-info`、`status:reproduced`、`status:in-progress`、`status:fixed-waiting-retest`、`status:retesting`、`status:verified`、`status:blocked`、`status:reopened`。

严重度标签：`severity:S0`、`severity:S1`、`severity:S2`、`severity:S3`。

类型/范围标签：`kind:bug`、`kind:safety`、`kind:test-gap`、`kind:build`、`kind:docs`、`area:M4`、`area:M5`、`area:M6`、`area:M7`、`area:INS`、`area:platform`、`verification:host`、`verification:sanitizer`、`verification:arm`、`verification:hil`。

标签缺失时可先在 Issue 正文填状态和严重度，并登记补齐标签任务；不能因为标签未创建而不登记问题。

## 入口

1. 问题登记入口与实时开放列表：[ISSUE_REGISTER.md](ISSUE_REGISTER.md)。
2. 新问题：使用 [Flight Validation Issue 模板](../../.github/ISSUE_TEMPLATE/flight-validation-issue.md)。
2. 状态流转：按 [Issue 工作流](ISSUE_WORKFLOW.md)。
3. 关闭前：逐项完成 [关闭检查清单](ISSUE_CLOSURE_CHECKLIST.md)。
4. 本地 Agent 操作：见 [spec/AGENT-操作指南.md](../AGENT-操作指南.md)。
5. 验收和证据：见 [Flight 验收与结果提交](../Flight-验收与结果提交.md)。

## 当前约束

M5-06 当前仍是 Host/SIL 独立验证阶段。S06 测试 CI 通过不等于 MR-001..MR-008 全部数学审查通过；M5-06 独立审查未 APPROVED 前，不得开始 M5-07。H723/NONE 不具备在线避障放行。
