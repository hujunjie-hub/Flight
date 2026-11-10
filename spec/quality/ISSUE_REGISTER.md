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
