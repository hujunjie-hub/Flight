# Flight Agent 会话交接规范

本文件是 Flight 项目所有本地编码 Agent 的长期强制规范，不依赖任何单轮 RUN_ID 或聊天记录。
每次长任务必须按本规范生成可独立恢复的交接包；新 Agent 只能依据仓库文件和 Git 状态接管。

## 1. 交接触发条件

满足任一条件即必须准备交接，不得等到上下文耗尽：

- A. Agent 上下文已使用约 70%。
- B. 剩余上下文不足 25%。
- C. Agent 收到上下文空间不足的提示。
- D. 当前阶段工作量大，预计无法在当前会话内安全完成。
- E. 需要更换模型、Agent 或人工接管。

## 2. 旧 Agent 交接前必须完成

1. 停止开始新的大规模修改。
2. 等待正在运行的测试结束；无法结束时记录 PID、命令和中断原因。
3. 保存全部命令、开始/结束时间、退出码和完整 stdout/stderr。
4. 保存 Git HEAD、分支、remote HEAD、ahead 数量和 staged/unstaged 状态。
5. 保存 staged diff、worktree diff 和未跟踪文件清单。
6. 对每个相关文件计算原始字节 SHA256，并保存 tracked modified 与 untracked 的原始字节副本。
7. 保存问题登记、阻塞板、测试台账、未完成工作和当前阻塞。
8. 明确下一条可执行命令和下一检查点。
9. 生成固定目录下的完整交接包（见第 3 节）。
10. 在最终输出中报告 `HANDOVER_READY`，然后停止修改。

禁止：

- 为交接执行 `git reset`、`git clean`、`git checkout -- .`、`git stash` 或删除文件。
- 把未完成修改伪装为已完成。
- 把失败测试改成 PASS，或删除失败证据。
- 只写一段聊天摘要代替交接包。
- 使用“继续上面的任务”“参考上一会话”“按之前要求”等依赖上下文的表述。

## 3. 固定交接目录与文件

每轮创建：

```text
build/validation/handover/<RUN_ID>/
```

至少包含：

| 文件 | 内容 |
|---|---|
| `SESSION_HANDOVER.md` | 人类可读的完整交接说明，字段见第 4 节 |
| `handover_state.json` | 机器可读状态：Git、RUN_ID、commit、ahead、测试、阻塞 |
| `git_status.txt` | `git status --porcelain=v2 --untracked-files=all` 原始输出 |
| `git_head.txt` | `git rev-parse HEAD` |
| `git_branch.txt` | `git branch --show-current` |
| `git_diff_cached.patch` | staged diff，二进制安全 |
| `git_diff_worktree.patch` | unstaged diff，二进制安全 |
| `untracked_manifest.json` | 未跟踪文件 path/size/SHA256/原始副本路径 |
| `modified_files_manifest.json` | tracked modified 文件 path/size/SHA256/原始副本路径 |
| `blocker_snapshot.json` | 当前 B01–B07 与 FLIGHT-LOCAL 阻塞快照 |
| `test_progress.json` | 已运行/未运行测试、命令、退出码、checks、failures、seed、日志 SHA |
| `active_processes.json` | 仍在运行或中断的进程 |
| `next_actions.md` | 按顺序排列的下一步命令和预期结果 |
| `NEXT_SESSION_START_PROMPT.txt` | 可直接粘贴给新 Agent 的启动提示 |
| `HANDOVER_SHA256SUMS.txt` | 交接目录内每个文件的 SHA256 |

原始字节副本目录：

```text
working_tree_modified/<path>
working_tree_untracked/<path>
```

所有副本必须保持原始字节，不能只依赖 Git patch。所有交接文件必须写入
`HANDOVER_SHA256SUMS.txt`；该清单本身不在自身清单内。

## 4. SESSION_HANDOVER.md 必填字段

必须逐项写明：

- A. 项目目标。
- B. 当前阶段。
- C. 当前 RUN_ID。
- D. 当前 Git Branch。
- E. 当前 HEAD 完整 SHA。
- F. Validated Code Commit。
- G. Tooling Commit。
- H. Remote HEAD。
- I. Ahead 数量。
- J. 工作区状态（staged/unstaged/untracked 数量）。
- K. 已完成任务。
- L. 未完成任务。
- M. 当前失败。
- N. 当前阻塞。
- O. 已运行测试及完整命令。
- P. 测试结果和日志 SHA256。
- Q. 正在运行或中断的进程。
- R. 修改文件列表。
- S. 未跟踪文件列表。
- T. 不允许修改的基线。
- U. 下一步具体命令。
- V. 风险与注意事项。
- W. 新 Agent 必须首先执行的接管步骤。

## 5. 新 Agent 接管规则

新会话开始后按顺序执行：

1. 读取 `SESSION_HANDOVER.md`。
2. 读取 `handover_state.json`。
3. 执行只读 Git 检查：HEAD、branch、status、ahead、remote。
4. 对 `modified_files_manifest.json`、`untracked_manifest.json` 和
   `HANDOVER_SHA256SUMS.txt` 中的每个文件重新计算 SHA256。
5. 核对测试日志是否存在且 SHA256 一致，核对命令、退出码、checks、failures、seed。
6. 核对问题登记、阻塞板和未完成工作。
7. 生成 `TAKEOVER_ACK.md`，写明已读取文件、已核对文件、Git 状态、未完成工作、
   阻塞、差异和接管结论。

接管结论只能是：

- `TAKEOVER_CONFIRMED`：所有差异均可解释且有证据。
- `TAKEOVER_BLOCKED`：存在无法解释的差异；停止修改并登记。

接管失败时不得继续代码修改。接管成功后只执行 `next_actions.md`，不得凭聊天记忆扩展范围。

## 6. 交接完成条件

旧 Agent 必须：

1. 输出 `HANDOVER_READY`。
2. 提供 `SESSION_HANDOVER.md`、`NEXT_SESSION_START_PROMPT.txt`、
   `HANDOVER_SHA256SUMS.txt`。
3. 停止当前任务，不再修改代码。

只有新 Agent 完成独立核对并生成 `TAKEOVER_ACK.md` 后，交接流程才可标记
`PASS`。未发生真实会话切换时，只能标记 `NOT_RUN`，不得伪造端到端通过。

## 7. 交接与归档的关系

- 交接包是过程证据，不能替代 R28 正式归档 ZIP。
- 正式归档必须同时包含交接包和 `TAKEOVER_ACK.md`。
- 归档后新增的元数据提交必须单独记录，不能声称包含在旧归档中。
- 任何 Git Bundle 必须包含 `HEAD` 和当前命名分支 ref；如果归档 HEAD 与工作
  HEAD 不同，必须同时记录两者及原因。

## 8. 常见错误

- 把 `git status` 中的 R27 失败证据误当成本轮污染；必须先对照 worktree manifest。
- 把旧 R25/R26 文档当成本轮状态；必须核对文件头 RUN_ID。
- 用 `PASS` 文本、文件存在或旧日志推断通过；必须绑定真实退出码、计数、SHA 和
  source manifest。
- 用 Host/SIL 结果代替 H723 实机、HIL 或飞行许可；不得跨证据等级替代。
