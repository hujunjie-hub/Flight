# Flight Issue 工作流

## 1. 新建问题（NEW）

以下任一情况必须登记：
- CI / 本地构建 / 单元 / 集成 / Sanitizer 失败；
- 关键测试缺失、测试数为零或日志/基线不可追溯；
- 接口、时间、坐标、epoch、revision、TTL、ABI 不一致；
- 数值边界、浮点舍入、碰撞检查或 fail-closed 行为存在疑问；
- 资源预算、固定内存、栈深、WCET/RTA 或目标编译不满足；
- 文档与代码/测试事实不一致；
- 需要用户决定的设计或安全假设。

每个 Issue 聚焦一个根因。一个 CI job 失败包含多个独立根因时，应拆分 Issue 并互相关联。

## 2. 必填字段

- 标题：`[Sx][Area] 简短可复现问题`。
- Severity：S0–S3。
- Type：bug / safety / test-gap / build / docs。
- Area：M4/M5/M6/M7/INS/platform。
- Quality Status：状态机中的一个值。
- First seen commit、baseline commit、branch、Profile/config。
- RUN_ID、测试 ID、完整命令、环境和工具链版本。
- 预期结果、实际结果、复现步骤、失败次数/总次数。
- 原始日志路径与 SHA256；GitHub Actions URL（如有）。
- 随机 seed、生成器版本、失败输入、最小反例（随机测试适用）。
- 安全影响、是否 fail-closed、被阻断的验收/里程碑。
- Owner、下一步动作、预计复测范围。

没有数据的字段明确写 `UNKNOWN` 或 `NOT_RUN`，不要猜测。

## 3. 状态定义与转移

| 状态 | 进入条件 | 下一状态 | GitHub Issue |
|---|---|---|---|
| NEW | Agent/CI/审查首次发现问题，Issue 已建立 | TRIAGED | Open |
| TRIAGED | 已确认类型、严重度、子系统、责任人和阻断范围 | NEEDS_INFO / REPRODUCED / IN_PROGRESS / BLOCKED | Open |
| NEEDS_INFO | 复现所需数据/硬件/参数/日志缺失 | TRIAGED / REPRODUCED / BLOCKED | Open |
| REPRODUCED | 按记录命令在指定基线稳定复现，或保留确定性反例 | IN_PROGRESS / BLOCKED | Open |
| IN_PROGRESS | 已明确根因假设并开始修改 | FIXED_WAITING_RETEST / BLOCKED | Open |
| FIXED_WAITING_RETEST | 修复 commit 已生成，回归测试已增加，但尚未复测 | RETESTING / BLOCKED | Open |
| RETESTING | 在修复 commit 上执行原失败测试和回归集 | VERIFIED / IN_PROGRESS / REOPENED / BLOCKED | Open |
| WAITING_INDEPENDENT_REVIEW | 修复与复测证据齐备，数学/安全证明已提交独立审查，但尚无独立审查签字 | VERIFIED（仅 APPROVED）/ IN_PROGRESS（需修正）/ NEEDS_INFO / BLOCKED | Open |
| VERIFIED | 独立复测证据满足关闭条件，审查人确认 | CLOSED | Open |
| CLOSED | 所有关闭条件满足，Issue 关闭 | REOPENED（如复发） | Closed |
| BLOCKED | 受外部条件/决策/硬件/工具链阻塞，原因和解除条件明确 | 原状态恢复或 TRIAGED | Open |
| REOPENED | 已关闭问题复发或发现原修复不完整 | TRIAGED / REPRODUCED | Reopened/Open |

注意：
- `FIXED_WAITING_RETEST` 只代表代码修改完成，不代表问题已解决。
- `VERIFIED` 必须有复测 commit 和实际测试证据，不能由修复者仅凭代码审查自评。
- `WAITING_INDEPENDENT_REVIEW` 表示作者侧实现/测试证据已准备好、独立审查仍未完成；它不是 APPROVED/VERIFIED，也不得据此关闭 Issue 或推进受阻里程碑。
- `BLOCKED` 必须写 blocker、责任方、解除条件和下一次检查点；不得作为长期隐藏失败的状态。
- `NEEDS_INFO` 不得被误写为 NOT_RUN 或 PASS。
- GitHub closed 只允许在 CLOSED 状态。

## 4. 严重度和阻断规则

- S0：立即阻断受影响功能/发布；任何可能绕过授权、把未知/过期/无效输入当安全、碰撞漏检或执行器危险输出的问题。
- S1：阻断当前里程碑验收；关键功能、构建、测试或安全假设失败。
- S2：可在有明确保守行为和风险说明时继续其他独立工作，但不得跳过相关验收。
- S3：非阻塞维护问题。

## 5. 复现和修复要求

1. 复现必须使用 Issue 记录的 commit 和命令；若当前 HEAD 不同，先 checkout 对应 SHA 或明确差异。
2. 失败测试先保存完整日志，不得先改断言。
3. 随机失败必须保存 seed 和失败向量；尽可能缩减成最小确定性反例。
4. 先判断是环境/工具链、测试夹具、断言、生产代码、接口还是规范问题。
5. 修复必须增加回归测试；禁止删掉原失败用例或放宽安全阈值。
6. 修复 commit 必须链接 Issue；测试日志绑定修复 commit SHA。
7. 原测试、相邻边界、Sanitizer 和受影响集成测试全部复测。
8. 重要数值/安全问题必须由独立数学审查者复核；修复作者不能作为唯一审查者。内部预审、作者自测或 CI PASS 均不能替代独立审查。

## 6. 多问题并发排查

多个失败同时出现时：
1. 先按 S0→S1→S2→S3 排序。
2. 先处理共同根因（如 ABI、时间基准、公共数据契约、构建参数）。
3. 不要把多个错误堆叠修复后只跑一次总测试；每个根因要有独立复现/回归。
4. 建立父子/关联 Issue，记录共同阻塞。
5. 修复一个问题后重跑受影响的相邻测试，排除连带回归。

## 7. 每次状态更新模板

```text
Issue: FLIGHT-<number>
Status: <state>
Severity: S0|S1|S2|S3
Branch / baseline SHA:
First seen SHA:
Fix SHA:
Retest SHA:
RUN_ID:
Test IDs:
Command:
Environment/toolchain:
Expected:
Actual:
Checks / failures:
Seed / failing vector:
Log path + SHA256:
Safety impact / fail-closed:
Blocker / exit criteria:
Next action / owner:
```

## 11. R16 本地临时登记与复验规则

Agent 无 GitHub 写权限时，`ISSUE_REGISTER.md` 的“本地临时登记”只用于保留复验证据；
不得将它描述为已发布 GitHub Issue。每条必须保留原始失败日志和 draft。状态最多到
`FIXED_WAITING_RETEST`，必须由独立审查者在 GitHub Issue 上复验后才能进入
`VERIFIED/CLOSED`。R16 的 `FLIGHT-LOCAL-22`（4,128 B 栈）保持 `OPEN/BLOCKED`，
不得用降低安全要求或压缩缓冲区的方式关闭。

## 12. R17 阻塞项治理

- `ISSUE_REGISTER.md` 是本地正式临时登记入口；`BLOCKER_BOARD.md` 是派生视图，
  不得与登记状态分叉。
- 每个 BLOCKED 项必须有责任方、审批事项、解除条件、复测命令和下一次检查点。
- GitHub 未授权时保留 `PUBLICATION_PENDING`；不得虚构 Issue 编号或自行
  VERIFIED/CLOSED。
- B01/B02（Spec/ABI EOL 重基线）、B03/B04（包版本/链接）、B05（MR-009）、
  B06（4,232 B 栈）、B07（H723 实机）在 R17 继续保留真实失败/BLOCKED 状态；
  该句为 R17 历史状态；MR-009 当前状态以 2026-11-13 R31 独立审查交接记录和 ISSUE_REGISTER 为准。

## 13. R19 发布状态规则

- 当前 `GITHUB_PUBLICATION = PARTIAL`；GitHub Issues #8–#19 已在线跟踪；`spec/quality/ISSUE_REGISTER.md` 是正式登记，
  `BLOCKER_BOARD.md` 只是派生视图。
- 未授权时不得创建/声称 GitHub Issue 编号，不得把本地状态写成 CLOSED。
- 授权后才可把本地条目同步到 GitHub Issues，并保持状态一致；GitHub 在线 Issue
  是同步后的在线入口。
- 独立复验前，本地状态最多 `FIXED_WAITING_RETEST`；`VERIFIED/CLOSED` 只能由
  独立审查者确认。

## 14. R28 严格证据与交接规则

- Collector 只接受 `FLIGHT_VALIDATION_LOG_V1` wrapped log；PASS 的
  checks/failures 必须从真实日志解析，不能由调用者填写。
- 每个 PASS 必须绑定 RUN_ID、code commit、command、exit code、duration、
  真实 toolchain、seed 策略、source manifest（git blob/worktree SHA/line
  ending/size/role）和 log SHA。
- `see r25_environment`、`see log`、`unknown`、`deterministic`、
  `NOT_RECORDED` 不得作为工具链/seed/时长占位。
- S06-108 与 S06-112 必须记录真实数字 seed；S06-112 JSON 与 test_results
  必须一致。
- Evidence checker 必须真实比较 Manifest/Environment/Test Results/Archive/
  Issues/Blocker/Product/Stack/Handover；UNKNOWN 不得 PASS。
- 交接按 `AGENT_SESSION_HANDOVER.md` 执行；新 Agent 必须独立核对 SHA 并生成
  `TAKEOVER_ACK.md`，不得凭旧聊天记录接管。
- 归档后的元数据提交必须单独记录，不得声称被旧 ZIP 包含；R22 原始字节仍为
  `NOT_RECOVERABLE`。


## 15. R29 归档元数据与证据门禁

- 新归档必须使用 `spec/quality/ARCHIVE_METADATA_PROTOCOL.md` 的 PREZIP + external digest 两阶段协议。
- 禁止 `PENDING_ARCHIVE_*`、`ahead_commit_count: null` 和归档后元数据提交。
- `prepare_archive_metadata.py` 在 ZIP 创建前从真实 Git 动态生成 archive head、remote、ahead、tooling/collector/script commits 和 tool source identity。
- ZIP 自身 SHA256/size 只能写入外部 report 和 `.sha256` sidecar，不得嵌入 ZIP 内部 Manifest。
- R29 checker 必须拒绝旧 R28 后置元数据模式；UNKNOWN/BLOCKED 不得 PASS。
- B01/B02/B03 的历史审批和当前状态以最新阻塞板为准；B05/MR-009 的独立审查要求已由项目方于 2026-11-14 明确豁免，仅限该项；其他安全问题仍遵循独立审查要求。


## 16. 原生 FMT-SIM 本机复测责任与交接

- 所有需要 Windows 原生 FMT-SIM 引擎的动态场景由项目 Owner 在本机执行；远程 Agent 不代跑、不推断、不伪造 FMT-SIM 结果。
- Agent 继续负责生产代码修改、Host/Host SIL/集成/Sanitizer、静态检查、CI、失败分析、回归测试、SPEC/Issue 更新和分支提交。
- FMT-SIM 的正式命令、配置、350 ms 门槛、日志与 SHA256 归档、失败回传、Agent 后续处理顺序，统一以 spec/validation/FMT_SIM_LOCAL_HANDOFF.md 和 tests/simulation/run_fmt_sim_regression.ps1 为准；关闭审查还要求本地 HEAD 与 upstream tracking HEAD 一致。
- 本机运行必须记录 branch、完整 HEAD、FMT-SIM binary/settings SHA256、运行参数、逐场景退出码和原始日志 SHA256；未提供证据的字段写 NOT_RUN / UNKNOWN。
- #17/#18/#19 只有在对应原生场景完成其终端回执、连续 LANDED 证据、DISARMED 回执及 350 ms 状态流条件后，才可进入复核；Host/CI PASS 不替代原生 FMT-SIM 复测。
- FMT-SIM 软件仿真不替代 B07 的 H723 目标运行、物理 HIL、WCET/RTA、目标栈水位或飞行验收。
