# FMT-SIM-R30-20261114 归档复核

> **Disposition: RECEIVED / HASH_VALID / DIAGNOSTIC_ONLY / POST_FIX_RETEST_NOT_ACCEPTED**
>
> 本报告记录 Owner 回传的原生 Windows FMT-SIM 证据。它可用于分析 2026-11-11 代码基线上的历史失败，但不能验证 2026-11-14 ENG-07 修复，也不能关闭 Issues #17/#18/#19。

## 1. 归档完整性

- ZIP：`FMT-SIM-R30-20261114.zip`
- 外部 SHA256：`eb52b0d6430662cc0a88869570dcf23c00c964add676d2ef31870c4ae9909063`
- 外部 sidecar：与 ZIP 计算值一致。
- 内部 `SHA256SUMS.txt`：31 个文件条目全部匹配。
- ZIP 共 34 个条目；包含环境、命令、结果、场景配置、原始 SIL/FMT-SIM 日志、Git 状态和报告。

归档字节完整性 **PASS**。这不代表场景通过。

## 2. 代码来源与严格门禁

归档记录的来源：

| 字段 | 归档值 |
|---|---|
| Branch | `opt/stage-02-m5-06-safety-validator` |
| Tested HEAD | `3299a4f0702083ca4374ccc978edccb5d0c900da` |
| HEAD subject | `docs(quality): bind R29 test ledger hash` |
| HEAD date | 2026-11-11 15:31:47 UTC |
| Archive remote-tracking HEAD | `51e7e923419733f5aa3b21efa1e46588629623ed` |
| Recorded ahead count | 96 |
| Worktree | DIRTY |

复核时 GitHub 当前目标分支 HEAD 为 `9063f31053ac301b71c806858a67e84f69402414`；GitHub compare 显示当前分支相对测试 HEAD **ahead 181 commits / behind 0**。测试 HEAD 因而不是当前分支的修复后代码。

当前分支在测试 HEAD 之后已经包含 ENG-07 相关修复，例如：

- `9ac05a3`：Land/Return handoff、垂直 LAND 活动轨迹加速度交接和 M4 Return 许可时间绑定修复；
- `52d7ee9`：Return 进近、制动和 Home 分段；
- `1df941b`：扩大近地制动带；
- `ce00ff9`：把 25% jerk 余量限制在制动及最终 Home 段。

这些变更不在本次测试 HEAD 中。因此本次失败不能被解释为上述修复在当前 HEAD 上失败。

另外，归档中的 `git/status.txt` 显示工作区 DIRTY：ABI/验证证据文件被修改，并存在 R28/R29 归档、patch、audit 文件和 `work/` 未跟踪内容。归档使用的是多条直接 `run_sil.ps1` 命令；没有正式批量入口所生成的 `manifest.json`，也没有 `-ConfirmLoopbackAddress` 的明确记录。归档未证明本地 HEAD 与刚 fetch 的 upstream HEAD 一致。

**结论：严格关闭门禁不满足。该包是有价值的旧代码诊断证据，不是修复后验收证据。**

## 3. 场景结果

以下为 `results.json` 和对应原始日志记录：

| 场景 | Checks | Failures | 最长状态间隔 | 末态/主要偏差 | 结果 |
|---|---:|---:|---:|---|---|
| Land / Ground / 2 vehicles | 36,600 | 5 | 95.6 ms | 高度约 0.766 m；近地下降速度 0；未完成终端 ACK、DISARMED、接地断言 | FAIL |
| Return / Ground / 2 vehicles | 36,601 | 6 | 79.2 ms | 高度约 0.762 m；Return touchdown error 1.174 m；未完成终端 ACK、DISARMED、接地断言 | FAIL |
| Mission / 2 waypoints + Return | 54,905 | 9 | 81.3 ms | 航点距离 0.553 m / 1.577 m；Return touchdown error 0.586 m；mission success、return accepted 和航点误差断言失败 | FAIL |
| Land / Ground / 1 vehicle | 36,600 | 5 | 67.1 ms | 高度约 0.736 m；近地下降速度 0；失败项与双机体 Land 相同 | FAIL |
| Land / Beach / 1 vehicle | 36,600 | 4 | 208.4 ms | 真值高度约 0.477 m、导航高度约 0.644 m；近地下降速度 0.109 m/s；终端 ACK、DISARMED、接地断言失败 | FAIL |

五个有效场景合计 **201,306 checks / 29 failures，5/5 FAIL**。各场景记录的状态间隔均小于 350 ms；因此这些失败不能归因于记录到的状态流间隔超过 350 ms。

`attempt_1_mainsettings` 是启动配置错误：厂商默认 `Map=MainMenu`，FMT-SIM 未打开 UDP 14653。该尝试没有进入飞行场景，不计入上表的五个有效场景。随后使用 Ground/Beach 场景配置的五个实际运行均已启动并执行。

## 4. 从日志能确认的事实

1. Ground Land 双机体和单机体结果接近：末态真值高度约 0.736–0.766 m，导航高度约 0.738–0.765 m。该场景下导航高度与仿真真值接近；现有证据不支持把 Ground 失败直接归因于 INS 高度发散。
2. Ground Return 同样停留在约 0.762 m，末态没有通过接地、终端 ACK 和解除解锁断言。
3. Mission 运行到结束时仍未满足任务成功、Return 接受及两个航点误差断言；这是返航/着陆前的任务闭环失败，不能仅归结为着陆末段。
4. Beach 场景末态真值高度约 0.477 m、导航高度约 0.644 m，存在约 0.167 m 差异；这提示场景/高度参考差异，需在修复后当前 HEAD 的完整 trace 中复查，不能仅凭此归因。
5. Land 的单机体对照仍失败，因此“第二架 SIH 机体导致 Ground Land 失败”不受本次对照结果支持。

## 5. 原报告中的根因假设需要保留为未证实

报告把约 0.74 m 的高度与 0.60 m approach 点之差约 0.14 m，描述为落在 0.20 m 门限之外；但 `0.14 <= 0.20`，这一算术关系并不支持该结论。日志也没有直接给出 `landing_stage`、Home 高度/坐标和规划段是否到期等必要状态。末态快照显示 guidance `reason=0`、`feasible=1`，不足以证明是 approach-stage 条件卡住。

因此“着陆参考平面偏移”“M5 分段门限卡住”仍是待验证假设，不作为已确认根因。当前不根据旧 HEAD 的假设改动生产代码，以免重复修复已经提交的代码或改变安全判据。

## 6. 下一步：修复后重新执行正式门禁

1. 保留本次旧工作区，不执行 `reset`、`clean` 或覆盖 R28/R29 归档。
2. 在新的干净克隆中检出当前目标分支；核对 `git rev-parse HEAD` 与 `git rev-parse '@{u}'` 完全相同，且 `git status --short` 为空。
3. 使用隔离 FMT-SIM 配置：Ground 场景、恰好一架 HIL/UDP 载具绑定 14653、`Autobound=true`，Owner 人工确认 loopback 为 `127.0.0.1`。可以保留额外 SIH 载具作为场景配置，但必须只有一架匹配端口的 HIL 载具。
4. 从仓库根目录运行正式批量入口（不要使用 `-AllowDirty`）：

```powershell
$sim = (Resolve-Path 'build/validation/sim/fmt_release/FMT_SIM/Binaries/Win64/FMT_SIM.exe').Path
pwsh -NoProfile -File tests/simulation/run_fmt_sim_regression.ps1 -FmtSimExe $sim -FmtHilPort 14653 -FmtStateTimeoutMs 350 -FmtGraphicsAdapter 1 -FmtHover 0.62 -ConfirmLoopbackAddress
```

5. 回传正式入口生成的 ZIP 和 `.sha256`；归档应包含 manifest、summary、每场景完整日志、退出码和 SHA256。若仍失败，按首次失败时刻和完整 trace 定位，不放宽接地、速度、dwell、permit 或 350 ms 条件。

在修复后当前 HEAD 的正式原生复测完成前，Issues #17/#18/#19 继续 OPEN。FMT-SIM PASS 也不替代 B07 H723 实机/HIL、WCET/RTA、栈水位或飞行验收。
