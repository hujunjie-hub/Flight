# ENG-07 原生 FMT-SIM 本机复测交接方案

> 状态：**OWNER_LOCAL_RUN_REQUIRED / NOT_RUN_AFTER_FIX**
>
> 适用分支：**main**
>
> 当前生产代码修复基线：**15a33756d4e269800e6778c6180f7086179c9429**；接地候选交接修复：**419f56962f74fd9d89bc62cfc11ee0923910177c**；测试补强：**3270cff5aec26e3ae0717c62d2d78e9f30752020**（运行前 fetch 最新 origin/main）
>
> 本方案只覆盖 Windows 原生 FMT-SIM + Flight Host SIL 软件动态复测；不代表 H723 真机、台架 HIL、WCET/RTA、目标栈水位或飞行验收。

## 1. 执行责任与长期约定

从本方案生效起，凡需要启动 Windows 原生 FMT-SIM 的动态场景，统一由项目 Owner 在自己的 Windows 工作站本地执行。远程 Agent 不启动、不代跑、不声称已执行原生 FMT-SIM。

| 工作 | 执行方 | 责任 |
|---|---|---|
| 代码分析、缺陷定位、修改与回归用例 | ChatGPT Agent | 在指定分支完成代码修改；不删减失败断言、不放宽安全阈值 |
| Host 单元/集成/Sanitizer、静态检查、Host SIL、CI | ChatGPT Agent | 在可用远程环境执行并绑定 commit SHA、日志和 CI URL |
| Windows 原生 FMT-SIM Land / Return / Mission | 项目 Owner 本地 | 拉取分支，按本方案运行，保存完整日志和环境指纹 |
| FMT-SIM 结果分析与下一轮修复 | ChatGPT Agent | 用户回传归档后分析首个失败点、确定根因、修改代码和测试、更新 SPEC/Issue、推送 main |
| H723 真机 / 物理台架 HIL / 飞行 | 对应硬件责任人 | 按 B07 独立计划执行；不得用 FMT-SIM 或 Host SIL 替代 |

“本地 FMT-SIM”指 Windows 上的原生仿真引擎与隔离 Ground/HIL 实例。它与不启动 FMT-SIM 的 Host SIL、GitHub Actions、H723 板载 HIL 是不同验证层级。

## 2. 当前代码修复与动态验证状态

本轮已提交的相关代码修复包括：

- M5 垂直自动 LAND 允许沿用仍有效且已验证的活动轨迹加速度；拒绝时保留实际 reason / constraint。
- LAND 先经 0.20 m 制动高度，再到 Home 平面，以零终端垂向速度/加速度结束参考，随后执行有界探针。
- LAND / RETURN 在 Home 上方最后 0.50 m 内限制垂向速度不超过 0.40 m/s；最后 0.20 m 继续限制为 0.12 m/s。物理接地判据仍为 0.06 m，LandDetector 原有接地、速度、加速度和 dwell 条件未放宽。
- RETURN 分为 0.60 m 进近、0.20 m 制动和最终 Home 段；25% 三轴 jerk 余量只作用于制动和最终段，不压缩长距离进近段。
- FMS 首次确认 LANDED 后，M5 锁存实际接地点的水平坐标并保持至解除武装交接结束；LANDED 快照短暂丢失时不再恢复追逐 Home XY。
- 新增 UNKNOWN 状态下的 advisory physical-contact evidence：在 ground contact、导航/能力/速度字段有效且垂向速度满足候选条件时，M5 提前锁存实测接触 XYZ 并停止追逐 Home/probe；该提示不授予 LANDED 或 DISARM 权限。M4 仍要求水平速度 ≤0.20 m/s、垂向速度 ≤0.15 m/s、垂向加速度 ≤0.5 m/s² 和完整 2 s dwell。导航无效时不得读取候选数据。
- M4 Mission→Return 许可绑定比较使用 M6 capability 的 publish 时间，覆盖 20 ms sample-to-publish 延迟。

生产代码修复提交 `3d8d29b5dfda21237f8ed02db3c31223c656bfbe` 负责 0.50 m 速度包络与 LANDED touchdown XY latch；R33 后的接地候选交接修复提交 `419f56962f74fd9d89bc62cfc11ee0923910177c`，导航有效性 guard 提交 `15a33756d4e269800e6778c6180f7086179c9429`。测试补强提交 `3270cff5aec26e3ae0717c62d2d78e9f30752020` 增加缺失导航回归。Owner 复测前必须 fetch 最新 origin/main；生产代码修复基线为 `15a33756d4e269800e6778c6180f7086179c9429`。

修复提交 3d8d29b5dfda21237f8ed02db3c31223c656bfbe 的三组 CI 全部通过：
- [Host validation #43574875444](https://github.com/hujunjie-hub/Flight/actions/runs/43574875444)：6/6 jobs SUCCESS。
- [M6/M7 host validation #43574875363](https://github.com/hujunjie-hub/Flight/actions/runs/43574875363)：5/5 jobs SUCCESS。
- [M7 host validation #43574875394](https://github.com/hujunjie-hub/Flight/actions/runs/43574875394)：2/2 jobs SUCCESS。

Host 证据包括 M4 945/0、M5 326,004/0、MR-009 focused 430/0、M4/M5 integration 1,018/0、M5 ASan/UBSan 326,004/0、integration sanitizer 1,018/0；Production ABI、Spec Audit、Cortex-M7 production-module compile、MPC RAM checks 和 PowerShell runner parse 均 PASS。

Test-only follow-up `2f8118f1b149183e5df2d6e9c9941a52bd52562e` expands `landing_alignment_and_speed` to the 2.4 m horizontal / 2.4 m above-Home Mission-return geometry and follows the complete approach, brake, Home-plane and probe stages; the touchdown-hold fixture now checks both X and Y after the LANDED snapshot drops. This commit changes tests only; the production fix remains `3d8d29b5dfda21237f8ed02db3c31223c656bfbe`. CI on `2f8118f1`: Host #43576600195 (6/6), M6/M7 #43576600154 (5/5), M7 #43576600112 (2/2), all SUCCESS.

接地候选修复 `419f56962f74fd9d89bc62cfc11ee0923910177c` / nav guard `15a33756d4e269800e6778c6180f7086179c9429` 后，最新测试提交 `3270cff5aec26e3ae0717c62d2d78e9f30752020` 的 CI：Host #43593701736（6/6）、M6/M7 #43593701892（5/5）、M7 #43593701803（2/2）全部 SUCCESS；M4 1,042/0、M5 327,030/0、M4/M5 integration 969/0、MR-009 focused 430/0、M7 274/0。

**以上均不是 R33 后接地候选修复的原生 FMT-SIM 证据。** R33 测试 HEAD `5e3b74f` 早于 `419f5696` / `15a33756`，完整 Land / Return / Mission 批量复测仍 NOT_RUN；#17/#18/#19/#21/#22 均保持 OPEN。

## 3. 本机运行前置条件

1. Windows 工作站；从 Flight 仓库根目录运行 PowerShell 7（pwsh）。批量入口会用 PowerShell Parser 检查 run_sil.ps1 语法。
2. 当前分支为 main，工作区干净；运行前执行 git fetch origin，确认本地 HEAD 与 upstream tracking HEAD 一致，不要只记录分支名。
3. 本机已配置 MinGW，默认路径为 D:\STM32CubeCLT\mingw64\bin；若安装位置不同，调整命令中的 GccDir。
4. 使用隔离的 FMT-SIM 安装，不要指向生产/飞行实例。正式批量入口只记录 EXE 文件名与 SHA256，不复制 FMT-SIM 二进制。默认路径示例：

   build/validation/sim/fmt_release/FMT_SIM/Binaries/Win64/FMT_SIM.exe

5. 隔离实例的 Content/Settings/settings.jason 配置恰好一架 HIL 载具：SimMode=HIL、MavproxyFCS.Interface=UDP、本机回环地址 127.0.0.1、本地 UDP 端口 14653、Autobound=true。现有 runner 会检查指定端口恰好匹配一架 HIL UDP 载具且 Autobound=true；当前脚本不解析地址字段，因此请人工确认 loopback 地址，并随回传提供脱敏配置片段。
6. 运行前关闭占用 UDP 14653 的旧实例/进程。每个场景由 run_sil.ps1 -FmtSimExe 启动独立 FMT-SIM 进程，并在退出时关闭窗口、超时则按该进程 PID 强制结束并等待退出；三个场景顺序运行，不并发启动。
7. 图形适配器默认编号 1 是既有测试主机的软件渲染适配器编号。其他电脑按本机情况设置 FmtGraphicsAdapter，并检查 fmt_sim_engine.log 确认实际适配器。
8. 验收超时固定为 **350 ms**。不得把 650 ms 或更宽松的诊断运行改报为 350 ms 验收通过。

## 4. 一次性执行 Land / Return / Mission

先更新到当前分支的最新远端 HEAD，并确认工作区干净：

```powershell
git fetch origin
git switch main
git pull --ff-only
git status --short
git rev-parse HEAD
git rev-parse '@{u}'
```

从 Flight 仓库根目录执行正式批量入口。它先构建一次 SIL 可执行文件，再按 Land → Return → Mission 顺序分别启动和关闭全新的 FMT-SIM 进程：

```powershell
$sim = (Resolve-Path 'build/validation/sim/fmt_release/FMT_SIM/Binaries/Win64/FMT_SIM.exe').Path
pwsh -NoProfile -File tests/simulation/run_fmt_sim_regression.ps1 -FmtSimExe $sim -FmtHilPort 14653 -FmtStateTimeoutMs 350 -FmtGraphicsAdapter 1 -FmtHover 0.62 -ConfirmLoopbackAddress
```

若 FMT-SIM 安装路径不同，只修改 `$sim`；MinGW 路径不同时使用 `-GccDir`。批量入口自身先执行 `git fetch --prune origin`，再拒绝错误分支、dirty worktree、非 `origin/main` upstream 或本地 HEAD 与刚拉取的 upstream HEAD 不一致。正式关闭资格还要求命令显式带 `-ConfirmLoopbackAddress`（表示 Owner 已人工确认 settings 中的地址为 127.0.0.1）、hover 保持 0.62、端口 14653 和 350 ms；否则只生成诊断证据。它也会检查 settings.jason 中指定端口恰好匹配一架 HIL/UDP 载具且 Autobound=true；回环地址 127.0.0.1 仍需按本方案人工核对。

批量入口调用 `run_sil.ps1 -BuildOnly` 构建一次，再通过 `run_sil.ps1 -ReuseBuild` 执行各场景；每个场景有独立 OutDir 和 FMT-SIM 引擎进程。即使一个场景失败，也会继续收集其余场景，逐项记录退出码，不覆盖之前 RUN_ID。

单场景诊断可执行：

```powershell
pwsh -NoProfile -File tests/simulation/run_fmt_sim_regression.ps1 -FmtSimExe $sim -Scenario Land
pwsh -NoProfile -File tests/simulation/run_fmt_sim_regression.ps1 -FmtSimExe $sim -Scenario Return
pwsh -NoProfile -File tests/simulation/run_fmt_sim_regression.ps1 -FmtSimExe $sim -Scenario Mission
```

单场景运行只用于定位问题，即使 PASS 也不能关闭整组 ENG-07 门禁。650 ms 或 1000 ms 可用于诊断链路，不属于严格 350 ms 验收，不得用于关闭 #17/#18/#19。若未显式带 `-ConfirmLoopbackAddress`，或 hover 不是 0.62，也只能作为诊断运行。

每次运行的原始目录为：

```text
build/validation/fmt-sim-local/FMT-SIM-LOCAL-<UTC时间>-<HEAD前8位>/
  manifest.json
  manifest.sha256
  summary.md
  SHA256SUMS.txt
  build/
    sil_flight.exe
    sil_flight.log
    build.stdout.log
    build.stderr.log
  Land/
    sil_flight.exe
    sil_flight.log
    fmt_sim_engine.log
    exit.txt
    runner.stdout.log
    runner.stderr.log
  Return/
    ...
  Mission/
    ...
```

脚本还会在原始目录旁生成同名 ZIP 和独立的 .sha256 sidecar，控制台输出 RUN_ID、ARCHIVE 和 ARCHIVE_SHA256。ZIP 仅用于本机证据回传；如需纳入正式项目归档，仍须另按 ARCHIVE_METADATA_PROTOCOL.md 执行，不把该 ZIP 自动视为正式项目归档。

Manifest 记录 schema、RUN_ID、branch、完整 commit、dirty before/after、upstream HEAD 一致性、FMT-SIM EXE/settings SHA256、HIL 端口、图形适配器、hover、Owner 对 127.0.0.1 loopback 的显式确认、状态超时、工具链版本、构建结果、逐场景开始/结束时间、退出码、checks/failures、最长成功状态间隔和原始日志 SHA256。不会复制 FMT-SIM 安装文件或完整 settings 配置进归档。

## 5. 三个场景的验收条件

| Issue | 场景 | 必须观察到的结果 |
|---|---|---|
| [#17 Land](https://github.com/hujunjie-hub/Flight/issues/17) | 自动降落 | 首个自动 LAND 参考被接受；完成降落终端命令回执；连续有效接地/LANDED 证据成立；收到 DISARMED 回执 |
| [#18 Return](https://github.com/hujunjie-hub/Flight/issues/18) | 自动返航 | 进入返航并完成 Home 末段；连续有效接地/LANDED 证据成立；返航终端完成并收到 DISARMED 回执 |
| [#19 Mission](https://github.com/hujunjie-hub/Flight/issues/19) | 双航点任务后返航 | 两个航点均完成；任务完成后 Return 准备和许可绑定成功；完成着陆终端回执、连续 LANDED 证据和 DISARMED 回执 |

整组验收还必须满足：

- 三场景均使用同一隔离 FMT-SIM 安装/配置、同一 UDP 端口和 350 ms 状态超时设置；按顺序运行，不并发。
- 每个场景进程退出码为 0，Host 日志无失败断言；状态帧最大间隔满足 350 ms 判据。
- 保留 [settle] 追踪：有效操作/制导模式、Home 有效性和坐标、导航 p/v/a、仿真真值速度、M6 sample/publish 时间戳、permit sample/publish 时间戳、guidance reference/state/reason 和 violated constraint。
- Land/Return/Mission 的终端回执、连续 LANDED 证据和 DISARMED 回执必须分别成立；不能用其中一个场景的结果代替另外两个。
- 不允许通过关闭断言、改变场景输入、放宽接地/速度/dwell/许可条件或改用 650 ms 来取得 PASS。

## 6. 失败时的处理与证据回传

若任一场景失败：

1. 不要清理或覆盖该 RUN_ID 目录；保留完整 sil_flight.log 和 fmt_sim_engine.log。
2. 不要修改场景阈值、接地判据、许可条件或固件代码后再补跑并覆盖原始失败。
3. 把 manifest.json、三场景日志、exit.txt 和 ZIP 的 SHA256 一起回传。
4. 同时提供配置文件中 HIL 载具的脱敏片段（仅需证明 SimMode、UDP interface、loopback、LocalPort 和 Autobound；不要发送密码、token 或无关个人配置）。
5. 若日志未显示完整 [settle] 追踪，明确标注缺失，不要推测根因。

归档默认位于：

build/validation/fmt-sim-local/<RUN_ID>.zip

可将 ZIP 直接作为对话附件提供。不要把 FMT-SIM 安装目录、可执行文件或包含密钥的完整配置提交到 Git。

## 7. Agent 收到本机结果后的处理顺序

1. 核对归档中的 branch、commit、settings / binary SHA256、命令参数和日志完整性。
2. 将实际 FMT-SIM 结果绑定到运行时的代码 SHA；若用户运行的 HEAD 与当前分支不同，先对齐差异。
3. 按 #17/#18/#19 分别定位首个失败时刻、状态、reason、constraint、参考和许可时间戳；不以最终状态单点猜根因。
4. 对已证实根因逐项修改生产代码并增加 Host 回归测试；必要时增加 Host SIL 场景回归。
5. 运行受影响 Host、集成、Sanitizer、静态检查和 CI；更新 SPEC、Issue、阻塞表和下一轮本地 FMT-SIM 命令。
6. 推送 main 分支，交付新的精确 HEAD 和本地复测步骤。用户再次在本机运行 FMT-SIM；未取得新日志前，#17/#18/#19 保持 OPEN。

**禁止状态跃迁：** Host/SIL/CI PASS 不会自动关闭 #17/#18/#19；本机 FMT-SIM 未运行时必须记为 NOT_RUN；FMT-SIM 软件仿真 PASS 也不能关闭 B07 的 H723 实机/HIL/WCET/栈水位/飞行门禁。


## 8. R30 归档接收与复核结论

2026-11-14 已收到 `FMT-SIM-R30-20261114.zip`。ZIP/sidecar 与内部 31 项 SHA256 均匹配，但归档测试 HEAD 为 `3299a4f0702083ca4374ccc978edccb5d0c900da`，而复核时 GitHub 当前目标分支 HEAD 为 `9063f31053ac301b71c806858a67e84f69402414`（前进 181 commits）。旧包工作区 DIRTY、upstream 记录过期，且使用直接 `run_sil.ps1` 场景命令，没有正式批量入口 manifest 和显式 loopback 确认。因此它是旧代码诊断，不是修复后验收；五个有效场景均 FAIL，#17/#18/#19 仍 OPEN。详细逐场景数据和未证实假设见 [R30 归档复核报告](FMT_SIM_R30_20261114_ARCHIVE_REVIEW.md)。

**不要在原有 DIRTY 工作区执行 reset/clean。** 请保留旧归档，在新的干净克隆检出最新目标分支后运行本节第 4 节的正式批量入口；脚本会自行 fetch 并拒绝 HEAD/upstream 不一致、dirty worktree 或缺少 `-ConfirmLoopbackAddress` 的关闭资格。

## 2026-11-14 main 唯一分支整合与 R31 复测登记

- 唯一工作分支：`main`。PR [#20](https://github.com/hujunjie-hub/Flight/pull/20) 已合并，合并提交 `eacb966b502b7f9cb547617e866641504346a5fd`；完整 177 提交历史已进入 main。后续本机 FMT-SIM 必须在 main 上运行。 旧 `opt/spec-sync-review` 与 `opt/stage-02-m5-06-safety-validator` 已通过一次性清理 workflow run [#43567201449](https://github.com/hujunjie-hub/Flight/actions/runs/43567201449) 删除；远程分支盘点只剩 `main`，临时清理 workflow 已移除。
- 合并源代码 HEAD：`bcbe4db5e665e5f678069afc1b206a00ae1cd36d`。该提交 13 项 CI checks 全部 SUCCESS；main 合并提交的 Host validation #43566259362（6/6）SUCCESS。M6/M7 与 M7 独立 workflow 已调整为 main push 触发，本次 runner/SPEC 更新后的 main-only CI 已全部通过。 Main HEAD `d7666c5aa093dbef36fef11b82909e91fdc61491` 的三组工作流全部 SUCCESS：Host [#43567633122](https://github.com/hujunjie-hub/Flight/actions/runs/43567633122) 6/6、M6/M7 [#43567633103](https://github.com/hujunjie-hub/Flight/actions/runs/43567633103) 5/5、M7 [#43567633072](https://github.com/hujunjie-hub/Flight/actions/runs/43567633072) 2/2；Host PowerShell runner parse step PASS。
- Owner 回传 RUN_ID `FMT-SIM-LOCAL-20261114T045144621Z-304fbd12`，测试 HEAD `304fbd12078679d46b80cb626aba9eb0727b6914`，测试时 branch/upstream 均为旧 stage-02 分支且一致、worktree clean。该 HEAD 比合并源 HEAD 落后 7 commits，不能代表当前 main。
- R31 结果：Land 36,600/4 FAIL；Return 36,601/3 FAIL；Mission 43,692/6 FAIL；合计 116,893 checks / 13 failures。最长状态间隔分别 78.7/48.0/48.5 ms，均小于 350 ms。构建 PASS，严格 issue-closure eligibility 为 false。ZIP SHA256 `af97c2ac451c2937fab186ccd5e1a9f553b4ee6d699470ab9963e0b105505d26` 与 sidecar 匹配；内部 102 项哈希全部通过。
- 逐场景失败断言、环境指纹和哈希见 [R31 本机归档复核报告](FMT_SIM_LOCAL_20261114_304FBD12_REVIEW.md)。#17/#18/#19 继续 OPEN。
- **下一轮本机复测只使用 main：** `git fetch origin --prune; git switch main; git pull --ff-only origin main`，确认 clean、HEAD 与 `origin/main` 一致后执行本方案正式批量入口。runner 已改为只接受 `main` / `origin/main`；旧 stage-02 checkout 会被拒绝。


## 2026-11-14 R32 本机归档复核与下一轮修复

- RUN_ID：`FMT-SIM-LOCAL-20261114T072713056Z-e470f00c`；正式门禁有效：`main` / HEAD `e470f00cd539f438b1923b0faa844456ba2d3b59`，`origin/main` 完全匹配，worktree clean 且运行期间稳定，fetch 成功，loopback 显式确认，UDP 14653、hover 0.62、350 ms。
- ZIP SHA256 `93f9ad1289a0b30f0a92afa3dee039b37b1e8c03435db54354a2986cdf3eb76d` 与 sidecar 匹配；104 个 ZIP entries CRC 通过；manifest SHA256 `50cd771b3b810bdf8d5dd08c761b201b4e5a19770ee4b9405d93799d0e039234` 匹配；内部 102 项 SHA256 全部通过。构建 PASS。
- 结果：Land 48,043/0 PASS；Return 44,667/0 PASS；Mission 62,793/3 FAIL；总计 155,503 checks / 3 failures。最大状态间隔 145.3 ms，小于 350 ms。Mission 失败为近地 0.50 m 最大下降速度 0.665 m/s（上限 0.40）、DISARMED ACK 和最终 arm state。
- Mission Return ACK 在 41,140 ms 完成，DISARM 在 41,160 ms 被 FMS 以 reason 11 拒绝；最终 ground contact=1 但 landed state UNKNOWN，nav horizontal speed 0.204 m/s，略高于 LandDetector 0.20 m/s 稳定门槛，touchdown distance 0.352 m。R32 完整断言、日志和哈希见 [R32 归档复核报告](FMT_SIM_LOCAL_20261114_E470F00C_REVIEW.md)。
- 根因修复提交：`3d8d29b5dfda21237f8ed02db3c31223c656bfbe`。M5 增加最后 0.50 m / 0.40 m/s 速度包络（最后 0.20 m / 0.12 m/s 保持不变）；首次 LANDED 后锁存 touchdown XY，避免解除武装期间继续追 Home。FMS 接地、速度、加速度、dwell 和 DISARM 判据均未放宽。SIL 新增 LandDetector 稳定时间/证据位/置信度与 near-ground peak 时高诊断。
- Fix commit `3d8d29b5dfda21237f8ed02db3c31223c656bfbe` passed Host validation [#43574875444](https://github.com/hujunjie-hub/Flight/actions/runs/43574875444) 6/6, M6/M7 [#43574875363](https://github.com/hujunjie-hub/Flight/actions/runs/43574875363) 5/5 and M7 [#43574875394](https://github.com/hujunjie-hub/Flight/actions/runs/43574875394) 2/2. M5 Host and ASan/UBSan each passed 326,004/0; M4 Host 945/0; M4/M5 integration and its sanitizer each 1,018/0; MR-009 focused 430/0. New tests `landing_alignment_and_speed` and `landing_contact_latches_xy` passed.
- **下一轮必须在最新 main 上重跑完整 Land → Return → Mission。** R32 的 Land、Return 场景虽然通过，但 manifest 的 `issue_closure_eligible=false`；#17/#18/#19 暂均保持 OPEN，不能以单场景 PASS 或 Host/CI PASS 代替完整原生验收。

## 2026-11-14 R33 归档复核与接地候选交接修复

R33 RUN_ID `FMT-SIM-LOCAL-20261114T084723472Z-5e3b74f4` 在修复前 HEAD `5e3b74f40293a005de693eceb61af4c0552e` 完成：Land 39,388/0 PASS，Return 40,485/0 PASS，Mission 67,112/3 FAIL；总计 146,985 checks / 3 failures，最长状态间隔 94.8 ms。ZIP SHA256 `0d6b75c578e2ae71b501f9f26b827559fe8b0887598253af55d09d09f63cdcfb` 与 sidecar 一致，manifest SHA256 `6604b49d3fe889c9c022b63dec313b762880d7aa8cc12e736975257e2490971d`，内部 102 项 SHA256 全部通过。Mission 最后 0.50 m 最大下降速度 0.374 m/s（上限 0.40，已通过），返航落点距 Home 0.171 m；仅 terminal ACK、DISARM ACK、最终 DISARMED 状态三项失败。

R33 在 t=34.280 s 首次记录 ground contact=CONTACT，但 FMS landed=UNKNOWN 且 evidence=0x0；当时水平速度约 0.291 m/s，未满足 M4 的 0.20 m/s 稳定门槛。旧代码在 UNKNOWN 状态清空接地证据，M5 无法提前停止追逐 Home/probe。最终快照 contact=1、landed=UNKNOWN、stable_ms=0、evidence=0x0，nav_vh=0.161 m/s、nav_vd=-0.005 m/s、nav_ad=0.688 m/s²；垂向加速度仍超过 0.5 m/s² 稳定门槛。

R33 后 main 新增生产修复：`419f56962f74fd9d89bc62cfc11ee0923910177c` 在导航/能力/速度数据有效且物理接触可信时保留 UNKNOWN 接地候选证据；M5 识别该提示后锁存实测接触 XYZ 并停止继续追逐 Home。该候选只用于 M5 过渡，不授予 LANDED/DISARM 权限。`15a33756d4e269800e6778c6180f7086179c9429` 增加 nav_ok 检查，避免导航指针缺失时被候选分支解引用。M4 的 0.20 m/s 水平速度、0.15 m/s 垂向速度、0.5 m/s² 垂向加速度与 2 s dwell、DISARM gate 均未放宽。测试提交 `3270cff5aec26e3ae0717c62d2d78e9f30752020` 增加缺失导航回归；两个 contact candidate 测试均通过。

Current-main CI on `3270cff5aec26e3ae0717c62d2d78e9f30752020`: Host [#43593701736](https://github.com/hujunjie-hub/Flight/actions/runs/43593701736) 6/6, M6/M7 [#43593701892](https://github.com/hujunjie-hub/Flight/actions/runs/43593701892) 5/5, M7 [#43593701803](https://github.com/hujunjie-hub/Flight/actions/runs/43593701803) 2/2 SUCCESS. M4 1,042/0; M5 327,030/0; M4/M5 integration 969/0; MR-009 focused 430/0; M7 274/0.

详细哈希、失败断言、逐帧分析和下一步见 [R33 归档复核报告](FMT_SIM_LOCAL_20261114_5E3B74F4_REVIEW.md)。下一轮必须在最新 main 上执行完整 Land → Return → Mission；R33 的 Land/Return 单场景 PASS 不构成整批关闭资格。