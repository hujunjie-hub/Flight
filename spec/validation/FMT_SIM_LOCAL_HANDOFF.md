# ENG-07 原生 FMT-SIM 本机复测交接方案

> 状态：**OWNER_LOCAL_RUN_REQUIRED / NOT_RUN_AFTER_FIX**
>
> 适用分支：**opt/stage-02-m5-06-safety-validator**
>
> 当前代码修复基线：**ce00ff9f0a8eb4989d1a171c3322f3640de2965c**
>
> 当前交接文档基线：**dffba6fd9e2f25e03083439b37a4d557fe11e24d**
>
> 本方案只覆盖 Windows 原生 FMT-SIM + Flight Host SIL 软件动态复测；不代表 H723 真机、台架 HIL、WCET/RTA、目标栈水位或飞行验收。

## 1. 执行责任与长期约定

从本方案生效起，凡需要启动 Windows 原生 FMT-SIM 的动态场景，统一由项目 Owner 在自己的 Windows 工作站本地执行。远程 Agent 不启动、不代跑、不声称已执行原生 FMT-SIM。

| 工作 | 执行方 | 责任 |
|---|---|---|
| 代码分析、缺陷定位、修改与回归用例 | ChatGPT Agent | 在指定分支完成代码修改；不删减失败断言、不放宽安全阈值 |
| Host 单元/集成/Sanitizer、静态检查、Host SIL、CI | ChatGPT Agent | 在可用远程环境执行并绑定 commit SHA、日志和 CI URL |
| Windows 原生 FMT-SIM Land / Return / Mission | 项目 Owner 本地 | 拉取分支，按本方案运行，保存完整日志和环境指纹 |
| FMT-SIM 结果分析与下一轮修复 | ChatGPT Agent | 用户回传归档后分析首个失败点、确定根因、修改代码和测试、更新 SPEC/Issue、推送分支 |
| H723 真机 / 物理台架 HIL / 飞行 | 对应硬件责任人 | 按 B07 独立计划执行；不得用 FMT-SIM 或 Host SIL 替代 |

“本地 FMT-SIM”指 Windows 上的原生仿真引擎与隔离 Ground/HIL 实例。它与不启动 FMT-SIM 的 Host SIL、GitHub Actions、H723 板载 HIL 是不同验证层级。

## 2. 当前代码修复与动态验证状态

本轮已提交的相关代码修复包括：

- M5 垂直自动 LAND 允许沿用仍有效且已验证的活动轨迹加速度；拒绝时保留实际 reason / constraint。
- LAND 先经 0.20 m 制动高度，再到 Home 平面，以零终端垂向速度/加速度结束参考，随后执行有界探针。
- LAND / RETURN 在 Home 上方 0.20 m 制动带内限制垂向速度；物理接地判据仍为 0.06 m，LandDetector 原有接地、速度、加速度和 dwell 条件未放宽。
- RETURN 分为 0.60 m 进近、0.20 m 制动和最终 Home 段；25% 三轴 jerk 余量只作用于制动和最终段，不压缩长距离进近段。
- M4 Mission→Return 许可绑定比较使用 M6 capability 的 publish 时间，覆盖 20 ms sample-to-publish 延迟。

代码修复基线为 ce00ff9。其后的文档 HEAD dffba6f 三条 CI 全部通过：

- [Host validation #642](https://github.com/hujunjie-hub/Flight/actions/runs/43547823624)：6/6 jobs SUCCESS。
- [M6/M7 host validation #130](https://github.com/hujunjie-hub/Flight/actions/runs/43547823672)：5/5 jobs SUCCESS。
- [M7 host validation #91](https://github.com/hujunjie-hub/Flight/actions/runs/43547823542)：2/2 jobs SUCCESS。

Host 证据包括 M4 1,074/0、M5 324,792/0、MR-009 focused 430/0、M4/M5 integration 1,147/0，以及对应 ASan/UBSan 回归 0 failures。

**以上均不是本轮修复后的原生 FMT-SIM 证据。** 最新代码修复后的 Land、Return、Mission 原生复测仍为 NOT_RUN；历史 RUN_ID M5-06-SOFTWARE-20261114 的三项失败继续保留为修复前动态基线。GitHub Issues #17/#18/#19 保持 OPEN。

## 3. 本机运行前置条件

1. Windows 工作站；从 Flight 仓库根目录运行 PowerShell 7（pwsh）。
2. 当前分支为 opt/stage-02-m5-06-safety-validator，工作区干净；运行前记录实际 HEAD，不要只记录分支名。
3. 本机已配置 MinGW，默认路径为 D:\STM32CubeCLT\mingw64\bin；若安装位置不同，调整命令中的 GccDir。
4. 使用隔离的 FMT-SIM 安装，不要指向生产/飞行实例。默认路径示例：

   build/validation/sim/fmt_release/FMT_SIM/Binaries/Win64/FMT_SIM.exe

5. 隔离实例的 Content/Settings/settings.jason 配置恰好一架 HIL 载具：SimMode=HIL、MavproxyFCS.Interface=UDP、本机回环地址 127.0.0.1、本地 UDP 端口 14653、Autobound=true。现有 runner 会检查指定端口恰好匹配一架 HIL UDP 载具。
6. 运行前关闭占用 UDP 14653 的旧实例/进程。每个场景由 run_sil.ps1 -FmtSimExe 启动并清理同一隔离安装中的 FMT-SIM 进程；三个场景顺序运行，不并发启动。
7. 图形适配器默认编号 1 是既有测试主机的软件渲染适配器编号。其他电脑按本机情况设置 FmtGraphicsAdapter，并检查 fmt_sim_engine.log 确认实际适配器。
8. 验收超时固定为 **350 ms**。不得把 650 ms 或更宽松的诊断运行改报为 350 ms 验收通过。

## 4. 一次性执行 Land / Return / Mission

以下命令从 Flight 仓库根目录在 PowerShell 7 中执行。若 FMT-SIM 安装路径不同，只修改 $sim；若 MinGW 路径不同，只修改 $gccDir。

~~~~powershell
$ErrorActionPreference = 'Continue'

$branch = (git branch --show-current).Trim()
$head = (git rev-parse HEAD).Trim()
if ($branch -ne 'opt/stage-02-m5-06-safety-validator') { throw "Wrong branch: $branch" }
$dirtyBefore = @(git status --porcelain)
if ($dirtyBefore.Count -ne 0) { throw 'Working tree must be clean before the FMT-SIM acceptance run' }

$sim = (Resolve-Path -LiteralPath 'build/validation/sim/fmt_release/FMT_SIM/Binaries/Win64/FMT_SIM.exe').Path
$simDir = Split-Path -Parent $sim
$simRoot = Split-Path -Parent (Split-Path -Parent $simDir)
$settings = Join-Path $simRoot 'Content/Settings/settings.jason'
if (-not (Test-Path -LiteralPath $settings -PathType Leaf)) { throw "Missing FMT-SIM settings: $settings" }

$gccDir = 'D:\STM32CubeCLT\mingw64\bin'
$runId = 'M5-06-FMT-SIM-LOCAL-' + (Get-Date -Format 'yyyyMMdd-HHmmss')
$runRoot = Join-Path (Get-Location).Path "build/validation/fmt-sim-local/$runId"
New-Item -ItemType Directory -Force -Path $runRoot | Out-Null

$results = @()
foreach ($scenario in @('Land', 'Return', 'Mission')) {
    $outDir = Join-Path $runRoot $scenario
    New-Item -ItemType Directory -Force -Path $outDir | Out-Null
    & pwsh -NoProfile -File (Join-Path (Get-Location).Path 'tests/simulation/run_sil.ps1') -GccDir $gccDir -OutDir $outDir -FmtSimExe $sim -FmtHilPort 14653 -FmtScenario $scenario -FmtStateTimeoutMs 350 -FmtGraphicsAdapter 1 -FmtHover 0.62
    $exitCode = $LASTEXITCODE
    "exit_code=$exitCode" | Set-Content -LiteralPath (Join-Path $outDir 'exit.txt') -Encoding utf8
    $results += [pscustomobject]@{
        scenario = $scenario
        exit_code = $exitCode
        log = (Join-Path $outDir 'sil_flight.log')
        engine_log = (Join-Path $outDir 'fmt_sim_engine.log')
    }
}

$logHashes = @(
    Get-ChildItem -LiteralPath $runRoot -File -Recurse |
    Where-Object { $_.Name -in @('sil_flight.log', 'fmt_sim_engine.log', 'exit.txt') } |
    ForEach-Object {
        [pscustomobject]@{
            path = $_.FullName.Substring($runRoot.Length + 1)
            size_bytes = $_.Length
            sha256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash
        }
    }
)

$manifest = [ordered]@{
    schema = 'FMT_SIM_LOCAL_HANDOFF_V1'
    run_id = $runId
    branch = $branch
    commit = $head
    dirty_before = $dirtyBefore
    dirty_after = @(git status --porcelain)
    fmt_sim_exe = $sim
    fmt_sim_sha256 = (Get-FileHash -LiteralPath $sim -Algorithm SHA256).Hash
    settings_path = $settings
    settings_sha256 = (Get-FileHash -LiteralPath $settings -Algorithm SHA256).Hash
    gcc_dir = $gccDir
    fmt_hil_port = 14653
    state_timeout_ms = 350
    graphics_adapter = 1
    hover = 0.62
    scenarios = $results
    logs = $logHashes
}
$manifest | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $runRoot 'manifest.json') -Encoding utf8

$archive = "$runRoot.zip"
Compress-Archive -Path (Join-Path $runRoot '*') -DestinationPath $archive -CompressionLevel Optimal -Force
$archiveHash = (Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash

$results | Format-Table -AutoSize
Write-Host "RUN_ID=$runId"
Write-Host "ARCHIVE=$archive"
Write-Host "ARCHIVE_SHA256=$archiveHash"
~~~~

该命令会对每个场景分别编译并运行，保留独立目录：

- Land/sil_flight.log、Land/fmt_sim_engine.log、Land/exit.txt
- Return/sil_flight.log、Return/fmt_sim_engine.log、Return/exit.txt
- Mission/sil_flight.log、Mission/fmt_sim_engine.log、Mission/exit.txt
- 根目录 manifest.json：记录分支、HEAD、工作区状态、FMT-SIM 可执行文件和设置文件 SHA256、端口、超时、图形适配器、各场景退出码及日志 SHA256。

脚本即使某个场景失败也会继续执行剩余场景，便于一次收集完整矩阵；每个场景的退出码单独记录。任何场景失败都不能判为整组 PASS。

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
6. 推送修复分支，交付新的精确 HEAD 和本地复测步骤。用户再次在本机运行 FMT-SIM；未取得新日志前，#17/#18/#19 保持 OPEN。

**禁止状态跃迁：** Host/SIL/CI PASS 不会自动关闭 #17/#18/#19；本机 FMT-SIM 未运行时必须记为 NOT_RUN；FMT-SIM 软件仿真 PASS 也不能关闭 B07 的 H723 实机/HIL/WCET/栈水位/飞行门禁。
