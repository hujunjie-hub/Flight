# R36 原生 FMT-SIM 归档复核报告

## 1. 结论

R36 正式三场景复测有效，但整体 FAIL，不能作为 ENG-07 关闭证据。三个场景共 164,715 checks / 10 failures；Land、Return、Mission 均未完成终端 ACK、DISARM ACK 和最终 DISARMED。Return 还未满足最终物理接地断言。

R36 没有复现 R35 的 M5 reason 1550 / JERK 拒绝、M6 -11 或 FAILSAFE_TERMINAL。上一轮 69626899 的低速 contact-hold 门控避免了该立即故障，但暴露出另一处问题：M5 允许在水平速度尚未收敛时开始垂直下降，M4 因水平运动不稳定而拒绝 LANDED。

## 2. 归档与环境完整性

| 项目 | 结果 |
|---|---|
| RUN_ID | FMT-SIM-LOCAL-20261114T150346337Z-3fb4dc23 |
| 测试仓库 / 分支 | hujunjie-hub/Flight / main |
| 测试 HEAD | 3fb4dc237e36b55899d3bcf0ca450470e72250ae |
| HEAD / origin/main | 精确匹配 |
| Worktree | 运行前后 clean，稳定 |
| Git fetch | exit 0 |
| ZIP SHA256 | d81bae7ea6684bb4a42b7b523a355dc1f5ff855746542257162c00d9de557ea3 |
| manifest SHA256 | a48dc4c70b8bc75fcb628add77386c932a0ef37972f7f4a589f89d72fb6ddc99 |
| ZIP / 内部文件 | 104 entries CRC PASS；SHA256SUMS 102 项全部通过 |
| Manifest | overall=FAIL；issue_closure_eligible=false |
| Build | PASS，exit 0 |
| FMT-SIM UDP / hover / timeout | 14653 / 0.62 / 350 ms |
| Loopback | 显式确认 127.0.0.1 |
| Graphics adapter | 1 |
| FMT-SIM.exe SHA256 | 4f9a0f12c78e3964b3e472ee579c547d13ea629456dc6a4c7fe72ea18ae2a6d6 |
| settings.jason SHA256 | f556717bcd96a32206065d5dbb97a8f9e57e2267c055bd75e3d6bc501102948b |
| MinGW GCC / G++ | 16.2.0 |
| PowerShell / Python / Git | 7.6.10 / 3.14.8 / 2.55.0.windows.5 |

ZIP 文件大小 5,224,749 bytes；构建产物 build/sil_flight.exe SHA256 为 74491e6624b0b78f5cfd7be7c4df5effb8aa7215729329ce70eb766a15b0dae1。

## 3. 三场景结果

| 场景 | Checks | Failures | 最长状态间隔 | 末段 0.50 m 最大下降速度 |
|---|---:|---:|---:|---:|
| Land | 48,801 | 3 | 92.3 ms | 0.336 m/s |
| Return | 48,802 | 4 | 147.2 ms | 0.338 m/s |
| Mission | 67,112 | 3 | 112.5 ms | 0.351 m/s |
| 合计 | 164,715 | 10 | 均小于 350 ms | 全部小于 0.40 m/s |

三个场景均失败于：terminal_ack、disarm_ack、最终 ARM_STATE_DISARMED。Return 额外失败于 flight_sil_ground_contact() == GROUND_CONTACT_CONTACT。

其他指标：Return touchdown distance 0.220 m；Mission touchdown distance 0.387 m；Mission 两个航点最近距离 0.250 m / 0.367 m。0.40 m/s 末段下降速度指标全部 PASS。

## 4. 首个接地阶段与最终状态

| 场景 | 首次物理接地样本 | 首次接地时水平速度 | 最终状态 |
|---|---|---:|---|
| Land | t=21.940 s，height=0.0597 m，contact=CONTACT | 0.220 m/s | t=32.000 s，height=0.0356 m，contact=CONTACT，landed=UNKNOWN，水平速度 0.270 m/s |
| Return | t=26.200 s，height=0.0593 m，contact=CONTACT | 0.234 m/s | t=32.000 s，height=0.1337 m，contact=NO_CONTACT，landed=IN_AIR，稳定计时 880 ms |
| Mission | t=36.840 s，height=0.0595 m，contact=CONTACT | 0.246 m/s | t=44.000 s，height=0.0136 m，contact=CONTACT，landed=UNKNOWN，水平速度 0.282 m/s |

Land 与 Mission 的最终接地提示存在，但水平速度仍超过 M4 的稳定阈值，LandDetector 保持 UNKNOWN。Return 首次接地时水平速度同样超限，随后接触信号丢失，最终状态回到 IN_AIR。

## 5. 根因分析

### 5.1 M4 接地安全门槛保持不变

当前 fms_land_tick() 仅在导航/能力字段有效、GROUND_CONTACT_CONTACT、水平速度 <=0.20 m/s、垂向速度 <=0.15 m/s、垂向加速度 <=0.5 m/s² 时选择 LANDED，并要求完整 2 s dwell。NO_CONTACT 会明确选择 IN_AIR。R36 的 M4 结果符合该规则，不能通过放宽 M4 阈值或缩短 dwell 解决。

### 5.2 M5 staged landing 的水平/垂直阶段门槛不一致

R36 所测旧代码中，LANDING_APPROACH 的初始选择只在水平误差 >0.30 m 或水平速度 >0.25 m/s 时进入；离开 APPROACH 也允许水平误差 <=0.30 m 且水平速度 <=0.25 m/s。该速度门槛高于 M4 的 0.20 m/s 接地稳定门槛，也高于 M5 contact-hold 使用的 0.15 m/s 最终低速门槛。

因此 M5 可能在 0.20–0.25 m/s 的水平运动仍存在时启动垂直下降。接地候选到达时，R36 三场景水平速度分别为 0.220 / 0.234 / 0.246 m/s；contact-hold 的 0.15 m/s 门槛不会触发，M4 也不能累计 LANDED dwell。该行为导致终端 ACK 与 DISARM 未完成。

这与 R35 的立即重规划 JERK 拒绝不同：R36 GuidanceFeasibility 保持有效，没有观察到 reason 1550、M6 -11 或 FAILSAFE_TERMINAL。M4→M5 advisory evidence=0x1 仍到达，属于 #22 已跟踪的 handoff 行为。

### 5.3 证据文件

| 场景 | sil_flight.log SHA256 |
|---|---|
| Land | 764534e8c10073e19c66d508946531f89cf4cdfdb6e285b522d02712a3e92cd4 |
| Return | 6b409921765b173c220a6275bade5e5bff96113a793f6244482f74ce650f2b6f |
| Mission | 8ad5678caeb08d0732fdcef8c4cfc2116bacdf7dcb3c26d38f7fd26359d6350e |

## 6. R36 后修复与自动验证

- 866f2d79dff583280b9b1e6f657eb29959b46df5：新增 0.15 m 水平误差和 0.15 m/s 水平速度统一门槛；M5 必须先完成水平对准/制动，才允许进入垂直下降；新增 landing_speed_only_requires_horizontal_brake 回归，并收紧 long-return landing regression。
- 507133064b5f9c4176ff793ede8fa6e8a42fcad1：将 brake-to-final 转换也纳入同一 0.15 m / 0.15 m/s 水平稳定门槛。
- 未修改 M4 0.20 m/s、0.15 m/s、0.5 m/s²、2 s dwell、M5 末段 0.40 m/s / 0.12 m/s 限值或 DISARM 授权策略。

最终代码 HEAD 507133064b5f9c4176ff793ede8fa6e8a42fcad1 的 CI 全部 SUCCESS：
- Host validation [#43612480283](https://github.com/hujunjie-hub/Flight/actions/runs/43612480283)：6/6 jobs。
- M6/M7 host validation [#43612480319](https://github.com/hujunjie-hub/Flight/actions/runs/43612480319)：5/5 jobs。
- M7 host validation [#43612480345](https://github.com/hujunjie-hub/Flight/actions/runs/43612480345)：2/2 jobs。

CI PASS 不等同于原生 FMT-SIM PASS。R36 是修复前 HEAD 的原生证据；50713306 之后的原生 FMT-SIM 尚 NOT_RUN。

## 7. Issue 状态

- #17 Land：OPEN，R36 terminal/disarm 未完成。
- #18 Return：OPEN，R36 接地丢失且 terminal/disarm 未完成。
- #19 Mission：OPEN，航点与下降速度指标通过，但 terminal/disarm 未完成。
- #21 末段下降速度：R36 三场景均 PASS；按既有完整 Mission/manifest 关闭规则保持 OPEN。
- #22 M4→M5 advisory handoff：R36 继续观察到 evidence=0x1，独立安全复核仍待完成。
- #23 R35 M5 contact-hold plan rejection：R36 未复现原来的 M5 plan rejection/terminal failsafe；按完整批次和独立复核条件保持 OPEN。
- #24 新问题：M5 在水平运动未收敛前启动垂直下降。修复已提交，等待 R37 原生复测。

## 8. R37 本机复测要求

Owner 在 Windows 工作站从 Flight 仓库根目录执行：

```powershell
git fetch origin
git switch main
git pull --ff-only
git status --short
git rev-parse HEAD
git rev-parse '@{u}'

$sim = (Resolve-Path 'build/validation/sim/fmt_release/FMT_SIM/Binaries/Win64/FMT_SIM.exe').Path
pwsh -NoProfile -File tests/simulation/run_fmt_sim_regression.ps1 -FmtSimExe $sim -FmtHilPort 14653 -FmtStateTimeoutMs 350 -FmtGraphicsAdapter 1 -FmtHover 0.62 -ConfirmLoopbackAddress
```

本次最新 main 基线为 507133064b5f9c4176ff793ede8fa6e8a42fcad1。必须运行完整 Land → Return → Mission；不得更改 350 ms、UDP 14653、hover 0.62 或安全阈值。回传 ZIP 与 SHA256 sidecar。R37 全部场景通过前，不关闭 #17/#18/#19/#24，也不宣称 ENG-07 完成。

FMT-SIM 是带 HIL 链路的 software SIL，不代表 H723 实机、物理台架 HIL、WCET/RTA、栈水位或飞行放行。