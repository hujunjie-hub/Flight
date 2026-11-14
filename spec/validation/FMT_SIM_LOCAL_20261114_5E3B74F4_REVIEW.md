# R33 FMT-SIM 本机归档复核报告

## 1. 结论

**整批 FAIL / 不具备 Issue 关闭资格。** 本归档本身有效，测试环境和正式门禁均有效；Land、Return 单场景通过，Mission 有 3 项终端阶段失败。该包测试的是修复前 main HEAD `5e3b74f`，不能用于验证随后提交的接地候选交接修复。

## 2. 归档完整性与测试基线

- RUN_ID：`FMT-SIM-LOCAL-20261114T084723472Z-5e3b74f4`
- 测试分支 / HEAD：`main / 5e3b74f40293a005de693eceb61af4c0552e`
- ZIP：`FMT-SIM-LOCAL-20261114T084723472Z-5e3b74f4.zip`
- ZIP SHA256：`0d6b75c578e2ae71b501f9f26b827559fe8b0887598253af55d09d09f63cdcfb`；sidecar 完全匹配。
- ZIP 共 104 entries；`SHA256SUMS.txt` 共 102 项，全部重新校验通过；`manifest.sha256` 与 `manifest.json` 匹配。
- manifest SHA256：`6604b49d3fe889c9c022b63dec313b762880d7aa8cc12e736975257e2490971d`
- Mission log SHA256：`c4259174d9133df9dc184cf3e8f65df3eb34fc0e9b4b4ac58ea85a7cf14dc4b2`
- Mission FMT-SIM engine log SHA256：`fa7f1e8276c29496bc6ec80fad1d7250bee3343410f519a3332b56ad2898247c`
- Build：PASS，exit 0；SIL EXE SHA256：`3b30c2911748cfd641f4c4e50ccf24ae131cb2798f3eaffad142d31fab5501d1`
- FMT-SIM EXE SHA256：`4f9a0f12c78e3964b3e472ee579c547d13ea629456dc6a4c7fe72ea18ae2a6d6`
- settings SHA256：`f556717bcd96a32206065d5dbb97a8f9e57e2267c055bd75e3d6bc501102948b`
- main / origin-main 一致，fetch 成功；worktree 运行前后均 clean 且稳定。
- 门禁：UDP 14653、状态超时 350 ms、graphics adapter 1、hover 0.62、Owner 显式确认 127.0.0.1 loopback。
- 最长成功状态间隔：三个场景均 94.8 ms，低于 350 ms。
- PowerShell 7.6.10、GCC/G++ 16.2.0、Python 3.14.8、Git 2.55.0.windows.5。
- 引擎实际选择 Microsoft Basic Render Driver；三个 runner stderr 均为空。

## 3. 场景结果

| 场景 | 检查次数 | 失败数 | 结果 | 最长状态间隔 |
|---|---:|---:|---|---:|
| Land | 39,388 | 0 | PASS | 94.8 ms |
| Return | 40,485 | 0 | PASS | 94.8 ms |
| Mission | 67,112 | 3 | FAIL | 94.8 ms |
| **合计** | **146,985** | **3** | **FAIL** | **94.8 ms** |

Land 最后 0.50 m 最大下降速度 0.350 m/s；Return 为 0.322 m/s，返航落点距离 0.079 m。Mission 的两个航点最近距离分别为 0.233 m、0.331 m，返航落点距离 0.171 m。

## 4. Mission 失败断言

```text
land_position_reference_seen=0 return_mode_seen=1
FAIL 1720: terminal_ack == COMMAND_ACK_COMPLETED
FAIL 1721: disarm_ack == COMMAND_ACK_COMPLETED
FAIL 1726: runtime_data.fms.arm_state == ARM_STATE_DISARMED
[fmt] return touchdown distance 0.171 m
```

Mission 最后 0.50 m 最大下降速度为 **0.374 m/s**，满足 0.40 m/s 上限；本轮不再出现 R32 的近地速度超限。Mission 最终状态仍为 ARMED，未取得终端命令完成 ACK 和 DISARM ACK。

注意：`land_position_reference_seen=0` 不是独立失败原因，因为同一批成功的 Return 场景也记录该值；该场景实际走的是 RETURN 模式（`return_mode_seen=1`）。

## 5. 根因分析

R33 在 t=34.280 s 首次记录 ground contact=CONTACT，但 FMS landed=UNKNOWN 且 evidence=0x0；当时水平速度约 0.291 m/s，未满足 M4 的 0.20 m/s 稳定门槛。旧代码在 UNKNOWN 状态清空接地证据，M5 无法提前停止追逐 Home/probe。最终快照 contact=1、landed=UNKNOWN、stable_ms=0、evidence=0x0，nav_vh=0.161 m/s、nav_vd=-0.005 m/s、nav_ad=0.688 m/s²；垂向加速度仍超过 0.5 m/s² 稳定门槛。

R33 的首个物理接触样本为：

```text
[settle] t=34280 contact=1 landed=0 landed_stable_ms=0 landed_evidence=0x0
height=0.0597 v=-0.2503,0.1490,0.0869 a=-0.1496,-0.9308,0.0501
```

当时水平速度约 `sqrt(0.2503² + 0.1490²) = 0.291 m/s`，超过 FMS 0.20 m/s 的稳定接地门槛，所以 M4 正确保持 UNKNOWN。旧实现的 UNKNOWN 分支同时把 evidence 清零，M5 只能继续执行 Home XY 追踪和 below-plane probe，无法在正式 LANDED dwell 完成前先切换到实测接触点保持。

这不是 350 ms 链路时序问题，也不是 R33 的近地速度包络问题。FMS 拒绝 DISARM 是 fail-closed 行为；缺陷在于物理接触提示没有及时交接给 M5，导致 M4 稳定条件难以收敛。

## 6. R33 后的 main 修复与验证

R33 后 main 新增生产修复：`419f56962f74fd9d89bc62cfc11ee0923910177c` 在导航/能力/速度数据有效且物理接触可信时保留 UNKNOWN 接地候选证据；M5 识别该提示后锁存实测接触 XYZ 并停止继续追逐 Home。该候选只用于 M5 过渡，不授予 LANDED/DISARM 权限。`15a33756d4e269800e6778c6180f7086179c9429` 增加 nav_ok 检查，避免导航指针缺失时被候选分支解引用。M4 的 0.20 m/s 水平速度、0.15 m/s 垂向速度、0.5 m/s² 垂向加速度与 2 s dwell、DISARM gate 均未放宽。测试提交 `3270cff5aec26e3ae0717c62d2d78e9f30752020` 增加缺失导航回归；两个 contact candidate 测试均通过。

Current-main CI on `3270cff5aec26e3ae0717c62d2d78e9f30752020`: Host [#43593701736](https://github.com/hujunjie-hub/Flight/actions/runs/43593701736) 6/6, M6/M7 [#43593701892](https://github.com/hujunjie-hub/Flight/actions/runs/43593701892) 5/5, M7 [#43593701803](https://github.com/hujunjie-hub/Flight/actions/runs/43593701803) 2/2 SUCCESS. M4 969/0; M5 327,030/0; M4/M5 integration 1,042/0; MR-009 focused 430/0; M7 274/0.

这些 Host/Sanitizer/CI 结果不是原生 FMT-SIM 结果。R33 没有测试 `419f56962f74fd9d89bc62cfc11ee0923910177c` 或 `15a33756d4e269800e6778c6180f7086179c9429`，因此不能宣称 Mission 已修复。

## 7. Issue 状态与下一步

- #17 Land：R33 单场景 PASS，但整批不具备关闭资格，继续 OPEN。
- #18 Return：R33 单场景 PASS，但整批不具备关闭资格，继续 OPEN。
- #19 Mission：R33 仍有 3 项终端失败，继续 OPEN。
- #21 近地下降速度：R33 的 0.374 m/s 指标通过；按该 Issue 的完整 Mission/manifest 关闭条件继续 OPEN，待整批通过。
- #22 接地点 / LANDED handoff：R33 复现，代码修复已提交但未经过原生 FMT-SIM 复测，继续 OPEN。

**下一步：** Owner 在最新 main 上重新执行完整 Land → Return → Mission 批量入口；不得只跑 Mission，不得提高 350 ms 门槛，不得放宽 FMS 速度/加速度/dwell/DISARM 条件。只有完整 manifest `issue_closure_eligible=true` 且原始日志复核通过后，才进入 #17/#18/#19/#21/#22 关闭审查。

FMT-SIM 仍不替代 B07 的 H723 目标板、物理 HIL、WCET/RTA、栈水位和飞行证据。
