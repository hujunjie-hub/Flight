# R34 FMT-SIM 本机归档复核报告

## 1. 结论

**R34 整批 FAIL / 不具备 Issue 关闭资格。** 归档完整、正式门禁有效，但 Land、Return、Mission 均在接地候选阶段触发 M5 轨迹规划拒绝并进入 FMS terminal failsafe。Mission 另有最后 0.50 m 下降速度超限。

- RUN_ID：`FMT-SIM-LOCAL-20261114T125449954Z-85f02af5`
- 测试分支 / HEAD：`main / 85f02af5ff2208f805b870d70593a73733db2f5c`
- ZIP：`FMT-SIM-LOCAL-20261114T125449954Z-85f02af5.zip`
- ZIP SHA256：`aaa98517fdcd5da799942a739eec47eddec6a5aa8fe23267b1617a7172d464c0`；sidecar 匹配。
- ZIP 大小：5,025,570 bytes；104 entries；CRC 校验通过。
- `SHA256SUMS.txt`：102 项，102/102 内部 SHA256 匹配。
- manifest SHA256：`8a8813929ba6733b12553b9ef0992e2943fbe2601283bdda69e33cfcf1b4bf82`
- 构建：PASS，exit 0；SIL EXE SHA256 `31b0a476a4e10a348c826cc9bfabc3745f6e71df4fde888082b7849cb89c3dd1`
- FMT-SIM EXE SHA256：`4f9a0f12c78e3964b3e472ee579c547d13ea629456dc6a4c7fe72ea18ae2a6d6`
- settings SHA256：`f556717bcd96a32206065d5dbb97a8f9e57e2267c055bd75e3d6bc501102948b`
- main / origin/main 一致，fetch 成功；运行前后 worktree clean 且稳定。
- 门禁：UDP 14653、状态超时 350 ms、graphics adapter 1、hover 0.62、Owner 显式确认 loopback `127.0.0.1`。
- manifest：`overall=FAIL`，`issue_closure_eligible=false`。

PowerShell 7.6.10、GCC/G++ 16.2.0、Python 3.14.8、Git 2.55.0.windows.5。FMT-SIM EXE/settings SHA 与 R33 相同；本轮构建与飞行轨迹日志均重新绑定到本 RUN_ID。

## 2. 三场景结果

| 场景 | 检查数 | 失败数 | 结果 | 最长状态间隔 |
|---|---:|---:|---|---:|
| Land | 33,837 | 4 | FAIL | 113.3 ms |
| Return | 33,346 | 4 | FAIL | 96.1 ms |
| Mission | 49,123 | 5 | FAIL | 110.3 ms |
| **合计** | **116,306** | **13** | **FAIL** | — |

三个状态间隔都远低于 350 ms；不是状态链路超时。

### Land

失败断言：`1695 failsafe_level != FAILSAFE_TERMINAL`、`1720 terminal_ack`、`1721 disarm_ack`、`1726 arm_state == DISARMED`。

- t=22.160 s：contact=1、landed=UNKNOWN、evidence=0x1、confidence=0.50。
- height=0.0583 m；nav vD=0.1187 m/s、aD=0.7835 m/s²。
- M5 guidance `4/1550`，feasibility reason=1550，violated constraint=2（SPEED）。
- t=22.161 s control rejected status=-11；随后命令 ACK FAILED reason=12。
- 最后 0.50 m 最大下降速度 0.332 m/s，速度指标通过。

### Return

失败断言与 Land 相同：terminal failsafe、terminal ACK、DISARM ACK、最终 DISARMED。

- t=21.840 s：contact=1、landed=UNKNOWN、evidence=0x1、confidence=0.50。
- height=0.0593 m；nav vD=0.1276 m/s、aD=0.8265 m/s²。
- M5 guidance `4/1550`，constraint=2（SPEED）；control rejected status=-11，ACK FAILED reason=12。
- 最后 0.50 m 最大下降速度 0.341 m/s；返航落点距 Home 0.142 m。

### Mission

失败断言：terminal failsafe、最后 0.50 m 下降速度、terminal ACK、DISARM ACK、最终 DISARMED。

- 两个航点最近距离：0.205 m、0.283 m；返航落点距 Home 0.050 m。
- t=24.610 s、height=0.490 m，最后 0.50 m 最大下降速度 0.674 m/s，超过 0.40 m/s 上限。
- t=32.180 s：contact=1、landed=UNKNOWN、evidence=0x1、confidence=0.50。
- height=0.0410 m；nav vD=0.0048 m/s、aD=2.9480 m/s²。
- M5 guidance `4/1550`，constraint=3（ACCELERATION）；control rejected status=-11，ACK FAILED reason=12。

## 3. 首个失败链与根因判断

三个场景共同的终端故障链：

1. M4 发布 UNKNOWN 状态下的 advisory physical-contact evidence（`evidence=0x1`）；此证据不授予 LANDED 或 DISARM 权限。
2. M5 接到候选后，旧实现将 `plan_valid` 清零并把目标覆盖为实测接触点。
3. 新轨迹因此从原始导航 p/v/a 重新起算，而不是从 M5 当前已验证轨迹的参考状态起算。接地瞬态/估计加速度可使起始状态超出轨迹约束。
4. Land / Return 首次拒绝为 SPEED constraint=2；Mission 首次拒绝为 ACCELERATION constraint=3。统一 reason=1550（ENVELOPE_INVALID）。
5. M6 控制输出被拒绝（status=-11），FMS fail-closed 进入 FAILSAFE_TERMINAL（failsafe=3、cause=18），命令 ACK reason=12，无法完成 DISARM。

这说明 R33 后 M4 advisory contact evidence 已经到达 M5，但 M5 contact-hold 重新规划仍不健壮。不能通过放宽速度/加速度约束、LANDED dwell 或 DISARM gate 解决。

Mission 近地速度超限是另一个问题：RETURN 的长水平对准与垂直下降同时进行，日志显示近地段仍有较大的水平运动，实际下降速度在 0.50 m 边界达到 0.674 m/s。仅靠最终 0.50 m 参考速度约束未能保证这段耦合运动下的实际 Plant 速度。

## 4. R34 后 main 已提交的修复

测试 HEAD `85f02af5` 之后，main 已提交以下修复：

- `c9cd17926239e088c105b8506052354fdda0d503`：contact hold 不再丢弃活动轨迹，也不在调用 plan() 前覆盖旧目标；新保持轨迹从 M5 当前有效参考状态采样起算，目标仍是实测接触位置。
- `1f9b4ecdbbe90b3e8be9439ec2170478a8083de2`：增加 Host 回归。着陆接触测试注入 raw nav 垂向速度 0.128 m/s、垂向加速度 2.95 m/s²，验证保持轨迹不从该瞬态重新起算；长返航测试验证水平未对准时保持当前高度。
- `d8ea0f05915d0dc52bd73d606c1f3963fff7d25b`：将 landing stage 拆分为：
  1. LANDING_APPROACH：当前高度水平对准 Home，等待水平位置/速度稳定；
  2. LANDING_DESCENT：水平对准后下降到 Home 上方 0.60 m 进近架；
  3. LANDING_BRAKE：到 0.20 m 制动高度；
  4. LANDING_FINAL：到 Home 平面；
  5. LANDING_PROBE：执行既有有界探针；
  6. LANDING_CONTACT_HOLD：从活动有效参考重新规划到实测接触位置并保持。
- 原有最后 0.50 m / 0.40 m/s、最后 0.20 m / 0.12 m/s、M4 0.20 m/s 水平速度、0.15 m/s 垂向速度、0.5 m/s² 垂向加速度、2 s dwell 和 DISARM 判据均未放宽。

## 5. d8ea0f05915d0dc52bd73d606c1f3963fff7d25b 自动化验证

精确绑定到当前 main HEAD `d8ea0f05915d0dc52bd73d606c1f3963fff7d25b`：

- Host validation [#43599814765](https://github.com/hujunjie-hub/Flight/actions/runs/43599814765)：6/6 jobs SUCCESS。
- M6/M7 host validation [#43599814768](https://github.com/hujunjie-hub/Flight/actions/runs/43599814768)：5/5 jobs SUCCESS。
- M7 host validation [#43599814733](https://github.com/hujunjie-hub/Flight/actions/runs/43599814733)：2/2 jobs SUCCESS。

Host 证据：
- M4 Host：969/0。
- M5 Host：329,654/0。
- M5 ASan/UBSan：329,654/0。
- M4/M5 integration：1,042/0。
- M4/M5 integration sanitizer：1,042/0。
- MR-009 focused：430/0。
- Cortex-M7 M4/M5/M6/M7 production-module compile：`M5_CORTEX_M7_COMPILE_OK`。
- Spec audit、Production ABI、M6/M7 Host + Sanitizer、M7 Host + Sanitizer 均通过。

CI 不启动 Windows 原生 FMT-SIM，不能替代本轮修复后的动态复测。

## 6. Issue 状态与下一步

- #17 Land：R34 FAIL，M5 contact-hold 重规划修复已提交，待原生复测。
- #18 Return：R34 FAIL，M5 contact-hold 重规划修复已提交，待原生复测。
- #19 Mission：R34 FAIL，终端故障与近地速度超限均待原生复测。
- #21 Mission near-ground speed：R34 实测 0.674 m/s，超过 0.40 m/s；分阶段着陆修复已提交，待原生复测。
- #22 M4 advisory contact handoff：R34 确认 contact=1、landed=UNKNOWN、evidence=0x1 已到达 M5；R33 的 handoff 缺陷已由 `419f5696` / `15a33756` 修复，仍待独立安全复核。
- #23 M5 contact-hold replan：R34 确认 handoff 后的 M5 重规划从 raw nav 瞬态起算并触发 reason 1550；`c9cd1792` / `d8ea0f05` 修复已提交，待原生复测。

**#17/#18/#19/#21/#22/#23 均保持 OPEN。** 其中 #17/#18/#19/#21/#23 等待完整原生复测；#22 等待独立安全复核。Owner 下一轮必须先 fetch 最新 main，再运行完整 Land → Return → Mission 批量入口；不可只跑单场景，不可提高 350 ms 门槛，不可放宽安全阈值。

FMT-SIM 软件 SIL 不替代 B07 H723 目标板运行、物理 HIL、WCET/RTA、目标 RAM/stack watermark、执行器响应或飞行证据。
