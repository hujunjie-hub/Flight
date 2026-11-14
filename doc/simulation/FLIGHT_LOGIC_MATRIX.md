# 飞行与策略仿真验证矩阵

本文件的 PASS 记录均绑定各自历史 RUN_ID；最新 Stage-02 复测曾失败，修复提交 `31a0d3d` 的 Host CI 已通过，但原生 FMT-SIM 尚未复跑，见文末。
本轮使用 `SIM_CONTROLLED_STOP` Host 构建和隔离的 FMT-SIM Ground 场景。
H723 的运行配置仍使用 `BENCH_INHIBIT`。设备固件、UART/DMA、板上时序和实机运动
留在各自的验证门禁。

## 整链证据

| 场景 | 结果 | 证据 |
|---|---|---|
| Host 静态 SIL | 通过 | 3151 项检查，0 失败。预检、解锁、首个有效控制许可、起飞指令和参考切换通过。守卫提交 1576 帧 |
| FMT-SIM SIH | 通过 | 原安装的位姿端点接收 `HIL_STATE` 后画面随高度更新 |
| FMT-SIM 原生 HIL | 双向通过 | Host 收到 `FMT_EXTERNAL_STATE`，仿真机体响应四路已提交的虚拟电机输出 |
| 起飞和悬停 | 通过 | 2 m 起飞回执完成。独立降落场景实测峰值 2.001 m，随后 2 s 悬停最大高度误差 0.032 m |
| 返航接地与解除解锁 | 通过 | Home 续期后返航获准，返航及解除解锁均收到完成回执。落点距起飞点 ≤ 0.5 m，22640 项检查通过 |
| 自动降落 | 通过 | 保持有效 Home 并完成运动参考切换。自动降落和解除解锁均收到完成回执，22940 项检查通过 |
| 双航点、返航接地与解除解锁 | 通过 | 到两个航点的实测最近距离为 0.253 m、0.297 m。任务完成后位置保持稳定，再返航接地并解除解锁，38220 项检查通过 |

日志位于 `build/validation/sim`。静态 SIL 使用 `release_sil/final_sil.log`，
FMT-SIM 独立降落使用 `fmt_release/final_flights/Land.log`，独立返航使用
`fmt_release/home_fix/Return.log`，双航点全流程使用
`fmt_release/mission_handoff/stable.log`。早期降落失败的对照记录在
`fmt_release/closed_loop_hover_062.log` 和 `fmt_release/home_fix/Land.log`。
联机配置、帧时间基准与命令见 [联机说明](HIL_LINK.md)。

## 策略回归

| 范围 | 通过的检查 | 策略和判据 |
|---|---:|---|
| FMS | 356 | 解锁、空中拒绝解除解锁、许可撤销水位、Home 续期与过期、双航点任务、暂停、恢复、取消、遥控接管、RC 丢失返航、导航降级恢复、低能量着陆、执行器故障终止 |
| 制导 | 6003 | 航线和位置保持、姿态/高度模式、许可激活、围栏拒绝、过期时钟与命令拒绝 |
| FMS 与制导组合 | 427 | 真实 M4/M5 首参考握手、起飞参考、导航降级后的垂向备份 |
| 控制与导航 | 2352 | MPC/内环、参考时效、控制分配和 FMT-SIM 复现的变化率边界。另有 M3 桥接 64 项，Cortex-M7 对象编译及重定位链接通过 |
| 运行时 | 3508 | 任务注册、遥测、输入接口和台架抑制 |
| 协议和串口桥 | 124 + 6 | 标准 HIL 与 FMT 扩展帧、CRC、截断、样本适配及 UDP 透传 |
| 传感器处理与导航重放 | 75 + 217 + 581 | 传感器标准化、IMU 注入、观测重放与故障恢复，相关 ARM 源文件编译通过 |

上述策略表为 Host 回归证据。更长航线、复合故障和目标 UART
仍需各自的端到端仿真或台架场景。

## 本轮修复与复验

1. 首次地面解锁允许等待第一份有效控制能力。Host 仿真在主动撤销旧参考后使用
   有限重绑定窗口，守卫继续校验许可和时效。
2. 起飞闭环复现了控制分配原缩放域为空的情况。上一周期输出在本周期变化率和推力
   包线内时，控制器使用它作为增量求解基点。捕获的请求与边界进入控制回归，
   失败时仍由 FMS 和 M7 执行既定保护。目标最坏执行时间待复测。
3. FMT-SIM 该版本的帧时间戳为零。按实测约 10 ms 的机体步长驱动 Host，
   并以 0.62 作为本场景的虚拟悬停标定值。0.65 的试验稳定在约 2.47 m，
   超出 2 m 起飞完成容差。
4. ABI 检查复验有 143 个类型，规范布局与 Host/ARM 差异均为 0，既有 ABI
   锁哈希仍不匹配。目录检查发现额外的空目录 `sensor_device/communication`。
5. M4 用有效位置保持证据续期当前飞行的固定 Home。导航标识变更或证据过期使
   Home 失效。固定的 5 s 首次期限曾导致正常悬停后拒绝返航及降级降落。
6. 双航点测试中，任务完成即切返航和仅等待首个回执的两次试验均触发保护。
   通过的编排在位置保持的回执有效后继续等待 1 秒稳定，随后再请求返航。
   未稳定时的连续切换仍是复合场景的验证项。

仿真放行结论限于上表已经通过的阶段。完整飞行流程以实际机体的阶段完成回执为准。

## 商用前复测

工程放行见 [09 商用前工程评估](../../spec/09_需求验证实施与发布管理.md)。
本轮把任务后稳定返航从脚本移到 FMS，覆盖超时仍保持、飞手接管期间拒绝地面站普通
运动命令、原执行时限之后重取完成回执、滑移/失去接地时拒绝解除解锁，以及超时首参考拒绝。
平坦仿真场景的 M5 先对准再执行末段，规划曲线与实际消费曲线均验证近地速度上界。

| 验证 | 结果 |
|---|---|
| FMS / M5 / FMS 与 M5 集成 | 440 / 7923 / 511 项检查，0 失败 |
| Host 静态 SIL / 运行时 | 3151 / 3508 项检查，0 失败 |
| Cortex-M7 对象与静态预算 | ARM 对象编译通过。ICD 与 M4 共 48864 B，限额 49152 B。M5 为 4672 B，限额 16384 B。目标栈、WCET 和最终内存放置待验证 |
| 双航点后立即请求返航 | 采用 650 ms PC 状态等待窗口。FMS 先保持再返航，最终稳定接地并解除解锁，41753 项检查无失败。最后 0.5 m 实测最大下降速度 0.300 m/s |
| 严格状态流时效 | 350 ms 默认窗口下出现仿真状态超时。停转探测 25 s 收到 381 个 CRC 有效状态帧，最大间隔 385 ms。650 ms 双航点场景的最长成功等待为 498 ms |
| 独立降落复跑 | 350 ms 与 650 ms 两轮分别收到 21 帧与 28 帧。两轮预检均报告导航未就绪（`nav=-2`、`failed=0x030d`），解锁帧为零，降落阶段未执行 |

双航点和链路探测日志位于 `build/validation/sim/commercial_final`，独立降落复跑日志
位于 `build/validation/sim/commercial_landing`，FMS 与制导回归日志位于
`build/validation/sim/commercial_m4_m5`。650 ms 通过记录仅覆盖对应仿真场景的飞行逻辑。
严格链路、目标飞行时序和实物运行仍有独立验证项。

## 启动对齐与故障回归

FMT-SIM 预检和飞行使用同一帧节拍。预检在虚拟输出关闭时积累 4 秒仿真对齐，仿真时间每消费一批新状态推进一个帧周期。PC 单调时钟从上次观察到有效状态起计时，拒绝超过门限后才读取的帧，任何阶段超时均结束该场景。默认 350 ms 仍作为严格链路验证判据。

| 验证 | 结果 |
|---|---|
| 同周期解锁门控 | 新出现的能源故障、失联、严重 FaultSummary、姿态证据丢失及非法能源数据均拒绝解锁。终态在接地上锁后仍禁止重新解锁 |
| 组合故障与回执 | 低电迫降持续覆盖能源证据消失、回弹和失联。当前返航路径被制导拒绝后转降落，旧路径的拒绝结果不起作用。中断的返航回执可重取 |
| 参考和许可交接 | M4/M5 集成验证旧许可撤销后可建立新参考。新控制能力超时则进入终态，超时期间没有可用许可 |
| 软件回归 | FMS 538 项、FMS/M5 611 项、静态 SIL 3151 项、运行时 3508 项检查，均无失败。Cortex-M7 FMS 对象编译通过，ICD 与 M4 静态预算 48864 / 49152 B |
| FMT-SIM 单机原生链路 | 350 ms 与 650 ms 两轮均完成预检对齐并起飞。350 ms 在仿真 t=8.520 s 超时，650 ms 在 t=7.650 s 超时，均结束场景并尝试发送虚拟停机。两轮均未完成降落，链路时效门槛未通过 |

日志位于 `build/validation/sim/commercial_safety`。目标时序、板上故障处置和整机环境验证保留在发布门禁。

## 原生状态流与遥测驱动迁移复测

前次 D3D11 硬件渲染的引擎日志记录了 0.5 秒 GPU 等待超时。隔离实例使用 Microsoft Basic Render Driver、45 FPS 上限及低画质后，两个场景均在 350 ms 原门槛内连续收帧。软件适配器和启动方式见 [联机仿真说明](HIL_LINK.md#4-host-sil-与原生-hil-设置)。

| 场景 | 原生链路 | 飞行结果 |
|---|---|---|
| 独立降落 | 24.0 s 仿真、2406 个有效状态帧，最长间隔 64.1 ms | 进入近地段，未完成接地确认和上锁 |
| 双航点 | 22.4 s 仿真、2252 个有效状态帧，最长间隔 81.6 ms | 完成两航点。返航前稳定等待 6 s 超时，FMS 保持位置 |

Host 运行时共 3519 项检查，覆盖释放数传后 ELRS 继续收帧、HIL UART 以 921600 打开以及 HIL 收发。静态 SIL 3151 项、HIL 编解码 124 项和串口桥 6 个用例通过。Cortex-M7 固件完成编译和链接，链接器仍报告工程对象的枚举宽度差异。日志位于 `build/validation/sim/telemetry_rework`，板上实物通信仍待验证。

## 返航等待与接地确认复测

独立降落基线末段 301 个 M4 周期均具备接地和速度证据，原始垂向加速度只有 83 个周期落在稳定门限内。仿真策略现用同一局部导航速度估计持续加速度，沿用返航和接地停留时间，逐周期检查接地、速度和强冲击。模式切换继续从有效导航取起点位置与速度，并在受限条件下从上一条已验证的活动轨迹取得起始参考加速度。合成 GNSS 的位置与速度改由同一 FMT-SIM 局部状态生成。

| 场景 | 闭环结果 | 末段最大下降速度 | 最长状态帧间隔 | 检查 |
|---|---|---|---|---|
| 独立降落 | t=19.600 s 确认降落，t=19.620 s 解除解锁 | 0.316 m/s | 61.4 ms | 27405 项，0 失败 |
| 双航点返航 | t=17.280 s 接受返航，t=20.700 s 切换模式。最终距 Home 0.415 m，t=29.320 s 解除解锁 | 0.323 m/s | 78.4 ms | 40931 项，0 失败 |

两轮均使用 350 ms 状态流门槛，最终状态为 LANDED 和 DISARMED。FMS、制导、两者集成、静态 SIL 和运行时回归分别通过 871、7944、944、3151 和 3519 项检查。Cortex-M7 固件完成编译和链接。日志位于 `build/validation/sim/flight_settle`，实机接地及输出验证保留在发布门禁。

## 2026-11-14 Stage-02 复测

隔离 Ground/HIL 实例的三个动态场景本轮均失败。Land 的首个落地制导参考被拒绝；Return 已接地但在场景截止前未获接地确认和上锁回执；Mission 完成两个航点后在返航准备阶段超时。三者的状态间隔都低于 350 ms。测试计数和原始日志见 [ENG-07 的现行验收记录](../../spec/09_需求验证实施与发布管理.md#2026-11-14-eng-07-动态-fmt-sim-复测fail)。


## ENG-07 Fix Iteration 1

Stage-02 code fixes are in commits 9ac05a3, 1e7ad7c, 31a0d3d, bf312b8, 1df941b, 52d7ee9, e1333ac, d072106, 03653a2, f6ceaeb and ce00ff9. The three CI workflows on ce00ff9 passed. Host evidence: M4 1,074/0; M5 324,792/0; MR-009 focused 430/0; M4/M5 integration 1,147/0; M5 ASan/UBSan 324,792/0; M4/M5 integration sanitizer 1,147/0.

- Land: active trajectory acceleration is carried into vertical automatic LAND only when the previous reference remains valid and active. The vertical LAND builder reports the actual rejection reason/constraint, reaches the Home plane with zero terminal velocity/acceleration, then executes a bounded 4 s below-plane probe.
- Land/Return: the 0.12 m/s vertical-speed bound applies in the final 0.20 m braking band; the physical contact oracle remains at 0.06 m. Return guidance now stages through a 0.60 m approach point, a 0.20 m braking point, then Home. Only the explicit 0.20 m brake and final Home stages use a copied guidance config with a 25% lower three-axis jerk limit for M6's 20 ms float-boundary reconstruction; the long-return approach retains its original jerk envelope. Vertical-only LAND uses the same braking point before Home and the bounded below-plane probe. Curves entering the braking band are checked with continuous Bernstein bounds; the boundary bisection requires a provably nondecreasing NED-down segment and fails closed on ambiguous crossings.
- Mission: FMS compares M6 capability publication time against the M4 permit-binding event. A 20 ms sample-to-publish delay is covered for both in-mission Return and post-mission Return.

The original Windows FMT-SIM Land/Return/Mission scenarios have NOT been rerun after these changes. The prior 350 ms Stage-02 failures remain the authoritative dynamic baseline; Issues #17/#18/#19 stay OPEN. The next native run uses expanded [settle] trace fields for effective operation/guidance mode, Home validity/position, nav p/v/a, truth velocity, M6 sample/publish timestamps, permit sample/publish timestamps, guidance reference/state/reason and violated constraint. No H723 target/HIL/WCET/stack-watermark result is implied.


## ENG-07 Braking-Margin Correction

The Return trace showed 0.358 m/s at 0.105 m above Home. A 0.06 m-only speed cap leaves 0.045 m before the contact sensor threshold, which is insufficient to brake to 0.12 m/s under the 1 m/s² vertical-acceleration envelope. The guidance braking band is now 0.20 m while the physical contact oracle remains at 0.06 m. Return is staged through 0.60 m approach, 0.20 m brake and Home; vertical LAND stages through the braking altitude, Home, then the bounded 0.25 m probe. Only the explicit 0.20 m brake and final Home stages use a copied guidance config with a 25% lower three-axis jerk limit for M6's 20 ms float-boundary reconstruction; the long-return approach retains its original jerk envelope. The FMS 0.15 m/s LandDetector threshold, contact evidence and dwell remain unchanged.

Host CI on ce00ff9 PASS: Host validation #43547565040, M6/M7 host validation #43547564966, M7 host validation #43547564942. M4 1,074/0; M5 324,792/0; MR-009 focused 430/0; M4/M5 integration 1,147/0; M5 ASan/UBSan 324,792/0; integration sanitizer 1,147/0.

Native FMT-SIM must still be rerun locally before #17/#18/#19 can close.


## ENG-07 原生 FMT-SIM 本机复测责任与证据入口

按项目方要求，后续 Windows 原生 FMT-SIM 动态复测统一由项目 Owner 在本机完成；远程 Agent 负责代码修改、Host/SIL/CI 和失败分析，不代跑原生引擎。正式操作及证据格式见 [FMT-SIM 本机复测交接方案](../../spec/validation/FMT_SIM_LOCAL_HANDOFF.md)。

当前产品代码修复基线为 `ce00ff9f0a8eb4989d1a171c3322f3640de2965c`；最新已完成自动化验证的代码 HEAD 为 `5a8e2c3079d230bbb5ca90ebbf229552487c2be9`。[Host #43552823302](https://github.com/hujunjie-hub/Flight/actions/runs/43552823302)、[M6/M7 #43552823284](https://github.com/hujunjie-hub/Flight/actions/runs/43552823284)、[M7 #43552823258](https://github.com/hujunjie-hub/Flight/actions/runs/43552823258) 分别 6/6、5/5、2/2 SUCCESS；M4 1,074/0、M5 324,792/0、MR-009 focused 430/0、M4/M5 integration 1,147/0、M5 ASan/UBSan 324,792/0；PowerShell Parser 对两个本地 runner 均 PASS。以上均未启动原生 FMT-SIM；修复后的原生 FMT-SIM 尚未复测。#17 Land、#18 Return、#19 Mission 保持 OPEN，待 Owner 在本机以同一隔离安装、UDP 14653、350 ms、hover 0.62，Owner 显式确认 loopback 127.0.0.1 后顺序运行，并回传 manifest、完整日志和 SHA256。Host/CI 通过不关闭动态仿真 Issue；FMT-SIM 通过也不代表 H723 实机、物理 HIL 或飞行通过。


## ENG-07 Native FMT-SIM: local execution tool

The project owner runs all native Windows FMT-SIM scenarios locally. The formal batch runner `tests/simulation/run_fmt_sim_regression.ps1` builds SIL once, starts a fresh simulator process for Land, Return and Mission, and emits a unique manifest, raw logs, SHA256SUMS and a handoff ZIP. Each simulator process is closed by its owned PID before the next scenario starts. The strict gate requires hover 0.62, an explicit owner loopback confirmation, a clean checkout and local HEAD matching the upstream tracking HEAD for issue-closure eligibility. Issues #17/#18/#19 remain OPEN until the returned local evidence is reviewed. See [FMT_SIM_LOCAL_HANDOFF.md](../../spec/validation/FMT_SIM_LOCAL_HANDOFF.md).


## ENG-07 R30 archive intake — stale code baseline

The owner returned R30 archive SHA256 `eb52b0d6430662cc0a88869570dcf23c00c964add676d2ef31870c4ae9909063`; ZIP and all 31 internal hashes validate. The five diagnostic runs all failed (201,306 checks / 29 failures). However, the tested commit is `3299a4f`, 181 commits behind current branch HEAD `9063f310`; its worktree was dirty and the package lacks the formal batch-runner manifest and loopback-confirmation evidence. This does not retest the ENG-07 fixes now on the branch. No production code change is justified from this stale run. Keep #17/#18/#19 OPEN and rerun from a clean current-HEAD clone using `run_fmt_sim_regression.ps1`. See [the full R30 archive review](../../spec/validation/FMT_SIM_R30_20261114_ARCHIVE_REVIEW.md).

## ENG-07 R31 FMT-SIM local run — old HEAD, all three scenarios FAIL

Owner archive RUN_ID `FMT-SIM-LOCAL-20261114T045144621Z-304fbd12`, tested HEAD `304fbd12078679d46b80cb626aba9eb0727b6914`. The archive has a clean worktree, successful fetch, exact upstream HEAD match, build PASS, 350 ms timeout, hover 0.62, UDP 14653 and explicit loopback confirmation. ZIP/sidecar SHA256 match; all 102 internal file hashes pass.

| Scenario | Checks | Failures | Longest state gap | Failing assertions |
|---|---:|---:|---:|---|
| Land | 36,600 | 4 | 78.7 ms | near-ground descent bound, terminal ACK, DISARMED ACK, arm state |
| Return | 36,601 | 3 | 48.0 ms | terminal ACK, DISARMED ACK, arm state |
| Mission | 43,692 | 6 | 48.5 ms | near-ground descent bound, terminal ACK, DISARMED ACK, arm state, ground contact, return touchdown error |

Total: 116,893 checks / 13 failures. The test HEAD is seven commits behind the integration source HEAD `bcbe4db5e665e5f678069afc1b206a00ae1cd36d`; those later commits include FMS Return binding and M5 landing-stage/probe corrections. Therefore this run is a valid failure record for `304fbd12`, not a post-fix result for the later code or current main. Keep #17/#18/#19 OPEN and rerun the formal batch runner on clean `main` / `origin/main`. The subsequent integrated main HEAD `d7666c5aa093dbef36fef11b82909e91fdc61491` passed Host [#43567633122](https://github.com/hujunjie-hub/Flight/actions/runs/43567633122) 6/6, M6/M7 [#43567633103](https://github.com/hujunjie-hub/Flight/actions/runs/43567633103) 5/5 and M7 [#43567633072](https://github.com/hujunjie-hub/Flight/actions/runs/43567633072) 2/2 CI workflows; this still does not substitute for the required native FMT-SIM rerun. Detailed hashes and failure lines: [R31 archive review](../../spec/validation/FMT_SIM_LOCAL_20261114_304FBD12_REVIEW.md).


## ENG-07 R32 FMT-SIM local run — Land/Return PASS, Mission 3 failures

Owner archive RUN_ID `FMT-SIM-LOCAL-20261114T072713056Z-e470f00c`, tested main HEAD `e470f00cd539f438b1923b0faa844456ba2d3b59`. Branch/upstream match, clean stable worktree, fetch success, explicit loopback, UDP 14653, hover 0.62 and 350 ms gate all verified. Build PASS; ZIP/sidecar SHA256 match; manifest SHA256 match; all 102 internal hashes pass.

| Scenario | Checks | Failures | Longest state gap |
|---|---:|---:|---:|
| Land | 48,043 | 0 | 145.3 ms |
| Return | 44,667 | 0 | 64.4 ms |
| Mission | 62,793 | 3 | 65.5 ms |

Mission's failures: final 0.50 m maximum descent 0.665 m/s (limit 0.40); DISARMED ACK rejected with reason 11; arm state not DISARMED. At 41,140 ms Return request 2163 completed; DISARM request 3064 was rejected at 41,160 ms. Final contact=1 but landed=UNKNOWN, nav horizontal speed 0.204 m/s and touchdown distance 0.352 m. This is consistent with the M5 reference continuing to chase Home XY after LANDED confirmation and exceeding the FMS 0.20 m/s stable-ground gate. The old M5 trajectory validator only limited the final 0.20 m to 0.12 m/s, while the native acceptance measured the full final 0.50 m.

Fix commit `3d8d29b5dfda21237f8ed02db3c31223c656bfbe` adds a 0.50 m / 0.40 m/s bound, retains the final 0.20 m / 0.12 m/s bound, and latches touchdown XY after LANDED. FMS safety thresholds are unchanged. Host #43574875444 6/6, M6/M7 #43574875363 5/5 and M7 #43574875394 2/2 all PASS; new M5 landing speed/contact-hold tests PASS. The patch still needs a new native FMT-SIM batch on latest main. Keep #17/#18/#19 OPEN because the R32 manifest is not closure-eligible. [Full R32 archive review](../../spec/validation/FMT_SIM_LOCAL_20261114_E470F00C_REVIEW.md).
