# Flight M5-06 阻塞项视图（R39）

> **2026-11-13 MR-009 R31 内部预审 CI 快照（代码/证明基线 8dcb60b3ac18e6a763c939d3e810984092cdd321）**：Host validation #43446380866（6/6）、M6/M7 #43446380836（5/5）、M7 #43446380724（2/2）全部 SUCCESS。M5 Host regression、M5 ASan/UBSan、MR-009 focused、S06-099/S06-107、Spec Audit、生产 ABI、18 mutation FP audit、Cortex-M7 模块编译、M6/M7 Host/Sanitizer/SIL 与 M7 Host/Sanitizer 均 PASS。

> **当前状态（2026-11-14）**：B01～B04 的本地规范/生产 ABI/安装包锁/默认 Release 链接通过。B05 MR-009 按项目方指示完成技术复核，S06-100～102 Host 与 ASan/UBSan 通过。B06 的 Cortex-M7 专用镜像静态链为 `-O2` 3,576/4,096 B、`-O0` 3,968/4,096 B，均 COMPLETE。B07 实机、WCET/RTA、栈水位、台架 HIL 与飞行仍 NOT_RUN。M5-07 未启动。

> **开放风险**：ICD+M4 余量 264 B（FLIGHT-13）；Release 的工具链枚举属性仍有 228 条链接警告（FLIGHT-14）。下方 B01～B06 的表格保留各原始验证时点。

本文件是 `spec/quality/ISSUE_REGISTER.md` 的**派生视图**，不维护第二套状态。

> **当前 ENG-07 动态验收状态（R39）**：RUN_ID `FMT-SIM-LOCAL-20261114T174058156Z-547f3788`，测试 HEAD `547f37883186c888d786c4293f2a4341d348b994`；main/origin/main 精确匹配、worktree clean/stable、loopback 显式确认、UDP 14653、hover 0.62、350 ms 均有效。ZIP SHA256 `3a09ba6f4e2f7adda930ffb1633f498f3c8bb31f3c1d581ccc8043cdeeb78dfb`；构建 PASS；`issue_closure_eligible=false`。
>
> R39 Land **48,801/3**、Return **48,802/3**、Mission **67,112/5**。三个场景均无 `FAILSAFE_TERMINAL`、无控制拒绝；剩余失败为 `terminal_ack`、`disarm_ack`、最终 `DISARMED`（Mission 另加近地实测 0.496 m/s 与结束 `contact=NO_CONTACT`）。Return touchdown 0.143 m、Mission 0.315 m、航点 0.292/0.413 m、近地下降 0.300/0.293 m/s，均通过各自阈值。
>
> 根因链已闭合到单一剩余项：R38（`a5df9070`）证明 CONTACT_HOLD 的"重新规划到实测接地点"会在接地瞬间以 1550/JERK 拒绝整份参考（Return t=28.28 s、Mission t=40.12 s，锁存 `FAILSAFE_TERMINAL`），而 Land 因 0.25 m/s 高于 0.15 m/s 等待门限永不进入该分支并在地面滑移。`a9a38bbb` + `547f3788` 改为静态零速度保持并加入"失去接地即释放"，确定性失败消失。剩余 [FLIGHT-26](https://github.com/hujunjie-hub/Flight/issues/26)：接地后约 0.2 m/s、周期约 1.4 s 的旋转水平极限环（导航与真值一致），使 M4 的 |vh| ≤ 0.20 与 2 s dwell 无法连续满足；M5 冻结/跟随两种语义均复现，接地前静态参考下亦存在，2 m 悬停 `hold_error` 仅 0.019–0.028 m。
>
> 精确 HEAD `547f3788` CI：Host validation、M6/M7 host validation、M7 host validation 全部 SUCCESS。M4 的 0.20 m/s、0.15 m/s、0.5 m/s²、2 s dwell、M5 0.40/0.12 m/s 与 DISARM 策略均未放宽。
>
> #17/#18/#19/#21/#23/#24/#25 保持 OPEN（技术闭合待 #26 与独立复核）；#22 独立安全复核待完成；#26 为本轮新增 S1。唯一开发分支仍为 `main`。详见 [spec/09 R38/R39 节](../09_需求验证实施与发布管理.md)。
>
> 历史 R31–R38 基线保留在下方历史记录。Host/SIL/CI 不是 H723 实机、物理 HIL、WCET/RTA、栈水位或飞行放行证据。

## B01 Spec Audit FAIL

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-05 |
| 状态 | **PASS（2026-11-15，T12 spec_layout）**；历史：BLOCKED / FAIL |
| 根因 | `dialect_report.json` 钉住 CRLF 变体哈希；Git/LF 工作区是另一哈希。文本未变，属于行尾表示差异。 |
| 证据 | `python spec/tools/audit_spec.py --round 3 --label m4-m5-implementation` exit 1；`build/m4_m5_implementation/evidence/spec_layout.log`。 |
| 安全影响 | 规范证据不可复现；不直接改变轨迹计算。 |
| 解除条件 | OWNER 批准 LF 重基线；真实审计 PASS；独立复核。 |
| 下一步 | 准备 OWNER 审批材料；本轮未修改正式 Spec/FCU/registry。 |

## B02 Production ABI FAIL

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-05（ABI 面） |
| 状态 | **PASS（2026-11-15，T17 生产 ABI 144 types / lock PASS）**；历史：BLOCKED / FAIL |
| 根因 | `abi_lock.h` 的 message registry hash 钉住 CRLF；LF blob 不同。布局比较 143 类型、981 偏移、0 mismatch。 |
| 证据 | `python tests/abi/abi_report.py` exit 1；`build/m4_m5_implementation/evidence/production_abi.log`。 |
| 安全影响 | ABI 锁失效会掩盖真实接口变化；不得用改锁掩盖结构差异。 |
| 解除条件 | ABI reviewer APPROVED + 真实 PASS + 原始日志。 |
| 下一步 | 提交 OWNER/ABI 审批包；本轮未修改 ABI lock。 |

## B03 RT-Thread Package Lock FAIL

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-06 |
| 状态 | **PASS（2026-11-15，T14 `--installed` problems 为空）**；历史：BLOCKED / FAIL |
| 根因 | CMSIS_CORE、STM32H7_CMSIS_DRIVER、STM32H7_HAL_DRIVER 仍为 `latest`，无 40-hex commit。 |
| 证据 | `python tools/check_rt_packages_lock.py` exit 1；`build/m4_m5_implementation/evidence/packages_lock.log`。 |
| 安全影响 | 固件依赖不可复现，阻断链接证据。 |
| 解除条件 | OWNER 批准版本/commit/许可证；真实 `pkgs --update`；锁文件 RESOLVED；安装日志与文件树 SHA。 |
| 下一步 | 准备三包版本/许可证/兼容性审批材料；不伪造 `packages/SConscript`。 |

## B04 Firmware Link BLOCKED

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-03 |
| 状态 | **PASS（2026-11-15，T18 默认 Release 链接 exit 0）**；历史：BLOCKED |
| 根因 | 依赖 B03；`packages/SConscript` 不存在，`packages/` 是 gitignored。 |
| 证据 | `python tests/guidance/check_target_build.py` exit 2；`build/m4_m5_implementation/evidence/scons_link.log`。 |
| 安全影响 | 无链接产物，不能声称固件可构建/可放行。 |
| 解除条件 | B03 RESOLVED + 真实 package tree + link check exit 0。 |
| 下一步 | 等 B03；保持对象编译证据，不冒充链接。 |

## B05 MR-009 审核记录

2026-11-14 项目方授权本地 Agent 对 MR-009 做一次技术复核，并免除该项独立审查。复核发现的同轴误差合成问题已修正，证据为 S06-116 和 spec/09 当日记录。S06-100～102 已完成 Host/ASan 复测。下表记录原 R31 时点。

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-04 |
| 状态 | **OWNER_WAIVED_INDEPENDENT_REVIEW / TECHNICAL_REVIEW_COMPLETED_WITH_CORRECTION / GitHub #8 CLOSED**；项目方 2026-11-14 明确豁免该项独立审查，复核发现已修正；不声称第三方独立 APPROVED。 |
| 已实现 | R30.3 rounded Horner product/sum recurrence、RN overflow midpoint fail-closed、gamma 定向舍入、正规/次正规误差界、系数构造/速度缩放/加速度缩放首个非有限值短路；R31 修复 velocity/acceleration time-shift derivative 使用实际 rounded vf/af 系数。 |
| 已测试 | Host #43446380866 6/6、M6/M7 #43446380836 5/5、M7 #43446380724 2/2；M5 Host/ASan、MR-009 focused、S06-099 60,002 offsets 双模型 Fraction、S06-107 rounded-derivative oracle、18 mutation FP audit、ARM 两对象无 FMA、Cortex-M7 模块编译均 PASS。 |
| 尚未声称 | degree 5/4/3 Horner 拒绝点完整 validator 输入可达性不作声称；H723 FPSCR 实机等价仍 NOT_RUN；M6 40 ms route preview 不在 M5-06 20 ms 证明域。 |
| 证据 | Proof：spec/history/implementation/guidance/M5_06_6_MR009_PROOF.md；Matrix：spec/quality/owner_approval/R30/05_MR009_INDEPENDENT_REVIEW_MATRIX.md；CI #43446380866 / #43446380836 / #43446380724。 |
| 解除条件 | MR-009 的独立审查豁免仅覆盖该 Issue；S06-100..102、B06/B07 等其余 M5-06 发布门禁仍须分别完成。 |
| 下一步 | 保留 R30/R31 proof、矩阵、代码差异与 CI 证据；按 B06/B07 等独立门禁继续验收，不推进 M5-07。 |

## B06 Validator 栈预算

2026-11-14 使用两份 Cortex-M7 专用镜像重建并完成调用图复核。Release `-O2` 为 **PASS / COMPLETE**，3,576 / 4,096 B；非优化 `-O0` 为 **PASS / COMPLETE**，3,968 / 4,096 B，余量 128 B。FLIGHT-LOCAL-22/24 的历史超限结论已由当前静态链修正；部署相关调用者、中断上下文和板上栈水位仍由 B07 单独验证。

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-22 / FLIGHT-LOCAL-24 |
| 状态 | **PASS / COMPLETE（静态专用镜像）**；`-O2` 3,576 B，`-O0` 3,968 B，均小于 4,096 B；`-O0` 余量 128 B。板上调用者/异常/中断余量和运行时水位仍 NOT_RUN。 |
| 历史链 | validate 1,112 + build_curve 2,824 + interval_scale 112 + interval_mul 88 + interval_bounds 64 + interval_invalid 48 = 4,248 B。 |
| 第二/动态/静态 | 3,288 B / 1,944 B / 1,408 B（R16 链）；本轮未把它们相加。 |
| 调用图 | 2026-11-14 两份专用镜像调用图均 COMPLETE；所有可达调用边纳入静态最大链，动态/间接边缺失不得 PASS。 |
| 未计入 | 部署调用者、异常/中断上下文、目标运行时栈水位；这些由 B07 目标板实测，不混入专用静态镜像链。 |
| 静态余量 | 当前 Release map 为 48,888 / 49,152 B，余量 **264 B**，低于 1 KiB 警戒线；继续监控，不扩大 48 KiB 上限。 |
| 方案 A | 单轴 helper 拆分：降低 build_curve 帧，但增加调用深度与 WCET；需真实 .su/objdump 重算。 |
| 方案 B | 区间数组复用：降低帧但增加生命周期/重入风险，需 ABI 与并发评审。 |
| 方案 C | 固定 workspace：把临时数组移出栈，增加静态 RAM；与 264 B 余量冲突，必须 OWNER 批准。 |
| 结论 | 三方案均未实施；当前静态专用镜像链满足 4,096 B 上限，但 B07 的目标调用者/异常栈/实测水位仍未通过前，不得宣称目标栈验收完成。 |
| 解除条件 | 真实调用链 ≤ 4,096 B 且保留调用者/中断余量；独立复核 `.su`/反汇编。 |

## B07 H723 实机验证 NOT_RUN

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-25 |
| 状态 | **BLOCKED / NOT_RUN** |
| 缺少 | H723ZGT6 目标板、ST-LINK/J-Link、DWT/ETM 或 GPIO 打点、HIL 台架、飞行许可。 |
| 未执行 | 栈水位、WCET/RTA、目标资源、HIL 故障注入、实机避障。 |
| 安全影响 | 不得把 Host/SIL 或 Cortex-M7 编译描述为 H723 实机通过。 |
| 解除条件 | 真实硬件和测试许可；原始 trace/水位/资源证据。 |
| 下一步 | 准备测试计划、接线和安全审查；不进入 M5-07。 |

## 派生规则

- 状态变化先改 `ISSUE_REGISTER.md`，再更新本视图。
- `FIXED_WAITING_RETEST` 不等于 VERIFIED；除项目方明确豁免的 MR-009/#8 外，安全/数学问题的 `VERIFIED/CLOSED` 必须由适用的独立审查者依据证据确认；#8 的关闭不构成第三方独立 APPROVED。
- S06-100..102 的 Host 证据已更新。MR-009 的单次技术复核完成并修复发现项。M5-06 仍受 B07 门禁阻塞。
- R27 文档、R27 快照和 R22 原始字节保留为 HISTORICAL/NOT_RECOVERABLE，不得覆盖。


## ENG-07 原生 FMT-SIM 动态复测交接（Owner 本机）

| Issue | 场景 | 当前状态 | 执行责任 / 解除证据 |
|---|---|---|---|
| [#17](https://github.com/hujunjie-hub/Flight/issues/17) | Land | OPEN / R39 48,801/3；仅终端链 | 无 failsafe、落地、近地 0.300 m/s；LANDED 受 #26 阻塞 |
| [#18](https://github.com/hujunjie-hub/Flight/issues/18) | Return | OPEN / R39 48,802/3；仅终端链 | touchdown 0.143 m；#25 的 12.2 s 失败已消失 |
| [#19](https://github.com/hujunjie-hub/Flight/issues/19) | Mission | OPEN / R39 67,112/5 | touchdown 0.315 m、航点 0.292/0.413 m 通过；近地 0.496 m/s 与结束 `contact=NO_CONTACT` 受 #26 阻塞 |
| [#21](https://github.com/hujunjie-hub/Flight/issues/21) | Mission near-ground speed | OPEN / R39 LAND+RETURN PASS | Land 0.300、Return 0.293 m/s ≤ 0.40；Mission 本轮 0.496 需复测 |
| [#22](https://github.com/hujunjie-hub/Flight/issues/22) | M4 advisory contact handoff | OPEN / INDEPENDENT REVIEW PENDING | 证据持续到达 M5；终端链受 #26 阻塞 |
| [#23](https://github.com/hujunjie-hub/Flight/issues/23) | M5 contact-hold / LANDED | OPEN / 拒绝路径已修 | 1550/JERK 拒绝与 `FAILSAFE_TERMINAL` 不再出现；LANDED 受 #26 阻塞 |
| [#24](https://github.com/hujunjie-hub/Flight/issues/24) | Landing descends before horizontal settled | OPEN / 部分有效 | 接地水平速度改善；LANDED 受 #26 阻塞 |
| [#25](https://github.com/hujunjie-hub/Flight/issues/25) | RETURN handoff re-plan regression | OPEN / 修复已验证 | R39 无 handoff 拒绝；等待整批与独立复核 |
| [#26](https://github.com/hujunjie-hub/Flight/issues/26) | M6 近地水平极限环 | NEW / S1 | ~0.2 m/s、~1.4 s 周期，导航与真值一致；阻断 LANDED 且需 M6/MPC 复核 |
| [#29](https://github.com/hujunjie-hub/Flight/issues/29) | 导航估计与真值发散 | NEW / S1 | Mission 延长视界下分离 10.441 m、`return_error` 16.927 m、`pos_std` 0.062→0.81、导航加速度 -80.7/-137.6 m/s²；SIL 已有 `ESTIMATOR_DIVERGENCE` 断言与 M6 fail-closed 门控，估计器侧修复仍未做 |

CI on exact HEAD `547f37883186c888d786c4293f2a4341d348b994`: Host validation, M6/M7 host validation and M7 host validation all SUCCESS. M5 Host and ASan/UBSan 330,302/0 each; M4 969/0; M4/M5 integration 1,042/0; Cortex-M7 module compile PASS. These workflows do not launch native FMT-SIM. #17/#18/#19/#21/#23/#24/#25 remain OPEN pending #26 and independent review; #22 remains OPEN pending independent safety review.


R30 archive SHA256 `eb52b0d6430662cc0a88869570dcf23c00c964add676d2ef31870c4ae9909063` is verified, but its tested HEAD `3299a4f` is 181 commits behind the reviewed current branch HEAD `9063f310`; worktree DIRTY and formal manifest/loopback gate evidence absent. Five diagnostic scenarios failed (29 assertions total). Do not close or repeat code fixes from this old result; wait for a clean current-HEAD formal rerun. [Full archive review](../validation/FMT_SIM_R30_20261114_ARCHIVE_REVIEW.md).

## 2026-11-14 main 整合与 ENG-07 R31 最新状态

- PR #20 已合并至唯一分支 `main`，merge commit `eacb966b502b7f9cb547617e866641504346a5fd`；整合源 HEAD `bcbe4db5e665e5f678069afc1b206a00ae1cd36d` 的 13 项 CI checks 全部 SUCCESS；main merge commit Host validation #43566259362 为 6/6 SUCCESS。 Main HEAD `d7666c5aa093dbef36fef11b82909e91fdc61491` 的三组工作流全部 SUCCESS：Host [#43567633122](https://github.com/hujunjie-hub/Flight/actions/runs/43567633122) 6/6、M6/M7 [#43567633103](https://github.com/hujunjie-hub/Flight/actions/runs/43567633103) 5/5、M7 [#43567633072](https://github.com/hujunjie-hub/Flight/actions/runs/43567633072) 2/2；Host PowerShell runner parse step PASS。
- Owner 的 R31 本机 FMT-SIM 归档测试 HEAD 为 `304fbd12078679d46b80cb626aba9eb0727b6914`，比整合源 HEAD 落后 7 commits。Land 4、Return 3、Mission 6 failures；总计 116,893 checks / 13 failures。该结果仅绑定旧 HEAD，不能判定当前 main 的原生仿真结果。
- #17/#18/#19/#21/#22 继续 OPEN，


## 2026-11-14 ENG-07 R32 归档与修复

- R32 RUN_ID `FMT-SIM-LOCAL-20261114T072713056Z-e470f00c` 在 main HEAD `e470f00cd539f438b1923b0faa844456ba2d3b59` 运行；正式门禁、worktree、upstream 和归档哈希均有效。
- Land 48,043/0 PASS；Return 44,667/0 PASS；Mission 62,793/3 FAIL；总计 155,503/3。Mission 失败为近地下降速度 0.665 m/s、DISARMED ACK reason 11、arm state 未 DISARMED。
- Fix commit `3d8d29b5dfda21237f8ed02db3c31223c656bfbe` 已增加 final 0.50 m / 0.40 m/s 限制并锁存 touchdown XY；FMS 安全阈值未放宽。Host/M6-M7/M7 全部通过（6/6、5/5、2/2），新增 M5 测试 PASS。
- R32 的 #17/#18 单场景 PASS 不构成全批次 closure eligibility；#17/#18/#19 继续 OPEN，等待 fix commit 上完整原生三场景复测。B07 H723、WCET/RTA、栈水位、物理 HIL、飞行仍 NOT_RUN。
- 完整报告：[R32 FMT-SIM archive review](../validation/FMT_SIM_LOCAL_20261114_E470F00C_REVIEW.md)。


## 2026-11-14 R33 原生 FMT-SIM 复测

R33 RUN_ID `FMT-SIM-LOCAL-20261114T084723472Z-5e3b74f4` 在修复前 HEAD `5e3b74f40293a005de693eceb61af4c055f1efdb` 完成：Land 39,388/0 PASS，Return 40,485/0 PASS，Mission 67,112/3 FAIL；总计 146,985 checks / 3 failures，最长状态间隔 94.8 ms。ZIP SHA256 `0d6b75c578e2ae71b501f9f26b827559fe8b0887598253af55d09d09f63cdcfb` 与 sidecar 一致，manifest SHA256 `6604b49d3fe889c9c022b63dec313b762880d7aa8cc12e736975257e2490971d`，内部 102 项 SHA256 全部通过。Mission 最后 0.50 m 最大下降速度 0.374 m/s（上限 0.40，已通过），返航落点距 Home 0.171 m；仅 terminal ACK、DISARM ACK、最终 DISARMED 状态三项失败。

R33 在 t=34.280 s 首次记录 ground contact=CONTACT，但 FMS landed=UNKNOWN 且 evidence=0x0；当时水平速度约 0.291 m/s，未满足 M4 的 0.20 m/s 稳定门槛。旧代码在 UNKNOWN 状态清空接地证据，M5 无法提前停止追逐 Home/probe。最终快照 contact=1、landed=UNKNOWN、stable_ms=0、evidence=0x0，nav_vh=0.161 m/s、nav_vd=-0.005 m/s、nav_ad=0.688 m/s²；垂向加速度仍超过 0.5 m/s² 稳定门槛。

R33 后 main 新增生产修复：`419f56962f74fd9d89bc62cfc11ee0923910177c` 在导航/能力/速度数据有效且物理接触可信时保留 UNKNOWN 接地候选证据；M5 识别该提示后锁存实测接触 XYZ 并停止继续追逐 Home。该候选只用于 M5 过渡，不授予 LANDED/DISARM 权限。`15a33756d4e269800e6778c6180f7086179c9429` 增加 nav_ok 检查，避免导航指针缺失时被候选分支解引用。M4 的 0.20 m/s 水平速度、0.15 m/s 垂向速度、0.5 m/s² 垂向加速度与 2 s dwell、DISARM gate 均未放宽。测试提交 `3270cff5aec26e3ae0717c62d2d78e9f30752020` 增加缺失导航回归；两个 contact candidate 测试均通过。

Current-main CI on `3270cff5aec26e3ae0717c62d2d78e9f30752020`: Host [#43593701736](https://github.com/hujunjie-hub/Flight/actions/runs/43593701736) 6/6, M6/M7 [#43593701892](https://github.com/hujunjie-hub/Flight/actions/runs/43593701892) 5/5, M7 [#43593701803](https://github.com/hujunjie-hub/Flight/actions/runs/43593701803) 2/2 SUCCESS. M4 969/0; M5 327,030/0; M4/M5 integration 1,042/0; MR-009 focused 430/0; M7 274/0.

完整报告：[R33 archive review](../validation/FMT_SIM_LOCAL_20261114_5E3B74F4_REVIEW.md)。