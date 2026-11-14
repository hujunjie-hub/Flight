# Flight M5-06 阻塞项视图（R33）

> **2026-11-13 MR-009 R31 内部预审 CI 快照（代码/证明基线 8dcb60b3ac18e6a763c939d3e810984092cdd321）**：Host validation #43446380866（6/6）、M6/M7 #43446380836（5/5）、M7 #43446380724（2/2）全部 SUCCESS。M5 Host regression、M5 ASan/UBSan、MR-009 focused、S06-099/S06-107、Spec Audit、生产 ABI、18 mutation FP audit、Cortex-M7 模块编译、M6/M7 Host/Sanitizer/SIL 与 M7 Host/Sanitizer 均 PASS。

> **当前状态（2026-11-14）**：B01～B04 的本地规范/生产 ABI/安装包锁/默认 Release 链接通过。B05 MR-009 按项目方指示完成技术复核，S06-100～102 Host 与 ASan/UBSan 通过。B06 的 Cortex-M7 专用镜像静态链为 `-O2` 3,576/4,096 B、`-O0` 3,968/4,096 B，均 COMPLETE。B07 实机、WCET/RTA、栈水位、台架 HIL 与飞行仍 NOT_RUN。M5-07 未启动。

> **开放风险**：ICD+M4 余量 264 B（FLIGHT-13）；Release 的工具链枚举属性仍有 228 条链接警告（FLIGHT-14）。下方 B01～B06 的表格保留各原始验证时点。

本文件是 `spec/quality/ISSUE_REGISTER.md` 的**派生视图**，不维护第二套状态。
RUN_ID `M5-06-R31-20261113`；validated code `8dcb60b3ac18e6a763c939d3e810984092cdd321`；
code-validation HEAD `8dcb60b3ac18e6a763c939d3e810984092cdd321`；
ahead 动态计算；branch `opt/stage-02-m5-06-safety-validator`。

本文件结论来自历史本地验证记录与本轮 GitHub Actions 远程 Host/SIL/Cortex-M7 编译证据；均不是 H723 实机、HIL、
WCET/RTA、栈水位或飞行放行证据。`GITHUB_PUBLICATION = PARTIAL`；GitHub Issues #8–#19 已在线跟踪，历史本地编号仅按明确映射关联。

## B01 Spec Audit FAIL

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-05 |
| 状态 | **BLOCKED / FAIL** |
| 根因 | `dialect_report.json` 钉住 CRLF 变体哈希；Git/LF 工作区是另一哈希。文本未变，属于行尾表示差异。 |
| 证据 | `python spec/tools/audit_spec.py --round 3 --label m4-m5-implementation` exit 1；`build/m4_m5_implementation/evidence/spec_layout.log`。 |
| 安全影响 | 规范证据不可复现；不直接改变轨迹计算。 |
| 解除条件 | OWNER 批准 LF 重基线；真实审计 PASS；独立复核。 |
| 下一步 | 准备 OWNER 审批材料；本轮未修改正式 Spec/FCU/registry。 |

## B02 Production ABI FAIL

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-05（ABI 面） |
| 状态 | **BLOCKED / FAIL** |
| 根因 | `abi_lock.h` 的 message registry hash 钉住 CRLF；LF blob 不同。布局比较 143 类型、981 偏移、0 mismatch。 |
| 证据 | `python tests/abi/abi_report.py` exit 1；`build/m4_m5_implementation/evidence/production_abi.log`。 |
| 安全影响 | ABI 锁失效会掩盖真实接口变化；不得用改锁掩盖结构差异。 |
| 解除条件 | ABI reviewer APPROVED + 真实 PASS + 原始日志。 |
| 下一步 | 提交 OWNER/ABI 审批包；本轮未修改 ABI lock。 |

## B03 RT-Thread Package Lock FAIL

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-06 |
| 状态 | **BLOCKED / FAIL** |
| 根因 | CMSIS_CORE、STM32H7_CMSIS_DRIVER、STM32H7_HAL_DRIVER 仍为 `latest`，无 40-hex commit。 |
| 证据 | `python tools/check_rt_packages_lock.py` exit 1；`build/m4_m5_implementation/evidence/packages_lock.log`。 |
| 安全影响 | 固件依赖不可复现，阻断链接证据。 |
| 解除条件 | OWNER 批准版本/commit/许可证；真实 `pkgs --update`；锁文件 RESOLVED；安装日志与文件树 SHA。 |
| 下一步 | 准备三包版本/许可证/兼容性审批材料；不伪造 `packages/SConscript`。 |

## B04 Firmware Link BLOCKED

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-03 |
| 状态 | **BLOCKED** |
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
| 状态 | **OWNER_WAIVED_INDEPENDENT_REVIEW / TECHNICAL_REVIEW_COMPLETED_WITH_CORRECTION**；项目方 2026-11-14 明确豁免该项独立审查，复核发现已修正；不声称第三方独立 APPROVED。 |
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
- `FIXED_WAITING_RETEST` 不等于 VERIFIED；安全/数学问题的 `VERIFIED/CLOSED` 必须由独立审查者依据证据确认。内部预审不能替代独立复验。
- S06-100..102 的 Host 证据已更新。MR-009 的单次技术复核完成并修复发现项。M5-06 仍受 B07 门禁阻塞。
- R27 文档、R27 快照和 R22 原始字节保留为 HISTORICAL/NOT_RECOVERABLE，不得覆盖。


## ENG-07 原生 FMT-SIM 动态复测交接（Owner 本机）

| Issue | 场景 | 当前状态 | 执行责任 / 解除证据 |
|---|---|---|---|
| [#17](https://github.com/hujunjie-hub/Flight/issues/17) | Land | OPEN / FIX NOT YET DYNAMICALLY RETESTED | Owner 本机按 350 ms 运行；终端命令回执、连续 LANDED、DISARMED 和完整日志 |
| [#18](https://github.com/hujunjie-hub/Flight/issues/18) | Return | OPEN / FIX NOT YET DYNAMICALLY RETESTED | Owner 本机按 350 ms 运行；Return 完成、连续 LANDED、DISARMED 和完整日志 |
| [#19](https://github.com/hujunjie-hub/Flight/issues/19) | Mission | OPEN / FIX NOT YET DYNAMICALLY RETESTED | Owner 本机按 350 ms 运行；双航点完成、Return、终端回执、连续 LANDED、DISARMED 和完整日志 |

最新已完成自动化验证的代码 HEAD `5a8e2c3079d230bbb5ca90ebbf229552487c2be9` 的 Host validation #43552823302（6/6）、M6/M7 #43552823284（5/5）、M7 #43552823258（2/2）均 SUCCESS；M4 1,074/0、M5 324,792/0、MR-009 focused 430/0、M4/M5 integration 1,147/0、M5 ASan/UBSan 324,792/0；PowerShell Parser 对两个本地 runner 均 PASS。以上 CI 不启动原生 FMT-SIM。后续原生 FMT-SIM 均由项目 Owner 在 Windows 本机执行，远程 Agent 不代跑。Agent 负责代码修复、Host/SIL/CI、日志分析和分支交付。命令、环境指纹、归档和回传规则见 [FMT-SIM 本机复测交接方案](../validation/FMT_SIM_LOCAL_HANDOFF.md)；批量入口为 `tests/simulation/run_fmt_sim_regression.ps1`。严格关闭资格另要求 hover 0.62、显式 `-ConfirmLoopbackAddress`、UDP 14653 和 350 ms。三项 Issue 在本机证据回传并复核前保持 OPEN；不得用 Host/CI PASS 或 650 ms 诊断运行关闭。
