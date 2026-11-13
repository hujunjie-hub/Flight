# Flight M5-06 阻塞项视图（R29）

> **R30 更新（2026-11-12，commit `efc28d7`）**：B01、B02、B03 经 OWNER 批准完成重基线后为 **PASS**；B04 固件链接 **PASS**（`Flight.elf` text 458,604 / data 15,732 / bss 267,656）；B06 为 **PASS**（3,864 B <= 4,096 B，调用图 COMPLETE）。下方 B01–B06 段落保留为历史描述。

本文件是 `spec/quality/ISSUE_REGISTER.md` 的**派生视图**，不维护第二套状态。
RUN_ID `M5-06-R29-20261111`；validated code `fd8198d2ac216fef87dafba73c252bb254ea3dd3`；
remote HEAD `51e7e923419733f5aa3b21efa1e46588629623ed`；
ahead 动态计算；branch `opt/stage-02-m5-06-safety-validator`。

本文件所有结论均为本地 Host/SIL/Cortex-M7 编译证据，不是 H723 实机、HIL、
WCET/RTA、栈水位或飞行放行证据。`GITHUB_PUBLICATION = PENDING`。

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

## B05 MR-009 数学审查 WAITING

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-04 |
| 状态 | **WAITING_INDEPENDENT_REVIEW**；未 APPROVED，Agent 不自行批准。 |
| 已实现 | 逐级 Horner 上界 `H_n=|c_n|`、`H_k=|c_k|+s_max*H_(k+1)`；每级 product/sum 向外取整；isfinite/FLT_MAX 检查；Position/Velocity/Acceleration 及 vf/af 缩放路径。 |
| 已测试 | R28 新增 `m5_safety_mr009_horner_bound_extremes`、`m5_safety_mr009_extreme_reference`；S06-112 420/0；S06-108 96/0；S06-113 76/0。 |
| 尚未证明 | 独立审查签字；编译器未来重排/FMA 变化；M6 区间外插值；Host FTZ/DAZ 的完整目标等价性。 |
| 前置条件 | `s_max` 上界、系数有限、无 contraction/fast-math、round-to-nearest、产品包线内输入。 |
| 解除条件 | 独立数学审查逐项 APPROVED；S06-100..102 状态明确。 |
| 下一步 | 提交证明文件、R28 测试日志和 B06 分析给审查者。 |

## B06 Validator 栈预算 BLOCKED

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-22 / FLIGHT-LOCAL-24 |
| 状态 | **BLOCKED**；已知下界 **4,248 B > 4,096 B**，超出 152 B；R29 设计评审见 `spec/quality/owner_approval/R29/04_B06_STACK_REMEDIATION_DESIGN.md`。 |
| 当前链 | validate 1,112 + build_curve 2,824 + interval_scale 112 + interval_mul 88 + interval_bounds 64 + interval_invalid 48 = 4,248 B。 |
| 第二/动态/静态 | 3,288 B / 1,944 B / 1,408 B（R16 链）；本轮未把它们相加。 |
| 调用图 | `build/validation/regenerated/r29_stack_analysis.json`：状态 BLOCKED，callgraph_status UNKNOWN；libm `fmin/fmax/nextafter` 及动态/间接边显式记录，未知不得 PASS。 |
| 未计入 | 调用者、异常/中断上下文、libm 实际帧、尾调用变化、target runtime；4,248 B 仍是下界。 |
| 静态余量 | ICD+M4 为 48,920 / 49,152 B，余量 **232 B**；任何静态增长必须重算，不得扩大 48 KiB 上限。 |
| 方案 A | 单轴 helper 拆分：降低 build_curve 帧，但增加调用深度与 WCET；需真实 .su/objdump 重算。 |
| 方案 B | 区间数组复用：降低帧但增加生命周期/重入风险，需 ABI 与并发评审。 |
| 方案 C | 固定 workspace：把临时数组移出栈，增加静态 RAM；与 232 B 余量冲突，必须 OWNER 批准。 |
| 结论 | 三方案均未实施；未经 OWNER 审批不得做架构级 RAM 调整。M5-06 不得进入 4,096 B Planner Worker。 |
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
- `FIXED_WAITING_RETEST` 不等于 VERIFIED；`VERIFIED/CLOSED` 只能由独立审查者确认。
- S06-100..102 保持 NOT_RUN；MR-009 未 APPROVED 前 M5-06 不得完成，M5-07 不得启动。
- R27 文档、R27 快照和 R22 原始字节保留为 HISTORICAL/NOT_RECOVERABLE，不得覆盖。
