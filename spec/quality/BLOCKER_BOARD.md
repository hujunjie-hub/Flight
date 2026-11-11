# Flight M5-06 阻塞项视图（R23）

本文件只做 Issue Register 的**派生视图**，不维护第二套状态。数据源：
`spec/quality/ISSUE_REGISTER.md`。状态、严重度、责任方和复验结论以该登记为准。
`GITHUB_PUBLICATION = PENDING`：Agent 无 GitHub 写权限，不虚构 Issue 编号或在线状态。

输入基线：R22 代码 `9e4f5eff5b7140815c09e0151e7f43ced128772f`；
R22 归档 HEAD `8fcd4eacda93bf953882f5bb079e891839a1f3da`。
R23 validated code `e89cb7d94e6c992c8dfa71c3cb565ae4c5f6248a`；
Remote HEAD `51e7e923419733f5aa3b21efa1e46588629623ed`；本地领先 68 commits；
RUN_ID `M5-06-R23-20261111`。
所有命令均从仓库根目录执行；Host/SIL 结果不构成 H723 实机、HIL 或飞行放行。

---

## B01 Spec Audit FAIL

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-05（关联 draft 05） |
| 严重度/类型/模块 | S2 / docs-build / platform-spec |
| 首次发现 / 当前 SHA | R4 / `fa1de8f`（R16 代码） |
| 预期 | `audit_spec.py --round 3` 退出 0，`dialect_report.json` 的 `spec_xml_sha256` 等于 `FCU.xml` 的 LF 内容 SHA256 |
| 实际 | 退出 1：`SPEC_AUDIT_FAILED ... spec_xml_sha256 does not match` |
| 根因 | 已提交证据钉住的是 CRLF 变体 `1be7adc3…446ed`；Git blob/工作区是 LF `17d6e980…4321`。内容文本未变，属于行尾表示差异 |
| 复现 | `python spec/tools/audit_spec.py --round 3 --label flight-local`；exit 1；日志 `build/m4_m5_implementation/evidence/spec_layout.log`；SHA256 `5206b6d1…08291` |
| 安全影响 | 不直接影响轨迹计算；但 spec 证据不可复现会掩盖真实规范变化，阻断“spec 可复现”验收 |
| fail-closed | 是；审计失败不会被改写为 PASS |
| 阻断 | B01、Spec/Acceptance 证据门禁；不阻断 M5-06 Host/SIL 算法测试 |
| 责任方 | OWNER（批准重基线）+ AGENT（执行） |
| 状态 | **BLOCKED**；未修改 `FCU.xml`、`message_registry.json` 或 `dialect_report.json` |
| 修复步骤 | 1) 复核 LF/CRLF 差异报告；2) 批准以 LF 哈希重钉 `dialect_report.json`；3) 重新生成证据；4) 在 CI 加审核门禁 |
| 需审批 | 允许更新证据哈希；如果规范文本有真实变化，需单独 CR |
| 解除条件 | OWNER 批准 LF 重基线 + 真实 `audit_spec.py` PASS 日志 |
| 复测/通过 | `python spec/tools/audit_spec.py --round 3 --label flight-local` exit 0；日志 SHA256；独立复核 |
| 下一步 | 把 `build/validation/rebaseline_proposal/` 提交 OWNER；批准前保持 FAIL |

---

## B02 Production ABI FAIL

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-05（与 B01 同根因，ABI 面） |
| 严重度/类型/模块 | S2 / ABI-lock / contracts |
| 首次发现 / 当前 SHA | R4 / `fa1de8f` |
| 预期 | `abi_report.py` exit 0，`abi_lock.h` 的 message registry hash 与 LF blob 一致 |
| 实际 | exit 1，`lock: FAIL`；`types=143`、`spec_mismatches=0`、`unexpected_host_arm=0`；ARM hash `12b64f63…66c4`、Host `0edc9d8b…eacf` |
| 根因 | `ABI_LOCK_MESSAGE_REGISTRY_SHA256` 钉住 CRLF `e2893aeb…bf41`；LF blob 是 `dd06554c…49e7`。布局比较 143 类型/981 偏移 0 mismatch，无结构漂移 |
| 复现 | `python tests/abi/abi_report.py`；exit 1；日志 `build/m4_m5_implementation/evidence/production_abi.log`；SHA256 `364d820d…01204` |
| 安全影响 | 不改变结构布局；但锁失效会掩盖真实 ABI 变更 |
| fail-closed | 是 |
| 阻断 | Production ABI 门禁；不阻断 Host/SIL 单元测试 |
| 责任方 | INDEPENDENT_REVIEWER（ABI lock 审批）+ OWNER |
| 状态 | **BLOCKED**；未修改 `abi_lock.h` |
| 修复步骤 | 使用 `build/validation/rebaseline_proposal/abi_lock.diff`；仅替换 message registry hash；保留布局哈希；走 ABI 评审 |
| 需审批 | ABI lock 变更批准；确认无结构差异 |
| 解除条件 | ABI reviewer APPROVED + `abi_report.py` PASS + 原始日志 |
| 复测/通过 | `python tests/abi/abi_report.py` exit 0；143/981/0；日志 SHA256 |
| 下一步 | 提交 B01/B02 联合审批包；不得用改锁掩盖布局差异 |

---

## B03 RT-Thread Package Lock FAIL

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-06（draft 06） |
| 严重度/类型/模块 | S2 / build-dependency / platform |
| 首次发现 / 当前 SHA | R4 / `fa1de8f` |
| 预期 | `tools/rt_packages.lock.json` `status=RESOLVED`，三个包有 40-hex commit，`.config` 不再 latest |
| 实际 | `status=UNRESOLVED`；CMSIS_CORE、STM32H7_CMSIS_DRIVER、STM32H7_HAL_DRIVER 均 `latest`；校验 10 条 FAIL |
| 根因 | 依赖版本未获批准；`latest` 不可复现；本地无 RT-Thread env 工具 |
| 候选（已核实上游 tag/commit） | CMSIS_CORE `5.9.0` = `61e36449f53c25ef7825c40f7dd93685736f457f`；STM32H7_CMSIS_DRIVER `v1.10.7` = `05ea8164ca19102bb5c9eebf0c45f7ac2cd1e35c`；STM32H7_HAL_DRIVER `v1.11.6` = `33ca46547d40b281a98b5ca497efc89dc628a4a7` |
| 复现 | `python tools/check_rt_packages_lock.py`；exit 1；日志 `build/m4_m5_implementation/evidence/packages_lock.log`；SHA256 `3350ddf8…39c6` |
| 安全影响 | 不直接改变飞控算法；不可复现固件依赖，阻断链接证据 |
| fail-closed | 是 |
| 阻断 | B03、B04、Firmware Link、目标资源检查 |
| 责任方 | OWNER（版本/许可证批准）+ AGENT（安装验证）+ EXTERNAL_DEPENDENCY（env 工具/网络） |
| 状态 | **BLOCKED** |
| 修复步骤 | 1) 批准候选或给替代版本；2) 更新 `.config` 关闭 latest；3) 填锁文件并记录审批人/时间；4) `pkgs --update`；5) 记录实际 commit/文件树 SHA256；6) 跑锁校验 |
| 需审批 | 三个包版本 + 许可证 + 依赖基线变更 |
| 解除条件 | 版本批准 + 真实安装成功 + 锁文件 RESOLVED + 校验 PASS |
| 复测/通过 | `python tools/check_rt_packages_lock.py` exit 0；`.config`/lock 一致；安装日志和文件树哈希 |
| 下一步 | OWNER 选择候选；未批准前保持 FAIL，不填虚假 RESOLVED |

---

## B04 Firmware Link BLOCKED

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-03（draft 03） |
| 严重度/类型/模块 | S2 / build / firmware-link |
| 当前 SHA | `7554b41` |
| 预期 | `check_target_build.py` exit 0，生成并检查 ELF/BIN/MAP |
| 实际 | exit 2 `BLOCKED`：`packages/SConscript` 不存在；`packages/` 是 gitignored，且从未进入 Git 历史 |
| 根因 | 依赖 B03 的 `pkgs --update`；本地没有 RT-Thread env/packages 树 |
| 复现 | `python tests/guidance/check_target_build.py`；exit 2；日志 `build/m4_m5_implementation/evidence/scons_link.log`；SHA256 `33071b99…40dc3` |
| 安全影响 | 无链接证据，不能声称固件可构建/可放行 |
| fail-closed | 是；exit 2 不记 PASS |
| 阻断 | Firmware Link、目标链接资源、HIL/实机前置 |
| 责任方 | EXTERNAL_DEPENDENCY（env/packages）+ OWNER（B03 批准） |
| 状态 | **BLOCKED**，依赖 B03 |
| 修复步骤 | 1) 完成 B03；2) 运行对应 `pkgs --update`；3) 确认 `packages/SConscript`；4) 运行 link check；5) 保存 ELF/BIN/MAP/链接日志/资源报告 |
| 需审批 | 不允许创建假 `packages/SConscript` 或跳过依赖 |
| 解除条件 | B03 RESOLVED + `packages/SConscript` 真实存在 + link check exit 0 |
| 复测/通过 | `python tests/guidance/check_target_build.py` exit 0；产物哈希与 resource report |
| 下一步 | 等 B03；先保持对象编译证据，不冒充链接 |

---

## B05 MR-009 数学审查 OPEN

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-04（draft 04） |
| 严重度/类型/模块 | S2 / safety-math / M5-06 |
| 当前 SHA | `fa1de8f`；R16 证明文件 `M5_06_7_JERK_INTERVAL_PROOF.md` |
| 预期 | 独立审查逐项 APPROVED；MR-009 关闭 |
| 实际 | `FIX_IMPLEMENTED_WAITING_INDEPENDENT_REVIEW`；未 APPROVED |
| 审查项 | Bernstein jerk `60/d³`；Bernstein→power `p0,p1,p2`；原区间/尾段 `u_tail=1+γ₂·ratio`；consumer binary32 `6c3+24c4s+60c5s²`；Fraction 端点/驻点；S06-108/112/113；FP 异常与 fail-closed |
| 复现 | `python tests/guidance/run_host_tests.sh`；S06-112 candidate=160/legal=140/fail_closed=20/axis=420，bernstein/tail/consumer/total 各 420 checks，旧公式 whole-tail 420/122 欠界；S06-113 76 checks、actual error 3.618–4.059γ₂；日志 `build/m4_m5_implementation/evidence/m5_host.log` |
| 安全影响 | 证明链未独立复验，不能宣称消费曲线净空已证明 |
| fail-closed | 是；S06-100..102 保持 NOT_RUN |
| 阻断 | M5-06 验收、M5-07 |
| 责任方 | INDEPENDENT_REVIEWER |
| 状态 | **OPEN / BLOCKED** |
| 修复步骤 | 审查者逐项给出 APPROVED/REJECTED；补充缺口证据；不得 Agent 自批 |
| 需审批 | 独立数学审查签字 |
| 解除条件 | 全部审查项 APPROVED + S06-100..102 状态明确 + M5-06 验收 |
| 复测/通过 | 独立复跑 S06-108/112/113 + 审查记录 |
| 下一步 | 提交证明文件、`analysis_p2`、S06 日志给审查者 |

---

## B06 Validator 栈预算 BLOCKED（R16 漏算修正）

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-22（栈超限）+ FLIGHT-LOCAL-24（R16 漏算） |
| 严重度/类型/模块 | S1 / resource / M5-06 |
| 当前 SHA | `7554b41`；`.su` `build/m5_arm/m5_trajectory_safety.su` SHA256 `20280ada…f57b` |
| 预期 | 最坏真实嵌套链 ≤ 4,096 B，且保留调用者/中断余量 |
| 实际 | **4,248 B**：validate 1,112 + build_curve 2,824 + interval_scale 112 (SP) + interval_mul 88 (SP) + interval_bounds 64 + interval_invalid 48 |
| 根因 | `interval_mul` 调用 `interval_bounds`，`interval_bounds` 错误分支调用 `interval_invalid`；R16 先选了较短的 consumer_eval_error_bound 路径（4,128 B，子调用 192 B）；R17 的真实路径子调用 296 B，差 104 B；R18 再发现 interval_scale/interval_mul 的 `.su` 各少 8 B，最终 4,248 B |
| 复现 | `arm-none-eabi-objdump -d build/m5_arm/m5_trajectory_safety.o`；`.su`；`build/validation/analysis_p2.md` |
| 第二/动态/静态 | 3,288 B / 1,944 B / 1,408 B；DFS 工作栈 464 B |
| 未计入 | 调用者、异常/中断上下文、libm `fmax/fmin/sqrt`、编译器尾调用变化、target runtime |
| 安全影响 | 不能进入 4,096 B Planner Worker；强行进入会栈溢出 |
| fail-closed | 是；产品化阻断 |
| 阻断 | M5-06 产品化、H723 Planner Worker |
| 责任方 | AGENT（设计/验证）+ OWNER（预算/架构批准） |
| 状态 | **OPEN / BLOCKED** |
| 减少栈的设计方案 | 1) 将 `M5SafetyBezier`/`curve_bounds` 的区间临时数组移入调用方固定 workspace，拆成两阶段，避免 build_curve 大帧；2) 把 `build_curve` 的单轴区间系数计算提取为独立函数，限制每层帧；3) 用静态断言锁定新上限并重跑 `.su`/objdump；风险：workspace 增长、重入/并发、可读性、与固定 512 B DFS 预算冲突 |
| 需审批 | 任何栈预算/架构变更；不得降低安全缓冲或改 4,096 上限 |
| 解除条件 | 真实调用链 ≤ 4,096 B 且保留规定余量；独立复核 `.su`/反汇编 |
| 复测/通过 | 重新生成 `.su`、objdump 调用图、最坏链计算；Host/SIL 回归 |
| 下一步 | 先做设计评审，不直接改代码；保持 M5-06 Host/SIL |

---

## B07 H723 实机验证 NOT_RUN

| 字段 | 内容 |
|---|---|
| Issue | FLIGHT-LOCAL-25（新登记） |
| 严重度/类型/模块 | S0 / verification-gap / H723 target |
| 当前 SHA | `7554b41` |
| 预期 | 真实硬件栈水位、WCET/RTA、DWT trace、目标资源、HIL、故障注入、实机避障证据 |
| 实际 | 全部 NOT_RUN；无硬件/调试器/HIL 台架 |
| 子项 | 栈水位（最高 watermark）；WCET/RTA（最坏执行时间/响应时间）；DWT trace；链接后 RAM/Flash/stack；HIL 故障注入；实机避障 |
| 所需资源 | H723ZGT6 目标板、ST-LINK/J-Link、DWT/ETM 或 GPIO 打点、HIL 台架、螺旋桨安全防护、测试场地/许可 |
| 执行流程 | 链接产物 → 烧录 → 启动/悬停/轨迹 → DWT/GPIO 采样 → 栈 watermark → 故障注入 → 记录原始 trace |
| 验收标准 | 栈峰值 < 预算且留规定余量；WCET ≤ 周期；无 deadline miss；HIL 故障均 fail-closed；实机避障需单独授权 |
| 未执行原因 | 无硬件、无 HIL、无飞行许可；不得伪造 |
| 责任方 | HARDWARE / OWNER |
| 状态 | **BLOCKED / NOT_RUN** |
| 解除条件 | 提供硬件与测试许可；完成上述原始证据 |
| 下一步 | 准备测试计划与接线/安全审查；不进入 M5-07 |

---


## R18 资源追加项

### FLIGHT-LOCAL-26 roll_map_to_position 32,840 B

- 状态：FIXED_WAITING_RETEST（R18/R19 代码，等待独立复验）。
- 修复：逐轴原地滚动（x→y→z），不再复制 4096-cell 栈数组；`roll_map_to_position`
  40 B + `roll_axis_inplace` 64 B + `map_cell_at` 24 B，峰值 **128 B**（原 32,840 B）。
- R18 报告 104 B 漏算 `map_cell_at`；R19 反汇编确认调用，登记 FLIGHT-LOCAL-29。
- 差分测试：`m5_map_roll_inplace_differential` 覆盖边界 shift，`m5_map_roll_inplace_random`
  固定 seed 0x4D415052、1000 cases、0 failed，比较 4096 cells/center/revision；Host PASS。
- 仍为 Host/SIL-only；不得据此宣称 H723 Planner 已产品化。

### FLIGHT-LOCAL-27 ICD+M4 余量 232 B

- 状态：OPEN / RISK（S2）。
- 当前 48 KiB 上限，使用 48,920 B，余量 232 B；`check_memory_budget.py` 输出 WARN。
- 任何 ICD/FMS/M4 静态布局变更必须重跑预算；不得擅自扩大 48 KiB 上限。
- 解除条件：预算复核/审批；当前不阻断 Host/SIL 算法测试，但属未来产品化风险。

### FLIGHT-LOCAL-28 S06-112 拒绝分类

- S1；R19 代码提交 `8b112f0279735f4f9977f4314b81dccf203b14c3`。
- 正常样本必须计算，只有 subnormal 预期拒绝（status=2/3 NUMERIC）；意外拒绝计 FAIL，
  不计 fail_closed_cases。

### FLIGHT-LOCAL-29 地图栈漏算

- S2；R18 报 104 B，R19 修正为 128 B（+map_cell_at 24 B）；已完成 .su/objdump 核对。

### FLIGHT-LOCAL-30 归档字段

- S2；R19/R20 archive script 生成外部 sidecar/report；正式 Manifest 不再用 PENDING 冒充完整值，
  ZIP 内不保存自身哈希。

### FLIGHT-LOCAL-31 S06-112 历史证据

- S2；R20 `S06_112_EVIDENCE_INDEX.md` 标记 R16/R18/R19 为 HISTORICAL/SUPERSEDED；
  只有 `s06_112_r20.json` 是当前结果，旧文件 SHA256 保留。

### FLIGHT-LOCAL-32 基线元数据

- S2；R20 统一 BLOCKER_BOARD/Manifest/资源摘要到 R19 代码、归档 HEAD 和 50 commits。

### FLIGHT-LOCAL-33 S06-112 严格断言

- S1；R21 0642bda；160/140/20(status2)/420、R14b 6/60 underbound；0 failures。
- 旧证据 HISTORICAL，R21 current。

### FLIGHT-LOCAL-34 证据索引 SHA

- S2；R21 0642bda；索引绑定 code/library/run/toolchain，R20 错误 SHA 修正。

### FLIGHT-LOCAL-35 资源文档历史冲突

- S2；R21 0642bda；analysis_p2 文首 R21 摘要，旧 32,840 B 标 HISTORICAL。

### FLIGHT-LOCAL-36 归档元数据

- S2；R21/R22；archive report 增加 archive_commit_sha/input baseline/remote/ahead。

### FLIGHT-LOCAL-37 环境证据

- S2；R22 `r22_environment.json` 记录 Host/ARM/Windows 完整工具链；旧 `environment.txt` 标 HISTORICAL。

### FLIGHT-LOCAL-38 逐项测试证据

- S2；R22 `r22_test_results.json` 逐项 RUN_ID/code/command/checks/failures/exit/seed/toolchain/log SHA。

### FLIGHT-LOCAL-39 S06-112 JSON

- S1；R23 e89cb7d；JSON-only + 独立 log；R14b 6 条精确记录；JSON_OK。

### FLIGHT-LOCAL-40 结果收集器

- S1；R23 e89cb7d `tools/collect_validation_results.py`；9 负向自测；逐项字段完整。

### FLIGHT-LOCAL-41 R22 工作区

- S2；R23 `R23_WORKTREE_REVIEW.md`；ABI 失败证据保留不提交；旧 collector 标 HISTORICAL。

## 派生规则

- 本文件任何状态变化必须先改 `ISSUE_REGISTER.md`，再更新本视图。
- `FIXED_WAITING_RETEST` 不等于 VERIFIED；`VERIFIED` 只能由独立审查者给出。
- 已知失败日志保留在 `build/m4_m5_implementation/evidence/` 和 ZIP 中，不得删除。
- S06-100..102 继续 `NOT_RUN`；MR-009 未 APPROVED 前 M5-06 不得完成，M5-07 不得启动。
