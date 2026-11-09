# T191-D-REVIEW — Agent B 独立复核意见（含作者自查说明）

| 项目 | 内容 |
|---|---|
| 复核人 | Agent B（SENSOR/NAV/FDIR） |
| 被复核任务 | `T191-D-REVIEW`（Owner D，`task_status=REVIEW`，`update_sequence=3`，acknowledgement 已记录） |
| 基线 | `V1.9.1-GLOBAL-AUDIT-R3` / Schema 6.0 / Product-Port `0x00010903` |
| 复核范围 | D 的复核记录与脚本、HANDOFF/09 现状、02 EstimatorPort、05（作者自查）、07 §6 预算、09 Gate C/REQ-NAV-KFG 行、10 角色边界；并与 A/C 已落盘交付交叉核对 |
| 复核时间 | 2026-11-04（Asia/Shanghai） |
| 机器证据 | `b_review_checks.json`、`b_review_checks.py`（本目录） |
| 结论 | **ACCEPT_WITH_CONDITIONS**（对 D-REV-01/03 无异议；D-REV-02 属 A 域、B 只确认接口影响；D-REV-04 由 B 本次补交 4 份状态请求关闭 B 部分；附 4 项关闭条件） |

**声明**：B 不是 `T191-D-REVIEW` 的执行者，未参与其实现或脚本编写。`05_…` 的维护 Owner 是 B，
因此 B 对 05 的核对属于**作者自查**，05 的独立复核仍须由 A 或 D 给出；B 对 02/07/00/09 及 A/C
交付物为非作者复核。本意见不代替 A/C 意见，也不构成 Gate A 会签或用户批准。

## 1. 复核方法

1. 通读 `spec/validation/T191-D-REVIEW/T191-D-REVIEW_report.md`（350 行）与 `review_checks.json` 摘要。
2. 用独立脚本（不复用 D 的 `review_checks.py`）核对：HANDOFF 记录合法性、D-REV-01..04 现状、
   09 Gate C/REQ-NAV-KFG 行、02 与 A 生产头的签名一致性、07 §6 预算算术、B 侧实测尺寸。
3. 交叉核对 A（`applications/contracts`、`applications/platform/rtthread53`）与 C（`applications/core/m4`）
   已落盘交付，只作一致性确认，不代替其验收。
4. 未执行：生产 Host/ARM ABI（A 另提交）、目标构建/WCET/HIL/飞行、dialect 复现（环境无 pymavlink）。

## 2. 独立检查结果

`b_review_checks.json`：10 项检查，PASS 9 / FAIL 1（B-02 在补交 4 份状态请求后转为 PASS；剩余 B-10 为任务归属缺口，见 §5 B-4）。

| 编号 | 检查 | 结果 | 证据 |
|---|---|---|---|
| B-01 | HANDOFF 记录合法；`T191-D-REVIEW=REVIEW/seq3` 且 ack 非空 | PASS | `spec/AGENT_HANDOFF.md` |
| B-02 | D-REV-04：B 记录仍 READY/ack=null | 原 FAIL，**本次补交后 PASS**（待 D 合并） | 各模块根 `status_request_T191-B-*.json` |
| B-03 | 09 Gate C 与 REQ-NAV-KFG-001…013 行存在且全为 `NOT_RUN`，与 B 交付口径一致 | PASS | 09 §2/§4.1 |
| B-04 | 02 §2.15 的 14 个 `estimator_*()` 与 A 的 `fc_port_api.h` 逐签名一致 | PASS | 02:2357–2370 / 头文件:58–71 |
| B-05 | 07 §6 预算可复算：88 KiB=8+54+20+6；checkpoint 2304 B；活动+shadow 28 KiB | PASS（余量仅 464 B，见 OBS-B-04） | `sizeof` 实测、D 报告 §8 |
| B-06 | 05 §7 `H_bg/H_sg` 记法未说明机体系 `skew(l)` 与矩阵次序 | OBS（B-R01） | 有限差分 4–10% 偏差复现 |
| B-07 | 00 §12.2 要求 Eigen 固定维数，但 Eigen 未锁定且 B 实现未使用 Eigen | OBS（B-R02，需 CR） | `packages/deps.lock`、`arm_objects.log` |
| B-08 | 32 位枚举 ABI 未写入构建要求，`arm-none-eabi-gcc` 默认 short-enums | OBS（B-R03） | 契约 `_Static_assert`；M2 组局部加旗标 |
| B-09 | 模块 Kconfig 聚合缺失：`FMS_QUAD_X4_ENABLE` 经 menuconfig 不可达 | OBS（B-R05） | `kconfiglib` 解析结果 |
| B-10 | `applications/calibration/` 不存在；GNSS 原生后端与 Mag 标定无任务归属 | OBS（B-R06/R07） | 目录探测、HANDOFF/09 对照 |

## 3. 对 D 四项发现的意见

### D-REV-01（11 号 Schema 5.0 残留）— 接受

- 不涉及 B 的接口/语义；修正确认（11:9 = Schema 6.0）与 B 交付使用的 6.0/`0x00010903` 一致。

### D-REV-02（07 号 pipeline 1900/3000 µs 冲突）— 接受，属 A 域；B 确认接口影响

- B 的 IMU 增量构造使用 950–1100 µs 的**采样间隔窗口**，与 `MAX_IMU_PIPELINE_DELAY_US` 不是同一门限，
  不因该值变化而改变；
- 但 B 的 `sensor_alignment.yaml` 中 `pipeline_delay_s` 必须≤该硬门限，否则 B-ADAPTER 的采样时刻修正会
  超出 05 §3.1 的时间预算。B 将在 A 修正后的正文（3000 µs）上增加一条上限校验与测试。
- 关闭条件见 §5 B-2。

### D-REV-03（证据链无法重建）— 接受

- `audit_spec.py` 的状态机检查与 `--label` 已具备；B 认可“工具修复 ≠ 新基线证据已重建”，
  与 A 的 C2 条件一致。B 不修改 `spec/validation/`。

### D-REV-04（HANDOFF 与工作区不一致）— **B 部分本次关闭**

- B 同时补交 4 份状态请求（均 `update_sequence=2`、带 acknowledgement、`blocked_by` 明确）：
  - `applications/plugins/estimator/kf_gins_quad/status_request_T191-B-UPSTREAM.json`
  - `applications/core/m2/status_request_T191-B-ADAPTER.json`
  - `applications/core/m3/status_request_T191-B-REPLAY.json`
  - `applications/services/health/status_request_T191-B-FDIR.json`
- 请求均不声称 `ACCEPTED/CLOSED`；若 D 严格按依赖门禁，可将后三项记为 `BLOCKED` 并保留 `resume_state`。
- A、C 已各自提交；B 提交完成后，D-REV-04 的 A/B/C 三者输入即齐备，是否关闭由 D 记录。

## 4. B 侧观察（除 §2 的 B-R01…R07 外）

| 编号 | 级别 | 内容 | 建议 |
|---|---|---|---|
| OBS-B-01 | P3 | 00:510 仍写“完整commit待锁定”，而 05:1133 已固定 `8291a93…` 且 B 已完成导入与哈希核对（与 D 的 OBS-03 同源） | 00 改写为“设计输入已锁定 / 导入包已锁源码 Hash”，D 处置 |
| OBS-B-02 | P3 | 05 §5.2 与 07 §6.4 要求 ring 只存 `base_sample_time_us`；B 初版多存 256×8 B 绝对时间，已在本次修正（`sizeof(InsHistory)` 73,760→71,720 B） | 无需改规范，建议 07 增一句“实现不得为每槽保留绝对时间” |
| OBS-B-03 | P2 | B 在 replay 等价性测试中发现并修复核心更新缺陷 F-01（三维更新后一维更新误用残留增益列，P 被放大 0.64），该缺陷会污染任何“GNSS 后接 Baro”的序列 | 建议 D 把该回归列入 Gate C 证据（`test_replay` 已覆盖） |
| OBS-B-04 | P2 | 活动+shadow 28,208 B 距 28 KiB 仅 464 B，任何 per-instance 元数据都会击穿 07 §6.4 | 07 增列每实例固定开销，或 05 §9 明确“含元数据” |

## 5. 关闭条件（供 D）

1. **B-1**：D 合并 B 的 4 份状态请求（dedup key `T191-B-*:2`），把 D-REV-04 的 B 部分标为已提交；
   B 不请求 `ACCEPTED/CLOSED`。
2. **B-2**：D/A 关闭 **B-R02**（Eigen 固定维数 vs B 自带固定维数层，需 CR）与 **B-R03**
   （`-fno-short-enums` 顶层构建规则）；在此之前 H723 目标不得宣称符合 05 §6.2。
3. **B-3**：D 确认后，B 作为 05 维护 Owner 提交 05 §7 记法澄清（B-R01）与 §7.1 的 pipeline 上限约束；
   这属于后续 patch，不在本次复核内直接修改。
4. **B-4**：D 为 B-R05（Kconfig 聚合）、B-R06（`applications/calibration/`）、B-R07（GNSS 原生后端/Mag 标定任务缺口）
   给出归属或“首版不做”的放行记录。

## 6. 证据索引

- 本目录：`b_review_checks.json`、`b_review_checks.py`
- B 交付与运行证据：`build/validation/kf_gins_upstream/`、`build/validation/m2_adapter/`、`build/validation/b_replay_fdir/`
- 早期工作稿（内容已并入本文）：`build/validation/review_b/T191-D-REVIEW_B_REVIEW.md`
- A/C 意见：`applications/tests/abi/reviews/A_REVIEW_T191-D-REVIEW.md`、`applications/core/m4/reviews/C_REVIEW_T191-D-REVIEW.md`

## 7. 复核后处置（2026-11-04 深夜，供 D 记录）

| 项 | 处置 | 证据 |
|---|---|---|
| B-R01（05 §7 记法） | **已关闭**：05 §7 增补 `l` 为机体系、`L=C·l` 及 `-skew(L)·C·B` 等价形式，并明确禁止把 NED 系 `L` 直接代入 `-C·skew(L)·B` | `spec/05` §7；SHA-256 `14cf67a1…` → `e7b54532…`；`build/validation/review_b/CLOSURE_B-R01_B-R06.md` |
| B-R06（calibration 目录） | **已关闭**：新建 `applications/calibration/README.md`（归档边界/命名/必要标定集/现状 TBD） | 同上 |
| B-R02/B-R03 | 仍开放：需 D 的 CR（Eigen 依赖）与 A/D 的顶层构建规则（`-fno-short-enums`） | 本意见 §5 |
| B-R05/B-R07 | 仍开放：Kconfig 聚合与 GNSS 原生后端/Mag 标定任务归属，待 D 分配或豁免 | 本意见 §5 B-4 |
| B-R04 | 已由 B 在实现中处理（IMU ring 只保存 base time），无需改规范 | `applications/core/m3` |
