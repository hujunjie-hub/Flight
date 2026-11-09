# T191-D-REVIEW — Agent A 独立复核意见（非作者）

| 项目 | 内容 |
|---|---|
| 复核人 | Agent A（PLATFORM/HARDWARE/ICD） |
| 被复核任务 | `T191-D-REVIEW`（Owner D，task_status `REVIEW`，`update_sequence=3`） |
| 基线 | `V1.9.1-GLOBAL-AUDIT-R3` / Schema 6.0 / Product-Port `0x00010903` |
| 复核范围 | `DOCUMENT_STATIC`：D 的复核记录、检查脚本/JSON、HANDOFF/09/11 处置及预算算术抽查 |
| 复核时间 | 2026-11-04（Asia/Shanghai） |
| 机器证据 | `a_review_checks.json`、`a_review_checks.py`（本目录） |
| 结论 | **ACCEPT_WITH_CONDITIONS**（对 D-REV-01/02/03 的结论无技术异议；附 3 项关闭条件） |

声明：A 不是 D 的执行者，未参与 `T191-D-REVIEW` 的实现；A 是 07 号文件 Owner，因此对
D-REV-02 既作复核也作修正，利益关系在本意见中明示。本意见不代替 B/C 意见，也不构成
Gate A 会签或用户批准。

## 1. 复核方法

1. 通读 `T191-D-REVIEW_report.md`（350 行）、`review_checks.json`（59 项）与
   `t191-d-review-report.json`。
2. 用独立脚本（不复用 D 的 `review_checks.py`）重算：HANDOFF 15 条记录/状态/DAG/owner
   独立性、11 号 Schema 现行声明、07 号 pipeline 单值、09 号登记行、预算算术抽查、
   D 快照计数与描述一致性、`audit_spec.py` 新增选项/状态机检查。
3. 抽查 D 的 4 项发现与 §17 处置；对 A 负责的 07 号冲突直接修正并验证。
4. 未执行：生产 Host/ARM ABI（A 另在 `applications/tests/abi` 提交）、目标构建/WCET/
   HIL/飞行、dialect 复现（环境无 pymavlink）。

## 2. 独立检查结果

`a_review_checks.json`：14/14 检查 PASS，0 FAIL，2 项观察。

| 检查 | 结果 |
|---|---|
| HANDOFF 15 任务、ID 唯一、owner∉reviewer、依赖存在、DAG 无环、状态合法 | PASS |
| `T191-D-REVIEW` = REVIEW / seq 3 / acknowledgement 非空 | PASS |
| `T191-A-ABI` 仍为 READY / seq 2 / ack=null（A 另提交状态请求） | PASS |
| 11 号无现行 Schema 5.0 声明（历史语境除外） | PASS |
| 07 号 pipeline 表值与正文均为 3000us，无 1900us 残留 | PASS（A 修正后） |
| `t191-d-review_report.json` 存在，09 已登记 `TEST-T191-D-REVIEW` = IN_PROGRESS | PASS |
| 预算抽查：CPU 545,705（54.5705%）、900us 链、4000us 窗、7500us 反馈 | PASS |
| D 快照计数 46 PASS / 6 OBS / 5 DRIFT / 2 FAIL | PASS |
| `audit_spec.py` 具备 `--label`、可选外部基线、状态机合法性检查 | PASS |

## 3. 对 D 四项发现的意见

### D-REV-01（11 号 Schema 5.0）— 接受，已验证修复

- 11:9 现为“Schema升为6.0 … 旧Major=5消费者必须一起迁移”；历史小节仍保留 R2 的 5.0/0x00010901
  并带历史限定，符合“现行声明唯一”的要求。
- 独立检查 `doc11_schema_6_no_current_5_claim` PASS。

### D-REV-02（07 号 pipeline 1900/3000us）— 接受，A 已修正

- 冲突事实成立：修正前 07:419 表值 1900us，07:432 与 00:385、05:1109、11:86 均为 3ms；
  D 的 `BUD-12` FAIL 证据准确。
- A 修正（07 号 Owner）：
  - 07:419 `MAX_IMU_PIPELINE_DELAY_US` = **3000us**；
  - 07:432 增加“唯一硬门限”说明（含 group delay/轴间对齐/M2 处理预算，不另设第二个常量，
    不按旧 1.8ms 解释）；
  - 修正前 SHA256 `27b419c4…30b7dc4` → 修正后 `aacf4ca9…e9a47ca`。
- 影响：D 的 round3/`inputs_sha256`、`t191-d-review_report.json` 与 `BUD-08/12` 需在
  修正后的树上重跑；这是关闭条件 C2。

### D-REV-03（证据链无法重建）— 接受，工具修复已抽查

- `audit_spec.py` 已将“全部任务 READY”断言改为 `handoff_task_records_legal` 状态机检查，
  新增 `--label`、`--outside-baseline`、`--write-outside-baseline`；
  `t191-d-review_report.json` 以 `label=t191-d-review` PASS（states = READY/REVIEW）。
- 保留意见：工具修复不等于新基线证据已重建；07 修正后仍须重建一次（C2）。

### D-REV-04（HANDOFF 与工作区不一致）— 部分闭环

- D 已将 `T191-D-REVIEW` 合并为 REVIEW/seq 3，记录合法。
- A 侧：本次补交 `T191-A-ABI` 状态请求（见 `applications/tests/abi/status_request_T191-A-ABI.json`，
  submission revision 2），并把本复核意见作为 reviewer evidence 引用；B/C 侧仍待其提交。
- 因此 D-REV-04 仍应保持开放，直到 A/B/C 全部登记。

## 4. 对 D 复核记录本身的观察（不阻塞）

| 编号 | 级别 | 内容 | 建议 |
|---|---|---|---|
| OBS-A-01 | P3 | `review_checks.json` 是 D 修复前快照（ABI-02 仍 FAIL），但报告摘要仍按其计数；当前 11:9 已修复，易误读。 | D 在修正后的树上用 `--label` 重跑并更新报告摘要/ABI-02 状态。 |
| OBS-A-02 | P3 | `TOOL-04` 描述为“reproducible … (pymavlink present)”，detail 却是 `pymavlink=false`，与报告 §2 矛盾。 | 改为“NOT reproducible: pymavlink absent”或补安装证据。 |

## 5. 关闭条件（供 D）

1. **C1**：D 在修正后的树上重跑 `review_checks.py`（新 label），更新报告摘要、ABI-02/
   BUD-12 状态，并保留旧快照为历史。
2. **C2**：D 依 D-REV-03 工具重建 round3/`inputs_sha256`/`change_summary`，使 07 修正后的
   基线可复核；`BUD-08` 与 `BUD-12` 重跑通过；并更新 09:386 中“D-REV-02 仍待 A 处置”的表述。
3. **C3**：D 合并 A-ABI 的补交状态请求；`T191-D-REVIEW` 进入 ACCEPTED 仍需 B/C 至少各一份
   非作者意见（A 的本意见仅为其中一份），且用户按 Gate A 记录适用批准。

## 6. 未执行

- D-REVIEW 范围内的生产 ABI/目标/HIL/飞行（NOT_RUN）。
- dialect 复现（pymavlink 缺失）。
- B/C 意见与用户 Gate A 批准。

## 7. 回滚

- 本复核只新增 `applications/tests/abi/reviews/` 三份文件，未修改 D 的
  `spec/validation/T191-D-REVIEW/` 文件、HANDOFF 或 09。
- 07 修正的回滚点为 SHA256 `27b419c4…30b7dc4`；除用户明确否决 3ms 结论外不应回滚到
  1900us（会恢复 D-REV-02 冲突）。
