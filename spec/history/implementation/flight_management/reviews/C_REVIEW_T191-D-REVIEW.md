# T191-D-REVIEW — Agent C 独立复核意见（非作者，revision 2）

| 项目 | 内容 |
|---|---|
| 复核人 | Agent C（VEHICLE/CONTROL） |
| 被复核任务 | `T191-D-REVIEW`（Owner D，`task_status=REVIEW`，`update_sequence=4`，acknowledgement 已记录） |
| 基线 | `V1.9.1-GLOBAL-AUDIT-R3` / Schema 6.0 / Product-Port `0x00010903` |
| 复核范围 | `DOCUMENT_STATIC`；D 的 59 项检查（含 post-fix 重跑）、处置记录、预算算术、证据哈希；C 负责的 04/06/08 与 02 的 M4/M5 契约交叉核对 |
| 复核时间 | 2026-11-04（Asia/Shanghai），revision 2 |
| 机器证据 | `c_review_checks.py` / `c_review_checks.json`：**22 项 = 15 PASS / 1 FAIL / 6 OBS**；D 的 post-fix 证据见 `spec/validation/T191-D-REVIEW/review_checks_post.json` |
| 结论 | **ACCEPT_WITH_CONDITIONS**（提交给 D 作为 C 的书面意见；不是 `ACCEPTED`、不是 Gate A 会签） |

声明：C 不是 `T191-D-REVIEW` 的执行者，未参与其记录/脚本编写。C 是 04/06/08 与 M4/M5
实现的 Owner，故对 C 领域的契约问题作技术判断；本意见不代替 A/B 意见、D 的合并或用户的
Gate A 批准，也不构成 09 台账 PASS。

本 revision 2 响应 D `update_sequence=4` 的 `next_action`（“待 C 书面意见与用户 Gate A 批准”），
并使用 D 在 23:28 的 post-fix 重跑，而不是 22:32 的修复前快照。

## 1. 复核方法

1. 通读 `T191-D-REVIEW_report.md`、`review_checks.json`（22:32 快照）、`review_checks_post.json`
   （23:28 post-fix 重跑）、`commands.log`、`SHA256SUMS.txt`（当前 6 个文件）及 §17 处置。
2. 独立脚本 `c_review_checks.py`（不导入 D 的 `review_checks.py`）解析 HANDOFF、09、11、07、02、06、
   生产契约头和 D 的证据文件；重算 CPU/时延/DShot/M5 预算；检索 C 领域契约。
3. 核对当前 HANDOFF：`T191-D-REVIEW=REVIEW/seq4`、`T191-A-ABI=ASSIGNED/seq3`、B 的四份状态请求与
   `B_REVIEW_T191-D-REVIEW.md`、C 的四份状态请求均已在盘。
4. 未执行：生产 Host/ARM ABI 复跑（A 负责）、目标构建/WCET/HIL/飞行、dialect 复现（无 pymavlink）、
   用户 Gate A 批准。

## 2. 独立检查结果

`c_review_checks.json`：22 项 = 15 PASS / 1 FAIL / 6 OBS。

| 检查 | 结果 | 说明 |
|---|---|---|
| GOV-01..04 | PASS | HANDOFF 15 任务合法、DAG 无环；D-REVIEW=REVIEW/seq4；A-ABI=ASSIGNED/seq3；B/C 各 4 份状态请求在盘 |
| EVID-01..03 | PASS | `SHA256SUMS.txt` 当前 6 个文件全部匹配；post-fix 59 项 = 46/2/6/5，FAIL 仅 EVID-01/02（历史输入漂移，预期） |
| REV-01 | PASS | D-REV-01 已修：11:9 = Schema 6.0 |
| REV-02 | PASS | D-REV-02：07 表值 = 3000us；post-fix 重跑已无 BUD-12 FAIL |
| REV-03 | PASS | `audit_spec.py` 具备 `--label`/`--outside-baseline`/`--write-outside-baseline` |
| BUD-01..04 | PASS | CPU 54.5705%、900µs 链、DShot 917/344/688/916 与 458/172/344/457、M5 62,016/61,952 差 64B 均可复算 |
| REC-01 | PASS | 09 存在 `TEST-T191-D-REVIEW` = IN_PROGRESS |
| **C-REV-01** | **FAIL** | 06 §12 要求 mission 命令携带当前 `mission_epoch`，但 `OperationRequest`/`ProductOperationPayload` 无该字段（§3） |
| C-REV-02 | OBS | 06:433 规定 M4 为围栏唯一 Owner，但 02 无围栏命令/字段，`FmsInputView` 无围栏输入 |
| C-REV-03 | OBS | 02:2559 要求实机 `output_policy_id>=0x100` 登记验证，08/09 未定义登记载体 |
| REC-02 | OBS | 09 尚无 C-FMS/C-M5 实现证据行（D 受理后登记） |
| OBS-01 | OBS | 22:58 的报告 Markdown 仍引用修复前快照，§17 仍写“07:419=1900µs、D-REV-02 未处置” |
| OBS-02 | OBS | D 计 143 个规范类型 vs A 生产 ABI 142；计数口径需注释 |
| OBS-03 | OBS | B-R05 同样适用于 C：`FMS_QUAD_X4_ENABLE` 已定义，但 `applications/core/Kconfig` 未被根/应用 Kconfig 链 source，menuconfig 不可达 |

## 3. C 侧发现

### C-REV-01 · P2 · Owner A/D（02/06）· mission 命令的 `mission_epoch` 无承载字段

- 06:632 要求 `MISSION_START/PAUSE/RESUME/ABORT/CLEAR` 携带“当前 `mission_epoch`”。
- `OperationRequest`（`fc_common_messages.h:32-42`）与 `ProductOperationPayload`
  （`product_contract.h:125-136`）均无该字段；`payload_schema_id`/`request_id` 不能替代。
- 影响：M4 只能按 `mission_id` + 认证 source/session 绑定，无法拒绝同一 mission 的旧 epoch 命令；
  C 的 `T191-C-FMS` 已如实标注该缺口，不能声称满足 06:632。
- D 的报告未发现此项（无 `mission_epoch` 命中）。
- 建议：按 10 §3 开 CR，在 02 增加 `mission_epoch` 字段（或明确由 `OperationRequest` 承载并给出
  valid_mask/零值规则），同步修改 06:632，升级 ABI 并重跑 A 的 ABI 锁。

### C-REV-02 · P2/P3 · Owner A/D · 围栏配置入口未定义

- 06:433 规定 M4 为 Geofence 唯一 Owner 并发布 `ProductGeofenceSnapshot`，Companion 只能提交候选。
- 02 的 14 条 QUAD 命令无 `SET_GEOFENCE`，`ProductOperationPayload` 无围栏字段，`FmsInputView`
  无围栏输入；08 又将围栏门限列为“车辆专用安全契约”，不属于普通参数。
- C 的实现只能提供产品非 Port 钩子 `quad_x4_fms_set_geofence()`；在 02/06/08 明确入口前，围栏
  链路只能记 NOT_RUN。
- 建议：新增受控命令/字段（CR/ABI），或明确围栏来自编译期 Product 配置并由 Profile 绑定、禁止
  运行时修改。

### C-REV-03 · P3 · Owner A/D · 实机输出策略登记位置缺失

- 02:2559 要求 `>=0x100` 的实机策略“需登记验证”，但未定义登记表、字段和批准人。
- C 的实现对 `>=0x100` fail-closed（`STATUS_UNSUPPORTED`）；首版安全，但实机策略无法被合法启用。
- 建议：08 增加受控安全策略登记表，或在 09 Gate F 明确登记载体。

## 4. 对 D 四项发现与当前处置的意见

| 项 | C 意见 |
|---|---|
| D-REV-01 | 接受；11:9 已修，C 独立验证通过。 |
| D-REV-02 | 接受事实；07 表值已为 3000us，post-fix 重跑无 BUD-12 FAIL。D 报告 Markdown §17 尚未更新（OBS-01），应同步后再关闭。 |
| D-REV-03 | 接受；工具修复存在；“新基线证据重建”仍需在 09 记录命令/结果，属 NOT_RUN 而非设计缺陷。 |
| D-REV-04 | **C 部分已提交**：C 的四份状态请求（FMS rev3、M5 rev2、M6/M7 NOT_STARTED）与索引在 `applications/core/status_requests/`；B 亦已提交。D 合并后可关闭 A/B/C 输入部分；本意见不代替 D 的记录动作。 |

## 5. 观察项与对 B 意见的交叉确认

| 编号 | 内容 | 建议 |
|---|---|---|
| OBS-C-01 | `SHA256SUMS.txt` 已覆盖 6 个文件（含 post-fix JSON），但报告 Markdown 未同步；§17 的 D-REV-02 描述已过期 | D 重生成报告/摘要，或在文件头明确“post-fix 以 JSON 为准” |
| OBS-C-02 | 143（规范类型）与 142（生产 ABI 类型）口径不同 | 在报告中注明“规范块/生产类型” |
| OBS-C-03 | B-R05（Kconfig 聚合缺失）同样命中 M4/M5：`FMS_QUAD_X4_ENABLE` 不可达 | D/A 统一在根/应用 Kconfig 增加受控 source；C 的模块 Kconfig 已准备好 |
| OBS-C-04 | 09 无 C-FMS/C-M5 实现行 | D 受理后登记 executor-run Host/ARM 证据，目标/HIL 保持 NOT_RUN |
| OBS-C-05 | B 的 B-R02（Eigen 未锁定）、B-R03（short-enums）与 C 的 ARM 编译旗标相关 | C 已在 ARM 检查中使用 `-fno-short-enums`；Eigen 属 B 域，C 不代签 |

## 6. 关闭条件（供 D）

1. **C1**：在 post-fix 树上重生成 `T191-D-REVIEW_report.md`/摘要（或明确标注“JSON 为准”），同步
   §17 的 D-REV-02 状态；`SHA256SUMS.txt` 与当前 6 个文件保持一致。
2. **C2**：按 C-REV-01 开 CR（`mission_epoch` 字段）；在 CR 解决前不得把 06:632 标为满足，也不得
   把 M4 任务验收为“命令 epoch 校验通过”。
3. **C3**：按 C-REV-02 明确围栏配置入口（命令或编译期配置）并同步 02/06/08；否则 M4/M5 围栏
   链路记 `NOT_RUN`。
4. **C4**：合并 C 的四份状态请求（`applications/core/status_requests/index.json` 为索引）与 B 的
   四份请求，关闭 D-REV-04 的 A/B/C 输入部分；C 部分不待本意见重复提交。
5. **C5**：受理 C-FMS/C-M5 时在 09 登记 executor-run Host/ARM 证据，目标构建/WCET/SIL/HIL 与
   独立审核保持 `NOT_RUN`；不得由 C 自签。
6. **C6**：处理 OBS-C-03 的 Kconfig 聚合（B-R05 同项），使 `FMS_QUAD_X4_ENABLE`/`M5_QUAD_X4_ENABLE`
   经 menuconfig 可达；在此之前保持默认关闭。

## 7. 未执行

- 目标固件构建、map/WCET/RTA/栈水位、HIL/台架/飞行。
- dialect 复现（pymavlink 缺失）。
- 用户 Gate A 批准。
- C-FMS/C-M5 的生产 ABI 重跑（A 的 ABI 工具负责；C 只做消费者编译）。

## 8. 回滚

本 revision 2 只更新 `applications/core/m4/reviews/` 三份文件（本意见、脚本、JSON）并新增
`C_REVIEW_SUBMISSION.json`；未修改 D 的 `spec/validation/T191-D-REVIEW/`、`spec/AGENT_HANDOFF.md`、
09 或任何 00–12 正文。删除新增的 submission JSON 并回退本目录三文件即回滚。
