# M5-07 候选轨迹安全放行门禁（设计与实施基线）

状态：**设计基线已建立；实现未完成；不得作为 M6 发布许可。**

## 1. 目标与边界

M5-05 只生成候选轨迹。M5-06 对 M6 实际消费的 anchor + 24 个预览点进行独立连续安全验证。M5-07 将 M5-06 的验证结果绑定到 FMS 授权、ActuationPermit 和 M6 消费入口，形成不可绕过的发布门禁。

唯一允许的轨迹发布链：

`M5-05 candidate → immutable reference snapshot → M5-06 PASS → FMS binding check → Permit bound to candidate → M6 final binding check → reference consumption`

任何缺失、超时、身份不匹配或非 PASS 结果都必须拒绝。不得将 M5-05 的 fallback、上一周期轨迹或任何未验证轨迹作为绕过路径。

本阶段的 M5-07 与执行器 DShot 的 M7（T191-C-M7）不是同一工作项；不得混淆。

## 2. 模块职责

| 模块 | 输入 | 责任 | 禁止事项 |
|---|---|---|---|
| M5-05 | 路径/任务/地图 | 输出候选 MotionReference | 不得声明轨迹安全，不得直达 M6 |
| M5-06 | M6 将消费的精确 MotionReference、导航、地图、动态 track、围栏、SafetyEnvelope | 独立连续验证，输出结构化结果 | 不签发 Permit，不发布 M6 reference |
| M5-07 Gate Adapter | candidate snapshot + M5-06 result | 冻结并绑定候选版本、摘要和验证结果 | 不重新规划、不放宽验证状态 |
| FMS | Gate Adapter 结果、当前导航/authority/directive/mission epoch | 检查结果身份与有效期，决定是否允许签发 Permit | 不自行把 UNKNOWN/STALE/INVALID 转成 PASS |
| M6 | reference snapshot + Permit + Gate result | 消费前再次检查 reference、digest、epoch、TTL 一致性 | 不接受只匹配 reference_id 的弱绑定 |

## 3. 安全绑定字段

验证结果与 Permit 至少绑定以下字段：

- boot_id、time_epoch；
- reference_id、reference_epoch；
- candidate_digest（覆盖 M6 实际消费的 anchor、24 个点、offset、valid mask、frame 和关键 reference header 字段）；
- state_epoch、origin_id；
- authority_epoch、directive_epoch；
- mission_epoch、current_item_id、execution_token（任务轨迹必需；非任务轨迹按契约为零）；
- map_revision；
- verified_at_us、valid_until_us；
- validator_version / algorithm_revision；
- status、reason_code、passed。

digest 算法、序列化顺序、端序、浮点 NaN/负零规范必须写入公共契约并固定测试向量。不得直接对含 padding 的 C struct 原始内存做不稳定哈希。digest 用于候选一致性绑定，不等同密码学认证；消息来源、总线访问控制和契约 header 校验仍需独立完成。

## 4. FMS Permit 门禁

对于需要 3D trajectory reference 的模式（POS_HOLD、AUTO_MISSION、AUTO_RETURN，以及使用 trajectory 的 AUTO_LAND/EMERGENCY_LAND）：

1. guidance 和 feasibility 通过现有检查后，还必须存在当前候选对应的 M5-06 验证结果。
2. 仅 `status == PASS && passed == 1` 可继续；UNKNOWN、STALE、INVALID、碰撞、未证明、预算耗尽、容量耗尽、数值失败一律拒绝。
3. 结果 header 必须与当前 FCU boot/time/frame/clock domain 匹配，且 verified_at 不得在未来、valid_until 必须覆盖本次 Permit lease 及 M6 最晚消费时刻。
4. reference_id/epoch、candidate_digest、state/origin/authority/directive/mission epoch、execution token 和 map_revision 必须逐项匹配当前候选及 FMS 状态。
5. 任一绑定字段变化，旧验证结果立即失效；必须对新候选重新验证。不得复用上一周期 PASS。
6. 门禁拒绝时不得签发或续租 CONTROL_ACTIVE Permit；按现有 FMS safe-action/failsafe 策略处理，并记录稳定 reason code。

手动 rate/attitude 等不消费 M5-05 trajectory 的模式不应伪造 M5-06 PASS；其现有授权策略单独保持，不能借此切换到未验证 trajectory。

## 5. M6 消费端最后一道检查

M6 在采样 MotionReference 前必须检查：

- Permit active、scope、TTL、boot/time/state/origin epoch；
- Permit reference_id/epoch 与 reference 一致；
- Permit 中 candidate_digest 与本次消费的 reference digest 一致；
- M5-06 结果为 PASS 且未过期，绑定字段与 Permit 和 reference 完全一致；
- 当前消息有效位、frame、时间戳和 24×20ms 预览契约有效。

检查失败时不采样候选轨迹、不沿用旧轨迹继续自动飞行；转入已定义的 hold/safe-action/failsafe 路径。M6 不重新解释碰撞几何，也不能自行把未验证状态提升为 PASS。

## 6. 原子性与实时约束

- 使用固定容量、固定内存的双缓冲/不可变快照；验证期间候选不得被写者修改。
- 发布顺序必须是：写完整候选快照 → 完成 digest → 完成验证 → 发布验证结果 → FMS 绑定 → Permit → M6 消费。
- 任何步骤超时、序号跳变、快照版本变化或队列溢出均 fail-closed。
- 不引入动态分配、递归、无界队列或等待式锁；所有检查有明确 WCET/工作预算。
- 不允许“先发布再补验证结果”的竞态；结果必须绑定同一 snapshot generation。

## 7. 必须新增的自动化测试

| ID | 场景 | 预期 |
|---|---|---|
| M507-001 | M5-05 candidate 无 M5-06 result | 不签 Permit、不进 M6 |
| M507-002 | M5-06 UNKNOWN / STALE / INVALID | 拒绝 |
| M507-003 | 碰撞/预算/容量/数值失败 | 拒绝 |
| M507-004 | reference_id 或 reference_epoch 不匹配 | 拒绝 |
| M507-005 | candidate_digest 不匹配（仅改一个 preview 字段） | 拒绝 |
| M507-006 | state/origin/authority/directive epoch 不匹配 | 拒绝 |
| M507-007 | mission token/item/epoch 不匹配 | 拒绝 |
| M507-008 | map_revision 不匹配 | 拒绝 |
| M507-009 | result 未来时间戳、过期、TTL 不覆盖 Permit lease | 拒绝 |
| M507-010 | 验证期间候选快照被替换 | 拒绝，不消费新快照 |
| M507-011 | Permit 续租期间候选 digest 变化 | 撤销旧绑定并重新验证 |
| M507-012 | M6 绕过 FMS 直接提交 reference | 拒绝 |
| M507-013 | 旧 PASS 重放到新 boot/time epoch | 拒绝 |
| M507-014 | 无验证器构建配置 | 所有 trajectory Permit 拒绝；不回退到旧放行逻辑 |
| M507-015 | 手动 rate/attitude 模式 | 不要求伪造 trajectory PASS，不改变其既有授权语义 |

## 8. 分阶段实施

1. **M5-07.1 公共契约**：定义 SafetyValidationResult、candidate digest 规范、消息 ID、ABI/schema 变更和版本兼容策略。
2. **M5-07.2 Gate Adapter**：固定快照、digest、generation 和验证结果原子发布。
3. **M5-07.3 FMS Permit 接线**：trajectory 模式强制验证结果与 Permit 精确绑定；缺失即拒绝。
4. **M5-07.4 M6 消费端复核**：验证 Permit、digest、result、epoch、TTL 一致性，消除旁路。
5. **M5-07.5 SIL 对抗测试**：覆盖上表全部场景、重放、乱序、丢失、重启、竞态和预算耗尽。
6. **M5-07.6 发布审查**：检查 ABI、代码路径、构建配置、静态分析、WCET/RAM/栈和证据链。

## 9. H723 放行约束

当前 M5-06 源码由 `FLIGHT_M5_HOST_PLANNER` 宏保护，明确不编译进 H723/NONE 生产配置。因此：

- 当前 Host/SIL PASS 仅证明 Host/SIL 实现和测试，不代表 H723 有运行中的安全验证器。
- 在 H723 安全验证器尚未移植、测时、资源审查和 HIL 验证前，生产配置不得把缺失的 M5-06 结果视作 PASS。
- M5-07 的无验证器配置测试必须证明 trajectory Permit 被拒绝，而不是绕过验证。
- 任何 H723 避障放行必须另行完成真实传感器标定、WCET/RTA、RAM/栈、HIL、场地测试和正式 Gate 审批。

## 10. 当前未完成项

- M5-06 独立安全算法复核、剩余几何/数值边界测试和 fuzz。
- 公共消息契约及 ABI/schema 变更评审。
- M5-07.1 至 M5-07.6 全部实现与自动化验证。
- H723 安全验证器实现及实机验证。

**当前结论：M5-06 Host/SIL 回归通过，但 M5-06 未完成整体验收；M5-07 目前只有设计基线，M6 仍不得消费 M5-05 未经独立验证的候选轨迹。**
