# Flight Issue 关闭检查清单

只有全部适用项目完成，才允许从 VERIFIED 进入 CLOSED。S0/S1 和安全相关问题默认不得跳过独立复核。仅在项目所有者对明确命名的单项问题豁免外部独立审查时，才可用记录完整的三轮内部复核替代；该状态必须标为 TRIPLE_SELF_REVIEWED_WITH_SCOPE_LIMITATIONS，不得冒充 VERIFIED 或独立第三方批准。当前仅 FLIGHT-LOCAL-04/MR-009（2026-11-13）适用。

## A. 问题身份与根因

- [ ] Issue 标题、严重度、类型、Area 和 Quality Status 正确。
- [ ] 问题根因已确认，不只是症状描述。
- [ ] 首次发现 commit、基线 commit、修复 commit、复测 commit 均准确。
- [ ] 相关 Issue 已关联；没有把多个独立根因混在一个 Issue。

## B. 修复完整性

- [ ] 修复范围与 Issue 一致，没有无关大规模重构。
- [ ] 公共接口/ABI/Profile/安全策略变化已走 CR 并获批准（如适用）。
- [ ] 原失败测试保留，并增加防止回归的测试。
- [ ] 没有删除用例、降低断言、放宽安全阈值、增加无解释 skip。
- [ ] 随机测试 seed、生成器版本、失败向量和最小反例已保存。

## C. 实际复测证据

- [ ] 在修复 commit 上重新执行原始复现命令。
- [ ] 原失败用例通过，退出码为 0。
- [ ] 受影响的单元/集成测试通过，检查数大于零、失败数为零。
- [ ] 适用的 ASan/UBSan、ARM compile、ABI、资源和 spec 检查通过。
- [ ] 原始日志、报告、环境、工具链、命令、测试数、失败数均可复核。
- [ ] 证据包已生成，SHA256 已记录。
- [ ] CI 结果对应精确复测 commit；没有沿用旧 commit 的 PASS。

## D. 安全/数学问题额外检查

- [ ] UNKNOWN、STALE、INVALID、数值失败、超时、容量/预算耗尽均按预期 fail-closed。
- [ ] 已检查相邻边界、精确边界、±1 ULP、时间/epoch/origin/frame 变化（适用时）。
- [ ] 数学推导有独立复核；若项目所有者对该单项明确豁免外部独立审查，则必须有三轮内部复核记录、证据和范围限制，且不得声称第三方认证；采样 oracle 未被描述为形式化证明。
- [ ] 目标编译、Host/SIL、目标板、HIL、飞行证据范围清晰区分。
- [ ] 没有把 Host/SIL 或 Cortex-M7 compile 误写为 H723 实机放行。

## E. 独立审核与关闭

- [ ] 非修复作者复核测试证据。
- [ ] 安全关键问题默认取得明确 APPROVED，而非 REVIEW_PENDING；owner-waiver 特例须记录豁免授权及 TRIPLE_SELF_REVIEWED_WITH_SCOPE_LIMITATIONS disposition，不得记为独立 APPROVED。
- [ ] Issue 描述和评论更新修复 SHA、复测 SHA、RUN_ID、日志 SHA256。
- [ ] spec/09 中相关验收项状态同步（如适用）。
- [ ] 关闭理由清楚，状态更新为 VERIFIED，再关闭 GitHub Issue。
- [ ] 如果仍有 blocker、待用户决策或待硬件验证，保持 Open / BLOCKED，不得关闭。

关闭评论必须简明写明：根因、修复、复测命令、实际计数、复测 commit、证据包、独立审核者或项目所有者明确豁免记录，以及未覆盖范围。
