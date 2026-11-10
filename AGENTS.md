# Flight 本地 Agent 执行入口

本文件是所有本地编码 Agent 的强制入口。开始任何任务前，必须先完整读取：

1. [本地 Agent 操作指南](spec/AGENT-操作指南.md)
2. [验收与结果提交指南](spec/Flight-验收与结果提交.md)
3. [问题登记与闭环规则](spec/quality/README.md)
4. [工程验收唯一台账](spec/09_需求验证实施与发布管理.md)
5. 当前任务对应的设计/实现/验证文件，M5-06 还必须读取：
   - `tests/guidance/ADVANCED_PLANNER_VALIDATION_MATRIX.md`
   - `spec/history/implementation/guidance/M5_06_2_VALIDATION_MATRIX.md`
   - `spec/history/implementation/guidance/M5_06_4_ADVERSARIAL_TESTS.md`
   - `spec/history/implementation/guidance/M5_06_5_INDEPENDENT_MATH_REVIEW.md`

## 强制规则

- 以 `spec/00` 至 `spec/09` 为设计和安全约束；以 `spec/09` 为唯一正式验收状态台账；以 GitHub Issues 为问题状态唯一权威记录。
- 默认在当前任务指定的功能分支工作。本轮 M5-06 分支为 `opt/stage-02-m5-06-safety-validator`，PR #2 目标分支为 `opt/stage-01-host-validation`。
- 不得切换或提交到 `main`；不得合并 PR；不得将 Draft PR 改为 Ready；不得 force push。除非用户明确要求，Agent 只做本地修改和本地 commit，不 push。
- 不得擅自修改公共 ICD、ABI、Product Profile、硬件绑定、FMS 授权边界或安全策略。需要修改时先提交变更影响和 CR 建议，等待用户批准。
- M5-05 输出始终是 candidate。M5-06 未通过验证和独立数学审查前，不得接入 M6，不得开始 M5-07，不得授予 Permit。
- H723 当前产品配置为 `QUAD_X4 / STM32H723ZGT6 / OBSTACLE_SOURCE_MODE=NONE / BENCH_INHIBIT`。M5-06 地图/动态障碍物验证属于 Host/SIL；不得宣称 H723 已具备在线避障能力。
- UNKNOWN、STALE、INVALID、数值失败、预算耗尽、容量耗尽、超时或任何无法证明安全的结果必须 fail-closed。
- 不得把源码存在、编译通过、单元测试通过、Host/SIL 通过、Cortex-M7 交叉编译通过互相替代。更不得将它们描述为 H723 实机、HIL 或飞行放行。
- 每次修改先复现问题，再增加/修复测试，再修复代码；必须保存测试 ID、命令、实际计数、失败数、随机 seed、最小反例、commit SHA 和日志。
- 不得删除失败测试、降低断言、放宽安全阈值、跳过失败步骤或把失败改成 SKIP 来制造 PASS。
- 完成后必须按验收指南生成本地证据包，并输出简明交接报告。未完成项明确标记 BLOCKED / NOT_RUN，不得猜测。

## Agent 开始工作时的第一条输出

先报告：仓库路径、当前分支、HEAD SHA、工作区是否干净、目标任务 ID、已读取的规范文件、准备运行的测试命令。若当前分支不正确或存在用户未提交改动，先停止并报告，不得自行 reset、clean、stash 或覆盖。