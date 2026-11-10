# Flight 规格入口

从 [00 工程基线](00_工程基线与唯一权威规范.md) 了解当前版本、适用范围和发布状态。

| 主题 | 章节 |
|---|---|
| 系统职责与数据流 | [01 总体架构](01_系统总体架构与设计决议.md) |
| 公共接口、数据与 ABI | [02 公共契约](02_接口契约与公共数据规范.md) |
| 源码目录、命名、驱动与构建 | [03 平台与驱动](03_平台软件架构与驱动设计.md) |
| 制导、控制、执行器与 Plant | [04 飞控核心](04_飞控核心模块与控制系统设计.md) |
| 采集、时间、导航与 FDIR | [05 采集与导航](05_传感器采集时间基准与GNSS独立运行.md) |
| FMS、通信、权限与故障响应 | [06 FMS 与安全](06_FMS通信与系统安全设计.md) |
| 线程、板级映射和 Flash/RAM | [07 实时与资源](07_实时调度与板级硬件资源设计.md) |
| 模型选型、参数与实机准入 | [08 配置与参数](08_产品配置插件与参数管理.md) |
| 工程验收与验证记录 | [09 验收记录](09_需求验证实施与发布管理.md) |

`history/` 保存原始交付与复核，`validation/` 保存规范检查产物。

从 Flight 根目录运行下列检查。

```powershell
python tools/check_layout.py
python spec/tools/audit_spec.py --round 3 --label spec-consolidation
python tests/abi/abi_report.py
```

实际任务与降落验证见 [FMT-SIM 验证矩阵](../doc/simulation/FLIGHT_LOGIC_MATRIX.md)。


## 本地 Agent 与验收交付

- 仓库根目录 `AGENTS.md` 是本地 Agent 强制入口。
- [本地 Agent 操作指南](AGENT-操作指南.md)：拉取分支、环境、命令、修改流程和交接模板。
- [验收与结果提交指南](Flight-验收与结果提交.md)：测试矩阵、M5-06 验收、PASS 证据和归档步骤。
- [问题登记与闭环](quality/README.md)：GitHub Issues 状态机、严重度、关闭条件。
- [机器可读验收清单](validation/flight_acceptance_manifest.json)：测试入口、验收条件、随机种子和最近代码基线。
- 证据归档脚本：`tools/archive_validation_run.ps1`，归档目录 `work/validation/archives/`（已加入 .gitignore）。
