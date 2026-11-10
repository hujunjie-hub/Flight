# Flight 飞控固件

应用接入 IMU、导航、飞行管理、制导、控制与执行器守卫，当前台架配置为 **BENCH_INHIBIT**。本板的 GNSS 与 PPS 固定接入，运行期按质量处理失锁、超时和恢复。资源归属见 [板级外设](board/README.md)和[时间基准](sensor_device/time_base/README.md)。

源码按 [三层工程目录](PROJECT_STRUCTURE.md) 组织。传感器实现位于 `sensor_device`，M1～M7、模型和公共接口位于 `middleware`，任务和运行数据位于 `applications`。机架参数集中在 [config](config/)。

默认使用 GNU Arm release 构建，USART2 提供 QGroundControl 台架遥测。串口和本机 UDP 接入见 [QGroundControl 联机](doc/simulation/HIL_LINK.md#qgroundcontrol-联机)。需要板载 HIL 时启用 `FLIGHT_SIM_LINK` 后重建。验证入口见 [测试说明](tests/README.md)，命名和依赖归属见 [平台架构 §0](spec/03_平台软件架构与驱动设计.md#0-flight-项目目录与现有工程接入)。


## 本地 Agent 开发与验收

本地编码 Agent 从仓库根目录的 [AGENTS.md](AGENTS.md) 开始读取操作规范。完整流程见 [spec/AGENT-操作指南.md](spec/AGENT-操作指南.md) 和 [spec/Flight-验收与结果提交.md](spec/Flight-验收与结果提交.md)。

问题统一登记在 GitHub Issues；测试清单位于 `spec/validation/flight_acceptance_manifest.json`。本地运行完成后使用：

```powershell
pwsh -NoProfile -File .\tools\archive_validation_run.ps1
```

归档输出到 `work/validation/archives/`，不会进入 Git 提交。
