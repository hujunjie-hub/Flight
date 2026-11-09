# 控制器 / 组合导航交付与验证

本次实现范围为 M3 导航 Port、M6 控制器、QUAD LTI ControlModel、OSQP 静态包及 Host 测试。
沿用现有 SCons/Kconfig、公共契约和 RT-Thread 平台，不重建 BSP，也不自动启动实机输出。

从 Flight 根执行：

```powershell
python middleware/model/control/inner_loop/quad_x4/simulation_config/generate_sim_config.py
python tests/algorithms/run_tests.py
pwsh -NoProfile -File tests/estimator_backend/run_host_tests.ps1 -OutDir build/validation/controller_navigation/upstream_regression
pwsh -NoProfile -File tests/navigation/run_host_tests.ps1 -OutDir build/validation/controller_navigation/replay_regression
```

默认 Host / ARM 编译器路径来自本项目已安装的 STM32CubeCLT。第一条综合测试执行数学/分配、权限与
时间拒绝、MPC 成功与超时、PD 备用、手动/定高、静止对准、无 GNSS、延迟多源融合、独立 GNSS
质量、NIS 异常、stale/gap 和重置行为；并编译全部模块、做 Cortex-M7 可重定位链接，检查未解析的
heap/异常/atomic runtime 符号、全局构造段以及 MPC 的 32 KiB RAM 上限。

实际结果在 `build/validation/controller_navigation/`：`host_tests.log`、`summary.json`、
`commands.json`、`source_hashes.json`、ARM `.su` 与对象文件。结果适用当前编译源；源或配置变化需重跑。
包来源/生成器见 `middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/README.md` 和 `middleware/model/ins/kf_gins/README.md`，接口集成见
`middleware/model/ins/README.md` 和 `middleware/model/control/README.md`。运行时配置保持显式选择，SIM_ONLY 参数不能当作实机标定。

固件台架线程接入、完整H723链接、分片replay与源恢复shadow probation的交付及实际证据见
`applications/runtime/README.md`和`build/firmware_integration/manifest.json`。
未执行或未完成：板上烧录/运行验证、WCET/RTA、动态闭环 SIL/HIL、传感器辨识与实际电机编号/推力标定。
这些状态不由 Host 检查替代，未修改既有交接台账或填入独立审核/飞行通过记录。
