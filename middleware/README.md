# Flight 业务模块

本目录按 [spec 01](../spec/01_系统总体架构与设计决议.md) 组织业务模块。可插拔实现集中在 [模型层](model/)。

| 模块 | 目录 | 入口与职责 |
|---|---|---|
| M1 | [communication](communication/) | `telemetry_link` 地面站链路，`sim` 提供 FMT-SIM HIL 串口与 PC UDP 桥 |
| M2 | [sensor](sensor/) | `sample_acquisition` 采集，`sample_normalization` 样本，`measurement_filter` 筛选，`calibration` 标定，`source_health` 源健康 |
| M3 | [ins](model/ins/) | [KF-GINS 模型](model/ins/kf_gins/)分导航生命周期、观测与回放、设备增量适配 |
| M4 | [fms](model/fms/) | 公共契约和围栏几何，模型内分引擎、任务、操控权、安全和策略 |
| M5 | [guidance](guidance/) | `guidance_update` 公共制导，`trajectory_planning` 轨迹生成与校验，`pilot_and_landing` 驾驶与着陆参考 |
| M6 | [control](model/control/) | 内环分反馈、分配、配置，外环分跟踪、预测和 [求解](model/control/outer_loop/quad_x4/mpc_solver/) |
| M7 | [actuator](actuator/) | `output_guard` 输出守卫，`device_adapter` 执行器设备适配 |
| 协议 | [protocol](protocol/) | `mavlink` 帧、HIL 和方言，`nmea` 句子，`unicore_ascii` 原生 GNSS 报文 |
| 契约 | [contracts](contracts/PUBLIC_INTERFACES.md) | `base_types` 基础类型，`messages` 消息与编号，`ports` 端口，`product_contract` 产品选择，`contract_checks` 契约检查 |
| 仿真 | [plant](model/plant/) | Host 被控对象接口 |

时间基准见 [time_base](../sensor_device/time_base/)，板级外设见 [board](../board/)。模块通过原生设备接口消费硬件，由应用层提供任务和数据交接。目录与构建接入见 [工程目录](../PROJECT_STRUCTURE.md)。
