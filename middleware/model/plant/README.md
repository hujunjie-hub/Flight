# 被控对象物理模型

Host/SIL 通过 `plant_interface_bind` 选择一个物理模型，通过 `plant_sim_init`、`plant_sim_reset` 和 `plant_sim_step` 驱动。实现放在以模型命名的子目录。PlantTruth 供传感器仿真和误差评估，控制预测模型在控制外环。

当前 HIL 的物理模型运行在外部 FMT-SIM。Flight 通过 MAVLink 交换传感器观测和控制需求。详见 [FMT-SIM 接入](fmt_sim/)。本机 SIL 的现有夹具使用合成测量。Host 接口与物理模型不进入固件构建。
