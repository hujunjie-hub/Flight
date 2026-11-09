# 模型层

应用通过各模块根目录的 `<模块>_interface.h` 调用公共 C 入口。开发者在本目录的 `SConscript` 为固件模块绑定实现和专用依赖，Host 通过 Plant 接口绑定被控对象模型。公共消息、Port 和 ABI 归 [contracts](../contracts/PUBLIC_INTERFACES.md)。

| 模块 | 公共入口 | 当前模型 |
|---|---|---|
| 组合导航 | [ins_interface.c](ins/ins_interface.c) | [KF-GINS](ins/kf_gins/) 管理初始化、融合与回放 |
| 飞行管理 | [fms_interface.c](fms/fms_interface.c) | [QUAD_X4](fms/quad_x4/) 管理任务、操控权和安全策略 |
| 控制 | [control_interface.c](control/control_interface.c) | QUAD_X4 [内环](control/inner_loop/quad_x4/)负责姿态、角速度和分配，[外环](control/outer_loop/quad_x4/)负责位置、高度和 MPC |
| 被控对象 | [plant_interface.c](plant/plant_interface.c) | 现有物理仿真由 [外部 FMT-SIM](plant/fmt_sim/) 提供，Host 可绑定本地模型 |

机架与控制参数统一维护在根目录 [config](../../config/)。
