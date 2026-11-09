# Flight 工程目录

```text
sensor_device/        # 传感器、执行器、存储、遥测和时间设备
  imu/adis16505.c      mag/bmm350.c      barometer/bmp585.c
  gnss/um982.c         pmu/pmu.c
  actuator/dshot.c    # DShot300，RT-Thread 设备
  telemetry/
    elrs/elrs.c        # ELRS 遥控接收机
    data_radio/data_radio.c  # 数传电台
    sim/hil_serial.c   # HIL 仿真串口
  storage/
    W25Q64/w25q64jv.c  # NOR Flash，RT-Thread MTD NOR
    SDMMC/drv_sdmmc.c  # microTF，RT-Thread MMC/SD 主机
  time_base/          # 硬件计数、捕获、单调时钟与 PPS
middleware/               # 模块头文件与实现并排，专用后端随模型归档
  communication/      # 遥测队列、HIL 帧收发与 PC 仿真链路
  param/              # 会话参数存储、范围校验与应用
  protocol/           # mavlink、nmea、unicore_ascii
  guidance/           # 制导入口与默认值、轨迹、驾驶和着陆参考
  actuator/           # output_guard、device_adapter
  storage/            # W25Q64 关键事件环
  contracts/          # base_types、messages、ports、product_contract、contract_checks
  sensor/
    sample_acquisition/  sample_normalization/  measurement_filter/  calibration/  source_health/
  model/
    ins/kf_gins/      # navigation_estimator、observation_replay、backend_adapter、trimmed_core
    fms/quad_x4/      # flight_state_machine、mission_lifecycle、operator_authority、flight_safety、flight_policy
    control/
      inner_loop/quad_x4/  # attitude_rate_control、control_allocation、simulation_config
      outer_loop/quad_x4/  # position_tracking、prediction_model、mpc_solver
    plant/fmt_sim/    # 外部物理仿真模型接入说明
applications/
  main.c
  tasks/
    task_config.h     task_manager.c
    actuator/         flash/
    logger/           mission/          mpc/
    sensor/           supervisor/
    communication/    # task_comm、task_rc、task_radio、task_gcs、task_sim
  runtime/            # runtime.c、runtime_data.c
config/               # 五份工程参数（含运行期校准目录）
board/                # 板级绑定、启动标识和诊断
tests/                # Host/ARM 验证与可选的板上 utest
tools/                # 工程通用生成器、布局检查与仿真桥接
spec/                 # 00～13 规范、contracts/、tools/、validation/、history/
doc/calibration/      # 标定报告和原始数据
```

| 源码层 | 内容 |
|---|---|
| [sensor_device](sensor_device/) | 传感器按类别归档，存储驱动分为 `W25Q64` 和 `SDMMC`。器件的源码和头文件就近维护，保留原厂许可证与来源记录。 |
| [middleware](middleware/) | spec 的 M1～M7、模型实现与公共接口。 |
| [applications](applications/) | 应用入口、任务表、每任务的周期工作和共享运行数据。 |
| [tests](tests/) | 按模块分组的 Host、目标编译与集成验证，仿真验证位于 `simulation/`。 |
| [tools](tools/) | 工程布局检查、MAVLink 与机架配置生成、FMT-SIM 串口桥接工具。 |

五份工程参数集中在 [config](config/)。控制调参见 [controller_tuning.yaml](config/controller_tuning.yaml) 的 `inner_loop` 和 `outer_loop`；运行期校准项见 [calibration.yaml](config/calibration.yaml)。

应用调用中间层接口。M1、M2 和 M7 使用 RT-Thread 原生设备接口，传感器驱动通过原生总线和平台硬件服务访问设备。板级角色与总线实例的绑定集中在 [board_device.h](board/board_device.h)。

| 模块 | 目录 | 职责 |
|---|---|---|
| M1 | [communication](middleware/communication/) | GCS/RC 链路诊断、遥测收发和 HIL 帧接入 |
| 参数 | [param](middleware/param/) | 会话参数的存储、校验与应用 |
| 协议 | [protocol](middleware/protocol/) | MAVLink 帧与方言、HIL 消息、NMEA 句子、Unicore ASCII 报文 |
| M2 | [sensor](middleware/sensor/) | 标准样本、时间映射、标定、源隔离与恢复 |
| M3 | [ins](middleware/model/ins/) | 导航状态估计、延迟观测重放；KF-GINS 后端整体在 [ins/kf_gins](middleware/model/ins/kf_gins/) |
| M4 | [fms](middleware/model/fms/) | 任务、模式、操控权、许可与围栏 |
| M5 | [guidance](middleware/guidance/) | 轨迹与驾驶、着陆参考 |
| M6 | [control](middleware/model/control/) | 位置 MPC 与 [OSQP 求解器](middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/)、姿态和角速度控制、控制分配 |
| 仿真 | [plant](middleware/model/plant/) | Host 物理被控对象接口与外部模型接入 |
| M7 | [actuator](middleware/actuator/) | 许可复核、输出守卫、设备接入与提交回执 |
| 设备 | [sensor_device](sensor_device/) | 传感器、`actuator/` 电调驱动、`storage/` 存储驱动和 `time_base/` 时间设备 |
| BSP | [现有驱动](libraries/HAL_Drivers/drivers/) 与 [板级接入](board/) | 现有外设驱动、硬件绑定、板级诊断和待补齐的设备后端 |
| 公共接口 | [contracts](middleware/contracts/PUBLIC_INTERFACES.md) | ICD、Port、消息编号、产品契约与 ABI 锁 |

公共文件、接口和配置路径按职责命名。机架名归模型实现目录，器件名归对应驱动。BSP、RT-Thread 和第三方库由原有 SCons 工程接入。

命名规则和构建接入见 [spec/03 §0](spec/03_平台软件架构与驱动设计.md#0-flight-项目目录与现有工程接入)。工程源码的对象和栈报告由 SCons 写到 `build/` 下的对应目录。

目录自检：`python tools/check_layout.py`（输出 `build/three_layer_refactor/layout_checks.json`）。
