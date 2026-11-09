# 应用层

`main.c` 是固件入口。[tasks](tasks/) 按任务职责分目录，通信与 HIL 共用 `communication` 目录。根目录的 `task_config.h` 定义公共调度配置，`task_manager.c` 负责创建、启动和失败回收。

| 文件 | RT-Thread 任务 | 周期或触发 | 职责 |
|---|---|---|---|
| [sensor/task_sensor.c](tasks/sensor/task_sensor.c) | sensor | IMU 事件 / 1 ms | 推进导航和控制快环 |
| [mission/task_mission.c](tasks/mission/task_mission.c) | mission | 规划事件 / 20 ms | 运行 FMS 和制导 |
| [mpc/task_mpc.c](tasks/mpc/task_mpc.c) | mpc | 20 ms | 执行 MPC 外环 |
| [actuator/task_actuator.c](tasks/actuator/task_actuator.c) | actuator | 1 ms | 执行输出守卫 |
| [communication/task_comm.c](tasks/communication/task_comm.c) | telemetry | 4 ms | 调度 RC、数传、地面站和慢速传感器 |
| [communication/task_sim.c](tasks/communication/task_sim.c) | sim | 1 ms（仅 `FLIGHT_SIM_LINK`） | 将 HIL_SENSOR/HIL_GPS 注入 M2，回传 HIL_ACTUATOR_CONTROLS |
| [supervisor/task_supervisor.c](tasks/supervisor/task_supervisor.c) | supervisor | 100 ms | 检查任务心跳并管理看门狗 |
| [logger/task_logger.c](tasks/logger/task_logger.c) | logger | 50 ms（`FLIGHT_LOGGER_ENABLE`） | 记录飞行快照和事件到 SD 卡 |
| [flash/task_flash.c](tasks/flash/task_flash.c) | flash | 50 ms（`FLIGHT_NOR_LOG_ENABLE`） | 在台架模式下保存复位和关键事件到 W25Q64 |

通信目录中 `task_rc.c` 处理 RC 轮询和状态，`task_radio.c` 对接数传收发，`task_gcs.c` 处理地面站遥测和命令，`task_sim.c` 负责 HIL。Kconfig 默认关闭板载 HIL。启用后 HIL 独占板级遥测 UART，`task_comm.c` 跳过原生慢速测量轮询。接线和桥接见 [HIL 链路说明](../doc/simulation/HIL_LINK.md)。地面路由和 MAVSDK 命令见 [MAVSDK 接入](../doc/simulation/MAVSDK.md)。

[runtime](runtime/) 保存生命周期、共享数据和遥测。任务复制共享数据后调用 [middleware](../middleware/) 的业务与平台接口。四份编译前参数归工程根的 [config](../config/)。

## 台架在线调参

控制器启动值由 [controller_tuning.yaml](../config/controller_tuning.yaml) 和 [vehicle_model.yaml](../config/vehicle_model.yaml) 生成后编入固件。当前固件可在本地 FinSH 列出、读取和修改 `CTL_*` 参数。

```text
flight_param
flight_param CTL_RATE_KP_X
flight_param CTL_RATE_KP_X 0.15
```

写入要求 `BENCH_INHIBIT` 和 `DISARMED`，并校验参数范围及控制器配置。接受后由后续控制周期使用，重启恢复编译默认值。需要跨重启保留时，更新对应 YAML，运行 `python tools/generate_ground_config.py` 并重编固件。参数名和范围见 [参数目录](../middleware/protocol/mavlink/parameters/ground_parameters.md)。

默认从 USART1 使用 FinSH。打开 USB CDC 虚拟串口后，在 USART1 控制台执行 `flight_console usb` 切换。在 USB 控制台执行 `flight_console uart` 切回 USART1。

## 飞行记录

当前配置启用 `FLIGHT_LOGGER_ENABLE`。启动前插入已格式化的 FAT32 SD 卡。日志任务挂载卡后在 `/LOG/Fxxxxxxx/` 新建会话目录，既有会话保留。

- `STATE.CSV` 名义 20 Hz，包含标准化 IMU、导航、飞行模式、失效状态、四路推力需求和提交回执。末尾记录单调时间 epoch、PPS 驯服状态与频率校正量、UTC 有效位、GNSS 接收时刻、定位质量和拒绝计数。GNSS 质量取自最新解码的 BESTNAV，可用于检查尚未进入融合的帧。零时刻表示尚无样本，列名给出单位。`hil_build=1` 表示使用 HIL 构建。
- `EVENTS.CSV` 记录事件序号、原因、严重度、时间与授权 epoch。通过 `boot_id_hex`、单调时刻和事件编号关联。`event_drops` 累计队列溢出和已出队但写入失败的事件。

文件每秒刷新，组合达到 64 MiB 后开始新会话。写卡在低优先级日志线程执行。板上首次识卡依赖 SDMMC 枚举，启动时插卡是当前支持的用法。

W25Q64 仅使用 `0x040000–0x0FFFFF` 的 768 KiB 关键事件分区。每个 4 KiB 扇区包含一个 64 B 扇区头和 63 条 64 B 记录，保存启动与复位标志、FMS 状态变化和高严重度故障。写入经 CRC32C 校验和单独提交，重启时跳过半写槽。记录与 SD 通过 `boot_id` 和 `event_id` 关联。A/B 参数区、出厂区及其他分区保留。

在 FinSH 执行 `flight_storage` 查看两种记录介质的状态、丢弃数和 I/O 错误数。
