# FMT-Firmware 线程任务与驱动梳理

> 源码: `ref/FMT-Firmware` (Firmament 自驾仪, 基于 RT-Thread)。
> 参考目标板: pixhawk fmu-v5 (`RT_THREAD_PRIORITY_MAX = 32`, `RT_TICK_PER_SECOND = 1000`,即 1 tick = 1 ms)。
> RT-Thread 中**优先级数值越小优先级越高** (0 = idle 线程)。
> 本文所有路径均相对 `ref/FMT-Firmware/`。

## 0. 任务注册与调度框架

- 系统任务通过 `TASK_EXPORT` 宏 (`src/module/task_manager/task_manager.h`) 静态注册到链接段 `TaskTab`,`src/module/task_manager/task_manager.c` 的 `task_manager_init()` 依据依赖关系依次调用各任务 `init()`,用 `rt_thread_create(name, entry, ..., priority, 1)` 创建线程;`task_manager_start()` 启动所有 `auto_start = true` 的任务。
- 线程几乎都是**事件驱动**: 周期性硬定时器 (`RT_TIMER_FLAG_HARD_TIMER`,在 OS tick 中断上下文执行) 发送 `rt_event` 唤醒对应线程,或串口 rx indicate 回调发事件。
- 大量轻量周期作业不占线程,作为 **work item** 挂到两个工作队列线程执行 (见 §3)。
- 任务间数据通过 uMCN 发布/订阅总线 (`src/module/mcn`) 传递。

## 1. 线程任务表 — 系统任务 (TASK_EXPORT 注册,task_manager 创建)

| 线程名 | 优先级 | 栈 (B) | 触发方式 / 执行频率 | 线程内容 | 源码 |
|---|---|---|---|---|---|
| **vehicle** | 3 (`VEHICLE_THREAD_PRIORITY`) | 6144 | 硬定时器 1 tick 发事件,事件驱动 (1 ms 粒度) | 飞控主算务线程。每次唤醒:① `sensor_collect_high_freq()` — 内部 1 ms timetag 门控,采集 IMU0/IMU1,做板级旋转、校准、三阶 Butterworth 滤波,发布 `sensor_imu0`/`sensor_imu1` (1 kHz);② `pilot_cmd_collect()`/`gcs_cmd_collect()`/`mission_data_collect()` 收遥控/GCS/任务指令;③ `PERIOD_EXECUTE` 门控分步执行模型: `ins_step` 2 ms (500 Hz, CF_INS,`INS_EXPORT.period=2`),`fms_step` 4 ms (250 Hz, MC FMS,`period=4`),`control_step` 2 ms (500 Hz, MC Controller,`period=2`) (换 INS/FMS/Control 模型时周期随 `*_EXPORT.period` 变化,如 px4_ecl INS 为 2 ms);④ `send_actuator_cmd()` 把控制量写到 main/aux 输出设备 (PWM/DShot/DroneCAN) | `src/task/vehicle/normal/task_vehicle.c` |
| **sensor** | 4 (`SENSORS_THREAD_PRIORITY`) | 3072 | 硬定时器 5 ms 发事件 (200 Hz) | 低频传感器采集线程 `sensor_collect_low_freq()`: 磁力计 10 ms timetag (100 Hz) 采集/旋转/校准后发布 `sensor_mag0`;气压计按设备就绪标志读取发布 `sensor_baro`;GPS 按就绪标志读取发布 `sensor_gps`;空速计 20 ms timetag (50 Hz) 发布 `sensor_airspeed`。HIL/SIH 模式下此函数被编译剔除 | `src/task/sensor/task_sensor.c`, `src/module/sensor/sensor_hub.c` |
| **dronecan** | 5 (`DRONECAN_THREAD_PRIORITY`) | 40960 | 硬定时器 1 tick 发事件 (1 kHz) | DroneCAN (UAVCAN v0) 节点线程 `dronecan_loop()`: 每次 `sendCanard()`/`receiveCanard()` 处理 CAN 收发;每 1000 次 (1 s) 广播 NodeStatus;承载 ESC/_GPS 等通过 DroneCAN 接入的设备 | `src/task/dronecan/task_dronecan.c`, `src/module/dronecan/dronecan.c` |
| **fmtio** | 7 (`FMTIO_THREAD_PRIORITY`) | 1536 | 串口 rx indicate 事件或 10 ms 超时轮询 | FMU↔IO 协处理器通信线程 `fmtio_loop()`: 启动时 `io_default_config()` 重配 IO;`handle_rx_packet()` 解析 IO 协议包 (RC 输入转发、PWM 输出、safety/LED 状态等)。目标无 IO 协处理器时设备打开失败即不工作 | `src/task/fmtio/task_fmtio.c`, `src/module/fmtio/fmtio.c` |
| **mavgcs** | 11 (`MAVGCS_THREAD_PRIORITY`) | 5120 | 通道硬定时器 1 tick 发 `EVENT_MAVPROXY_UPDATE` 事件 + 控制台/参数事件 | 地面站 MAVLink 链路线程 `mavproxy_channel_loop(GCS)`: 按注册速率发送周期消息 — HEARTBEAT/SYS_STATUS/SYSTEM_TIME/EXTENDED_SYS_STATE 1 Hz、ATTITUDE_QUATERNION 10 Hz、LOCAL_POSITION_NED/GLOBAL_POSITION_INT/VFR_HUD/ALTITUDE/GPS_RAW_INT/RC_CHANNELS 各 5 Hz、HIL_STATE 50 Hz;处理收到的 MAVLink (命令、参数上/下载、任务协议、FTP、RTCM 注入、mavlink console) | `src/task/comm/task_comm.c`, `src/task/comm/mavgcs.c`, `src/module/mavproxy/mavproxy.c` |
| **mavobc** | 12 (`MAVOBC_THREAD_PRIORITY`) | 2048 | 同 mavgcs 机制 (OBC 通道);依赖任务 `mavgcs` 初始化成功 | 机载计算机 (OBC) MAVLink 链路线程 `mavproxy_channel_loop(OBC)`: 周期消息 HIL_SENSOR 100 Hz、HIL_GPS 10 Hz、HEARTBEAT/SYS_STATUS/EXTENDED_SYS_STATE 1 Hz;支持 GCS 侧 `REQUEST_DATA_STREAM` 按 `mav_msg_cb_table` 动态注册消息流 | `src/task/comm/task_comm.c`, `src/task/comm/mavobc.c` |
| **logger** | 13 (`LOGGER_THREAD_PRIORITY`) | 2048 | `mlog`/`ulog` 更新事件或 20 ms 超时;依赖任务 `vehicle` | 日志线程: `mlog_async_output()` 将飞行数据二进制日志缓冲刷入 SD 卡,`ulog_async_output()` 刷系统日志;超时分支额外对 ulog 文件 `fsync`。解锁/常录模式 (MLOG_MODE 1/2/3) 由 status 任务或上电时启动 mlog | `src/task/logger/task_logger.c` |
| **status** | 14 (`STATUS_THREAD_PRIORITY`) | 4096 | `sys_msleep(10)` 固定 10 ms 轮询 (100 Hz) | 系统状态监视线程: 订阅 `fms_output`/`ins_output`/`pilot_cmd`/`mission_data` MCN 话题,检测并打印飞行状态 (Disarm/Standby/Arm)、飞行模式、FMS 命令变化;状态切换时联动启停 mlog、发送 HOME_POSITION/MISSION_CURRENT;`PERIOD_EXECUTE` 每 100 ms 调 `pmu_poll_battery_status()` 采集电池 | `src/task/status/task_status.c` |
| **sim** | 18 (硬编码) | 2048 | `mavproxy_channel_loop(SIM)` 1 tick 事件 + hp_workqueue 4 ms 周期 work item;仅 HIL/SIH 编译 | 半实物/仿真链路线程: 注册 SIM 通道与 HEARTBEAT 1 Hz;work item `sim` 每 4 ms (250 Hz) 从 `control_output` 取控制量编码 HIL_ACTUATOR_CONTROLS 异步发出;收 HIL_SENSOR/HIL_GPS 解码后直接发布 `sensor_imu0`/`sensor_mag0`/`sensor_baro`/`sensor_gps` 供 INS 使用 | `src/task/sim/task_sim.c` |

## 2. 线程任务表 — 模块级线程 (task_manager 之外)

| 线程名 | 优先级 | 栈 (B) | 触发方式 / 执行频率 | 线程内容 | 源码 |
|---|---|---|---|---|---|
| **init** | 16 (`RT_THREAD_PRIORITY_MAX/2`) | 10240 | 一次性,创建后自行退出 | 系统初始化线程 `rt_init_thread_entry()`: `bsp_initialize()` (systime/电源/uMCN/工作队列/SDIO+FRAM/文件系统/参数/mavproxy/USB CDC/ADC/板载传感器驱动注册) → `task_manager_init()` 创建全部系统任务线程 → `bsp_post_initialize()` → `task_manager_start()` 启动任务,随后线程结束 | `src/startup.c`, `target/pixhawk/fmu-v5/board/board.c` |
| **mav_rx** | 10 (`MAVLINK_RX_THREAD_PRIORITY`) | 6144 (静态) | 各 MAVLink 通道设备 rx indicate 回调发事件,纯事件驱动 | MAVLink 接收分发线程 `mavproxy_rx_entry()`: 从通道设备读字节流喂给 mavlink 解析器,收完整帧后分发给该通道注册的 handler (任务命令、参数、任务协议、FTP、console 等) | `src/module/mavproxy/mavproxy_monitor.c` |
| **wq:hp_work** | 6 | 2048 | workqueue 派发器,挂起/唤醒执行 | 高优先级工作队列 `workqueue_executor()`: 按最小堆取到期 work item 执行,周期 item 执行后重新排队。承载: ms5611 气压测量 (3 ms 周期)、sim HIL actuator (4 ms)、加热器控制 (默认 10 ms) 等 | `src/module/workqueue/work_queue.c`, `workqueue_manager.c` |
| **wq:lp_work** | 19 | 2048 | 同上 | 低优先级工作队列: `devmq` work item 每 50 ms 把设备消息队列分发到已注册 handler (传感器的数据就绪处理);各类外设线程 (光流/测距) 延迟 1 s 启动的跳板也在此执行 | 同上 + `src/module/utils/devmq.c` |
| **msp** | 6 (`MSP_THREAD_PRIORITY`) | 8096 | 5 个端口的 rx indicate 事件,纯事件驱动;`FMT_USING_MSP` 时编译 | Betaflight MSP 协议服务线程 `msp_server_main()`: 打开所有注册的 MSP 端口,事件到达后解析/应答 MSP 请求 (供 Betaflight 配置器等调试) | `src/protocol/msp/msp.c` |

## 3. RTOS 内核级线程 (pixhawk fmu-v5 配置)

| 线程名 | 优先级 | 栈 (B) | 频率 | 内容 | 来源 |
|---|---|---|---|---|---|
| idle | 0 | 1024 | 持续 | 空闲线程,挂接 FMT CPU 占用统计钩子 (`src/module/system/statistic.c`) | RT-Thread 内核 |
| timer | 5 (`RT_TIMER_THREAD_PRIO`) | 1024 | 按软定时器到期 | 软定时器线程。注意 FMT 自身周期定时器 (sensor/vehicle/dronecan/mavproxy/ms4525/ak09916) 全部是 `HARD_TIMER`,在 tick 中断里执行,不占用此线程 | RT-Thread 内核 |
| tshell | 20 (finsh 默认) | 4096 | 事件驱动 | finsh/msh 控制台,承载 `fmt status`、`top`、`task`、param 等调试命令 | RT-Thread finsh 组件 |

## 4. workqueue 中的周期 work item (复用 §2 工作队列线程,不单独占线程)

| Work item | 所在队列 | 周期 | 内容 | 源码 |
|---|---|---|---|---|
| ms5611 | wq:hp_work | 3 ms (OSR=1024,随过采样档位 1~10 ms) | 发起温度/压力转换并读取,计算压强发布到 hal baro 设备,由 sensor 任务读取 | `src/driver/barometer/ms5611.c` |
| sim | wq:hp_work | 4 ms (250 Hz) | HIL_ACTUATOR_CONTROLS 发送 (仅仿真) | `src/task/sim/task_sim.c` |
| heater | 可配置队列 | 默认 10 ms (`HEATER_DEFAULT_CONTROL_PERIOD_MS`) | IMU 加热器 PWM 占空比控制 + 状态发布 | `src/module/heater/heater_controller.c` |
| devmq | wq:lp_work | 50 ms | 遍历设备节点,把 `rt_mq` 中积压的设备消息分发到已注册 handler | `src/module/utils/devmq.c` |
| 各外设 start_thread | wq:lp_work | 一次性 (schedule_time 1000 ms) | 延迟 1 s 启动 tfxx/fl04/mtf-01/up-tx 等接收线程,等总线稳定 | 对应驱动文件 |

## 5. 驱动任务表 — 驱动层线程

| 驱动 / 线程名 | 优先级 | 栈 (B) | 触发方式 / 执行频率 | 线程内容 | 源码 |
|---|---|---|---|---|---|
| ak09916 (ICM20948 内置磁力计) | 8 | 3072 | 硬定时器 2 ms 发事件 (500 Hz 触发,器件配置 100 Hz 出数) | 等待事件后 `probe_ak09916()` 读取磁力计原始数据写入 hal mag 设备,由 sensor 任务以 10 ms timetag 取走发布 | `src/driver/imu/icm20948.c` |
| ms4525 (空速计) | 7 | 2048 | 硬定时器 10 ms 发事件 (100 Hz) | 等待事件后 I2C 读差压+温度,扣去 `DIFF_PRESS_OFFSET`,发布 `sensor_airspeed` MCN 话题 | `src/driver/airspeed/ms4525.c` |
| tfxx (串口激光测距) | 7 | 2048 | UART rx indicate 事件或 10 ms 超时;lp_wq 延迟 1 s 启动 | 逐字节解析 TFxx 帧做信号强度/距离有效性检查,发布 `sensor_rangefinder` | `src/driver/range_finder/tfxx.c` |
| tofsense (测距) | 7 | 4096 | 阻塞读 UART 921600,数据到达即循环执行 | 解析 NOYTOF 帧,`dis_status>0` 判有效,发布 `sensor_rangefinder` | `src/driver/range_finder/tofsense.c` |
| nlink (LinkTrack UWB) | 7 | 4096 | UART 230400 rx indicate 事件,纯事件驱动 | 解析 LinkTrack 帧得到 UWB 定位/速度/航向,按 EXTPOS_* 参数裁剪后发布 `external_pos` 供 INS 融合 | `src/driver/uwb/nlink_linktrack/nlink_linktrack.c` |
| fl04 (PMW3901 光流+TOF) | 7 | 8192 | UART rx indicate 事件或 10 ms 超时;lp_wq 延迟 1 s 启动 | 解析帧发布 `sensor_optflow` (光流速度/质量) 与 `sensor_rangefinder` (TOF 高度) | `src/driver/vision_flow/pmw3901_fl04.c` |
| mtf-01 (光流+TOF) | 7 | 1536 | UART 115200 rx indicate 事件或 10 ms 超时;lp_wq 延迟 1 s 启动 | 同 fl04,轴向映射不同 (vx/vy 取反交换),发布 `sensor_optflow` + `sensor_rangefinder` | `src/driver/vision_flow/mtf_01.c` |
| up-tx (光流+TOF) | 7 | 2048 | UART 115200 rx indicate 事件或 10 ms 超时;lp_wq 延迟 1 s 启动 | 按积分光流 ÷ timespan 换算速度,置信度 ≥50 判有效,发布 `sensor_optflow` + `sensor_rangefinder` | `src/driver/vision_flow/up_tx.c` |
| gps_probe (UBX/NMEA/DroneCAN GPS) | 30 (`RT_THREAD_PRIORITY_MAX-2`) | 4096 | 一次性,探测完成后线程退出 | GPS 探测/配置线程: 自动扫描波特率、UBX 协议配置,成功后 `hal_gps_register()` 并退出;此后 GPS 数据不经线程,由串口 rx indicate 回调解析、经 hal gps 设备被 sensor 任务读取 | `src/driver/gps/gps_ubx.c`, `gps_nmea.c`, `gps_dronecan.c` |

**无线程驱动的数据通路** (供对照): ICM20689/BMI055 IMU (SPI 由 vehicle 线程 1 kHz 直接读)、IST8310 磁力计、MS5611 气压计 (hp_work 3 ms work item)、RGB LED/PMU/FRAM/MTD 等均为直接读/写或 work item,不创建线程。

## 6. target 板级特有线程 (非通用,按编译目标存在)

| 线程名 | 优先级 | 来源目标 | 内容 |
|---|---|---|---|
| sbus_rx / rc_thread | 见 `target/infineon/edge-e83/drivers/drv_rc.c` | infineon edge-e83 | SBUS/RC 接收解析 |
| usb_cdc / usb_ftpw | 见 `target/infineon/edge-e83/drivers/drv_usbd_cdc.c` | infineon edge-e83 | USB CDC 收发与 FTP 写线程 |
| phy | 见 `target/sieon/s1`、`target/yundrone/x1` 的 `drivers/drv_eth.c` | sieon/yundrone | 以太网 PHY 链路监测 |

## 7. 优先级总览 (高 → 低)

```
0   idle (内核)
3   vehicle   — 主算务: IMU 1kHz + INS 500Hz + FMS 250Hz + 控制 500Hz
4   sensor    — 低频传感器采集 200Hz
5   dronecan / timer(内核软定时器)
6   wq:hp_work / msp
7   fmtio 及多数驱动线程 (ms4525/tfxx/tofsense/nlink/fl04/mtf-01/up-tx)
8   ak09916 (ICM20948 内置磁力计)
10  mav_rx    — MAVLink 接收分发
11  mavgcs    — 地面站链路
12  mavobc    — OBC 链路
13  logger    — 日志落盘
14  status    — 状态监视 100Hz
16  init      — 启动初始化 (一次性)
18  sim       — 仿真链路 (仅 HIL/SIH)
19  wq:lp_work
20  tshell    — finsh 控制台
30  gps_probe — GPS 一次性探测
```

设计要点: 实时链路 (采集→估计→控制→输出) 集中在最高优先级的 **vehicle** 单线程内按 `PERIOD_EXECUTE` 分频执行,避免跨线程同步抖动;通信、日志、状态等非实时负载全部让位到更低优先级,并以事件驱动为主、空闲即挂起,因此多数线程不消耗周期性 CPU 时间。
