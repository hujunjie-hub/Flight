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

## 8. src/model — Simulink 模型生成代码 (算法核心层)

FMT 的算法层 (INS 估计 / FMS 决策 / Controller 控制 / Plant 被控对象模型) 用 Simulink 开发,源模型在 `ref/FMT-Model` (对应 `model/Controller`、`model/FMS`、`model/INS`、`model/Plant` 下的 .slx),经 Embedded Coder 代码生成后嵌入固件 `src/model/`。**固件不手写控制律,只写适配层**。

每个模型目录统一为两块:

- `lib/` — 生成代码,不可手改。`lib/<Model>.c` 为 step 函数与全部子系统实现,`<Model>_data.c` 为常量矩阵/参数初值,`<Model>_types.h` 为输入输出总线结构体 (Bus) 与枚举,另有 `<Model>_private.h`、`rtmodel.h`、`rtwtypes.h` 等支撑文件。生成代码导出 `*_EXPORT` 结构体 (内含 `.period` 执行周期 ms 与 `.model_info` 字符串),供任务层分频门控与状态显示。
- `<model>_interface.c` — 手写适配层: `*_interface_init()` 完成 MCN 话题 advertise/subscribe、参数链接 (`param_link_variable` 把 `*_PARAM` 结构体变量绑到 param 系统,如 CONTROL 组的 MC_ROLLRATE_P 等)、模型信息注册;`*_interface_step()` 解包输入总线→调生成代码 `*_step()`→发布输出总线。

**模型选择在编译期完成**: `target/<板卡>/config/model.py` 按 `vehicle_type × sim_mode` 组合从每类模型中**各选一个**组成一套 (如 Multicopter 实机 = `plant/multicopter` + `ins/cf_ins` + `fms/mc_fms` + `control/mc_controller`;HIL 仿真 = `plant/template_plant` + `ins/external_ins` + 其余同),由 `src/model/SConscript` 递归收集所选目录的 SConscript 编译,其余模型不进固件。

### 8.1 control/ — 控制器模型 (7 实 + 1 模板)

| 目录 | 适配器周期 (默认) | 内容 | 源码 |
|---|---|---|---|
| mc_controller | 2 ms (500 Hz) | 多旋翼全链路: 模式切换/手动指令混入 → 姿态环 (欧拉角 P → 角速率 PID+前馈) → **控制分配** `Control_Allocation` (rate PID 输出归一化 ±1 + 油门,按机架固定矩阵混控,三级饱和策略: 油门平移补偿 → roll/pitch 缩放 0.5~1.0 → `R_Remix` 削 yaw) → 16 通道 1000~2000 µs PWM (按 FMS status 分档: standby=1000/armed=1150/airborne=混控输出)。机架用编译期 variant `AIRFRAME` 1~8 选择 (X/+ 四轴、六轴、共轴六轴 12 路、八轴等,各一组混控矩阵),默认 1 = X 四轴 | `lib/Controller.c` (约 5000 行), 控制分配矩阵 `lib/Controller_data.c` |
| fw_controller | — | 固定翼: 姿态/滚转-俯仰指令处理 + 角速率 PID + **TECS** 总能量控制 (油门/俯仰分配,`FW_TECS_*` 参数),`FW_*` 参数组 | `lib/Controller.c` (约 1400 行) |
| vtol_controller | — | 垂起固定翼: MC/FW 双套控制律 + 倾转过渡状态机 | `lib/Controller.c` |
| adrc_controller | — | 自抗扰 (ADRC) 控制器变体,其余接口同 mc | `lib/Controller.c` |
| boat_controller / car_controller / submarine_controller | — | 船/车/潜航器控制器 (结构同模板,按载具运动学混控) | `lib/Controller.c` |
| template_controller | — | 空模板,自定义控制器起点 (只有 interface + 头文件) | `control_interface.c` |

输出统一为 `Control_Out_Bus` (16 通道 uint16 PWM 值) 经 `control_output` 话题发布,由 actuator 模块按 TOML 映射写到输出设备 (§1 vehicle 线程第 ④ 步)。

### 8.2 fms/ — 飞行管理系统模型 (5 实 + 1 模板)

| 目录 | 内容 |
|---|---|
| mc_fms | 多旋翼 FMS 状态机: 接收 RC (`pilot_cmd`)/GCS (`gcs_cmd`)/任务 (`mission_data`) 三路指令,仲裁 `FMS_Cmd` (PreArm/Arm/Disarm/Takeoff/Land/Return/Pause/SetHome) 与 `PilotMode` (Manual/Acro/Stabilize/Altitude/Position/Mission/Offboard),维护 `VehicleStatus` (Disarm/Standby/Arm) × `VehicleState` 二级状态,记录 home 点 (起飞点坐标/高度),失联/低电降级 (`FMS_Error`),生成期望姿态/位置/油门/航向指令 |
| fw_fms / vtol_fms | 固定翼 / 垂起固定翼 FMS: fw_fms 内含 **L1 侧偏航迹制导** (寻访航点/盘旋),vtol 含 MC/FW 过渡调度 |
| boat_fms / car_fms | 船 / 车 FMS |
| template_fms | 空模板 |

输出 `FMS_Out_Bus` (mode/status/state/home[3]/期望姿态角与角速率/位置期望/油门/航向 cmd_mask 等) 经 `fms_output` 话题发布,同时被 Controller、状态监视 (§1 status 线程)、MAVLink 上报 (§1 mavgcs HEARTBEAT 模式映射) 消费。`FMS_types.h` 中的枚举定义是全系统模式语义的权威来源。

### 8.3 ins/ — 导航估计器模型 (3 实 + 1 模板)

| 目录 | 内容 | 源码 |
|---|---|---|
| cf_ins | 互补滤波 INS (默认,轻量): IMU 机械编排 + 加计/磁/气压/GPS 互补融合,输出姿态四元数/欧拉角、NED 位置速度、纬经高 (含 lat_0/lon_0/alt_0 原点与 dx_dlat/dy_dlon 地理换算系数)、AGL 高度、传感器在位 status 位图。订阅 `sensor_imu0`/`sensor_mag0`/`sensor_baro`/`sensor_gps`/`sensor_rangefinder`/`sensor_optflow`/`sensor_airspeed`/`external_pos` 八路输入 | `lib/INS.c` |
| px4_ecl | PX4 ECL EKF2 移植: `ecl/EKF` 为 24 维误差状态 EKF 完整实现 (含 EKFGSF_yaw 备份航向),`lib/INS.c` 为 ecl 封装 wrapper,附带 `ecl/geo` (WGS84/磁偏角)、`mathlib`、`land_detector` (多旋翼/固定翼着地检测)、`hysteresis` | `lib/INS.c`, `ecl/EKF/*` |
| external_ins | HIL 专用"透传 INS": 订阅 `external_state` 话题 (仿真器经 MAVLink `FMT_EXTERNAL_STATE` 发来,§1 mavgcs 收包),直接变换为 `INS_Out_Bus` 发布,首帧记录 LLA 原点;与 `plant/template_plant` 配对用于 HIL | `ins_interface.c` |
| template_ins | 空模板 (period=10 ms,只 advertise `ins_output`) | `ins_interface.c` |

输出 `INS_Out_Bus` 经 `ins_output` 话题发布,消费者: FMS、Controller、status 线程、MAVLink 上报、GINS 换算等。`ins_interface.h` 亦定义 `external_pos` (外部位置观测,UWB/视觉接入,见 §5 nlink)。

### 8.4 plant/ — 载具动力学模型 (仅 SIH 仿真闭环参与实时运算)

| 目录 | 内容 |
|---|---|
| multicopter | 多旋翼动力学 (电机模型 + 刚体六自由度),SIH (software-in-hardware) 模式下由 vehicle 线程 `PERIOD_EXECUTE3` 调 `plant_step`,消费 `control_output` 产生仿真 IMU/气压/GPS 等传感器数据回馈 (task_vehicle.c:70) |
| fixwing / vtol | 固定翼 / 垂起固定翼动力学,额外携带 `rt_look*` 一维/二维查表运行时支持 |
| boat | 船动力学 |
| template_plant | 空模板,供 HIL (plant 在仿真器侧) 使用 |

非 SIH 构型中 plant 目录仍被编译进固件 (model.py 实机列表含 `plant/multicopter`),但其 step 由 `FMT_USING_SIH` 宏与 task_vehicle 的条件编译剔除,不消耗算力。

### 8.5 数据流总览

```
sensor_* (驱动/采集) ──▶ ins_step (2ms) ──▶ ins_output ──▶ fms_step (4ms) ──▶ fms_output
                ▲                                                        │
                │                                                        ▼
pilot_cmd/gcs_cmd/mission_data ────────────────────────────▶ control_step (2ms) ──▶ control_output
                                                                              │
                                                    (仅 SIH)                   ▼
                                            plant_step ◀────────────── actuator 输出设备
```

四步全部在 vehicle 单线程内按各自 `*_EXPORT.period` 分频串行执行 (§1),模型间无锁——同一线程天然互斥,这是 FMT "模型化算法 + 单线程实时核" 架构的基础。

## 9. src/task — 系统任务线程入口

每个子目录一个 (或一组) `TASK_EXPORT` 注册的系统任务 (§0 框架),目录按**载具功能域**划分;线程级细节 (优先级/栈/触发) 见 §1,此处按目录记录职责与关键文件。

| 目录 | 任务名 | 职责 | 关键文件 |
|---|---|---|---|
| vehicle/ | vehicle (优先级 3,最高业务) | 实时主算务: 1 kHz IMU 采集 → INS/FMS/Control 三模型分频步进 → actuator 输出,一条线程串起全部实时链路 (§1 vehicle 行) | `normal/task_vehicle.c` |
| sensor/ | sensor (4) | 低频传感器采集: mag 100 Hz/baro/GPS/airspeed 50 Hz,读 hal 设备校准后发布 (§1 sensor 行);HIL/SIH 下被编译剔除 | `task_sensor.c` |
| comm/ | mavgcs (11) + mavobc (12) | 双 MAVLink 链路: GCS 通道 (QGC) 与 OBC 通道 (机载计算机),周期上报 + 命令/参数/任务/FTP/RTCM 处理;`task_comm.c` 持有两任务的 TASK_EXPORT 与全部 MAVLink pack 回调 (`mavlink_msg_*_pack_func`,数据取自 MCN 总线) | `task_comm.c`, `mavgcs.c`, `mavobc.c` |
| status/ | status (14) | 系统状态监视 100 Hz: 订阅 fms/ins/pilot/mission 话题打印状态变化,联动 mlog 启停、电池周期采集 | `task_status.c` |
| logger/ | logger (13) | mlog 飞行数据二进制日志与 ulog 系统日志异步落 SD (§1 logger 行) | `task_logger.c` |
| dronecan/ | dronecan (5) | DroneCAN (UAVCAN v0) 节点: ESC/GPS 等 CAN 设备收发与 NodeStatus 广播 | `task_dronecan.c` |
| fmtio/ | fmtio (7) | FMU↔IO 协处理器串口协议线程 (RC 转发/PWM 输出/safety) | `task_fmtio.c` |
| sim/ | sim (18, 仅 HIL/SIH) | 仿真链路: HIL_ACTUATOR_CONTROLS 发送 + HIL_SENSOR/HIL_GPS 接收发布 | `task_sim.c` |

任务依赖关系在 TASK_EXPORT 描述符的 `.dependency` 字段声明 (如 mavobc 依赖 mavgcs),task_manager 按拓扑序 init。

## 10. src/module — 中间件模块库

模块按职能分四组。各目标板同样经 `target/<板卡>/config/module.py` 选择编译。

### 10.1 通信链路模块

| 目录 | 职责 | 关键文件 / 接口 |
|---|---|---|
| mavproxy/ | MAVLink 链路引擎 (§1 mavgcs/mav_rx 行的载体): 每通道周期消息调度 + 即时队列 + 设备抽象。`mavlink_param.c` 参数协议 (PARAM_REQUEST/SET,参数修改回调自动推送),`mavlink_mission.c` 任务上传/下载状态机,`mavlink_rtcm.c` RTCM 差分数据聚合注入 GNSS,`mavlink_console.c` SERIAL_CONTROL 式 mavlink shell,`mavlink_log.c` 飞行日志 MAVLink 下载协议,`mavproxy_cmd.c` COMMAND_LONG 执行与标定流程,`mavproxy_monitor.c` 接收分发线程,`mavproxy_dev.c` 通道设备抽象 (串口带宽估算/热切换),`px4_custom_mode.h` PX4 模式编码 | `mavproxy.c`: `mavproxy_register_msg()/send_msg()`,`MAVPROXY_GCS_CHAN=0/OBC_CHAN=1` |
| dronecan/ | DroneCAN 协议栈 (基于 canard 库): 节点生命周期、DNA (动态节点分配) | `dronecan.c`, `dronecan_dna.c` |
| fmtio/ | FMU↔IO 协处理器私有协议: 上行 (RC/safety/LED 状态)/下行 (PWM/DShot 输出) 包编解码与上传队列 | `protocol.c` 帧格式, `fmtio.c` 会话, `fmtio_uploader.c` |
| ftp/ | MAVLink FTP 文件传输 (read/write/list/remove 等 opcode 状态机),文件操作走 file_manager | `ftp_manager.c` |

### 10.2 系统 IO 总线模块 (sysio/) — 内外指令的唯一出入口

定义并承载系统级 MCN 总线 (§0),是"外部输入 → 模型"与"模型 → 输出设备"的适配枢纽:

| 文件 | 总线 | 内容 |
|---|---|---|
| pilot_cmd.c | `pilot_cmd` + `rc_channels`/`rc_trim_channels` | 遥控器指令采集: SBUS/PPM 设备读原始通道,按 TOML 配置的摇杆映射/模式通道/死区/trim 校准为 `Pilot_Cmd_Bus` (stick_* [-1,1] + mode);GCS 虚拟摇杆 (MANUAL_CONTROL) 在无 RC 数据时由此兜底 |
| gcs_cmd.c | `gcs_cmd` | 地面站命令/模式/心跳: `gcs_set_cmd()/gcs_set_mode()/gcs_cmd_heartbeat()`,mavobc/mavgcs 收到 MAVLink 命令后写入,FMS 模型消费 |
| auto_cmd.c | `auto_cmd` | offboard 自动指令总线: MAVLINK SET_POSITION_TARGET/SET_ATTITUDE_TARGET (mavobc) 或任务脚本写入带 cmd_mask 位图的多域指令 (位置/速度/加速度/姿态/角速率/油门/frame 坐标系),FMS 在 Mission/Offboard 模式消费 |
| mission_data.c | `mission_data` | 任务航点表 (seq/command/frame/x/y/z/param),MAVLink 任务协议与 FMS 之间的缓冲 |
| actuator_cmd.c | `mav_actuator_control` + 输出设备写入 | 控制分配输出落地: 读 `control_output` (§8.1),按 TOML actuator 映射表 (from control_out/rc_channels/mav_actuator_control → to pwm/dshot/fmtio 设备通道) 写 1000~2000 µs;HIL 下改发 HIL_ACTUATOR_CONTROLS |

### 10.3 传感器与标定模块

| 目录 | 职责 | 关键接口 |
|---|---|---|
| sensor/ | 传感器汇集枢纽 `sensor_hub`: 从 hal 设备 (注册表遍历) 读取各类传感器原始数据,做板级旋转/单位换算/标定补偿后按 timetag 门控发布 `sensor_imu0/imu1` (1 kHz)、`sensor_mag0` (100 Hz)、`sensor_baro`、`sensor_gps`、`sensor_airspeed` 等话题;`sensor_collect_high_freq()` 由 vehicle 线程调,`sensor_collect_low_freq()` 由 sensor 线程调 | `sensor_hub.c`, per-type `sensor_imu.c`/`sensor_mag.c`/... |
| calibration/ | 传感器标定数学库 (Simulink 生成): 六面加计标定与磁力计**椭球拟合** (`ellipsoid_fit_step` 递推最小二乘,实时渐进拟合软硬磁 9 参数),由 `cmd_calib` 命令与 MAVLink PREFLIGHT_CALIBRATION 触发,结果存 param | `calibration.c` 生成代码 + `calibration_param.c` 持久化 |
| heater/ | IMU 恒温控制: 目标温度 PI → 加热器 PWM work item (§4) | `heater_controller.c` |

### 10.4 基础服务模块

| 目录 | 职责 | 关键接口 / 备注 |
|---|---|---|
| ipc/ | **uMCN 发布/订阅总线** (进程内消息中间件,全系统数据骨架): 话题 advertise/subscribe/publish,订阅者可 poll/copy/挂回调 | `uMCN.c`;`MCN_DEFINE` 定义话题,`MCN_HUB()` 取句柄 |
| param/ | 参数系统: 参数表 `PARAM_GROUP_DEFINE` 分组声明、`param_link_variable` 绑定到模型参数结构体、W25Q/FRAM 持久化、修改回调;`rc_param.c` 遥控相关参数,`sys_param.c` 系统参数 | `param.c`;MAVLink 参数协议 (§10.1) 与 finsh `param` 命令共用 |
| config/ | **sysconfig.toml 各节解析器**: console/mavproxy/pilot-cmd/actuator/gnss/ethernet 六个 `*_config.c` 把 TOML 配置表解析成 C 结构供各模块 init 使用;新板卡配置只改 toml 不改代码 | `mavproxy_config.c` 等六个 |
| toml/ | TOML 解析器库 (第三方单文件实现 + util) | `toml.c` |
| log/ | 三层日志: `ulog` 系统文本日志 (分级 TAG,console 与文件双输出)、`mlog` 飞行数据二进制日志 (MCN 总线按 MLOG_BUS_DEFINE 录制,异步落盘 §1 logger)、`boot_log` 启动信息缓存 | `mlog.c`/`ulog.c`/`boot_log.c` |
| console/ | 控制台抽象: 多设备 (serial/USB/mavlink) 挂接、auto-switch 自动切换、`console_printf` 分流 | `console.c` |
| system/ | 系统服务: `systime` (tick/us/ms 时间接口 + 超时宏)、`statistic` (idle 钩子 CPU 占用率统计,`get_cpu_usage()` 供 SYS_STATUS) | `systime.c`/`statistic.c` |
| task_manager/ | 任务注册/依赖/启动框架 (§0) | `task_manager.c` |
| workqueue/ | 优先级工作队列 (§2/§4): 最小堆定时 work item,两个执行线程 | `work_queue.c`/`workqueue_manager.c` |
| pmu/ | 电源管理: PMU 芯片驱动 (电压/电流/余量采集→`bat_status` 话题,§1 status 线程 100 ms 轮询)、开机/重启控制 | `battery.c`/`power_manage.c` |
| file_manager/ | 文件操作封装 (供 FTP/日志/任务文件) + `yxml` 轻量 XML 解析器 | `file_manager.c` |
| filter/ | 数字滤波: Butterworth 二/三阶设计 (IMU 低通) | `butter.c` |
| math/ | 数学库: `quaternion` (四元数/欧拉互换)、`rotation` (旋转矩阵/轴角)、`conversion` (LLA↔NED、ECEF)、`light_matrix` (小型矩阵运算)、`ap_math` (杂项) | 全部无依赖 C 实现 |
| utils/ | 通用件: `ringbuffer`、侵入式链表 `list.h`、设备消息队列 `devmq` (§4) | |
| syscmd/ | finsh/msh 调试命令集 (18 个文件): `act`(执行器)/`adc`/`calib`(标定)/`param`/`mcn`(话题 echo)/`mlog`/`dshot`/`fmtio`/`boot_log`/`task`/`ps`/`top` 等,`optparse` 参数解析 | `FINSH_FUNCTION_EXPORT_ALIAS(__cmd_xxx)` 注册 |

## 11. src/protocol — 外设私有协议解析库

面向外接传感器/定位设备的**收方向协议解码器**,解码结果喂给对应驱动发布 MCN/hal 设备 (§5)。编译选择同样在 `target/<板卡>/config/protocol.py` 按需裁剪 (如 cuav 7_nano 默认不编 msp)。

| 目录 | 协议 | 内容 | 消费驱动 |
|---|---|---|---|
| ublox/ | u-blox UBX 二进制 (0xB5 0x62) | NAV 定位解算消息 (PVT/POSLLH/VELNED/SOL/DOP/TIMEUTC/SVINFO) + CFG 配置消息 (端口/消息率/NAV5 动态模型/SBAS),可解析可配置 | `src/driver/gps/gps_ubx.c` |
| nmea/ | NMEA-0183 ASCII | 通用语句状态机解析 + 国产组合导航 `$KSXT` 扩展句 (时间/经纬高/速度/姿态 21 字段一次输出) | `src/driver/gps/gps_nmea.c` |
| nlink/ | Nooploop 私有帧 | LinkTrack UWB 定位全套帧 (nodeframe0~6 标签/基站/AOA) + TOFSense 激光测距 + IOT 帧,公共校验工具 | `src/driver/uwb/nlink_linktrack/`, `src/driver/range_finder/tofsense.c` |
| msp/ | Betaflight MSP v1/v2 ('MM'/'X') | 仅传感器侧从机用途: 接收 `MSP2_SENSOR_RANGEFINDER`/`MSP2_SENSOR_OPTIC_FLOW`,可把刷 BF 的飞控当测距/光流枢纽 | `src/driver/vision_flow/pmw3901_l0x.c` (另有独立 msp 服务线程 §2,供 BF 配置器调试) |

四个协议加上 DroneCAN (§10.1) 和 MAVLink (§10.1),覆盖 FMT 的全部对外协议面;方向上 MAVLink/DroneCAN/MSP-server 是双向或主方,protocol/ 下四个是纯从方解码。对无 GNSS 场景,ublox/nmea (GNSS 回归)、nlink (UWB 室内定位 + 激光定高) 是给 INS 补外部观测的现成通道。

## 12. 本工程算法栈对接 FMT — 移植方案与清单 (驱动层冻结)

> 目标: 以 ref/FMT-Firmware (v1.1.4) 为基座, 把本工程算法栈整体对接过去——**KF-GINS 作 INS, 其解算输出即全系统的观测值** (经 `ins_output` 总线广播, FMS 与控制器只消费观测值、不各自滤波);**位置控制 MPC + PID 级联**: 位置外环 MPC (`position_mpc` 凝结 QP+SCA), 姿态/角速度内环 PID;**姿态误差在 SO(3) 上求解** (`so3` 库, e_b = Log(R_d^T·R), 不经欧拉奇异点)。
> **约束不变: FMT 驱动层 (src/driver, src/hal) 既有文件一行不动** (为本板新增传感器驱动属另一工作包, 见 §12.8), 适配落在 model/module 两层 + 构建配置, task 层除 vehicle 线程栈常量一行 (§12.2-6) 外零改动。
> 调度基准事实 (2026-10-03 对 ref 源码核实): vehicle 线程 (`src/task/vehicle/normal/task_vehicle.c`) 1 ms 硬定时器事件驱动 (优先级 3, 栈 6144), 每拍顺序 `sensor_collect_high_freq → pilot/gcs/mission collect → ins_step → fms_step → control_step → send_actuator_cmd`;各模型由 `PERIOD_EXECUTE3` 按 `*_model_info.period` (ms) 独立门控 (`fmt_model_info_t` = {period, info}, `src/fmt_def.h:119`)。本章取代原"KF-GINS 融合方案存档"。

### 12.1 对接总架构 — 数据流与驱动边界

```
sensor_hub (uMCN, FMT 原生不动)
 ├ sensor_imu0  1 kHz ─┐
 ├ sensor_gps   10 Hz  ├→ [gins_ins](新, period=1) → ins_output (KF-GINS 解 = 观测值) ─┬→ [mc_fms](原样) → fms_output
 ├ sensor_mag0         │                                                            │                      │
 └ sensor_baro        ─┘                                                            └──────────┐           ▼
                                                                                             [my_controller](新) → control_output → send_actuator_cmd
                                                                                             (MPC+PID+SO3+混控)   (actuator_cmd[16])  → actuator HAL (TOML, 原生 PWM/DShot)
```

四条 MCN 话题即 KF-GINS 与驱动层的全部契约 (字段名已对 `sensor_hub.h:85-121` 核实):

| MCN 话题 | KF-GINS 用途 | 契约字段 (单位) |
|---|---|---|
| `sensor_imu0` | 机械化推进 (1 kHz) | `gyr_B_radDs[3]`、`acc_B_mDs2[3]` (imu_data_t) |
| `sensor_gps` | 位置/速度观测 (10 Hz) | `lat/lon` int32 (1e-7°)、`height`、`velN/velE/velD`、`hAcc/vAcc/sAcc`、`fixType/numSV` (gps_data_t) |
| `sensor_mag0` | 磁航向观测 | `mag_B_gauss[3]` (mag_data_t, ×100 = µT) |
| `sensor_baro` | 气压高度观测 | `pressure_pa`、`altitude_m` (baro_data_t) |

丢拍兜底 (DR 丢拍计数、GNSS >50 ms stale 丢弃、对准播种、posguard 强制 reseed) 属桥接守卫逻辑, 随 §12.2 第 4 条重构迁入。

### 12.2 移植件 ①: `src/model/ins/gins_ins` — KF-GINS 作 INS (工作量主体)

新建, 手写 interface (template_ins 形态, 无生成代码);`target/*/config/model.py` Multicopter 分支 `ins/cf_ins` → `ins/gins_ins`。

1. **时序核心决策**: `ins_model_info.period = 1`。vehicle 线程每 1 ms tick 先跑 `sensor_collect_high_freq()` 再门控 ins_step, period=1 使 ins_step 与 `sensor_imu0` 发布 1:1 同拍——不丢拍、不加锁、不改 task_vehicle;period=2 会因 uMCN 只留最新值丢一半 IMU 样本。
2. **step 流程**: poll `sensor_imu0` → `GIEngine::addImuData()` + `newImuProcess()` (机械化+观测更新同入口);poll gps/mag/baro → `addGnssData/addMagData/addBaroData` (+ZUPT 检测内建);`getNavState()` 快照 → 填 `INS_Out_Bus` → `mcn_publish(ins_output)`。GIEngine 喂入口全 public (`gi_engine.h:57-105`), `gnssUpdate/magUpdate/EKFUpdate` 等为 private (`:277-328`)——只能走 add* 队列, 无旁路。时间轴统一 systime 毫秒 (IMU/GNSS/磁/气压同源);GNSS "测量时刻 vs 到达时刻" 之差 (PVT 固有 ~百 ms 滞后) 的精度取舍见 §12.5 时基行。
3. **INS_Out_Bus 填充** (字段全表 `cf_ins/lib/INS.h:173-203`): C_nb (DCM) 为主表示 → `phi/theta/psi` (wrap ±π, so3 DCM→euler) + `quat[4]` 兼容字段 (so3 DCM→quat, Shepperd 法);`p/q/r` 陀螺直通;`vn/ve/vd`;`lat/lon/alt` + `lat_0/lon_0/alt_0` + `dx_dlat/dy_dlon` (首帧记原点);`x_R/y_R/h_R` 本地 NED;`flag/status` 位图由引擎 ready + 对准状态 + 各观测计数合成 (位定义 `ins_interface.h:26-58`), 供 SYS_STATUS 上报与 pre-arm。
4. **桥接层去线程化**: 本工程 `middleware/gins/gins_bridge.cpp` 的 gins/ginsaux 两线程 (prio 9/10) 取消, 其守卫与编排逻辑 (DR 丢拍/gnss stale/播种/posguard/NOGNSS 状态机) 抽成无 RT-Thread 依赖的 `gins_step_feed()` 纯函数由 ins_step 调用;`gins_bridge_get_solution()` 关中断快照改为模型内直接读引擎。
5. **GINS 参数组** `PARAM_GROUP_DEFINE(GINS,...)`: 对准时长/杆臂/NOGNSS/ZUPT/MAG_ENABLE;磁偏角与轴向映射沿用运行期 `gins nav` 命令方案 (§12.4), 不依赖 param_calib 分区。
6. **栈与看门狗 (复查补)**: 本工程 gins 解算线程栈 8192, 注释明言 "Eigen 局部对象较大" (`gins_bridge.h:70`);vehicle 线程栈仅 6144 (task_vehicle.c TASK_EXPORT), 21 维 EKF 的 Eigen 栈临时装不下——需把 vehicle `stack_size` 提到 ≥16384, 这是 task 层唯一一行改动。gins_wdt 是 H7 IWDG1/DBGMCU 寄存器直写的板级代码, 而 FMT 全树无任何看门狗 (它是唯一安全兜底), 是否随迁随目标板定 (§12.8);迁则喂狗点从 gins 线程改为 ins_interface_step 末尾, 不新增 task 改动。

### 12.3 移植件 ②: `src/model/control/my_controller` — MPC 外环 + SO(3)/PID 内环

新建, 手写 interface (template_controller 形态), 替换 `control/mc_controller` (model.py 同步改, 保留一行切回的回退路径)。复用本工程 `middleware/control` 纯 C 核心, **弃用 *_gins 胶水四件** (那是无 FMS 工程的取数权宜, FMT 下由本 interface 组装输入)。级联与拍频 (control 模型 2 ms 步内分频, MPC 按自身周期门控):

| 级 | 入口 (本工程源) | 周期 | 输入 → 输出 |
|---|---|---|---|
| 位置外环 MPC | `mpc_pos_step` (`mpc_pos.h:130`, 凝结 QP+SCA, 逐轴双积分器 + LQR 终端权, 缺省 N=10/dt=0.05) | 50 ms (cfg.dt 与拍对齐, 算力紧可放宽) | p/v 当前 + p_ref/v_ref/a_ff → `a_cmd[3]` NED m/s² |
| 姿态内环 SO(3)+PID | `att_pid_step` (`att_pid.h:118`): `att_thrust_to_attitude` 构造期望姿态 (z_b_des=unit(g−a_des) → 倾斜锥限幅 → Lee 正交化) → SO(3) 姿态误差 e_b=Log(R_d^T·R) → 角度环 P → 角速度环 PID | 2 ms (500 Hz) | a_des + yaw_des + q_cur + omega_b → `thrust_n` + `alpha_b[3]` rad/s² |
| 控制分配 | `mixer_step` (`mixer.h:87`, quad-X 解析逆, clamp + 饱和计数) | 2 ms | thrust_n + alpha_b → `u_norm[4]` ∈[0,1] |
| 总线输出 | interface 尾部 | 2 ms | `u_norm` → `actuator_cmd[16]` |

**输入组装 (interface 层)** — 已对 FMS_Out_Bus 实际字段核实 (`mc_fms/lib/FMS_types.h:270-403`): **该总线没有 pos_sp/vel_sp/yaw_sp 字段**, 只有 `*_cmd` 系列 (旧版方案此处为误记, 本章更正):

| 内核输入 | 来源 | 备注 |
|---|---|---|
| p/v 当前 | `ins_output` x_R/y_R/h_R + vn/ve/vd | NED 约定一致 (z 向下为正) |
| v_ref | `fms_output` u_cmd/v_cmd/w_cmd | 字段注释为 "control frame" (`FMS_types.h:96`), 坐标系按 mc_controller 同款用法核对——mc_controller 自身即积分 u_cmd 作位置参考 (`Controller.c:537`);POSCTL/Hold 下 FMS 出速度期望, Hold 时取 hold 点作 p_ref |
| p_ref | `auto_cmd` 总线 x_cmd/y_cmd/z_cmd (`fms_interface.c:89-115`, Offboard/Mission 位置期望) | auto_cmd 消费方式与 mc_fms 一致 |
| a_ff | `fms_output` ax_cmd/ay_cmd/az_cmd | 前馈直通, MPC 内 u_0+a_ff 再 clamp |
| yaw_des | `psi_rate_cmd` 积分 (ψ̇=0 即航向保持) | FMS 无 yaw_sp, 积分方案保持 FMT 摇杆手感 |
| q_cur | gins_ins 的 C_nb → so3 DCM→quat | so3 库以 quat 为交换格式、DCM 为主表示 |
| omega_b | `ins_output` p/q/r (rad/s FRD) | 陀螺直通 |

**有效性门控 (复查补)**: 消费 `cmd_mask` (bit14-16 标 u/v/w_cmd 有效, bit6 标 psi_rate_cmd 有效, `FMS_types.h:131-135`), mask 未置位的轴回落 hold/上一有效值;`FMS_Out.reset` 置位时调 `mpc_pos_reset()` + `att_pid_reset()` 清积分器与 MPC 计划。

**输出映射**: `u_norm[4]`×1000+1000 → `actuator_cmd[16]` (uint16 µs;`Control_Out_Bus` = {timestamp, actuator_cmd[16]}, `Controller_types.h:230`)。PWM 设备直接用;**DShot 设备由 FMT actuator HAL 原生换算 (DSHOT_MIN 48/MAX 2047, `hal/actuator/actuator.h:34-35`, µs→48-2047 归一换算 `actuator.c:151-154`)——本工程 `dshot_output` 整模块不移植**, `send_actuator_cmd()` → actuator TOML 链路现成。输出分档按 `FMS_Out.status` (VehicleStatus: Disarm 停转 / Standby 1000 / Arm 1150 怠速 + 飞行混控) 复刻 mc_controller 行为。

**模式支持**: 第一版 MPC 系模式 (ctrl_mode POSCTL/ALTCTL/Offboard + state Mission/Takeoff/Land/Return/Hold);Manual/Acro/Stabilize 暂不支持 (回落 mc_controller 或后续补杆量→姿态期望映射)。ALTCTL 过渡: 高度轴先 w_cmd 积分作 h_ref 走 PID, MPC z 轴旁路, 全 MPC 化为后置项。

**验收与账本**: 纯核心已被 build_host 的 mpc_xcheck/so3_xcheck 对拍全过 (H/M ≤1e-12、闭环 1e-15、mixer 往返 ≤2e-16);新增整链对拍——录制 ins/fms 序列喂 my_controller, 对拍 `u_norm` 与 20 Hz MPC 计划序列 `u_pred`。机体参数占位 (mixer l/cq/J/f 界按 500g 级);**调参走 FMT PARAM 体系** (`PARAM_GROUP_DEFINE`, `param.h:269`;持久化 /sys/param.xml + MAVLink/QGC 在线改参), 不自造 `mix set`/`att set` 命令 (复查优化), `gins nav` 只留磁偏角/轴向映射运行期项。CPU/栈: MPC 20 Hz double QP + att/mixer 500 Hz 全落 vehicle 线程 (优先级 3, 栈按 §12.2-6 提至 16384;ctx 全静态无动态内存), `get_cpu_usage()` + 栈水位验证;超限先降 MPC 频率 (cfg.dt 对齐), 再议挪 hp_work work item。

### 12.4 移植件 ③: MODULE 层 — 算法内核随迁

| FMT 新增 | 来源 (本工程) | 内容与注意点 |
|---|---|---|
| `src/module/gins/` | `middleware/KF-GINS/src/{kf-gins,common}` + `middleware/gins/{kf_math.*,aligned_new.cpp,gins_config.h,gins_options.hpp}` | 固件清单: gi_engine.cpp / insmech.cpp / kf_gins_types.h + common 四头 (angle/earth/rotation/types) + kf_math (double EKF 内核, P 传播 ~100 Hz 节流) + aligned_new (C++17 对齐 new/delete, Eigen 定长成员 alignas16——缺席则对齐性静默退化为 8 字节);**Eigen 3.3.9 头随迁** (KF-GINS/ThirdParty/eigen-3.3.9, FMT 树无 Eigen)。构建参数与本工程 gins/SConscript 同: `-O3 -ffp-contract=fast` 只给 C++ 组, `KF_GINS_EMBEDDED` + `M_PI` 宏;关异常/RTTI 与 -std=c++17 落在 target rtconfig.py (§12.7-13, FMT 现默认 c++14 且未关, rtconfig.py:61-63)。gins_wdt 不在此组 (板级条件项, §12.2-6) |
| `src/module/math/so3/` | `middleware/control/so3/so3.{c,h}` | 纯数学库 (quat↔DCM↔euler、exp/log、`so3_att_error_dcm/quat`);gins_ins 与 my_controller 共用;与 KF-GINS 对拍已验证 (≤0.0008°)。**so3_gins.c 不迁** (取数点改到模型内部) |
| `src/module/syscmd/gins_cmd.c` | 新写, 参照本工程各 FinSH 命令 | `gins status` (ready/观测计数/协方差)、`gins nav` (磁偏角/轴向映射);机体参数不走命令、走 PARAM 组 (§12.3 验收段);复用 syscmd/optparse |

`module/math/quaternion.c` 保留但降级为纯 MAVLink 线上编码工具;其余 FMT 模块零改动。

### 12.5 不移植清单 (与移植清单同等重要)

| 本工程件 | 不迁原因 / FMT 侧替代 |
|---|---|
| `*_gins` 胶水四件 (mpc_pos_gins / att_pid_gins / so3_gins / mixer_cmd) | 取数/发布逻辑由 gins_ins 与 my_controller 的 interface 层重组装, 纯核心直迁 |
| `middleware/gins/gins_bridge.cpp` (线程+喂入编排) | 线程取消, 守卫逻辑抽 `gins_step_feed()` 进 gins_ins (§12.2 第 4 条);对外 API (init/get_solution/set_pause) 不再存在 |
| `middleware/control/dshot_output/` | FMT actuator HAL 原生支持 DShot, TOML 配协议即可 |
| `middleware/data/` (imu_data/gnss_data 环形缓冲) | sensor_hub uMCN 话题取代;ZUPT 所需 IMU 事件时间戳改从 imu_data_t.timestamp_ms 取 |
| `middleware/timebase/` (TIM2/PPS) | 基线 (方案 A): 时间戳统一 systime 毫秒, IMU/GNSS 相对一致性成立即可跑通, 但 GNSS PVT 到达滞后 ~百 ms 在动态下化为观测滞后;精度 (方案 B, 可选): PPS 输入捕获以**新增**驱动接入 gins_ins 恢复测量时刻, 不触碰既有驱动 (复查修订, 原文 "systime 即满足" 过于乐观) |
| `middleware/param_calib/` (W25Q64 分区) | 磁偏角/轴向映射运行期化改走 `gins nav` 命令 + GINS 参数组 |
| 规划中的 ctl_att/ctl_mpc 独立线程 (README 线程表) | 并入 FMT vehicle 线程分频执行, 不新增线程 |

### 12.6 TASK 层改动边界与 quat 兼容

task 层除 vehicle 线程栈常量一行 (§12.2-6, 6144→16384) 外 0 行: 调度本就由 `*_model_info.period` 门控, 喂引擎在 ins_step 内, comm/status/logger 消费的总线字段兼容 (mlog 按话题录制不换 ID)。**保留 `INS_Out_Bus.quat[4]`**——Simulink 生成类型, 改结构体 = 全模型重生成, 且 mc_fms 与 MAVLink 消费它;内部主表示 SO3 (DCM), quat 退化为两个边界的字节格式 (总线兼容字段 + MAVLink 线上编码)。消费点 (数据源变, 语义不变): task_comm ATTITUDE_QUATERNION、task_status HOME_POSITION、mavobc SET_ATTITUDE_TARGET 解码、mc_fms FMS.c 内部 quat 归一化转欧拉。

### 12.7 移植文件对照总表

| # | 本工程源 | FMT 目标 | 形态 |
|---|---|---|---|
| 1 | `KF-GINS/src/kf-gins/{gi_engine.cpp, insmech.cpp, kf_gins_types.h}` + `src/common/{angle,earth,rotation,types}.h` | `src/module/gins/` | 直拷 (KF_GINS_EMBEDDED) |
| 2 | `KF-GINS/ThirdParty/eigen-3.3.9/` (纯头) | `src/module/gins/eigen-3.3.9/` | 直拷, CPPPATH 引入 |
| 3 | `middleware/gins/{kf_math.*, aligned_new.cpp, gins_config.h, gins_options.hpp}` | `src/module/gins/` | 直拷 (gins_options 已无 RT-Thread 依赖) |
| 4 | `middleware/gins/gins_bridge.cpp` 守卫/编排段 | `src/model/ins/gins_ins/` (抽 `gins_step_feed()`) | **重构**: 去线程化、去关中断快照 |
| 5 | `middleware/control/so3/so3.{c,h}` | `src/module/math/so3/` | 直拷 |
| 6 | `middleware/control/position_mpc/mpc_pos.{c,h}` | `src/model/control/my_controller/lib/` | 直拷 (纯 C 核心) |
| 7 | `middleware/control/attitude_so3/att_pid.{c,h}` | `src/model/control/my_controller/lib/` | 直拷 (CPPPATH 加 module/math/so3) |
| 8 | `middleware/control/control_allocation/mixer.{c,h}` | `src/model/control/my_controller/lib/` | 直拷 |
| 9 | — (新写) | `src/model/ins/gins_ins/{ins_interface.c, SConscript}` | 手写, template_ins 形态, period=1 (CPPPATH 挂 module/gins + Eigen) |
| 10 | — (新写) | `src/model/control/my_controller/{control_interface.c, SConscript}` | 手写, template_controller 形态, period=2, 步内分频 |
| 11 | — (新写) | `src/module/syscmd/gins_cmd.c` | 手写 FinSH 命令 |
| 12 | — (编辑) | `target/*/config/model.py` | 改 2 行: `ins/cf_ins`→`ins/gins_ins`、`control/mc_controller`→`control/my_controller` |
| 13 | — (编辑) | `target/*/rtconfig.py` | CXXFLAGS `-std=c++14`→`-std=c++17` + 补 `-fno-exceptions -fno-rtti` (rtconfig.py:61-63;aligned_new 的 align_val_t 是 C++17 语义, c++14 编不过——复查补) |
| 14 | `middleware/gins/gins_wdt.c` | 条件迁 (仅路线 B) | H7 IWDG1 寄存器直写板级代码, FMT 全树无看门狗;喂狗点改 ins_interface_step (§12.2-6) |
| 15 | — (新写, 可选) | gins_ins / my_controller 的 `MLOG_BUS_DEFINE` 调参总线 | mlog 宏表机制, `cf_ins/ins_interface.c:99-107` 同型;录 P 对角/新息/u_pred 供整定 |

### 12.8 板级落地与硬件路线 (复查补, 原清单遗漏的整个维度)

**路线 A — SIH 仿真验证 (零板级工作, 第一步就做)**: `--sim=SIH` 构建 (`SConstruct:53-55` 注入 FMT_USING_SIH), `plant/multicopter` 的 plant_interface 把仿真的 imu/mag/baro/gps 直接发布进 sensor_hub (`plant_interface.c:145-214`), gins_ins/my_controller 对此无感——整条控制链闭环可在任何 FMT 支持的板子 (甚至 `target/qemu` 纯软件) 上先行验证, 不依赖真传感器、不冒系留风险。

**路线 B — 本工程 H7 板真机落地**: FMT `target/` 现有 amov/cuav/infineon/minimum/pixhawk/qemu/sieon/yundrone, 无本板。需**新增** target/<board> 板级包 + 四个传感器驱动: ADIS16505 (SPI)、BMM350 (I2C)、BMP585、UM982 (UART+PPS)。全部是新增文件、不触碰既有驱动, 但属"驱动层新增"工作包, 与本章 model/module 清单分开管理;本工程 `middleware/sensor` + `libraries/HAL_Drivers` 的驱动逻辑可作蓝本, 注册面从自有 data 缓冲改为 sensor_hub (`register_sensor_*`)。磁力计标定 (硬磁 25µT 等) 落点在驱动层或 gins_ins 内二选一, 实施时定。

**路线 C — Pixhawk 等现成板 (中期算法验证)**: 传感器全换, `gins_config` 先验 (ADIS 零偏 ~1900 deg/h、噪声参数) 必须按新 IMU 重整定;仅作算法验证, 不作长期路线。

可选扩展: `external_pos` 话题是 FMT 现成的外部位置观测通道 (nlink UWB/视觉);GIEngine 现无 addExtPos 接口, 接入需扩引擎——NOGNSS 播种之外的连续外部位置融合, 后置项。

### 12.9 实施顺序与验收 (复查修订)

1. `module/gins` + Eigen 迁入, rtconfig.py C++17/关异常 RTTI 改妥 (§12.7-13), SCons 构建打通 (主机 gcc 先编过) → `so3` 拷入
2. **SIH 闭环** (§12.8 路线 A): gins_ins + my_controller 进 SIH 构建, 仿真闭环先验证控制链与 MPC 收敛——对拍锚点前移, 不依赖真机传感器 (复查新增步骤)
3. gins_ins 真机骨架: 只接 sensor_imu0 跑机械化, `mcn echo ins_output` 看姿态积分 (period=1, 丢拍应为 0)
4. 接 GNSS/磁/气压观测 → 对准 ready → **对拍锚点**: FMT gins_ins 与本工程 middleware/gins 喂同一份录制数据 (build_host 工具), 输出逐位一致 (角度 wrap 表示除外)
5. my_controller 上线 (mc_controller 回退路径保留: model.py 一行切回) → mlog 录 `u_norm`/`u_pred` 整链对拍 → QGC 全消息回归
6. flag/status 填全 → SYS_STATUS / ATTITUDE_QUATERNION / GLOBAL_POSITION_INT → CPU 余量与栈水位 (§12.2-6 的 16384 预算) 确认
7. 上板: actuator TOML 配 DShot/PWM → 系留测试

## 13. QGC 地面站连接 — FMT 链路梳理与本工程落地方案

> 定位: 在**当前 RT-Thread 工程**上直连 QGC (导航验证阶段就有地面站: 态势/地图/曲线);实现逐条参照 FMT (事实见 §13.1-13.2)。走 FMT 基座时 task_comm 整链白拿 (§12.6), 本章装配层日后可平移——pack 是纯函数, 数据源从 gins 快照换成 ins_output 即可。

### 13.1 FMT 的 QGC 链路构成 (事实速查)

**三层**: MAVLink 库 (`src/lib/mavlink`, v2.0 vendored, common + fmt 自定义方言 FMT_* 12000-12003) → mavproxy 模块 (通道/设备/路由) → comm 任务线程。

**线程表** (`task_comm.c:907-927`, `fmt_def.h:34-36`):

| 线程 | 优先级 | 栈 | 职责 |
|---|---|---|---|
| mav_rx | 10 | 6144 | 全通道 RX: 串口 rx_indicate→事件唤醒, 逐字节 `mavlink_parse_char`, 按通道分离解析状态→handler 链表分发 (`mavproxy_monitor.c:61-90`) |
| mavgcs | 11 | 5120 | chan0 (GCS) 主循环 `mavproxy_channel_loop`, 1ms 定时器事件驱动下行 |
| mavobc | 12 | 2048 | chan1 (伴飞计算机), `dependency` 依赖 mavgcs |

**mavproxy 通道机制** (`mavproxy.c`): 每通道 = 1ms rt_timer + event + 立即消息环形队列 (20 深) + 周期消息表 (25 项, msgid+rate+pack 回调) + tx 锁 + 1024B 发送缓冲;`mavproxy_register_msg(chan, msgid, rate_hz, pack_cb, start)` 注册周期下行;`mavproxy_send_msg(sync)` 直发 (500ms 超时) 或入队。

**串口配置走 TOML 不在代码里** (`sysconfig.toml:20-41`): chan0=serial1@57600 + usbd0 (auto-switch, 插拔自动切), chan1=serial2@115200, rtcm=serial3;pixhawk TELEM1/2 = USART2/3 注册名 serial1/2 (`drv_usart.c:26-30`)。`mavproxy_init` 在 board.c `bsp_initialize` (param_init 之后) 调用。

**QGC 握手三板斧** (`mavgcs.c`): ① HEARTBEAT 1Hz——type=QUADROTOR、**autopilot=MAV_AUTOPILOT_PX4 (伪装)**、custom_mode 按 `px4_custom_mode.h` 编码 (MAIN_MODE<<16 | SUB_MODE<<24)、armed→SAFETY_ARMED+ACTIVE;② MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES → capabilities 声明 MAVLINK2|PARAM_FLOAT|MISSION_*|FTP、**flight_sw_version 伪装 PX4 v1.10.0** (源码注释原话 "cheat QGC");③ MAV_CMD_REQUEST_PROTOCOL_VERSION → version=200。sysid/compid 出自参数组 SYS.MAV_SYS_ID/MAV_COMP_ID (默认 1/1)。

### 13.2 FMT GCS 消息集

下行 (注册表 `mavgcs.c:519-543`):

| 消息 | 频率 | 数据源 |
|---|---|---|
| HEARTBEAT | 1 Hz | fms_output (status/state/mode) |
| SYS_STATUS | 1 Hz | ins status/flag 位图 + 电池 + `get_cpu_usage()` |
| SYSTEM_TIME / EXTENDED_SYS_STATE | 1 Hz | systime / fms landed_state |
| ATTITUDE_QUATERNION | 10 Hz | ins_output quat + p/q/r |
| LOCAL_POSITION_NED | 5 Hz | x_R/y_R/-h_R + vn/ve/vd |
| GLOBAL_POSITION_INT | 5 Hz | lat/lon/alt/h_R + vel + yaw |
| VFR_HUD / ALTITUDE | 5 Hz | 地速/航向/高度/爬升;ALTITUDE 汇集 AMSL/相对/AGL |
| GPS_RAW_INT | 5 Hz | sensor_gps 原始定位 |
| RC_CHANNELS | 5 Hz | rc_channels 话题 |
| STATUSTEXT | 事件 | 校准进度 (QGC `[cal]` 格式) 等 |

上行 (`handle_mavlink_message`, `mavgcs.c:156-507`): SET_MODE→`gcs_set_mode/cmd`;PARAM_REQUEST_READ/LIST/SET——**两段参数空间**: ~150 个 PX4 虚拟参数在前 (SYS_AUTOSTART=4001/CAL_*/RC*, 供 QGC 机架识别) + FMT 真参数按 index 偏移, 逐个下发按带宽自适应节流 (默认 ~20ms/个, 占链路 ≤30%);COMMAND_LONG (arm/disarm/takeoff/land/校准), COMMAND_INT DO_REPOSITION→单航点+Mission 模式;MISSION_* 状态机→文本任务文件 (`sysio/mission_data`);SERIAL_CONTROL→QGC 内嵌 MAVLink Console;FTP;LOG_*;MANUAL_CONTROL 虚拟摇杆 (真实 RC 失联 1s 才接管);GPS_RTCM_DATA→RTCM 转发。

### 13.3 本工程现状盘点 (已就绪 / 缺口)

| 件 | 状态 |
|---|---|
| MAVLink 库 | ✅ `middleware/protocol/mavlink` = 官方 v2.0 common 库, 与 FMT 同源同版 (均为 2020-08-18 vendored, 零漂移;仅去掉 fmt 方言)——FMT 的 GCS 消息全在 common.xml, **无方言缺口** |
| 协议适配层 | ✅ `mavlink_link.c/h`: feed 逐字节收 / attach handler (容量 8) / send_msg / 心跳 / v1 兼容 / 统计 / FinSH `mavlink`;单通道缓冲 ~0.8KB;sysid=1 / compid=AUTOPILOT1 |
| 业务消息 | ✅ `applications/mavgcs.c` (2026-10-03): 8 类消息 (见 §13.4 装配表), `gcs on` 会话启动 |
| 握手 | ✅ 同上: COMMAND_LONG(519/520) 应答 AUTOPILOT_VERSION (伪装 PX4 v1.10.0) + PROTOCOL_VERSION(200) + COMMAND_ACK |
| 串口 | ✅ 阶段 1 已落地 (2026-10-04) = USART2 独占数传电台 (PD5 TX/PD6 RX @115200, mavgcs 显式下发波特率, RX 从设备接管/恢复逻辑不变);USART1 仍为 console/数据链路;USART3 = ELRS CRSF (PD8/PD9 @420000, RC 遥控) |
| 数据源 | ✅ `gins_bridge_get_solution()` 关中断快照 (线程安全): latitude/longitude/altitude、vn/ve/vd、roll/pitch/yaw(deg)、ready/degraded/nognss + 各观测计数;`gnss_data_peek_latest()` 原始 UM982;`cpuload` CPU 遥测 |

### 13.4 落地方案 — 三阶段 + 一个新文件

**阶段 0 (零硬件, 当天可通)**: 复用 USART1/COM9——QGC 串口连接 COM9@460800 (MAVLink2);先 `vofa off`/关调试文本流, 残余文本混线时 QGC 解析器丢弃坏帧, 只影响观感。验收: QGC 连接绿、心跳 1Hz、姿态/位置随晃动变化。

**阶段 1 (正式数传口)**: ✅ 已落地 (2026-10-04)——定案 USART2 PD5/PD6 @115200 (数传电台, 波特率经 mavgcs 显式下发, 电台侧改参数时同步 `GCS_UART_BAUD`);CubeMX 重生成按回植清单走。带宽预算: 下行满配 ~1.4 KB/s ≈ 11 kbps, 富余一倍以上。

**阶段 2 (参数 + 控制)**: 上行控制指令**已先行落地 (2026-10-03)**——COMMAND_LONG 400 解锁/上锁 (走 quad_model 预检, 拒绝回 DENIED)、COMMAND_INT DO_REPOSITION 点地图定点 (LLA→NED, 相对当前位置钳位 h50/v10 m, `mpc_pos_gins_set_sp_lla`)、SET_MODE 接受 POSCTL、心跳如实上报 armed/POSCTL (见 mavgcs.c 头注"上行指令"节);剩余: PARAM_* 接 param_calib sys/nav 域 + MANUAL_CONTROL 虚拟摇杆 + Mission 航线。

**新增代码只有一个文件**: `applications/mavgcs.c` (本工程惯例: 协议层不占串口不开线程, 装配在 applications 层):
- 线程 "mavgcs", prio 12 (与 vofa 同级), 栈 4096 (FMT 5120, 我们消息少一档);
- 收: 串口 rx_indicate→信号量唤醒 (100ms 超时兜底) → 排空读 → `mavlink_link_feed()`;
- 发: 周期 timetag 门控 pack → `mavlink_link_send_msg` (sender 注入串口写; 阶段 0 走 app_out 共享互斥);
- attach: REQUEST_AUTOPILOT_CAPABILITIES / REQUEST_PROTOCOL_VERSION / GCS 心跳记录 (链路监控);
- 拷 FMT `px4_custom_mode.h` (纯枚举头) 供 custom_mode 编码;sysid/compid 后续参数化。

**消息装配表 (数据源 = gins 快照 + 原始 GNSS)**:

| 消息 | 频率 | 装配 (本工程字段) |
|---|---|---|
| HEARTBEAT | 1 Hz | QUADROTOR + 伪装 PX4 + custom_mode=POSCTL;armed/ACTIVE 如实反映 quad_model 状态机 (2026-10-03 升级) |
| SYS_STATUS | 1 Hz | imu/gnss/mag/baro_cnt>0 → 传感器在位/健康位;cpuload;无电池→0 (QGC 电池区显示 0, 台架可接受) |
| ATTITUDE_QUATERNION | 10 Hz | roll/pitch/yaw (deg→rad) → `so3_euler_to_quat` (复用 control/so3 纯 C) |
| LOCAL_POSITION_NED | 5 Hz | latlon→NED 球面近似 (逻辑同 `mpc_pos_gins.c:71-79`, 抽公共) + vn/ve/vd |
| GLOBAL_POSITION_INT | 5 Hz | latitude/longitude/altitude + vn/ve/vd + yaw |
| GPS_RAW_INT | 5 Hz | `gnss_data_peek_latest`: lat/lon/alt/hAcc/sAcc/numSV;fix 映射 NMEA quality→GPS_FIX_TYPE (3D=3, RTK float=5, RTK fixed=6) |
| VFR_HUD | 5 Hz | √(vn²+ve²) 地速, yaw, altitude, -vd 爬升率 |
| STATUSTEXT | 事件 | ready/degraded/nognss 进出/reseed/协方差告警 (gins_solution 计数边沿触发) |

**与 FMT 的有意差异**: 不做 mavgcs/mavobc 双通道与 fmt 方言 (单 GCS 通道, MAVLINK_COMM_NUM_BUFFERS=1);不做 150 个 PX4 虚拟参数表 (监控用途 QGC 不强制, 机架识别不可用不影响地图/曲线);MISSION/SERIAL_CONTROL/FTP/RTCM 后置 (FinSH console 已够调试, 任务规划对导航验证非必需);SYSTEM_TIME/EXTENDED_SYS_STATE/ALTITUDE 裁剪 (信息已被其余消息覆盖)。

### 13.5 实施顺序

1. ~~阶段 0: mavgcs.c 上 USART1 (COM9)~~ **已落地 (2026-10-03)**: `applications/mavgcs.c` 单文件——"mavgcs" 线程 (prio 12, 栈 4096) + RX 从 finsh 接管/恢复 (`gcs on/off`, 保存并归还 `dev->rx_indicate`) + 心跳/握手 (伪装 PX4 v1.10.0) + 全部 8 类消息 (HEARTBEAT+SYS_STATUS 1Hz / ATTITUDE_QUATERNION 10Hz / LOCAL+GLOBAL+VFR_HUD+GPS_RAW_INT 5Hz / STATUSTEXT 边沿事件);双构建 0 告警。剩余: 上板接 QGC 实测 (COM9@460800, 先 `vofa log off`)
2. ~~补全消息表~~ 已随 1 一起交付;latlon→NED 抽公共 (现随 mavgcs 内联, 球面公式同 mpc_pos_gins) 与 FinSH 丢帧率核对留待上板
3. ~~阶段 1 数传 UART 板级启用~~ **板级已落地 (2026-10-04, USART2 PD5/PD6)**: 剩余数传电台硬件实测
4. 阶段 2 参数协议 (param_calib sys/nav 域) + 控制链上线后补 SET_MODE/arm
5. (远期) FMT 基座路线成行后本章装配层退役, 由 task_comm 白拿 (§12.6)

**QGC 点地图→定点飞行 打通任务清单** (2026-10-03 备注于此; 状态随进度更新, 未完成项的落地步骤附表后):

| # | 事项 | 依赖 | 状态 |
|---|---|---|---|
| 1 | 电机输出硬件落地 (dshot 引脚/TIM/DMA 从占位变实际) | 无 | **未完成** (占位 TIM4+PB6-9+DMA1_S5, 见 dshot_hw.h) |
| 2 | 机体参数辨识与整定 (质量/惯量/静推力界/姿态增益) | #1 | **未完成** (500g 级占位值, 从未上机) |
| 3 | QGC 上行指令: DO_REPOSITION 定点 (LLA→NED, h50/v10 m 钳位) + 400 arm/disarm (预检拒 DENIED) + SET_MODE (POSCTL) + 心跳如实回显 armed/模式 | 无 | **已完成** (2026-10-03, 双构建 0 告警): QGC 地图 Go-To → MPC 定点全链, dry-run 可验 |
| 4 | RC 遥控 (手动接管安全底线): CRSF 硬件接收 + QGC 虚拟摇杆源 + 摇杆叠加/kill 开关/失联保护 | 无 (软件) / #1#2 (真飞) | **已完成 (软件链, 2026-10-03, 双构建 0 告警)**: crsf 协议层 + rc_data 双源快照 + 叠加/kill/失联保护全链; 残留: CRSF 硬件已启用 (2026-10-04, USART3 PD8/PD9 @420000, BSP_USING_UART3 + RX DMA ping 环), 剩余接收机接线实测 |
| 5 | RTK 前提: UM982 RTK 固定解 → KF-GINS cm 级位置 (NOGNSS 位置开环不可定点) | QGC 侧 NTRIP + UM982 口配置 | **已完成 (转发链 + 状态引脚, 2026-10-03, 双构建 0 告警)**: GPS_RTCM_DATA 重组 → UART4 回注 + `gcs rtcm` 开关/计数; **RTK 状态引脚 PC0** (GPIO 输入, 固定解=高; `gnss_data_rtk_pin_high()` + `gnssdata` 命令显示, 与 NMEA quality 交叉校验; .ioc/xlsx 已同步); 残留: UM982 口配置 RTCM3 (步骤 3) 与 NTRIP 实测 (步骤 4) |

**#1 输出硬件 — 未完成, 落地步骤**: ① xlsx 引脚表给 4 路电机输出分配 TIM 通道 (占位 TIM4 CH1-4@PB6-9 须按实际空闲复核); ② dshot_hw.h 头注四步清单改硬件定义 (引脚/AF/TIM/DMA/DMAMUX); ③ 拆桨台架 `quad power on` 后 `dshot` 命令逐通道验帧 (示波器/单电机); ④ u_norm→电机映射与实际桨向对表 (m0 FR CW/m1 FL CCW/m2 RR CCW/m3 RL CW)。

**#2 机体参数 — 未完成, 落地步骤**: ① 称重 + 摆法/天平法测 m/J, 静推力台架测单电机 f(u) 得 f_min/f_max; ② `mix set l|cq|j|fmin|fmax` 现场整定 (占位 l=0.11/cq=0.015); ③ att 增益保守起步悬停整定 (`att set`, 权威清单见根 README 调参清单章); ④ 增益持久化接 param_part ctrl 分区 (已预留)。

**#4 RC 遥控 — 已完成 (软件链 2026-10-03), 落地步骤** (步骤 1-4 已交付, 步骤 5 硬件后置):
1. `middleware/protocol/crsf/`: 纯 CRSF 解析层 (帧 sync 0xC8/len/type/CRC8 poly 0xD5; RC_CHANNELS_PACKED 0x16 = 16ch×11bit, 值域 172..1811 中点 992 → 归一化 [-1,1]), 仿 nmea/mavlink 约定不占串口不开线程;
2. `middleware/data/rc_data.c/h`: 源无关遥控快照 (4 摇杆 + ch5-8 开关 + 源标签 + 时间戳), publish/get/age 关中断快照 (双发布者并发安全); CRSF UART 接收线程 "rcrx" (占位 RC_CRSF_DEV="uart3"@420k 8N1, 设备不存在则告警不启动); FinSH `rc`;
3. mavgcs: MANUAL_CONTROL (QGC 虚拟摇杆) → rc_data 发布 (双源并存以最新到达为准); RC_CHANNELS 5Hz 下行 (QGC 摇杆页回显);
4. quad_model ARM 期 20Hz 摇杆叠加 (POSCTL 语义): 死区 0.08 外杆量 → v_ref (水平 5 m/s / 爬升 2 m/s NED) + 滑设定点 (p_ref = 当前位姿 + v_ref×0.5s 视界, 复用 h50/v10 钳位), yaw 杆 → 航向速率 90°/s; 全中位 → 复捕当前位置保持; **ch5 kill → 立即 disarm**; RC 曾在线后失联 >3s 且 ARM → 失效保护自动 disarm;
5. 硬件落地 (后置): xlsx 给接收机分 UART (USART2 不可用, PA2=PPS), CubeMX 重生成 + .config BSP_USING_UARTx。

**#5 RTK — 已完成 (转发链 2026-10-03), 实施步骤** (步骤 1-2 已交付, 3-4 板侧配置与实测):
1. mavgcs: GPS_RTCM_DATA (msgid 126) 分片重组 (frag_id 0-3, flags bit0=末片) → 整包写 UART4 (与 UM982 NMEA 同口双向; RTCM 1-5 kB/s @460800 带宽富余);
2. FinSH `gcs rtcm on|off` (默认 on) + 转发字节/包计数入 `gcs` 状态;
3. UM982 侧 (Unicore 工具配置, 非固件职责): 对应 COM 口使能 RTCM3 输入;
3b. **RTK 状态引脚 (已落地 2026-10-03)**: UM982 RTK 指示脚 → STM32 **PC0** (GPIO 输入, 固定解=高电平; 注意 H723 的 PC0 无任何 UART 复用, 仅 ADC/GPIO —— CubeMX DB 逐脚核实, 作状态脚恰合适); 读取: `gnss_data_rtk_pin_high()` / `gnssdata` 命令 rtk pin 行 / `gnss_data_status.rtk_pin`, 与 NMEA GGA quality 交叉校验 (不一致优先怀疑接线/模块配置); .ioc PC0.GPIO_Label=UM982_RTK;
4. 验收: QGC Application Settings 配 NTRIP 源 → `gcs` 看 rtcm 计数增长 → GPS_RAW_INT fix_type 5→6 (RTK fixed) → gins ready → 定点前提闭环。
