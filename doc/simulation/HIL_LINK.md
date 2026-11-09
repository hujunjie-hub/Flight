# Flight 与 FMT-SIM 联机仿真

发布版 ABI 锁默认开启。需要用当前源码契约联机调试时，Host SIL 启动命令可加 `-SkipAbiLock`。板载串口 HIL 可在 `menuconfig` 的 Flight application tasks 中开启 `FLIGHT_SKIP_ABI_LOCK` 后重编。手动关闭后跳过发布版 ABI 锁校验和 `FCU_BUILD_INFO` 的 ABI 指纹上报。其他遥测、线协议和模块间契约校验照常运行。真机构建禁用 HIL 时会启用发布版 ABI 锁。

| 路径 | 数据流 | 本轮证据 |
|---|---|---|
| FMT-SIM SIH | 上游发送 MAVLink `HIL_STATE`，FMT-SIM 更新机体位姿 | 原安装的回环端口 14553 已验证画面跟随。本地位置为毫米，垂直分量为高度向上 |
| FMT-SIM 原生 HIL | FMT-SIM 发送 `FMT_EXTERNAL_STATE`，Host SIL 回传守卫已提交的 `HIL_ACTUATOR_CONTROLS` | 隔离的 Ground 场景 14653 已双向联通，记录了真实机体离地和保护动作 |
| Flight 标准串口 HIL | UDP 字节流经串口桥交给 `task_sim`，输入为 `HIL_SENSOR` 和 `HIL_GPS` | 编解码和桥接用例已通过，板端 UART 另行验证 |

## 1. 标准串口 HIL 数据流

| 方向 | 报文 | 进入/离开的层 | 说明 |
|---|---|---|---|
| 仿真 → 固件 | `HIL_SENSOR` | M2 标准样本出口 | `sensor_imu_inject()` 注入 IMU；气压计走 `runtime_submit_measurement()` |
| 仿真 → 固件 | `HIL_GPS` | M2 标准样本 | `runtime_submit_measurement()`，`PRODUCT_MEASUREMENT_GNSS` |
| 固件 → 仿真 | `HIL_ACTUATOR_CONTROLS` | M6 `ActuatorDemand` | 用 M7 的推力上下限归一化到 0..1，**不是** M7 已提交的 DShot 帧 |
| 固件 → 仿真 | `HEARTBEAT` | 链路保活 | 1 Hz，`MAV_TYPE_QUADROTOR` |

必须保持的边界（spec 01 PLANT、spec 02 §4.9、spec 05 §6.2）：

- **不写**导航状态、估计器、Permit 或执行器帧：M3/M7 的唯一 Owner 不变；
- 仿真时间**不作为**样本时戳。桥接有数毫秒抖动，样本用本地单调时钟打戳，
  并声明 50 µs 的有界不确定度（`header_valid()` 要求 ≤ 100 µs）；
- `PlantTruth` 与仿真真值不进入估计器；M2 的校准/质量元数据照常携带；
- 输出策略仍是 `output_policy_id = 1`（BENCH_INHIBIT）：物理输出保持禁止，
  **HIL 不会让电机转**，推力只送到仿真器。

## 2. 固件侧开关

`applications/Kconfig`：`CONFIG_FLIGHT_SIM_LINK`（`.config` / `rtconfig.h`
里对应 `FLIGHT_SIM_LINK`）。打开后：

1. `task_sim` 加入任务表（优先级 17，周期 1 ms，栈 4096 B）；
2. `runtime_initialize()` 调用 `telemetry_link_release_radio()`，M1 不再打开
   共享的数传 UART，ELRS UART 继续独立收帧；
3. `task_communication` 停止原生 GNSS/气压/磁力计采集，避免真实与仿真测量混入同一估计器。

`sensor_device/telemetry/sim/hil_serial.c` 通过 RT-Thread 独占 HIL UART。
`middleware/communication/sim/hil_link.c` 管理有界收包、MAVLink 帧收发和统计。
`task_sim.c` 负责标准样本注入和虚拟控制输出。

默认串口取自板级 `TELEMETRY_UART_NAME`，当前为 `uart2`。
可用 `-DFLIGHT_SIM_LINK_DEVICE` 覆盖设备名。波特率在 `hil_serial.c` 中设置。

```c
#define FLIGHT_SIM_LINK_BAUD   921600u
```

`uart2` 是首版载体唯一引到外部连接器的空闲口（uart1 控制台、uart3 RC、
uart4 GNSS）。**2026-11-05 已由工程 Owner 确认该口可被 HIL 独占**，因此默认值
不变；若后续硬件改版另有空闲口，改一行宏即可，不必动协议代码。

任务栈为 4096 B，包含串口收发、帧处理、样本注入及运行时锁调用。目标实测栈余量见发布门禁。
**115200 不够**：250 Hz `HIL_SENSOR`（64 B）+ 250 Hz
`HIL_ACTUATOR_CONTROLS`（81 B）≈ 58 kB/s，115200 只能到约 50 Hz。因此桥接与
固件统一用 921600。

## 3. 标准串口 HIL 桥接

将串口桥用于能提供标准 `HIL_SENSOR` 和 `HIL_GPS` 的 UDP 对端。桥按字节转发，依赖
`pyserial`。以下串口和端口用于说明配置，实际运行以所用对端为准。

```powershell
python middleware/communication/sim/fmt_sim_udp_bridge.py --port COM7 --udp-peer 127.0.0.1:14560
```

串口默认波特率为 `921600`。诊断参数包括 `--udp-bind`、`--stats`、`--hexdump`
和 `--self-test`。`--sim-root` 从本机 FMT-SIM 配置读取 SIH 位姿端点，可供上游
`HIL_STATE` 字节透传。

## 4. Host SIL 与原生 HIL 设置

本轮使用隔离的 FMT-SIM 实例。它以 `/Game/Maps/MainMenu` 启动，读取
`FMT_SIM/Content/Settings/settings.jason` 后进入 `Ground`。测试机体为
`Multicopter`，设置 `SimMode=HIL`，`MavproxyFCS` 使用 UDP 回环地址
`127.0.0.1:14653` 和 `Autobound=true`。

```powershell
# Flight 根目录。使用已配置 Ground/HIL 端口的隔离实例
$sim = (Resolve-Path build/validation/sim/fmt_release/FMT_SIM/Binaries/Win64/FMT_SIM.exe).Path
pwsh tests/simulation/run_sil.ps1 -FmtSimExe $sim -FmtHilPort 14653 -FmtScenario Mission
```

`-FmtSimExe` 负责启动并清理该隔离进程。低负载 D3D11 软件渲染配置降低本机 GPU 阻塞对状态发布的影响。此主机的 Microsoft Basic Render Driver 枚举为适配器 1；其他 PC 用 `-FmtGraphicsAdapter` 指定对应编号，并在 `fmt_sim_engine.log` 确认选中的适配器。保留现有实例时可省略 `-FmtSimExe`，连接其原生 HIL 端口。

Host 从原生状态帧生成 IMU、磁、气压与 GNSS 样本，导航由 KF-GINS 输出。合成 GNSS 使用同一局部 NED 帧的位置和速度，按 10 Hz 送入观测入口。Home 落点误差由仿真真值独立验收。
`FMT_EXTERNAL_STATE` 为 MAVLink v2 消息 12001，CRC_EXTRA 为 168。观测到的
载荷为 85 字节，协议标称 88 字节，MAVLink v2 裁掉了末尾零字节。
该安装版输出的时间戳为零。对齐和飞行按有效状态批次推进，每批约 10 ms
机体时间。这个步长由位置和速度的实测关系估算，采样间仍使用合成 IMU。
状态帧等待超出 350 ms 时，Host 停止场景并发送虚拟停机帧。

四路上行值只取自有效的 M7 提交回执，核对当前许可与回执 epoch。起飞获准后
才将输出发往外部机体，按 FMT 旋翼顺序映射到 `[-1,1]`。`-FmtHover`
为虚拟机体的归一化悬停指令标定值。H723 使用自己的构建策略与设备驱动。
双航点场景在地面提交任务，起飞并悬停后执行任务。两个航点完成后，脚本立即
请求返航。FMS 在位置保持许可和新鲜控制能力成立后，用飞控估计判断速度小于
0.15 m/s、加速度小于 0.4 m/s² 连续 1 秒，再切换返航。心跳按 PC 单调时钟发送，
350 ms 状态等待仍由 Host 超时判据控制。

商用前复测把通信验收与飞行逻辑复测分开。默认 350 ms 用于严格链路判据。
仅复查 FMT-SIM 飞行逻辑时可显式指定 `-FmtStateTimeoutMs 650`，报告须保留
使用的超时与最长等待。这个参数仅属于 Windows Host 仿真。

## 5. 诊断

### QGroundControl 联机

**H723 台架。** 默认 GNU Arm Release 用 USART2 发送 MAVLink 2，设备端为 115200 baud、8N1。将数传或 USB 转串口接到 USART2，在 QGroundControl 的应用设置 → Comm Links 添加 Serial 链路，选择实际 COM 号并设为 115200。系统 ID 和组件 ID 均为 1。板端可在控制台执行 `link_status` 查看收发计数。

板载链路提供标准心跳、系统状态、有效姿态、本地 NED 位置和原始 GNSS，以及版本和公开参数读取。没有有效 GNSS 时报告无定位。板载参数仅包含 Profile、契约版本和台架状态。参数写入、任务修改和远程飞行命令受 `BENCH_INHIBIT` 与远程鉴权策略限制。板端当前没有融合全球位置或 Home 遥测，QGroundControl 的实机地图和返航点显示不在本次范围内。USART2 在启用板载 HIL 时转为 921600 baud 的 HIL 专用链路。

**本机台架。** 保持 QGroundControl 的 UDP 14550 监听，从 Flight 根目录运行。Windows Host 构建需要现有 MinGW 工具链和 Python 的 PyYAML。

使用 MAVSDK 或同时连接多个客户端时，可采用 [MAVLink 路由配置](MAVSDK.md)。

```powershell
pwsh -File tests/simulation/run_sil.ps1 -OutDir build/validation/qgc_live -GroundPort 14550
```

该模式以静态传感器样本运行 Flight Host，通过本机回环向地面站发送状态。地图坐标用于显示测试，固定场景下飞行命令返回不支持。按 Ctrl+C 退出。

**FMT-SIM 动态仿真。** 使用已配置独立 Ground/HIL 场景和 14653 端口的 FMT-SIM 实例。

```powershell
$sim = (Resolve-Path build/validation/sim/fmt_release/FMT_SIM/Binaries/Win64/FMT_SIM.exe).Path
pwsh -File tests/simulation/run_sil.ps1 -OutDir build/validation/qgc -FmtSimExe $sim -FmtHilPort 14653 -GroundPort 14550 -FmtStateTimeoutMs 1000
```

Flight Host 通过本机回环接收地面站报文，由 FMS 验证请求，再由制导、控制和执行器守卫驱动 FMT-SIM。地图位置、任务坐标和 Home 使用同一个仿真坐标映射。动作完成回执由 FMS 生成。心跳和上传重试按墙钟计时，导航和飞行时限按仿真时间计时。低帧率时，地面站动作可能比仿真时间显示的持续时间长。按 Ctrl+C 结束联机仿真。

1. 将 [Flight SIL Actions.json](../../tests/simulation/qgc/Flight%20SIL%20Actions.json) 复制到 QGroundControl 保存目录的 `MavlinkActions`，在 Settings → Fly View → MAVLink Actions 的 Fly View Actions 中选中它。飞行视图的 **Actions** 菜单提供仿真专用动作。QGroundControl 仍将载具识别为 Generic。
2. 在 **Plan** 中打开 [自动降落计划](../../tests/simulation/qgc/Flight%20SIL%20Auto%20Land.plan)或[自动返航计划](../../tests/simulation/qgc/Flight%20SIL%20Auto%20RTL.plan)，等待导航收敛后上传。在 **Fly → Actions** 选择 **SIL 开始任务**。含起飞项的计划从起飞点 1 m 内的地面启动。末项为降落或返航时，FMS 确认接地后由 SIL 请求上锁。任务由 FMS 原子提交，上传完成后可下载比对。
3. [仅航点计划](../../tests/simulation/qgc/Flight%20SIL%20Two%20Waypoints.plan)沿用手动分阶段流程。先在 **Actions** 解锁、起飞至 2 m，待悬停后开始任务。暂停、恢复和终止需等待上一模式的控制提交恢复。Mission 接收最多 32 个相对高度航点，高度为大于 0 且不超过 10 m，坐标限制在场景原点 100 m 内。Plan 支持清空任务。

**日志。** 在 **Analyze → Onboard Logs** 可刷新、选择和下载仿真的最新日志。日志保留最近 2048 帧，下载使用当次列表查询生成的快照。QGroundControl 5.1.5 给 Generic 载具的下载文件加 `.bin` 后缀。把下载文件另存为 `.tlog` 后，可以在 **Analyze → Log Viewer → Open .tlog** 打开。Erase All 只清空该仿真实例内存中的记录，要求载具处于上锁状态。

`ground_parameters.json` 保存五份 YAML 的完整字段路径、值与 MAVLink 名称映射。109 个参数（70 控制器 + 39 校准）供地面站读回，修改源 YAML 后重建 Host 生效。`SIM_TLM_HZ` 可在线设置仿真导航遥测频率，范围为 1–10 Hz。原版 QGroundControl 的 Generic 参数配置页受插件限制，可在 MAVLink Inspector 观察参数协议，或使用下述独立客户端验证读写和范围检查。

围栏由 Flight 产品配置在 FMS 初始化时装载。当前仿真场景没有围栏或备降点，上传这些计划类型返回不支持。校准、执行器测试、固件升级和视频功能需要各自的设备或后端。

使用独立 MAVLink 客户端复测时，将 Host 的 `-GroundPort` 改为 14554，并在另一终端执行下列命令。两个场景依次运行，共用一套 FMT-SIM/Flight Host。静态仿真可用 `--bench` 验证遥测、参数和飞行指令拒绝。

```powershell
python tests/simulation/verify_ground_station.py --port 14554 --dialect build/firmware_integration/wire/fcu.py --mission --return-home
python tests/simulation/verify_ground_station.py --port 14554 --dialect build/firmware_integration/wire/fcu.py --mission --abort
python tests/simulation/verify_ground_station.py --port 14554 --dialect build/firmware_integration/wire/fcu.py --mission --no-pause --hold-before-mission 6.5
python tests/simulation/verify_ground_station.py --port 14554 --dialect build/firmware_integration/wire/fcu.py --mission --auto-plan
python tests/simulation/verify_ground_station.py --port 14554 --dialect build/firmware_integration/wire/fcu.py --mission --auto-plan --return-home
```

第一条验证参数读写边界、Home 设置、任务上传与下载、拒绝无效替换、解锁、起飞、暂停、恢复、到达末航点、返航着陆、上锁和清空。第二条验证终止任务后原地降落。第三条在起飞后悬停 6.5 秒，连续执行航点任务并在控制重新绑定后原地降落。动态验证使用 M7 真实提交的虚拟控制输出。H723 串口、量产授权和实物执行器另行验收。

最后两条分别验证一条开始命令触发的自动降落和自动返航计划。日志协议可在独立静态台架上用 `--bench --logs build/flight-sil-replay.tlog` 复测。详细入口和验收证据见 [QGroundControl 交付验收清单](QGC_ACCEPTANCE.md)。

固件 FinSH：

```text
msh /> sim_status
HIL link uart2: rx 12345 frames / 987654 bytes, 0 discarded bytes, tx 5432 frames (0 dropped)
  imu=4321/0 baro=210/0 gnss=21/0 mag-seen=4321 fresh-demand=4100 imu-fifo-drop=0
```

`fresh-demand` 不增长说明 M6 没有产出新需求（未解锁/未收到指令），此时上行
`controls` 为 0、`mode` 为 0，仿真器不应机动。

## 6. 验证记录

- 编解码和测量适配 124 项检查，桥接 6 项测试通过。金帧独立生成并由脚本校验。
- UDP 开环测试取得 91 个通过 CRC 校验的原生状态帧，机体响应平衡电机指令并爬升。
  记录位于 `build/validation/sim/fmt_release/motor_sweep.csv`。
- Host 静态 SIL 验证了预检、解锁、起飞请求、新旧参考切换和守卫回执。实际机体闭环
  运行及失败位置见 [飞行逻辑验证矩阵](FLIGHT_LOGIC_MATRIX.md)。

目标 UART/DMA、921600 波特率误码、目标 WCET/栈水位及实机台架仍待目标验证。
`tools/fmt_sim_probe.py`、`tools/fmt_sim_fly.py` 和 `verify_live.ps1` 面向标准
`HIL_SENSOR` 流。原生 FMT 状态使用上面的 Host 入口验证。

## 7. 附带修复与既有阻塞

1. **`middleware/sensor/sample_acquisition/sensor_input.h` 缺少 `#include "common_messages.h"`**
   （`SensorMeasurement` 未声明）。该头被 `sensor_imu.c`、`sensor_baro_mag.c`、
   `sensor_gnss.c` 引用，这三者原本都编不过，固件无法链接。本次补上该包含，
   使整树可构建。若需要独立变更记录，可把这一行单独拆出——它不依赖 HIL 改动。
2. **ABI 门禁当前为 FAIL（与本次改动无关）**：`middleware/contracts/
   product_types.h` 于 17:15 被修改，而 `product_abi_lock.h`
   （17:09）与 `spec/validation/abi_layout.json` 未随之重新生成。
   `python tests/abi/abi_report.py` 显示 143 个类型 / 981 个字段偏移 **0 失配**、
   host/ARM 0 意外差异，但 lock 哈希不一致（lock `386c8b8f…` vs 计算
   `2e480058…`）。重新生成需要 `--emit-lock` 并同步规范基线，属于 ABI 正式
   制品变更，未在本次 HIL 任务中代为执行。

