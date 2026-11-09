# QGroundControl 按钮适配矩阵（Flight 专用构建）

## r5 修复记录

对应 [QGC 遗留问题](../../spec/qgc遗留问题.md)。参数与校准细节见
[QGC 参数与校准](QGC_PARAMETERS_AND_CALIBRATION.md)。

| 项 | r5 状态 |
|---|---|
| Configure → 参数 | 入口常驻：未连接/未下载/飞行中跳过时显示说明和“下载参数”，不再整项消失。 |
| 参数链路 | H723 暴露 48 项核心调参；PARAM_REQUEST_LIST/READ/SET 全部由 `parameter_store` 处理，写入范围和已解锁检查后立即应用并回读。 |
| 参数文件 | QGC `配置 → 参数 → 导入参数…` 预览差异后逐项 `PARAM_SET`；SIL 验证覆盖回读和越界拒绝。 |
| 参数页 | 固件内部 109 项，但 MAVLink/QGC 只暴露 48 项核心调参（47 `CTL_*` + `VEH_MASS`）；SIL 加 `SIM_TLM_HZ` 为 49 项。模型几何与 `CAL_*`/`RC_*` 不进入参数页/参数文件。r5 修复收藏列遮挡、列宽塌缩和 `单位/说明`截断。 |
| 传感器校准 | 新增 `传感器校准` 页：陀螺零偏、加速度六面、磁罗盘、水平姿态、气压基准、遥控行程，全部通过 `MAV_CMD_PREFLIGHT_CALIBRATION`(241) 和 `MAV_CMD_USER_2`(31011) 下发。 |
| 校准结果 | 固件回写 `CAL_*`/`RC_*` 参数并置 `*_OK`；SIL 陀螺校准完成，QGC 状态条显示“陀螺 ✓”。 |
| 遥控监视 | ELRS 增加 CRSF 解析，回传标准 `RC_CHANNELS`；校准页显示 8 通道实时条。 |
| 重启飞行器 | `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`(246) 在 DISARMED 下接受，500 ms 后复位；SIL 只回 ACK。 |
| Configure → 固件 | 通用 PX4 固件页从 Flight 构建隐藏，只保留 `Flight 固件`（SWD + STM32CubeProgrammer）。 |
| 中文化 | 以本机 CUAV LGroundControl 为参照合并 3,500+ 条译文：常规→基本配置、计划视图→航线规划、ADS-B→ADSB、远程识别→远程 ID、遥测→遥测数据、视频→视频服务、应用文件存取路径→应用程序加载/保存路径、发送地面站位置→流动地面站位置；参数差异对话框全中文。 |
| 其余按钮 | 仍按能力隐藏：C1–C6、C8、C10、C11、C13 没有固件后端；电机测试因 BENCH_INHIBIT 不提供。 |

实机 H723 使用 BENCH_INHIBIT；带飞行命令的回归使用动态 FMT-SIM。

## r4 修复记录

对应 [QGC 遗留问题](../../spec/qgc遗留问题.md)。实机 H723 使用 BENCH_INHIBIT。带飞行命令的回归使用动态 FMT-SIM。

| 编号 | r4 状态 |
|---|---|
| A1、A2 | QGC 界面已验证画多边形、点选备降点。动态 SIL 已验证上传、回读和清空，也验证了非法围栏回滚与越界航点拒绝。 |
| A3 | 变换入口移到航点列表前。平移和旋转后的航点坐标仍待对比。 |
| A4 | 修复任务中返航的控制参考切换。恢复计划的起飞项采用落地后载具当前位置，保留原相对高度。动态 SIL 已完成中途返航、上锁、上传剩余任务、重新起飞并执行至落地上锁。 |
| A5 | KML 导出使用原子写入，写入失败时显示文件错误。QGC 导出的示例已通过 KML 2.2 命名空间和航点坐标解析。 |
| B1 | Configure 增加 Flight SWD 刷写页，探测 STM32 设备 ID 和 ST-LINK 序列号后才允许写入并校验。已验证无探头时显示错误且禁用写入，实物刷写待验。 |
| B2 | 视频设置保留 RTSP 和 UDP MPEG-TS。修正 Qt Multimedia 后端的 UDP 地址、超时单位和启停状态。裸 RTP 视频选项随 GStreamer 后端提供。 |
| B3 | 3D 保持可配置，加载出错时切回 2D。DEM 与位姿的渲染联调待验。 |
| B4 | ADS-B SBS-1 服务器地址或端口提交后重新连接。Remote ID 设置页标明需要兼容广播设备。 |
| B5、B6 | Plan 提供常驻任务、地理围栏、备降点按钮及选中态。补齐本次改动、视频源和任务变换涉及的中文文案。 |
| B7 | r4 使用独立构建目录与 Qt 运行时生成 Windows 便携包，附界面截图、示例和验证记录。 |
| C7 | 动态 SIL 接受当前位置和指定坐标 Home。指定坐标经过坐标系、区域、海拔和 FMS 围栏检查，解锁时保留仍有效的手动 Home。 |
| C9 | 空任务在 DISARMED + IDLE 时幂等清空。已提交任务仍绑定 ID 和 epoch。QGC 顺序清空任务、围栏和备降点，收到成功 ACK 后才更新各层缓存。 |
| C14 | H723 台架接收 GPS_RTCM_DATA，重组、校验 CRC24Q 并通过 UART4 异步发送至 UM982。已通过 Host 帧测试和 ARM 构建，差分源与 RTK Fix 待实机验证。 |
| C1–C6 | 待实现可校验的外部位置、航向和轨迹参考，并验证失败时的参考交接。 |
| C8、C11、C12 | 待实现可信控制链路与对应的权限和安全状态。校准、电机测试需要实际设备验证。 |
| C10、C13 | 待提供相机、云台、Remote ID 广播设备及其接口信息。 |

以下是 r3 的按钮逐项验收记录。状态说明：

- **接通**：按钮下发的 MAVLink 命令由 Flight 运行时处理，已在 SIL 或协议测试中验证。
- **隐藏**：固件没有对应处理函数，QGC 不显示该按钮，避免下发必然被拒绝的命令。
- **待验证**：代码已就位，还需要在 armed/flying 状态下做界面点击验收。

## 1. Fly 视图

| 按钮 / 控件 | QGC 下发 | Flight 处理 | 状态 |
|---|---|---|---|
| 模式下拉框（未就绪/角速度/姿态/定高/定点/任务/返航/降落/紧急降落/起飞） | `MAV_CMD_DO_SET_MODE`(1, mode) | `case 176` → `PRODUCT_CMD_*` | 已验收：点状态条弹出 10 个中文模式（`shot_status_pill.png`）；固件约束见第 7 节 |
| 解锁 / 上锁滑块 | `MAV_CMD_COMPONENT_ARM_DISARM`(1/0, 0) | `case 400` | 接通（静态台架会拒绝） |
| 起飞 | `MAV_CMD_NAV_TAKEOFF`(param7=高度) | `case 22`，0–10 m | 接通 |
| 降落 | `MAV_CMD_NAV_LAND` | `case 21` | 已验收：界面按住确认后 mode 变「降落」，高度 3.0 m → 0.0 m |
| 返航 | `MAV_CMD_NAV_RETURN_TO_LAUNCH` | `case 20` | 已验收：界面按住确认后 mode 变「返航」，返航按钮随即置灰 |
| 暂停 | `MAV_CMD_DO_PAUSE_CONTINUE`(0) | `case 193` | 已验收：任务中点按后 mode 变 `定点`、暂停按钮消失、出现「继续任务」 |
| 开始任务 | `MAV_CMD_MISSION_START` | `case 300` | 已验收：地面点按后自动解锁、起飞，状态条 `起飞` → 任务 |
| 继续任务 | `MAV_CMD_DO_PAUSE_CONTINUE`(1) | `case 193`（恢复） | 已验收：从 `定点` 恢复到 `任务`（新增 `continueMission()`，见第 9 节） |
| 恢复任务（Resume mission） | 上传剩余航点后 `MISSION_START` | 任务上传协议 | 接通 |
| 紧急停 | `MAV_CMD_DO_SET_MODE` 组合 | 无 | 隐藏（`EmergencyStopAction`） |
| 强制解锁 | `COMPONENT_ARM_DISARM`(1, 21196) | 固件要求 param2=0 | 隐藏（`ForceArmAction`） |
| Go to location（地图点选） | `MAV_CMD_DO_REPOSITION` | 无 | 隐藏（`GotoAction`） |
| 修改高度 | `MAV_CMD_DO_CHANGE_ALTITUDE` | 无 | 隐藏（`ChangeAltitudeAction`） |
| 修改速度 | `MAV_CMD_DO_CHANGE_SPEED` | 无 | 隐藏（`ChangeSpeedAction`） |
| 修改盘旋半径 | `DO_REPOSITION` + loiter | 无 | 隐藏（`ChangeLoiterRadiusAction`） |
| Orbit | `MAV_CMD_DO_ORBIT` | 无 | 隐藏（`OrbitModeCapability`） |
| ROI | `MAV_CMD_DO_SET_ROI*` | 无 | 隐藏（`ROIModeCapability`） |
| 修改航向 | `MAV_CMD_CONDITION_YAW` | 无 | 隐藏（`ChangeHeadingCapability`） |
| 地图设置 Home | `MAV_CMD_DO_SET_HOME`(param1=0) | 只接受 param1=1（当前位置） | 隐藏（`SetHomeAction`） |
| 设置估算原点 | `SET_GPS_GLOBAL_ORIGIN` | 无 | 隐藏（`EstimatorOriginAction`） |
| 复飞（Land abort） | `MAV_CMD_DO_GO_AROUND` | 无 | 隐藏（`LandAbortAction`） |
| 相机 / 云台 / 视频 | 相机协议 | 无相机后端 | 无载具相机时不显示 |
| 摇杆（虚拟摇杆 / 手柄） | `MANUAL_CONTROL` | 固件无该消息处理 | `supportsJSButton()==false`，Configure 不提供摇杆页 |
| Follow me | `FOLLOW_TARGET` | 无 | `followFlightMode()` 为空，按钮不出现 |
| 顶栏状态 | 读 `HEARTBEAT`/`GPS_RAW_INT`/`BATTERY_STATUS`/`SYS_STATUS` | 本地显示 | 按 Flight 实际模式、飞行前状态与接收机状态显示；无实机电池数据时标为不可用 |
| 地图中心（左下角） | 无 MAVLink，纯视图操作 | 本地地图 | 提供地面站位置、指定位置、当前载具和跟随；校验坐标及链路，跟随启用时立即置中 |
| 载具信息卡（点地图载具图标） | 无 MAVLink，读载具 Fact | 本地 | r5 新增：深色圆角卡片，显示载具/地速/相对高度/纬度/经度；点地图空白关闭 |
| 远程ID扫描器（左下角） | — | 需要 Remote ID 接收机后端 | 本轮按指示先不做 |

代码位置：能力位与 `GuidedAction` 在
[FirmwarePlugin.h](../../build/validation/qgc_native/upstream/src/FirmwarePlugin/FirmwarePlugin.h)，
判定在 [FlightFirmwarePlugin.cc](../../build/validation/qgc_native/upstream/src/FirmwarePlugin/Flight/FlightFirmwarePlugin.cc)，
可见性在 [GuidedActionsController.qml](../../build/validation/qgc_native/upstream/src/FlyView/GuidedActionsController.qml)。

> QGC 5.1.5 的 Fly 工具条本身只挂了 3D、检查单、起飞、降落、返航、暂停、附加动作、夹爪这几项
> （`FlyViewToolStripActionList.qml`），**没有独立的解锁/上锁按钮**；解锁由地面站按
> `MAV_CMD_COMPONENT_ARM_DISARM` 触发，已用 MAVLink 客户端验证（`ACK 400 = 0`）。

## 2. Plan 视图

| 按钮 | 说明 | 状态 |
|---|---|---|
| Waypoint | `MAV_CMD_NAV_WAYPOINT`，相对高度 ≤ 32 项 | 接通 |
| Takeoff | 首项 `MAV_CMD_NAV_TAKEOFF` | 接通（命令树仅保留 16/22/21/20） |
| Land / RTL（末项） | `NAV_LAND` / `NAV_RETURN_TO_LAUNCH` | 接通 |
| Pattern（Survey / Corridor / Structure） | 展开为航点 | 接通 |
| ROI / Cancel ROI | `DO_SET_ROI` 任务项 | 隐藏（`supports.roiMode==false`） |
| 上传 / 下载 | `MISSION_COUNT/ITEM/REQUEST/ACK` | 已验收：上传后下载回读一致 |
| 清空（Clear） | `MISSION_CLEAR_ALL` | 隐藏（`supports.missionClear==false`）：FMS 要求清空请求携带 `mission_id` 与 `mission_epoch`，MAVLink 的 CLEAR_ALL 没有这两个字段，必然被拒；用重新上传覆盖即可 |
| 任务项编辑器 | 相机/云台段（`CameraSection`） | 隐藏，改为说明文字（`supports.cameraSection==false`）；未连接载具时保留以便离线规划 |
| GeoFence | 依赖 `AUTOPILOT_VERSION` 的 `MISSION_FENCE` 位 | 动态场景接通，静态场景不显示 |
| Rally Points | 依赖 `MISSION_RALLY` 位 | 上传/回读/清空接通，自动选点未实现 |
| 保存 / 打开 / 统计 | 本地 `.plan` 文件 | 接通 |

## 3. Configure 视图

| 入口 | 说明 | 状态 |
|---|---|---|
| 概览 | 显示 Flight 状态卡：固件、模式、解锁、相对高度、地速、参数、刷写方式 | 接通；r5 删除重复的左侧 `Flight` 入口，只保留概览 |
| ~~Flight（部件）~~ | 与概览是同一张 `FlightSummary.qml` | r5 移除左侧入口，`summaryQmlSource` 仍供概览使用 |
| 传感器校准 | 陀螺、加速度六面、磁罗盘、水平、气压、遥控行程；CRSF 通道监视 | 接通（r5 截图 `r5_sensors.png`、`r5_sensors_status.png`） |
| Parameters | 48 项核心调参（SIL 49）：47 `CTL_*` + `VEH_MASS`（+SIL `SIM_TLM_HZ`）；分组/单位/范围/中文说明；可编辑、可从文件上传 | 接通（r5 截图 `core_params.png`） |
| Firmware | 通用 PX4 刷写页 | 隐藏；只保留 `Flight 固件` SWD 页 |
| Joystick | 固件不支持手柄输入 | 隐藏 |
| Optical Flow | 无光流设备 | 不显示 |
| 电台 / 电源 / 电机测试 | 遥控有监视与校准，但没有飞行操作权；PMU 无电池链路；BENCH_INHIBIT 禁止电机输出 | 不显示 |
| 参数页工具 | 刷新、从文件加载、保存、重置、重启飞行器 | 前四项走参数协议；重启走 `MAV_CMD 246` |

代码位置：[FlightAutoPilotPlugin](<../../build/validation/qgc_native/upstream/src/FirmwarePlugin/Flight/FlightAutoPilotPlugin.h>)，
按钮可见性在 [VehicleConfigView.qml](../../build/validation/qgc_native/upstream/src/Vehicle/VehicleSetup/VehicleConfigView.qml)。

## 4. Analyze 视图

| 页面 | 说明 | 状态 |
|---|---|---|
| Log Viewer | 打开 `.tlog` 回放 | 保留 |
| Onboard Logs | 刷新/选择/下载/清空板载日志 | 接通（本次下载 88,614 B，状态 `Downloaded`） |
| MAVLink Inspector | 实时消息查看 | 保留 |
| GeoTag Images | 需要相机照片 | 本构建移除 |
| MAVLink Console | 需要 `SERIAL_CONTROL` shell | 本构建移除 |
| Vibration | 需要 `VIBRATION` 消息 | 本构建移除 |

## 5. Settings 视图

| 入口 | 说明 | 状态 |
|---|---|---|
| Comm Links | UDP 14550 / 串口 115200 | 接通（UDP 已实测） |
| Fly View / Plan View / Maps / App Logging | QGC 通用设置 | 保留 |
| NTRIP/RTK | UM982 支持 RTK，保留 | 保留 |
| Video / 3D View / Remote ID / ADSB Server / Telemetry | Flight 没有视频、3D、Remote ID、ADSB、电台 RSSI 后端 | 保留为 QGC 应用设置（不影响载具），后续可按需隐藏 |

## 6. 证据

| 项 | 文件 |
|---|---|
| 插件能力与 GuidedAction 实现 | `qgc_flight_plugin3.log`（编译） |
| Configure / Analyze 适配编译 | `qgc_flight_plugin4.log` |
| Flight 状态卡 + 部件 | `qgc_flight_plugin5.log`、`shot_configure3.png` |
| Analyze 只保留三页 | `shot_analyze.png` |
| 板载日志下载（88,614 B） | `shot_logdownload2.png` |
| Fly 视图（未解锁，仅起飞/返航可见） | `shot_fly2.png`、`shot_after_summary.png` |
| Fly 视图（ONLINE SIL 飞行中，仅降落/返航可见） | `shot_flying.png`、`shot_flying2.png` |
| 引导动作提示框（Land/RTL） | `shot_land_click.png`、`shot_rtl_click.png` |
| 降落验收（mode 变降落，3.0 m → 0.0 m） | `shot_land_confirm.png`、`shot_landed.png` |
| 返航验收（mode 变返航） | `shot_rtl_confirm.png` |
| 任务上传/下载、开始任务、暂停、继续任务 | `shot_upload_confirm2.png`、`shot_download_mission.png`、`shot_mission_start.png`、`shot_mission_pause.png`、`shot_continue_mission.png` |
| 清空确认框与固件拒绝提示 | `shot_clear2.png`、`shot_cleared.png` |
| 隐藏 Clear 后的 Plan 工具条 | `shot_no_clear.png` |
| 围栏/备降点计划的上传与回读 | `shot_fence_loaded.png`、`shot_fence_upload.png`、`shot_fence_download.png` |
| 相机段隐藏后的任务设置页 | `shot_plan_settings.png` |
| Settings 应用设置 | `shot_settings.png` |
| Plan 视图（无 ROI 按钮） | `shot_plan.png` |
| 参数页（70 项、Units 列完整） | `shot_params3.png` |
| r5 Configure 入口（参数 + 传感器校准） | `build/validation/qgc_native/r5_configure.png` |
| r5 未连接载具时参数入口仍保留 | `build/validation/qgc_native/r5_noveh_params2.png` |
| r5 109 项参数页 | `build/validation/qgc_native/r5_params.png` |
| r5 校准页 | `build/validation/qgc_native/r5_sensors.png` |
| r5 陀螺校准进度与完成状态 | `build/validation/qgc_native/r5_gyro2.png`、`r5_sensors_status.png` |
| r5 参数文件差异与写入 | `build/validation/qgc_native/r5_diff_dialog.png`、`r5_diff_applied.png` |
| r5 LGroundControl 术语（基本配置/航线规划/ADSB/远程 ID/遥测数据/视频服务） | `build/validation/qgc_native/i18n2_settings.png` |
| r5 参数差异对话框全中文 | `build/validation/qgc_native/i18n3_diff.png` |
| r5 配置页“飞行器配置”与中文入口 | `build/validation/qgc_native/i18n3_configure.png` |
| r5 去除重复 Flight 入口、参数表格满宽 | `build/validation/qgc_native/core_configure3.png` |
| r5 核心参数页（48/49 项、收藏列完整、SO3 姿态增益分组） | `build/validation/qgc_native/core_params.png` |
| r5 校准完成状态（不依赖 CAL 参数） | `build/validation/qgc_native/core_gyro.png` |
| r5 Fly 左下角“中心”入口 | `build/validation/qgc_native/flytools_home.png` |
| r5 顶栏（载具 1 / 准备飞行 / Custom:0x0 / GPS 16/1.0 / 电池 85%） | `build/validation/qgc_native/qgc_topbar3.png` |
| r5 地图中心面板（当前位置/指定位置/当前载具/锁定当前载具） | `build/validation/qgc_native/center4_panel.png` |
| r5 指定位置经纬度对话框 | `build/validation/qgc_native/center_specified.png` |
| LGroundControl 参考：中心工具提示 | `build/validation/qgc_native/lg_hover2.png`、`lg_center_click.png` |

## 6. 已知固件约束（QGC 侧只能如实反映，不改固件）

1. **定点（POS_HOLD）不是任意模式间可切换的目标**：`MAV_CMD_DO_SET_MODE`(1,4) 只在已经处于定点、
   或任务正在运行（作为暂停）时才被接受；从降落/返航/起飞切到定点会返回
   `MAV_RESULT_UNSUPPORTED`。QGC 的模式下拉框会显示这次失败提示，这是固件行为。
2. 静态台架（无 FMT-SIM）下 `ground_station_step(..., fmt_online ? &ground_actions : NULL)` 传入空动作表，
   所有飞行命令返回 `MAV_RESULT_UNSUPPORTED`，只能看遥测和参数；按钮级验收必须在 ONLINE SIL 下做。
3. **任务/围栏命令要求载具已有有效导航原点**：`commit_auxiliary_plan()` 在
   `!nav->origin_id || !nav->state_epoch` 时直接 `MISSION_DENIED`。新建的 SIL 实例刚启动、
   尚未建立 Home/原点时，上传、清空、围栏都会返回「不接受任何任务命令」。
   先让载具拿到位置与 Home（例如起飞一次或等待导航收敛）再操作。
4. SIL 在飞行中若超过设定时长收不到 FMT-SIM 状态帧，会以 `state timeout during flight` 退出（
   `exit=1`），这是仿真器的保护行为，与 QGC 无关。
5. **任务命令必须携带任务身份**：`fms_mission_command_matches()` 要求请求里带
   `mission_id` 与 `mission_epoch`（否则 reason 1 / 12 拒绝）。MAVLink 的
   `MISSION_CLEAR_ALL` 没有这两个字段，所以「清空」在固件侧永远不可能成功；
   `MISSION_START` 则由 SIL 从当前计划补全身份后才能通过。

## 7. 待验证

1. 实机 H723：参数页应显示 48 项核心参数（SIL 为 49，多 `SIM_TLM_HZ`），并完成一次参数文件上传。
2. 实机校准：六面加速度、磁罗盘旋转、ELRS 遥控行程需要人工操作；气压基准需要实际 BMP585 数据。
3. 实机 `重启飞行器`：确认 QGC 收到 ACK 后飞控复位并重新连接。
4. Plan 的 **Clear**（清空任务）与 GeoFence / Rally Points 的界面上传（动态场景已出现这两节）。
5. Settings → Comm Links 的串口链路。
6. 交付 zip 重新打包（现有包仍是 09:13 旧版，早于插件与全部本轮改动）。

> 重要：QGC 的引导动作是**按住确认**（`QGCDelayButton`，约 1 s），不是单击。
> 自动化点击必须用 `-HoldMs 2000` 并按在顶部弹出的确认条上，单击只会弹出确认条。

## 8. ONLINE SIL 复现步骤

```powershell
# 1. 在线 SIL + FMT-SIM（Ground/HIL 场景，14653 → 14550）
pwsh -File tests/simulation/run_sil.ps1 -OutDir build/validation/qgc/sil_online `
     -FmtSimExe build/validation/sim/fmt_release/FMT_SIM/Binaries/Win64/FMT_SIM.exe `
     -FmtHilPort 14653 -GroundPort 14550 -FmtStateTimeoutMs 1000

# 2. 让 SIL 起飞（脚本占用 14550，起飞后退出，SIL 继续悬停）
python build/validation/qgc_native/drive_fly.py

# 3. 启动定制 QGC，自动连上 14550，Fly 视图显示 Flying/定点 + 降落/返航按钮
pwsh tools/run_qgc_flight.ps1 -UdpPort 14550
```

界面点击辅助脚本：`tools/qgc_ui_click.ps1 -X <物理像素> -Y <物理像素> -Out <png>`（自动设置 DPI 感知）。

## 9. 本轮修复记录

1. **「继续任务」用错了命令**：QGC 原本对「开始」和「继续」都发 `MAV_CMD_MISSION_START`，
   Flight FMS 在暂停态下对 MISSION_START 返回 `MAV_RESULT_TEMPORARILY_REJECTED`。
   新增 `FirmwarePlugin::continueMission()`（默认回落 `startMission()`），Flight 覆写为
   `MAV_CMD_DO_PAUSE_CONTINUE(1)`；`Vehicle::continueMission()` 暴露给 QML，
   `GuidedActionsController.qml` 把 `actionContinueMission` 单独分流。实测恢复到 `任务`。
2. **任务项相机段**：新增 `FirmwarePlugin::supportsCameraSection()`（默认 true），Flight 返回 false；
   `SimpleItemEditor.qml` 与 `MissionSettingsEditor.qml` 据此隐藏 `CameraSection`，
   并在任务设置页给出说明文字。
3. **参数 Units 列宽**：列宽取 `max(内容宽, 6 字符)`，表头不再截断。
4. **Plan 的 Clear**：新增 `FirmwarePlugin::supportsMissionClear()`（默认 true），Flight 返回 false，
   `PlanToolBarIndicators.qml` 据此隐藏按钮。原因见第 6 节第 5 条——FMS 要求任务身份，
   MAVLink 清空报文无法携带，按钮只会弹错误。
