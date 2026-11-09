# QGroundControl 定制构建（Flight 专用）

本构建在 QGroundControl v5.1.5 官方源码上完成，不使用独立客户端。新增 Flight 固件插件
与参数元数据，使 QGC 原生界面直接显示 48 个核心调参参数（47 个 `CTL_*` + `VEH_MASS`）的名称、中文说明、单位、范围和分组；
模型几何与校准值留在固件内部，由模型重编译/校准页管理。FlightMode 0–9 显示为可选择的飞行模式。

## 源码树

```text
D:\STM32Project\FMT-Firmware-1.2.0\FMT_GroundControl\  # QGC v5.1.5 + Flight 补丁（权威源码）
  src/FirmwarePlugin/Flight/              # 新增：Flight 固件插件
    FlightFirmwarePlugin.{h,cc}           # 模式、起降/返航/暂停/任务动作
    FlightFirmwarePluginFactory.{h,cc}    # 注册到 MAV_AUTOPILOT_GENERIC
    FlightParameterMetaData.{h,cc}        # 读取参数元数据 JSON
    FlightParameterFactMetaData.json      # 由 tools/generate_qgc_parameter_metadata.py 生成
    FlightSensorsComponent.{h,cc}         # Configure → 传感器校准 页
    FlightSensors.qml                     # 陀螺/加速度/磁罗盘/水平/气压/遥控校准
    FlightRcMonitor.{h,cc}                # RC_CHANNELS → QML 通道条
  src/FlyView/GuidedActionsController.qml # 上一轮 Flight SIL 按钮补丁
  src/Vehicle/InitialConnectStateMachine.cc, Vehicle.{h,cc}
```

## Flight 插件做了什么

| 能力 | 实现 |
|---|---|
| 飞行模式 | `custom_mode` 0–9 映射为「未就绪/角速度/姿态/定高/定点/任务/返航/降落/紧急降落/起飞」 |
| 切换模式 | `MAV_CMD_DO_SET_MODE`（param1=1，param2=模式号），`MAV_CMD_DO_SET_MODE_is_supported()` |
| 起飞 | `MAV_CMD_NAV_TAKEOFF`，默认 2 m |
| 降落 / 返航 | `MAV_CMD_NAV_LAND` / `MAV_CMD_NAV_RETURN_TO_LAUNCH` |
| 暂停 | `MAV_CMD_DO_PAUSE_CONTINUE(0)` |
| 任务 | `MAV_CMD_MISSION_START` |
| 参数元数据 | `:/FirmwarePlugin/Flight/FlightParameterFactMetaData.json` |
| 参数编辑 | 只有 **Value 单元格**可点击进入编辑对话框；Name/Units/Description 不触发编辑 |
| 参数分组 | 机体模型 / 姿态角速度环 / 位置环回退 / MPC 权重 / 传感器校准 / 遥控校准 |
| 传感器校准 | `MAV_CMD_PREFLIGHT_CALIBRATION`(241) + `MAV_CMD_USER_2`(31011)，进度走 STATUSTEXT |
| 遥控监视 | CRSF 解码后由固件发 `RC_CHANNELS`，页面显示 8 通道 |

参数 JSON 由参数目录（`middleware/protocol/mavlink/parameters/ground_parameters.json`）
生成，因此 QGC 与固件看到的是同一份参数定义、同一组上下限。

```powershell
python tools/generate_qgc_parameter_metadata.py
```

生成的文件写入 `FMT_GroundControl/src/FirmwarePlugin/Flight/`，
随后重新构建 QGC 即生效。

## 构建

本机 Qt 为 6.11.1（MinGW，`build/validation/qgc_native/qt`）。MinGW 的 GCC LTO 目标
与 lld 不兼容，会链接出没有 Qt 依赖的空壳，因此本构建显式关闭 IPO：

```powershell
# Qt 6.11.1 官方包由 MinGW 13.1（emutls）编译，QGC 必须使用同一工具链。
$mingw = "$PWD\build\validation\qgc_native\mingw1310\Tools\mingw1310_64\bin"
$qt    = "$PWD\build\validation\qgc_native\qt"
$env:PATH = "$mingw;$qt\bin;" + $env:PATH

cmake -S D:/STM32Project/FMT-Firmware-1.2.0/FMT_GroundControl -B build/validation/qgc_native/cmake_delivery -G Ninja `
      -DCMAKE_BUILD_TYPE=Release `
      -DCMAKE_C_COMPILER="$mingw/gcc.exe" -DCMAKE_CXX_COMPILER="$mingw/g++.exe" `
      -DCMAKE_PREFIX_PATH="$qt" -DCMAKE_RC_COMPILER="$PWD\build\validation\qgc_native\rcwrap.exe" `
      -DQGC_DISABLE_IPO=ON -DQGC_ENABLE_GST_VIDEOSTREAMING=OFF
cmake --build build/validation/qgc_native/cmake_delivery -j 10
```

构建产物：`build/validation/qgc_native/cmake_delivery/Release/QGroundControl.exe`。

> 不能用 MinGW 16.2：Qt 6.11.1 的 DLL 期待 `__emutls_v._ZSt11__once_call`（GCC 13 的
> emulated TLS），MinGW 16 生成的是原生 TLS 符号，运行时会报“无法定位程序输入点”。
> 同理不能把 `-static-libstdc++` 混进链接。

## 中文化（参照 LGroundControl）

界面中文以本机 `D:\LGroundControl`（CUAV LGroundControl，QGC 分支）为参照。
其应用翻译编译在 `LGroundControl.exe` 的资源里，r5 从 PE 的 `.rdata` 中定位
`qgc_source_zh_CN.qm` / `qgc_json_zh_CN.qm`，用 Qt `lconvert` 转成 `.ts` 后
按 context + source 合并：

```powershell
# 已在 build/validation/lgc_translations 保存解出的参考 TS
python tools/merge_lg_translations.py `
  --ours FMT_GroundControl/translations/qgc_source_zh_CN.ts `
  --lg build/validation/lgc_translations/cand6_800000.qm.ts `
  --output FMT_GroundControl/translations/qgc_source_zh_CN.ts
```

合并规则：

1. 相同 `context + source` 的 LG 译文优先；
2. 该 source 在 LG 中只有唯一译文时使用；
3. Application Settings 用显式术语表（LG 与本 QGC 的 context 名不同）：
   `General → 基本配置`、`Plan View → 航线规划`、`Video → 视频服务`、
   `Telemetry → 遥测数据`、`ADSB Server → ADSB 服务器`、`Remote ID → 远程 ID`、
   `Application Load/Save Path → 应用程序加载/保存路径`、
   `Stream GCS Position → 流动地面站位置`、`Area → 区域`；
4. 参数差异对话框等 LG 没有的新字符串，按同一风格在
   `tools/merge_lg_translations.py` 的 `OVERRIDES` 中补齐。

当前 `qgc_source_zh_CN` 完成 3,516 条、`qgc_json_zh_CN` 完成 3,661 条；剩余
未完成项主要是本 Flight 构建不显示的 APM/Bluetooth/日志解析文本。构建时
CMake 只跑 `lrelease`，不会重新生成其他语言。

已验证：Settings → 基本配置页显示“基本配置 / 航线规划 / ADSB 服务器 /
远程 ID / 遥测数据 / 视频服务 / 流动地面站位置 / 处于‘跟随我’飞行模式时 /
应用程序加载/保存路径”，参数差异对话框显示“从文件加载了 110 项参数：将修改
5 项，105 项与载具一致”。截图见
`build/validation/qgc_native/i18n2_settings.png`、`i18n3_diff.png`。

## 联机

```powershell
# 飞控/SIL 先发送到 QGC 默认端口 14550
.\build\validation\gcs\params_build\sil_flight.exe 0 0.62 Ground 350 14550

# QGC 启动后由 AutoConnect 监听 14550；也可在 Comm Links 手工添加 UDP 14550
.\build\validation\qgc_native\cmake_mingw\Release\QGroundControl.exe
```

QGC 的 **Configure → Parameters** 按 Flight 元数据分组显示 H723 的 48 项调参。
**Fly** 的模式选择以当前连接能力为准。

## 飞行页四角

| 位置 | 界面与数据来源 |
|---|---|
| 左上 | 载具编号、飞行前检查、解锁与飞行状态使用 `SYS_STATUS`、`HEARTBEAT` 和 `EXTENDED_SYS_STATE`。模式显示 Flight 的实际模式名。只有有对应控制能力的 SIL 连接提供模式切换，H723 台架连接显示当前模式。 |
| 右上 | `GPS_RAW_INT` 显示接收机定位状态、卫星数和可用的 HDOP。实机的显示使用新鲜的接收机回报，融合仍使用原有的 PPS 时间映射和准入检查。SIL 电池独立发送标准 `BATTERY_STATUS`。H723 的电流计型号和换算系数待确认，地面站显示电池遥测不可用。 |
| 左下 | 参照 LGroundControl 提供地面站位置、指定位置、当前载具和持续跟随。当前载具置中要求有效坐标及链路。开始跟随时立即置中，拖动暂停跟随，松手后按原定延时恢复。 |
| 右下 | 速度和升降率由估计器有效字段经 `VFR_HUD` 更新，航向使用飞控姿态。相对高度需要相应报文，SIL 通过 `GLOBAL_POSITION_INT` 提供。无有效值或链路中断时显示占位。 |

点击地图上的载具仍可查看地速、相对高度和坐标。地图置中和跟随属于地面站操作。

## 按钮能力模型（不修改固件）

QGC 的按钮可见性由 `FirmwarePlugin` 决定，Flight 插件把固件真正实现的 MAVLink 能力声明出来，
其余按钮由 QGC 隐藏，不再下发会被固件拒绝的命令。

| 能力 | 插件声明 | 对应固件行为 |
|---|---|---|
| `SetFlightModeCapability` | 已识别的 Flight SIL 连接 | `MAV_CMD_DO_SET_MODE`（param1=1，param2=4–7） |
| `GuidedModeCapability` | 支持 | 起飞 / 降落 / 返航 / 暂停的公共前置条件 |
| `PauseVehicleCapability` | 支持 | `MAV_CMD_DO_PAUSE_CONTINUE(0)` |
| `TakeoffVehicleCapability` | 支持 | 任务首项 `MAV_CMD_NAV_TAKEOFF` |
| `GuidedTakeoffCapability` | 支持 | `MAV_CMD_NAV_TAKEOFF`（0–10 m） |
| `OrbitModeCapability` / `ROIModeCapability` / `ChangeHeadingCapability` | 不支持 | 固件无对应处理函数，Fly 与 Plan 的 Orbit/ROI/改航向按钮隐藏 |
| `GuidedAction`（Goto / ChangeAltitude / ChangeSpeed / ChangeLoiterRadius / SetHome / EmergencyStop / LandAbort / ForceArm / EstimatorOrigin） | 全部不支持 | 固件没有 `DO_REPOSITION`、`DO_CHANGE_ALTITUDE`、`DO_CHANGE_SPEED`、`DO_SET_HOME(param1=0)`、`DO_GO_AROUND` 等入口 |

`FlightFirmwarePlugin::supportsGuidedAction()` 与新增的 `VehicleSupports` 属性
（`gotoLocation`、`changeAltitude`、`changeSpeed`、`changeLoiterRadius`、`setHome`、`emergencyStop`、
`landAbort`、`forceArm`、`estimatorOrigin`）把上述结论传给 `GuidedActionsController.qml`。

任务命令树同样来自插件：`supportedMissionCommands()` 只返回 `NAV_TAKEOFF`、`NAV_WAYPOINT`、
`NAV_LAND`、`NAV_RETURN_TO_LAUNCH`，与 FMS 只接受「相对高度航点 + 可选首项起飞 + 可选末项降落/返航」一致。

上一轮为 SIL 联调加入的 `Vehicle::flightSil`（按 `gitHash == "FLTSIL01"` 判断）以及
`Vehicle` 里五处写死的 SIL 命令分支已删除：模式名、暂停、起飞、降落、返航全部回到插件的标准路径，
QGC 不再需要识别 SIL 身份。

## Configure / Analyze 视图适配

- Configure 使用 `FlightAutoPilotPlugin`，只提供一个只读的 Flight 状态卡（固件、模式、解锁、
  相对高度、地速、参数状态、刷写方式），不提供摇杆、传感器校准、电源、电机测试等固件没有的页面。
- `Firmware` 入口由新能力 `FirmwarePlugin::supportsFirmwareUpgrade()` 控制，Flight 返回 false，
  按钮隐藏（刷写走 STM32CubeProgrammer / SWD）。
- Analyze 只保留 Log Viewer、Onboard Logs、MAVLink Inspector；GeoTag、MAVLink Console、Vibration
  三个页面在本构建的 `QGCCorePlugin::analyzePages()` 中移除。
- 参数页新增独立 Units 列，列宽取 `max(内容宽度, 6 字符宽)`，表头不再被截断。

## 与上一轮 SIL 补丁的关系

`tests/simulation/qgc_native/qgc-5.1.5-flight-sil.patch` 仍负责 Fly 视图的 SIL 专用按钮
（开始/继续任务、起飞、降落、返航、暂停）。本轮新增的 Flight 插件在固件层补齐模式名、
模式切换、参数元数据和通用 Guided 动作；两者可以同时存在。

## 已验证

构建产物 `build/validation/qgc_native/cmake_delivery/Release/QGroundControl.exe`
已连接 Host SIL 运行：

| 界面 | 结果 | 截图 |
|---|---|---|
| Fly | 模式指示显示 Flight 的「未就绪」，左侧动作栏显示「起飞」「返航 / 降落」 | [qgc_fly.png](images/qgc_fly.png) |
| Configure → Parameters | 左侧分组为「机体模型 / 姿态角速度环 / 位置环回退 / MPC 权重 / 装订角速度环」，表格列为 **Fav / Name / Value / Units / Description**，`CTL_ACT_DLY` 单位显示 `s`，其余无单位参数显示 `-` | [qgc_parameters_units.png](images/qgc_parameters_units.png) |
| Configure | 进入 Vehicle Configuration，Generic / Quadrotor，参数与固件页可用 | [qgc_vehicle_configuration.png](images/qgc_vehicle_configuration.png) |

复现：

```powershell
pwsh tools/run_qgc_flight.ps1 -UdpPort 14550
```

`tools/run_qgc_flight.ps1` 会先启动 SIL，再用 MinGW 13.1 + Qt 6.11.1 的 PATH 启动
定制 QGC。

## 状态

- Flight 插件（模式、动作、参数元数据）已编译进官方 QGC，并在界面上验证。
- 参数写入语义与固件一致：会话内有效，重启回落 YAML，解锁状态拒绝。
- 参数页的分组来自 `FlightParameterFactMetaData.json`，与固件同一份目录。
- QGC 版本标题显示 `QGroundControl Daily v5.1.5`，这是缓存里 `QGC_DAILY_BUILD` 的构建标识。

