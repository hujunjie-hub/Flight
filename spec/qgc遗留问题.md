# QGC 遗留问题

本页保留原始问题清单和复现步骤。修复、验证和交付状态以 [QGC 按钮矩阵](../doc/gcs/QGC_BUTTON_MATRIX.md) 顶部的 r5 记录为准；参数与校准的固件契约见 [QGC 参数与校准](../doc/gcs/QGC_PARAMETERS_AND_CALIBRATION.md)。

按三类划分：**A 功能验收**、**B 地面站适配**、**C Flight 固件完善**。动态 SIL 与 H723 台架固件分别验证。

## 0. 源码位置

| 项 | 路径 / 说明 |
|---|---|
| 官方项目来源 | `https://github.com/mavlink/qgroundcontrol`，tag/提交 `3a67d31`（v5.1.5） |
| 下载到本地的位置（构建用） | `D:\STM32Project\FMT-Firmware-1.2.0\Flight\build\validation\qgc_native\upstream` |
| 移动后的项目位置（交付源码） | `D:\STM32Project\FMT-Firmware-1.2.0\FMT_GroundControl`（17,454 个文件，约 1.16 GB，已排除 `.git`、`.venv`） |
| 现有构建目录 | `Flight\build\validation\qgc_native\cmake_mingw13`（其 CMake 缓存仍指向旧路径） |

注意：拷贝是**源码级**迁移，`cmake_mingw13` 的缓存里写死了旧的 `upstream` 绝对路径，
若要在 `FMT_GroundControl` 下重新构建，需要用同样的参数重新 configure 一个新的构建目录
（详见下一节的完整命令）。

## 0.1 编译工具链（交接给 Codex）

### 工具清单与版本（均为本机实测）

| 工具 | 版本 | 路径 |
|---|---|---|
| CMake | 4.3.1 | `D:\STM32CubeCLT\CMake\bin\cmake.exe` |
| Ninja | 1.13.2 | `D:\STM32CubeCLT\Ninja\bin\ninja.exe` |
| MinGW-w64 GCC/G++ | 13.1.0（x86_64-posix-seh-rev1） | `D:\STM32Project\FMT-Firmware-1.2.0\Flight\build\validation\qgc_native\mingw1310\Tools\mingw1310_64\bin` |
| Qt | 6.11.1（MinGW 13.1 官方包） | `D:\STM32Project\FMT-Firmware-1.2.0\Flight\build\validation\qgc_native\qt` |
| rc 包装器 | 自建 | `...\qgc_native\rcwrap.exe`，源码 `Flight\tools\qgc_rc_wrapper.c` |
| Patchelf/windeployqt | 构建期自动使用 | Qt 目录内 |

### 为什么必须用这套组合（三个硬约束）

1. **Qt 6.11.1 是 MinGW 13.1 编译的**，DLL 期待 `__emutls_v._ZSt11__once_call`（GCC 13 的 emulated TLS）。
   换 MinGW 16.x 会生成原生 TLS 符号，运行时报"无法定位程序输入点"。
2. **不能加 `-static-libstdc++`**：会破坏 Qt 的 emutls 依赖。
3. **MinGW 的 GCC LTO 与 lld 不兼容**，会链出没有 Qt 依赖的空壳，必须显式 `-DQGC_DISABLE_IPO=ON`。

### 构建命令（照抄可用）

```powershell
$root = 'D:\STM32Project\FMT-Firmware-1.2.0\Flight'
$src  = 'D:\STM32Project\FMT-Firmware-1.2.0\FMT_GroundControl'          # 或原来的 upstream
$build= "$root\build\validation\qgc_native\cmake_mingw13"                # 新位置请换新目录
$mingw= "$root\build\validation\qgc_native\mingw1310\Tools\mingw1310_64\bin"
$qt   = "$root\build\validation\qgc_native\qt"

$env:PATH = "$mingw;$qt\bin;" + $env:PATH

cmake -S $src -B $build -G Ninja `
      -DCMAKE_BUILD_TYPE=Release `
      -DCMAKE_C_COMPILER="$mingw/gcc.exe" `
      -DCMAKE_CXX_COMPILER="$mingw/g++.exe" `
      -DCMAKE_PREFIX_PATH="$qt" `
      -DCMAKE_RC_COMPILER="$root/build/validation/qgc_native/rcwrap.exe" `
      -DQGC_DISABLE_IPO=ON `
      -DQGC_ENABLE_GST_VIDEOSTREAMING=OFF

cmake --build $build -j 10
```

产物：`<build>\Release\QGroundControl.exe`（约 62 MB），日志示例 `Flight\build\qgc_flight_plugin*.log`。

### 交给 Codex 的注意事项

1. **路径分隔符**：`-DCMAKE_RC_COMPILER` 等参数必须用**正斜杠**或 `/`，反斜杠会让 CMake 报
   `Invalid character escape '\S'`（本次已踩过）。
2. **运行时 PATH**：启动 QGC 前必须把 `$mingw` 和 `$qt\bin` 放进 PATH，否则缺 Qt DLL / 报 App 启动失败。
3. **exe 被占用**：重新链接前必须先关闭正在运行的 `QGroundControl.exe`（否则 link 失败）。
4. **QPixmap 之外的资源**：`FlightSummary.qml` 用 `qt_add_resources` 挂在
   `PREFIX "/qml/QGroundControl/FirmwarePlugin/Flight"`，改 QML 后只需增量构建。
5. **测试构建**：仓库支持 `-DQGC_BUILD_TESTING=ON`，但本机未完成全量编译（已放弃，属可选路径）。
6. **中文翻译**：改 `translations/qgc_source_zh_CN.ts` / `qgc_json_zh_CN.ts` 后，构建会自动
   `lrelease` → `rcc` → 重新链接；`lupdate` 会重写全部 23 个语言文件，慎跑。
7. **备份**：`Flight\build\validation\qgc_native\upstream` 仍是当前构建实际使用的源码；
   `FMT_GroundControl` 是同一份源码的拷贝，二者不要同时当"源"改，选一个为准。

---

## 0.2 r5 参数与校准（已实现）

用户反馈“配置下面创建的参数部分没了”。原因是旧镜像只暴露 `FLT_PROFILE/FLT_CONTRACT/FLT_BENCH`
三项，QGC 在参数下载完成前又把整个“参数”入口隐藏；r5 已修正：

- H723 `task_gcs.c` 改用 `parameter_store`，与 SIL 共用 109 项目录；
- `PARAM_SET` 做类型/范围/解锁检查，应用后回读；
- Configure 的“参数”入口常驻，未连接/下载中/飞行中跳过时给出说明和“下载参数”；
- 新增 39 项校准参数和 `传感器校准` 页；
- `工具 → 从文件加载以查看` 的差异确认后逐项 `PARAM_SET`，SIL 实测 3 项覆盖写入；
- `重启飞行器` 走 `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`。

细节和验收记录见 [QGC 参数与校准](../doc/gcs/QGC_PARAMETERS_AND_CALIBRATION.md)。

---

## A. 待验证（功能已实现，只差点击验收）

恢复验证时的入口、句柄和步骤都已定位，不需要再写代码。

### A1. GeoFence 画多边形
- 交互：图层切换器是**两段式**——先点方形图标按钮（`layerSwitcher.toggle()`）展开，再点
  展开出来的「地理围栏」胶囊；折叠时选项 `opacity: 0`，看起来像点不动
- 句柄：`objectName: "layerSwitcher_choice_fenceGroup"`
- 代码：`src/PlanView/PlanView.qml:596-640`，`case _layerMission` 的 fence 分支
- 步骤：展开 → 点「地理围栏」→ 地图点 3~4 个顶点 → 上传 → 回读比对

### A2. Rally 备降点
- 同上入口，句柄 `objectName: "layerSwitcher_choice_rallyGroup"`
- 步骤：展开 → 点「备降点」→ 地图点 1~2 点 → 上传 → 回读

### A3. Transform 平移 / 旋转
- 位置：Plan 右栏底部「变换」节，需滚动任务项列表才露出
- 风险：右栏滚动容器与地图抢滚轮，容易滚成地图缩放
- 步骤：展开「变换」→ 点「平移」→ 填东/北偏移 → 应用 → 检查航点坐标变化

### A4. Resume Mission
- 链路：起飞 → 上传 ≥3 航点任务 → 开始任务 → 中途「返航」→ 落地解锁 → 「恢复任务」
- 代码已就位：`missionController.resumeMission(resumeMissionIndex)`（上传剩余航点 + MISSION_START）

### A5. 附带复验
- 另存为 KML 实际落盘（对话框已验证，未确认文件内容可被 Google Earth 解析）
- GeoFence / Rally 上传后的回读一致性

---

## B. 地面站端未完成（需要在 QGC 侧继续做）

### B1. 固件刷写页
- 现状：`Firmware` 入口已隐藏（`supportsFirmwareUpgrade()==false`）
- 待做：Flight 专用页，调用 STM32CubeProgrammer CLI 或 STM32 ROM DFU 刷 `Flight.bin`，
  显示目标型号/序列号与校验结果
- 依赖：无需固件改动（工具链已在本机）

### B2. 视频页 / 视频流
- 现状：页面保留，无后端数据
- 待做：若后续机载相机走 IP 流，配置 RTSP/UDP 地址即可，不需要飞控参与

### B3. 3D 视图
- 现状：页面保留，未验证渲染
- 待做：用现有 DEM + 位姿验证地图/模型是否正常，必要时关闭该入口

### B4. ADSB 服务器 / Remote ID 设置页
- ADSB：地面站侧接入接收机或网络源即可
- Remote ID：页面只做配置，广播需固件（见 C13）

### B5. 图层切换器可达性
- 现状：两段式交互，折叠态选项透明，用户容易误判"点不动"
- 待做：改成一个常驻的三段切换条（Mission / GeoFence / Rally），带选中态高亮
- 影响：只改 QGC 交互，不涉及协议

### B6. 剩余汉化
- r5 以本机 CUAV LGroundControl 为参照合并界面术语：常规→基本配置、计划视图→航线规划、
  ADS-B→ADSB、远程识别→远程 ID、遥测→遥测数据、视频→视频服务、
  应用文件存取路径→应用程序加载/保存路径、发送地面站位置→流动地面站位置；
  参数差异对话框已全中文。
- 生成工具：`tools/merge_lg_translations.py`；参考 TS 解出到
  `build/validation/lgc_translations/`。
- 待做：补齐 Flight 实际界面中仍显示英文的少量日志/蓝牙/APM 文本；LGroundControl
  独有的“限飞管理/快捷方式/关于”入口在本 QGC 5.1.5 构建中不存在，不在本轮范围。

### B7. 交付包
- 待做：重打 `Flight-SIL-QGC-*-r4.zip`（含最新二进制说明、按钮矩阵、本轮截图）

---

## C. Flight 固件端完善

方案要点见 `doc/gcs/QGC_GAP_ANALYSIS.md`，建议顺序：C7 → C9 → C2/C3/C6 → C1/C4/C5 → 其余。

| # | 能力 | MAVLink | 要点 |
|---|---|---|---|
| C1 | 前往指定点 | `DO_REPOSITION` | guidance 增「外部位置目标」入口，复用位置环/MPC，加限幅与超时 |
| C2 | 改高度 | `DO_CHANGE_ALTITUDE` | 改参考高度、保持水平位置 |
| C3 | 改速度 | `DO_CHANGE_SPEED` | 改参考速度上限/轨迹时间参数 |
| C4 | 盘旋 | `DO_ORBIT` | 生成圆周轨迹 |
| C5 | 兴趣点 ROI | `DO_SET_ROI` | 机头指向目标，与任务 yaw 策略统一 |
| C6 | 改航向 | `CONDITION_YAW` | 定点模式下的 yaw 目标通道 |
| C7 | 指定坐标设 Home | `DO_SET_HOME`(param1=0) | 现只接受 param1=1，放开坐标并做地理校验 |
| C8 | 紧急停 / 强制解锁 / 复飞 | 多命令 | 安全相关，需独立安全状态机与审计日志 |
| C9 | 清空任务 | `MISSION_CLEAR_ALL` | FMS 要求 mission_id+epoch；可在 DISARMED+IDLE 放宽 |
| C10 | 任务项相机/云台 | 相机协议 | 需相机组件与 MAVLink 代理 |
| C11 | 摇杆 | `MANUAL_CONTROL` | 手动控制通道 + 接管判据 |
| C12a | 传感器校准 | `PREFLIGHT_CALIBRATION`、`USER_2` | r5 已实现陀螺/加速度六面/磁罗盘/水平/气压/遥控行程，QGC 专用页；实机人工验收待做 |
| C12b | 电机测试 | `DO_MOTOR_TEST` | BENCH_INHIBIT 禁止物理输出，不提供 |
| C13 | Remote ID 广播 | `OPEN_DRONE_ID_*` | UAS ID 配置与广播周期 |
| C14 | NTRIP/RTK | `GPS_RTCM_DATA` | RTCM 注入 UM982（现只有 unicore ASCII 配置通道） |

### C 类相关的既有固件约束（QGC 只能如实反映）

1. 定点（POS_HOLD）不能作为任意模式间切换目标，跨模式切换返回 `MAV_RESULT_UNSUPPORTED`
2. 静态台架（无 FMT-SIM）所有飞行命令返回 `UNSUPPORTED`
3. 任务/围栏命令要求载具已有有效导航原点（`nav->origin_id` / `state_epoch`）
4. SIL 收不到 FMT-SIM 状态帧超时会以 `state timeout during flight` 退出
5. 已提交任务的生命周期命令必须携带 `mission_id` + `mission_epoch`。空任务只在 `DISARMED + IDLE` 时允许幂等清空。

---

## 已完成（供对照）

| 视图 | 已验证按钮 |
|---|---|
| Fly | 解锁、上锁、起飞、降落、返航、暂停、开始任务、继续任务、模式下拉 |
| Plan | 打开、保存、另存为、另存为 KML、上传、下载、起飞项、航点、图案（勘测）、返航项、保存校验、未保存确认 |
| Configure | 概览、Flight 状态卡、109 项参数（SIL 110）、参数写入/回读、参数文件差异上传、传感器校准页、重启飞行器 |
| Analyze | 日志查看器、打开 .bin/.ulg/.tlog、板载日志刷新/下载（88,614 B）/全部清除 |
| Settings | 应用设置各页、通信链路、新增链路对话框 |

已完成的 QGC 侧改动：

- `src/FirmwarePlugin/Flight/`：Flight 固件插件（能力位、模式、动作、参数元数据、AutoPilotPlugin 与状态卡）
- `src/FlyView/GuidedActionArm.qml`、`GuidedActionDisarm.qml`：解锁/上锁动作（QGC 5.1.5 原缺）
- `FirmwarePlugin::continueMission()` + `Vehicle::continueMission()`：修复「继续任务」命令
- `supportsCameraSection()` / `supportsMissionClear()` / `supportsFirmwareUpgrade()`：隐藏无后端入口
- 中文化：`qgc_source_zh_CN.ts` 与 `qgc_json_zh_CN.ts` 补词

## 相关文档

- `doc/gcs/QGC_CUSTOM_BUILD.md`：构建方法、插件说明、能力模型
- `doc/gcs/QGC_BUTTON_MATRIX.md`：按钮→能力矩阵与逐项验收记录
- `doc/gcs/QGC_GAP_ANALYSIS.md`：无后端项的地面站/固件分工
- `tools/qgc_ui_click.ps1`：DPI 感知的窗口点击/截图脚本（`-HoldMs` 用于按住确认）
