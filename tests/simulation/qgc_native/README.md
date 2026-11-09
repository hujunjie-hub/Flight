# Flight SIL 原版按钮源码适配

补丁基于 QGroundControl v5.1.5，提交 `3a67d31f0c36bf3fe38ec52970d250a89d0aaf67`。Flight 的动态 SIL 通过 `AUTOPILOT_VERSION.flight_custom_version` 上报 `FLTSIL01`。补丁依据该标识和 Generic 固件类型启用 Flight 行为。台架模式和实机串口不发送此标识。

| Fly 按钮 | 发往 Flight SIL 的操作 |
|---|---|
| 开始任务、继续任务 | `MAV_CMD_MISSION_START`。有完整计划时按起飞、任务、终点动作执行。任务已暂停时恢复 |
| 起飞 | 已解锁且在地面时，发送 2 m 的 `MAV_CMD_NAV_TAKEOFF` |
| 降落 | `MAV_CMD_NAV_LAND` |
| 返航 | `MAV_CMD_NAV_RETURN_TO_LAUNCH` |
| 暂停 | 任务飞行中发送 `MAV_CMD_DO_PAUSE_CONTINUE(0)` |

FMS 仍负责飞行授权和命令回执。可见性按对应操作设置，地图临时目标、改高、改速和绕飞沿用上游能力判断。

在可编译上游 v5.1.5 的 Windows 环境中，从 QGroundControl 源码根目录应用补丁。构建所需 Qt 版本与模块列表见该源码的 `.github/build-config.json`。

```powershell
$flightRoot = 'D:\STM32Project\FMT-Firmware-1.2.0\Flight'
$patch = Join-Path $flightRoot 'tests/simulation/qgc_native/qgc-5.1.5-flight-sil.patch'
git switch --detach v5.1.5
git apply --check $patch
git apply $patch
```

此补丁的应用检查已通过，修改后的 Fly QML 已通过 Qt 6.11.1 字节码生成。若在 Flight SIL 悬停仍在减速时点击继续，飞控会暂时拒绝，待悬停稳定后可重试。

## Flight 定制构建（插件 + 参数元数据）

`qgc-5.1.5-flight-custom.patch` 在上一补丁的基础上增加：

- `src/FirmwarePlugin/Flight/`：Flight 固件插件，模式 0–9 中文映射、模式切换、
  起飞/降落/返航/暂停/任务动作；
- `FlightParameterFactMetaData.json`：由
  `tools/generate_qgc_parameter_metadata.py` 从本项目 70 个参数生成的中文元数据
  （分组、单位、上下限、说明）；
- `qgc_winmain.cpp`、`QGC_DISABLE_IPO` 与 16 MB 栈配置：MinGW 13.1 构建所需的兼容改动。

构建步骤、工具链要求和联机截图见
[QGroundControl 定制构建](../../../doc/gcs/QGC_CUSTOM_BUILD.md)。

