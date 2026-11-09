# HIL 链路 Host 测试

覆盖 `applications/tasks/communication/task_sim.c` 中可脱离硬件执行的部分：MAVLink v2 组帧/
解帧（含 CRC_EXTRA）、`HIL_SENSOR`/`HIL_GPS` 解码、M2 标准样本构造、推力归一化。

## 运行

```powershell
pwsh tests/simulation/run_host_tests.ps1
```

默认使用 `D:\STM32CubeCLT\mingw64\bin\gcc.exe`，可用 `-Gcc` 覆盖；日志写入
`build/validation/sim/test_hil_link.log`，退出码非 0 表示失败。

整链 Host SIL 使用相同的任务、导航、FMS、制导、控制和输出守卫。默认用静态
传感器样本，指定端口时接入 FMT-SIM 原生 HIL。

```powershell
pwsh tests/simulation/run_sil.ps1
pwsh tests/simulation/run_sil.ps1 -FmtHilPort 14653 -FmtScenario Land
pwsh tests/simulation/run_sil.ps1 -FmtHilPort 14653 -FmtScenario Return
pwsh tests/simulation/run_sil.ps1 -FmtHilPort 14653 -FmtScenario Mission
```

需要由脚本启动隔离实例时，传入其 `Binaries/Win64/FMT_SIM.exe` 路径：

```powershell
pwsh tests/simulation/run_sil.ps1 -FmtSimExe $sim -FmtHilPort 14653 -FmtScenario Mission
```

本机的软件渲染适配器编号为 1。`-FmtGraphicsAdapter` 可按目标 PC 调整，状态流仍按默认 350 ms 验收。

隔离实例的配置和原生状态帧说明见 [联机仿真说明](../../doc/simulation/HIL_LINK.md)。

## 文件

| 文件 | 作用 |
|---|---|
| [test_hil_link.c](test_hil_link.c) | 测试主体。以 `-DFLIGHT_SIM_UNIT_TEST` 包含 `task_sim.c`，只链接纯协议部分 |
| [test_bridge.py](test_bridge.py) | 桥的端到端测试：假串口 + 真实 UDP socket，验证字节不改写、计数一致、参数校验 |
| [verify_live.ps1](verify_live.ps1) | 对运行中的 FMT-SIM 联机验证：探测 HIL 流，再用固件解码器复算抓包 |
| [decode_capture.c](decode_capture.c) | 用固件同一份解码器解析抓包（`-DFLIGHT_SIM_UNIT_TEST` 包含 `task_sim.c`） |
| [golden/generate_golden.py](golden/generate_golden.py) | **独立实现**的 MAVLink v2 组帧器，生成金帧 |
| [golden/hil_golden.h](golden/hil_golden.h) | 生成物，随仓库提交；测试逐字节比对 |
| [run_host_tests.ps1](run_host_tests.ps1) | 先 `--check` 金帧是否过期，再跑桥测试，再编译并运行 C 测试 |

金帧由 Python 按字段偏移独立打包并自行计算 CRC，不复用固件代码，避免
"编码器与解码器一起错"的假通过（同 `tests/support/sensor_sim` 的思路）。改动
`task_sim.c` 的协议常量后必须重新生成：

```powershell
python tests/simulation/golden/generate_golden.py
```

`--check` 模式只比较不写入，CI/本地运行脚本已内置。

## 当前覆盖

- CRC-16/MCRF4XX 标准校验值（`"123456789"` → `0x6F91`）；
- `HIL_SENSOR` 全字段解码（加速度、角速度、磁、气压、温度、`fields_updated`）；
- `HIL_GPS` 解码，含 36 字节（无扩展）与 37 字节两种载荷；
- `FMT_EXTERNAL_STATE` 的 85 字节裁剪载荷、坐标转换、坏 CRC 和截断拒绝；
- 解析器：垃圾前缀重同步、逐字节投递、坏 CRC 丢弃、未知 msgid 跳过、连续多帧；
- 编码器：`HIL_ACTUATOR_CONTROLS`、`HEARTBEAT` 与金帧逐字节一致，全零需求仍成帧；
- 推力归一化：边界、钳位、零跨度、NaN；
- 样本构造：`ImuSample`/`BaroSample`/`GnssSample` 的 header（source/frame/时戳
  关系/不确定度/validity）、质量位、`fix_type` 映射、协方差来源、2D/无定位拒绝。

## 未覆盖（需目标或台架）

- 真实 UART/DMA 时序、921600 误码、桥接端到端时延与抖动；
- `sensor_imu_inject()` 的并发 FIFO（1 kHz 生产者/消费者）在目标上的表现；
- 估计器在 HIL 输入速率下的收敛、NIS 与重放预算；
- 目标机端到端联调与飞行安全。

这些项在 `doc/simulation/HIL_LINK.md` 第 6 节按 `NOT_RUN` 记录，不得据本测试
宣称 HIL 通过或飞行放行。
