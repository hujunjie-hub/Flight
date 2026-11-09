# 独立 IMU 仿真通道（Host/SIL）

`tests/support/sensor_sim/` 生成 **原始器件帧**（ADIS16505-2 32 字节 burst），
先由器件驱动解码，再交给 `middleware/sensor/sample_normalization` 的样本构建代码处理。仿真器与解码器不共享代码，
避免“编码器与解码器一起错”的假通过（spec 05 §6.2、§10 第 3 步）。

> **SIM_ONLY：** 本目录**不得**进入 H723 固件镜像（spec 01 §SIM_ONLY、09 REQ-SIM-001）。
> 它没有 `SConscript`，只由显式的 Host 命令构建。

## 构建与运行

```powershell
pwsh tests/sensor_processing/run_host_tests.ps1
```

## 标准样本

| 文件 | 场景 | 内容 |
|---|---|---|
| `samples/imu_static_5x200hz.json` | 静止 + 20 mg 确定性振动、25 °C | 5 帧原始 burst + 生成端物理值 |
| `samples/imu_rotation_5x200hz.json` | 绕 Z 90 °/s、比力 (0.35, −0.20, −9.79) m/s²、−10 °C | 5 帧原始 burst + 生成端物理值 |

格式：`flight-standard-sample/1`；`frame_hex` 为 32 字节 burst（含 16-bit 大端校验和），
可用 Host 夹具接到器件解码与标准样本构建链做回放。样本是**生成物**，重新生成后应逐字节一致
（确定性，无随机源）。

## 覆盖范围

- 静止/旋转/振动三种输入，逐样本校验校验和、量程换算、温度与计数器；
- 与器件解码及样本构建的往返误差限制在半个量化步长 + 一个 float ulp；
- 尚未覆盖：IMU 缺样/重复/时间倒退（由 `kf_gins_quad` 的 `ImuIncrementBuilder` 负责）、
  GNSS/Baro 仿真通道（后续 ADAPTER/REPLAY 任务）。
