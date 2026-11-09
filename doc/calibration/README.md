# doc/calibration — 标定原始证据与数据集

本目录是 **B 域（M2 语义）** 标定证据的唯一归档位置，对应规范
`05 §7.1`（"原始标定数据和报告只放 `doc/calibration/`"）与
`08 §4`（"`doc/calibration/` 保存原始报告和数据集；最终有效杆臂/安装外参只有
`sensor_alignment.yaml` 一个运行时来源"）。

## 1. 这里放什么 / 不放什么

| 放这里 | 不放这里 |
|---|---|
| 原始测量：静止窗口、转台/轴向旋转、温度循环、磁环境（电机停转/起转） | 运行期参数（在 `config/*.yaml`） |
| 标定报告：方法、设备、环境、日期、操作者、判定与残差 | 第二份杆臂/安装旋转副本（禁止） |
| 回放数据集与处理脚本、数据 hash 清单 | `sensor_alignment.yaml` 的副本或"临时"参数表 |
| 与本次标定绑定的 `calibration_version` 记录 | 飞控运行日志/遥测原始文件（按 06/09 另行归档） |

数据一旦被 `sensor_alignment.yaml` 或 `estimator_tuning.yaml` 引用，必须能用
`calibration_dataset_hash` 追溯到本目录内的不可变原件。

## 2. 目录约定

```text
doc/calibration/
  README.md                     # 本文件
  imu/                          # ADIS16505-2：轴序/符号、零偏、比例因子、失准角、pipeline 延迟
  gnss/                         # UM982：天线杆臂、PPS/epoch 映射、到达时延统计
  baro/                         # BMP585：压力基准、温度漂移、桨流
  mag/                          # BMM350：硬/软铁、安装旋转、电机干扰对比
  <sensor>/<YYYYMMDD>-<topic>/  # 单次标定：原始数据 + report.md + manifest.json
```

每个 `<YYYYMMDD>-<topic>/manifest.json` 至少包含：

```json
{
  "sensor": "imu|gnss|baro|mag",
  "topic": "axis-order | lever-arm | pipeline-delay | hard-soft-iron | ...",
  "date": "YYYY-MM-DD",
  "operator": "...",
  "method": "...",
  "environment": {"temperature_c": 0.0, "notes": "..."},
  "files": [{"path": "raw.csv", "sha256": "...", "bytes": 0}],
  "calibration_version": 1,
  "result_ref": "report.md#<section>",
  "status": "COMPLETE | INCOMPLETE | SUPERSEDED"
}
```

## 3. 需要的最小标定集（对应 05 §4.2/§7.1）

| 项目 | 方法 | 影响的运行参数 |
|---|---|---|
| IMU 轴序/符号、静止重力方向 | 水平静止，三轴重力响应 | `sensor_alignment.yaml: imu.installation_rotation_wxyz` |
| IMU 安装旋转 | 已知轴向旋转/转台 | 同上 |
| IMU 零偏/比例因子/失准角 | 静止 + 多位置/转台 | `calibration_version` + 估计器初值 |
| IMU pipeline/group delay | DRDY 与物理激励的相位/阶跃响应 | `imu.pipeline_delay_s` |
| GNSS 天线杆臂 | 已知基线/旋转法 | `gnss.antenna_lever_m` |
| GNSS epoch/PPS 映射与到达时延 | PPS 捕获 + 时间消息对比 | `gnss.epoch_offset_s`、M2 age 门限 |
| BMP585 压力基准/温漂/桨流 | 压力腔/静止 + 电机台架 | `baro.reference_pressure_pa`、噪声 floor |
| BMM350 硬/软铁与干扰 | 无磁转台 + 电机停转/起转对比 | `mag.hard_iron_ut`、`soft_iron_matrix`、`aiding` |

## 4. 状态（2026-11-04）

- 目录与约定已建立（关闭复核项 **B-R06**），**尚无任何实测数据**：`sensor_alignment.yaml` 的
  对应字段仍为 `TBD`，按 08 §4 阻止 Release 构建。
- 下一步（需 A 的原始通道与台架）：先做 IMU 轴序/重力与 pipeline 延迟两项，再做 GNSS 杆臂与
  磁标定；每项完成后更新 manifest 与 `calibration_version`，并重跑 `estimator_tuning.yaml`
  的数据集辨识。
