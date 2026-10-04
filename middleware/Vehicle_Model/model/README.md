# model/ — 四旋翼控制模型 (quad_model)

控制链执行顶层 (2026-10-03): 把 position_mpc / attitude_so3 /
control_allocation / dshot_output 四个纯 C 级联件组装成可运行的四旋翼
控制模型, 结构移植自 FMT 的 "controller 模型 + vehicle 任务调度"
(对照表见 `quad_model.h` 头注与 FMT_README §12.3)。

## 组成

| 文件 | 内容 |
|---|---|
| `quad_model.h` | 对外接口: 状态机 (DISARM/STANDBY/ARM)、模型信息 (内外环双周期)、全链快照 `quad_model_out`、hold/arm/disarm/power API |
| `quad_model.c` | "ctl" 线程 (prio 8, IMU 事件驱动): 20Hz MPC 外环 + 500Hz SO(3)/PID 内环 + 混控 + 输出门控, FMT PERIOD_EXECUTE3 同型的 ms 时间门控分频 |

## 安全约定 (默认双重锁定)

- 上电 DISARM + **dry-run**: 不初始化输出硬件 (dshot 引脚/TIM 占位),
  闭环结果只进快照 —— 台架 `quad hold` → `quad arm` 即可全程观察
  `u_norm[4]` 序列而不接电调;
- `quad power on` 显式使能输出引擎后才可能真出力; ARM 期连续 25 拍
  (50ms) 内环/混控无有效输出自动 disarm 停转。

## 数据来源

gins 解算快照 + IMU 陀螺 (经 `mpc_pos_gins` / `att_pid_gins` /
`mixer_cmd` 三个桥接层取数, 各自的 FinSH 调参命令 `mpc`/`att`/`mix`
作用于同一条链)。期望值: `quad hold` 捕当前位置+航向为悬停目标;
设定点微调走 `mpc` 命令; FMS/GCS 期望值通道为后续项 (见根 README
飞行控制章与 FMT_README §12.3/§13)。
