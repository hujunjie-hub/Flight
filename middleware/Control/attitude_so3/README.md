# middleware/Control/attitude_so3 — SO(3)/PID 姿态控制 (内环)

期望加速度 (外环 [position_mpc](../position_mpc/README.md) 输出) + 期望航向
→ 总推力 + 期望姿态 + 姿态误差 SO(3) 串级 PID → 角加速度指令。

## 文件

| 文件 | 说明 |
|------|------|
| `att_pid.h` / `att_pid.c`       | 纯 C 核心: 推力矢量→期望姿态 (含倾斜限幅) + 串级 PID。仅依赖同层 `Control/so3` 的纯数学, 可主机测试 |
| `att_pid_gins.h` / `att_pid_gins.c` | 桥接: KF-GINS 姿态/IMU 陀螺/外环 a_des 采集, 期望姿态发布到 so3_target + FinSH `att` |

## 控制律结构 (一拍)

```
a_des (NED) ──▶ z_b_des = unit(g_vec − a_des)     倾斜限幅 (30° 锥, 超界投影回边界)
                f      = m·‖g_vec − a_des‖          总推力 (按未限幅方向计算)
yaw_des ─────▶ R_des:  y_b = unit(z_b × x_c), x_b = y_b × z_b   (Lee/PX4 标准构造)
                         │
                         ▼  e_b = Log(R_desᵀ·R_cur)   SO(3) 体轴误差 (control/so3)
角度环 P     ω_des = Kp_att · e_b                    (逐轴限幅)
角速度环 PID  α = Kp·e_ω + Ki·∫e_ω − Kd·dω/dt|LPF    (D 项作用于测量, 30Hz 低通;
                                                      条件积分抗饱和: 饱和方向冻结)
```

关键语义与安全性质 (`att_pid.h` 头注有完整约定):

- **`alpha_b` 是角加速度指令 (rad/s², 期望体轴系)**, 与转动惯量解耦;
  混控层按机体惯量换算电机差动力矩。
- `valid=0` (输入非有限/gins 未就绪) 时输出全零并复位控制器状态 ——
  电机层必须停转, 不得把 0 当悬停。
- 倾斜限幅保证期望姿态总在锥内 (推力大小不受限幅影响), R_des 恒正交。

## 数据来源 (桥接层)

- 当前姿态: `so3_current_quat()` (KF-GINS 融合, 与 `gins_fused_data` 链路同源);
- 角速度: `imu_data_peek_latest()` 陀螺 (FRD rad/s);
- dt: 相邻 IMU 样本 `T_event` 差 (DR 硬件时戳, 与线程抖动无关),
  超窗 (>10ms/跳变) 退回标称 2ms;
- a_des: `mpc_pos_gins_last_accel()`, 外环无有效输出时取 0 (悬停姿态目标);
- q_des 同拍发布到 `so3_target` —— `so3` 命令可直接观察 e_b/e_n, 链路同源。

## FinSH `att`

参数含义/缺省值/调参顺序的权威清单见根 README "调参清单" 章
(缺省值源码: `att_pid.h` 的 `att_pid_cfg_default`)。

```
att                      # 配置/最近一拍输出 (期望姿态/误差/推力/alpha)
att set mass 1.2         # 质量 (推力换算)
att set katt 5 5 3       # 角度环 P (1/s); 向量型: katt kpr kir kdr amax
att set kpr 30 30 12     # 角速度环 P (1/s)  [kir I, kdr D 同型]
att set tilt 35          # 倾斜限幅 (deg)
att set dcut 30          # D 项低通截止 (Hz); omax ω限幅 / amax / thrmin thrmax ihold
att yaw 90               # 期望航向 (deg)
att hold                 # 航向保持 (捕当前航向)
att step                 # 单步台架验证
att reset | default      # 清积分器/D 历史 / 恢复缺省
```

## 验证

`build_host/mpc_xcheck.py` (与 position_mpc 共用): R_des 正交性、第三列
== 推力方向、倾斜限幅锥角精确性、航向不变量 (y_b ⊥ x_c, 水平时航向精确)、
推力大小、e_b 与独立四元数 Log 一致 (≤2e-16)、ω_des/α 限幅与关系式、
零误差零输出、饱和抗积分。2026-10-03 首版全过。

## 未完成 / 边界

- 控制任务未建 (建议内环 500Hz: `att_pid_gins_step()` 挂 IMU 数据流);
- 混控 (alpha_b/thrust → 电机) 与电调接口未建;
- 增益为保守占位 (500g 级四旋翼量级), 须现场调参; 不持久化
  (param_part 已预留 ctrl 分区);
- Kp_att·e_b 是 Lee 几何控制的 P 项; 需要时可在角速度环加 ω_ff 前馈。
