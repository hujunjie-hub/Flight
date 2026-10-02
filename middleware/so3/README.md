# middleware/so3 — SO(3) 姿态误差

对 KF-GINS 融合姿态计算 SO(3) 姿态误差：**当前姿态**实时读取
`gins_bridge_get_solution()`（middleware/gins 的融合快照，roll/pitch/yaw，deg，
即 USART1 `gins_fused_data` 链路同源数据，行尾/输出标记 `fused_data`），
**期望姿态**由上层（制导/遥控/任务）经 API 设定，输出可直接喂控制律的
体轴系误差 `e_b` 与用于记录/回传的导航系失准角 `e_n`。

## 文件

| 文件 | 说明 |
|------|------|
| `so3.h` / `so3.c`       | 纯 SO(3) 数学：欧拉/DCM/四元数互转、Exp/Log、姿态误差（无 RT-Thread 依赖） |
| `so3_gins.h` / `so3_gins.c` | 桥接：当前姿态<-KF-GINS、目标姿态设定（关中断快照）、一键误差 + FinSH `so3` |
| `test/rtthread.h`       | 主机测试用 RT-Thread 头桩（同 protocol/test 做法） |
| `test/tc_so3.c`        | utest 单元测试（板上运行）：独立参考实现交叉验证 + 解析用例 |

## 坐标系与姿态约定（与 KF-GINS 一致）

- 导航系 n = **NED**，体坐标系 b = **FRD**（前右下）；
- 欧拉角 rpy = [roll, pitch, yaw]，ZYX（3-2-1）顺序：`C_bn = Rz(y)·Ry(p)·Rx(r)`，
  即 KF-GINS `Rotation::euler2matrix` 的体->导航阵；
- 四元数 Hamilton 约定 (w,x,y,z)，`R(q1⊗q2)=R(q1)R(q2)`，与 Eigen 一致。

## 姿态误差定义与坐标变换（核心）

设当前姿态 `R = C_bn`，期望姿态 `R_d = C_bd_n`：

```
导航系误差 (失准角):  e_n = Log( R · R_dᵀ )          [NED 轴系]
体轴系误差 (控制):    e_b = Log( R_dᵀ · R )          [期望体轴 FRD]
精确坐标变换:         e_b = R_dᵀ · e_n               (so3_err_nav_to_body)
```

`e_b` 与 Lee 几何控制的 `e_R`、PX4 的 `q_error` 同款（误差在期望体系表达，
小误差时与当前体系二阶一致），控制律直接 `τ = -K·e_b`。

两条常见错误，本模块已规避：

1. **欧拉角逐分量相减不是姿态误差**。欧拉角差不属于任何坐标系。
   例：水平姿态、航向 100° 时 1° 的纯偏航差，体轴误差是
   `[0°, 0.98°, -0.17°]`——误差几乎全落在俯仰/滚转轴，直接拿 yaw 差喂
   偏航通道会彻底耦合错。需要欧拉差形式时用
   `so3_euler_err_to_body/_nav`（T 阵变换：单轴分量与精确解严格一致，
   组合误差 O(Δ²)）。
2. **e_n 与 e_b 差一个姿态阵旋转**（上式精确关系），不是同一向量的两种写法；
   两套结果混用前必须经 `so3_err_nav_to_body / so3_err_body_to_nav` 互算。

## API

纯数学（so3.h）：

| 函数 | 说明 |
|------|------|
| `so3_euler_to_dcm/_to_quat` | rpy(rad) → C_bn / 四元数 |
| `so3_dcm_to_euler`、`so3_quat_to_*` | 反向转换（含俯仰奇点保护；yaw 输出 (-π,π]） |
| `so3_exp` / `so3_log` | 旋转矢量 ↔ 四元数；Log 取最短旋转，角度 ∈ [0,π] |
| `so3_att_error_quat/_dcm` | **姿态误差**：输出 `e_b`、`e_n`、`angle` |
| `so3_err_nav_to_body/_body_to_nav` | 误差旋转矢量跨系精确互算 |
| `so3_euler_err_to_body/_nav` | 欧拉角差 → 误差旋转矢量（T 阵，小角） |
| `so3_wrap_angle`、`so3_euler_diff` | 角度归一 [-π,π)、欧拉差归一（仅用于显示） |

桥接（so3_gins.h）：

| 函数 | 说明 |
|------|------|
| `so3_target_set_rpy_deg/_rad/_quat` | 设定期望姿态（任意线程，关中断发布；yaw 连续值或缠绕值均可） |
| `so3_target_get / so3_target_clear` | 读目标快照 / 清除 |
| `so3_current_quat / so3_current_rpy_rad` | KF-GINS 最新融合姿态（未就绪返回 RT_FALSE） |
| `so3_att_error` | 当前<-GINS、期望<-target，一步得 `struct so3_att_err` |

控制律典型用法：

```c
struct so3_att_err err;

if (so3_att_error(&err))            /* RT_FALSE = GINS 未就绪/目标未设, 输出已清零 */
{
    /* err.e_b: 体轴误差旋转矢量 (rad), 姿态环直接消费 */
    torque_x = -kp_roll  * err.e_b[0];
    torque_y = -kp_pitch * err.e_b[1];
    torque_z = -kp_yaw   * err.e_b[2];
}
```

## FinSH 调试

```
so3                        打印当前姿态/目标姿态/误差 (e_b 体轴, e_n 导航系, deg)
so3 target <r> <p> <y>     设定目标姿态 (deg), 如: so3 target 0 0 90
so3 clear                  清除目标
```

上板验证：`so3 target 0 0 0` 后手动倾斜机体，`e_b` 三轴应按倾斜方向变化，
`angle` 为总偏差角。

## 构建与测试

固件：`middleware/SConscript` 自动扫描子目录，`so3/SConscript` 无需配置即编入；
`so3.c` 仅依赖 libc 的 math（同 um982_nmea.c 先例）。

单元测试（utest，板上运行，与固件同一份源码；当前姿态快照由
`so3_test_inject()` 注入，代替主机版的假 `gins_bridge_get_solution`）：

```bash
msh> utest_run middleware.so3
```

测试覆盖：互转往返、Exp/Log 全量程（含 >π 主值回绕、零姿态）、
`e_b = R_dᵀ·e_n` 精确关系双路径交叉验证、yaw 缠绕（±179°/连续 400°）、
欧拉差 T 阵单轴严格一致 + 组合小角偏差 O(Δ²)、俯仰 90° 奇点、
gins 桥接 deg→rad / 未就绪 / 未设目标分支。当前 129 例全部通过。

## 资源与精度

- double 精度（与 KF-GINS 链路一致）；Cortex-M7 软浮点下单次
  `so3_att_error()` 约 2 次四元数乘 + 2 次 Log，量级在百 µs 内，
  1kHz 姿态环可承受，后续如紧张可整模块 float 化（结构已按此设计）；
- 固件 ROM 占用约 5KB（-O2，含 FinSH 命令）；
- 目标姿态快照用关中断拷贝（同 gins_bridge 模式），任意线程/优先级安全。
