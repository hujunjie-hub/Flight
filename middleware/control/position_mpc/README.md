# middleware/control/position_mpc — 位置 MPC (外环)

NED 位置/速度 → 期望加速度 `a_cmd` (m/s²), 作为内环
[attitude_so3](../attitude_so3/README.md) 的推力矢量输入。
每轴独立双积分器模型的**凝结式模型预测控制**, 框约束 QP 用序贯坐标法
(SCA) 求解 —— 迭代点恒可行、扫描次数上限固定 (单拍耗时确定)。

## 文件

| 文件 | 说明 |
|------|------|
| `mpc_pos.h` / `mpc_pos.c`       | 纯 C 核心: 闭式预计算 (H/M + Riccati 终端权) + SCA 求解 + step。无 RT-Thread 依赖, 可主机测试 |
| `mpc_pos_gins.h` / `mpc_pos_gins.c` | 桥接: 经纬高→本地 NED (等距圆柱), 设定点管理 (NED 与 LLA 两个入口, LLA 带相对当前位置 h50/v10 m 钳位 —— QGC DO_REPOSITION 通路), 单步入口 + FinSH `mpc` |

## 方法摘要 (公式与推导见 `mpc_pos.h` 头注)

- 模型 (逐轴): `p' = v, v' = u`; ZOH 离散。状态 [p,v]×3, 输入 u×3。
- 代价: 位置/速度跟踪 (Qp/Qv) + 输入惩罚 (Ru) + **LQR 终端权 P**
  (每轴 2 维 Riccati 迭代收敛, 与 (Qp,Qv,Ru) 自洽)。
- 凝结 QP: `min ½UᵀHU + gᵀU`, `g = M(x0 − x_ref)`, H 正定;
  H/M 在 `mpc_pos_setup` 闭式预计算 (改 N/dt/权重后须重调)。
- 约束: `u_min ≤ u_k ≤ u_max` 逐轴 (z 轴向下为正: 上升加速度是负值)。
- 求解: SCA (投影 Gauss-Seidel), 热启动 = 上一拍解左移; 投影梯度 < tol
  提前收敛, 到上限未收敛输出当前可行解并计数 (`stall_cnt`)。
- 输出: `a_cmd = u_0 + a_ff` (前馈不进预测模型, 整体 clamp)。

缺省 N=10 @ dt=0.05 (20Hz, 0.5s 视界); 权重为安全保守初值,
闭环带宽 ≈ (q_p/r)^(1/4) 量级 (缺省 ~1.6 rad/s), 现场按机体调参。

## 板上单拍预算

N=10: 三轴各 10 维 QP, 每 sweep O(N²) 双精度乘加, 上限 60 sweeps
(实测典型几 sweep 收敛)。全程无动态内存, `mpc_pos_ctx` 约 8.3KB
(板上实例 g_mpc 实测, 按 N_MAX=16 静态上限; N_MAX 降到 10 可再省 ~4.5KB)。

## FinSH `mpc`

参数含义/缺省值/调参顺序的权威清单见根 README "调参清单" 章
(缺省值源码: `mpc_pos.h` 的 `mpc_pos_cfg_default`)。

```
mpc                      # 配置/设定点/最近一拍输出与统计
mpc set N 12             # 改预测步数 (重预计算)
mpc set dt 0.02          # 改外环周期 (控制任务须同步改调用频率)
mpc set qpos 2 2 4       # 向量型权重/界: qpos qvel racc umin umax
mpc pos 5 -3 -1          # 设定点 (NED m, D 向下为正; 首次自动捕参考点)
mpc vel 1 0 0            # 速度设定 (NED m/s)
mpc aff 0.5 0 0          # 前馈加速度 (m/s²)
mpc ref                  # 把当前位置捕为本地坐标系原点
mpc step                 # 单步台架验证 (打印 a_cmd/sweeps/res)
mpc reset | default      # 清热启动 / 恢复缺省配置
```

## 验证

`build_host/mpc_xcheck.py`: 主机 gcc (CLion MinGW) 直接编译**固件源码本体**
跑确定性用例, numpy 独立构造对拍 —— H/M 与稠密 Φ/Γ 构造一致 (≤1e-12
相对), QP 解满足框约束 KKT (最优性充要条件), 闭环 600 拍收敛到 1e-15,
NaN 输入拒绝。2026-10-03 首版全过。

## 未完成 / 边界

- 控制任务未建 (由未来 ctl 任务按 1/dt 周期调 `mpc_pos_gins_step`);
- 增益不持久化 (param_part 已预留 ctrl 分区, 落地后接入);
- 设定点恒值预测, 无轨迹前瞻 (制导接入时扩 ref 序列)。
