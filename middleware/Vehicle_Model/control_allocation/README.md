# middleware/Vehicle_Model/control_allocation — 控制分配 (混控)

`T_d`(总推力 N) + `alpha_b`(角加速度指令 rad/s²) ——attitude_so3 内环输出——
→ 四电机推力 `f[4]`(N) 与归一化 `u[4]`∈[0,1] (喂
[dshot_output](../dshot_output/README.md))。

## 文件

| 文件 | 说明 |
|------|------|
| `mixer.h` / `mixer.c`    | 纯 C 核心: τ=J·α → quad-X 解析逆 → 逐电机饱和; 无 RT-Thread 依赖, 可主机测试 |
| `mixer_cmd.c`            | 桥接: 取内环最新输出 `mixer_cmd_step_att()` + FinSH `mix` (调参/单步) |

## 方法 (公式推导见 mixer.h 头注)

quad-X 固定布局 (m0 前右 CW / m1 前左 CCW / m2 后右 CCW / m3 后左 CW),
`τ = diag(J)·α` 后按解析逆分配到四电机 (`Mx=√2τx/l` 等, 见头注公式),
逐电机 clamp `[f_min, f_max]` 并记录饱和计数。**未做推力/姿态优先级
协调** (简单裁剪; 上游 PID 已有抗饱和), 需要时再升级。

## ⚠ 参数全部为占位值 (硬件方案落地后整定)

`arm_l`(力臂 0.11m, 轴距=l·√2)、`tau_coeff`(反扭矩系数 0.015 N·m/N)、
`inertia`(J 对角 2.0/2.0/3.5e-3 kg·m²)、`f_min/f_max`(0.2/3.0 N) 均按
500g 级四旋翼量级**随意取的占位值**, 未按实际机架测量。落地后:

```
mix                  # 查看 (标注"占位")
mix set l 0.13       # 力臂 m (轴距/√2)
mix set cq 0.012     # 反扭矩系数 (τ_z 每单位推力)
mix set j 2.1e-3 2.1e-3 3.8e-3   # 转动惯量对角
mix set fmin 0.15 / fmax 3.5     # 单电机推力界 (按静推力实测)
mix step             # 单步: 取内环最新 (thrust,alpha) 打印 f/u
mix default
```

J/c_q 可由机架摆动实验或 CAD 估算; f 界由电机+桨静推力台架实测。
参数仅 RAM (持久化未接, ctrl 分区已预留)。

## 电机-通道映射

与 dshot_output 占位引脚表一致: m0→TIM4_CH1, m1→CH2, m2→CH3,
m3→CH4 (引脚同样占位, 见 dshot_output/dshot_hw.h)。换向: 电机转向由
布局符号表承载, 装反/换向时改 mixer.h 头注符号表 + 重验, 或现场用
`dshot raw 7/8` (DSHOT_CMD_SPIN_DIR) 反转电调。

## 验证

`build_host/mpc_xcheck.py`: 正映射重建往返 (Σf=T, A·f=J·α, 80 随机样本
≤2e-16)、悬停分解 T/4、上下界饱和全触、NaN 输入拒绝。2026-10-03 全过。
