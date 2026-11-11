# M5-06.7 R15 连续 jerk 区间证明记录

状态：**实现与本地 Host/SIL 验证记录**，不是独立数学审查 APPROVED。
适用范围：QUAD_X4 / Host/SIL M5-06 validator；不构成 H723 目标运行、WCET/RTA、HIL 或飞行证据。

## 1. 五次 Hermite 的 Bernstein 形式

对一条消费区间，设段时长 `d > 0`，局部参数 `u in [0,1]`：

```text
p(u) = sum_{i=0..5} B_i * C(5,i) * u^i * (1-u)^(5-i)
```

其中 `B_i` 是五次 Bernstein 位置控制点。生产 validator 还保存每个控制点的
向外区间 `[B_i.low, B_i.high]`，用于覆盖 binary64 系数构造和 MR-009 的
consumer 误差。

## 2. 三阶导

五次 Bernstein 的三阶导为二次 Bernstein：

```text
j(u) = (60 / d^3) * sum_{k=0..2} C(2,k) u^k (1-u)^(2-k) * Δ^3 B_k
Δ^3 B_k = B_{k+3} - 3B_{k+2} + 3B_{k+1} - B_k
```

凸包性质给出原区间 `[0,1]` 的严格上界：

```text
|jerk| <= (60 / d^3) * max_{k=0..2} |Δ^3 B_k|
```

R14 初版误写为 `6/d^3`，可把上界低估 10 倍；R14b 修正为 `60/d^3`，R15 保留。

## 3. Bernstein 到幂基

二次 Bernstein 多项式

```text
b(u) = b0(1-u)^2 + 2b1 u(1-u) + b2 u^2
```

的幂基形式为：

```text
b(u) = p0 + p1 u + p2 u^2
p0 = b0
p1 = 2(b1 - b0)
p2 = b0 - 2b1 + b2
```

R14b 的尾段直接把 `b0,b1,b2` 当幂基系数（`((b2 u + b1)u + b0)`），这是错误
的；R15 使用上式，并用区间运算计算 `p0,p1,p2`。

## 4. 扩展时间区间

消费端时间误差满足：

```text
|s_c - s_exact| <= gamma_2 * s_exact,  gamma_2 = 2u/(1-2u), u=2^-24
```

子区间结束的物理时间为 `s_end = end_time`，局部归一化坐标为 `1`。扩展尾段：

```text
ratio = end_time / duration
u_tail = 1 + gamma_2 * ratio
```

`u_tail` 不是 `ratio*(1+gamma_2)`。后者是错误坐标映射，可能覆盖过多或过少的
区间。尾段使用区间 Horner 在 `[1,u_tail]` 上求值，再与原区间凸包取最大值。

## 5. Consumer binary32 多项式

M6 `control_sample_reference()` 使用 binary32 系数：

```text
j(s) = 6*c3 + 24*c4*s + 60*c5*s^2
```

`c3/c4/c5` 由真实 C 系数 API 返回的 binary32 值精确转成 Fraction。生产
validator 在物理时间 `[start_time, end_time + gamma_2*end_time]` 上用区间
Horner 求上界。该上界与 validator 的 Bernstein 原区间、幂基尾段取最大值。

三者关系：

1. Bernstein 原区间覆盖 `u in [0,1]` 的数学曲线；
2. 幂基尾段覆盖 `u in [1,u_tail]` 的扩展坐标；
3. consumer 多项式覆盖真实 binary32 系数在物理时间上的 jerk；
4. 任一区间无效（非有限、除法/乘法溢出、无效控制点）都返回
   `M5_SAFETY_NUMERIC_FAILURE`，不返回 PASS。

## 6. 独立 Fraction 最大值的构造

对二次多项式 `j(s)=a0+a1 s+a2 s^2`，在闭区间 `[s0,s1]` 上的最大绝对值由：

- `|j(s0)|`；
- `|j(s1)|`；
- 若 `a2 != 0` 且 `s* = -a1/(2a2) in [s0,s1]`，`|j(s*)|`；

三者取最大。S06-112 的 `j(s)=6c3+24c4s+60c5s^2` 对应
`a1=24c4`、`a2=60c5`，所以驻点为 `-24c4/(120c5) = -c4/(5c5)`。
所有运算使用 Fraction，不引入浮点容差。

## 7. R15 验证结果

- S06-112：checked=360，tail_cases=360，fail_closed=20，0 failures。
- 生产 bound 对每个可计算案例均 `>=` Fraction 连续最大值（无容差）。
- 旧 Bernstein-as-power 公式在 300 个可比较尾段案例中有 248 个欠界，作为
  负向对照。
- R14b 反例：`h=0.02, a0=+0.03, a1=-0.03`，真实 max|jerk|=18 m/s³；16 m/s³
  产品限制必须拒绝。

## 8. 边界

- 不把有限采样当作连续证明；连续最大值来自解析端点/驻点。
- 不把本记录当作 MR-009 APPROVED；MR-009 仍为 WAITING_INDEPENDENT_REVIEW。
- 不把 Host/SIL 结果当作 H723 实时、栈水位、HIL 或飞行结果。
- 不改变产品 jerk 限制（当前 QUAD_X4 为 16/16/8 m/s³）。

## 9. R16 三类上界的独立记录

R16 在 `M5SafetyCurveBounds` 中分开保存：

```text
jerk_bernstein_bound  : Bernstein 原区间 [0,1] 凸包
jerk_tail_bound      : 幂基尾段 [1, u_tail]
jerk_consumer_bound  : consumer binary32 多项式 [s0, s_tail]
jerk_axis_bound      : 上述三者最大值
```

`m5_trajectory_safety_test_curve_jerk_components()` 分别返回四组值。S06-112
对每类界单独检查：Bernstein 对 `[s0,s1]` 的 Fraction 最大值、尾段对
`[s1,s_tail]`、consumer 对 `[s0,s_tail]`；component_checks=1080，0 failures。
`min_slack` 非负；零曲线的最小 slack 为 binary32 最小次正规数（向上取整）。

S06-113 不改变本证明；它验证时间转换的注入误差，不是 jerk 证明的一部分。

## 10. R18 局部子区间 Bernstein 修正

R17 的负向测试使用**整段** Bernstein 控制点，却用局部子区间坐标 `u_tail`，二者
不匹配。R18 改为从幂基系数构造局部子区间控制点：

```text
t0 = h*u0, dt = h*(u1-u0)
q_k = sum_{i=k..5} c_i * C(i,k) * t0^(i-k) * dt^k
B_j = sum_{k=0..j} q_k * C(j,k) / C(5,k)
J_k = 60/dt^3 * (B[k+3]-3B[k+2]+3B[k+1]-B[k])
```

旧错误公式 `J0+J1*u+J2*u²` 在 `[1,u_tail]` 全闭区间求最大绝对值，J2≠0 时纳入
`u* = -J1/(2J2)`（位于区间时）。

实际统计（不再沿用 R16 的 360/1080）：

```text
candidate_cases=160, legal_cases=140, fail_closed_cases=20
axis_cases_checked=420
bernstein_checks=420, tail_checks=420, consumer_checks=420, total_checks=420
wrong_tail_checked=420, wrong_tail_underbound=122
```

新增真正零 jerk 状态（位置/速度/加速度全零，X/Y/Z）和独立恒加速度非零状态。
60/d³、position_low/high、正确 Bernstein-to-power 与 consumer binary32 上界保持。

## 11. R20 测试状态与生产状态分离

测试 API 使用 `M5_TEST_JERK_STATUS_*`（0 OK、1 INVALID_INPUT、2 BUILD_NUMERIC、
3 BOUNDS_NUMERIC），不得与生产 `M5_SAFETY_INVALID=3`、`M5_SAFETY_NUMERIC_FAILURE=11`
混用。合法样本要求 status=0；只有 subnormal 接受 2/3；invalid input 要求 1。
生产路径另测：动态极端与 subnormal jerk 夹具均返回 `M5_SAFETY_NUMERIC_FAILURE`
且 `reason_code=M5_SAFETY_REASON_NUMERIC`。
