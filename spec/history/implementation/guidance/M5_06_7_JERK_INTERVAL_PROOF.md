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
