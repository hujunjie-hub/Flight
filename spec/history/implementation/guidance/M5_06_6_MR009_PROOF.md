# MR-009 独立数学推导：binary32 消费曲线误差包络

状态：**PROOF_RECORD / WAITING_INDEPENDENT_REVIEW**。本文给出 `m5_trajectory_safety.c` 中误差包络的完整推导，并逐项核对源码中的实现。结论只有独立审查者 APPROVED 后才生效。

## 1. 被证明的对象

M6 控制器实际执行的曲线（`middleware/model/control/outer_loop/quad_x4/position_tracking/control_reference.c`）：

```c
s  = (float)(offset - a->offset_us) * 1e-6f;      /* binary32 */
h  = step_us * 1e-6f;                             /* binary32 */
d  = (p1 - p0) - h*v0 - .5f*h*h*a0;               /* binary32 */
v  = (v1 - v0) - h*a0;                            /* binary32 */
ac = a1 - a0;                                     /* binary32 */
c0 = p0; c1 = v0; c2 = .5f*a0;
c3 = 10*d/(h*h*h) - 4*v/(h*h) + ac/(2*h);
c4 = -15*d/(h*h*h*h) + 7*v/(h*h*h) - ac/(h*h);
c5 = 6*d/(h*h*h*h*h) - 3*v/(h*h*h*h) + ac/(2*h*h*h);
pos = ((((c5*s + c4)*s + c3)*s + c2)*s + c1)*s + c0;
vel = (((5*c5*s + 4*c4)*s + 3*c3)*s + 2*c2)*s + c1;
acc = ((20*c5*s + 12*c4)*s + 6*c3)*s + 2*c2;
```

验证器要证明的不等式（对每个子区间、每个轴、每个量）：

```
|consumer_float(s) − exact_real(s)| ≤ E(s_max)        对全部 s ∈ [0, s_max]
```

其中 `exact_real` 是同一组 **binary32 端点** 上的五次 Hermite 精确实数曲线，`s_max = h·u1` 是该子区间的绝对时间上界。

## 2. 引理（Higham 标准结果）

设浮点运算满足 `fl(x ∘ y) = (x ∘ y)(1 + δ)`，`|δ| ≤ u = 2^-24`，且运算过程中**无溢出、无下溢到次正规区**。定义

```
γ_n = n·u / (1 − n·u)
```

则：

1. **Horner 链**：次数 n 的多项式用 Horner 求值需要 n 次乘法 + n 次加法，共 2n 次舍入，满足
   `|p̂(x) − p(x)| ≤ γ_{2n} · Σ_i |c_i| |x|^i`。
2. **超可加性**：对 `a,b ≥ 1`，`γ_a + γ_b ≤ γ_{a+b}`（`n·u < 1` 时成立）。
3. **单项缩放**：`|fl(k·c) − k·c| ≤ u·|k·c|`，即一个 `γ_1`。

推导：逐次展开 `(1+δ_i)` 的乘积，`∏(1+δ_i) ≤ 1 + γ_n`（当 `n·u < 1`），再对绝对值求和。

## 3. 系数项：为什么用区间而不是标称值

验证器先构造精确系数区间 `c_i = [low_i, high_i]`（向外舍入的 interval arithmetic，基于同一组 binary32 输入与精确实数 `h`、`u0/u1`）。消费端系数 `cf_i` 是 binary32 计算的浮点数，于是

```
|c f_i − c_true_i| ≤ max(|cf_i − low_i|, |cf_i − high_i|) =: Δ_i
```

这一步不需要 γ 模型：区间已经包含精确实数系数，`cf_i` 是确定浮点数，最大距离是精确可算的（用 `nextafter` 向外）。

**velocity / acceleration 的“系数”是缩放后的量**：消费端计算 `5*c5`、`4*c4`、… 与 `20*c5`、`12*c4`、… 时各自有 1 次舍入。验证器用 `interval_scale(c_i, k)` 得到 `k·c_true` 的区间，再把 `fl(k·cf_i)` 与它比较——缩放舍入同样被 `Δ_i` 覆盖。

因此系数项为

```
E_coef = Σ_i Δ_i · s_max^i
```

源码对应 `consumer_eval_error_bound()` 的第一个累加项 `coefficient_term`，其中 `power` 用 `multiply_upper_nonnegative` 逐步向外。

## 4. Horner 项：逐量运算计数

`cf_i` 已是消费端最终使用的 binary32 系数（velocity/acceleration 为缩放后的值），Horner 只在其上做乘加：

| 量 | 表达式 | 乘法 | 加法 | n | 采用的 γ_n |
|---|---|---:|---:|---:|---|
| position | 5 级 Horner，系数 c5..c0 | 5 | 5 | 10 | `γ_10` |
| velocity | 4 级 Horner，系数 (5c5)..c1 | 4 | 4 | 8 | `γ_12` |
| acceleration | 3 级 Horner，系数 (20c5)..(2c2) | 3 | 3 | 6 | `γ_10` |

velocity/acceleration 采用比 Horner 实际次数更大的 γ（12 与 10），原因是代码把“4 次系数缩放”也计入 γ 余量。由于系数缩放已由 `Δ_i` 覆盖，这在数学上是**重复覆盖**——只会更保守，不会更松。用超可加性可写：

```
γ_8 + 4u ≤ γ_12,      γ_6 + 4u ≤ γ_10
```

position 不需要缩放，n = 10 与 Horner 次数完全一致。

**结论**：三项计数与源码一致；velocity/acceleration 的 n 比严格需要多 4，属于保守冗余。

## 5. γ_n 的向外取整

`γ_n = n·u/(1−n·u)` 在 binary64 中计算时，`n·u`、`1−n·u`、除法都可能向下舍入，导致 γ 偏小。源码现在：

```c
double nu = (double)rounding_count * unit;
nu = nextafter(nu, INFINITY);
gamma = nu / (1.0 - nu);
gamma = nextafter(gamma, INFINITY);
```

`n ≤ 12` 且 `u = 2^-24`，`n·u` 本身就是 2 的幂缩放、binary64 精确；两次 `nextafter` 保证即使后续实现改变也不会低估。最终 `E_coef + γ·Σ` 与绝对项都用 `add_upper_nonnegative` / `multiply_upper_nonnegative` 向外。

## 6. 时间量化项

消费端时间参数：

```
s_c = fl( fl(Δµs) · fl(1e-6) ),   Δµs ∈ [0, 20000]
```

`Δµs` 是整数且 `≤ 20000 < 2^24`，到 binary32 的转换为零误差。`fl(1e-6) = 1e-6(1+δ1)`，乘法再引入 `δ2`：

```
s_c = Δµs·1e-6·(1+δ1)(1+δ2),   |δ1|,|δ2| ≤ u
|s_c − s| ≤ (2u + u²)·s ≤ 2u·s_max     （s ≤ s_max）
```

验证器在精确时间 `s` 上求值，因此位置差至多 `|p'|·2u·s_max`；速度差至多 `|p''|·2u·s_max`；加速度差至多 `|p'''|·2u·s_max`。源码：

```c
double time_scale = multiply_upper_nonnegative(2.0 * 0x1p-24, s_max);
... + time_scale * polynomial_derivative_magnitude(cf, 5u, k, s_max)
```

`polynomial_derivative_magnitude` 用 `Σ i(i−1)…|c_i| s^{i−k}` 的向外和，覆盖整个 `[0, s_max]`。

**s_max 覆盖性**：验证器在子区间 `[u0,u1]` 上的所有求值点满足 `s ≤ h·u1 = s_max`（`u1 ≤ 1`，`h` 为消费端同一个 float 时长）。因此 `s_max` 严格覆盖消费端在本子区间内的所有时间参数；细分时 `u1` 减小，界随之收紧。

## 7. 次正规、溢出与圆整模式

### 7.1 次正规绝对项

若某次运算的结果落在次正规区，单次舍入误差不超过半个最小次正规 `2^-150`。`n` 次运算总计不超过 `n·2^-149`（两倍余量）。源码加

```c
add_upper_nonnegative(result, (double)rounding_count * 0x1p-149)
```

该项**不依赖 `s_max ≤ 1`**：它是每次舍入的绝对上界，与时间尺度无关。`s_max` 只出现在系数项与 γ 项里。

### 7.2 正规范围前提的显式拒绝

`γ_n` 要求中间量处于正规范围且无溢出。源码：

```c
if (magnitude_term > (double)FLT_MAX * 0.0625) return INFINITY;   /* 溢出风险 */
if (magnitude_term < (double)FLT_MIN * 1024.0) return INFINITY;    /* 抵消入次正规区 */
```

`+Inf` 在调用处被判定为失败（`build_curve` 返回 0 → `NUMERIC_FAILURE`），不会退化成点估计。完全零曲线 `magnitude_term = 0` 不触发拒绝（`E = 0 + 0 + n·2^-149`）。

### 7.3 圆整模式

`γ_n` 与"半个 ulp"都假设 round-to-nearest。R5 之前只有编译期假设；现在验证器在入口读取运行时状态：

- Cortex-M7：`FPSCR.RMode[23:22] == 00b`、`FZ[24] == 0`、`DN[25] == 0`；
- Host/SIL：`fegetround() == FE_TONEAREST`。

任一条件不满足返回 `M5_SAFETY_NUMERIC_FAILURE`。回归用例 `m5_safety_floating_environment` 把 host 圆整模式改成 `FE_DOWNWARD`，要求必须拒绝，恢复 RN 后必须通过。

## 8. 系数构造的运算顺序一致性

`consumer_power_coefficients()` 与消费端逐行对应：

| 消费端 | 验证器 |
|---|---|
| `h*h`, `h*h*h`, … | `h2 = h*h; h3 = h2*h; …`（幂等顺序，2 的幂缩放不改变舍入） |
| `(p1-p0) - h*v0 - ((.5f*h)*h)*a0` | `((p1 - p0) - h * v0) - ((0.5f * h) * h) * a0` |
| `(v1-v0) - h*a0` | `(v1 - v0) - h * a0` |
| `10*d/(h*h*h) - 4*v/(h*h) + ac/(2*h)` | `10 * d / h3 - 4 * dv / h2 + da / (2 * h)` |
| `-15*d/(h*h*h*h) + 7*v/(h*h*h) - ac/(h*h)` | `-15 * d / h4 + 7 * dv / h3 - da / h2` |
| `6*d/(h*h*h*h*h) - 3*v/(h*h*h*h) + ac/(2*h*h*h)` | `6 * d / h5 - 3 * dv / h4 + da / (2 * h3)` |

`2*h`、`2*h*h*h` 中的 2 是 2 的幂，`2*x` 精确，因此 `2*h3` 与 `(2*h)*h*h` 的舍入一致。整数常量（10/4/15/7/6/3）先转 float 再参与运算，两边相同。

**验证方式**：S06-086 把真实 `control_sample_reference()` 链接进测试，逐点比较两条曲线；若系数顺序不一致，误差会显著超出包络（实测 `max gap/envelope = 0.089`）。此外 `check_consumer_fp_contraction.py` 用生产 flags 编译两端并确认没有 FMA/MLA 融合，保证"分离乘加"的计数前提成立。

## 9. 与源码的逐项对照

| 推导项 | 源码位置 | 实现 |
|---|---|---|
| `Δ_i` 用区间 | `consumer_eval_error_bound()` 的 `exact[i].low/high` | ✅ 传入 `c_i` / `vc` / `ac` 区间 |
| `E_coef` 向外 | `coefficient_term` + `multiply_upper_nonnegative` | ✅ |
| `γ_n` 向外 | `nu/gamma` 两次 `nextafter` | ✅ |
| `γ` 项 | `result = coefficient_term + γ·magnitude_term` | ✅ |
| 绝对项 | `+ n·2^-149` | ✅ |
| 溢出/次正规拒绝 | `FLT_MAX/16`、`FLT_MIN×1024` | ✅ |
| 时间项 | `time_scale = 2u·s_max` 向外 + 导数上界 | ✅ |
| 圆整模式 | `floating_environment_is_supported()` | ✅ 运行时读取 |

## 10. 剩余风险（需独立审查确认）

1. **编译器重排**：GCC 在 `-O2` 下未观察到 FMA/MLA，但如果未来工具链改变行为，`γ` 计数与系数复刻都要重算；门禁是 `check_consumer_fp_contraction.py`。
2. **Host 的 FTZ/DAZ**：x86 MXCSR 的 flush-to-zero / denormals-are-zero 不在 `fenv.h` 暴露范围内；Host/SIL 测试默认未开启，目标端由 `FPSCR.FZ == 0` 检查覆盖。
3. **`s_max` 与消费端时间参数**：依赖消费端在子区间内只用 `s ≤ h·u1`；若 M6 未来在区间外插值，本推导不适用。
4. **NaN/Inf 输入**：由 `finite3` / `finite_motion_point` 在入口拒绝，不在误差模型范围内。
5. **`velocity_mid_norm` 与 `speed_bound` 的组合**：两个 swept 候选各自包含 `eval_velocity_error` 后才取 `fmin`；这一点由源码结构保证，但需要审查者确认没有第三处使用未修正的 `|v_mid|`。

 
## 14. R6 修正（2026-11-10）：时间量化不等式与次正规传播

### 14.1 R5 的不等式错误

R5 使用

```
(2u + u²) · s ≤ 2u · s_max
```

该式不成立（2u+u² > 2u）。正确的参考时间与推导为：

```
h   = RN(step_us   · 1e-6f)
s_c = RN(offset_us · 1e-6f)
c   = RN(1e-6) = 1e-6(1+d0)
h   = step   · c · (1+e1)
s_c = offset · c · (1+e2)          |e1|,|e2| ≤ u
s_exact := h · (offset/step) = offset · c · (1+e1)
⇒ s_c / s_exact = (1+e2)/(1+e1)
⇒ |s_c − s_exact| ≤ γ_2 · s_exact,  γ_2 = 2u/(1−2u)
```

`offset` 与 `step` 都是 ≤ 2^24 的整数，binary32 转换精确，所以只有两次乘法舍入。
取 `s_max = h·u1`，则 `s_exact ≤ s_max`，消费者时间落在

```
s_eval_max = s_max + γ_2 · s_max
```

验证器现在用 `s_eval_max` 计算系数项、Horner 幅值项与一/二/三阶导数界，时间项为
`γ_2·s_max · |d^k p/ds^k|(s_eval_max)`；`γ_2` 与乘积均向外取整。

### 14.2 次正规项的传播

R5 的 `n·2^-149` 隐含假设“早期舍入不被后续 Horner 乘法放大”。正确形式是

```
Σ_k 2^-150 · s^(n-k) ≤ n · 2^-149 · max(1, s_eval_max)^n
```

验证器现在使用该放大形式，并在 `build_curve` 中强制 `PRODUCT_PREVIEW_STEP_US < 1e6`
（20 ms 预览 → `s_eval_max ≈ 0.0200001 ≤ 1`）；更大的步长必须重新评审。`M5_06_6` §7.1
中“该项不依赖 s_max”的表述已更正为上述放大形式。

### 14.3 新增证据

| ID | 内容 | 结果 |
|---|---|---|
| S06-099 | 两个 step 常量（20000/40000 µs）的全部 60,002 个整数微秒偏移，精确有理数参考时间，检查 `|s_c−s_exact|/s_max ≤ γ_2` 与是否越出 `s_eval_max` | max ratio 6.61e-08 ≤ γ_2 1.19e-07，`outside=0` |
| S06-094（扩展） | 精确有理数 Bernstein 上界改用 `s_eval_max`，覆盖两个 step 与 5 个 (u0,u1) 子区间 | 120 组检查，max required/envelope 0.963，无失败 |

### 14.4 S06-100..102 状态

要求“消费者曲线碰撞、数学曲线不碰撞”的静态/动态/围栏差分用例**未实现**：
在验证器允许的参考范围内，`|consumer − exact|` 处于几何 float 分辨率量级
（S06-085 平滑段 1.5e-05 m 对应 500 m 尺度；本仓库地图 ±4 m 时偏差 ~1e-7 m），
而 voxel 网格与 float 坐标的分辨率同量级，无法稳定地把边界放在两条曲线之间。
保留 R5 的 S06-087..097 与 S06-085/086/094 作为现状证据，S06-100..102 标记
**NOT_IMPLEMENTED / BLOCKED_GEOMETRY_RESOLUTION**，不得记 PASS。

 
## 16. R7 更新：差分碰撞与真实 C 时间证据

R6 §14.4 的“几何分辨率阻断”结论不成立。R7 用确定性搜索（端点位移 0.5–2.5 m、端点速度 0–200 m/s）找到合法参考，使 `|consumer − exact|` 达 ~1e-6 m（几何分辨率 ~1e-7 m），并把障碍/轨道/围栏边界放在 `inflation + d/2`，由真实 Validator 返回 `STATIC_COLLISION` / `DYNAMIC_COLLISION` / `GEOFENCE_VIOLATION`，同时提供安全对照。
S06-103 用真实 `control_sample_reference()` 验证时间参数，60,002 个偏移与 S06-099 的精确有理数结果一致。

 
## 17. R8：撤回差分碰撞结论

R7 的“差分碰撞”用例不满足产品包线（使用 200 m/s、5000 m/s²）且只以中点为据，R8 撤回。S06-104 给出产品包线内定量不可行性（最大偏差 1.26e-07 m，为放置分辨率的 2.1 倍，低于 8 倍阈值），S06-100..102 标记 NOT_IMPLEMENTED_PRODUCT_ENVELOPE。R8 不改变 §14 的时间与次正规推导。

 
## 18. R9：时间参数与可行性界的修正

S06-103 Phase B 此前用名义局部时间覆盖实测值，R9 改为共享 `control_reference_locate_segment()` 的实测值，并新增负向注入测试。S06-104 由有限采样最大值改为差值多项式的 Bernstein 连续上界，并用导数的 Bernstein 界过滤非包线内轨迹；679 例自洽轨迹的上界为 1.22e-08 m，低于三个通道的放置分辨率，S06-100..102 保持 NOT_RUN。

 
## 19. R10：时间参考与可行性结论修正

- 时间参考改为物理微秒时间 `local_us/1e6`，不再使用舍入后的 h；接缝两侧分别验证。
- 新增 E_total 分量预算（系数/Horner/次正规/时间），三维范数向外取整，995 个真实 C 采样验证。
- 撤回 R9 的“三通道均不可构造”结论：该结论依赖固定 4× 系数与单一 ULP，未覆盖完整验证器链；现标记 **MATHEMATICAL_FEASIBILITY_NOT_ESTABLISHED**。

## 20. R15：Jerk 扩展区间与独立 Fraction 验证

R15 修正 R14b 扩展尾段：Bernstein 二次控制点必须先转幂基
`p0=b0, p1=2(b1-b0), p2=b0-2b1+b2`，尾坐标统一为
`1 + gamma_2*(end/duration)`。生产仍保留 `60/d^3` 与区间位置控制点，并同时
取 Bernstein 原区间、幂基尾段、consumer binary32 多项式三者最大值。

S06-112 用真实 C binary32 系数和 Fraction 连续最大值（端点 + 驻点
`s*=-c4/(5c5)`）逐例验证，无容差；旧 Bernstein-as-power 公式在 248/300 个尾部
案例中欠界，作为负向对照。S06-113 把时间转换与 C gamma_2 谓词提取为共享函数，
正常输入 PASS，错误局部时间、4*gamma_2、错误段时长 FAIL。
MR-009 仍为 WAITING_INDEPENDENT_REVIEW，未 APPROVED。

## 21. R16：Jerk 三类上界与时间注入的实际误差

R16 将 validator 的 jerk 上界拆成 Bernstein 原区间、幂基尾段、consumer binary32
三类。S06-112 分别对 Fraction 连续最大值验证，组件检查 1080 次；总界为三者最大。
S06-113 用 additive-seconds 参数注入 4×γ₂，实测 error/γ₂ 为 3.618–4.059，而不是
整数微秒截断产生的 1 微秒误差。MR-009 仍未 APPROVED。

## 22. R24：Horner 中间值上界（补充，不是 APPROVED）

`consumer_eval_error_bound()` 旧量 `sum |c_i| s^i` 只约束输入幅度，不能单独证明
Horner 每级中间值不溢出。R24 记录所需的逐级向外舍入上界（生产代码尚未实现，见“尚未证明”）：

```text
H_n = |c_n|
H_k = |c_k| + s_max * H_(k+1)，k=n-1..0
```

对 position（5 次）、velocity（4 次）、acceleration（3 次）及导数缩放
`3c3, 24c4, 60c5` 分别计算；任一级非有限即返回 INFINITY/NUMERIC_FAILURE。
保留 gamma_n 误差项和次正规项，不改变 4,096 B 预算或安全阈值。

分类：
- 已证明：Bernstein jerk `60/d^3`、Bernstein-to-power、`u_tail`、六个 R14b Fraction 反例。
- 已测试：S06-112 局部控制点/Fraction、S06-113 注入、生产 NUMERIC/reason 夹具。
- 尚未证明/未实现：生产代码尚未加入逐级 H_k 检查；这是 MR-009 的未关闭缺口，需独立审查与实现评审。
- 前置条件：`s_max` 上界、系数有限性、编译器 FP 环境（无 contraction、无 fast-math）。

MR-009 仍为 **WAITING_INDEPENDENT_REVIEW**，Agent 不 APPROVE。

## 23. R27：逐级 Horner 上界落地（生产实现，未 APPROVE）

R27 在 `consumer_eval_error_bound()` 中加入逐级 Horner 上界：

```text
H_n = |c_n|
H_k = |c_k| + s_max * H_(k+1),  k = n-1..0
```

- 每一级 `multiply_upper_nonnegative()` 与 `add_upper_nonnegative()` 都向外取整。
- 每级检查 `isfinite` 与 `> FLT_MAX`；任何非有限/超限返回 `INFINITY`。
- 覆盖 position（5 次）、velocity（4 次，含 `5c5..2c2` 缩放）和 acceleration
  （3 次，含 `20c5..2c2` 缩放）。
- 该检查与 `gamma_n` 误差项、次正规项、时间项并存；不改变 4,096 B 预算或
  安全阈值。
- 仍未证明：独立审查签字、未来编译器重排/FMA、M6 区间外插值、Host FTZ/DAZ
  与目标的完整等价性。

R27 没有新增专用生产测试，这是 R28 的缺口。

## 24. R28：专用生产测试与边界证据（未 APPROVE）

R28 新增两个测试到 `tests/guidance/test_m5_main.c`，并加入 test-only
`m5_trajectory_safety_test_consumer_error_bound()`（仅在
`FLIGHT_M5_SAFETY_TEST_API` 下编译，不进入产品对象/ABI）：

| 测试 | 覆盖 |
|---|---|
| `m5_safety_mr009_horner_bound_extremes` | FLT_MAX 附近系数；Horner 中间值超限；正负抵消；Position/Velocity/Acceleration 缩放；次正规；极短 `s_max`；NaN 系数 |
| `m5_safety_mr009_extreme_reference` | 真实 `m5_trajectory_safety_validate()`：全 FLT_MAX 参考；±FLT_MAX/2 抵消；FLT_MAX 加速度差；危险输入不得 `M5_SAFETY_PASS` |

测试结果（R28）：

- M5 Host: `checks=320647 failures=0`（含新增 20 个检查）。
- S06-112: 420 axis checks / 0 failures，真实 seed `0x53313632`。
- S06-108: 96 cases / 0 failures，真实 seed `0x53303631`。
- S06-113: 76 checks / 0 failures。
- 普通与 `-O` 采集器自测：28/28 负向场景通过。
- Stack: 4,248 B > 4,096 B，B06 BLOCKED；与 MR-009 数学结论独立。

### 分类

- **已证明**：Bernstein jerk `60/d^3`、Bernstein→power、`u_tail`、R14b 六个
  Fraction 反例、逐级 Horner 上界公式与向外取整实现。
- **已实现**：生产 `consumer_eval_error_bound()` 的 H_k 检查；test-only 边界钩子。
- **已测试**：上述生产路径极端输入、S06-108/112/113、S06-105/107。
- **尚未证明**：独立数学审查；编译器未来重排/FMA；M6 区间外插值；Host
  FTZ/DAZ 对目标 FPSCR 的完全等价；产品包线外输入的数学可行性。
- **依赖前置条件**：`s_max` 上界、系数有限、无 contraction/fast-math、
  round-to-nearest、输入在产品包线内。

**MR-009 继续 `WAITING_INDEPENDENT_REVIEW`。Agent 不自行 APPROVED。**



## 25. R29：归档修复、精确状态断言与分支证据边界

R29 应用 `R29_R28_REMEDIATION.patch`，新增/加强：

- MR-009 focus runner：`bash tests/guidance/run_mr009_focus.sh`，
  15 checks / 0 failures。
- `test_m5_safety_mr009_extreme_reference_rejected()` 明确区分：
  - 全 FLT_MAX 与正负抵消输入在 `validate_reference_shape()` 被
    `M5_SAFETY_INVALID / M5_SAFETY_REASON_REFERENCE_INVALID` 拒绝；
  - 正常位置 + `±FLT_MAX` 加速度差进入
    `M5_SAFETY_NUMERIC_FAILURE / M5_SAFETY_REASON_NUMERIC`。
- M5 Host R29：320,650 checks / 0 failures。

### 25.1 分支证据状态（未完成，不得 APPROVED）

R29 审查矩阵要求位置/速度/加速度 Horner、速度/加速度系数缩放的
test-only trace/counter。当前补丁只完成精确状态/reason 断言，**没有**
实现六类分支计数器，也没有合法 companion 的计数器零值断言。

因此：

- Position Horner 分支可达性：**NOT_PROVEN_IN_R29**
- Velocity Horner 分支可达性：**NOT_PROVEN_IN_R29**
- Acceleration Horner 分支可达性：**NOT_PROVEN_IN_R29**
- Velocity coefficient scaling：**NOT_PROVEN_IN_R29**
- Acceleration coefficient scaling：**NOT_PROVEN_IN_R29**
- 合法 companion 计数器零值：**NOT_PROVEN_IN_R29**

上述缺口已登记在
`spec/quality/owner_approval/R29/05_MR009_INDEPENDENT_REVIEW_MATRIX.md`，
不得用状态断言替代分支证据。

### 25.2 分类

- 数学证明：已有 Horner 上界推导；未变。
- 生产实现：已有 H_k 检查；未变。
- 生产路径测试：R29 精确状态断言通过；分支级 trace 未完成。
- 独立审查：仍未完成。

**MR-009 保持 WAITING_INDEPENDENT_REVIEW。Agent 不自行 APPROVED。**


## 26. R30：实际 binary32 舍入后的 Horner 中间值证明（未 APPROVE）

R29 的理想实数递推

~~~text
H_n = |c_n|
H_k = |c_k| + s_max * H_(k+1)
~~~

只包络理想实数 Horner 递推；它没有把上一层 binary32 乘法/加法的实际舍入结果传入下一层。若上一层向上舍入，下一层实际操作数可能大于理想 H_k。因此仅检查 H_k <= FLT_MAX 不是完整的中间值溢出证明。

### 26.1 round-to-nearest 溢出阈值

binary32 round-to-nearest/ties-to-even 的正向溢出中点为：

~~~text
T_RN = 2^128 - 2^103
~~~

在该中点，ties-to-even 选择假想的 2^128，其指数超出 binary32 有限范围，结果为 +Inf。负向阈值对称。因此，对每个乘法和加法的绝对值上界，必须严格小于 T_RN；等于或超过即 fail-closed。

### 26.2 递推实际 rounded intermediate

令 u=2^-24，eta=2^-149。对任意无溢出的 binary32 RN 操作：

~~~text
|fl(x)-x| <= u*|x| + eta
~~~

正规结果由相对项覆盖；次正规结果由绝对项覆盖。定义单次 binary32 结果的向上界：

~~~text
round32_upper(X) =
  next_up(X*(1+u)),  X >= FLT_MIN
  next_up(X+eta),   0 <= X < FLT_MIN
~~~

已证明对应操作不溢出后，可将该上界截到 FLT_MAX，因为实际 RN 结果必为有限 binary32。

从最高次项开始，逐级执行：

~~~text
R_n = |c_n|
P_k = up64(s_max * R_(k+1))
require P_k < T_RN
P32_k = min(FLT_MAX, round32_upper(P_k))
A_k = up64(|c_k| + P32_k)
require A_k < T_RN
R_k = min(FLT_MAX, round32_upper(A_k))
~~~

其中 up64 是 binary64 nextafter(...,+Inf) 向外运算。这样每一级的乘积输入包含上一层真实 binary32 舍入的上界；每一级加法也以实际 rounded product 上界为输入。position/velocity/acceleration 分别按 degree 5/4/3 执行。任何非有限值或 P_k/A_k >= T_RN 返回 +Inf，由调用者转成 M5_SAFETY_NUMERIC_FAILURE。

### 26.3 S06-115：R29 exact-H 漏洞反例

固定 binary32 输入（无随机 seed）：

~~~text
s  = 0x1.8f5c28p-2f
c3 = 0x1.64befeP+127f
c4 = 0x1.b6eda2P+126f
c5 = 0x1.ca01dcP+127f
c0 = c1 = c2 = 0
~~~

该输入满足所有理想 H_k < FLT_MAX，最大 H_3/FLT_MAX = 0.9999999992172294；sum |c_i|s^i / FLT_MAX = 0.05931899342614322 < 1/16，因此 R29 的 exact-H 与 magnitude guard 均不会拒绝。真实分离的 binary32 multiply/add 在 k=4 向上舍入后，k=3 的精确加法输入恰好落在 T_RN，RN ties-to-even 结果为 +Inf。旧 exact-H 检查会接受该输入；R30 的 rounded recurrence 必须拒绝，并记录一次 POSITION_HORNER 命中。

S06-115 同时断言：理想 H_k 均小于 FLT_MAX、幅度项小于 FLT_MAX/16、真实 volatile binary32 Horner 在 k=3 发生非有限、生产 consumer_eval_error_bound() 返回 +Inf、对应 test-only branch counter 恰为 1。

### 26.4 分支证据与边界

- Test-only 计数器对系数构造、速度缩放、加速度缩放、position/velocity/acceleration Horner 六类拒绝分别记录 0/1 命中；保留 first-stage enum。
- 计数器只在 FLIGHT_M5_SAFETY_TEST_API 编译，复位/读取 API 不进入产品 ABI；无测试宏的生产对象不增加状态、栈或运行时分支。
- 全验证器路径继续验证系数构造和两类导数缩放拒绝的 NUMERIC_FAILURE / NUMERIC / passed=0，并验证合法 companion 的所有计数为 0。
- 三个 Horner 度数通过同一个生产 consumer_eval_error_bound() helper 逐项命中；S06-115 另外给出实际 binary32 溢出反例。这不冒充三个 Horner 分支均已由完整 validator 输入可达；完整 validator 可达性仍交由独立审查人判断。
- 专用 runner bash tests/guidance/run_mr009_focus.sh 现在运行 Horner、六阶段追踪、极端参考、浮点环境和真实 C 时间转换五组测试。

### 分类

- 已实现：逐级 rounded product/sum 上界和 RN overflow-midpoint 拒绝。
- 已增加：S06-115 确定性 exact-H 反例、六类 test-only 0/1 branch counter、合法 companion 零计数断言。
- 待验证：本轮 commit 的 Host、ASan/UBSan、Cortex-M7 编译及专用 runner CI 结果。
- 待独立审查：消费者系数身份、FP 环境/目标等价、M6 时间域和三个 Horner 拒绝分支的完整 validator 可达性。

**MR-009 仍为 WAITING_INDEPENDENT_REVIEW；Agent 不 APPROVE。M5-06 继续 BLOCKED，S06-100..102 继续 NOT_RUN，M5-07 不得启动。**


### R30.1：数值阶段短路（CI 首轮失败后的修复）

首轮 CI 的三项失败来自计数语义而非数学反例：系数构造失败后仍继续计算 vf/af；速度缩放失败后仍继续计算 af。生产路径现改为每一阶段遇到首个非有限值即返回 NUMERIC_FAILURE，不继续执行下游数值阶段。这样六类 0/1 counter 互斥且与 first-stage enum 一致，不会把同一根因计成多个拒绝分支。该修改只提前拒绝，不放宽任何安全阈值。


## 27. R30.1 CI 复验结果（代码基线 80670f）

Host validation #43417453111：6/6 jobs SUCCESS；M6/M7 host validation #43417453109：5/5 jobs SUCCESS；M7 host validation #43417452817：2/2 jobs SUCCESS。M5 Host 322,299/0；MR-009 focused 430/0；M5 ASan/UBSan 322,299/0；focused ASan/UBSan 430/0；M4/M5 Integration ASan/UBSan 1,075/0。Spec Audit、生产 ABI、Cortex-M7 模块编译均 PASS。

R30.0 首轮三项 branch-counter 失败保留为历史结果；R30.1 fail-fast 修正后所有上述检查通过。远程 CI 结果不等于本地 evidence ZIP，也不等于独立数学 APPROVED。H723 实机、WCET/RTA、HIL、飞行仍 NOT_RUN。
