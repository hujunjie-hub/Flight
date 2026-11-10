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
