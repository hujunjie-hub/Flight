# MR-009 数学推导：binary32 消费曲线误差包络

状态：**INTERNAL_THREE_PASS_REVIEW_COMPLETE_WITH_LIMITATIONS**。项目所有者已明确取消外部独立审查要求，改由三轮内部复核；本文不声称获得独立第三方批准。

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

## 5. γ_n 与 γ₂ 的向外取整

令 `ν = n·u`，其中 `u = 2^-24`、`n ∈ {10,12}`。误差因子
`γ_n = ν/(1−ν)` 在 `0 ≤ ν < 1` 上单调递增。要得到可证明的上界，
分子必须不小于精确 `ν`，分母必须不大于精确 `1−ν`，最后的商再向上取整。
仅对分子和商调用 `nextafter(+Inf)` 不足以证明分母方向，因此 R30.3
显式把分母向下取整：

```c
double nu = (double)rounding_count * unit;
nu = nextafter(nu, INFINITY);
denominator = nextafter(1.0 - nu, -INFINITY);
if (!(denominator > 0.0) || !isfinite(denominator)) return INFINITY;
gamma = nu / denominator;
gamma = nextafter(gamma, INFINITY);
```

在 round-to-nearest 环境下，第一次 `nextafter` 给出
`nu_hi ≥ ν`；对 binary64 计算结果 `fl(1−nu_hi)` 再向负无穷移动一格，
得到 `den_lo ≤ 1−nu_hi ≤ 1−ν`。分母正值时，
`nu_hi/den_lo ≥ ν/(1−ν)`；商向正无穷移动一格，得到最终上界。
若分母非正或非有限，直接返回 `INFINITY`，不继续计算。

`gamma2_upper()` 对 `n=2` 使用同一方向：`nu2` 向上、`1−nu2`
向下、商向上，并对分母做正值/有限性检查。运行时浮点环境检查保证
round-to-nearest；Host 与 Cortex-M7 对 FMA / contraction 的构建审计则保证
consumer 的 binary32 操作序列没有被融合改变。

最终 `E_coef + γ·Σ` 与绝对项仍通过 `add_upper_nonnegative` /
`multiply_upper_nonnegative` 向外传播。

## 6. 时间量化项

M6 consumer 在一个有效 segment 内使用 binary32 局部时间参数。对验证器
正在处理的子区间 `[u0,u1]`，`0<=u1<=1`，消费端段时长 `h` 是 binary32
值（进入 validator 后精确提升为 binary64），因此精确局部参数满足
`s_exact<=s_max=h*u1`。实现通过 `multiply_upper_nonnegative(h,u1)`
向外取得 `s_max`；即使当前两个 binary32 操作数的乘积可在 binary64 中
精确表示，也保留显式 outward guard。

时间转换最多涉及两次 round-to-nearest binary32 舍入。令
`u=2^-24`、`gamma2=2u/(1-2u)`，使用定向舍入实现得到
`gamma2_upper>=gamma2`，则

```
delta_s = gamma2_upper * s_max
s_eval_max = s_max + delta_s
|s_consumer - s_exact| <= delta_s
```

以上乘法与加法均通过 `multiply_upper_nonnegative()` /
`add_upper_nonnegative()` 向外取整。注意时间偏差必须作用于**实际消费端
对应输出的多项式**，不能把速度/加速度输出简单视为未舍入位置多项式的
二阶/三阶导数。消费端系数定义为：

```
P_c(s) = Σ cf[i] * s^i
V_c(s) = Σ vf[i] * s^i,  vf[i] = RN((i+1) * cf[i+1])
A_c(s) = Σ af[i] * s^i,  af[i] = RN((i+1)(i+2) * cf[i+2])
```

其中 `RN` 是 binary32 round-to-nearest。因而三个时间偏差项分别为：

```
E_time,pos = delta_s * sup |P_c'(s)|
E_time,vel = delta_s * sup |V_c'(s)|
E_time,acc = delta_s * sup |A_c'(s)|
```

上确界均在 `s ∈ [0,s_eval_max]` 上计算。生产实现分别使用 `cf` 的一阶
导数、实际 binary32 缩放后的 `vf` 的一阶导数、实际 binary32 缩放后的
`af` 的一阶导数。此前以 `cf` 的二阶/三阶导数代替后两项，会漏掉
`vf/af` 系数缩放舍入对导数上界的影响；R31 已修正。

导数上界用绝对系数和计算，并在扩大后的区间 `[0,s_eval_max]` 上求值；
系数差、Horner 舍入和次正规项也使用同一个扩大区间。

M5-06 的输入形状要求 `PRODUCT_PREVIEW_STEP_US=20000u`，故当前产品域为
20 ms segment，`s_eval_max≈0.020000002`。M6 locator 拒绝超出 preview
horizon 的 offset，不做区间外外插值。M6 另支持 40 ms route preview，但
M5-06 validator 不接受该 40 ms 输入；本证明不声称覆盖该路径。

## 7. 次正规、溢出与圆整模式

### 7.1 次正规绝对项

round-to-nearest 下，精确结果落入次正规区时，单次绝对舍入残差
不超过半个最小次正规间距，即 `2^-150`。该残差进入后续 Horner 链后，
会被后续乘法中的 `|s|` 以及后续正规运算中的相对因子放大。设总舍入
次数为 `n∈{10,12}`：任一残差最多经过 `n` 个后续运算，且最多经过
`n` 次乘以 `s`，因此其传播量不超过

```
2^-150 * (1+u)^n * max(1, s_eval_max)^n
```

由于 `u=2^-24` 且 `n<=12`，有 `(1+u)^n < 2`，故进一步得到源码采用的
保守界

```
n * 2^-149 * max(1, s_eval_max)^n
```

这里的 `2^-149` 不是把单次实际残差误写成一个最小次正规数；它是将
半间距残差经过最多 `n` 个后续相对舍入放大后，再用因子 2 吸收的
统一上界。实现通过 `multiply_upper_nonnegative()` 传播放大因子，再用
`add_upper_nonnegative()` 加入总误差。仅当 `s_eval_max<=1` 时，该项才
退化为 `n*2^-149`。

### 7.2 正规范围前提的显式拒绝

`γ_n` 要求中间量处于正规范围且无溢出。源码：

```c
if (magnitude_term > (double)FLT_MAX * 0.0625) return INFINITY;   /* 溢出风险 */
if (magnitude_term < (double)FLT_MIN * 1024.0) return INFINITY;    /* 抵消入次正规区 */
```

`+Inf` 在调用处被判定为失败（`build_curve` 返回 0 → `NUMERIC_FAILURE`），不会退化成点估计。完全零曲线 `magnitude_term = 0` 不触发拒绝（`E = 0 + 0 + n·2^-149`）。

### 7.3 圆整模式与次正规环境

`gamma_n`、`gamma_2` 与 RN overflow midpoint 均假设 round-to-nearest。
验证器入口执行运行时环境检查：

- Host/SIL：`fegetround()==FE_TONEAREST`；x86 额外读取 MXCSR，要求 RC 为
  nearest 且 FTZ/DAZ 位清零；
- Cortex-M7：读取 `FPSCR.RMode[23:22]`、`FZ[24]`、`DN[25]`，要求 RN、
  FZ=0、DN=0。

Host 负向测试覆盖 `FE_DOWNWARD`、`FE_UPWARD`、`FE_TOWARDZERO` 以及
MXCSR FTZ/DAZ 注入；不满足条件时返回 `M5_SAFETY_NUMERIC_FAILURE`。
Cortex-M7 FPSCR 路径已交叉编译并由源码审计，但尚无 H723 实机运行结果；
该硬件等价性尚无 H723 实机运行证据，属于单独的目标验证门禁；内部数学复核不替代该证据。

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
| `γ_n` / `γ₂` 向外 | 分子向上、分母向下、商向上；分母正值检查；Host/ARM source audit token gate | ✅（Host #43424620489；ARM audit PASS） |
| `γ` 项 | `result = coefficient_term + γ·magnitude_term` | ✅ |
| 绝对项 | `+ n·2^-149·max(1,s_eval_max)^n`，向外传播 | ✅ |
| 溢出/次正规拒绝 | `FLT_MAX/16`、`FLT_MIN×1024` | ✅ |
| 时间项 | `delta_s=gamma2_upper·s_max`；`s_eval_max=s_max+delta_s`；position 用 `cf'`、velocity 用 `vf'`、acceleration 用 `af'` | ✅ R31 修正并由 S06-107 oracle 复验 |
| 圆整模式 | Host fenv+MXCSR；ARM FPSCR.RMode/FZ/DN | ✅ Host 负向测试；ARM 硬件运行未做 |

## 10. 剩余边界与目标验证门禁

1. **编译器重排 / contraction**：当前 `rtconfig.py` 固定
   `-ffp-contract=off -frounding-math -fexcess-precision=standard`；
   ARM object audit 对 M6 consumer 与 validator 均报告
   `fused_operations=0`。未来工具链或 flags 改动仍必须重跑
   `check_consumer_fp_contraction.py`。
2. **目标 FP 环境等价**：Host 已检查 x86 MXCSR 的 RC/FTZ/DAZ 并执行负向测试；
   Cortex-M7 源码读取 FPSCR.RMode/FZ/DN，已交叉编译，但没有 H723 硬件运行
   证据。该项保持 NOT_RUN，不把交叉编译等同于实机验证。
3. **M6 时间域**：M5-06 只验证 20 ms preview；M6 locator 不允许 horizon
   外外插值。M6 的 40 ms route preview 不属于 M5-06 当前证明域，不能混用。
4. **NaN/Inf 输入**：由 `finite3` / `finite_motion_point` 在入口拒绝，不在
   误差模型范围内。
5. **`velocity_mid_norm` 与 `speed_bound` 的组合**：源码复核确认
   `velocity_mid_abs` 先加入 `eval_velocity_error`，再向外求三维范数；
   swept-radius 两个候选均使用已修正的 `velocity_mid_norm`，随后才取
   `fmin`，没有发现未修正的第三条使用路径。
6. **完整 Validator 的 Horner 分支可达性**：degree 5/4/3 拒绝分支由生产
   `consumer_eval_error_bound()` helper 直接测试；完整
   `m5_trajectory_safety_validate()` 输入可达性不作声称。三轮内部复核接受
   该显式边界：helper 的 fail-closed 算术路径已单测，完整 Validator 另有
   系数/速度缩放/加速度缩放拒绝测试及合法输入通过测试。若未来产品域或
   上游约束变化使这些 Horner 拒绝分支成为可达产品路径，应补充端到端夹具。

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

- **已证明**：Bernstein jerk `60/d^3`、Bernstein→power、`u_tail`、R14b
  六个 Fraction 反例、逐级 rounded Horner 上界公式、RN overflow midpoint
  与向外取整实现；`gamma_n/gamma_2` 的分子上/分母下/商上方向。
- **已实现**：生产 `consumer_eval_error_bound()` 的逐级 rounded product/sum
  检查；每一级均按 `T_RN=2^128-2^103` fail-closed；test-only 0/1 branch
  counters 与阶段性 fail-fast。
- **已测试**：S06-115、S06-108/112/113；R30.3 Host 6/6、M6/M7 5/5、
  M7 2/2；FP audit `M5_FP_MODEL_SELF_TEST_PASS`（9 mutations detected）、
  `model_check=MATCH` 且两个 ARM object 均为 `fused_operations=0`。
- **尚未完成**：独立数学审查签字；degree 5/4/3 Horner 拒绝点的完整
  Validator 可达性决定；H723 FPSCR 硬件运行、WCET/RTA、HIL 与飞行验证。
- **依赖前置条件**：当前 M5-06 20 ms 时间域、系数有限、无 contraction /
  fast-math、round-to-nearest、Host/M7 浮点环境满足检查条件、输入在产品包线内。

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

**产品域边界：** S06-115 的 `s=0x1.8f5c28p-2f` (~0.39) 是 helper 级对抗输入，不属于 M5-06 当前产品时间域。`PRODUCT_PREVIEW_STEP_US=20000u`，M5-06 仅接受 20 ms preview 间隔；在 `u1<=1` 下，生产调用的 `s_eval_max` 约为 0.020000002。S06-115 证明理想 H_k 递推不能普遍替代实际 rounded-intermediate 传播，但不声称旧漏洞反例可由当前产品域的完整 Validator 输入触发。degree 5/4/3 的 helper 拒绝测试另有 `s_max=0.02` 的测试路径；其系数仍是直接构造的 adversarial helper 输入，不代表已证明完整 Validator 可达。

### 26.4 分支证据与边界

- Test-only 计数器对系数构造、速度缩放、加速度缩放、position/velocity/acceleration Horner 六类拒绝分别记录 0/1 命中；保留 first-stage enum。
- 计数器只在 FLIGHT_M5_SAFETY_TEST_API 编译，复位/读取 API 不进入产品 ABI；无测试宏的生产对象不增加状态、栈或运行时分支。
- 全验证器路径继续验证系数构造和两类导数缩放拒绝的 NUMERIC_FAILURE / NUMERIC / passed=0，并验证合法 companion 的所有计数为 0。
- 三个 Horner 度数通过同一个生产 consumer_eval_error_bound() helper 逐项命中；S06-115 另外给出实际 binary32 溢出反例。这不冒充三个 Horner 分支均已由完整 validator 输入可达；完整 validator 可达性仍交由独立审查人判断。
- 专用 runner bash tests/guidance/run_mr009_focus.sh 现在运行 Horner、六阶段追踪、极端参考、浮点环境和真实 C 时间转换五组测试。

### 26.5 gamma 上界的定向舍入

`gamma_n = nu/(1-nu)` 在 `0<=nu<1` 上单调递增。为保证上界，当前实现对 `nu` 使用 `nextafter(+Inf)`，对分母 `1-nu` 使用 `nextafter(-Inf)`，对最终商使用 `nextafter(+Inf)`；`gamma_2_upper()` 与 `gamma_n` 采用同一方向。不得把分母也写成向上舍入。支持的 `n=10/12` 和时间项 `n=2` 均远小于 1，分母正值检查失败时返回 `INFINITY` 并 fail-closed。

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


## 28. R30.3 gamma 定向舍入与最终 CI 复验

- 数学实现提交：`9af2d4864dfcc8bd4b9434749b67f3e5c2fd4e5a`；浮点 source-token 审计更新：`882028c6771180dbd997bcff19de48c88b617307`；proof/matrix/9-mutation self-test wiring：`00339febd045a85285a3df571268f3333c867aae`。
- Host validation #43424620489：6/6 jobs SUCCESS；M6/M7 host validation #43424620578：5/5 SUCCESS；M7 host validation #43424620424：2/2 SUCCESS。
- M5 Host 322,299/0；MR-009 focused 430/0；M4/M5 integration 1,075/0；S06-099 60,002 offsets/0 failures；S06-108/109/113 96/96/76 checks with 0 failures。
- ARM FP audit job 131882878679：`M5_FP_MODEL_SELF_TEST_PASS`（9 mutations detected）、`model_check=MATCH`、`missing_tokens=[]`、compiler `arm-none-eabi-gcc 14.2.1 20241119`，consumer 与 validator objects 均 `fused_operations=0`。
- Consumer object SHA256：`26966662352c95e8d63b9cefa1bd00300d807ab927bc9bcd5fd854f0539e729f`；validator object SHA256：`67e3e9824142585b8f4be3a2ed97573b22eaaba8f8ebdd155d1fe63d9dbf3a0d`。
- Validator source SHA256：`fe63488b3d3dbf164d8de7281dac682818484198bc0c43d363b27bce0ae03aae`；consumer source SHA256：`48fa1f629380ce354617af7082e7f19bd81974f00f9b9c126c9b4c690ff1c31b`。
- 当前文档同步 HEAD `241784357e7a8f3b92514612307df66a13d51fe9` 的 Host #43425062263（6/6）、M6/M7 #43425062339（5/5）、M7 #43425062346（2/2）也全部 SUCCESS。
- CI artifacts：Host `m5-cortex-m7-compile` 14036950533、`abi-evidence` 14036490022、`m4-m5-host-regression` 14035697038；M6/M7 `cortex-m7-module-compile` 14036700759、`m6-m7-host-evidence` 14036135841、`sil-ground-stop-evidence` 14037190065；M7 `m7-host-evidence` 14035836365。未生成本地 evidence ZIP。

以上为实现与 CI 证据，不是独立审查批准。H723 实机、WCET/RTA、HIL、飞行仍 NOT_RUN；MR-009 继续 WAITING_INDEPENDENT_REVIEW。


## 29. R31：项目所有者指定的三轮内部复核与修正（2026-11-13）

项目所有者明确取消 MR-009 外部独立审查要求，指定对当前分支进行三轮内部复核。
本节是当前处置结论，覆盖并取代本文历史章节中所有 `WAITING_INDEPENDENT_REVIEW`
状态文字；不声称独立第三方批准。

### 第一轮：数学推导

- 复核 consumer/validator 系数身份、binary32 运算顺序、Horner rounding count、
  `gamma_n/gamma_2` 定向舍入、RN overflow midpoint、正规/次正规误差传播。
- 发现并修复速度/加速度时间偏差导数使用了未舍入位置系数导数的问题。
  现在对 `vf`、`af` 实际 binary32 缩放系数分别求一阶导数上界。
- `s_max` 采用 outward multiplication，`s_eval_max`、`delta_s` 继续向外传播。

### 第二轮：实现与浮点语义

- 对照 `control_reference.c` 逐项核对位置、速度、加速度系数和 Horner 顺序。
- 确认系数缩放误差由 interval coefficient term 覆盖，Horner 乘加由
  `gamma_n` 与绝对次正规项覆盖；逐级 rounded recurrence 在 RN overflow
  midpoint 前 fail-closed。
- 确认 Host fenv/MXCSR 与 Cortex-M7 FPSCR 检查；H723 实机等价性仍 NOT_RUN。

### 第三轮：测试、边界与证据

- 更新 `verify_consumer_error_budget.py`：速度/加速度 oracle 系数先按
  binary32 逐项舍入，再计算系数差、Horner 幅值与对应输出多项式的一阶导数时间项。
- 保留 S06-115 的域外 helper 反例边界；不把 helper-level Horner stress
  误写成完整 Validator 输入可达性证明。
- 代码修正提交：`79f23d0200c31ebe934bfc8793e65d7af1e6bdd4`；
  oracle 修正提交：`a55071966209a1af615ff6137963eab4631e8bae`；
  后续文档/注释同步提交以分支 HEAD 为准。
- 当前分支 CI：Host validation #43443825526（6/6 SUCCESS），M7 validation
  #43443825495（2/2 SUCCESS）；M6/M7 validation #43443825676 需待最终状态
  写入本节的后续同步记录确认。

**当前结论：** MR-009 的数学实现与内部三轮复核完成（带明确边界），不再等待外部
独立审查。该结论不等于 M5-06 整体验收或飞行放行。S06-100..102、H723 实机、
WCET/RTA、目标 RAM/stack watermark、HIL、飞行仍按各自门禁处理；M5-07 不启动。

## 29. 2026-11-13：MR-009 三轮内部复核结论

项目所有者明确要求不再等待单独独立审查，改为对当前分支执行三轮复核。
本节为当前结论，历史章节中的 WAITING_INDEPENDENT_REVIEW 均保留为当时
状态记录，不再代表当前 MR-009 disposition。

### 第 1 轮：数学模型

- 五次 Hermite 系数区间以同一组 binary32 端点和 binary32 段时长 `h`
  为输入；消费端系数与精确实数系数的差由区间距离项覆盖。
- Position/Velocity/Acceleration 的 Horner 舍入次数分别为 10/12/10；
  Velocity 与 Acceleration 的次数包含系数缩放，属于保守计数。
- `gamma_n`、`gamma_2` 的有向舍入、RN overflow midpoint、次正规残差
  传播和时间参数偏差均已逐项核对。次正规项的 `2^-149` 传播界已在 §7.1
  明确给出半最小次正规间距及后续相对放大的推导。
- 复核发现 R30 的 velocity/acceleration 时间偏差曾分别使用 `cf` 的二阶/
  三阶导数，未计入 `vf/af` 系数缩放舍入；R31 已改为对实际 rounded
  consumer polynomials `V_c`、`A_c` 分别求一阶导数上界。
- 结论：当前证明范围内通过；没有发现需要放宽阈值或改变安全判定的
  数学缺陷。

### 第 2 轮：实现与浮点语义

- 对照 M6 consumer 和 M5 validator 的系数构造、时间转换、三组 Horner
  表达式；新增双源 token 审计，要求实际 consumer 和 validator replica
  的运算顺序均与证明模型一致。
- `consumer_eval_error_bound()` 的系数、Horner、溢出递推和次正规传播
  全部使用扩大后的 `s_eval_max`；子区间 `s_max` 也显式向外计算。
- FP 审计自测由 9 个扩展为 18 个变异用例，除 consumer 时间换算和三组
  Horner 表达式外，还覆盖 expanded-time 区间及 `cf'`/rounded `vf'`/
  rounded `af'` 时间导数，及原有 gamma、区间、次正规、圆整环境条件。
- Cortex-M7 object audit 仍要求 consumer 与 validator 均无 FMA/MLA
  融合；运行时继续检查 RN、FZ、DN 条件，不满足时 fail-closed。
- 结论：实现与证明模型的静态一致性检查已加强；没有发现需改动生产
  Horner 公式的问题。

### 第 3 轮：测试、集成与边界

- S06-099 仍穷举 20 ms/40 ms 的 60,002 个整数微秒偏移；新增对生产证明
  参数 `s_model=h*(offset_us/step_us)` 的精确 Fraction 比较，直接检查
  `|s_consumer-s_model|<=gamma_2*s_model`，并保留对物理微秒时间的检查。
- R31 生产数学修正：`79f23d0200c31ebe934bfc8793e65d7af1e6bdd4`；
  S06-107 Fraction oracle 修正：`a55071966209a1af615ff6137963eab4631e8bae`；
  双源/expanded-time 审计强化：`ad9df36687b68b43f9118ae6e26be8716d900c90`。
  这几项分别覆盖生产误差公式、独立精确 oracle 和防回归审计；最终全套
  CI 以最终文档提交对应的 HEAD 为准。
- 三个 degree 5/4/3 Horner 拒绝分支仍由同一个生产 helper 直接测试；
  本结论不声称它们都能由合法产品包线输入经完整 validator 触发。
  完整 validator 的 coefficient/scaling fail-fast 路径有状态、reason、
  passed 和互斥计数断言；Horner helper 的返回值与各自计数也有断言。
- M5-06 当前证明域仍为 20 ms preview；M6 40 ms route preview 不在本次
  M5-06 证明范围内。H723 实机运行、WCET/RTA、HIL 和飞行验证仍未执行，
  不得将 Host/交叉编译 PASS 描述为这些验证已完成。

### 当前 disposition

**MR-009：TRIPLE_SELF_REVIEWED / PASS_WITH_DECLARED_SCOPE_LIMITATIONS。**
不再要求单独外部数学审查；不把本次内部复核表述为独立第三方认证。
本次只关闭 MR-009 数学复核工作，不推进 M5-07，不改变其他阶段或分支。
