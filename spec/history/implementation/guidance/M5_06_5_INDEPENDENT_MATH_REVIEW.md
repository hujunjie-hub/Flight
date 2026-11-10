# M5-06.5 独立数学审查（初审记录）

状态：**BLOCKED / REVIEW_INCOMPLETE**。本文件记录对当前 M5-06 实现的独立静态数学审查，不构成算法批准，也不授权进入 M5-07。

审查基线：
- 分支：`opt/stage-02-m5-06-safety-validator`
- 生产实现：`middleware/guidance/trajectory_planning/m5_trajectory_safety.c`
- 测试适配器：`tests/guidance/m5_trajectory_safety_test_api.h`
- 现有测试证据：S06-074..079，Host/ASan/UBSan；PR #2 描述的 Host validation #438
- 本次审查方式：独立检查公式、浮点运算路径、编译约束和现有测试的证明边界；尚未执行新的数值实验或目标机验证。

## 1. 审查结论

| ID | 审查项 | 初审结论 | 放行状态 |
|---|---|---|---|
| MR-001 | 五次 Hermite 系数与 Bernstein 控制点转换 | 公式结构与五次 Hermite 边界条件相符；需独立高精度 oracle 覆盖非 dyadic 子区间及极端时长 | OPEN |
| MR-002 | Bernstein convex-hull 位置/导数上界 | 控制点凸包可给出分量范围；向量范数上界需逐轴绝对值上界平方和并向外舍入 | OPEN |
| MR-003 | double outward interval 运算 | add/sub/mul/div 结果以 nextafter 扩张；其可靠性依赖 IEEE-754 binary64、正确舍入、非 fast-math、无重结合/收缩改变运算语义 | OPEN |
| MR-004 | sqrt 与浮点上下界转换 | 当前分支已改为 `sqrt_upper_float()` / `sqrt_lower_float()` 的 binary32 位模式有界二分搜索，不再依赖 libm sqrt；平方和向外舍入及 IEEE 60559 表示假设已加入代码约束。需独立复核证明并等待精确 Fraction oracle 的 CI 证据 | REVIEW_PENDING / CI_PENDING |
| MR-005 | point-to-AABB 下界 | 输入为 float，差值/平方在 binary64 下有较强精确性基础；但应把完整误差论证写成可检查的不变量，并覆盖零距离、次正规数和极端尺度 | OPEN |
| MR-006 | 动态相对运动扫掠界 | 取 speed-based 与 acceleration-based 两个上界的较小值，只有在两者分别对整个子区间成立时才保守；需独立解析/区间 oracle 对速度不确定度、加速度界、时间端点和舍入共同验证 | OPEN |
| MR-007 | 编译器/目标约束 | 当前 ARM/Host runner 已显式声明 `-fno-fast-math`、`-fno-unsafe-math-optimizations`、`-ffp-contract=off`、`-frounding-math`、`-fexcess-precision=standard`；ARM 脚本交叉编译 Host-only validator object 并记录工具链。需以本轮 CI 验证目标工具链支持与实际命令 | IMPLEMENTED / CI_PENDING |

## 2. 具体数学说明

### MR-004：binary32 平方根上下界（当前实现复核）

当前分支已不再通过 libm `sqrt()` 构造范数边界，而是使用 `sqrt_upper_float()` 与 `sqrt_lower_float()`：

- binary32 正有限数的 IEEE 754 编码按无符号位模式单调递增；从 `0x00000000` 到 `0x7f7fffff` 可用固定最多 32 轮二分搜索。
- 任意 binary32 数转为 binary64 精确；其平方最多需要 48 个有效二进制位，且平方指数范围仍处于 binary64 可表示范围内，因此候选 binary32 的平方可精确比较。
- 上界路径先形成平方和上界，再找满足 `f*f >= sum_upper` 的最小 binary32 候选；下界路径先将各分量绝对值、平方及求和逐步向下扩张，再找满足 `f*f <= sum_lower` 的最大 binary32 候选。
- 若上界超出最大有限 binary32，则返回 +Inf（安全拒绝）；下界在超范围时返回 FLT_MAX，仍是保守下界。

该论证依赖 IEEE 754 binary32/binary64 表示、次正规数支持、binary64 正确舍入及禁止 fast-math/重结合。代码现已通过静态断言约束 radix、mantissa、指数范围、subnormal、`FLT_EVAL_METHOD` 和 IEC 60559 宏；仍需独立数学复核这些前提是否足以覆盖目标编译器 ABI。

S06-080 新增 Fraction 精确平方范数 oracle，当前为 IMPLEMENTED / CI_PENDING。未取得 Host、ASan/UBSan 和 Cortex-M7 compile job 对本轮 commit 的证据前，不关闭 MR-004。

### MR-003 / MR-007：区间运算前提

当前 interval helper 对每次基本运算调用 `nextafter` 扩张一个 double ULP。该证明依赖：
- binary64 运算遵守 IEEE-754；
- 基本运算按 round-to-nearest 正确舍入；
- 不允许 fast-math、代数重结合或不受控的表达式收缩改变运算序列；
- `nextafter`、`INFINITY`、`isfinite` 在 Host 与目标工具链上具有预期语义；
- 编译器和数学库版本纳入构建证据。

当前 `tests/guidance/arm_build_check.sh` 已显式加入 `-fno-fast-math -fno-unsafe-math-optimizations -ffp-contract=off -frounding-math -fexcess-precision=standard`，并将编译器版本、size 工具版本、完整 flags 和 M5-06 Host-only 编译范围写入 `build/m5_arm/toolchain.txt`。脚本还将 `m5_trajectory_safety.c` 以 `FLIGHT_M5_HOST_PLANNER` 宏交叉编译为 Cortex-M7 object，但不链接进产品。仍需 CI artifact 确认本轮实际使用的编译器及参数。

## 3. 必须新增的测试（尚未执行）

以下编号预留；在代码、CI 实际运行并保存日志前，状态一律为 NOT_RUN。固定 seed 用于可复现生成，不代表已经获得结果。

| ID | 场景 | 独立 oracle / 断言 | Seed | 状态 |
|---|---|---|---|---|
| S06-080 | norm3 上/下界在 float ULP 边界、零、次正规数、极端尺度的差分 | Fraction 精确有理数平方范数 oracle；upper 不低估、lower 不高估 | 0x6D350680 | IMPLEMENTED / CI_PENDING |
| S06-081 | 动态相对运动端到端连续最小距离，覆盖恒速解析最近点、恒加速度相对运动及不确定度膨胀 | 恒速解析最近点 + 恒加速度二次相对位置的端点/驻点/零点解析 oracle；后续仍需补充不确定度区间 oracle | 0x6D350681 | IMPLEMENTED / CI_PENDING |
| S06-082 | 动态障碍物精确相切、内外各 1 ULP、相邻预览区间接缝 | 几何切触/内外 1 ULP 的 fail-closed 断言；20ms seam 前/后 1 ULP 动态穿越 | deterministic | IMPLEMENTED / CI_PENDING |
| S06-083 | 动态障碍物在自适应细分点精确位置及 ±1 float ULP | 端到端 collision + fail-closed + work_units 证明实际细分；完整独立区间等价 oracle 仍待补 | deterministic | IMPLEMENTED / CI_PENDING |

所有随机测试必须记录：测试 ID、seed、生成器版本、case 数、首次失败输入、缩减后的最小反例、预期/实际状态、CI run 与 commit SHA。不得把采样 oracle 描述成连续数学证明。

## 4. 进入 M5-07 的前置条件

1. 完成 MR-004 的独立数学复核，并取得 S06-080 的 Host/ASan/UBSan 及 Cortex-M7 compile CI 证据；确认 MR-007 的 flags、编译器版本和目标对象范围均符合约束。
2. 独立复核 Hermite→power→Bernstein 转换、导数控制点及 convex-hull 推导，提交可复算的公式和边界条件。
3. 完成 S06-080..083，Host 与 ASan/UBSan 全通过；ARM 重新编译并保存明确的浮点编译参数。
4. 动态障碍物连续距离 oracle 采用解析/区间方法，采样仅作补充。
5. 独立审查者给出明确 APPROVED 结论；未完成标定的动态障碍物加速度上界仍不得作为产品批准参数。
6. M5-06 整体验收仍需单独确认；Host/SIL 和 Cortex-M7 编译不能替代 H723 WCET/RTA、实机 RAM/stack watermark、HIL 与场地验证。

## 5. 安全边界

- M5-05 输出仍是 candidate。
- M5-06 未获独立数学审查批准前，不进入 M5-07。
- 不得合并 PR #2 或修改 main 以绕过验收。
- H723/NONE 不具备在线避障放行；Host/SIL 结果不构成实机避障许可。

### S06-080 实现增量（2026-11-10）

已在 Host 测试中加入 `tests/guidance/verify_norm_bounds.py`：通过 ctypes 调用生产 C helper，以 Python `Fraction` 精确还原 binary32 输入并比较平方范数，不依赖 `sqrt` / `sqrtl` 作为唯一 oracle。固定 seed `0x6D350680`，覆盖 7 组显式边界向量及 10,000 组任意有限 binary32 位模式；测试脚本已接入 `tests/guidance/run_host_tests.sh`。生产代码增加 binary32/binary64 exponent、subnormal、evaluation precision 和 IEC 60559 编译期约束。

当前状态：**IMPLEMENTED / CI_PENDING**。本次提交触发的 GitHub Actions 尚未取得可核验结果；不得标记 PASS。Host/Sanitizer 与 Cortex-M7 编译仍需本次提交的真实 CI 证据。
 
## 6. S06-081..083 源码复核增量（2026-11-10）

后续检查发现，S06-081、S06-082、S06-083 的测试函数已实际存在于 `tests/guidance/test_m5_main.c` 并注册到 Host 测试表；此前表格中的 S06-083 NOT_RUN 已过时。更新后的状态均为 **IMPLEMENTED / CI_PENDING**，尚无本轮 commit 的 CI 结果，不得标记 PASS。

| ID | 已实现覆盖 | 独立 oracle / 局限 | 状态 |
|---|---|---|---|
| S06-081 | 64 组恒速相对运动连续最近点差分；64 组恒加速度相对运动解析候选点差分 | 恒加速度 oracle 检查端点、相对 x 速度为零的驻点、相对 x 位置根；以 long double 计算，属于独立解析数值 oracle，不是向外舍入区间证明；随机速度/位置不确定度传播仍需单独覆盖 | IMPLEMENTED / CI_PENDING |
| S06-082 | 动态碰撞精确相切、内侧 1 ULP、外侧 1 ULP、安全净空；20ms 预览接缝碰撞 | 外侧 1 ULP 允许保守拒绝，唯一硬要求是不得错误 PASS；接缝用例覆盖精确位置及起点 ±1 float ULP | IMPLEMENTED / CI_PENDING |
| S06-083 | 自适应 DFS 子区间中点碰撞，精确起点及 ±1 float ULP 三组 | 端到端断言 DYNAMIC_COLLISION、passed=0 且 work_units 超过预览区间数，证明进入细分；目前不是整段/分段独立连续距离 oracle 等价证明 | IMPLEMENTED / CI_PENDING |

### S06-080 oracle 扩展

`verify_norm_bounds.py` 已进一步扩展：binary32 上界 10,007 组（7 个显式边界 + 10,000 个固定种子随机向量），以及 binary64 下界 10,007 组（7 个显式边界 + 10,000 个固定种子随机向量）。binary64 下界测试覆盖次正规数、最小正规数、极大有限值、指数跨度及混合尺度；所有断言仍使用 Fraction 精确平方范数，不调用 sqrt 作为 oracle。

新增代码提交：`61dbc4c56240a54aa2a8a3bc54e8392093a1b029`。该提交触发的 CI 结果尚未核验，S06-080 仍为 CI_PENDING。

 
## 7. 新发现阻塞：动态 track 预测位置的数值误差包络（MR-008）

状态：**OPEN / SAFETY BLOCKER**。

在复核 `check_dynamic_tracks()` 时发现，动态障碍物中点预测目前按以下路径计算：

1. 将 `horizon_mid_s` 乘以 `1e6f` 并转换为 `uint64_t`，形成整数微秒中点；
2. 通过两个 uint64 时间戳相减，再乘 double 常量 `1e-6` 得到 `dt_exact`；
3. 使用 double 计算 `position + velocity * dt_exact`；
4. 将预测位置转换成 float；
5. `obstacle_rounding_error` 仅统计 float 结果与已舍入 double `predicted` 的差。

第 5 步没有覆盖第 1～3 步的时间量化、秒单位换算和 double 乘加误差。特别是当 double 预测结果恰好落在 float 可表示值上时，float 转换误差可能为零，但这并不证明精确实数预测位置也恰好落在该 float 上。当前代码没有为该路径建立完整的向外误差区间。

这构成动态障碍物距离下界证明的缺口；目前没有证明它一定产生错误 PASS，但在补齐之前不能认定动态距离下界严格保守。

**修复要求：**
- 避免用绝对 epoch 的浮点秒数做差；使用相对参考时间的有界整数微秒差，避免大 epoch 精度损失。
- 对子区间中点时间偏移建立 outward interval，显式包住微秒到秒的精确比例（(10^{-6})）。
- 使用 interval multiply/add 包住 `track.position + track.velocity * dt` 的精确实数结果。
- 将最终 float obstacle 点到完整预测区间两端的最大距离作为位置误差上界，再向上扩张。
- 用预测时间区间端点的最大绝对值计算不确定度传播时域。
- 任何 interval 溢出或非法值必须返回 NUMERIC_FAILURE，禁止退化成点估计。

**新增测试预留：**

| ID | 测试内容 | Oracle | Seed | 状态 |
|---|---|---|---|---|
| S06-084 | 动态 track 预测位置的时间换算、double 乘加及 float 转换误差；覆盖大 epoch、正负时间差、非 dyadic 子区间中点、精确相切/近切 | Fraction 精确时间/位置参考 + 大 epoch 端到端不变性 | 0x6D350684 | IMPLEMENTED / CI_PENDING |

MR-008 关闭前，S06-081..083 即使 CI 通过，也不能单独证明动态障碍物连续距离路径已完成数学验收。

 
### MR-008 修复实现增量（2026-11-10）

生产代码已调整动态 track 预测路径：
- 以相对 reference timestamp 的有界整数微秒偏移计算时间差，不再将绝对 epoch 加上 float 秒数后截断。
- 对子区间中点微秒偏移使用 interval 运算；对 (10^{-6}) 秒/微秒比例使用相邻 binary64 值构造包围区间。
- 对 `position + velocity * dt` 使用 outward interval multiply/add。
- 以最终 float 预测点到完整预测区间两端的最大距离形成三维位置误差上界。
- 不确定度传播时域取预测时间区间最大绝对值；非法或溢出区间 fail-closed。

新增测试适配器与 `verify_dynamic_prediction.py`，使用 Fraction 精确计算时间差和位置，固定 seed `0x6D350684`，7 组边界 + 10,000 组随机输入，并包含预测溢出拒绝。已接入 Host runner 和 Python syntax CI。

代码提交序列：`40bb5c7`（生产修复）、`7346235`（测试 API）、`8baba30`（Fraction oracle）、`d906d6e`（runner）、`cd71774`（py_compile CI）、`9f07029`（指数范围扩展）。

当前状态：**FIX IMPLEMENTED / CI_PENDING**。MR-008 仍为阻塞，直到当前 HEAD 的 Host、ASan/UBSan、Cortex-M7 compile 均通过，且独立审查确认区间推导和大 epoch 边界。


### S06-084 实现补充

- 新增 `verify_dynamic_prediction.py`：7 个显式边界 + 10,000 个随机向量，使用 Fraction 精确计算时间差和三轴位置，并断言生产误差半径覆盖精确预测位置；seed `0x6D350684`。
- 新增大绝对 epoch 端到端不变性测试：低 epoch 与接近 uint64 上限但仍留有 proof/lease 空间的 epoch，使用相同相对 track 时间和运动状态，要求安全结果与 track 检查数一致。
- Host runner 与 CI `py_compile` 已接入新 oracle。

当前 **FIX IMPLEMENTED / CI_PENDING**；未运行当前 HEAD 的 Host/Sanitizer/ARM 工作流前不得关闭 MR-008。

 
## 8. CI run #499 compile blockers and fixes (2026-11-10)

Run #499 / commit `1144ce8d792ab4f5a4d2b229f3ab6461c9722649` exposed two portability failures before M5 test execution:

1. Host GCC rejected `DBL_TRUE_MIN == 0x1p-1074` inside `_Static_assert` because the floating comparison is not an integer constant expression.
2. GNU Arm GCC did not define `__STDC_IEC_559__`, so the preprocessor guard rejected the target compile despite the binary radix/precision/exponent assertions being available.

Fixes committed:
- `b0f40cb`: binary64 subnormal static assertion now checks the integer-valued `DBL_HAS_SUBNORM == 1` property.
- `0349587`: removed reliance on the optional `__STDC_IEC_559__` macro and explicitly asserts 64-bit double storage; binary32/binary64 radix, precision, exponent and subnormal properties remain checked.

Run #499 evidence:
- M4 Host: 927 checks / 0 failures.
- Source layout/time service: PASS.
- Navigation/control Host: PASS.
- M5 Host: compile failed at the non-ICE DBL_TRUE_MIN assertion; no M5 test cases executed.
- Cortex-M7 MPC compile/RAM stage: PASS (MPC RAM 29,252 B, worker stack 8,192 B, largest static frame 2,024 B).
- Cortex-M7 M4/M5 production-source compile: failed at optional IEC macro guard.
- Integration and sanitizers skipped because the M5 Host step failed.

Run #501 is triggered by commit `0349587`; status must be rechecked before declaring the fixes validated. M5-06 remains BLOCKED.

 
## 9. CI #509 verified result (2026-11-10)

Verified commit: `0c761964d4bc6cfeec73ba60464657db450743ee`. GitHub Actions run #509 completed with conclusion SUCCESS; all four jobs passed.

| Job / suite | Result |
|---|---:|
| M4 Host | 927 checks / 0 failures |
| M5 Host | 269,207 checks / 0 failures |
| M4/M5 Integration | 1,000 checks / 0 failures |
| M5 ASan/UBSan | 269,207 checks / 0 failures |
| Integration ASan/UBSan | 1,000 checks / 0 failures |
| Source layout / time-service | PASS |
| Navigation / control | PASS |
| Cortex-M7 compile and MPC RAM checks | PASS |

Independent Fraction oracles:
- S06-080: binary32 upper-bound 10,007 cases and binary64 lower-bound 10,007 cases; seed `0x6D350680`.
- S06-084: predicted-position exact Fraction oracle 10,007 cases; seed `0x6D350684`.

Cortex-M7 compile evidence:
- M5 Host/SIL-only translation unit compiled for Cortex-M7 as an object only; it is not linked into the product.
- MPC RAM 29,252 B; worker stack 8,192 B; largest static frame 2,024 B.
- This is compiler/architecture compatibility evidence only, not H723 runtime WCET, target RAM/stack watermark, HIL, or flight validation.

S06-083 test correction: the initial LIFO work stack processes interval zero first. A successful adaptive collision therefore correctly needs only two work units (parent + child), not more than the 24 preview intervals. The test now asserts `work_units > 1` and all three exact/±1 ULP cases pass.

Status: **HOST/SIL AND CORTEX-M7 COMPILE VALIDATION PASS at #509**. This does not close MR-008's independent mathematical proof review, does not authorize M5-07, and does not authorize H723 obstacle avoidance. Remaining open items are listed below.

 
## 10. S06-081 constant-acceleration oracle strengthened (2026-11-10)

The previous 64-case constant-acceleration differential used 513 uniform time samples to estimate minimum distance. That was not an independent continuous-time oracle and could miss a narrow minimum between samples.

Replaced it with the analytic candidate set for one-dimensional constant relative acceleration plus fixed lateral offset:
- interval endpoints (t=0,T);
- stationary relative x-velocity root (x'(t)=0);
- every in-horizon quadratic root (x(t)=0).

Because (d^2(t)=x(t)^2+y^2), every interior minimum must satisfy either (x(t)=0) or (x'(t)=0). The oracle evaluates all valid candidates in long double and uses `sqrtl`; no fixed time grid is used.

This is an independent analytic continuous-distance differential oracle for the 1D constant-acceleration test model, not a general 3D interval proof for arbitrary acceleration and uncertainty. Code commit: `a14b9f1`. CI pending.

 
## CI #509 状态更新（2026-11-10）

此前记录的 S06-080..084 CI_PENDING 已有对应代码基线结果：commit `0c761964d4bc6cfeec73ba60464657db450743ee` / Host validation #509 全部 jobs success。M5 Host 和 M5 ASan/UBSan 均为 269,207 checks / 0 failures；S06-080 两条 Fraction oracle 各 10,007 cases（seed `0x6D350680`）；S06-084 Fraction prediction oracle 10,007 cases（seed `0x6D350684`）；S06-081..083 对应测试通过；Cortex-M7 M5-06 compile 通过。

CI：[Host validation #509](https://github.com/hujunjie-hub/Flight/actions/runs/42745124335)。

**重要：CI PASS 不等于数学审查 APPROVED。** MR-001 Hermite/Bernstein 推导、MR-002 导数凸包与向量范数界、MR-003 interval outward rounding 假设、MR-004 binary32 sqrt 上下界证明、MR-005 point-to-AABB 下界、MR-006 动态相对 sweep 组合界、MR-007 编译器浮点语义和 MR-008 动态预测位置误差包络仍需逐项独立复核并给出明确结论。当前状态仍为 **BLOCKED / REVIEW_INCOMPLETE**，M5-07 不得开始；H723 目标运行、WCET/RTA、实机 RAM/stack、HIL 和飞行仍 NOT_RUN。

 
## 完整 CI #542 结果与数学审查边界（2026-11-10）

commit `dafe47b8b17e7f32f676cb258eaa7a407a1dfb90` / Host validation #542 全部 jobs success。M4 Host 927/0、M5 Host 269,207/0、integration 1,000/0、M5 ASan/UBSan 269,207/0、integration ASan/UBSan 1,000/0；S06-080..084 本轮对应测试均执行并通过，Cortex-M7 M5-06 compile PASS。

S06-081 恒加速度 oracle 已从固定 513 点采样替换为解析候选点（端点、相对速度驻点、相对位置根），64 恒速 + 64 恒加速度 cases。本 oracle 使用 long double，仅对明确的 1D 恒加速度测试模型提供独立解析数值参考；不是任意 3D 相对运动的不确定度区间证明。

CI：[Host validation #542](https://github.com/hujunjie-hub/Flight/actions/runs/42746417706)。

**审查结论不变：BLOCKED / REVIEW_INCOMPLETE。** MR-001..MR-008 仍需逐项独立数学复核并给出 APPROVED；M5-07 不得开始。H723 目标运行、WCET/RTA、实机 RAM/stack、HIL 和飞行仍 NOT_RUN。

 
## 11. MR-009：验证曲线与消费曲线数值路径不一致（2026-11-10 本地复现）

状态：**OPEN / S2 / 证明范围缺口**（不是已证实的碰撞漏检）。

spec/02 要求“M5-06 验证的对象是这套实际发布/消费的分段曲线”。当前实现存在两条不同的数值路径：

- 消费端 `control_sample_reference()`（`middleware/model/control/outer_loop/quad_x4/position_tracking/control_reference.c`）用 **binary32** 系数和 Horner 求值五次 Hermite；
- M5-06 `build_curve()` / `curve_bounds()` 用 **binary64** 区间重建同一条代数曲线，`position_rounding_error_m` 只覆盖“中点值转 float”的误差，不覆盖两侧求值路径之差。

因此验证器证明的是 binary64 曲线，而实际被 M6 消费的是 binary32 曲线；两者的差没有进入任何保守包络。

**复现命令：**

```bash
python tests/guidance/verify_curve_eval_gap.py
```

固定 seed `0x6D350685`，256 段 × 每段 33 个采样点，两个寄存器：

| 场景 | 构造 | 实测最大偏差 |
|---|---|---:|
| 平滑段 | 端点由单个 20 ms 区间内的有界 jerk 积分生成（真实参考数据形态） | 2.98e-05 m |
| 对抗段 | 端点 p/v/a 相互独立随机，最大化 binary32 Horner 的抵消误差 | 2.17e-03 m |

偏差远小于当前 `clearance_margin_m` + 机体半径（≥0.15 m），没有发现因此漏检的碰撞；但它意味着验证结论与“实际消费曲线”之间存在一个未量化的 30 µm～2 mm 量级缺口，且随位置尺度和系数抵消程度增长。

**可选修复方向（需独立审查决定）：**

1. 在验证器内复刻消费端的 binary32 运算，并对每一步舍入做向外区间包络（严格，改动最大）；
2. 由 Horner 前向误差给出可证明的求值误差上界，作为 `position_rounding_error_m` 的附加项（需要非平凡推导，且对抗系数下界可能很松）；
3. 明确把该偏差划入 `clearance_margin_m` 的用途并由审查批准（当前文档没有这样声明，不能默认成立）。

在 MR-009 关闭前，M5-06 动态/静态连续距离的“消费曲线等价性”不得标记 APPROVED，M5-07 不得启动。`tests/guidance/verify_curve_eval_gap.py` 只是回归测量（上限 1e-4 m / 5e-3 m），不是证明。

 
### MR-009 修复实现（2026-11-10，本轮）

状态：**FIX IMPLEMENTED / WAITING_INDEPENDENT_REVIEW**（不再是“无包络”状态）。

实现（方案 2 + 消费端系数逐运算复刻）：

1. `consumer_power_coefficients()` 按 `control_sample_reference()` 的 binary32 运算顺序复刻 c0..c5；验证器不再只假设“同一条代数曲线”。
2. `consumer_eval_error_bound()` 给出
   `E = Σ|c_f,i − c_e,i|·s_max^i + 9u·Σ|c_f,i|·s_max^i`，
   其中 `u = 2^-24`；`9` 覆盖 5 次 Horner 舍入与最多 4 次可能被 FMA 融合的系数运算。
3. 时间量化项：消费端用 `s = (float)(offset_us) * 1e-6f`，验证器用精确时间；差值按 `2u·s_max·|d^k p/dt^k|` 的导数上界计入位置/速度/加速度。
4. 三种误差分别进入 `curve_bounds()`：位置加入 `position_low/high` 与 `position_rounding_error_m`；速度加入 `velocity_axis_bound`/`speed_bound`/`velocity_rounding_error_mps`；加速度加入 `acceleration_axis_bound`/`acceleration_bound`。不再依赖 `clearance_margin_m` 吸收。
5. 新增测试 API `m5_trajectory_safety_test_curve_envelope()`，暴露三条包络范数。

验证（同一提交）：

| 测试 | 内容 | 实测 |
|---|---|---|
| S06-086（C，真实消费端） | 链接 `control_sample_reference()`，24 区间 × 33/17 采样 × 9 个分量，平滑 + 8 组对抗参考 | 314,687 checks / 0 failures；位置、速度、加速度全部落在包络内 |
| S06-085（Python，Fraction 精确） | 同一 float32 端点；真实 `offset_us → s`；精确有理数参考；包络来自生产 C API | 平滑位置 1.52e-05 m、对抗 1.41e-03 m；速度 3.03e-01 m/s、加速度 5.90e+01 m/s²（对抗）；`max gap/envelope = 0.094`，`uncovered=0` |
| 消费端浮点语义 | 用 `rtconfig.py` 的生产 flags 编译 `control_reference.c`，反汇编统计 | `vfma/vfms/vmla/vmls = 0`；`build/m5_arm/consumer_fp.json` |

包络保守度约 10 倍（`max ratio 0.094`），对平滑参考只增加 ~1e-4 m 量级膨胀，不改变现有 PASS/FAIL 判定；对物理上不合理的对抗参考会显著收紧动力学门限（测试 `m5_safety_keepin_excursion` 的夹具已给出 100 m/s² 余量以继续覆盖围栏路径）。

**仍未关闭的部分：** 独立审查者需要确认 (a) `9u` 系数是否覆盖目标编译器的收缩行为（当前工具链实测无 FMA）、(b) 时间量化项的导数上界推导、(c) `s_max = h·u1` 的单调性论证。在这些确认前，MR-009 保持 OPEN，M5-06 整体保持 BLOCKED，M5-07 不得启动。



 
## 12. MR-009 数学修正（2026-11-10，R4）

R3 的包络有三处不满足严格误差模型，本轮按逐运算证明修正。

### 12.1 Horner 舍入次数（R3 用 9u，实际至少 10 次）

按 `control_reference.c` 中**分离乘加**（ARM 反汇编确认无 FMA）逐条计数：

| 量 | 表达式 | 乘法 | 加法 | 合计 n |
|---|---|---:|---:|---:|
| position | `((((c5*s + c4)*s + c3)*s + c2)*s + c1)*s + c0` | 5 | 5 | **10** |
| velocity | `(((5c5*s + 4c4)*s + 3c3)*s + 2c2)*s + c1` | 4 系数 + 4 次 `*s` = 8 | 4 | **12** |
| acceleration | `((20c5*s + 12c4)*s + 6c3)*s + 2c2` | 4 系数 + 3 次 `*s` = 7 | 3 | **10** |

误差项改为严格的 `γ_n = n·u/(1 − n·u)`（`u = 2^-24`），不再使用经验系数 9u。若目标编译器将来把乘加融合，n 只会变小（FMA 更精确），但系数计算会变化；`check_consumer_fp_contraction.py` 会在检测到 `vfma/vfms/vmla/vmls` 时失败并要求复核。

### 12.2 系数误差改用区间系数

R3 使用 `|cf[i] − c[i]|`，其中 `c[i]` 是 binary64 **标称**系数，不是精确数学值。现在改为使用验证器已有的向外区间系数 `c_i[i] = [low, high]`：

```
coef_err_i = max(|cf_i − c_i.low|, |cf_i − c_i.high|)   （向外取整）
```

position/velocity/acceleration 三组系数（velocity 为 `(k+1)·c_{k+1}`、acceleration 为 `(k+1)(k+2)·c_{k+2}` 的区间缩放）全部覆盖；减法、乘法、求和与最终相加均经 `nextafter` 向外。

### 12.3 次正规、溢出与舍入模式

`γ_n` 只在“无溢出、中间量处于正规范围、round-to-nearest”下成立。新增保守拒绝：

- `Σ|c_i|·s^i > FLT_MAX/16` → 溢出风险，返回 +Inf → `NUMERIC_FAILURE`；
- `0 < Σ|c_i|·s^i < FLT_MIN·1024` → 可能抵消进次正规区，返回 +Inf → 保守拒绝；
- 系数为 0 的完全零曲线不触发上述拒绝（`Σ = 0`）；
- 每种舍入再叠加 `n·2^-149` 的绝对下限，覆盖最终运算落在次正规区的情况；
- round-to-nearest 假设由 `FLT_EVAL_METHOD == 0`、IEEE-754 静态断言、显式浮点编译 flags 和 FMA 检查共同约束；固件不修改 FPSCR 舍入模式。

### 12.4 `velocity_mid_norm` 漏项

`swept_radius` 取

```
min(speed_bound·h/2, |v_mid|·h/2 + 0.5·a·(h/2)²) + position_rounding_error
```

R3 只把 `eval_velocity_error` 加进 `speed_bound` 与 `velocity_rounding_error_mps`，没有加进 `velocity_mid_norm`，导致第二候选可能低估。现在 `velocity_mid2` 累加 `|v_mid| + eval_velocity_error` 的平方，两个候选各自独立成立后才取 `fmin`。

### 12.5 修复后的覆盖证据

| 测试 | 结果 |
|---|---|
| S06-085（Fraction 精确参考） | smooth 位置 `1.52e-05 m`、adversarial `1.41e-03 m`；`max gap/envelope = 0.089`；`uncovered = 0` |
| S06-086（真实 `control_sample_reference`） | 314,731 checks / 0 failures（Host 与 ASan/UBSan） |
| S06-087..093（端到端判定） | 静态穿越、接缝、动态障碍、围栏边界、自适应细分、最大偏差参考全部按预期拒绝或通过 |
| FMA 检查 | `vfma/vfms/vmla/vmls = 0`（生产 flags） |

**状态：FIX IMPLEMENTED / WAITING_INDEPENDENT_REVIEW。** 独立审查仍需确认：`γ_n` 前提（正规数、无溢出、RN）、区间系数与 binary32 复刻的一致性、时间量化导数上界、`s_max = h·u1` 单调性。MR-009 未 APPROVED 前 M5-06 保持 BLOCKED，M5-07 不得启动。

 
## 13. R5 证明闭环（2026-11-10）

R5 不再修改误差公式，而是逐项证明 R4 实现是保守的，并把假设变成运行时检查：

1. **γ_n 推导**：`M5_06_6_MR009_PROOF.md` 给出 Higham 引理、Horner 的 2n 计数（10/12/10）、系数区间项、时间量化 2u·s_max、次正规 n·2^-149（不依赖 s_max≤1）与溢出/次正规拒绝条件。
2. **γ 向上取整**：`nu` 与商各补一次 `nextafter`，时间缩放改 outward。
3. **运行时圆整模式**：入口读 FPSCR/`fegetround()`，不满足即 `NUMERIC_FAILURE`；负向用例 `m5_safety_floating_environment`。
4. **浮点环境审计**：`spec/validation/fp_environment_audit.md` 记录 3,362 文件扫描、无 FPSCR 写、Cortex-M7 只保存 s16–s31、启动代码不在仓库（`packages/` 缺失）→ 以运行时检查取代假设。
5. **FMA**：消费端与验证器两个对象都检查，记录对象 SHA256 与完整 flags。
6. **连续覆盖**：S06-094 用精确有理数 + Bernstein 覆盖整个区间，`0.970` 的上界比仍有余量。
7. **围栏缺口**：`check_geofences()` 之前没有减去 `position_rounding_error_m`，R5 修复并由 S06-097 覆盖。

审查状态：**六项分别有证据，等待独立审查逐项 APPROVED**；MR-009 不因整体 CI 通过而关闭。

 
## 16. R6：时间量化与次正规证明修正（2026-11-10）

R5 的时间不等式 `(2u+u²)s ≤ 2u·s_max` 不成立，R6 改为以 `s_exact = h·(offset/step)`
为统一参考、`γ_2 = 2u/(1−2u)` 为界，并把所有系数/Horner/导数界扩到
`s_eval_max = s_max + γ_2·s_max`。次正规项改为带幂次放大的
`n·2^-149·max(1,s_eval_max)^n`，并以 `PRODUCT_PREVIEW_STEP_US < 1e6` 作为产品约束。

新增 S06-099（60,002 个整数偏移穷举）与扩展的 S06-094（120 组子区间）。S06-100..102
标记 NOT_IMPLEMENTED / BLOCKED_GEOMETRY_RESOLUTION。Host 侧新增 MXCSR RC/FTZ/DAZ
检查与 5 个负向用例。

审查状态不变：**六项证据齐备但未 APPROVED**，MR-009 不因整体 CI 通过而关闭。
