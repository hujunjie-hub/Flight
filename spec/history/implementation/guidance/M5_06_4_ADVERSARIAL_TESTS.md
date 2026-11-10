# M5-06.4 对抗性安全验证与边界测试

状态：**新增边界用例已通过 Host/SIL、ASan/UBSan 与 Cortex-M7 编译回归；M5-06 整体仍未验收。**

## 目的

验证独立安全验证器不会因离散预览点安全而漏掉连续区间碰撞，并确认输入不完整、状态过期、几何非法和动态预测不确定时均 fail-closed。

## 本轮新增覆盖

| DFS work-unit 预算 | 每个 20ms 区间需要深度 5 才能证明 UNKNOWN 栅格外的净空，跨区间累计超过 256 | BUDGET_EXCEEDED | PASS：第 257 个工作单元时拒绝，cell-check 未达到 8192 |
| 静态地图 cell-check 预算 | 宽扫掠 FREE 地图体素，超过 8192 次检查 | BUDGET_EXCEEDED | PASS：第 8193 次检查时拒绝 |

| 测试 | 构造 | 预期 | 实际 |
|---|---|---|---|
| 静态中段碰撞 | 24 区间连续轨迹中段穿过占用 voxel | STATIC_COLLISION | PASS |
| 动态中点碰撞 | 动态障碍物位于车辆轨迹中点 | DYNAMIC_COLLISION | PASS |
| 动态交叉碰撞 | 车辆与障碍物以交叉速度穿越同一位置 | DYNAMIC_COLLISION | PASS |
| KEEP_IN 连续越界 | 预览端点位于围栏内，曲线中段越界 | GEOFENCE_VIOLATION | PASS |
| STALE voxel | 轨迹膨胀区覆盖 STALE cell | STALE | PASS |
| INVALID voxel | 栅格状态编码非法 | INVALID | PASS |
| 动态租约不足 | track expires_us 早于 proof_end | STALE | PASS |
| 动态 track 元数据非法 | flags 非零或 source_id 为零 | STALE | PASS |
| 动态 track 缺少加速度界 | 存在 track，但 max_dynamic_obstacle_accel_mps2 为零 | INVALID | PASS |
| 重复 track | 同一 track 身份重复出现 | INVALID | PASS |
| 动态加速度上界 | 较低加速度界可通过，批准上界导致预测包络碰撞 | PASS / DYNAMIC_COLLISION | PASS |
| 导航有效位缺失 | 清除必需导航 valid bit | INVALID | PASS |
| 地图 revision 失配 | candidate revision 与当前 map 不同 | INVALID | PASS |
| 自交围栏 | bow-tie polygon | INVALID | PASS |
| 预览点 offset 错误 | 单点 offset 偏离 20ms 均匀采样契约 | INVALID | PASS |
| Inf 输入 | 预览点速度含 Inf | INVALID | PASS |
| 导航 epoch/origin/frame | state_epoch、origin_id 不匹配或 frame 非 LOCAL_NED | INVALID | PASS |
| 围栏 origin | fence origin_id 与导航不一致 | INVALID | PASS |
| 围栏重复顶点 | 相邻顶点重复形成退化边 | INVALID | PASS |
| 动态 track 时间 | sample 在未来或超过陈旧阈值 | STALE | PASS |
| 动态 track 数值 | position 为 NaN 或 radius 为负 | STALE | PASS |

## 测试夹具问题及修复

测试回归曾统一返回 REFERENCE_INVALID。根因不是验证器接受了错误轨迹，而是测试辅助函数 `safety_point_set()` 对整个 MotionReferencePoint 执行 `memset`，把调用方提前设置的 `offset_us` 清零。

修复方式：保留原 offset，再清零并设置运动数据与有效位。修复后 24 个点恢复为严格的 20ms 间隔，所有新增安全用例通过。

## 算法保守性修正

导航不确定度随预测时间传播。此前静态障碍和围栏膨胀使用子区间中点时域，未覆盖区间远端。现将不确定度时域改为：

`horizon_mid + subinterval_half_duration + response_delay`

使膨胀覆盖整个子区间最远时刻及响应延迟。动态 track 预测原本已包含子区间半宽。

## 新增几何差分、Bernstein 数值 oracle、多 track 与连续距离界 Fuzz（S06-064..079 已通过）

- 独立参考实现使用 winding-number 分类和双精度点到线段距离，与生产实现的 ray-parity 分类进行差分。
- L/C 凹多边形、CW/CCW 绕序、共线边和近共线边；随机点、顶点、边中点及边两侧；KEEP_IN/KEEP_OUT 均做差分，阈值 2e-5m。
- 16 组 8 顶点交替半径凹多边形与动态 track 联合验证；每组另抽 32 个点做 clearance oracle 差分。
- 新增相邻共线回折重叠边、非相邻重复顶点的 fail-closed 几何拒绝用例。
- 32 组动态 track 属性测试扩展：在有效 PASS 基线上分别破坏速度不确定度、预览 offset、map time_epoch、track sample timestamp，必须 fail-closed。
- DFS 栈保护测试明确断言 29 项容量、最大深度 5、workspace≤512B，并保留前后 canary。

以上用例已在 Host、ASan/UBSan 与 Cortex-M7 编译工作流通过，详见最新证据。

- 256 组随机 Hermite 起终端状态、随机 duration 和非 dyadic 子区间；每组 65 个采样点以零容差检查 position control-hull bounds、速度范数和加速度范数上界；每组 33 个点直接比较生产 Bernstein 曲线评估与独立 Hermite 基函数参考值。
- 额外拒绝 duration≤0、u0/u1 反向或越界、local_u 越界。
- 16 组 1..8 个动态 track 的多目标组合；覆盖完整 8 个 track slot、随机速度、future sample、duplicate identity，以及 7 个安全 track 中夹杂 1 个碰撞 track 的 fail-closed 行为。
- 2,048 组随机 point-to-AABB 距离下界 oracle：以独立双精度 AABB 距离计算作为参考，验证生产下界扣除位置舍入误差后不高估净空；补充 face/edge/corner 精确接触和非法输入拒绝。
- Hermite 系数、子区间 power transform、Bernstein 控制点及一/二阶导数界采用双精度 outward interval 运算；区间加减乘除每步使用 `nextafter` 向外扩张。曲线中点位置/速度误差也纳入 swept-volume inflation；区间溢出/非法区间直接拒绝。
- 子区间 duration 保持 double 参与导数计算，并将 half-duration 向上取整，避免 float 时长舍入低估扫掠范围。
- 零位置/速度/加速度曲线专门验证区间 hull 包含精确零值。
- 512 组动态相对运动组合，每组 33 个采样时刻：相对初速度、车辆/障碍物加速度、速度不确定度和子区间时长随机化，检查生产 sweep 上界始终包络参考相对位移；非法速度/加速度/时间参数拒绝。
- 将静态 voxel 距离与动态 relative sweep 计算提取为小型纯函数，生产验证器和 Host oracle 共用同一实现；测试 API 仅在 Host/SIL 的 `FLIGHT_M5_SAFETY_TEST_API` 编译宏下启用，不进入 H723/NONE 产品配置。
- S06-074：64 组静态 voxel 线性扫掠端到端差分，32 组轨迹穿过占用 voxel，32 组障碍物与轨迹保持独立 oracle 计算出的净空；分别要求 STATIC_COLLISION / PASS。
- S06-075：64 组动态恒速相对运动端到端差分，独立解析相对运动最近接近时刻，32 组碰撞和 32 组安全近失；分别要求 DYNAMIC_COLLISION / PASS。
- S06-076：64 组动态障碍物有界加速度差分；实际障碍物加速度不超过 0.45m/s²，输入批准测试上界为 0.5m/s²，每组用 257 个时间样本建立独立参考，32 组碰撞拒绝、32 组安全场景通过。
- S06-077/078：静态障碍物在 20ms 预览区间接缝，以及动态障碍物在自适应细分区间和预览接缝穿越；要求分别返回 STATIC_COLLISION / DYNAMIC_COLLISION。
- S06-079：64 组全时域五次 Hermite 轨迹，使用独立 Hermite 基函数构造 24 个消费者 knot；每组对轨迹/AABB 距离采样 1,025 个时刻，32 组路径穿过占用 voxel、32 组保持大于 0.70m 的 AABB 净空，分别要求 STATIC_COLLISION / PASS。

## 最新证据

- CI：[Host validation #438](https://github.com/hujunjie-hub/Flight/actions/runs/42720186326)
- Validated code commit：`cfbb6b6b6c9ec484b83c5ab36a4b3e3da5050d2d`
- M4 Host：927 / 0 failures
- M5 Host：261,669 / 0 failures
- M4/M5 Integration：1,000 / 0 failures
- M5 ASan/UBSan：261,669 / 0 failures
- M4/M5 Integration ASan/UBSan：1,000 / 0 failures
- Cortex-M7 compile/RAM and M4/M5 production-source compile：PASS
- MPC RAM：29,252 bytes；worker stack：8,192 bytes；largest single frame：2,024 bytes
- Source/time and Navigation/Control：PASS
- All workflow jobs：success
- Cortex-M7 compile and MPC RAM checks：PASS
- Source/time and Navigation/Control host jobs：PASS
- All workflow jobs：success

## 本轮新增测试结果

S06-074..079 静态/动态端到端扫掠差分、加速度包络、预览接缝、自适应细分和全时域五次 Hermite 轨迹扫掠用例均在 run `42720186326` 的 Host/ASan/UBSan 回归通过。S06-053..063 已在新 Head 执行通过：reference/map time_epoch、directive frame、动态 sample age/预测租约、地图预测时域、围栏高度边界/退化高度、uint64 proof horizon 溢出、有限极值计算溢出及固定 seed 32 组属性测试。首轮 fixture 缺陷已修正；以上结果仅代表 Host/SIL 与 Cortex-M7 编译检查，不是 H723 实机避障放行。

## 下一批必须补齐

1. 独立复核 position/velocity/acceleration Bernstein control-hull bound 的数学推导、双精度 outward interval 算术的 IEEE-754/编译器假设及 `nextafter` 舍入策略；当前随机采样 oracle 不是穷尽证明。
2. 将 point-to-AABB 与 dynamic relative sweep helper oracle 扩展为端到端 swept-volume/continuous-distance 差分，覆盖边界擦碰、相邻区间接缝及自适应细分边界。
3. 扩大多 track 时间戳、预测 lease、source identity、速度/加速度不确定度和相对运动组合 Fuzz，保存 seed 与最小反例。
4. 动态障碍物加速度上界的传感器/目标类别标定与参数敏感性分析。
5. DFS 深度与最大工作栈压力；两个独立预算（work-unit、cell-check）均已由可达用例验证。
6. 随机/属性 fuzz，并对关键几何边界建立可复现最小反例。

## 本轮追加用例（S06-053..067 已在新 Head CI 通过）

- 动态 track 样本年龄 500ms 等值与超 1us；预测租约 1s 等值与超 1us；地图预测时域不足拒绝。
- KEEP_IN 围栏上下高度 margin 内外侧；精确下边界保守拒绝；零高度区间 INVALID。
- 24 个预览区间时间加法 uint64 溢出拒绝。
- 有限极大动态速度引发的数值计算溢出拒绝。
- 固定 seed `0x6D350607` 的 32 组常量轨迹属性测试：PASS 不变量与 offset 单点破坏后的 INVALID。

首轮 CI（run `42648938161`）中，动态时间边界和 proof-time overflow 通过；围栏高度用例错误地未把 vehicle radius + clearance inflation 计入安全高度，随机轨迹超出 map readiness 半径，且围栏高度 fixture 未随高度重新居中地图，有限极值未形成组合溢出。以上属于测试夹具/断言设计缺陷，已修正，必须以新 Head 重新执行 Host、ASan/UBSan、ARM 构建。

## 放行约束

- 本文的 PASS 仅代表当前 Host/SIL 自动化测试通过。
- M5-05 仍只输出 candidate。
- M5-07 尚未把 M5-06 PASS 绑定为 FMS/Permit/M6 的不可绕过发布门禁。
- 不得将 Host/SIL 结果解释为 H723 实机避障放行；真实传感器标定、WCET/RTA、RAM/栈、HIL 和场地验证仍未完成。


## M5-06.5 独立数学审查与后续边界用例（2026-11-10）

详见 `M5_06_5_INDEPENDENT_MATH_REVIEW.md`。初审提出的 libm sqrt 舍入风险已在后续代码中通过 binary32 位模式有界二分搜索替代；并新增 IEEE 754 binary32/binary64 静态约束、显式浮点编译 flags、Cortex-M7 Host-only validator object 交叉编译，以及 Fraction 精确平方范数 oracle。上述实现当前等待新 commit 的 CI 证据和独立数学复核，不应沿用旧版 sqrt 风险描述作为当前代码事实。

预留测试 S06-080..083，固定 seed 分别为 `0x6D350680`.. `0x6D350683`，覆盖 sqrt/float ULP、动态端到端连续距离、精确相切与接缝、自适应分裂点 1 ULP 边界。以上全部 **NOT_RUN**，不得将编号或 seed 当作已执行证据。

当前 ARM 脚本已将 M5-06 Host-only validator translation unit 交叉编译为 Cortex-M7 object，但不链接进产品；完整目标 CI 结果仍待本轮提交确认。MPC RAM 数字不是 M5-06 validator RAM，object 编译也不构成 H723 运行期资源或飞行放行证据。

## S06-082 动态切触、ULP 与预览接缝边界（2026-11-10）

新增 `test_m5_safety_dynamic_tangency_and_preview_seam_ulp`：构造静止车辆与动态障碍物的精确膨胀半径切触、内侧 1 ULP、外侧 1 ULP 和有明确净空的安全对照；外侧 1 ULP 允许保守拒绝，但禁止 PASS/碰撞边界漏放。另构造障碍物在 20ms preview seam 精确穿越，以及起始位置向两侧各偏移 1 ULP 的穿越场景，均要求 DYNAMIC_COLLISION。确定性边界用例，无随机 seed。状态 **IMPLEMENTED / CI_PENDING**。

## S06-081 动态障碍物恒加速度连续距离 oracle（2026-11-10）

新增 `test_m5_safety_dynamic_constant_acceleration_continuous_oracle`：64 组恒加速度动态障碍物场景，32 组碰撞、32 组安全近失。独立参考将车辆与障碍物的相对位置表示为二次多项式，检查区间端点、相对位置驻点及二次方程零点，计算整个预测时域的连续最小距离；不是固定时间采样 oracle。实际加速度绝对值限制在 0.45m/s²，validator 输入上界为 0.5m/s²。固定 seed `0x6D350681`。当前 **IMPLEMENTED / CI_PENDING**，等待 Host、ASan/UBSan 和 Cortex-M7 编译回归。

## S06-080 精确有理数范数 oracle（2026-11-10）

新增 `tests/guidance/verify_norm_bounds.py`，通过生产 C test API 获取 upper/lower binary32 结果，Python `Fraction` 对输入 binary32 值精确还原并比较平方范数，避免使用浮点 sqrt 作为唯一判据。固定 seed `0x6D350680`；覆盖零向量、单位向量、`sqrt(1 + 2^-160)`、最小次正规数、最大有限数及 10,000 组任意有限 binary32 位模式。已接入 Host runner；当前状态 **CI_PENDING**，尚无本次提交的 PASS 记录。
 
## S06-080..083 源码复核补充（2026-11-10）

- S06-080 已使用 Python `Fraction` 精确平方范数 oracle，覆盖 binary32 上界和 binary64 下界各 10,007 组，固定 seed `0x6D350680`；binary64 扩展由 commit `61dbc4c` 加入。
- S06-081 已加入恒速解析最近点和恒加速度解析候选点两组端到端差分，每组 64 cases。恒加速度参考使用 long double 计算端点、驻点与相对位置根；它不是形式化区间 oracle，且随机不确定度传播组合仍待扩展。
- S06-082 已加入精确相切、内侧/外侧 1 ULP、明确安全净空，以及 20ms seam 精确位置和 ±1 float ULP 的动态碰撞测试。
- S06-083 已加入自适应细分点碰撞测试，覆盖精确起点和 ±1 float ULP，断言结果为 DYNAMIC_COLLISION、passed=0 且 work_units 确实超过预览区间数。
- 上述新增用例当前均为 **IMPLEMENTED / CI_PENDING**。在对应新 commit 的 CI 运行完成前，不得把源码存在误报为测试通过。

 
## 新增阻塞：MR-008 动态预测位置舍入误差（2026-11-10）

代码复核发现，动态障碍物预测位置的误差项只覆盖 float 转换与 double 点估计的差值，没有覆盖时间中点量化、微秒到秒换算和 double 乘加的完整误差。新增 S06-084，要求用 Fraction 精确时间/位置 oracle 覆盖大 epoch、正负时间差、非 dyadic 子区间中点及相切/近切。固定 seed `0x6D350684`，当前 NOT_RUN。

在生产实现建立时间与位置的 outward interval，并以预测 float 点到区间两端的最大距离形成误差上界之前，动态 track 连续距离检查保持阻塞；不得将 S06-081..083 的源码实现或后续 CI 通过解释为数学证明完成。

 
## S06-084 动态预测时间/位置区间修复（2026-11-10）

动态 track 预测不再使用 float 中点时间转整数微秒后直接点估计。生产实现现以相对 reference epoch 的时间差构造 outward interval，包住 (10^{-6}) 换算和 position/velocity 乘加，并以 float 预测点到区间两端的最大距离作为误差上界。预测区间无效/溢出时返回数值失败，保持 fail-closed。

新增 Fraction oracle：7 个显式边界 + 10,000 个固定 seed `0x6D350684` 的随机输入；检查三轴精确位置误差范数不超过生产误差上界，并验证超范围预测拒绝。当前 **FIX IMPLEMENTED / CI_PENDING**，不是测试 PASS。


S06-084 另增加大绝对 epoch 端到端不变性测试，使用接近 uint64 上限但保留完整预测租约和 proof horizon 的时间戳；相同相对时间下必须与低 epoch fixture 得到相同安全结果。Fraction oracle 覆盖 10,007 组输入。当前仅 IMPLEMENTED / CI_PENDING。

 
## CI #499 compile portability failures

Run #499 did not execute M5 tests: Host compilation rejected a floating-point equality inside `_Static_assert`; Cortex-M7 compilation rejected the optional `__STDC_IEC_559__` macro guard. M4 Host and independent source/time/navigation checks passed; MPC static RAM check passed. Fixes: `b0f40cb`, `0349587`. Await Run #501 before rerunning adversarial cases; do not classify this as an algorithm test failure or a pass.

 
## CI #509 adversarial and sanitizer result

Commit `0c761964d4bc6cfeec73ba60464657db450743ee`:
- M5 Host 269,207 / 0 failures.
- M5 ASan/UBSan 269,207 / 0 failures.
- M4/M5 integration 1,000 / 0 failures.
- Integration ASan/UBSan 1,000 / 0 failures.
- S06-080 Fraction oracle: 10,007 binary32 upper + 10,007 binary64 lower PASS.
- S06-084 Fraction prediction oracle: 10,007 PASS.
- S06-083 adaptive split exact/±1 ULP now verifies actual parent→child processing with `work_units > 1`; all cases PASS.

This closes the CI execution gap for these test suites at the verified commit. It does not close the independent proof review, H723 target runtime, HIL, or flight release gates.

 
### S06-081 constant-acceleration reference correction

The constant-acceleration differential now computes the continuous minimum analytically from endpoint, relative-velocity stationary, and relative-position zero candidates. The prior 513-point grid oracle has been removed. The analytic test covers its defined 1D constant-acceleration + fixed lateral-offset model; broader uncertainty and 3D acceleration interval proof remain open. Commit `a14b9f1`; CI_PENDING.

 
## CI #509 对抗性回归状态（2026-11-10）

代码基线 `0c761964d4bc6cfeec73ba60464657db450743ee` 对应 Host validation #509 全部 jobs success。M5 Host 269,207 checks / 0 failures，M5 ASan/UBSan 269,207 / 0；M4/M5 integration 与 integration sanitizer 均为 1,000 / 0。S06-082 的动态精确相切、内外 1 ULP 和 20ms preview seam 用例通过；S06-083 的自适应细分精确位置及 ±1 ULP 用例通过，且 work_units 断言确认实际进入细分。S06-081 动态连续距离差分用例通过。

CI：[Host validation #509](https://github.com/hujunjie-hub/Flight/actions/runs/42745124335)。

该结果仅证明指定 commit 的自动化测试通过；有限采样/数值 oracle 仍不能代替独立数学证明。MR-001..MR-008 的审查状态和 M5-07 门禁按 `M5_06_5_INDEPENDENT_MATH_REVIEW.md` 执行，当前不得宣称 M5-06 整体验收或 H723 实机避障放行。

 
## 完整 CI #542 复测更新（2026-11-10）

commit `dafe47b8b17e7f32f676cb258eaa7a407a1dfb90` 对应 Host validation #542 全部 jobs success。M5 Host 269,207 checks / 0 failures；M5 ASan/UBSan 269,207 / 0；M4/M5 integration 与其 Sanitizer 均 1,000 / 0。S06-081 恒速/恒加速度连续距离 oracle、S06-082 相切/ULP/seam、S06-083 自适应细分边界均实际执行并通过。

CI：[Host validation #542](https://github.com/hujunjie-hub/Flight/actions/runs/42746417706)。

long-double 恒加速度 oracle 仅覆盖其明确的 1D 运动模型；不得将其描述为通用 3D 连续区间证明。M5-06 独立数学审查和 H723 实机放行仍未完成。

 
## S06-085 验证曲线与消费曲线求值差分（2026-11-10，MR-009 修复后）

`tests/guidance/verify_curve_eval_gap.py`（固定 seed `0x6D350685`）比较两条求值路径：

- 消费端 `control_sample_reference()` 的 binary32 Horner（Python 逐运算复刻，真实 C 路径由 S06-086 覆盖）；
- 精确有理数参考（`Fraction`），与消费端使用**同一组 binary32 端点**，采样时间使用消费端真实的 `offset_us → s` 换算；
- 包络来自生产 C API `m5_trajectory_safety_test_curve_envelope()`，不在此脚本内重新实现。

| 场景 | 位置最大偏差 | 速度最大偏差 | 加速度最大偏差 | max gap/envelope |
|---|---:|---:|---:|---:|
| 平滑段（恒 jerk 积分端点） | 1.52e-05 m | 1.15e-06 m/s | 1.42e-04 m/s² | 0.092 |
| 对抗段（端点 p/v/a 独立随机） | 1.41e-03 m | 3.03e-01 m/s | 5.90e+01 m/s² | 0.094 |

`uncovered = 0`：所有采样点的精确偏差都落在生产包络内。脚本输出 `M5_S06_085_EXACT_ENVELOPE_COVERAGE_PASS`，任一采样越界即 FAIL。

## S06-086 真实消费端与验证器曲线的 C 差分（2026-11-10）

`tests/guidance/test_m5_main.c::test_m5_safety_consumer_curve_envelope` 把 M6 的 `control_sample_reference()` 直接链接进 Host 测试二进制，按真实 `offset_us` 采样，与 `m5_trajectory_safety_test_curve_eval()` 的 binary64 曲线逐点比较，并用 `m5_trajectory_safety_test_curve_envelope()` 的包络判定：

- 平滑参考：24 区间 × 33 采样点 × 3 轴 × 3 个分量；
- 对抗参考：8 组独立随机端点，24 区间 × 17 采样点；
- 同时断言包络严格大于 0，防止用零裕度“通过”。

实测：`314,687 checks / 0 failures`（Host 与 ASan/UBSan 相同）。该用例证明验证器现在覆盖的是控制器实际执行的曲线，而不只是同一条公式的 binary64 版本。

