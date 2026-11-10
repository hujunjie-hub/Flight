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
| MR-004 | sqrt 与浮点上下界转换 | `norm3_upper()` 未在 sqrt 结果转 float 前显式向 +∞ 扩张；当精确根值极接近 float 边界时，sqrt 的向下舍入可能使 round_up_float 仍返回低于精确值的 float。C 数学库精度契约及 Cortex-M7 实际实现尚未被本仓库验证 | BLOCKER |
| MR-005 | point-to-AABB 下界 | 输入为 float，差值/平方在 binary64 下有较强精确性基础；但应把完整误差论证写成可检查的不变量，并覆盖零距离、次正规数和极端尺度 | OPEN |
| MR-006 | 动态相对运动扫掠界 | 取 speed-based 与 acceleration-based 两个上界的较小值，只有在两者分别对整个子区间成立时才保守；需独立解析/区间 oracle 对速度不确定度、加速度界、时间端点和舍入共同验证 | OPEN |
| MR-007 | 编译器/目标约束 | 当前 ARM 编译脚本未显式声明 `-fno-fast-math`、舍入模型及浮点收缩策略；默认选项虽未启用 fast-math，但应把安全假设显式锁定并检查宏/编译命令 | BLOCKER |

## 2. 具体数学说明

### MR-004：sqrt 上界

当前实现：

```c
static float norm3_upper(const float v[3])
{
  const double x = v[0], y = v[1], z = v[2];
  return round_up_float(sqrt(x * x + y * y + z * z));
}
```

`round_up_float` 只能保证结果不小于传入的 double 值，不能修复此前 sqrt 已向下舍入造成的误差。若精确范数略高于某个 float 可表示值，而 sqrt 的 double 结果略低于该边界，向上转 float 仍可能落在精确范数之下。

建议修复方向：将平方和累积作为明确的上界，并在 sqrt 结果向 float 转换前，按已声明且可验证的 sqrt 误差契约向 +∞ 扩张；若目标数学库不能提供所需误差保证，则采用可证明的保守替代实现。不得只凭随机采样通过来关闭此项。

`curve_bounds()` 中部分 sqrt 调用已有 `nextafter(..., INFINITY)`，但仍需将统一的平方和、sqrt、float 转换规则抽成单一审计过的 helper，避免不同路径的舍入策略不一致。

### MR-003 / MR-007：区间运算前提

当前 interval helper 对每次基本运算调用 `nextafter` 扩张一个 double ULP。该证明依赖：
- binary64 运算遵守 IEEE-754；
- 基本运算按 round-to-nearest 正确舍入；
- 不允许 fast-math、代数重结合或不受控的表达式收缩改变运算序列；
- `nextafter`、`INFINITY`、`isfinite` 在 Host 与目标工具链上具有预期语义；
- 编译器和数学库版本纳入构建证据。

当前 `tests/guidance/arm_build_check.sh` 使用 `-std=c11 -Wall -Wextra -Werror`，但没有把上述安全浮点选项显式写入命令。应锁定选项并将完整命令/编译器版本保存到 CI artifact。

## 3. 必须新增的测试（尚未执行）

以下编号预留；在代码、CI 实际运行并保存日志前，状态一律为 NOT_RUN。固定 seed 用于可复现生成，不代表已经获得结果。

| ID | 场景 | 独立 oracle / 断言 | Seed | 状态 |
|---|---|---|---|---|
| S06-080 | norm3 上/下界在 float ULP 边界、零、次正规数、极端尺度的差分 | Fraction 精确有理数平方范数 oracle；upper 不低估、lower 不高估 | 0x6D350680 | IMPLEMENTED / CI_PENDING |
| S06-081 | 动态相对运动端到端连续最小距离，覆盖恒速解析最近点、恒加速度相对运动及不确定度膨胀 | 独立解析解 + 区间分支定界；记录最小反例，不以固定时间采样作为唯一 oracle | 0x6D350681 | NOT_RUN |
| S06-082 | 动态障碍物精确相切、内外各 1 ULP、相邻预览区间接缝 | 解析最小距离与生产 PASS/碰撞分类一致；相切按保守规则拒绝 | 0x6D350682 | NOT_RUN |
| S06-083 | 动态障碍物在自适应细分的分裂点前、恰在分裂点、分裂点后各 1 ULP | 对整段与分段独立 oracle 等价；任一段未证明必须 fail-closed | 0x6D350683 | NOT_RUN |

所有随机测试必须记录：测试 ID、seed、生成器版本、case 数、首次失败输入、缩减后的最小反例、预期/实际状态、CI run 与 commit SHA。不得把采样 oracle 描述成连续数学证明。

## 4. 进入 M5-07 的前置条件

1. 修复并关闭 MR-004、MR-007。
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