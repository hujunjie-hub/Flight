# M5-04 / M5-05 实施增量：有界时标器与 Host minimum-jerk 内核

- 基线：Flight / QUAD_X4 / V1.9.1
- 实施分支：`opt/stage-01-host-validation`
- 适用范围：M5 局部规划时间边界；Host/SIL minimum-jerk 数学内核
- 非目标：本报告不声明 A*、局部地图、障碍感知、M5-06、H723 实时性或飞行放行完成。

## 1. M5-04 有界时间参数化

### 修改

`middleware/guidance/trajectory_planning/mission_trajectory.c`：

1. 统一使用 200,000 us 时间格。
2. 单次规划最多 31 格，即 6.2 s。
3. 初始时长和每次 ×1.25 的增长均向上量化到时间格。
4. 最多执行 `GUIDANCE_MAX_DURATION_ATTEMPTS=16` 次尝试。
5. Mission/Return 不再有 240 s 长航段特例；单段超出 6.2 s 或在 6.2 s 内无法满足约束时拒绝。
6. M5 不擅自把长任务切成未授权子任务；长距离分段由 M4 负责。

### 行为变化

此前 Mission/Return 可能把一个长航段拉长到 240 s，与 V1.9.1 §7 的 31 个 200ms time_bin、6.2 s 局部搜索域冲突。现在 18 m Mission/Return 用例验证为 fail-closed 拒绝。

**限制：** 当前 M4 自动中间航点/任务分段尚未完成。长距离任务不能因 M5 拒绝而被描述为“已自动分段继续执行”。

## 2. M5-05 Host/SIL minimum-jerk 数学内核

### 文件与构建隔离

新增 `middleware/guidance/trajectory_planning/minimum_jerk_optimizer.c`。

- 仅定义 `FLIGHT_M5_HOST_PLANNER` 时编译优化器实现。
- `tests/guidance/run_host_tests.sh` 显式启用该宏。
- H723/NONE 当前直接 quintic 生产路径不调用该优化器，也不实例化其 workspace。
- `GuidanceMinimumJerkWorkspace` 为调用者拥有的固定 workspace，头文件以 compile-time assertion 限制 `sizeof(workspace) <= 2304 B`；调用者集成时应放在静态存储区，不应放在线程栈或运行期 heap。

### 数学定义

- 最多 8 段、最多 14 个内部变量。
- 所有段时长固定在 200ms 格点，总时长不超过 6.2s。
- 每个 knot 的位置固定；首尾 p/v/a 固定；只优化内部 knot 的速度和加速度。
- 目标函数为三轴 jerk 平方积分之和：
  
  \[
  J=\sum_{axis\in\{N,E,D\}}\sum_{i=0}^{N-1}
  \int_0^{T_i}\left(p_{axis,i}^{(3)}(t)\right)^2dt
  \]

- 每段的 jerk 能量是其边界速度/加速度的二次函数；按相邻段局部组装，得到带宽 3 的块三对角 SPD 系统。
- 使用固定维数、原位带状 Cholesky 和前/回代求解；没有动态内存分配。
- 优化后重新计算精确目标函数；若任一轴非有限、SPD 求解失败或目标函数未下降到容差内，返回失败。
- 输出先写入 workspace 候选区，所有轴成功后一次性复制；失败时调用者可观察到清零输出，不会拿到半条轨迹。

### 安全边界

该模块是数学候选生成器，不是完整轨迹规划器或安全验证器。它不做：
- Kinodynamic A* 或局部地图搜索；
- 速度、加速度、jerk、推力、倾角和安全包线验收；
- 围栏、障碍物、动态预测或连续碰撞证明；
- 优化失败后的初始路径回退与 M5-06 重新验证；
- MotionReference READY/ACTIVE 生命周期或 M5→M6 发布。

任何接入必须遵循“优化候选 → 完整 M5-06 验证 → 原子发布”；验证器 UNKNOWN/REJECTED 时不得发布。

## 3. Host 回归覆盖

`tests/guidance/test_m5_main.c` 当前包含 32 个命名场景。minimum-jerk 新增覆盖：

- 单段边界条件；
- 双段连续位置/速度/加速度及目标函数下降；
- 非等时长多轴路径；
- 8 段上限与重复运行确定性；
- 非法段数、非法时长格、总时长超限、非有限输入和失败时输出清零；
- 三轴目标函数正确求和；
- 非零起止速度/加速度边界。

M5 时标回归包括超出 6.2 s 的 Mission/Return 拒绝。

从仓库根目录执行：

    bash tests/guidance/run_host_tests.sh
    bash tests/guidance/run_integration_tests.sh

    EXTRA_CFLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer" \
      bash tests/guidance/run_host_tests.sh
    EXTRA_CFLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer" \
      bash tests/guidance/run_integration_tests.sh

    bash tests/guidance/arm_build_check.sh

`run_host_tests.sh` 编译 Host/SIL optimizer kernel；`run_integration_tests.sh` 仍验证现有 M4/M5 生产切片，不代表 optimizer 已接入完整闭环。`arm_build_check.sh` 只交叉编译其显式列出的 M4/M5 生产源；不编译 Host-only optimizer，也不是目标链接、WCET 或 H723 运行证据。

## 4. 状态与后续工作

- M5-04 有界时间参数化：实现切片已完成，长航段分段依赖 M4，整体能力仍 IN_PROGRESS。
- M5-05 minimum-jerk Host/SIL 数学内核：实现切片与 Host 单测已增加，完整规划器集成仍 IN_PROGRESS。
- M5-03 A*、M5-02 局部地图、M5-06 连续碰撞/围栏完整验证、原子 fallback、M5→M6 消费闭环：NOT_RUN。
- H723 WCET/RTA、静态 RAM/栈、PIL/HIL、真实障碍源、飞行安全与放行：NOT_RUN。
- 飞行状态仍为 `FLIGHT_RELEASE_NOT_AUTHORIZED`。

唯一验收状态台账为 `spec/09_需求验证实施与发布管理.md`；本报告仅记录实现边界和复现入口。


## 5. CI evidence and exact scope

The implementation baseline \`b9832c8ce1e8accb3c1e5591c02d4637c28e2d99\` completed GitHub Actions run [42589657670](https://github.com/hujunjie-hub/Flight/actions/runs/42589657670) successfully:

- M4 Host: 927 checks, 0 failures.
- M5 Host: 8,054 checks, 0 failures.
- M4/M5 integration: 1,000 checks, 0 failures.
- M5 Host AddressSanitizer/UndefinedBehaviorSanitizer: passed.
- M4/M5 integration AddressSanitizer/UndefinedBehaviorSanitizer: passed.
- Cortex-M7 production-source compile check and MPC RAM checks: passed.
- Source-layout, OSQP source-lock and time-service jobs: passed.

Artifacts:
- [M4/M5 Host regression evidence](https://github.com/hujunjie-hub/Flight/actions/runs/42589657670/artifacts/13660916940)
- [Cortex-M7 compile evidence](https://github.com/hujunjie-hub/Flight/actions/runs/42589657670/artifacts/13660637382)

Important scope distinction: this successful run contains 31 named M5 scenarios. The 32nd scenario, which adds nonzero start/end velocity and acceleration boundary checks, was added later in commit \`63f831d69718f1f77488440095b241dce3a670be\`; its subsequent CI run was still queued at the time this report was updated. Do not treat that later test as passed until its exact commit's CI completes.

The ARM job compiles the explicit M4/M5 production source list and checks the existing Cortex-M7/MPC build. It does not compile or execute the Host-only optimizer kernel on H723. No target link, H723 optimizer runtime, WCET/RTA, stack watermark, PIL/HIL or flight evidence is implied.
