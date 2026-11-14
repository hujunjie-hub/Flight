# 组合导航 M3

`ins_interface.c` 提供公共 `estimator_*` 入口。`kf_gins/navigation_estimator/` 管理导航生命周期和状态发布，`observation_replay/` 管理观测与回放，`backend_adapter/` 管理增量构造和私有 C 接口。
输入为 M2 的 FRD IMU 比力/角速度及 GNSS、Baro、Mag 标准测量；输出为带明确 State Time 的
NED NavigationSnapshot、Origin、Capability 和 Health。IMU 原始姿态矩阵/协方差不进入公共接口。

实现内容：

- 1 s 连续静止窗口的重力对准、陀螺零偏初始化、磁倾斜补偿航向、GNSS 初始位置/速度；不等待 GNSS/PPS。
- 无磁航向时保持大 yaw 不确定度；无 GNSS 时不虚构水平位置/速度能力。位置与速度 GNSS 质量独立门控。
- 实际采样间隔的 IMU 梯形增量、双子样机械编排和协方差传播。gap/序号异常后停止有效状态发布，需显式重置。
- GNSS 位置/速度、相对气压高度、质量门控磁航向的 NIS 更新；杆臂、CG 参考点与协方差变换。
- 固定 IMU 历史/checkpoint/去重日志、shadow 重放与 revision 提交检查；量测时间量化到最近 IMU 端点，
  保留原始时间并增加量化噪声。修正之后失效旧 checkpoint，安全回收过期日志。
- M4 显式静止重对准或有效的 provided-state 重置，以及 `ins_take_reset_event()` 单次事件读取。晚到首个GNSS通过`ins_get_pending_origin()`交给M4在地面授权重建原点，导航owner验证候选后执行；不偷偷更换活动原点。

调用顺序为 `estimator_init(config)` → 为已安装源设置 HEALTH 发布的 `SensorUsePolicy` →
逐样本 `estimator_push_imu()`/`estimator_push_measurement()` → `estimator_update(now)` → 查询快照。
所有导航调用归同一串行 owner，不能让 replay 和传播并发改写 active。快照保留原 State Time；
查询不会把旧状态续租成新测量。安装源不受允许策略、时间/标定/映射不可信时不接纳。

`estimator_update()` 支持Host同步回放与`ins_set_replay_budget()`选择的分片cursor；固件启用
400 us/5 ms记账及每次最多16个数值操作。不可分数值操作的目标WCET及server RTA仍未测量，
不能把该记账值当作目标硬时限证据。
`estimator_get_sensor_consistency()` 发布真实normal/shadow计算结果的有界摘要；隔离辅助源使用
独立历史先验probe，不进入active journal或提交active/P。证据按策略/状态版本、时刻和源序号去重，
映射变化/降级使其不可用。同epoch策略续租不续租证据。Health消费公开证据控制观察期与分级恢复，
入队/解码成功无法恢复源；单IMU没有独立冗余恢复证明。

`ins_sim_config()` 提供所选模型的合成测试配置。实机必须用已辨识的 estimator_tuning 与
sensor_alignment 构建配置；现有 YAML 中的 TBD 保持未解决。磁场范围/静止窗口/数值运动域
属于当前参考实现结构，仍需数据集与板测确认。

构建选项沿用 `INS_BACKEND_ENABLE` + `INS_REPLAY_ENABLE`（GNU Arm/C++14）。生命周期在
显式 init 中构造静态 arena，无 heap、异常、RTTI 或全局 C++ 构造依赖。运行验证命令见
`tests/algorithms/README.md`。

## KF-GINS 后端

`KF-GINS_QUAD` 是 QUAD_X4 的 M3 实现（05 §6.1）。算法核心、适配、来源锁和复现工具
归 [`kf_gins/`](kf_gins/)。估计器更换以 INS 模块为边界，内部同步选择后端和对应的观测、
回放实现，调用方通过公共 EstimatorPort 接入。

### 文件

| 文件 | 职责 |
|---|---|
| `kf_gins/backend_adapter/kf_gins_quad_port.h` | 插件私有 C ABI（不透明句柄、调用方提供静态存储、无 heap） |
| `kf_gins/backend_adapter/kf_gins_quad_port.cpp` | 生命周期、状态映射、单位与 frame 编组。`sizeof` 14112 B，`alignof` 8 B |
| `kf_gins/backend_adapter/kf_gins_adapter.hpp/.cpp` | 1 kHz IMU 增量构造，固定梯形积分，dt∈[950,1100] µs，相邻 dt 偏差 ≤1%，拒绝序号或时间倒退 |
| `kf_gins/trimmed_core/` | 受控 22 维裁剪核心（唯一进入固件的算法源） |
| `kf_gins/upstream/` | 锁定 commit 的原样源码，仅用于参考/差分，不参与编译 |
| `kf_gins/SConscript` | 按 `INS_BACKEND_ENABLE` 选择 C++14 后端源码 |
| `tests/estimator_backend/` | Host 单测及与上游原版的数值差分 |

### 单位与坐标

- 位置：大地纬度/经度 [rad]、椭球高 [m]；速度：局部 NED [m/s]
- 姿态：`q_nb_wxyz`（body FRD→nav NED）与 ZYX roll/pitch/yaw
- 陀螺/加速度计：body FRD，rad/s、m/s²；比力 f 未做重力补偿
- 杆臂：IMU 测量中心→GNSS 天线相位中心，body FRD [m]
- 气压：调用方给出 `h_baro - h_baro0`（相对压力高度），核心用
  `QuadConfig.baro_origin_height_m` 只做垂直基准定位；`h_baro0`/Home/Origin 归 M3/调用方

### EstimatorPort 边界

- 公共 `estimator_*()` 声明在 `middleware/contracts/ports/port_api.h`。
  本插件**不定义**任何公共 ICD 类型，也不实现 `estimator_*()`。
- 公共 Port 调用选定模型的生命周期、状态发布和观测回放实现。M2 负责原始测量标定、安装旋转和时间修正。
- `tests/estimator_backend/test_contract_coexistence.cpp` 检查两套头文件可同时编译、冻结值一致及命名冲突。

### 资源

| 项 | 实测 |
|---|---|
| 单实例（含 22×22 协方差与工作区） | 14,112 B（Host `sizeof(Handle)`；ARM `.bss` 14,104 B） |
| 无 heap / 无异常 / 无 RTTI | ARM `nm -u` 无 `malloc/free/operator new/__cxa_throw`（`build/validation/kf_gins_upstream/arm_objects.log`） |
| 活动 + shadow 两实例 | 28,208 B ≤ 07 号 28 KiB 目标，余量仅 464 B；REPLAY 任务须复核 |

### Host 验证

```powershell
pwsh tests/estimator_backend/run_host_tests.ps1 `
     -Upstream <锁定 commit 的 KF-GINS 检出目录>
```

| 测试 | 覆盖 | 结果（2026-11-04） |
|---|---|---|
| `test_jacobian` | GNSS-P/GNSS-V/Baro 解析 Jacobian 有限差分 | 220 checks / 0 fail |
| `test_core` | 22 维解耦、随机游走、NIS 拒绝与失败原子性、dt 守卫 | 2636 checks / 0 fail |
| `test_port` | C ABI 生命周期、空指针/未初始化句柄、增量规则 | 179 checks / 0 fail |
| `test_contract_coexistence` | 与公共契约共存 | PASS |
| `test_upstream_compare` | 21 维核心 vs 上游原版（400 历元/20 次 GNSS） | 426 checks / 0 fail |

日志默认位于 `build/validation/kf_gins_upstream/`。上游参考使用单独检出的锁定 commit
和其中的 Eigen。验收时显式传入 `-Upstream`。入口校验检出的 commit 和工作区清洁状态，
参考不完整、版本错误或有本地改动时返回失败。省略 `-Upstream` 且默认位置缺少参考时，
对照行标记 SKIPPED，其余本机用例照常运行。

### 2026-11-04 历史状态

- 任务 `T191-B-UPSTREAM` 仍为 `READY_FOR_ASSIGNMENT`，前置 `T191-D-REVIEW` 未执行；本目录内容为
  **独立准备**，不表示已接单/已审核/已通过，verification_status 仍为 `NOT_RUN`。
- 未执行：官方数据集复现、H723 链接/map/WCET、SIL/HIL、独立审查与 Gate C 会签。
