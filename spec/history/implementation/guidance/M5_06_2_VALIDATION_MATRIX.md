# M5-06.2 / M5-06.4 验证矩阵与 Host 证据

状态：**本轮 Host/SIL 自动化回归通过；M5-06 整体仍未验收，不具备 H723 实机避障放行效力。**

## 本轮修复与验证结论

1. 修复测试夹具中的参考点偏移初始化问题：`safety_point_set()` 原先通过 `memset` 清除了调用方预先设置的 `offset_us`，导致 24 个预览点均为零偏移。现在保留偏移字段，再初始化点的有效位和运动数据。
2. 追加对抗性测试，覆盖 STALE/INVALID 栅格、动态障碍物中点碰撞和交叉碰撞、静态障碍物区间中段碰撞、KEEP_IN 连续越界、动态预测租约过期、重复 track ID、超限响应延迟、预览点数量/偏移错误以及 Inf 输入。
3. 移除仅用于定位问题的临时诊断输出；测试辅助函数不再引用生产 C 文件私有宏。
4. 先前 CI 失败均已定位：一次为测试夹具偏移清零，一次为临时诊断代码引用私有宏。最新提交已修复并完成完整回归。

## 最新 CI 证据

- Workflow：Host validation
- Run：[#358](https://github.com/hujunjie-hub/Flight/actions/runs/42619284762)
- Commit：`e026fe0c63acd494ab9d18caba9833e469288ee9`
- 结果：**全部 jobs success**

| CI 项目 | 实际结果 |
|---|---:|
| M4 Host | 927 checks / 0 failures |
| M5 Host | 8,357 checks / 0 failures |
| M4/M5 Integration | 1,000 checks / 0 failures |
| M5 ASan/UBSan | 8,357 checks / 0 failures |
| M4/M5 Integration ASan/UBSan | 1,000 checks / 0 failures |
| Cortex-M7 compile and MPC RAM checks | PASS |
| Navigation and control host tests | PASS |
| Source layout and time-service tests | PASS |

以上为本次 CI 实际执行结果，不代表真实飞行环境、传感器或 H723 实机验证。

## 自动化测试矩阵

| ID | 场景 | 期望 | 当前结果 |
|---|---|---|---|
| S06-001 | 全覆盖 FREE 地图、静止轨迹 | PASS | Host PASS |
| S06-002 | 预览点 NaN / Inf | INVALID | Host PASS |
| S06-003 | 预览点数量或 offset 不符合 24×20ms 契约 | INVALID | Host PASS：数量不足、偏移不等距 |
| S06-004 | reference 在 proof_end 前过期 | INVALID，拒绝 | Host PASS |
| S06-005 | map_revision 与 candidate 不一致 | INVALID | 待补 |
| S06-006 | swept volume 穿过 UNKNOWN voxel | UNKNOWN | Host PASS |
| S06-007 | swept volume 穿过 STALE voxel | STALE | Host PASS |
| S06-008 | swept volume 穿过 INVALID voxel 编码 | INVALID | Host PASS |
| S06-009 | 静态障碍物与机体膨胀区相交 | STATIC_COLLISION | Host PASS：占用栅格 |
| S06-010 | 预览区间中段穿过静态障碍，端点安全 | STATIC_COLLISION | Host PASS |
| S06-011 | 动态障碍物响应延迟膨胀 | DYNAMIC_COLLISION | Host PASS |
| S06-012 | 动态障碍物交叉轨迹碰撞 | DYNAMIC_COLLISION | Host PASS |
| S06-013 | 动态障碍最大加速度上界导致扫掠包络相交 | DYNAMIC_COLLISION / 未证明拒绝 | 待补独立边界用例 |
| S06-014 | track expires_us 早于 proof_end | STALE | Host PASS |
| S06-015 | track 重复 ID | INVALID | Host PASS；未来/过旧 sample 和非法数值组合待补 |
| S06-016 | KEEP_IN 连续轨迹越界，端点在围栏内 | GEOFENCE_VIOLATION | Host PASS |
| S06-017 | KEEP_OUT 连续穿越，端点安全 | GEOFENCE_VIOLATION | Host PASS |
| S06-018 | 围栏自交、重复顶点、非法类型和保留字段 | INVALID | 待补 |
| S06-019 | 导航 valid bit 缺失、epoch/origin/frame 不匹配 | INVALID | 待补 |
| S06-020 | response delay 超过配置上限 | INVALID | Host PASS |
| S06-021 | DFS 工作单元或体素检查预算耗尽 | BUDGET_EXCEEDED | 待补 |
| S06-022 | 工作栈容量边界和最大细分压力 | CAPACITY/BUDGET 拒绝 | 待补 |
| S06-023 | 工作栈固定 512B 编译期断言 | 编译通过 | Host/Cortex-M7 CI PASS |
| S06-024 | Host ASan/UBSan | 无内存或未定义行为错误 | CI PASS；fuzz 待补 |
| S06-025 | 动态障碍物中点碰撞 | DYNAMIC_COLLISION | Host PASS |
| S06-026 | 动态障碍物预测租约不足 | STALE | Host PASS |
| S06-027 | 多个动态 track 重复身份 | INVALID | Host PASS |
| S06-028 | 预览点数量不足 | INVALID | Host PASS |
| S06-029 | 预览点 offset 单点偏差 | INVALID | Host PASS |

## 仍未关闭的审查项

1. 独立复核 Bernstein 变换、浮点外向舍入和连续几何距离下界；当前不是形式化证明。
2. 补齐动态加速度不确定性边界、导航状态失效、围栏几何畸形、map revision 失配和预算耗尽等测试。
3. 对地图边界、障碍物擦边、动态轨迹切向相交和数值极值开展系统性测试与 fuzz。
4. M5-07 尚未完成，M5-06 PASS 尚未成为 M6 不可绕过的发布门禁。
5. H723 实机仍需真实传感器标定、WCET/RTA、RAM/栈、HIL 和逐级场地安全验证。

## 安全声明

- M5-05 输出始终只是候选轨迹。
- 只有 M5-06 返回 PASS 才能作为后续门禁的输入；本阶段尚未完成 M5-07 接线。
- UNKNOWN、STALE、INVALID 和任何无法证明安全的情况均不得放行。
- Host/SIL CI 全通过不等于 H723 已获得实机避障放行。
