# M5-06.2 验证矩阵与当前审查记录

状态：**已补齐第一批 Host 单测源码；尚未编译、尚未执行、不得判定 PASS。**

## 本轮代码审查发现并修复

1. 多类安全检查共享 unresolved 标志，静态地图检查会清除此前的围栏未决状态。已改为跨检查累积，并按 INVALID > STALE > UNKNOWN > 静态/动态碰撞 > 围栏未决的顺序保留最严格结果。
2. 移除未使用的函数/常量，避免 Host 的 -Wall -Wextra -Werror 构建被无用静态符号阻断。
3. 增加输入保留字段、地图保留字段、围栏 fence_id/保留字段和操作枚举检查。
4. 结果有效期纳入导航、Home 的租约；并增加参考、directive、safety envelope 和 directive expiry 必须覆盖完整 24×20ms 预览 + response delay 的校验，不能只在验证时刻有效。

## 自动化测试矩阵（源码已覆盖部分场景；全部仍为 NOT_RUN）

| ID | 场景 | 期望 | 源码状态 |
|---|---|---|---|
| S06-001 | 合法、全覆盖 FREE 地图、静止轨迹 | PASS | 已覆盖：m5_safety_static_free_pass |
| S06-002 | 24 个区间任意一个点数值 NaN/Inf | INVALID | 已覆盖 NaN：m5_safety_nan_reject；Inf 待补 |
| S06-003 | point_count 少于/多于 24、offset 不等距 | INVALID | 待补 |
| S06-004 | reference/header/directive/envelope 任一在 proof_end 前过期 | STALE/INVALID，拒绝 | 已覆盖 reference TTL：m5_safety_reference_ttl；其他待补 |
| S06-005 | map_revision 与 candidate 不一致 | INVALID | 待补 |
| S06-006 | swept volume 穿过 UNKNOWN voxel | UNKNOWN | 已覆盖：m5_safety_unknown_fail_closed |
| S06-007 | swept volume 穿过 STALE voxel | STALE | 待补 |
| S06-008 | swept volume 穿过 INVALID voxel | INVALID | 待补 |
| S06-009 | 静态障碍在 vehicle radius + clearance 外侧/内侧临界值 | 外侧可证明才 PASS；内侧拒绝 | 部分覆盖：m5_safety_occupied_static_voxel；临界值待补 |
| S06-010 | 24 个区间中间穿过静态障碍、端点均安全 | STATIC_COLLISION | 待补 |
| S06-011 | 动态障碍迎面相撞 | DYNAMIC_COLLISION | 部分覆盖：m5_safety_dynamic_delay；直接碰撞待补 |
| S06-012 | 动态障碍交叉路径相撞 | DYNAMIC_COLLISION | 待补 |
| S06-013 | 动态障碍最大加速度上界导致扫掠包络相交 | DYNAMIC_COLLISION/未证明拒绝 | 待补 |
| S06-014 | 动态 track expires_us 早于 proof_end | STALE | 待补 |
| S06-015 | track sample 超前、过旧、重复 ID 或数值非法 | STALE/INVALID | 待补 |
| S06-016 | keep-in 围栏内部穿越边界、端点仍在内部 | GEOFENCE_VIOLATION | 待补 |
| S06-017 | keep-out 围栏穿越、擦边、垂直上下限越界 | GEOFENCE_VIOLATION | 已覆盖穿越：m5_safety_keepout_crossing；擦边/高度待补 |
| S06-018 | 围栏顶点重复、自交、无效类型/保留字段 | INVALID | 待补 |
| S06-019 | 导航缺少任一所需 valid bit 或 epoch/origin/frame 不匹配 | INVALID | 待补 |
| S06-020 | response delay 超过配置上限 | INVALID | 待补 |
| S06-021 | DFS 深度/工作单元/体素检查预算耗尽 | BUDGET_EXCEEDED | 待补 |
| S06-022 | 工作栈容量边界及所有区间都需细分 | CAPACITY_EXCEEDED 或 BUDGET_EXCEEDED | 待补 |
| S06-023 | 编译期断言验证工作栈不超过 512B | 编译通过 | 待执行 |
| S06-024 | Host ASan/UBSan、边界 fuzz | 无内存/未定义行为错误 | 待执行 |

## 尚未关闭的审查项

- 独立复核 Bernstein 变换、浮点外向舍入和几何距离下界；当前不能宣称形式化证明。
- 核对动态 track 与静态 occupancy 的重复表示及其误拒绝率。
- 验证 keep-in/keep-out 几何符号和边界容差的完整组合。
- 第一批 Host 单测已加入 `tests/guidance/test_m5_main.c`：全 FREE 地图 PASS、UNKNOWN 拒绝、静态占用拒绝、动态响应延迟膨胀、NaN 预览拒绝、参考租约不足拒绝、KEEP_OUT 连续穿越拒绝。另增加地图 revision 和 TTL 检查仍待覆盖。
- 真实编译和运行测试。当前 GitHub Actions 没有发现该 PR commit 对应的 workflow run；不能据此宣称 CI 通过。
- M5-07 仍未接线，任何 PASS 结果都不能单独进入 M6。

## 安全声明

所有 S06 测试在取得可复现日志前均为 NOT_RUN。Host/SIL 测试不能替代 H723 的传感器标定、WCET/RTA、内存/栈分析、HIL、场地试验和安全放行。
