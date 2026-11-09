# M5-06.4 对抗性安全验证与边界测试

状态：**本轮 Host/SIL 对抗性测试通过；M5-06 整体仍未验收。**

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

## 最新证据

- CI：[最新 Host validation](https://github.com/hujunjie-hub/Flight/actions/runs/42638931948)
- Commit：`c35e6497e483ff2c0925612786d4ca8eb6acb851`
- M4 Host：927 / 0 failures
- M5 Host：8,458 / 0 failures
- M4/M5 Integration：1,000 / 0 failures
- M5 ASan/UBSan：8,458 / 0 failures
- M4/M5 Integration ASan/UBSan：1,000 / 0 failures
- Cortex-M7 compile and MPC RAM checks：PASS
- 全部 workflow jobs：success

## 本轮追加测试源（新 Head CI 待验证）

新增 MotionReference time_epoch、LocalMap time_epoch、Directive frame_id 三个失配用例，均要求 INVALID 且 passed=0。测试源码已登记；新 Head 的 Host、ASan/UBSan 结果尚未取得。

## 下一批必须补齐

1. 工作栈容量压力与最大细分树边界用例（DFS work-unit 256 和 cell-check 8192 两个预算均已覆盖）。

1. 导航及围栏 frame/origin/state epoch 交叉组合；当前已覆盖导航三项独立失配与围栏 origin 失配。
2. 围栏重复顶点、退化边、边界擦边和垂直高度边界。
3. 地图外边界、障碍物膨胀临界值和 cell 状态组合。
4. 动态 track 非有限速度、零半径、最大预测时域边界及时间戳整数极值。
5. DFS 深度与最大工作栈压力；两个独立预算（work-unit、cell-check）均已由可达用例验证。
6. 随机/属性 fuzz，并对关键几何边界建立可复现最小反例。

## 本轮追加用例（新 Head CI 待验证）

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
