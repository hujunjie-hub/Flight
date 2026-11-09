# M5-06.4 对抗性安全验证与边界测试

状态：**本轮 Host/SIL 对抗性测试通过；M5-06 整体仍未验收。**

## 目的

验证独立安全验证器不会因离散预览点安全而漏掉连续区间碰撞，并确认输入不完整、状态过期、几何非法和动态预测不确定时均 fail-closed。

## 本轮新增覆盖

| 测试 | 构造 | 预期 | 实际 |
|---|---|---|---|
| 静态中段碰撞 | 24 区间连续轨迹中段穿过占用 voxel | STATIC_COLLISION | PASS |
| 动态中点碰撞 | 动态障碍物位于车辆轨迹中点 | DYNAMIC_COLLISION | PASS |
| 动态交叉碰撞 | 车辆与障碍物以交叉速度穿越同一位置 | DYNAMIC_COLLISION | PASS |
| KEEP_IN 连续越界 | 预览端点位于围栏内，曲线中段越界 | GEOFENCE_VIOLATION | PASS |
| STALE voxel | 轨迹膨胀区覆盖 STALE cell | STALE | PASS |
| INVALID voxel | 栅格状态编码非法 | INVALID | PASS |
| 动态租约不足 | track expires_us 早于 proof_end | STALE | PASS |
| 重复 track | 同一 track 身份重复出现 | INVALID | PASS |
| 动态加速度上界 | 较低加速度界可通过，批准上界导致预测包络碰撞 | PASS / DYNAMIC_COLLISION | PASS |
| 导航有效位缺失 | 清除必需导航 valid bit | INVALID | PASS |
| 地图 revision 失配 | candidate revision 与当前 map 不同 | INVALID | PASS |
| 自交围栏 | bow-tie polygon | INVALID | PASS |
| 预览点 offset 错误 | 单点 offset 偏离 20ms 均匀采样契约 | INVALID | PASS |
| Inf 输入 | 预览点速度含 Inf | INVALID | PASS |

## 测试夹具问题及修复

测试回归曾统一返回 REFERENCE_INVALID。根因不是验证器接受了错误轨迹，而是测试辅助函数 `safety_point_set()` 对整个 MotionReferencePoint 执行 `memset`，把调用方提前设置的 `offset_us` 清零。

修复方式：保留原 offset，再清零并设置运动数据与有效位。修复后 24 个点恢复为严格的 20ms 间隔，所有新增安全用例通过。

## 算法保守性修正

导航不确定度随预测时间传播。此前静态障碍和围栏膨胀使用子区间中点时域，未覆盖区间远端。现将不确定度时域改为：

`horizon_mid + subinterval_half_duration + response_delay`

使膨胀覆盖整个子区间最远时刻及响应延迟。动态 track 预测原本已包含子区间半宽。

## 最新证据

- CI：[最新 Host validation](https://github.com/hujunjie-hub/Flight/actions/runs/42630463108)
- Commit：`ffc597b12c825c0897f263502b754d7eb1c8a267`
- M4 Host：927 / 0 failures
- M5 Host：8,371 / 0 failures
- M4/M5 Integration：1,000 / 0 failures
- M5 ASan/UBSan：8,371 / 0 failures
- M4/M5 Integration ASan/UBSan：1,000 / 0 failures
- Cortex-M7 compile and MPC RAM checks：PASS
- 全部 workflow jobs：success

## 下一批必须补齐

1. DFS work-unit 预算 256 耗尽与工作栈容量压力用例（当前已覆盖 cell-check 预算 8192，二者是独立计数器）。

1. 导航 epoch、origin、frame 不匹配组合。
2. 围栏重复顶点、退化边、边界擦边和垂直高度边界。
3. 地图外边界、障碍物膨胀临界值和 cell 状态组合。
4. 动态 track 过旧/未来 sample、非有限位置/速度、零/负半径和最大预测时域边界。
5. DFS 深度、工作单元、体素访问预算耗尽与最大工作栈压力。
6. 随机/属性 fuzz，并对关键几何边界建立可复现最小反例。

## 放行约束

- 本文的 PASS 仅代表当前 Host/SIL 自动化测试通过。
- M5-05 仍只输出 candidate。
- M5-07 尚未把 M5-06 PASS 绑定为 FMS/Permit/M6 的不可绕过发布门禁。
- 不得将 Host/SIL 结果解释为 H723 实机避障放行；真实传感器标定、WCET/RTA、RAM/栈、HIL 和场地验证仍未完成。
