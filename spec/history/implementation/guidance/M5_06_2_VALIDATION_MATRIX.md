# M5-06.2 / M5-06.4 验证矩阵与 Host 证据

状态：**本轮 Host/SIL 自动化回归通过；M5-06 整体仍未验收，不具备 H723 实机避障放行效力。**

## 本轮修复与验证结论

1. 修复测试夹具中的参考点偏移初始化问题：`safety_point_set()` 原先通过 `memset` 清除了调用方预先设置的 `offset_us`，导致 24 个预览点均为零偏移。现在保留偏移字段，再初始化点的有效位和运动数据。
2. 追加对抗性测试，覆盖 STALE/INVALID 栅格、动态障碍物中点碰撞和交叉碰撞、静态障碍物区间中段碰撞、KEEP_IN 连续越界、动态预测租约过期、重复 track ID、超限响应延迟、预览点数量/偏移错误、Inf 输入、动态加速度上界、导航有效位缺失、地图 revision 不匹配和自交围栏。
3. 修正导航不确定度传播时域：静态障碍和围栏膨胀采用子区间远端时间，再叠加 response delay，避免只按区间中点估计不确定度。
4. 临时诊断输出已移除；测试辅助函数不引用生产 C 文件私有宏。

## 本轮新增属性/差分用例（S06-064..070 已通过新 Head CI）

| ID | 场景 | 期望 | 状态 |
|---|---|---|---|
| S06-064 | 多边形解析距离 oracle：L/C 凹多边形、CW/CCW、共线边、近共线边；随机点/顶点/边中点/边两侧，KEEP_IN/KEEP_OUT | 误差≤2e-5m | Host + ASan/UBSan PASS；run 42656744088 |
| S06-065 | 16 组 8 顶点交替半径凹多边形 + 动态 track 联合验证，并对每组随机点做 oracle 差分 | PASS / 误差≤2e-5m | Host + ASan/UBSan PASS；run 42656744088 |
| S06-066 | 32 组随机轨迹 + 动态 track，按时间/epoch/offset/数值字段单点破坏 | PASS invariant / fail-closed | Host + ASan/UBSan PASS；run 42656744088 |

| S06-067 | 相邻边共线回折（重叠边）与非相邻重复顶点 | INVALID | Host + ASan/UBSan PASS；run 42656744088 |

| S06-068 | 256 组随机端点 Hermite 曲线与非 dyadic 子区间；65 点严格检查 position hull、speed norm、acceleration norm；33 点比较 Bernstein eval 与独立 Hermite basis | 零容差边界包含全部参考样本；eval 误差在既定阈值内 | Host + ASan/UBSan PASS；run 42660981911 |
| S06-069 | duration、u0/u1、local_u 非法值 | TEST API 拒绝输入 | Host + ASan/UBSan PASS；run 42656744088 |
| S06-070 | 16 组 1..8 动态 tracks；未来 sample、重复身份、8-slot 容量及 1 个碰撞 track + 7 个安全 track | PASS / STALE / INVALID / DYNAMIC_COLLISION | Host + ASan/UBSan PASS；run 42656744088 |
| S06-071 | 2,048 组随机点到 AABB 距离下界，随机半径/rounding error；面、棱、角精确接触与非法输入 | 下界不超过独立 double oracle；接触距离为 0 | Host + ASan/UBSan PASS；run 42658820703 |
| S06-072 | 512 组动态相对运动/加速度/速度不确定度组合，每组 33 个时刻检查 sweep 上界 | 全采样相对位移≤sweep 上界；非法参数拒绝 | Host + ASan/UBSan PASS；run 42658820703 |
| S06-073 | 零位置/速度/加速度五次曲线，20ms 区间，检查 outward interval 控制点和边界 | position hull 覆盖精确零值，速度/加速度上界有效 | Host + ASan/UBSan PASS；run 42660981911 |
| S06-074 | 64 组静态 voxel 线性扫掠差分；32 组实际碰撞、32 组安全近失；独立点到 AABB 距离 oracle | 碰撞返回 STATIC_COLLISION；净空组返回 PASS | Host + ASan/UBSan PASS；run 42718225825 |
| S06-075 | 64 组动态障碍物恒速相对运动差分；32 组碰撞、32 组远离；解析最近接近时刻 oracle | 碰撞返回 DYNAMIC_COLLISION；安全组返回 PASS | Host + ASan/UBSan PASS；run 42718225825 |
| S06-076 | 64 组动态障碍物有界加速度差分；实际加速度≤0.45m/s²、批准测试上界0.5m/s²；每组257个时刻独立采样 | 32 组碰撞拒绝，32 组安全轨迹 PASS | Host + ASan/UBSan PASS；run 42718225825 |
| S06-077 | 静态占用 voxel 位于相邻 20ms 预览区间接缝；高速轨迹触发自适应细分 | STATIC_COLLISION；工作量统计包含细分工作单元 | Host + ASan/UBSan PASS；run 42718225825 |
| S06-078 | 动态障碍物分别在自适应细分内部及 20ms 预览接缝穿越车辆轨迹 | DYNAMIC_COLLISION；不得因区间分段漏检 | Host + ASan/UBSan PASS；run 42718225825 |

## 最新 CI 证据

- Workflow：Host validation
- Run：[Host validation #433](https://github.com/hujunjie-hub/Flight/actions/runs/42718225825)
- Commit：`d06920f254611732fe971d5f60bfeb6431b8d6f6`
- 结果：**全部 jobs success**

| CI 项目 | 实际结果 |
|---|---:|
| M4 Host | 927 checks / 0 failures |
| M5 Host | 261,285 checks / 0 failures |
| M4/M5 Integration | 1,000 checks / 0 failures |
| M5 ASan/UBSan | 261,285 checks / 0 failures |
| M4/M5 Integration ASan/UBSan | 1,000 checks / 0 failures |
| Cortex-M7 compile and MPC RAM checks | PASS |
| M4/M5 production-source Cortex-M7 compile | PASS |
| Navigation and control host tests | PASS |
| Source layout and time-service tests | PASS |

ARM 静态检查报告：MPC RAM 29,252 bytes、worker stack 8,192 bytes、最大单函数栈帧 2,024 bytes。该结果不代表 H723 实机 WCET、实际栈水位或飞行放行。

以上为 CI 实际执行结果，不代表真实飞行环境、传感器或 H723 实机验证。

## 自动化测试矩阵

| ID | 场景 | 期望 | 当前结果 |
|---|---|---|---|
| S06-001 | 全覆盖 FREE 地图、静止轨迹 | PASS | Host PASS |
| S06-002 | 预览点 NaN / Inf | INVALID | Host PASS |
| S06-003 | 预览点数量或 offset 不符合 24×20ms 契约 | INVALID | Host PASS：数量不足、偏移不等距 |
| S06-004 | reference 在 proof_end 前过期 | INVALID，拒绝 | Host PASS |
| S06-005 | map_revision 与 candidate 不一致 | INVALID | Host PASS |
| S06-006 | swept volume 穿过 UNKNOWN voxel | UNKNOWN | Host PASS |
| S06-007 | swept volume 穿过 STALE voxel | STALE | Host PASS |
| S06-008 | swept volume 穿过 INVALID voxel 编码 | INVALID | Host PASS |
| S06-009 | 静态障碍物与机体膨胀区相交 | STATIC_COLLISION | Host PASS：占用栅格 |
| S06-010 | 预览区间中段穿过静态障碍，端点安全 | STATIC_COLLISION | Host PASS |
| S06-011 | 动态障碍物响应延迟膨胀 | DYNAMIC_COLLISION | Host PASS |
| S06-012 | 动态障碍物交叉轨迹碰撞 | DYNAMIC_COLLISION | Host PASS |
| S06-013 | 动态障碍最大加速度上界导致扫掠包络相交 | DYNAMIC_COLLISION / 未证明拒绝 | Host PASS：低上界通过、高上界拒绝 |
| S06-014 | track expires_us 早于 proof_end | STALE | Host PASS |
| S06-015 | track 重复 ID | INVALID | Host PASS |
| S06-032 | track flags/source metadata 非法 | STALE | Host PASS：fail-closed |
| S06-033 | 存在动态 track 但缺少正的加速度上界 | INVALID | Host PASS：fail-closed |
| S06-034 | 导航 state_epoch 与参考/地图不匹配 | INVALID | Host PASS |
| S06-035 | 导航 origin_id 与参考/地图不匹配 | INVALID | Host PASS |
| S06-036 | 导航消息 frame_id 非 LOCAL_NED | INVALID | Host PASS |
| S06-037 | 围栏 origin_id 与导航不匹配 | INVALID | Host PASS |
| S06-038 | 围栏存在重复相邻顶点/退化边 | INVALID | Host PASS |
| S06-039 | 动态 track sample 时间在未来 | STALE | Host PASS |
| S06-040 | 动态 track sample 超过陈旧阈值 | STALE | Host PASS |
| S06-041 | 动态 track 位置含 NaN | STALE | Host PASS |
| S06-042 | 动态 track 半径为负值 | STALE | Host PASS |
| S06-016 | KEEP_IN 连续轨迹越界，端点在围栏内 | GEOFENCE_VIOLATION | Host PASS |
| S06-017 | KEEP_OUT 连续穿越，端点安全 | GEOFENCE_VIOLATION | Host PASS |
| S06-018 | 围栏自交几何 | INVALID | Host PASS；S06-067 补充非相邻重复顶点、相邻共线回折拒绝 |
| S06-019 | 导航缺少所需 valid bit、state/time epoch/origin/frame；directive frame 失配 | INVALID | Host PASS：S06-019、034-036、053-055；run 42650028230 |
| S06-020 | response delay 超过配置上限 | INVALID | Host PASS |
| S06-021 | 静态地图 cell-check 预算超过 8192 | BUDGET_EXCEEDED | Host PASS：8193 次时 fail-closed |
| S06-022 | DFS work-unit 预算 256 耗尽 | BUDGET_EXCEEDED | Host PASS：第 257 个工作单元触发 fail-closed，cell-check 预算未耗尽 |
| S06-023 | 工作栈容量边界和最大细分压力 | CAPACITY/BUDGET 拒绝 | 明确断言 29 项容量、深度5、workspace≤512B、canary；run 42650028230 通过 |
| S06-024 | 工作栈固定 512B 编译期断言 | 编译通过 | Host/Cortex-M7 CI PASS |
| S06-025 | Host ASan/UBSan | 无内存或未定义行为错误 | M5 8,837 / 0；M4/M5 integration 1,000 / 0；run 42650028230 |
| S06-026 | 动态障碍物中点碰撞 | DYNAMIC_COLLISION | Host PASS |
| S06-027 | 动态障碍物预测租约不足 | STALE | Host PASS |
| S06-028 | 多个动态 track 重复身份 | INVALID | Host PASS |
| S06-029 | 预览点数量不足 | INVALID | Host PASS |
| S06-030 | 预览点 offset 单点偏差 | INVALID | Host PASS |
| S06-031 | 导航不确定度传播至子区间远端 | 保守膨胀 | 代码修正并通过完整 CI；专项边界用例待补 |

## 本轮新增测试源（CI 已通过）

| 新用例 | 场景 | 预期 | 状态 |
|---|---|---|---|
| S06-053 | MotionReference header time_epoch 失配 | INVALID | Host PASS；run 42650028230 |
| S06-054 | LocalMap time_epoch 失配 | INVALID | Host PASS；run 42650028230 |
| S06-055 | Directive header frame_id 失配 | INVALID | Host PASS；run 42650028230 |

## 本轮新增边界用例（修正 fixture 后 CI 已通过）

| ID | 场景 | 期望 | 当前结果 |
|---|---|---|---|
| S06-056 | 动态 track sample age 恰为 500ms / 超 1us | PASS / STALE | Host PASS；run 42650028230 |
| S06-057 | 动态预测租约恰为 1s / 超 1us | PASS / STALE | Host PASS；run 42650028230 |
| S06-058 | 地图预测能力低于 1s 最低要求 | INVALID | Host PASS；run 42650028230 |
| S06-059 | 围栏最小/最大高度，计入垂直 margin 与机体/净空膨胀 | 安全内侧 PASS、边界外拒绝 | Host PASS；高度场景同步滚动地图中心；run 42650028230 |
| S06-060 | 围栏零高度区间 | INVALID | Host PASS；run 42650028230 |
| S06-061 | 24×20ms proof horizon 的 uint64 加法溢出 | INVALID | Host PASS；run 42650028230 |
| S06-062 | 动态障碍物有限但极大位置/速度不确定度组合导致计算界溢出 | NUMERIC_FAILURE | Host PASS；run 42650028230 |
| S06-063 | 固定种子 32 组有效静态轨迹（位置保持在 map readiness 半径内）与 offset 单点破坏 | PASS invariant / INVALID | Host PASS；32 组有效/无效属性对；run 42650028230 |

## 最新 CI 证据

- Run: [Host validation #425](https://github.com/hujunjie-hub/Flight/actions/runs/42660981911)
- Commit: `3cc58e8683d504b3f12615406d8713274054dbad`
- M4 Host: 927 checks / 0 failures
- M5 Host: 260,119 checks / 0 failures
- M4/M5 Integration: 1,000 checks / 0 failures
- M5 ASan/UBSan: 260,119 checks / 0 failures
- M4/M5 Integration ASan/UBSan: 1,000 checks / 0 failures
- Source/time, Navigation/Control, Cortex-M7 compile and M4/M5 production-source compile: PASS
- All workflow jobs: success

## 仍未关闭的审查项

1. 独立复核 Bernstein 变换、浮点外向舍入和连续几何距离下界；当前不是形式化证明。
2. 已增加静态 voxel 线性扫掠、动态恒速/有界加速度相对运动的端到端差分，以及静态/动态区间接缝和自适应细分碰撞用例。下一步仍需扩展至任意五次 Hermite 曲线的独立连续距离 oracle、精确擦边/切触边界和最小反例归档；当前有限采样 oracle 不等同形式化证明。
3. 对 interval arithmetic 的 IEEE-754/编译器舍入假设进行独立审查，并为区间运算增加可复现边界与溢出用例。
3. 对动态障碍物加速度上界进行更多目标类别/传感器标定和参数敏感性分析。
4. M5-07 尚未完成，M5-06 PASS 尚未成为 M6 不可绕过的发布门禁。
5. H723 实机仍需真实传感器标定、WCET/RTA、RAM/栈、HIL 和逐级场地安全验证。

## 安全声明

- M5-05 输出始终只是候选轨迹。
- 只有 M5-06 返回 PASS 才能作为后续门禁的输入；本阶段尚未完成 M5-07 接线。
- UNKNOWN、STALE、INVALID 和任何无法证明安全的情况均不得放行。
- Host/SIL CI 全通过不等于 H723 已获得实机避障放行。
