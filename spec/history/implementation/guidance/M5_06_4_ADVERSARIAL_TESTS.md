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

## 新增几何差分、Bernstein 数值 oracle、多 track 与连续距离界 Fuzz（S06-064..078 已通过）

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

## 最新证据

- CI：[Host validation #433](https://github.com/hujunjie-hub/Flight/actions/runs/42718225825)
- Commit：`d06920f254611732fe971d5f60bfeb6431b8d6f6`
- M4 Host：927 / 0 failures
- M5 Host：261,285 / 0 failures
- M4/M5 Integration：1,000 / 0 failures
- M5 ASan/UBSan：261,285 / 0 failures
- M4/M5 Integration ASan/UBSan：1,000 / 0 failures
- Cortex-M7 compile/RAM and M4/M5 production-source compile：PASS
- MPC RAM：29,252 bytes；worker stack：8,192 bytes；largest single frame：2,024 bytes
- Source/time and Navigation/Control：PASS
- All workflow jobs：success
- Cortex-M7 compile and MPC RAM checks：PASS
- Source/time and Navigation/Control host jobs：PASS
- All workflow jobs：success

## 本轮新增测试结果

S06-074..078 静态/动态端到端扫掠差分、加速度包络、预览接缝与自适应细分用例均在 run `42718225825` 的 Host/ASan/UBSan 回归通过。S06-053..063 已在新 Head 执行通过：reference/map time_epoch、directive frame、动态 sample age/预测租约、地图预测时域、围栏高度边界/退化高度、uint64 proof horizon 溢出、有限极值计算溢出及固定 seed 32 组属性测试。首轮 fixture 缺陷已修正；以上结果仅代表 Host/SIL 与 Cortex-M7 编译检查，不是 H723 实机避障放行。

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
