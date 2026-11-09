# M5-06 独立轨迹安全验证器（实现切片）

状态：**Host/SIL 自动化回归通过；整体实现进行中；尚未验收；不具备 H723 实机避障放行效力。**

## 边界

- 输入为 M6 实际消费的 MotionReference anchor + 24 个预览点；每个 20ms 区间按消费者相同的五次 Hermite 边界重建。
- M5-05 优化结果和 fallback 都只是候选。此验证器只输出独立验证结果，不签发 ActuationPermit、不直接发布 M6 参考。
- Host/SIL 专用，FLIGHT_M5_HOST_PLANNER 宏保护；H723/NONE 不编译、不分配此模块，也不因此获得避障能力。
- 需要 M5-07 才能把“候选 → M5-06 PASS → FMS/Permit/epoch 绑定 → M6”做成不可绕过的发布门禁。

## 当前实现

- 固定 24 个根区间；固定 DFS 工作栈（29 项）、最大深度 5、最多 256 个区间工作单元、8192 次栅格检查。
- 对每个子区间重建 Bernstein 控制点，使用凸包得到位置、速度、加速度界；按扫掠界无法证明时二分细分，达到上限拒绝。
- 静态局部地图检查机体半径、净空、导航不确定度和反应延迟；地图外、UNKNOWN、STALE、INVALID 不作为 FREE。
- 动态 track 按同一 FCU monotonic 时间轴做相对运动预测，计入目标半径、位置/速度不确定度、最大动态加速度上界和反应延迟。track 的预测租约必须覆盖完整验证窗口。
- 检查导航、参考、指令、安全包络、地图、Home 和围栏的消息类型、有效位、frame、boot/time/state/origin epoch、TTL 和数值范围。
- 只接受 GUIDANCE_REFERENCE_TRAJECTORY；Pilot/Altitude-only 参考不冒充 3D 避障轨迹。
- 失败原因独立编码；所有非 PASS 状态都必须拒绝。
- Host 测试覆盖连续静态碰撞、动态中点/交叉碰撞、UNKNOWN/STALE/INVALID、KEEP_IN/KEEP_OUT 穿越、动态租约过期、重复 track、输入数值错误和响应延迟上限。

## 最新 Host/SIL 证据

- GitHub Actions：[Host validation Run #358](https://github.com/hujunjie-hub/Flight/actions/runs/42619284762)
- Commit：`e026fe0c63acd494ab9d18caba9833e469288ee9`
- M4 Host：927 checks / 0 failures
- M5 Host：8,357 checks / 0 failures
- M4/M5 Integration：1,000 checks / 0 failures
- M5 ASan/UBSan：8,357 checks / 0 failures
- M4/M5 Integration ASan/UBSan：1,000 checks / 0 failures
- Cortex-M7 编译及 MPC RAM 检查：PASS
- 导航/控制、源码布局和时间服务回归：PASS

本轮还修复了测试夹具中 `safety_point_set()` 清零 `offset_us` 的问题。该问题导致测试轨迹不满足 24×20ms 契约，并使安全验证器正确拒绝输入；修复后所有当前纳入 CI 的 M5 安全测试通过。

## 产品参数要求

max_dynamic_obstacle_accel_mps2 是独立输入，必须由产品安全配置批准，不能从单次障碍观测猜测。当前代码对该值设置有限数值域；有动态 track 时要求大于零。该上界尚未完成真实传感器/目标类别标定，不得用于实机放行。

response_delay_us 必须覆盖感知数据年龄、验证/调度延迟和控制/执行器响应延迟的批准上界。当前接口限制在 200ms 内，超过即拒绝；这不是实测响应时间。

## 未完成项

1. 补齐动态加速度边界、导航状态有效位/epoch、围栏畸形几何、map revision 失配和预算耗尽测试。
2. 复核地图边界、擦边碰撞、切向动态交会和数值极值；开展边界 fuzz。
3. 独立复核 Bernstein 变换、浮点舍入误差、几何距离下界和所有连续性证明；当前不能宣称形式化证明。
4. M5-07 接入 M5-05 candidate、M5-06 result、FMS Permit 和 M6 消费端，保证候选无法绕过验证器。
5. H723 实机仍需真实障碍传感器、标定、WCET/RTA、RAM/栈、HIL 和逐级安全测试。

## 本切片的结论

当前 Host/SIL 回归已取得可复现 CI 证据，但 M5-06 尚未完成独立安全审查与 M5-07 门禁接线。不得合并到 main，不得接入可执行飞行链路，不得据此宣称 H723 实机避障能力已放行。
