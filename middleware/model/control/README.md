# QUAD 控制器 M6

实现 `controller_validate/init/update/get_health/stop` 与 QUAD 双积分器 `ControlModelPort`。
公共入口是 `control_interface.c/.h`。内环模型分为 [反馈](inner_loop/quad_x4/attitude_rate_control/)、[分配](inner_loop/quad_x4/control_allocation/)和[配置](inner_loop/quad_x4/simulation_config/)。外环模型分为 [跟踪](outer_loop/quad_x4/position_tracking/)、[预测](outer_loop/quad_x4/prediction_model/)和[求解](outer_loop/quad_x4/mpc_solver/)。

控制链为：50 Hz LTI-MPC → 当前加速度/jerk/倾角/推力复核 → 推力向量与期望姿态 →
SO(3) 主值 Log → 测量微分低通的 Rate PID → X4 物理推力分配。手动 RATE、ATTITUDE、
ALT_HOLD 分支复用角速度环与分配器；ALT_HOLD 不要求水平位置。无有效 MPC 时立即使用受限 PD。

分配器按横滚/俯仰、总推力、偏航的顺序取可行解，硬性限制每电机上下界、变化率和总推力；
输出为 N。分配估计残差用于抗积分饱和，匹配的 M7 Receipt 仅作为附加命令级证据。
不会把分配估计值描述成实测推力，也不实现 DShot/PWM。

`controller_update()` 是快环入口。独立的 50 Hz worker 通过 `control_outer_update()` 调用 OSQP。
输入在调用期间不可变。worker 与快环通过完整快照和非阻塞短锁交换数据，
快环不等待求解。初始化和停止必须与两条执行路径串行化。MPC 用 TimeService 检查 2 ms
求解预算、输入种子年龄与完整上下文，过期/失败结果不能续租。

每次输出要求匹配的 Navigation、GuidanceStatus、Reference、Directive、Authority、Permit、
SafetyEnvelope、ActuatorStatus/Capability；检查 boot/time/state/origin/reference/authority/directive 等
epoch、NED/FRD、时间质量、有限值和 TTL。失败返回明确 Status，Demand 的 valid mask 为零。
同一 State Time 不得重复控制；状态不在当前模型边界时拒绝输出。

`control_sim_config()` 提供 SIM_ONLY 参数，源于 `config/vehicle_model.yaml` 和
`controller_tuning.yaml`。后者的 `inner_loop` 放姿态增益、角速度 PID 和滤波参数，
`outer_loop` 放位置及高度回退增益、MPC 结构和权重、预瞄和种子容差。
参数变更后用生成器刷新夹具。

```powershell
python middleware/model/control/inner_loop/quad_x4/simulation_config/generate_sim_config.py
```

目标侧选项为 `CONTROLLER_ENABLE`，默认关闭，要求 GNU Arm。本模块没有创建实机线程，
也没有自动解锁或驱动电机。M3 replay 的分片调度、整机调度/RTA/WCET、实际机体/ESC 参数、
SIL 动态闭环和 HIL 仍需系统集成与硬件证据；不能把本目录的静态编译、合成输入测试当作飞行放行。
