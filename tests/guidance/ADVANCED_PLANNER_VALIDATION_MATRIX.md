# M5 Advanced Planner 独立验证矩阵（V1.9.1）

> 类型：独立验证设计与证据模板；不是算法实现，也不是飞行放行声明。  
> 基线：spec/04、spec/07、spec/09 的 V1.9.1；适用产品 QUAD_X4 / STM32H723。  
> 唯一验收状态台账仍为 spec/09。本文件只细化测试输入、预期、失败判据和证据要求。

## 1. 当前实现边界

当前固件基线为 H723、OBSTACLE_SOURCE_MODE=NONE、BENCH_INHIBIT。必须区分以下两条验证路径：

- **H723 / NONE：** 使用有界五次轨迹和围栏验证；不分配局部地图或 A* arena。任务若要求障碍避让保证，必须拒绝并报告障碍保证不可用。不得生成全 FREE 地图，也不得把障碍几何检查记为 PASS。
- **Host / SIL：** 已增加独立 minimum-jerk 数学内核；完整地图、时空 Kinodynamic A*、动态障碍验证及 M5-06 集成仍待实现。Host 合成数据只能证明算法在模型假设下的行为，不能证明 H723 实时性或真实传感器能力。
- **ONBOARD_SENSOR：** 当前不支持；没有用户批准的传感器、安装、时间/坐标和质量证据前，不得开启在线避障。
- **Companion：** 只能作为可选地图/障碍/路径建议来源；不能成为基础任务依赖，也不能直通 M6/M7。

当前代码已覆盖基础参考生成、输入授权/时效、Epoch、连续围栏、Bernstein 动力学边界和 NONE 模式避障任务拒绝。当前生产切片尚不包含完整 A*/局部地图/动态障碍管线，也没有完整 M4→M5→M6→M7→Plant 运行期闭环。对应测试不得因接口或文档存在而标记通过。

当前 tests/guidance/test_m5_main.c 登记 33 个 Host 场景。其实际执行结果以对应 commit 的 CI 日志为准；历史交付报告不能替代当前分支复跑。

## 2. 状态与证据规则

验证状态只使用 spec/09 规定的 NOT_RUN、IN_PROGRESS、PASS、FAIL、WAIVED。

- NOT_IMPLEMENTED 是实现状态说明，不是验证通过状态。
- PASS 必须同时具备：基线 commit、Profile/参数 hash、实际命令、退出码、原始日志、测试数据/输入 hash 和适用范围。
- Host、SIL、PIL、HIL、目标板实测、飞行验证分别记录；不得互相替代。
- 任何 UNKNOWN、过期、超预算、无法证明安全的结果都按拒绝处理；不能把“没有检测到碰撞”写成“证明无碰撞”。
- 任何部分路径、优化中间结果或过期结果均不得发布给 M6。

## 3. 基础轨迹与生命周期矩阵（H723/NONE + Host）

| ID | 输入/场景 | 预期结果 | 失败判据 | 自动化与证据 | 当前状态 |
|---|---|---|---|---|---|
| M5-BAS-001 | 点到点、返回目标、有效任务快照 | 参考 anchor 和未来点有效，frame、origin、state_epoch 一致 | 目标/状态不一致仍发布 | Host 单测；tests/guidance/evidence/ | NOT_RUN |
| M5-BAS-002 | 多航点切项、mission_epoch / execution_token 改变 | 只执行当前授权项；旧 token 不能推进或复活 | 旧任务项继续生成参考 | M4/M5 integration；build/ci/m4-m5/ | NOT_RUN |
| M5-BAS-003 | 起点/终点位置、速度、加速度边界向量 | 五次多项式满足端点 p/v/a 条件 | 边界误差超容差、非有限系数 | Host golden vectors；build/ci/m5/ | NOT_RUN |
| M5-BAS-004 | T 初值、速度/加速度/jerk 限值变化 | 按 V1.9.1 有界时标规则选 T，0.2s 向上取整，最多 16 次且总时长不超过 6.2s | 无限延长、越过尝试上限仍发布 | Host property test；build/ci/m5/ | NOT_RUN |
| M5-BAS-005 | 端点与区间内 v/a/jerk 极值 | Bernstein 凸包界覆盖解析/高密度数值极值 | 任何实际极值落在已声明界外 | Host property/golden test；build/ci/m5/ | NOT_RUN |
| M5-BAS-006 | 长航段、不可行终端状态、超过时长上限 | 返回明确不可行/超限结果，不截断成成功 | 超长任务被静默缩短或输出部分路径 | Host boundary test；build/ci/m5/ | NOT_RUN |
| M5-BAS-007 | 连续滚动 24 点预览 | 新参考 anchor 与上一帧对应预览点的 p/v/a 连续 | 滚动时跳变或 reference_epoch 无故变化 | Host repeated-step test；build/ci/m5/ | NOT_RUN |
| M5-BAS-008 | state_epoch、origin_id、directive_epoch、permit/token 改变或过期 | 旧参考失效，按新授权上下文重建或拒绝 | 旧 epoch 参考继续 ACTIVE | Host fault injection；build/ci/m5/ | NOT_RUN |
| M5-BAS-009 | NONE 配置下提交 requires_obstacle_avoidance=true 的任务 | M4/M5 fail-closed 拒绝；遥测明确障碍保证不可用 | 任务继续、伪造全 FREE 地图或报告碰撞检查 PASS | Host negative test；build/ci/m5/ | NOT_RUN |
| M5-BAS-010 | Keep-in/Keep-out 围栏边界、退化/自交多边形、TTL/epoch 错配 | 整段检查；无效围栏拒绝依赖该围栏的任务 | 只检查航点、忽略边界穿越或过期围栏 | Host geometry tests；build/ci/m5/ | NOT_RUN |

## 4. Kinodynamic A* 与局部地图矩阵（仅 Host/SIL）

本组在对应模块尚未实现前保持 NOT_RUN；不能用 NONE 模式的围栏测试代替 A* 或地图测试。

| ID | 输入/场景 | 预期结果 | 失败判据 | 自动化与证据 | 当前状态 |
|---|---|---|---|---|---|
| M5-AST-001 | 固定 primitive manifest、状态 key、同一输入重复运行 | primitive 顺序、key 编码和输出路径确定性一致 | 相同输入产生非确定性路径或 key 冲突 | Host golden vectors；build/sil/m5_ast/ | NOT_RUN |
| M5-AST-002 | 窄通道、转弯、上升/下降、等待边 | 搜索得到可行路径；time_bin 不同的等待节点不得合并 | 违反动力学、时间状态被错误合并 | Host deterministic map fixture；build/sil/m5_ast/ | NOT_RUN |
| M5-AST-003 | 目标被障碍封闭、目标不可达 | 返回 NO_PATH，不发布部分路径 | 无路仍返回成功或输出截断路径 | Host negative map test；build/sil/m5_ast/ | NOT_RUN |
| M5-AST-004 | Node/Open pool 达 512、每节点 64 边、单请求 512 扩展上限 | 明确返回容量/预算错误，内存不越界 | 越界写、无界扩展、超限后继续成功 | Host capacity/fuzz test；ASan/UBSan；build/sil/m5_ast/ | NOT_RUN |
| M5-AST-005 | 哈希冲突、线性探测上限、time_bin 越界 | 完整 key 比较；探测/时间上限触发明确错误 | 只比较部分 key 或无限探测 | Host collision/property test；build/sil/m5_ast/ | NOT_RUN |
| M5-AST-006 | 根节点非网格位置/速度/加速度/yaw | 根状态保留真实导航值，首条 quintic 连续接到规范端点 | 把根状态吸附到网格造成跳变 | Host golden vectors；build/sil/m5_ast/ | NOT_RUN |
| M5-AST-007 | 到达接受球边缘、终端速度或 yaw 不满足 | 不算到达；继续搜索或返回 NO_PATH | 瞬移到航点或忽略末端条件 | Host boundary test；build/sil/m5_ast/ | NOT_RUN |
| M5-AST-008 | planning_deadline_monotonic 到期 | 返回 PLANNING_DEADLINE_MISSED；不发布部分结果 | deadline 过期后仍发布候选 | Host fake-clock test；build/sil/m5_ast/ | NOT_RUN |
| M5-MAP-001 | 16×16×16 固定地图、容量边界和地图外查询 | 4096 cells 固定容量；越界显式拒绝 | 隐式扩容、越界访问或地图外当 FREE | Host capacity/fuzz test；build/sil/m5_map/ | NOT_RUN |
| M5-MAP-002 | UNKNOWN/FREE_CONFIRMED/OCCUPIED_CONFIRMED/STALE/INVALID 状态转换 | UNKNOWN、STALE、INVALID、地图外均不可当作自由空间 | 未知或过期单元被解释为 FREE | Host state-machine test；build/sil/m5_map/ | NOT_RUN |
| M5-MAP-003 | 有效观测中断超过 500ms、source health 退化/REJECT | cell 转 STALE；依赖障碍保证的任务拒绝或保守降级 | 观测失效后继续沿用旧 FREE 结论 | Host fake-clock/fault injection；build/sil/m5_map/ | NOT_RUN |
| M5-MAP-004 | frame、origin_id、state_epoch、sample time 或 TTL 不一致 | 拒绝地图/观测并返回稳定原因码 | 跨原点或旧时间数据参与规划 | Host contract test；build/sil/m5_map/ | NOT_RUN |
| M5-MAP-005 | 安全走廊超容量、地图队列溢出、观测乱序 | 固定容量和有界处理；溢出显式报告并 fail-closed | 静默丢观测、扩大数组或错误宣称安全 | Host stress/fuzz；build/sil/m5_map/ | NOT_RUN |

规范边界：4096 cells；历史静态地图夹具分辨率 0.5m/cell；动态障碍 track 最多 8 条；UNKNOWN/STALE/INVALID/地图外不得视为 FREE。上述是 Host/SIL 设计夹具，不是当前 H723 障碍感知能力。

## 5. Minimum-jerk 优化与动力学矩阵（Host/SIL）

V1.9.1 采用固定时长 minimum-jerk 五次样条，最多 8 段；内部 knot 位置固定，优化内部 knot 速度/加速度，目标为 jerk 平方积分。固定维数带状 SPD 求解，静态 workspace。不得恢复旧版“运行时联合时间优化”或把 S-Curve 作为另一个算法分支。

| ID | 输入/场景 | 预期结果 | 失败判据 | 自动化与证据 | 当前状态 |
|---|---|---|---|---|---|
| M5-OPT-001 | 1～8 段、不同段时长和边界状态 | 段数/工作区固定上限；不分配 heap | 越界段数或动态扩容 | Host + ARM static scan；build/sil/m5_opt/ | NOT_RUN |
| M5-OPT-002 | 内部 knot 位置固定、速度/加速度可优化 | 满足 p/v/a 连续性并降低 jerk 平方积分目标 | knot 位置漂移、连续性断裂或目标计算错误 | Host golden/numerical comparison；build/sil/m5_opt/ | NOT_RUN |
| M5-OPT-003 | SPD 分解成功、奇异/病态输入 | 成功时输出有限系数；失败原子回退到同一路径初始 quintic 并重新完整验证 | 回退到另一条未验证路径或发布中间结果 | Host numerical fault injection；build/sil/m5_opt/ | NOT_RUN |
| M5-OPT-004 | 优化失败且初始 quintic 也无法通过 M5-06 | 返回 OPTIMIZATION_FAILED / TRAJECTORY_REJECTED | 未通过验证仍发布 | Host negative test；build/sil/m5_opt/ | NOT_RUN |
| M5-OPT-005 | 时间参数化反复不满足 Bernstein 界 | 最多 16 次；总时长 ≤6.2s；失败明确拒绝 | 无界重试或无可行时长仍发布 | Host property test；build/sil/m5_opt/ | NOT_RUN |
| M5-DYN-001 | 三轴速度、加速度、jerk 边界及刚好超限输入 | 全轴检查；边界内通过，超限拒绝 | 只检查合速度或只检查采样点 | Host property/golden test；build/sil/m5_dyn/ | NOT_RUN |
| M5-DYN-002 | 推力、倾角、yaw rate、终端速度/航向约束 | 使用 Product SafetyEnvelope 和模型有效域；超限拒绝 | 以位置可达代替动力学可行 | Host boundary test；build/sil/m5_dyn/ | NOT_RUN |
| M5-DYN-003 | 导航不确定度、测量年龄、控制/执行器延迟变化 | 安全裕度按已批准系数保守膨胀；缺失系数拒绝 | 将 confidence 当概率或忽略延迟 | Host parameter/fault injection；build/sil/m5_dyn/ | NOT_RUN |

## 6. M5-06 连续碰撞、动态障碍与围栏矩阵（Host/SIL）

| ID | 输入/场景 | 预期结果 | 失败判据 | 自动化与证据 | 当前状态 |
|---|---|---|---|---|---|
| M5-COL-001 | 障碍在两个预览采样点之间穿越轨迹 | swept-volume/区间证明发现冲突并拒绝 | 只检查离散点而漏检 | Host adversarial geometry；build/sil/m5_collision/ | NOT_RUN |
| M5-COL-002 | 中点净空 c_mid 与相对速度/加速度上界 | 只有 c_mid - b > 0 才能认证该区间安全 | 缺少上界仍报告 VALID | Host proof-boundary test；build/sil/m5_collision/ | NOT_RUN |
| M5-COL-003 | 24 个消费区间、细分深度 5、总访问 256 | 固定预算内完成；达到深度/访问上限即拒绝 | 超预算继续、栈越界或未证明仍通过 | Host capacity/fuzz + ASan/UBSan；build/sil/m5_collision/ | NOT_RUN |
| M5-COL-004 | 交叉/迎面动态障碍、速度/加速度预测误差 | 同一 monotonic 时间轴相对运动验证；不确定时保守拒绝 | 不同时间轴直接比较或忽略预测误差 | Host time-series simulation；build/sil/m5_collision/ | NOT_RUN |
| M5-COL-005 | 预测过期、观测年龄/不确定度窗口超出有效期 | 超出有效窗口按 UNKNOWN/保守占据处理 | 预览尾部无预测仍假定安全 | Host fake-clock test；build/sil/m5_collision/ | NOT_RUN |
| M5-COL-006 | 车辆包络、位置/速度标准差、反应延迟变化 | 膨胀包含机体包络、产品裕度、不确定度和延迟 | 未标定参数用于真实障碍源或漏算响应延迟 | Host parameter sweep；build/sil/m5_collision/ | NOT_RUN |
| M5-COL-007 | 围栏连续穿越、无效/过期/错 origin 围栏 | 连续段检查；无效围栏 fail-closed | 只检查航点或使用旧坐标围栏 | Host geometry/fault injection；build/sil/m5_collision/ | NOT_RUN |
| M5-COL-008 | 验证器 UNKNOWN、容量耗尽、reason code 映射 | 对 M4/M6 等同拒绝；原因码稳定 | UNKNOWN 被降级当 VALID | Host contract test；build/sil/m5_collision/ | NOT_RUN |

预测窗口要求：Host 合成源至少提供 1s 预测界；实际参考覆盖 [t0, t0+480ms]，并额外覆盖有效观测年龄/不确定度窗口。ONBOARD_SENSOR 当前不支持，不能把 Host 合成预测当作实物保证。

## 7. 重规划、M5→M6 与 Companion 矩阵

| ID | 输入/场景 | 预期结果 | 失败判据 | 自动化与证据 | 当前状态 |
|---|---|---|---|---|---|
| M5-REP-001 | 活动参考中途生成新候选，切换时刻 p/v/a/yaw 对齐 | 仅在验证通过后原子切换；切换连续 | 位置/速度/加速度跳变或先发布后验证 | Host trajectory-splice test；build/sil/m5_replan/ | NOT_RUN |
| M5-REP-002 | reference_epoch 更新、旧候选延迟到达 | 新 epoch 生效；旧 epoch 永不复活 | 旧候选覆盖新参考 | Host reorder/fault injection；build/sil/m5_replan/ | NOT_RUN |
| M5-REP-003 | NO_PATH、容量耗尽、deadline、优化失败、validator 拒绝 | 错误码区分；无部分路径发布 | 错误被映射为成功或复用旧候选 | Host negative test；build/sil/m5_replan/ | NOT_RUN |
| M5-INT-001 | MotionReference anchor + 24 点，20ms 间隔 | 预览覆盖 480ms；offset 严格递增且时间锚点一致 | offset 倒退/重复、点数不足仍发布 | Contract + Host test；build/sil/m5_m6/ | NOT_RUN |
| M5-INT-002 | MPC Np、控制周期、插值时刻与预览长度组合 | 覆盖 MPC 实际预测时域；不足时拒绝或按已批准策略处理 | 静默外推预览尾部或使用旧点 | M5/M6 integration + trace；build/sil/m5_m6/ | NOT_RUN |
| M5-INT-003 | Hermite 插值、yaw 跨界、NaN/Inf、点无效位 | 与 02 唯一插值契约一致；异常输入拒绝 | M5/M6 对同一参考得到不同插值 | Host golden vectors；build/sil/m5_m6/ | NOT_RUN |
| M5-INT-004 | M6 跟踪误差、QP infeasible/timeout、参考过期 | 验证跟踪误差和求解失败行为；不把 M5 规划通过等同于控制通过 | 无效参考仍输出可执行 Demand | Full SIL；build/sil/m5_m6/ | NOT_RUN |
| M5-CMP-001 | Companion 关闭、断链、超时、重启 | H723 本机基础点到点/多航点/返航不依赖 Companion | 断链导致基础任务中断或自动接纳旧建议 | Full SIL/HIL；build/sil/companion/ | NOT_RUN |
| M5-CMP-002 | Companion 候选过期、乱序、重放、frame/origin/state_epoch 不一致 | 拒绝候选；不得覆盖本机参考 | 错坐标/旧 epoch 候选被接纳 | Host protocol + SIL fault injection；build/sil/companion/ | NOT_RUN |
| M5-CMP-003 | Companion 试图直通 MotionReference、ActuatorDemand 或电机命令 | 主飞控只接收可选建议；最终参考由本机 M5 生成/验证/发布 | 外部数据绕过 M4/M5/M6/M7 | Dependency/dataflow scan；build/sil/companion/ | NOT_RUN |
| M5-CMP-004 | RC 接管、RC 恢复、摇杆回中 | 按 M4 授权切换；恢复/回中不自动复活旧自动参考 | 旧自动参考重新 ACTIVE | FMS/M5 full SIL；build/sil/companion/ | NOT_RUN |

## 8. H723 资源与实时性验收矩阵

以下为 spec/07 的设计接纳上限，不是当前目标板实测结果。Host 的运行时间不得换算成 H723 WCET。

| ID | 验收项 | 设计基线/通过标准 | 失败判据 | 证据 |
|---|---|---|---|---|
| M5-RT-001 | Cortex-M7 编译 | GNU Arm release；C11、Wall/Wextra/Werror、fno-short-enums；生产源完整编译 | 编译警告被屏蔽或 ABI flags 不一致 | build/m5_arm/ |
| M5-RT-002 | M5 内存 | NONE 直接轨迹/validator arena ≤16KiB；MotionReference 2×1536B；对象均纳入 map | 隐藏静态副本、超限或 map 未覆盖 | ELF/MAP、size、arena report |
| M5-RT-003 | 任务栈 | M5GuidanceTask 3072B；Planner Worker 4096B；压力测试使用量 ≤70%，保留 ≥30% | 栈余量不足 30%、canary/overflow | stack watermark、.su、stress log |
| M5-RT-004 | Guidance 周期 | 50Hz；单次预算 100us；20ms deadline | WCET 超预算或 deadline miss | DWT/trace、RTA report |
| M5-RT-005 | Planner Worker | ≤300us/slice；每 100ms 累计 ≤300us；Host/SIL 单请求参考上限 2s 不是 H723 WCET | 借用其他任务预算、超限后仍继续 | DWT/trace、budget counters |
| M5-RT-006 | CPU 预算 | M5 Guidance ≤0.5%；Planner Worker ≤0.3%；不侵占 M3/M6/DMA 保留 | 只报 1s 平均值，短窗过载或关键 deadline miss | CPU trace + fixed-priority RTA |
| M5-RT-007 | 并发最坏组合 | M3 1kHz、M6 fast/MPC、M7、M4/M5、通信/日志同时运行；关键 deadline miss=0 | 只测 M5 空载或平均耗时 | H723 DWT/trace + HIL |
| M5-RT-008 | NONE 能力边界 | H723/NONE 不分配 A*/地图 arena；ONBOARD_SENSOR 不可启用 | 把 Host 地图/A* 测试当作 H723 已部署能力 | Profile/build manifest + map |

## 9. SIL / PIL / HIL / 发布门禁

| Gate | 范围 | 必须完成 | 不允许的替代 |
|---|---|---|---|
| G0 Host 单元 | M4/M5 基础轨迹、数学、授权、Epoch、围栏、NONE 拒绝 | GCC Werror、Host tests、ASan/UBSan、golden vectors | 只看编译通过 |
| G1 Host 高级规划 | A*、地图、minimum-jerk、连续碰撞、容量/超时/故障注入 | 固定数据集、确定性 golden、边界/随机测试、完整日志 | 用 H723/NONE 围栏测试代替 A*/碰撞 |
| G2 闭环 SIL | M2→M3→M4→M5→M6→M7→Plant/Sensor | 起飞/悬停/点到点/多航点/返航/降落；GNSS/IMU/RC/Companion/MPC/执行器故障 | 将 PlantTruth 直接送入 M3 或只测 M4/M5 |
| G3 目标构建与资源 | H723 生产配置、ELF/MAP、ABI、DWT/WCET、栈、RAM、RTA | 最坏并发负载，关键 deadline miss=0，栈余量≥30% | Host 时间、静态 sizeof 或 ARM 单文件编译代替 |
| G4 HIL/台架 | H723 真实固件 + 实时 Plant/Sensor 仿真 + 隔离执行器接口 | 输出守卫、Permit、RC、GNSS/PPS、时间异常、传感器失效、链路断开和恢复 | Host SIL 代替 HIL；带桨开放输出 |
| G5 飞行放行 | 实机参数/传感器/机体/ESC 标定、危险分析、逐级飞行测试 | 用户/系统架构负责人批准，全部适用 Gate 有可追溯证据 | 任何 CI、Host、SIL 或 HIL 单项通过直接等同可飞行 |

当前基线仍为 FLIGHT_RELEASE_NOT_AUTHORIZED。BENCH_INHIBIT 是台架配置，不是飞行放行。

## 10. 每项测试的证据模板

每个 TEST ID 单独保存一份记录；同一日志不得复用到不同 commit/Profile。

    test_id:
    requirement_id:
    baseline_spec:
    commit:
    profile_id:
    obstacle_source_mode:
    schema_version:
    product_port_revision:
    config_hashes:
    compiler_and_version:
    host_or_target:
    board_revision:
    command:
    exit_code:
    input_dataset_and_sha256:
    expected:
    actual:
    verification_status:
    log_path:
    artifact_paths:
    source_sha256_manifest:
    wcet_max_us:
    deadline_us:
    cpu_window_and_peak:
    static_ram_bytes:
    stack_allocated_bytes:
    stack_peak_bytes:
    concurrent_load:
    fault_injection:
    reviewer:
    scope_limitations:
    residual_risks:

## 11. 当前可复现命令

从 Flight 仓库根目录执行：

    bash tests/flight_management/run_host_tests.sh
    bash tests/guidance/run_host_tests.sh
    bash tests/guidance/run_integration_tests.sh
    EXTRA_CFLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer" bash tests/guidance/run_host_tests.sh
    EXTRA_CFLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer" bash tests/guidance/run_integration_tests.sh
    bash tests/guidance/arm_build_check.sh

Host/SIL 高级 A*/地图/动态障碍测试入口须随对应实现一并提交；在实现和数据集缺失时，不得创建空壳测试并标记 PASS。


## 12. 2026-11-09 实施切片与边界

### M5-04：有界时间参数化

- 所有 M5 单次规划请求统一使用 200ms 时间格，最多 31 格、总时长不超过 6.2s；最多执行 16 次有界可行性尝试。
- 不再为 Mission/Return 设置 240s 特例。超出局部规划窗口的 Mission/Return 请求 fail-closed，不能发布截断或超长参考。
- M4 必须负责把长距离任务拆成新的、逐项授权的中间航点/任务段。当前尚未实现 M4 自动分段，因此长航段能力仍受限，不能宣称长距离任务完整可用。

### M5-05：Host/SIL Minimum-jerk 数学内核

- 新增 \`middleware/guidance/trajectory_planning/minimum_jerk_optimizer.c\`，仅在 \`FLIGHT_M5_HOST_PLANNER\` 下编译；H723/NONE 不调用该内核、不分配其 workspace，仍使用直接有界 quintic。
- 固定最多 8 段、14 个内部变量；内部 knot 位置固定，只优化内部 knot 速度/加速度，起止 p/v/a 固定。
- 目标为三轴 \`Σ∫ jerk² dt\`。局部段二次型组装为带宽 3 的块三对角 SPD 系统，使用固定维数 Cholesky 求解。
- 调用者提供固定 workspace；workspace 编译期大小上限 2304B。内核无 heap 分配；输出在所有轴均成功后一次性复制，失败时输出保持清零。
- 内核只生成数学轨迹候选，不验证速度/加速度/jerk、推力/倾角、围栏或障碍；必须经过独立 M5-06 验证后才可进入 M5 Publisher。当前尚未接入 A* 路径、M5-06、原子回退和实际 MotionReference 发布链。

### 已增加的 Host 用例

33 个命名场景中新增：单段边界、双段连续性与目标下降、非等时长多轴、8 段确定性、非法输入/时间格、三轴目标函数求和、非零起止速度/加速度。长 Mission/Return 超出 6.2s 的请求现验证为拒绝；另新增验证确保 1.25 倍增长越过上限时仍会尝试最终 6.2s 时间格。

### 验收边界

- M5-04 的局部时标器回归和 M5-05 数学内核单测属于 IMPLEMENTATION/Host 范围；不能把它们记作完整 M5-03、M5-05 或 M5-06 通过。
- A*、局部地图、动态障碍、连续碰撞、M5-06、优化失败原子回退、M5→M6 完整消费、H723 WCET/栈/RAM、PIL/HIL 与飞行放行仍需独立证据。
- 具体状态只登记在 \`spec/09_需求验证实施与发布管理.md\`；本矩阵不建立第二套验收状态台账。
