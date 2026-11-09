# FMS、通信与系统安全设计（V1.9.1）

> **规范性：** 本文件是 M4 FMS 状态、权限、Failsafe 和 MAVLink 命令安全的唯一详细来源。所有状态/接口字段以 02 ICD 为准。


> **项目落位：** M1 链路在 `middleware/communication/`，MAVLink 编解码和生成布局在 `middleware/protocol/`。M4 公共入口在 `middleware/model/fms/fms_interface.c/.h`，QUAD_X4 实现在 `middleware/model/fms/quad_x4/`。自定义 Dialect 在 `spec/contracts/FCU.xml`，`tools/generate_fcu_mavlink_layout.py` 生成协议布局，`middleware/protocol/mavlink/wire_generation.json` 记录来源。UART/CAN 传输复用 `libraries/HAL_Drivers/` 和 RT-Thread 设备类。目录和接入见 03 §0。

## 1. M1 IO & COMMUNICATION

### 1. 职责

M1 负责 RC、GCS/数传、Companion、ADS-B/外部系统的协议接入、来源认证/完整性、序号/时效、链路质量监测、原始操作输入归一化和遥测。M1 只报告通信事实，不决定飞行模式/返航/接管。

### 2. 组件

- RC Receiver Adapter：解析通道、接收机 failsafe、帧序号、输入布局和链路诊断。
- GCS/Telemetry Adapter：协议校验、身份/权限凭据、命令时效和遥测。
- Companion Adapter：外部里程计、地图/障碍物摘要和路径建议接入；不得把外部轨迹作为最终MotionReference发布。
- Link Monitor：RC/GCS/Companion 独立状态机和质量统计。
- Command Parser：输出 OperationRequest，不直接改变 FMS 状态。
- Operator Input Normalizer：按产品通道映射和校准输出归一化 OperatorInput；不判断是否接管。
- Telemetry Publisher：状态、链路、故障和决策遥测。

### 3. 链路状态

每个 link_instance 独立维护 UNKNOWN/INITIALIZING/HEALTHY/DEGRADED/LOST/RECOVERING、last valid frame、sequence gap、packet loss、CRC、RSSI/LQ、receiver failsafe、time quality 和 recovery dwell。链路恢复须连续有效帧稳定一段配置时间，不能由单帧触发。

`INTERFERENCE_SUSPECTED` 只能是证据标签；无接收机干扰指标时不能把失联原因断言为干扰/压制。

### 4. 操作输入

M1 输出 `OperatorInput` 原始归一化意图和有效性，不输出 `AuthorizedOperatorInput`。M4 根据模式开关、接管手势、链路状态和安全门控进行权限仲裁，再把授权输入交给 M5。M1 不把摇杆中位解释为接管或退出接管。

### 5. 故障行为

- 帧过期：标记输入 STALE，不重复发布为新鲜命令。
- 序号回退/重放：拒绝并计数。
- CRC/认证失败：拒绝该消息并上报证据。
- GCS 丢失不等于 RC 丢失；Companion 丢失不等于导航估计失效。
- Companion ExternalOdometry/ObstacleSnapshot/PathProposal过期、reset epoch或frame不匹配时拒绝；Companion不得提供绕过H723 M5验证的最终MotionReference。
- 缓冲溢出必须计数并上报，不得静默覆盖关键命令。

### 6. 参数

帧超时、LQ/RSSI 判定、恢复 dwell、接管死区和命令 TTL 由接收机适配器与车辆 Link Supervision 安全契约定义。实际数值必须依据接收机协议、帧率、failsafe 输出和地面测试确定；变更走 CR，不作为普通控制调参。

### 7. 地面站联机仿真

LGroundControl 安装版通过本机 UDP 使用标准 MAVLink。Host 仿真适配器接收 MAVLink 1 和 2 请求，发布 Flight 估计状态和当前有效的 FMS 返航点，把解锁、起飞、降落、返航和上锁转换成 FMS 请求，并返回 FMS 回执。FMT-SIM 在独立回环端口提供机体状态，只接收经 Host 输出守卫确认的虚拟电机指令。该入口仅编入 Host SIL。

地面站适配所用参数目录从根 `config/` 四份源文件派生，区分已接入 Host 的字段与资料、待标定和固定结构字段。MAVLink 参数读取提供当前 Host 配置，CORE 参数通过源文件编辑并重建 Host 生效。原生参数页和内置仿真菜单的替换依赖地面站源码或厂商扩展接口。

---

## 2. M4 Vehicle Management / FMS Policy

### 1. 唯一职责

M4 由编译期选定的车辆专用 Policy 完整实现操作管理。航空产品是 Aerial FMS；无人车是 Ground VMS；无人船是 Marine VMS。公共 Port 只统一输入/输出 envelope、生命周期、有效期、审计事件和基础契约，不统一所有车辆的操作状态图。

M4 是唯一操作级决策者：命令权限仲裁、操作模式/阶段、导航/控制/执行器能力门控、安全约束、通信失联策略、返航/停车/靠泊/降落决策和人工接管决策。

M4不运行INS/KF-GINS，不做单次观测融合门控，不隔离传感器，不生成路径/轨迹，不执行 MPC/PID，不访问 HAL/RT-Thread Device。

### 2. 输入

- M1：OperationRequest、OperatorInput、CommunicationLinkStatus、来源认证/完整性状态。
- M3：NavigationSnapshot、NavigationCapability、EstimatorHealth。
- Sensor Integrity：SensorUsePolicy 状态摘要和源故障事件。
- M5：GuidanceStatus、GuidanceFeasibility、GuidanceHealth；MissionStatus由M4唯一发布。
- M6：ControlCapability、ControlHealth、控制饱和/模型失配证据。
- M7：ActuatorCapability、ActuatorStatus、执行器故障。
- PLATFORM/HEALTH：电源、电池、资源、时钟、任务健康和 FaultSummary。
- 环境/能源：风/流/障碍物/地理围栏/剩余能量等产品相关数据。

M4 只读取公开快照和能力，不得读取 M3 协方差矩阵、滤波器内部状态或 M6 私有增益。

### 3. 输出

- `VehicleDirective`：产品专用操作/阶段、允许的 Guidance、SafetyEnvelope、directive epoch 和有效期。
- `ActuationPermit`：许可标志、permit epoch、有效期和失效行为策略 ID。
- `AuthorizedOperatorInput`：经权限仲裁后的操作员输入，含 authority epoch 和选定来源。
- `CommandAck`、`DecisionEvent`、`FmsHealth`。

### 4. 状态建模：避免单一巨大枚举

每个车辆 Policy 可按正交维度组织状态，而不是把所有组合塞入一个 enum：

- 生命周期：DISARMED / READY / ACTIVE / TERMINAL（具体车辆定义）。
- 操作模式：MANUAL / ASSISTED / AUTO_MISSION / AUTO_RETURN / AUTO_LAND 等产品专用集合。
- QUAD普通Authority仅NONE/PILOT/MISSION/GCS_SUPERVISED；SAFETY是强制约束，Companion只提供ADVISORY。
- 安全状态：NORMAL / DEGRADED / FAILSAFE_ACTIVE / RECOVERY_PENDING。
- 车辆阶段：航空的 TAKEOFF/CRUISE/APPROACH/LANDING；其他车辆自行定义。

正交状态必须由单一 FMS Policy Owner 更新，并在每次权限/模式转换时递增 epoch。

### 5. 操作权限仲裁原则

1. M7 硬件输出范围/电气保护始终最终生效。
2. 强制安全策略（例如关键能源、不可控、严重地理围栏/碰撞风险）可限制普通人工/任务命令。
3. 经认证且新鲜的显式飞手接管优先于任务、GCS 和 Companion 的普通任务请求。
4. GCS/Companion 默认只能提交任务或轨迹建议，不得静默夺取飞手权限。
5. 失联/过期/重复/旧 authority epoch 命令必须拒绝。
6. 中位摇杆不是接管意图；链路恢复不是权限恢复；权限恢复不是自动模式恢复。
7. 所有权威切换必须有 DecisionEvent、原因、时间、来源和新 epoch。

### 6. RC 失联与恢复：Aerial Policy 默认规则

- RC 链路质量由 M1 独立监控；RC 接收机 failsafe 标志或超过配置的帧超时后，M1 发布 LINK_LOST 证据。M1 不发布 RTL。
- Aerial FMS 根据当前模式/飞行阶段、导航能力、Home 有效性、返航路径可行性、能源余量、地理围栏和执行器能力决定 RTL、保持、受控降落或紧急处置。
- 在 MANUAL/ASSISTED 中丢失 RC：如果位置导航、Home、能源和返航可行性满足产品条件，进入 RTL；否则选择当前可用能力下的安全降级动作（如定高/姿态稳定/受控降落），不得盲目发出位置返航命令。
- 在 AUTO_MISSION 中丢失 RC：默认中止/暂停任务并评估 RTL；仅当配置明确允许、任务已证明可无人监督且能源/围栏/障碍约束满足时，才允许继续任务。
- 在 RTL 中 RC 恢复：必须先经过链路稳定恢复判定。若没有显式接管意图，继续 RTL，不自动退出。
- 显式接管：优先使用专用模式开关；可选摇杆手势必须由产品安全契约显式启用，并满足校准死区、连续确认时间和链路健康条件。接管成功后发布新 authority epoch。
- 默认 `override_mode=LATCHED`：接管后保持飞手权限；摇杆回中不自动恢复 RTL。中位输入的实际行为由当前人工/辅助模式定义，例如定位有效时位置保持，只有高度有效时定高/姿态稳定；不得假定所有模式的中位行为相同。
- 可选 `MOMENTARY_OVERRIDE` 仅用于明确设计的产品：只有操纵量回中持续满足 dwell、模式开关仍选择 AUTO、被暂停任务/轨迹仍有效、导航/能源/围栏条件重新通过时才允许恢复；恢复前必须生成新 directive/authority epoch。默认关闭。
- `rc_recovery_no_takeover_timeout` 默认禁用（0）；在 RTL 中恢复链路但无人接管，继续 RTL，不因等待超时切换手动、不取消 RTL。
- 到达返航终点后的动作由 `rtl_terminal_action` 决定：优先在 Home/着陆点有效且降落能力满足时自动降落；否则在能源/安全预算内保持或盘旋；超过产品安全契约定义的终端等待/能源边界后执行预先验证的替代降落策略。不得无限期悬停，也不得仅因 RC 无人接管而自动解锁/停机。
- 接管后再次 RC 丢失：重新按当前实际模式/阶段评估，不机械重放旧返航路线；需要基于当前状态重新规划。

### 7. GCS/数传/Companion 丢失

- GCS 数传失联但 RC 正常：默认不触发 RC failsafe；继续当前安全模式，拒绝过期地面命令，记录监督链路丢失。
- RC 失联但 GCS 正常：仍按 RC Loss Policy 决策；地面链路不能自动等同于飞手操纵链路。
- RC 与 GCS 同时失联：按 RC Loss 策略执行，同时停止依赖实时地面监督的任务。
- Companion失联：M5丢弃过期外部地图/障碍/路径建议，继续本机Guidance；若本机任务/导航/规划能力不足，M4再选择RTL/保持/降落。Companion不得成为基础飞行安全的唯一依赖。
- RSSI/LQ 下降但有效帧仍在：LINK_DEGRADED，预警并限制新任务；达到丢失条件才进入 LINK_LOST。
- 怀疑干扰/压制时只标记 `INTERFERENCE_SUSPECTED`，不得在没有频谱/接收机证据时断言被干扰。

### 8. 传感器/导航/运动异常响应

M4 消费的是能力和证据，不接管诊断：

- 单个 GNSS 失效但惯导/视觉/其他定位仍可用：降级导航能力，按任务需求继续或返航。
- GNSS 疑似欺骗/压制：不得继续信任可疑位置；评估其他导航源、Home 和围栏能力，必要时受控降落。
- 磁力计干扰：航向能力降级，不等同于位置全部失效。
- 气压计异常：高度能力重新评估，按 range/GNSS/其他高度源选择可用模式。
- IMU 全部不可用或姿态解失效：进入产品定义的紧急处置；不得要求 M3 自行决定动作。
- 外部扰动/碰撞/模型失配：先评估 M6 控制余量、M7 执行器能力和导航状态；必要时限制速度/倾角、退出任务、降落。不得仅凭大创新隔离传感器。
- 电机故障/控制饱和：按可控自由度和机型危险分析决定继续、返航或紧急降落；不能假定剩余电机必然可控。

### 9. FMS 故障与安全许可

VehicleDirective/ActuationPermit 必须有 epoch 和有效期。M7 对许可过期采取的动作由产品危险分析定义；禁止通用“许可过期就电机归零”。在空中立即切断四旋翼电机可能造成灾难性后果。M4 任务失效、M6 控制失效、MCU 复位是不同故障，必须分别分析。M4 故障时 M6 可在受限时间内保持最后一个经验证的安全控制模式，具体由产品配置和系统安全分析决定；M6 本身失效不能靠 M7 单独恢复姿态控制。

### 10. 非航空车辆

Ground VMS 对 RC/遥控丢失通常采用受控减速/停车/驻车；Marine VMS 根据航道、流、风、推进器和靠泊能力选择降速、定点保持、返航或安全漂航。不得复用航空 RTL 枚举或状态图。

### 11. 参数和审计

超时、阈值、恢复 dwell、接管死区、RTL 终端动作和能源阈值由车辆专用 FMS Policy/Link Supervision 安全契约定义，不属于日常工程调参。任何变更必须通过 CR、独立安全审查和场景回归。每次决策记录触发证据、参数集 hash、policy version、时间、旧/新模式、authority epoch 和拒绝原因。

### 12. 验证

必须覆盖 RC/GCS/Companion 独立丢失/恢复、RTL 中恢复但摇杆中位、明确接管、接管后回中、接管后再次失联、模式开关抖动、RC/GCS 冲突、过期命令、任务失效、导航/能源/执行器降级、Home 无效、RTL 不可行、FMS 任务超时和 M7 permit 过期。

### 13. 场景验收矩阵

首版航空 FMS 的逐场景输入、前置能力、默认动作和禁止行为见 `06_FMS通信与系统安全设计.md`（Aerial场景决策矩阵章节）。实现不得只覆盖“RC loss -> RTL”单一路径，必须覆盖 RTL 中恢复但无接管、接管后回中、接管后再次丢失、Home/能源/导航/执行器不可用等组合场景。

## 3. 故障安全、权限和状态机边界

### 1. 状态所有权

| 状态/事实 | 唯一 Owner |
|---|---|
| 设备驱动状态、DMA/总线事实 | RT-Thread 原生驱动/BSP |
| 样本质量与校准应用结果 | M2 |
| 导航状态/协方差/导航能力/观测级拒绝 | M3 |
| 传感器源级健康/隔离/恢复策略 | Sensor Integrity Manager |
| RC/GCS/Companion 链路状态与原始操作输入 | M1 |
| 操作员/任务/GCS权限仲裁、车辆模式和操作响应 | 选定 M4 FMS Policy；Companion仅作为建议输入，不拥有authority |
| 任务执行进度/MissionStatus/当前项 | M4 MissionManager |
| 路径/轨迹/GuidanceStatus | M5 |
| 控制器/模型/控制饱和/备用控制状态 | M6 |
| 执行器输出保护/反馈/执行器状态 | M7 |
| 故障事件历史和摘要 | HEALTH |

禁止两个模块维护同一状态机的可变副本。其他模块只读快照或提交请求。

### 2. 故障处理链

```text
硬件/总线/DMA事实 -> Driver/BSP -> M2 样本诊断 -> M3 观测门控
                                   \              /
                                    Sensor Integrity Manager
                                      | SensorUsePolicy -> M3
                                      | FaultSummary -> M4

M1 LinkStatus/OperatorInput ------------------------------------> M4
M3 NavigationSnapshot/NavigationCapability ----------------------> M4
M6 ControlCapability/ControlHealth ------------------------------> M4
M7 ActuatorStatus/ActuatorCapability/ActuatorHealth -------------> M4
HEALTH FaultSummary/ResourceHealth ------------------------------> M4
M4 车辆专用策略 -> VehicleDirective/ActuationPermit -------------> M5/M6/M7
```

### 3. 故障不是一个布尔值

分别报告：设备健康、样本质量、观测一致性、导航解能力、车辆运动/控制状态、通信链路状态、执行器能力。一个维度退化不能无条件推导另一个维度失败。

### 4. 故障响应原则

1. 先由本地模块保护数据/输出，再由 M4 做操作级决策。
2. M2/M3 可以拒绝当前坏样本；源级隔离由 Sensor Integrity Manager 管理。
3. M3 只发布导航能力，不发 RTL/停车/靠泊命令。
4. 大创新/大跟踪误差可能是真实外部扰动、模型失配、杆臂/时间错误或传感器异常；单一残差不得定因。
5. M4 根据当前车辆阶段、可用导航能力、执行器能力、能源、地理围栏和任务可行性选择动作。
6. M6 MPC 超时/不可行不得无限期使用旧解；切换到已验证备用控制或产品定义的降级策略。
7. M7 本地输出范围和硬件保护独立于 M4；输出许可过期动作不得统一写成“输出零”。
8. 链路恢复不自动恢复操作员权限；摇杆回中不自动恢复自动模式；重新接管必须新 epoch。
9. 故障恢复必须经过稳定时间、样本数、滞回和 probation；不能因单帧恢复立即清除锁存故障。
10. 安全动作必须来自车辆专用危险分析和配置，不得使用跨车辆通用 RTL/STOP/DOCK 行为。

### 5. 分层状态机

- Sensor Integrity：UNKNOWN/INITIALIZING/HEALTHY/SUSPECT/DEGRADED/QUARANTINED/FAILED/RECOVERY_PROBATION。
- Estimator：INITIALIZING/VALID/DEGRADED/INVALID；内部观测拒绝独立记录。
- Link：UNKNOWN/INITIALIZING/HEALTHY/DEGRADED/LOST/RECOVERING。
- M4：车辆专用状态，不要求跨车辆统一节点。
- M6：ACTIVE/DEGRADED/BACKUP_ACTIVE/FAILED 等控制器状态。
- M7：输出许可、输出保护、硬件故障状态；本地保护动作产品专用。

### 6. 故障恢复

恢复需满足对应域的独立条件：链路恢复需连续有效帧和质量稳定；传感器恢复需自检/数据/时间/创新一致性通过 probation；导航恢复需状态重新初始化或 reset 事件被消费者正确处理；FMS 自动任务恢复需任务和轨迹重新验证；执行器恢复需台架/反馈/输出能力重新确认。恢复不自动等同于模式恢复。

### 7. 单点故障限制

单 MCU 系统不能保证在 MCU 复位、M6 完全失效或关键 IMU 全失效后仍能维持可控飞行。软件不能凭空创造冗余控制能力。必须在本文件第6章与09 Gate F记录系统级残余风险、硬件独立保护需求和产品放行限制。

### 8. 安全验证

传感器单/多故障、外部扰动、模型失配、共同时间源异常、链路失联/恢复、权限抢占、MPC 超时、执行器失效、FMS/M6 任务挂死、M7 permit 过期、RTOS 资源耗尽和 MCU watchdog reset 均需测试或明确列为硬件安全分析项。

## 4. Companion 集成规范

### 1. 定位

Companion是可选外部计算平台，可运行VIO、SLAM、感知、建图、大范围路径搜索和轨迹候选优化。其结果仅作为建议输入，最终路径/轨迹必须由H723本机M5核对既有授权、时间参数化和验证；Companion不是M3/M4/M5/M6的替代品，也不是基础飞行安全依赖。

### 2. 三类接口

1. 外部里程计：M1 校验后形成 ICD `ExternalOdometry`，送 M3；M3 独立决定是否融合。
2. 路径/障碍候选：M1 校验后送 M5 的 Product Contract 适配接口；M5 检查来源、时间、坐标、epoch、车辆运动约束和安全边界。
3. 操作请求：M1 转为 `OperationRequest`，送 M4；M4 仲裁权限并决定是否接受。

### 3. 时序和坐标

所有外部数据必须携带 source ID、sample time、clock domain、mapping epoch、time uncertainty、frame/origin、reset counter、validity 和有效期。未经时间/坐标验证的数据不得进入导航融合或轨迹生成。

### 4. 权限边界

Companion 不得直接发布 M6 ActuatorDemand，不得直接写 M7，不得绕过 M4 切换操作状态，不得向 M3 写滤波器内部变量。Companion 断电/失联时，H723 必须按产品安全策略独立运行或进入安全降级。

### Companion 失联与权限补充规则

Companion的地图/障碍/路径建议不是飞手接管，也不是FMS命令，更不授予Companion任务权限。M1校验来源、时间、frame、origin、reset counter和TTL；M4只决定是否接受任务请求/建议作为任务输入；M5在H723本机完成授权核对、路径/轨迹生成和验证。Companion heartbeat/数据超时后，M5丢弃过期建议并继续本机Guidance；若本机能力不足，M4再选择保持/返航/降落。基础飞行不得依赖Companion在线。

## 5. 故障证据分类与场景决策

### 1. 目的

本文件用于避免把所有异常都归因于传感器故障。诊断必须区分“观测到了什么”与“推测原因是什么”。模块先报告事实和证据，Sensor Integrity Manager 只负责传感器源级状态，M4 才作操作级响应。

### 2. 诊断流程

```text
原始硬件/总线/时间证据
        |
        v
M2: 样本是否完整、及时、在量程内、校准版本正确？
        |
        +-- 否 --> 标记样本 INVALID/DEGRADED；发布 SensorDiagnostic
        |
        v
M3: 当前观测与预测/其他独立观测是否一致？
        |
        +-- 单次不一致 --> 拒绝当前观测；保留源级状态；记录残差证据
        |
        v
跨源/共因/运动/控制/执行器证据关联
        |
        +-- 直接设备故障或持续独立佐证 --> FDIR 可隔离源
        +-- 多源支持真实运动 + 控制响应对应 --> 外部扰动/运动异常候选
        +-- 模型残差持续且传感器本体健康 --> 模型失配候选
        +-- 证据不足 --> UNKNOWN/SUSPECT，不武断定因
        |
        v
M3 更新 NavigationCapability；M6 更新 ControlCapability；M7 更新 ActuatorCapability
        |
        v
M4 根据车辆阶段/能源/围栏/任务/能力选择操作响应
```

### 3. 证据强度

- DIRECT：设备自检失败、总线断开、CRC 连续失败、时间倒退、数据冻结等直接可验证证据。
- CORROBORATING：独立传感器、执行器反馈、控制输出、环境感知等提供佐证。
- MODEL_BASED：innovation/NIS、预测残差、动力学模型偏差；本身不能定因。
- UNKNOWN：证据冲突、共因未排除或观测条件不足。

不要把统计 NIS 结果当作物理因果标签。阈值必须按噪声模型、自由度、采样率、相关性和延迟校准。

### 4. 场景矩阵

| 场景 | 主要证据 | 初步分类 | 责任模块 | 操作决策边界 |
|---|---|---|---|---|
| IMU 数据冻结/序号停止 | 序号不增、重复样本、FIFO/驱动状态 | SAMPLE_INTEGRITY / DEVICE | Driver、M2、FDIR | M3 降低/失去姿态能力；M4 选安全处置 |
| IMU 饱和/削顶 | 量程标志、连续满量程、时间段 | SAMPLE_DEGRADED；也可能是真实冲击 | M2 + M3 + FDIR | 不凭单个峰值永久隔离；判断是否冲击/超量程 |
| IMU 高振动 | 高频能量/频谱/电机转速相关 | 运行环境/安装/机械异常候选 | M2、M6、M7 | 降低估计/控制信任，检查机械/电机，不等同传感器损坏 |
| GNSS 单次位置跳变 | NIS 高、IMU/其他源不支持 | MEASUREMENT_INCONSISTENCY | M3 | 拒绝该次观测；连续证据后 FDIR 可 SUSPECT |
| GNSS C/N0/AGC/接收机干扰标志异常 | 接收机直接状态 + 多源冲突 | GNSS_INTERFERENCE/SPOOF_SUSPECTED | M1/M2、M3、FDIR | 降低/拒绝 GNSS；M4 评估无 GNSS 导航能力 |
| 磁力计受电流干扰 | 磁场强度与电机电流相关、航向残差 | MAG_INTERFERENCE_SUSPECTED | M2/M3/FDIR | 航向能力降级，不自动宣布位置失效 |
| 气压计 propwash/动态压力 | 高度残差与油门/气流相关，其他高度源正常 | BARO_MEASUREMENT_DEGRADED | M2/M3/FDIR | 降低气压高度权重；评估 range/GNSS 替代 |
| 测距仪失效/反射异常 | 无回波、角度/材质/距离超域 | RANGE_SAMPLE_INVALID | M2/M3 | 仅影响对应高度/地形能力 |
| 光流低纹理/模糊 | quality 低、曝光/纹理条件差 | FLOW_OUT_OF_DOMAIN | M2/M3 | 不隔离设备本体；停止依赖光流的能力 |
| VIO reset/relocalization | reset counter、origin/frame 变化 | STATE_RESET / FRAME_CHANGE | M1/M2/M3 | 必须处理 reset epoch；禁止把跳变当作真实瞬移 |
| 突发大风/阵风 | 多源共同支持加速度/速度变化、控制输出增加、执行器正常 | UNEXPECTED_MOTION / DISTURBANCE_SUSPECTED | M3/M6/M7/HEALTH | 限制任务/速度/倾角或降落由 M4 决定；不隔离健康传感器 |
| 鸟撞/碰撞疑似 | 高 jerk/冲击、振动、姿态突变、执行器响应变化 | IMPACT_SUSPECTED（非确定物种/原因） | M2/M3/M6/M7 | 检查结构/执行器能力；必要时退出任务/降落 |
| 真实快速机动 | 多源一致、指令和动力学相符 | EXPECTED/UNEXPECTED MANEUVER | M3/M6 | 不误判传感器故障；模型应覆盖适用域 |
| MPC 跟踪误差持续增大 | 参考/状态误差、控制输出接近约束 | CONTROL_LIMITATION/MODEL_MISMATCH | M6 | 切备用控制或限制机动；M4 决策模式 |
| 电机输出饱和 | 需求超限、推力/RPM/电流反馈 | ACTUATOR_LIMITATION | M6/M7 | 重新评估可控自由度和安全包线 |
| 电机/ESC 无响应 | 指令与 RPM/电流/反馈不符、设备故障码 | ACTUATOR_FAULT | M7 | M4 按剩余控制能力决定；不假设必可继续飞行 |
| GNSS 与 VIO 同时漂移 | 共用时间/外部视觉/环境遮挡等 | COMMON_CAUSE_OR_UNKNOWN | M1/M2/M3 | 不将二者视作独立冗余；导航能力按保守方式评估 |
| 多传感器时间跳变 | 同时 sample age/epoch 错误 | TIME_BASE_FAULT | PLATFORM/M2/M3 | 暂停受影响融合；时钟恢复后重新对齐 |
| DMA overflow | buffer watermark/overrun counter | DATA_PATH_FAULT | Driver/PLATFORM/M2 | 丢弃不完整批次、计数并降级，不伪造样本 |

### 5. 真实扰动与传感器故障的判断约束

- 多源一致必须考虑独立性、共同电源/时钟/天线、安装杆臂、时间延迟、相关噪声和共同软件路径。
- 传感器与运动模型残差同时增大时，先检查模型适用域、时间同步、执行器动态和真实机动，不直接隔离传感器。
- 若只有一个传感器异常，其他独立源和物理响应一致，则该源更可疑；仍需持续性/直接证据。
- 如果没有足够观测，结论应是 UNKNOWN/SUSPECT，而不是强制二选一。
- 风、鸟、碰撞等具体物理原因通常需要外部感知或专用传感器；基础飞控只报告异常运动/冲击证据。

### 6. M4 响应矩阵

M4 选择动作前至少检查：当前操作模式/阶段、导航能力、控制能力、执行器能力、能源余量、Home/路径有效性、地理围栏、环境风险和当前权限。

- 单源退化但任务所需能力仍满足：可继续当前任务但限制能力/告警。
- 任务所需能力不满足但 RTL/安全返回可行：中止任务并返回。
- RTL 不可行但局部保持可行：保持/等待并持续评估能源。
- 导航能力不足：不得输出依赖无效位置的返航参考；采用车辆专用降级/着陆/停车策略。
- 控制/执行器能力不足：按可控自由度和危险分析选择降级/终止任务。

### 7. 诊断输出审计

每条证据必须包含 source_id、sample interval、time quality、evidence class、reason code、持续时间/计数、相关传感器/控制/执行器来源、配置集 hash。不得只记录“sensor bad”或“innovation high”。

## 6. 系统级故障遏制与残余风险

### 1. 目的

软件架构必须说明能处理什么、不能处理什么。不能把单 MCU 软件包装成具备硬件冗余的系统。此文件列出跨模块故障遏制和必须通过硬件/危险分析解决的残余风险。

### 2. 故障遏制边界

| 故障 | 软件可做 | 软件不能保证 |
|---|---|---|
| 单个辅助传感器异常 | 样本拒绝、源隔离、能力降级、FMS 处置 | 没有替代源时维持原导航精度 |
| IMU 单路异常且有独立冗余 IMU | 交叉检查、隔离、切换/重初始化 | 共电源/共时钟/共安装造成的共因失效被完全消除 |
| 所有姿态 IMU 失效 | 检测、告警、产品紧急策略 | 单 IMU 配置下继续稳定飞行 |
| GNSS 干扰/欺骗 | 识别可疑证据、拒绝/降权、使用其他导航源 | 仅靠单天线/单接收机可靠识别所有欺骗方式 |
| RC/GCS 链路丢失 | 本机 failsafe 策略、继续/返航/停车/靠泊 | 失联后仍能接受人工新命令 |
| Companion失联 | 丢弃外部建议并继续H723本机M5；若本机导航/规划能力不足由M4选择安全模式 | 在无本机感知能力时声称完成在线复杂避障 |
| MPC 超时/不可行 | 限时求解、拒绝旧解、切换已验证备用控制 | 备用控制未经验证仍能保证安全 |
| M4 FMS 任务故障 | watchdog、保持最后安全 directive 的有限策略、事件记录 | 单独靠 M7 恢复任务决策 |
| M6 主控制器完全失效 | 检测、执行预定义的产品级输出策略 | M7 仅凭电机输出接口恢复姿态稳定 |
| MCU 复位/掉电 | 硬件 watchdog、上电状态安全门控、日志保留 | 单 MCU 复位期间维持闭环控制 |
| 电机/ESC 物理失效 | 反馈诊断、剩余控制能力评估、降级动作 | 任何四旋翼单电机失效后都可安全继续飞行 |
| 机体结构损伤/鸟撞 | 检测冲击/振动/控制能力异常，触发安全策略 | 仅靠软件恢复结构完整性 |

### 3. M4/M6/M7 失效行为

- M4 失效：Directive/Permit 过期；系统必须有明确的 lease 语义。不得简单规定空中 Permit 过期即电机归零。可在有限 grace window 内沿用最后一个已验证控制模式，随后进入产品专用安全行为；该窗口和行为由危险分析及 HIL 决定。
- M6 失效：M7 无法生成稳定姿态/速度控制。M7 只负责输出边界和设备安全，不能冒充备用飞控。若产品要求 M6 单点故障后仍可控，需要独立备用控制通道/MCU/执行器路径，属于硬件架构升级。
- M7/驱动失效：输出刷新丢失、DMA/Timer 停止或 ESC 链路异常需要独立监测。实际安全动作与电机/舵机/推进器危险分析相关。
- MCU reset：飞行中复位是严重单点故障。硬件看门狗不能保证安全着陆；必须在系统安全分析中作为残余风险明确记录。

### 4. 安全输出不得通用化

`output_zero` 对地面车辆可能是安全停车，对空中多旋翼可能是立即坠落；`hold_last` 对某些推进系统可能持续危险输出。M7 的 stale command、permit expiry、driver failure、watchdog reset 分别定义策略，并由产品专用配置和危险分析验证。

### 5. 建议的硬件安全增强（非首版默认实现）

- 关键传感器独立供电/时钟/总线/安装，避免共因故障。
- 双 IMU 或独立姿态监测通道；需要时评估独立备用控制器。
- 独立电源监测、硬件电流/电压保护和 ESC telemetry。
- 输出定时器/看门狗的独立安全路径。
- 对高风险运行环境增加外部感知和 Companion 冗余，但基础稳定控制不能依赖 Companion。

### 6. 安全放行条件

架构文档完成不等于飞行放行。必须完成 FHA/FMEA/FTA 或适合项目等级的危险分析，验证每个故障的检测时间、隔离时间、控制响应、残余风险和操作限制。高严重度风险没有可验证缓解措施时不得进入载桨飞行。

---

## 7. QUAD_X4 FMS五正交状态、权限和通信安全（唯一规范）

### 7.1 M4 FMS八个逻辑子模块（单一状态所有者）

| 子模块 | 唯一职责 | 输出/写权限 |
|---|---|---|
| M4-01 MissionManager | 任务上传/验证/提交、当前任务项、任务进度、暂停/恢复/中止/完成 | 唯一写入MissionState/MissionStatus |
| M4-02 ModeManager | FlightMode合法状态转换、模式请求校验 | 唯一写入FlightMode |
| M4-03 AuthorityManager | RC/GCS/Companion来源优先级、接管/释放、权限锁存 | 唯一写入AuthorityState；Companion仅ADVISORY |
| M4-04 HomeManager | Home采集、有效性、版本、返航目标快照 | 唯一写入HomeSnapshot |
| M4-05 GeofenceManager | 围栏配置/版本/hash、位置和轨迹约束结论 | 唯一发布ProductGeofenceSnapshot/GeofenceStatus |
| M4-06 PreArmManager | 解锁前健康、传感器、定位、执行器和配置检查 | 唯一发布PreArmStatus |
| M4-07 LandDetector | 基于导航/高度/垂向速度/执行器证据发布LandedStateSnapshot | 唯一发布LandedStateSnapshot；不直接控制电机 |
| M4-08 FailsafePermitManager | 故障分级、FailsafeLevel、SafetyEnvelope、ActuationPermit和终态策略 | 唯一写入FailsafeLevel/SafetyEnvelope/ActuationPermit |

M4内部子模块由一个串行状态更新入口协调；其他模块只能通过02定义的快照/事件请求，不得直接写M4状态。 M4是MissionStatus唯一写入者；MissionStatus通过FmsOutputView输出，授权当前项带mission_epoch/execution_token，M4核对到达条件后推进。M5只报告GuidanceStatus/GuidanceFeasibility，不能推进任务项。M5不得写MissionStatus，M6不得写FlightMode/Authority，M7不得自行选择返航/降落模式。


### 1. M4唯一状态模型

M4 QUAD Policy使用五个正交状态域。`PREARM_CHECK`是派生检查流程，不是FlightMode；`TERMINATED`不作为与Arm/Mode并列的万能状态。

| 状态域 | 唯一Owner | 合法值 |
|---|---|---|
| `ArmState` | M4 | DISARMED、ARMING、ARMED、DISARMING |
| `FlightMode` | M4 | NONE（DISARMED/未授权）、MANUAL_RATE、MANUAL_ATTITUDE、ALT_HOLD、POS_HOLD、AUTO_MISSION、AUTO_RETURN、AUTO_LAND、EMERGENCY_LAND |
| `Authority` | M4 | NONE、PILOT、MISSION、GCS_SUPERVISED |
| `FailsafeLevel` | M4 | NORMAL、DEGRADED、ACTION_REQUIRED、TERMINAL |
| `MissionState` | M4 | IDLE、LOADED、RUNNING、PAUSED、ABORTED、COMPLETED |

`PreArmCheckResult`输出聚合mask/首个拒绝原因，逐项结果经FCU_PREARM_STATUS发布。Authority表示普通操作权归属；SAFETY不是Authority枚举值，而是对所有普通请求生效的强制约束。Companion永远是ADVISORY，不拥有飞行Authority。

M4内部LandDetector唯一发布02定义的`LandedStateSnapshot`，FMS状态机消费。合法值为LANDED/IN_AIR/TAKEOFF/LANDING/UNKNOWN；证据不足必须UNKNOWN。不得仅凭低油门、短时零速度或单一高度读数判定LANDED。普通Disarm和AUTO_LAND完成必须要求LANDED及稳定dwell；实机地面接触证据未提供时固定UNKNOWN，相关自动完成/普通Disarm不成立。Host/SIL使用显式fixture：ground_contact=CONTACT、|v_D|≤0.15m/s、|a_D|≤0.5m/s²连续2s才LANDED；|v_D|>0.3m/s或ground_contact=NO_CONTACT持续200ms即IN_AIR；其余UNKNOWN。该fixture不导入实机Profile。该子模块只消费既有NavigationSnapshot/ActuatorStatus等外部接口，不修改INS内部实现。

M4是Geofence唯一Owner；M5只读消费`ProductGeofenceSnapshot`。围栏Revision/Hash、frame/origin/state_epoch、TTL、顶点数、水平多边形、高度上下界和安全缓冲必须有效。围栏过期/坐标不匹配/几何无效时，依赖围栏的Mission拒绝启动；Companion只能提交候选，不能修改或绕过围栏。首版围栏由编译期`ProductGeofenceConfig`（Product Manifest/Profile生成的`.rodata`）提供：Composition Root在`fms_policy_init`前调用产品专用安装钩子`fms_configure_product_geofence(const ProductGeofenceConfig*)`（不是公共Port，不允许运行时/Companion调用），M4在`fms_policy_init`校验`profile_hash/config_version/geofence_hash`、NED frame、顶点数、几何、高度上下界和margin后唯一持有；不新增运行时`SET_GEOFENCE`命令，`OperationRequest.command_code`不分配围栏写入口。围栏变更必须改Profile并重编译，按CR/Profile hash流程处理。


### 2. 状态/权限唯一职责

- M1只报告RC/GCS/Companion链路事实和原始OperationRequest/OperatorInput，不授予权限、不选择RTL。
- M4唯一生产AuthorityStatus、AuthorizedOperatorInput、VehicleDirective和ActuationPermit；唯一决定FlightMode/Failsafe/MissionState。
- M5只执行M4授权任务并生成MotionReference；不授予权限、不改变飞行模式。
- M6只消费有效NavigationSnapshot、MotionReference、VehicleDirective和ActuatorCapability，输出物理ActuatorDemand；不决定返航/降落。
- M7只做Permit/epoch/TTL/执行器范围/变化率复核、协议输出和本地故障保护；不做控制分配。
- 硬件电气Kill若真实存在属于独立硬件安全链；当前项目尚无已核验的原理图证据，软件不得宣称已具备物理Kill/BKIN。

权限/安全仲裁顺序：已验证的硬件电气保护（若硬件确实存在）→ M7本地输出边界保护 → M4强制Failsafe/SafetyEnvelope → 导航/姿态/执行器能力门控 → 明确Pilot takeover → 已授权Mission → GCS任务请求 → Companion建议。RC恢复不自动接管；人工接管默认锁存，摇杆回中不自动恢复AUTO。GCS心跳不能掩盖RC丢失，RC有效也不自动授予GCS控制权。

### 3. 状态转换约束

1. DISARMED收到有效Arm请求后进入ARMING；只有PreArmCheck全部必需项通过且M7报告输出就绪，才进入ARMED。失败保持DISARMED并返回具体拒绝原因。
2. FlightMode只在当前ArmState、导航能力、执行器能力和SafetyEnvelope允许时切换；POS_HOLD/AUTO_MISSION必须有有效位置/速度能力与匹配Origin；AUTO_RETURN另要求Home有效，只有引用相对Home高度的普通任务才额外要求Home。
3. AUTO_RETURN期间RC恢复但没有明确接管动作时继续AUTO_RETURN；明确接管后M4递增authority_epoch并进入对应人工模式。
4. Pilot接管后保持锁存；摇杆回中不自动恢复任务。再次进入AUTO必须有新的显式请求、重新校验任务/轨迹/Home/围栏和Epoch。
5. RC失联由接收机failsafe或QUAD Profile唯一参数`rc_lost_timeout_us`触发；该参数必须依据实际接收机帧周期、漏帧行为和failsafe输出实测确定，未取得证据前为`SOURCE_EVIDENCE_PENDING`，不得把300ms冻结为飞行阈值。Flight.xlsx与05已统一名称ELRS；准确接收机型号/串口帧协议仍待BOM和抓包确认。GCS/Companion丢失不自动改变模式，除非当前任务明确依赖该链路并由M4策略决定。
6. FailsafeLevel与FlightMode分开更新；TERMINAL进入后锁存，只有落地/解锁状态满足维护清除条件并通过完整健康检查后才允许清除；不得在空中自动回到NORMAL。TERMINAL不自动等于电机归零。四旋翼空中输出策略必须由危险分析定义，禁止通用Permit过期即关闭电机。

#### 3.1 五状态域最小迁移表

| 状态域 | 当前状态 | 事件/守卫 | 新状态 | 强制副作用 |
|---|---|---|---|---|
| ArmState | DISARMED | ArmRequest且PreArm检查启动 | ARMING | 锁存请求来源；输出仍保持未解锁 |
| ArmState | ARMING | 所有必需检查通过且M7 ready | ARMED | 递增相关决策序号；启用已授权控制链 |
| ArmState | ARMING | 任一必需检查失败/超时 | DISARMED | 返回原因码；不部分解锁 |
| ArmState | ARMED | DisarmRequest或策略要求地面停机 | DISARMING | 进入受控停机流程；不绕过M7 |
| ArmState | DISARMING | 输出状态确认安全停机 | DISARMED | 清除普通Authority和活动Permit |
| Authority | NONE/MISSION/GCS_SUPERVISED | 明确有效Pilot takeover动作且RC新鲜 | PILOT | M4递增authority_epoch；任务按策略暂停/中止 |
| Authority | PILOT | 摇杆回中/RC恢复 | PILOT | 保持接管锁存，不自动切回AUTO |
| Authority | PILOT | 显式释放人工控制且任务重新校验通过 | MISSION或NONE | M4递增authority_epoch；新Directive/Permit |
| MissionState | IDLE | 已验证Mission原子提交 | LOADED | 保存mission_id/hash/profile hash |
| MissionState | LOADED | 显式Start且M4授权 | RUNNING | Authority=MISSION（若无Pilot takeover） |
| MissionState | RUNNING | Pause请求/可恢复故障 | PAUSED | 冻结任务进度，轨迹按TTL失效策略处理 |
| MissionState | PAUSED | Resume且全量重验通过 | RUNNING | 新Directive/reference epoch |
| MissionState | RUNNING/PAUSED | Abort/Clear/不可恢复任务错误 | ABORTED/IDLE | 清除活动任务引用，禁止复用旧轨迹 |
| FailsafeLevel | NORMAL | 可降级故障/单源异常 | DEGRADED | 发布健康原因和能力降级 |
| FailsafeLevel | DEGRADED | 需要M4主动动作 | ACTION_REQUIRED | M4选择返航/降落/人工接管等策略 |
| FailsafeLevel | 任意非TERMINAL | 不可恢复安全条件 | TERMINAL | 按产品安全包线执行；不等价于电机立即归零 |

所有状态迁移在M4单线程/事件序列化上下文内提交。

参考/Permit切换规则唯一引用02：M5候选READY足以供M4签发匹配Permit，M6在READY+匹配Permit下可执行，M5随后确认ACTIVE，不能互相等待ACTIVE。普通重规划可维持原有效bundle至新bundle完整提交；安全撤销立即发布，新旧epoch不得混用。

首次地面解锁可先签IDLE_ONLY scope的短lease（reference_epoch=0），M7只执行已验证地面arming-idle策略，不接受M6控制Demand。收到候选READY且全套条件有效后再签CONTROL_ACTIVE scope、非零reference_epoch的Permit；Takeoff/运动任务必须等该链路建立。每次IDLE_ONLY lease≤25ms，仅凭新鲜地面证据续租；自ARMING请求起绝对100ms内未建立CONTROL_ACTIVE或失去LANDED证据，拒绝运动并按地面受控Disarm/产品策略处理，不无限续租IDLE_ONLY。空中模式切换先准备可用参考再提交普通模式；必须立即撤销的安全事件优先撤销，随后执行已批准SAFE_ACTION，不能用等待新参考来延长旧权限。

首次地面握手中，M6在收到首个有效CONTROL_ACTIVE后才能产出有效ControlCapability。M4在地面且尚未建立控制链路时允许等待首帧，仍执行首参考超时和执行器故障检查。仿真策略主动撤销旧运动参考时，允许在两个许可周期内等待匹配的新参考及控制能力；此间 M7 拒绝旧 Demand。缺失控制能力在等待窗口结束后触发终止。显式报告的控制故障在等待期间同样触发终止。

当前四旋翼的 SIM_CONTROLLED_STOP 配置使用 1.5 kg 质量和 30 N 总推力上限，使重力补偿落在 MPC 的保守输入域内。此推力上限只用于仿真策略。

PC 端 FMT-SIM 联机走通信层的仿真适配。HIL 模式通过回环 UDP 接收 `FMT_EXTERNAL_STATE`，由 Host 测试入口转换为合成传感器样本，估计器仍按原路径运行。上行仅将 M7 实际提交的四路虚拟电机值转换为 `HIL_ACTUATOR_CONTROLS`。过期状态停止供样并发送虚拟停机。SIH 模式单独接收 `HIL_STATE` 位姿用于显示。

对外先发布一致的状态快照，再发布新Directive/Permit。状态快照包含同一boot_id和对应Epoch，禁止多个模块分别维护“当前飞行模式”副本。

### 4. Failsafe 场景决策矩阵（当前规范）

#### 4.1 适用范围

本文件定义 QUAD_X4 首版 Aerial FMS 的场景策略原则。具体阈值和动作由 Aerial FMS Policy、接收机 Link Supervision 契约及车辆能源/能力安全分析确定；这些是受控安全策略，不是普通工程调参项。表中“建议动作”是决策顺序，不替代实测危险分析。

#### 4.2 RC / 操作员场景

| ID | 场景 | 必须检查 | 默认策略 | 禁止行为 |
|---|---|---|---|---|
| RC-01 | MANUAL/ASSISTED 中 RC 失联 | 接收机 failsafe、帧时效、导航能力、Home、能源、路径、围栏 | 满足返航条件则 RTL；否则按剩余能力定高/姿态稳定/受控降落 | 无位置能力时盲目生成 RTL |
| RC-02 | AUTO_MISSION 中 RC 失联 | 任务是否允许无人监督、Home、能源、围栏、障碍物 | 默认暂停/中止任务并评估 RTL；只有明确配置且风险证明允许才继续任务 | 默认无限继续任务 |
| RC-03 | RTL 中 RC 仍丢失 | 当前状态、返航可行性、能源、执行器能力 | 继续 RTL；条件变化时从当前位置重新规划 | 复用失效旧轨迹 |
| RC-04 | RTL 中 RC 恢复、摇杆中位、模式开关仍 AUTO/RTL | Link 恢复稳定、输入新鲜、模式开关 | 继续 RTL；不恢复 Pilot Authority | 把 link recovered 当 takeover |
| RC-05 | RTL 中 RC 恢复、模式开关明确切到 Pilot/Assisted | 链路稳定、导航能力、能源/围栏/阶段、安全接管允许 | M4 显式授权，增加 authority_epoch，进入所选人工/辅助模式 | M1 或 M5 直接接管 |
| RC-06 | 接管后摇杆回中 | 当前 Pilot 模式、导航能力、模式开关 | 默认保持 LATCHED Pilot Authority；中位按当前模式解释 | 自动恢复 RTL/原任务 |
| RC-07 | 摇杆接管手势抖动/噪声 | deadband、dwell、帧质量、开关状态 | 未满足连续确认不接管；使用滞回 | 单帧偏移触发接管 |
| RC-08 | Pilot 接管后再次 RC 失联 | 当前人工/辅助模式、当前位置、能源、Home、导航能力 | 重新执行 RC Loss Policy，从当前状态决策 | 无条件重放最初 RTL 轨迹 |
| RC-09 | RC 链路恢复但无人工接管直到到达 Home | RTL 轨迹和终点可行性、能源、着陆能力 | 继续 RTL；终点按 LAND_IF_VALID / 有限保持 / 替代降落策略 | 因等待超时切回手动或无限悬停 |
| RC-10 | Pilot 接管时电量已 critical | 能源、着陆点、围栏、控制能力 | 允许飞手在安全边界内选择更合适降落点，但不得取消强制能源安全动作 | 允许普通手动命令无限延迟迫降 |

#### 4.3 GCS / Companion 场景

| ID | 场景 | 默认策略 |
|---|---|---|
| COM-01 | GCS 丢失、RC 正常 | 不触发 RC Loss；拒绝过期 GCS 命令，当前模式按监督依赖策略继续/中止 |
| COM-02 | RC 丢失、GCS 正常 | 仍按 RC Loss Policy；GCS 在线不自动成为 Pilot |
| COM-03 | RC/GCS 同时丢失 | 按 RC Loss Policy，同时停止依赖实时地面监督的任务 |
| COM-04 | Companion heartbeat丢失 | 立即使外部地图/障碍/路径建议过期；H723本机Guidance继续，必要时由M4选择安全模式 |
| COM-05 | Companion数据恢复但reset_counter/origin改变 | 不续用旧建议；先处理state reset/origin变更，再由H723 M5重新验证/规划 |
| COM-06 | GCS 命令重放/过期/未认证 | M1 拒绝，M4 不改变状态，记录审计事件 |

#### 4.4 导航能力场景（仅消费INS外部能力接口）

| ID | 场景 | 能力变化 | FMS 策略原则 |
|---|---|---|---|
| NAV-01 | GNSS 丢失，其他位置源可用 | GNSS source degraded；位置能力按其他源重新评估 | 任务可行则限能力继续，否则 RTL/降落 |
| NAV-02 | GNSS 疑似 spoof/jamming | GNSS 位置/速度可信度降低或拒绝 | 不使用可疑 GNSS 位置执行 RTL；检查其他来源、Home 和围栏 |
| NAV-03 | 磁力计受干扰 | 航向能力退化，其他能力可能保持 | 不将其等同于全导航失效；限制依赖绝对航向的任务 |
| NAV-04 | 气压计异常 | 高度能力降级 | 评估 range/GNSS 等替代；不直接判定位置失效 |
| NAV-05 | 单个 IMU 故障且冗余 IMU 可用 | 估计器切换/重初始化，短时能力退化 | 降低机动、检查姿态解稳定性；必要时降落 |
| NAV-06 | 所有姿态 IMU 失效 | 姿态/角速度估计无效 | 进入产品紧急处置；软件不承诺继续稳定飞行 |
| NAV-07 | VIO reset/重定位 | state_epoch/origin 改变 | 暂停依赖旧轨迹的控制，重新建立参考；FMS 评估是否继续任务 |
| NAV-08 | 单次 GNSS innovation 超限 | 当前观测拒绝 | 不直接永久隔离 GNSS；等待多证据判断 |

#### 4.5 运动/控制/执行器场景

| ID | 场景 | 默认策略原则 |
|---|---|---|
| DYN-01 | 突发阵风，传感器本体健康，多源支持真实运动 | M6 尽力恢复跟踪并报告余量；M4 可限速/限倾角/退出任务/降落，不隔离健康传感器 |
| DYN-02 | 碰撞/鸟撞疑似 | 标记 IMPACT_SUSPECTED；检查结构振动、IMU 饱和、执行器反馈和控制能力；必要时尽快降落 |
| DYN-03 | 持续模型失配 | 限制模型适用域外机动，切换经过验证的控制模型/备用控制；M4 决定任务降级 |
| DYN-04 | MPC timeout/infeasible | 拒绝本次结果，切已验证 fallback；不无限期沿用旧 MPC 输出 |
| DYN-05 | 电机输出饱和但反馈正常 | 识别 CONTROL_LIMITATION；降低轨迹要求/速度/倾角或退出任务 |
| DYN-06 | 电机/ESC响应异常 | 当前Profile反馈能力为NONE时只能报告响应可疑/不可观测；不得直接确认单电机失速。具备独立反馈且通过验证后，才可升级为故障确认 |
| DYN-07 | 电池低 | 告警、限制任务/返航评估，按能源模型决策 |
| DYN-08 | 电池 critical | 进入强制能源策略；飞手仍可在安全边界内改变降落地点，但不能取消安全约束 |
| DYN-09 | 地理围栏接近/越界 | M4 Safety Policy 限制/拒绝冲突指令，M5 生成边界内可行轨迹 |

#### 4.6 恢复和解除 failsafe

- `LINK_RECOVERED` 只更新链路事实，不自动清除 `FAILSAFE_ACTIVE`。
- `SENSOR_RECOVERED` 需要源级 probation，不自动恢复旧导航状态/旧任务。
- `NAV_VALID` 恢复后，M4 仍需重新检查 Home、任务、轨迹、能源、围栏和控制能力。
- Failsafe 清除必须由 M4 产生 DecisionEvent，说明触发原因已消失、能力满足和新模式为何安全。
- 自动任务恢复必须新建 directive/authority epoch；旧轨迹不得自动复活。
- 当前 QUAD_X4 策略在解锁期间锁存已确认的 `ENERGY_CRITICAL`。电量回弹、能源报文过期或与 RC 丢失叠加时，继续强制降落；自动降落已经可行时，单纯 RC 丢失也不切换到返航。当前策略尚未提供安全边界内的迫降点人工调整，普通运动命令与模式开关不能取消迫降。只有已验证接地并上锁后才可在新的解锁周期重新评估能源。已锁存的 TERMINAL 在满足维护清除条件前拒绝 ARM。
- M5 对当前返航指令的明确不可行结果触发降级。M4 校验 authority/directive/state/origin 和 reference 对应关系，转入具备导航能力的降落策略。旧指令的不可行结果不能影响当前返航；失联持续时，避免重复选择已拒绝的返航。
- ARM 使用上一份已发布的预检 epoch，同时重新检查本周期输入。新发生的能源、链路、导航或执行器故障按当前必需项拒绝请求，严重 FaultSummary 同样拒绝。落地/返航操作离开对应模式时，结束被中断的命令跟踪并报告取消。
- Host/SIL 已建立控制后的受控模式切换立即撤销旧许可。最多等待两个许可周期以建立新参考与控制能力，重复切换不延长窗口。不可控、执行器故障、时间/状态 epoch 变化仍按各自的保护判据处理。M7 在等待期间逐帧执行原有许可校验。实机输出策略按目标证据另行验证。

#### 4.7 验证

每个场景都应有正常路径、边界路径和失败路径；验证 mode/authority epoch、消息有效期、轨迹/current_item/execution_token epoch、参数hash、状态转移、告警/日志、执行器输出和最终能力。没有实测依据的阈值保留 TBD，不能在测试中临时调到“通过”。

### 5. 第一版通信拓扑和端点

- RC：USART3/115200，独立输入M1 RC Adapter，不经过MAVProxy/MAVSDK；实际接收机型号/协议以BOM和抓包确认为准。
- FCU数传：USART2/MAVLink 2/115200，接入M1 MAVLink Gateway。
- Companion临时占位：USART1/MAVLink 2/115200，与Debug/FinSH互斥；Companion断开不影响H723本机M5基础功能。
- MAVSDK：地面端或Companion上的高层客户端，不是FCU内部模块。
- MAVProxy：地面/Companion主机侧MAVLink路由器，负责多Master/多Out/UDP fan-out；不进入飞控实时闭环，不拥有Authority。

推荐地面侧：`FCU USART2 ↔ 数传链路 ↔ MAVProxy Router ↔ QGC / MAVSDK Application / Recorder`。如果QGC或MAVSDK直连FCU，则作为独立Gateway endpoint登记；禁止MAVProxy形成回环、重复转发风暴或把地面路由状态当作飞行权限。

MAVProxy路由配置必须保持有向无环：输入master不得作为自己的output回送；两个Router不得互相全量转发形成环；QGC、MAVSDK、Recorder使用不同的UDP endpoint/端口并在部署配置中登记。Router只负责转发和路由状态，不做命令授权；FCU Gateway按source system/component白名单和命令策略最终过滤。冗余master切换必须避免同一命令被两条路径重复执行，非幂等操作使用OperationRequest ID/mission transaction ID去重。

### 6. PreArm Check ID 与状态绑定

检查表中CONTROL_MODEL同时检查MPC保守推力盒的悬停可行性T_min/m≤g≤T_max/(sqrt(3)*m)，不满足返回CONTROL_REASON_MPC_HOVER_OUTSIDE_APPROX_DOMAIN。基于NONE障碍能力的普通任务只在明确不要求避障时允许，要求避障的任务拒绝；当前没有目标/安全证据时不宣称Flight模式已开放。

PreArm Check ID 使用稳定注册表，不允许按位号临时重排：PLATFORM_READY、IMU_READY、ESTIMATOR_ATTITUDE、ESTIMATOR_POSITION（按模式要求）、RC_INPUT（按模式要求）、ACTUATOR_OUTPUT_READY、BATTERY_ENERGY、BARO_HEIGHT（按模式要求）、GNSS_HOME_ORIGIN（按模式要求）、COMPASS_HEADING（按模式要求）、CONTROL_MODEL、CONTROL_SOLVER、SAFETY_ENVELOPE、GEOFENCE（按模式要求）、STORAGE_EVENT_LOG（按安全策略）。每项结果包含PASS/FAIL/WARNING、reason_code、evidence timestamp、适用模式和恢复条件。Arm请求必须绑定最新check_epoch；过期PreArm结果不得用于解锁。SD卡不可用不自动阻止基础手动飞行；关键事件存储失效按产品安全策略降级。

`COMPASS_HEADING`为既有稳定PreArm Check ID，名称保留以兼容注册表；在KF-GINS_QUAD中其语义是“估计器航向能力满足当前模式门限”，不是要求BMM350必须在线。磁力计可按质量策略关闭；只有其他经过验证的航向来源仍满足`heading_std_rad`与新鲜度门限时，位置相关模式才可通过。

**KF-GINS_QUAD导航能力门控：** M4只根据02号`NavigationCapability`的能力mask/聚合标准差字段，以及`NavigationSnapshot`中公开的`attitude_std_rad[]`、`position_std_m[]`、`velocity_std_mps[]`、数据年龄、`origin_id/state_epoch`和Home有效性进行模式/任务门控；禁止读取22维协方差或KF-GINS私有状态。姿态稳定模式要求有效姿态/角速度；定高/垂向任务要求有效垂向能力；位置保持/航迹/位置RTL要求水平位置、速度、航向/姿态、Origin和不确定度全部满足QUAD_X4 Profile门限。BMM350磁航向辅助可以关闭，但关闭后必须由其他有效航向能力满足门限；单天线GNSS course不得被当作机体yaw。GNSS失锁后能力随协方差增长而退化，M4根据当前能力选择限制/保持/返航/降落，不得因INS仍在传播就认为位置有效。

Home 作为当前飞行的固定起飞点。已解锁时，M4 用新鲜且满足位置保持门限的导航证据续期，保持捕获位置、时间、标识与修订号。Origin 或 state epoch 变化立即使旧 Home 失效。连续缺少续期证据达到策略期限时 Home 过期，恢复导航后需重新授权捕获。

任务成功后在位置保持阶段收到普通返航请求，M4 接受并准备返航。当前保持参考需要匹配有效控制许可，且 M6 在许可生效后报告有效控制能力。导航三轴速度范数小于 0.15 m/s、加速度范数小于 0.4 m/s² 连续 1 s 后，M4 才撤销保持许可并切入返航。准备限时 6 s，超时报告失败并保持原模式。飞手接管、强制安全动作、Home 或模式失效取消准备。强制失联和能源动作按既有优先级执行。该判据使用飞控估计与能力，仿真真值仅用于验收。

接地确认需新鲜接地证据、有效水平及垂直速度和垂向加速度。水平速度不超过 0.20 m/s、垂直速度绝对值不超过 0.15 m/s、垂向加速度绝对值不超过 0.5 m/s² 持续满足原接地停留时间后确认 LANDED。证据丢失、反弹或滑移在当周期清除 LANDED 及停留时间，解除解锁和着陆完成均使用重新确认的状态。

平坦 Host/SIL Profile 对 FMT-SIM 帧间推力和接地震荡使用同一份运动证据。按有效导航的 State Time，使用三轴速度与其 250 ms 一阶滤波值之差除以时间常数，估算持续加速度。返航与接地沿用上述速度、加速度门限和停留时间。接地证据逐周期判断。原始加速度范数超过 6 m/s²、导航证据失效、时间回退、State Time 间隔超过 100 ms 或 boot/time/state/origin 变化时重置估计并清除稳定计时。连续积累至少 250 ms 新导航证据后才重新计时。实机策略继续使用原始判据，直到有接地及飞行数据支持独立整定。

普通地面站起飞、降落和返航请求与 SET_MODE 一样服从显式飞手接管锁存。命令去重同时比较命令码和载荷。长操作的进度和结束结果更新回执缓存，认证来源可用新鲜报文重取最近结果，原请求的执行时限约束首次执行。

**门限归属与判定规则：** 导航能力的数值门限由C维护的QUAD_X4 FMS Safety Policy定义，不由KF-GINS内部硬编码，也不放入`estimator_tuning.yaml`作为普通滤波调参项。Policy至少固定以下门限：姿态/航向/水平位置/垂向位置/水平速度/垂向速度最大标准差、NavigationSnapshot最大age、允许的Origin/Home状态、各能力的进入/退出滞回和dwell。每种模式分别声明必需能力：MANUAL_RATE要求有效角速度及该产品允许的姿态门控；MANUAL_ATTITUDE要求姿态/角速度；ALT_HOLD再要求垂向能力；POS_HOLD要求水平位置/速度/航向；AUTO_MISSION增加有效当前项token/GuidanceFeasibility/Reference；AUTO_RETURN增加Home/Origin和返航路径能力。AUTO_LAND/EMERGENCY_LAND还必须满足已批准的垂向/着陆证据策略，缺项拒绝该自动模式。未配置或未验证的门限按fail-closed处理：对应自动/位置模式不可用，但不阻止基础手动模式。门限值必须由数据集、SIL/HIL和受控飞行证据确定，不得由M3算法Agent自行放宽。

### 7. 命令处理和MAVSDK允许范围

所有外部命令统一走：`MAVLink2 parse → CRC/signature → source allowlist → target_system/component → replay/sequence → TTL → command allowlist/parameter range → OperationRequest → M4 policy → COMMAND_ACK`。

第一版允许的高层命令：Arm/Disarm、Takeoff、Land、RTL、白名单Set Flight Mode、Mission Upload/Start/Pause/Resume/Abort/Clear，以及受控参数Read/Write。Parameter Read仅开放公开白名单；Parameter Write仅开放逐项登记的非安全参数，必须DISARMED并具备维护授权，且只能进入唯一Parameter Manager事务。核心控制参数、ControlModel、执行器映射、安全包线和飞行安全策略不得通过普通MAVLink参数写入。Mission Upload必须先进入staging，具备Transaction ID、分片序号/完整性、超时、去重和容量上限；完成schema/profile/hash/坐标/Origin/Geofence/速度与高度约束检查后原子提交。M4将验证后的完整任务与当前项ID/index/执行token发布为带Header/TTL/mission_epoch/authority_epoch/directive_epoch/state_epoch的`AuthorizedMissionSnapshot`；M5不得接收无Header/无TTL的裸`MissionRequest`。上传成功不代表Mission自动启动。

第一版关闭未经M4授权的Offboard直接控制、MANUAL_CONTROL/GCS joystick与RC竞争控制、任意电机测试命令和任意参数内存写入。需要开放时必须新增权限CR和独立验收。

按[MAVLink Command Protocol](https://mavlink.io/en/services/command.html)：普通命令ACCEPTED表示接受并尝试执行；需要跟踪完成的长命令先IN_PROGRESS，最后ACCEPTED/FAILED/CANCELLED。其最终ACCEPTED有完成语义。ACCEPTED不等价于已起飞/已落地，具体完成条件见下表。ACK丢失重发须命中去重记录，仅回放结果，不重复副作用。

### 8. MAVLink 2签名与MAVProxy安全边界

MAVLink 2 signing提供来源认证/完整性和防重放能力，不提供报文加密。每台飞控必须使用独立密钥；密钥生命周期包括制造/维护环境安全注入、身份绑定、轮换、吊销、丢失/泄露响应和重新配对。密钥不得写入SD、W25Q64普通日志/参数区或遥测；H723的RDP/PCROP/Option Bytes保护能力必须以实际芯片和配置验证，不能证明时需安全元件或受控外部保护方案。重启后无法证明签名时间戳/防重放高水位连续性时必须fail-closed并只允许本地有线维护换新key/清对端旧会话；同key普通重连不能恢复可信性。当前C库/通道若只支持每个物理channel一个签名Key，则同一物理UART trust domain使用同一link key；USART1 Companion独立channel可使用独立key。若要求同一UART2上多个逻辑端点各自独立Key，必须实现并测试多Key选择/验证层，不能仅在文档中声明已支持。

MAVProxy只转发原始签名帧，不修改签名内容、不代替FCU重新签名。FCU Gateway必须校验Source/Target、签名、签名时间戳/防重放窗口和命令白名单；MAVLink packet sequence只用于链路丢包/乱序统计，不可单独作为防重放安全机制。密钥不写入SD、公开日志或普通W25Q64事件记录。签名时间戳恢复状态只保存必要的高水位/会话元数据；若掉电后无法证明时间戳连续性，远端命令必须fail-closed并只允许本地有线维护换新key/清对端旧会话；同key普通重连不能恢复可信性，不能接受旧重放包。

### 9. 地面站状态上报

标准MAVLink消息首版频率：

| 消息 | 频率/触发 | 主要内容 |
|---|---|---|
| HEARTBEAT | 1Hz | 系统类型、飞行器状态、模式 |
| SYS_STATUS | 1Hz | 系统/传感器健康摘要 |
| BATTERY_STATUS | 1Hz | 电压、电流、电量 |
| ATTITUDE_QUATERNION | 10Hz | 姿态四元数、角速度 |
| LOCAL_POSITION_NED | 10Hz | 本地位置/速度 |
| GLOBAL_POSITION_INT | 2–5Hz（有有效全球位置时） | 全球位置、高度、航向 |
| GPS_RAW_INT | 2Hz（GNSS启用时） | GNSS fix/卫星/精度 |
| ESTIMATOR_STATUS | 2Hz | 导航估计器健康 |
| VFR_HUD | 2–5Hz | 高度、速度、爬升率 |
| EXTENDED_SYS_STATE | 1Hz/状态变化 | 着陆/飞行阶段 |
| HOME_POSITION | 建立/更新/清除时 | Home位置 |
| MISSION_CURRENT | 1Hz/航点变化 | 当前任务进度 |
| AUTOPILOT_VERSION | 连接时/请求时 | 固件、能力和版本 |
| STATUSTEXT | 事件触发 | 人类可读告警/诊断摘要 |

FCU Dialect 定义 13 个私有消息：`FCU_OPERATION_STATUS`、`FCU_NAV_STATUS`、`FCU_SENSOR_HEALTH`、`FCU_CONTROL_STATUS`、`FCU_ACTUATOR_STATUS`、`FCU_LINK_STATUS`、`FCU_STORAGE_STATUS`、`FCU_RESOURCE_STATUS`、`FCU_TIME_STATUS`、`FCU_EVENT`、`FCU_PREARM_STATUS`、`FCU_RC_INPUT_STATUS`、`FCU_BUILD_INFO`。其中PREARM_STATUS提供逐项检查ID/结果/原因/检查版本；RC_INPUT_STATUS提供帧新鲜度、质量和failsafe；BUILD_INFO提供Firmware/ABI/Profile/参数集标识。网络定义来自 `spec/contracts/FCU.xml`，消息 ID 为 43000–43012。固定版本 mavgen 与 common.xml 一起生成字段布局和 CRC_EXTRA，来源与验证记录见 09。

原始1kHz IMU、KF-GINS_QUAD内部协方差、MPC矩阵、完整地图Cell和高频调试数据默认写SD，按需导出，不持续通过115200链路发送。

### 10. 115200带宽与拥塞策略

115200 baud、8N1理论字节上限为11520 B/s。持续遥测预算初始不超过理论上限60%（6912 B/s）。私有消息目标合计不超过 3500 B/s，标准消息、ACK 和突发事件保留至少 2500 B/s。实际预算按每条消息`payload + MAVLink header/checksum + 可选13B签名`计算，并用真实Dialect生成帧测量。拥塞顺序：丢弃P3调试/日志→降低P2常规遥测→保留P1 HEARTBEAT/ACK/安全状态；任何网络拥塞不得阻塞RC、M4、M6或M7。

### 11. 事件与安全审计

每次Arm/Disarm、FlightMode变化、Authority变化、Failsafe变化、Mission提交/启动/中止、命令拒绝、签名/重放失败、Permit/Demand Epoch不匹配都记录结构化事件，至少包含事件ID、source、reason、monotonic时间、旧/新状态和关联Epoch。遥测发送失败不影响本地事件记录。

### 12. 命令字段、结果和资源边界

所有字段以02 ProductOperationPayload.valid_mask显式存在；未声明/不允许字段必须零，非有限/越界拒绝。ARM必需PREARM_EPOCH；SET_MODE必需REQUESTED_MODE且在0..8枚举内；TAKEOFF必需ALTITUDE_ABOVE_HOME（>0且包线内，Home有效）；SET_HOME必需TARGET_NED（与当前Origin匹配）且DISARMED；MISSION_COMMIT必需MISSION_ID+MISSION_TRANSACTION+非零mission_hash并匹配上传source/session；START/PAUSE/RESUME/ABORT/CLEAR必需MISSION_ID、VALID_MISSION_EPOCH置位且mission_epoch非零并等于M4当前任务epoch（bit8清零、epoch=0或同mission_id旧epoch一律拒绝，记reason并保留当前任务）；SETTINGS_AUTHORIZE必需SETTINGS_TRANSACTION且有维护权限；LAND/RETURN/DISARM无位置字段。未支持的payload组合返回INVALID_ARGUMENT，紧急终止不能伪装普通DISARM。

M1只在认证成功后分配source_id/session，session在换key/重建维护身份时新建且不复用；request_id为该session递增uint32，耗尽换session。32 项固定去重缓存记录键、命令码、规范化载荷字节、阶段/结果与 ACK 路由；保留60s，满时拒绝新非幂等操作，不逐出运行中的请求。CommandAck.Header.source_id为发布者M4，request_source_id/source_session_id回显请求方。MAVLink8bit seq与confirmation字段均不是完整事务ID：普通standard command同一source/session/command只允许一项运行，参数一致重传关联原request_id；完成后60s内相同参数视为重传，重新执行需显式本项目request_id协议/新会话。MISSION协议使用其staging transaction；禁止以payload时间戳猜唯一性。

同时最多4个长操作tracker，request入队TTL≤1s，M4首次处理超期拒绝；运行deadline单独为ARM100ms、DISARM2s（地面策略）、MISSION_COMMIT1s、其余长运动120s或任务明确更小的受控deadline。IN_PROGRESS每1s一次，超期先撤销关联授权并报FAILED/TIMEOUT；已有实时安全动作不因ACK失败而停止。Takeoff完成需已授权模式且高度误差≤0.2m、|v_D|≤0.2m/s保持1s（Host/SIL fixture）；LAND完成需LANDED，RETURN完成需到达当前Home token且终端动作完成。实机这些数值由verified policy提供，否则相应长运动UNSUPPORTED。SET_MODE/任务提交/启停的成功仅表示状态原子提交，任务到达另由MissionStatus报告。

PreArm ID按上述检查表次序固定0..14，禁止重排；状态0UNKNOWN、1PASS、2FAIL、3WARNING。mask bit=id，required_mask由请求模式决定，UNKNOWN/FAIL的必需项一律拒绝。FCU_PREARM_STATUS每条携带单项ID/state/reason/evidence_time/mode_mask/recovery_condition，不声称PreArmCheckResult结构本身包含15份detail。FCU_BUILD_INFO携带完整SHA256分片；遥测hash字段不能替代完整发布manifest。

五状态域枚举按本章表列顺序从0起固定，LandedState等以02显式C枚举为准。first-reference绝对100ms超时独立于25ms许可租约，FMS任务不得阻塞等待M5；ARMING期间软件保持抑制输出，BENCH_INHIBIT在所有状态均不使能物理输出。

签名使用[官方Signing规范](https://mavlink.io/en/guide/message_signing.html)的48bit/10µs时间戳及(system,component,link)流防重放。当前无可信跨掉电高水位/密钥保护证据，H723默认REMOTE_MUTATION=DISABLED；只读非敏感遥测可用。重新启用必须有可信时钟/持久高水位方案的目标证据，或地面本地换新key并重建source_session。不得发明不在协议中的“自动可信握手”。
