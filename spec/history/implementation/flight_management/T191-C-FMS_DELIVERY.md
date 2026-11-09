> 历史交付记录。2026-11-05 M4/M5实施及当前0x00010905接口、验证结果见 [M4_M5_IMPLEMENTATION.md](../m5/M4_M5_IMPLEMENTATION.md)。

# T191-C-FMS 交付报告（独立准备）

| 项目 | 内容 |
|---|---|
| 任务 ID | `T191-C-FMS`（Agent C 首项；M5/M6/M7 均依赖它） |
| 类型 | IMPLEMENTATION |
| Owner | C（VEHICLE/CONTROL） |
| 基线 | `V1.9.1-GLOBAL-AUDIT-R3` / Schema 6.0 / Product-Port `0x00010904`（CR-V191-C01） |
| Profile | `QUAD_X4_H723`（`0x00010001`） |
| 前置 | `T191-A-ABI`（已 `ACCEPTED`）、`T191-B-ADAPTER`（`ASSIGNED`，未验收；见 §1） |
| 验证范围 | Host 生产契约下 M4 状态机/边界测试 + ARM Cortex-M7 编译；`IMPLEMENTATION`（Host） |
| 本次结果 | Host 测试 **17/17 PASS（240 checks，0 failures）**；ARM 编译 **PASS**；`T191-C-FMS` 现为 `REVIEW/seq5`；目标构建/WCET/SIL/HIL/Gate D：`NOT_RUN` |
| 交付时间 | 2026-11-04（Asia/Shanghai） |

本报告由实际执行该 prompt 的 Agent C 执行上下文编写，不是 D 的任务记录更新。
任务主状态、09 证据台账与发布状态仍由 D 维护；本报告不声称已接单、已审核、已通过 Gate A/D。

## 0. 结论摘要

1. 在 `applications/core/m4/` 交付 M4 公共引擎：任务管理、权限、模式/失效等级、Permit、Home/Geofence、PreArm、LandDetector 和统一串行入口。
2. 在 `applications/plugins/fms/quad_x4/` 交付 `QUAD_X4 AERIAL_FMS` 产品绑定：`fms_policy_validate/init/update/get_health/stop`、策略常量、配置校验、SafetyEnvelope 构造和 Profile/ID 校验。
3. 直接使用 A 已落盘的**生产契约头**（`applications/contracts/` + `applications/products/QUAD_X4_H723/`）编译，不保留第二份 ICD/ABI 副本；开始时使用的测试夹具已删除。
4. 覆盖任务要求的核心边界：MissionStatus 唯一写入、execution_token 轮换、两航点推进与完成、Authority 接管锁存与 RC 丢失、READY→Permit→ACTIVE 握手（不等待 ACTIVE）、25 ms 许可续租、撤销 watermark、暂停/恢复/中止/清除。
5. Host 测试 17 项全部通过；ARM Cortex-M7 6 个生产对象在 `-Werror` 下编译通过。
6. 未执行项见 §6，CR-C01/C02 实现与设计决定见 §7。

## 1. 前置状态与“不伪称”声明

读取 `spec/AGENT_HANDOFF.md`（revision 4 提交前的最新状态）：

- `T191-D-REVIEW`：`ACCEPTED/seq6`（A/B/C 意见已记录，C 为 revision 2：22 项 15/1/6）；`CR-V191-C01/C02` 由 D 登记为 `PROPOSED`。
- `T191-A-ABI`：`ACCEPTED/seq7`；生产头与 `abi_lock.h` 已按 `0x00010904` 生成并由 A 复核接受。
- `T191-B-ADAPTER`：`ASSIGNED/seq3`；有落盘产物，但未验收。
- `T191-C-FMS`：`ASSIGNED/seq4`；`T191-C-M5` 保持 `READY`（待 C-FMS），`T191-C-M6/M7` 保持 `READY/NOT_STARTED`。

因此 C-FMS 的两个直接前置（A-ABI、B-ADAPTER）仍**未 ACCEPTED**；本次是 CR-C01/C02 实现后的再次独立准备提交，不是验收或 Gate 结论。执行期间 A 已并发更新 `applications/contracts/`、`product_contract.h`、`abi_lock.h`；C 的 M4 直接包含这些生产头，不保留第二份契约副本。

## 2. 交付物与落点

### 2.1 M4 核心（`applications/core/m4/`）

| 文件 | 内容 |
|---|---|
| `include/m4_contract.h` | 唯一契约入口：`fc_port_api.h` + `fc_message_ids.h` + M4 状态值 |
| `include/m4_state_values.h` | 06 §7 固定的五个正交状态域、PreArm ID、飞行阶段等 M4 自有取值（ICD 只用 uint8/uint16 承载） |
| `include/m4_policy_profile.h` | `M4Policy` 产品策略常量视图与产品钩子声明 |
| `src/m4_internal.h` | 引擎/子模块状态、固定容量、内部 API |
| `src/m4_engine.c` | 输入校验/时钟绑定、命令分发与去重、长操作 tracker、快照发布 |
| `src/m4_mission.c` | M4-01：staging→commit、LOADED/RUNNING/PAUSED/ABORTED/COMPLETED、token/epoch、到达判定、MissionStatus/AuthorizedMissionSnapshot 数据源 |
| `src/m4_authority.c` | M4-02/03：模式合法转换、显式接管锁存、GCS 禁止越权、Failsafe 升级与强制安全动作 |
| `src/m4_permit.c` | M4-08：IDLE_ONLY/CONTROL_ACTIVE 签发、同 epoch 续租、撤销 watermark、READY 握手、100 ms 首参考窗口 |
| `src/m4_support.c` | M4-04/05/06/07：Home、Geofence、PreArm、LandedState |
| `Kconfig`、`SConscript` | 默认关闭的构建接入（`FMS_QUAD_X4_ENABLE`），不改变现有固件源集 |

### 2.2 产品绑定（`applications/plugins/fms/quad_x4/`）

| 文件 | 内容 |
|---|---|
| `quad_x4_fms.c` | `FmsPolicyPort` 五个符号、`M4Policy` 常量、配置/Profile 校验、SafetyEnvelope、每模式导航能力与 PreArm 需求表 |
| `quad_x4_fms.h` | Port 声明与产品集成钩子（默认配置、Geofence 注入） |
| `SConscript` | `FMS_QUAD_X4_ENABLE` 守卫的构建组 |

### 2.3 测试与证据（`applications/core/m4/hosttest/`）

| 文件 | 内容 |
|---|---|
| `test_m4_main.c` | 17 个 Host 场景、240 个断言，直接调用生产 `fms_policy_*`（含 CR-C01/C02 负向路径） |
| `run_host_tests.sh` | WSL/gcc 构建 + 运行 |
| `arm_build_check.ps1` | Cortex-M7 `-Werror` 编译 + `arm-none-eabi-size` |
| `evidence/host_tests.log` | 实际测试输出（exit 0） |
| `evidence/arm_build.log` | 实际 ARM 编译/size 输出（exit 0） |
| `evidence/commands.json` | 命令、退出码、结果、NOT_RUN 清单 |
| `evidence/environment.txt` | 工具链/基线/日期 |
| `evidence/artifact_sha256.json` | 全部交付文件 SHA256（无 Git 时的快照） |

## 3. 实现边界（按 06 §7.1 八个子模块）

| 子模块 | 本次实现 | 未实现/后续 |
|---|---|---|
| M4-01 MissionManager | 上传候选校验、原子提交、当前项、token、MissionState、MissionStatus、到达+停留推进、暂停/恢复/中止/清除 | M1 staging 分片/超时（M1 职责）；跳项命令（当前 ICD 无对应命令） |
| M4-02 ModeManager | 合法模式表、能力/Home/任务约束、directive_epoch 变更 | 模式进入 dwell（`nav_enter_dwell_us` 仅校验未使用） |
| M4-03 AuthorityManager | 显式接管边沿 + 新鲜 RC、锁存、GCS_SET_MODE 拒绝、RC 丢失降权、释放条件 | MOMENTARY_OVERRIDE（默认关闭，按规范不实现） |
| M4-04 HomeManager | ARM 自动采集、SET_HOME、TTL、origin/state_epoch 变化失效 | 真实 GNSS Home 质量证据（无硬件） |
| M4-05 GeofenceManager | 配置校验、Keep-in/Keep-out、水平 margin、高度界、任务点检查 | M5 轨迹级连续验证（M5-06 职责） |
| M4-06 PreArmManager | 15 项固定 ID、按 `verified_input_mask` 的外部证据位、mask/首个原因、check_epoch、Arm 绑定与 1 s 过期拒绝 | IMU/求解器/存储的直接证据（由 A/B 提供） |
| M4-07 LandDetector | Host 夹具语义：CONTACT+低速低加速度 2 s→LANDED；NO_CONTACT 200 ms→IN_AIR；其余 UNKNOWN | 实机地面接触证据（无传感器，固定 UNKNOWN） |
| M4-08 FailsafePermitManager | FailsafeLevel、TERMINAL 锁存、Permit 生命周期、IDLE_ONLY≤25 ms、CONTROL_ACTIVE、撤销、100 ms 首参考窗口 | 完整场景矩阵（见 §6） |

## 4. 核心语义与测试覆盖

| 语义（来源） | 实现要点 | 对应测试 |
|---|---|---|
| MissionStatus 唯一写入（02 §8/§21、06 §7.1） | 只有 M4 填 `FmsOutputView.mission_status`；M5 无入口 | `two_waypoint_mission`、`pause_resume` |
| execution_token 非零、严格递增、换项/恢复换 token、溢出拒绝（02 §8） | `token_next` 从 1 起；`UINT64_MAX` 拒绝新执行 | 两航点、暂停恢复、中止 |
| 两航点推进与完成 | 半径+速度+dwell 判定；完成置 COMPLETED/SUCCESS/1.0，撤销 token | `two_waypoint_mission` |
| READY→Permit→ACTIVE 不互等（02 §9、00 §4.6） | READY+feasible 即签 CONTROL_ACTIVE；随后 ACTIVE 不改变 epoch | `control_active_ready_handshake` |
| 续租保持 permit_epoch（02 §8） | 绑定不变仅刷新 `permit_expires_at_us` | `idle_lease_renewal_bounded` |
| 撤销立即且不可回退（02 §8、06 §7.3） | 撤销独立一次发布 REVOKED，新 epoch>旧；撤销后一个新 epoch 才可重签 | `permit_revocation_watermark` |
| 首次解锁 100 ms 绝对窗口（06 §3.1） | 超时未建立 CONTROL_ACTIVE：地面→受控 Disarm；空中→ACTION_REQUIRED | `first_reference_window` |
| Authority epoch 只在真实切换时递增（02 §17） | 链路恢复/回中不递增；锁存后 GCS 不能改模式 | `pilot_takeover_latch` |
| RC 丢失→暂停任务+返航（06 §4.2） | 有 Home/位置能力→AUTO_RETURN，否则 LAND，再否则 TERMINAL | `rc_loss_return` |
| 暂停/恢复边界（06 §3.1） | PAUSE 冻结进度、token=0、撤销 Permit；RESUME 全量重验、新 token/epoch | `pause_resume` |
| 命令去重（02 §5、06 §12） | 32 项 cache，同键同 payload 回放 ACK；同键不同 payload 拒绝 | `duplicate_request_replay` |
| 无效 ABI/纪元/时戳拒绝 | 头一致性、TTL、future publish、epoch 绑定逐项校验 | `guidance_epoch_mismatch`、`arm_requires_fresh_prearm_epoch` |
| Geofence 必需时拒绝越界任务 | Keep-in 点检查 + 高度界，失败在 commit 拒绝 | `geofence_required` |

## 5. 验证证据（实际命令与结果）

环境：

```text
host: WSL2 Ubuntu 26.04, x86_64, gcc 16.0.1
arm : GNU Tools for STM32 14.3.1 (arm-none-eabi-gcc 14.3.rel1)
baseline: V1.9.1-GLOBAL-AUDIT-R3 / Schema 6.0 / Product-Port 0x00010904
```

### 5.1 Host 生产契约测试

```text
command: wsl.exe -d Ubuntu-26.04 -- bash applications/core/m4/hosttest/run_host_tests.sh
exit: 0
result: 17 tests, 240 checks, 0 failures
```

测试使用 A 的生产头（`applications/contracts`）与 QUAD 产品契约，不启用任何测试专用 ABI 定义；
覆盖列于 §4，日志见 `evidence/host_tests.log`。

### 5.2 ARM Cortex-M7 编译

```text
command: pwsh -NoProfile -File applications/core/m4/hosttest/arm_build_check.ps1
flags  : -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
         -std=c11 -Wall -Wextra -Werror -fno-short-enums
exit: 0
text: 1520 + 11484 + 3744 + 2168 + 4512 + 2340 = 25768 B
bss : 7213 B (quad_x4_fms.o: 静态 M4 引擎实例 + ProductGeofenceConfig 副本)
```

这是**编译与静态布局**证据，不是目标构建：没有链接脚本/map、没有 WCET/栈水位、没有板级运行。

## 6. 未执行 / NOT_RUN（不得据此声称通过）

1. `ARM_TARGET_BUILD`：未运行 `scons`/Keil/IAR 固件构建，未生成 `Flight.map`，未核对 52.5 KiB 栈、48 KiB `ICD_AND_M4_ARENA` 与 M4 静态状态的实际占用。
2. WCET/RTA/DWT/栈水位/CPU 预算：07 §任务表的 150 µs、1.5% 预算未验证。
3. SIL/HIL/目标板/飞行：无。
4. M1 集成：无 MAVLink 解析、签名、去重 cache 的生产传输；ACK 仅生成结构，未编码到 `FCU.xml` 帧。
5. M5/M6/M7 消费者联调：无；READY/Permit/ACTIVE 仅以 Host 夹具验证。
6. M2/M3 真实输入：B-ADAPTER 未交付，导航/链路/执行器数据为夹具。
7. 长操作 tracker 路径：TAKEOFF/LAND/RETURN 的完成/超时/IN_PROGRESS 未单独建测试（实现已存在）。
8. Failsafe 全矩阵：仅覆盖 RC 丢失→RTL；GNSS/磁/气压/IMU 降级、GCS/Companion 丢失、碰撞/模型失配、TERMINAL 清除未覆盖。
9. 时间/重启边界：time_epoch 变化、boot 变化仅实现代码路径，未建测试；无掉电/持久化。
10. Geofence 轨迹级验证、动态障碍、NONE 适用矩阵：属 M5-06/REQ-REV-191-R14。
11. 真实输出策略：`output_policy_id>=0x100` 被 fail-closed 拒绝，未登记/验证任何实机策略。
12. 独立审核：D 未审核；09 台账未登记，`verification_status` 仍应为 `NOT_RUN`。

## 7. 设计决定、偏差与需 CR 的缺口

1. **CR-V191-C01（CLOSED）**：`ProductOperationPayload` 增加 `mission_epoch`（0=不适用，`PRODUCT_OPERATION_VALID_MISSION_EPOCH=1u<<8`），M4 对 START/PAUSE/RESUME/ABORT/CLEAR 校验“bit 置位 + 非零 + 等于当前 epoch”，缺字段/旧 epoch 拒绝并回 reason 12；非任务命令必须为零。布局：`ProductOperationPayload` 44→48 B，`OperationRequest` 保持 160 B。`FC_PORT_API_REVISION` 与 `PRODUCT_CONTRACT_REVISION` 均为 `0x00010904`；A 已复核契约 diff 并重建 `abi_lock.h`（11 §4.1：CR-C01 CLOSED）。
2. **Mission commit 仅允许 DISARMED**：06 未明确允许空中提交；本实现选择保守策略（NOT_PERMITTED），已在报告中声明。
3. **任务完成策略**：最后一航点到达后 `COMPLETED/SUCCESS`、token=0、`POS_HOLD`、Authority=NONE；不自动 LAND/RETURN，需显式命令。该策略属首版产品选择，需 D/安全审查确认。
4. **暂停/恢复 epoch**：高级别 `mission_epoch` 在 commit/start/pause/resume/abort/clear/completion 递增；换项只换 token 不换 mission_epoch（测试已固定该语义）。
5. **任务 TTL 刷新**：M4 是 `AuthorizedMissionSnapshot` 生产者，每个 RUNNING 周期刷新 `mission_expires_at_us`；TTL 只约束消费侧快照新鲜度，不用于让 M4 自己“超时”。
6. **IDLE_ONLY 续租**：首参考 100 ms 窗口内按 25 ms lease 续租，绑定 LANDED+稳定 dwell；超时未建立 CONTROL_ACTIVE 时地面受控 Disarm、空中 ACTION_REQUIRED，不会无限续租。
7. **PreArm 外部证据**：IMU/求解器/存储/气压等无 M4 可见输入，采用 `verified_input_mask` 位（fail-closed UNKNOWN→拒绝 Arm）；`COMPASS_HEADING` 按 06 语义检查“估计器航向能力”，不要求磁力计在线。
8. **LandDetector**：实机无已验证地面接触源时固定 UNKNOWN，Disarm/AUTO_LAND 完成不成立；Host 夹具显式 CONTACT/NO_CONTACT 才有 LANDED/IN_AIR。
9. **CR-V191-C02（SPECIFIED，已实现待 Gate D）**：首版围栏为编译期 `ProductGeofenceConfig`（Product Manifest/Profile 生成的只读数据），`quad_x4_fms_configure_product_geofence()` 在 init 前安装，M4 校验 `profile_hash/config_version/geofence_hash`、NED/顶点/高度/margin 后唯一持有并发布 `ProductGeofenceSnapshot`；无运行时 `SET_GEOFENCE`，required 但未配置时 init fail-closed。引擎 setter 在 ARMED 后拒绝修改。11 §4.1 仍将 CR-C02 的 Gate D/目标证据记为未完成。
10. **静态预算**：M4 引擎实例约 7.2 KiB `.bss`（7213 B，含 `ProductGeofenceConfig` 副本），不在 `FmsOutputView` scratch 内。07 要求 48 KiB arena 统计所有对象；该占用必须由 A/D 计入 map/预算，本次未做目标 map，不能声称满足。
11. **输出策略**：配置校验只接受 0/1/2（NONE/BENCH_INHIBIT/SIM_CONTROLLED_STOP）；`>=0x100` 需登记验证，未登记即拒绝（`STATUS_UNSUPPORTED`）。
12. **构建接入默认关闭**：`FMS_QUAD_X4_ENABLE` 未出现在根 `.config`/`rtconfig.h`，新增 SConscript 组不编译任何源文件；`applications/core/Kconfig` 尚未被根链路 source（root Kconfig/`applications/Kconfig` 归 A/B，未擅自修改）。

## 8. 回滚

删除以下新增路径即可完全回滚（无既有文件被修改；`applications/plugins/SConscript` 为既有文件，未改）：

```text
applications/core/
applications/plugins/fms/
build/m4_host/        (生成物，已被 .gitignore 的 build 规则覆盖)
build/m4_arm/         (同上)
```

无 Git 元数据，因此 `commit=null`；快照以 `evidence/artifact_sha256.json` 和本报告的文件清单为准。

## 9. 状态请求与 D 的动作

本次提交为**独立准备交付**，不请求 `REVIEW/ACCEPTED/CLOSED`：

1. D 记录/确认 `T191-A-ABI` 与 `T191-B-ADAPTER` 的真实状态；若前置未验收，`T191-C-FMS` 应保持 `READY_FOR_ASSIGNMENT` 或按 10 §4 记 `BLOCKED`（`blocked_by=[T191-A-ABI,T191-B-ADAPTER]`，`resume_state=READY_FOR_ASSIGNMENT`）。
2. D 对 M4 Host 证据做独立复核，并把 `IMPLEMENTATION`/`HOST` 结果登记到 09（在此之前 `verification_status=NOT_RUN`）。
3. A 确认 `FmsInputView`/`ProductFmsConfig`/`FmsPolicyPort` 头为最终基线；B 交付 ADAPTER 后，C 重跑与真实 M2/M3 输入的联调。
4. CR-C01 已按 02/06 的字段语义实现；在 A 完成契约复核与 ABI 锁确认、D 合并前，不得把 06:632 标为验收满足。
5. 目标构建/WCET/SIL/HIL 由 `T191-D-HOST`/`T191-A-TARGET` 等后续任务执行，本次不代替。

## 10. 文件清单与 SHA256

见 `applications/core/m4/hosttest/evidence/artifact_sha256.json`（27 个文件，含证据日志）。
本报告与 `status_request_T191-C-FMS.json` 位于 `applications/core/m4/`。
