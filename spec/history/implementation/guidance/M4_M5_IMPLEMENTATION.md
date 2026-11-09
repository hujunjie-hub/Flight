# STM32 M4 FMS / M5 Guidance 实施交付

日期：2026-11-05（Asia/Shanghai）。范围：现有 Flight 工程的 QUAD_X4_H723 / NONE 障碍源配置。当前 Schema 6.0，Product/Port Revision **0x00010905**；变更记录 **CR-V191-M45-01**。

本轮在已有 M4/M5 基础上完成固件模块实现与回归。没有改写其他任务的验收状态，没有把作者测试记为独立审核或飞行放行。

## 已落地功能

### M4：操作管理与授权

- 保留 MissionManager 唯一写入任务状态、当前项和 execution_token 的边界；任务提交、开始、暂停、恢复、中止、完成继续使用生产契约。
- 执行导航能力恢复 dwell 和进入/退出门限滞回；检查所需维度的有效位、有限值、不确定度与状态纪元。
- 水平导航丢失时暂停任务并选择可执行的着陆参考；能源 critical 强制着陆；执行器/可控自由度不足进入锁存 TERMINAL。链路恢复或摇杆回中不自行恢复 Pilot Authority。
- 起飞命令锁存明确 NED 目标，使用 Home Down 减去起飞高度，更新 directive_epoch；不再只创建完成监测 tracker。上下文改变时取消旧起飞操作。
- 任务到达判定使用三维距离及三维速度，避免水平到点但高度错误、仍在快速升降时推进任务。
- 地面接触证据失效时立即清除旧 LANDED 结论；NED 正向下降识别为着陆。没有可靠地面证据时不假定已经着陆。
- Geofence 配置检查非有限值、负 margin、退化/自交多边形；第一次绑定导航坐标上下文，导航 reset 后不把旧围栏静默变换到新坐标。
- READY + 匹配 feasibility 即可签发 Permit；保持续租、撤销 watermark 和首参考窗口，避免等待 ACTIVE 的闭环死锁。
- H723/NONE 在 M4 提交阶段拒绝要求避障保证的任务；M5 仍独立拒绝此类输入。

围栏配置在 `quad_x4_fms_configure_product_geofence()` 调用内验证并复制到 M4 独占存储。调用方的局部配置在函数返回后即可失效。init 消费安装标志，并保留围栏存储区、重置其余引擎状态；失败重试需重新安装。没有第二份 1144B 全局配置，也不需要完整围栏的临时栈副本。

### M5：参考生成与连续验证

- 拆分输入适配、轨迹、人工参考、自动垂向着陆、数学与独立验证模块，仍通过 `guidance_validate/init/update/get_health/stop` 提供统一 Port。
- 使用固定端点单段 minimum-jerk quintic；0.2s 时格，最多16次时长尝试，最多6.2s。不可行的长距离任务明确拒绝，需要上层显式分段。
- 用 Bernstein 凸包界检查整段速度、加速度、jerk、总推力、倾角和 yaw rate；同时执行 SafetyEnvelope、产品配置和当前航点速度上限。
- 修复原校验循环始终检查终点的问题；实际发布的 anchor + 24×20ms 点按消费者的五阶 Hermite 重建并独立验证，yaw 使用最短角差的三阶 Hermite。
- 修复原求解器按绝对行列式门限把20ms区间误判为奇异的问题；使用归一化边界解和内部 double 系数，保持公共载荷 float32。
- 连续围栏验证使用曲线区间的 Bernstein 位置包围界及有符号净空。实际24区间采用固定4096B工作表、深度上限5、最多256个工作单元；无法证明或预算耗尽即拒绝。
- 围栏几何膨胀计入配置机体半径/裕度、导航位置及速度不确定度、State Time 到 evaluation_time 的延迟。配置和不确定度不足时不发布可行参考。边界过紧可能保守拒绝，不能据此放松配置绕过检查。
- 在同一授权上下文内持续推进同一条曲线；下一帧 anchor 与上一帧对应预览点一致。保持模式锁存位置；导航 reset、任务换项/token、权限/指令变化时重建参考。
- 人工 RATE 按 roll→FRD X、pitch→FRD Y、yaw→FRD Z；姿态参考限制合成倾角，按实际 State Time 积分 yaw；ALT_HOLD 积分垂向参考，摇杆回中后锁存高度。
- 水平导航不可用时，M4 可明确授权 `GUIDANCE_REFERENCE_ALTITUDE_AUTOMATIC=5`。M5 发布 typed `ALTITUDE_ATTITUDE` 参考，不生成虚构 XYZ MotionReference。采用最多1m的有界五次下降段；有效 Home 存在时限制到 Home 高度，可靠 LANDED 时保持。存在必须证明的围栏时拒绝该无水平保证的路径。
- 校验授权输入、任务 token/epoch/索引、来源时间轴、有效位、过期和 Permit 的 boot/state/authority/directive/reference/policy 绑定。参考与状态截止时间不晚于原授权源的最早截止，不把发布时刻冒充 State Time。

## 接口与文件

| 路径 | 内容 |
|---|---|
| `applications/core/m4/src/` | 八个 M4 子模块及统一串行入口 |
| `applications/core/m5/src/m5_update.c` | 参考生命周期、状态和健康发布 |
| `applications/core/m5/src/m5_input.c` | 输入、时效、纪元、有效域及授权适配 |
| `applications/core/m5/src/m5_trajectory.c` | 目标、时标、曲线缓存及24点预览 |
| `applications/core/m5/src/m5_validate.c` | Bernstein 动力学、包络及连续围栏验证 |
| `applications/core/m5/src/m5_pilot.c` | RATE/姿态/定高人工参考 |
| `applications/core/m5/src/m5_vertical_land.c` | 无水平导航保证的显式自动垂向参考 |
| `applications/core/common/fc_fence.*` | M4/M5 共用多边形与带符号净空几何 |
| `applications/plugins/fms/quad_x4/` | FmsPolicyPort 产品绑定 |
| `applications/plugins/guidance/quad_x4/` | Guidance 配置辅助函数 |
| `applications/contracts/`、`applications/products/QUAD_X4_H723/` | 唯一生产契约及重建的 ABI lock |

新增 `ProductDirectivePayload.target_valid_mask/target_position_ned_m/target_yaw_rad` 和 `ProductGuidanceConfigPayload.mass_kg/gravity_mps2`。mass/gravity 必须来自与 M6/Plant 相同的模型数据；默认辅助函数的1kg/9.80665m/s²仅为 Host 合成夹具。旧0x00010904 ABI不能与本版混用。

## 集成调用顺序

1. 在应用初始化阶段提供真实 ProductProfile、FMS/Guidance 配置及可选围栏，分别 validate/init。
2. 串行执行 M4，输入当前公开导航/能力/链路/请求及上一帧 M5 状态，取得 Directive、Envelope、Authority、Mission、Permit。
3. 用同一导航 State Time 和当前 evaluation_time 构造 GuidanceInput，将 M4 输出直接传给 M5。
4. 下一周期 M4 消费 M5 READY/feasibility，签发匹配 Permit；M5 随后确认 ACTIVE。M6/M7必须消费一致、未过期的 bundle。

`FMS_QUAD_X4_ENABLE` / `M5_QUAD_X4_ENABLE` 已接入现有 SCons/Kconfig。BSP 的 `applications/main.c` 仍为基础循环，**本轮没有安装运行期 dispatcher、M6/M7闭环或自动解锁逻辑**。相关数据生产者/调度属于完整系统集成，不能把本模块链接测试当作板上运行。

## 验证与证据

实际结果以 `build/m4_m5_implementation/evidence/validation.json` 为准：

- M4：23场景、340项断言；M5：20场景、6003项断言；另有2个真实 Port 联调场景（联调程序同时重跑23个 M4 回归，共411项断言）。11类验证命令全部退出0，断言失败数为0。
- ASan/UBSan：运行 M5 和含全部 M4 回归的联调程序；不关闭栈生命周期检查。
- Cortex-M7：生产源 C11 `-Wall -Wextra -Werror -fno-short-enums` 编译；原 SCons 固件另按现有 C99 配置链接。
- ABI：145个公共类型，C11/C++14，ARM与Host布局对照及规范字段对照。
- 静态内存：规范 slot/scratch/保留42472B，加 M4 实际 BSS6218B（按8B对齐保留6224B），合计48696B≤49152B，余456B；M5状态576B+验证工作表4096B=4672B≤16KiB。
- 原工程 SCons：临时启用 M4/M5，以 Port 函数作为链接根保留实现，生成 `build/m4_m5_implementation/target/Flight.elf`、`.bin`、`.map`；恢复原 `rtconfig.h` 和根目录原输出，根 `.config` 不变。

编译/运行日志、命令退出码、源 SHA256 在 `build/m4_m5_implementation/evidence/`；ABI报告在 `applications/tests/abi/evidence/`。静态内存检查不等于完整系统arena定位、栈峰值或WCET证明。

从 Flight 根目录复现：

```powershell
# 本次环境已安装；新环境先创建独立构建工具环境
uv venv build/m4_m5_tools
uv pip install --python build/m4_m5_tools/Scripts/python.exe scons==4.11.1

python applications/core/m5/hosttest/run_validation.py
```

测试依赖当前工作站的 WSL `Ubuntu-26.04`、GNU Host GCC和 `D:/STM32CubeCLT/GNU-tools-for-STM32/bin`。ARM工具路径可用 `ARM_GCC` / `ARM_SIZE`（对象编译）及 `FC_ARM_BIN`（ABI/内存/固件链接）覆盖。

## 适用边界与未执行项

- 本版 H723 只接纳 NONE 障碍源。STATIC_MAP/SIMULATION/ONBOARD在init拒绝；不包含板端地图、A*、动态障碍预测或避障保证。H723资源规范本就不接纳完整地图配置。
- 真实质量/惯量、机体危险包络、不确定度系数及输出安全策略尚需产品实测。现有默认 helper 为合成/台架配置，真实输出策略仍按未验证拒绝。
- 原 GNU Arm 固件链接存在 enum-size 警告，完整日志保留，未通过屏蔽警告声明解决。公共生产契约的 C/C++ ARM/Host 布局检查通过，仍不能代替完整工具链/库 ABI 审核。
- 运行期调度、M6对自动高度kind=5的消费、M7实际输出、全系统负载、目标板/WCET/栈/RTA、SIL/HIL/飞行和独立审核均未执行。

## 回滚

原文件快照在 `build/m4_m5_implementation/originals/`。恢复对应原文件后，删除本轮新增的 `applications/core/common/`、M5新增源/验证脚本和本交付文档，再从恢复的规范与生产头重新生成 ABI证据。不得只回滚公共头而保留0x00010905消费者或ABI锁。

历史 T191 交付/任务主状态没有被本报告改写；本轮入口和证据以此文档及09号规范增量为准。
