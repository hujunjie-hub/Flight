> 历史交付记录。2026-11-05 M4/M5实施及当前0x00010905接口、验证结果见 [M4_M5_IMPLEMENTATION.md](../m5/M4_M5_IMPLEMENTATION.md)。

# T191-C-M5 交付报告（独立准备，NONE 障碍源切片）

| 项目 | 内容 |
|---|---|
| 任务 ID | `T191-C-M5` |
| Owner | C |
| 基线 | `V1.9.1-GLOBAL-AUDIT-R3` / Schema 6.0 / Product-Port `0x00010904`（CR-V191-C01） |
| 前置 | `T191-C-FMS`（C 侧已提交独立准备，D 未验收）；`T191-B-REPLAY`（未满足） |
| 验证范围 | Host 生产契约 + ARM Cortex-M7 编译；`IMPLEMENTATION`（Host） |
| 结果 | Host **10/10 PASS（72 checks）**；ARM 编译 PASS；A*/地图/避障、目标、SIL/HIL、D 审核 `NOT_RUN` |

本报告是执行上下文提交的交付回执，不是 D 的任务状态或 09 验证结论。

## 1. 前置状态

- `T191-C-FMS`：C 已交付（Host 16 场景/215 断言 + ARM 编译），HANDOFF 仍为 `READY_FOR_ASSIGNMENT/NOT_RUN`，D 未审核。
- `T191-B-REPLAY`：HANDOFF `READY_FOR_ASSIGNMENT/NOT_RUN`，B 的 estimator 骨架存在但无 replay 接口。
- 本次只做独立准备，不声称前置满足或已验收。

## 2. 交付物

| 路径 | 内容 |
|---|---|
| `applications/core/m5/src/m5_internal.h` | 状态、约束/原因码、数学接口 |
| `applications/core/m5/src/m5_math.c` | 五次边界值求解/采样、五阶 Hermite、四元数、围栏几何 |
| `applications/core/m5/src/m5_update.c` | `guidance_validate/init/update/get_health/stop`、参考构造、校验、READY/ACTIVE |
| `applications/core/m5/Kconfig`、`SConscript` | `M5_QUAD_X4_ENABLE`（依赖 `FMS_QUAD_X4_ENABLE`，默认关闭） |
| `applications/plugins/guidance/quad_x4/` | QUAD 默认 `ProductGuidanceConfig`、构建接入 |
| `applications/core/m5/hosttest/` | 10 场景 Host 测试、构建/ARM 脚本、证据与 SHA256 |

## 3. 实现语义

1. **参考类型**：按 `VehicleDirective.operation_code`/`guidance_mode_code` 选择 Pilot RATE/ATTITUDE/ALT_HOLD 或轨迹（HOLD/MISSION/RETURN/LAND/SAFE_ACTION）。
2. **轨迹**：以 nav State Time 为 t0；从当前 p/v/a 到目标（任务当前项/Home/着陆点/锁存保持点）构造五次多项式；`anchor.offset_us=0` + 24 点×20 ms（0..480 ms），五点 `valid_mask` 全置；超出时长后保持终点。
3. **校验**：逐点检查速度/加速度/jerk；围栏 Keep-in/Keep-out+margin+高度；用消费者同款五阶 Hermite 在每段中点复算并与解析五次曲线比对（0.005 m / 0.05 m/s）；失败返回 `REJECTED`+`GuidanceFeasibility`（约束 ID/原因码）。
4. **生命周期**：`GuidanceStatus` READY；当 `ActuationPermit` 的 `reference_epoch/authority_epoch/directive_epoch` 匹配且非撤销、未过期→ACTIVE；不要求 M4 等待 ACTIVE。参考 id/epoch 在上下文不变时保持，换项/换 token/换模式时递增。
5. **能力/时效**：检查 nav/directive/envelope/authority 头一致（schema/profile/boot/time_epoch）、TTL、`control_time_us==State Time`、state/authority epoch 匹配；不符返回错误且输出无效。
6. **NONE 适用矩阵**：`obstacle_source_mode` 仅 0；`mission.flags.requires_obstacle_avoidance=1` 的任务拒绝（`0x0605`），不生成伪 FREE 地图。

## 4. 证据

```text
Host: wsl bash applications/core/m5/hosttest/run_host_tests.sh  -> exit 0
      10 tests, 72 checks, 0 failures
ARM : pwsh -File applications/core/m5/hosttest/arm_build_check.ps1 -> exit 0
      text 13320 B, bss 304 B (M5_ARM_COMPILE_OK)
```

日志/命令/环境/哈希：`applications/core/m5/hosttest/evidence/`。

## 5. NOT_RUN / 明确缺口

1. M5-02 局部地图、M5-03 Kinodynamic A*、M5-06 连续碰撞/swept-volume/动态障碍：未实现；依赖避障的任务全部拒绝。
2. M5-05 minimum-jerk 轨迹优化器（固定时长五次样条优化）未实现；当前为直接五次边界曲线 + 时域延长重试。
3. 倾角/推力约束（需控制模型）未参与 feasibility；仅速度/加速度/jerk/围栏/时间。
4. 目标构建、WCET/栈、SIL/HIL、与 M4/M6/M7 的闭环联调：未执行。
5. M4 侧 `mission_epoch` ICD 缺口仍开放（见 `T191-C-FMS` 报告），M5 仅复制该字段。
6. D 独立审核与 09 登记：`NOT_RUN`。

## 6. 回滚

删除 `applications/core/m5/`、`applications/plugins/guidance/` 及 `build/m5_*`；`applications/core/Kconfig` 仅追加了一行 `source`（可回退）。
无 Git 元数据，`commit=null`；快照见 `evidence/artifact_sha256.json`。
