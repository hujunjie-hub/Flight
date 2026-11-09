# T191-A-PLATFORM 交付报告（独立准备·第一阶段）

| 项目 | 内容 |
|---|---|
| 任务 ID | `T191-A-PLATFORM` |
| 类型 | IMPLEMENTATION |
| Owner | A（PLATFORM/HARDWARE/ICD） |
| 基线 | `V1.9.1-GLOBAL-AUDIT-R3` / Schema 6.0 / Product-Port `0x00010903` |
| 前置 | `T191-A-ABI`（已提交，**未 ACCEPTED**；HANDOFF 仍为 READY/`update_sequence=2`） |
| 验证范围 | `IMPLEMENTATION`（本阶段实际执行：Host 单元测试 + ARM 编译） |
| 结果 | 服务核 Host PASS、ARM C99 编译 PASS；平台/目标集成 **NOT_RUN / IN_PROGRESS** |
| 交付时间 | 2026-11-04（Asia/Shanghai） |

本报告是执行上下文提交的独立准备，不是 D 的任务记录、不是 Gate B 会签，也不声称
`T191-A-PLATFORM` 已完成。任务主状态与 09 台账由 D 维护。

## 0. 本阶段结论

1. 补齐任务明确点名的两项平台缺失服务并可被 B/M2 直接消费：
   - `applications/services/time`：TIM2 tick→FCU monotonic 映射、PPS 频率驯服、UTC 映射；
   - `applications/services/raw_ingress`：160B raw slot lease ring，`fc_sensor_acquire_raw`/`fc_sensor_release_raw` 实现。
2. 生成可复核的平台资产/缺口矩阵：17 项要求中 6 项 `MISSING`、7 项 `ENABLE_NEEDED`、
   3 项 `REUSE`、1 项 `STUB`，每项附现有文件/Kconfig/`.ioc`/IRQ 证据。
3. Host 单元测试：raw ingress **72 checks / 0 fail**；Time Service **114 checks / 0 fail**。
4. 同一源码以 Cortex-M7、C99、hard-float 编译通过（ARM 目标编译仅证明语法/类型兼容，
   不代替固件链接/目标运行）。
5. 未改动 `board/`、`libraries/HAL_Drivers/`、`rt-thread/`、`.config`、`.ioc` 或链接脚本；
   无第二套设备框架。

## 1. 前置与“不伪称”声明

- `T191-A-ABI` 已由 A 提交（见 `applications/tests/abi/status_request_T191-A-ABI.json`），
  但 `spec/AGENT_HANDOFF.md` 未被 D 更新，`T191-D-REVIEW` 与 `T191-A-ABI` 仍为
  `READY_FOR_ASSIGNMENT / NOT_RUN / acknowledgement=null`。
- 因此本任务前置**未满足**。按 prompt，本次仅做独立准备并报告依赖；不声称已接单、
  已完成平台适配或已通过 Gate B。

## 2. 交付物

| 文件 | 内容 |
|---|---|
| `applications/services/time/fc_time_service.h` | 配置/句柄/API：`fc_time_service_init`、`FcTickSource`、诊断 helper；公共 Port 原型仍由 02 唯一声明 |
| `applications/services/time/fc_time_service.c` | 64 段 Q32.32 连续映射、单调钳制、PPS 门控（1000 ppm/50 µs/8 间隔）、±500 ppm 限幅与 5 ppm/s slew、2.5 s 超时 HOLDOVER、UTC offset/不确定度与 LOCKED/DEGRADED 阈值、外部 Companion 映射 |
| `applications/services/time/hosttest/test_time_service.c` | 映射连续性、历史换算、epoch 拒绝、PPS 锁定/周期拒绝/超时、UTC 锁定/配对拒绝/不确定度失效、外部映射 |
| `applications/services/raw_ingress/fc_raw_ingress.h` | 160B slot、lease/generation、lock hooks、统计 |
| `applications/services/raw_ingress/fc_raw_ingress.c` | SPSC 风格 ring、按 sensor 过滤且按发布序取最旧、满 ring 丢弃并记录 gap、重复/过期 release 拒绝、错误时清空 out |
| `applications/services/raw_ingress/hosttest/test_raw_ingress.c` | 生命周期、generation、筛选顺序、满池不覆盖、非法 payload、公共 API 绑定 |
| `applications/platform/rtthread53/tools/platform_inventory.py` | 扫描现有资产并生成缺口矩阵 |
| `applications/platform/rtthread53/tools/run_platform_tests.ps1` | Host 测试 + ARM 编译证据生成器 |
| `applications/platform/rtthread53/README.md` | 服务边界、板级绑定契约、未交付项 |
| `applications/platform/rtthread53/evidence/` | `platform_gap.md`、`platform_asset_scan.json`、`platform_tests.json`、日志、`artifact_sha256.json` |

关键文件 SHA256：

| 文件 | SHA256 |
|---|---|
| `fc_time_service.c` | `b857c2e53079834495091e1eda7ef97a76ff89e50394278fe40d71b76bf7618b` |
| `fc_time_service.h` | `b27da5d54e339a43085cfb9e8f24aa64e62ae7cb870d903aab6c4ac4f35b44da` |
| `fc_raw_ingress.c` | `e8ac7bedde5723b63864e08f69126da7f494e85c92138e07b9546136ab0a1bcf` |
| `fc_raw_ingress.h` | `6379bee367e162cdb5317dc0ae209b73243b99c6b39d02afc17414fe350ac74e` |
| `platform_gap.md` | `66da2a7623b4c07c9eb9efde3365f88ceb0596f0149ba34b77a70f475fc366df` |

## 3. 实际命令与输出

```text
> python applications/platform/rtthread53/tools/platform_inventory.py
{"requirements": 17, "missing": 6, "enable_needed": 7, "reuse": 3, "output": "applications/platform/rtthread53/evidence"}
[exit 0]

> pwsh -NoProfile -File applications/platform/rtthread53/tools/run_platform_tests.ps1
{"baseline":"V1.9.1-GLOBAL-AUDIT-R3","scope":"HOST_UNIT + ARM_COMPILE_ONLY",
 "production_target_validation":"NOT_RUN",
 "tests":[{"name":"raw_ingress","build_exit":0,"run_exit":0,
           "output":"raw_ingress host tests: 72 checks, 0 failures"},
          {"name":"time_service","build_exit":0,"run_exit":0,
           "output":"time_service host tests: 114 checks, 0 failures"}],
 "arm_compile":[{"name":"raw_ingress_c99","exit":0},
                {"name":"time_service_c99","exit":0}]}
[exit 0]
```

工具链（`evidence/toolchains.txt`）：

- ARM：`arm-none-eabi-gcc` 14.3.1 20250623（GNU Tools for STM32 14.3.rel1），
  flags `-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -Wall -Wextra -Werror -fno-short-enums -std=c99`。
- Host：`x86_64-w64-mingw32-gcc` 16.2.0，`-std=c11 -Wall -Wextra -Werror`。

## 4. 平台缺口矩阵摘要

完整表见 `applications/platform/rtthread53/evidence/platform_gap.md`。

| 状态 | 项 |
|---|---|
| REUSE | 系统时钟树（`board/board.c`）、TimeService 服务核、Raw ingress 服务核 |
| ENABLE_NEEDED | D-Cache（`BSP_SCB_ENABLE_D_CACHE` 未定义）、SPI1、I2C、USART2/3/4、PWM、SDMMC、OCTOSPI |
| STUB | NVIC 向量（`stm32h7xx_it.c` 调用 HAL，外设 init/优先级归属未核验） |
| MISSING | MPU 区域、TIM2 自由运行/64-bit 扩展、TIM2 CH3/CH4 捕获、双槽快照服务、`fc_os_*` RTOS 适配、DShot TC→Update |

现有 `.ioc` 已声明 TIM1/TIM2/SPI1/I2C1/2/4/USART1-3/UART4/SDMMC1/OCTOSPI1；`.config`
仅使能 `RT_USING_SERIAL` + UART1 控制台，SPI/I2C/PWM/SDIO/CAN 均为 `n`。

## 5. 与 B/C 的接口

- M2 可直接 link `fc_raw_ingress.c` 并调用公共 Port：
  `fc_sensor_acquire_raw(sensor_instance_id, &lease)` / `fc_sensor_release_raw(lease_id, generation)`；
  lease 只在复制 payload 期间有效，`payload_bytes` 指向 slot 内 64B CPU 可读区。
- M2/M3/M4/M6 通过 `fc_clock_read` / `fc_clock_now_us` / `fc_clock_ticks_to_mono_us`
  获取同一 `boot_id/time_epoch/monotonic` 时间轴；`fc_time_get_status` 分离本地时间与 UTC 状态。
- 板级只需提供 `FcTickSource` 与 raw lock hooks，不需要修改公共 ICD。

## 6. 未执行 / NOT_RUN

1. TIM2 初始化、CH3/CH4 捕获、DRDY→预建 SPI DMA 快路径、UIF 64-bit 扩展 ISR。
2. MPU region 表、D-Cache 使能与 clean/invalidate 实测；`board/` 未修改。
3. DShot TC→Update、TIM1 UP DMA burst、目标波形/时延。
4. 驱动 Kconfig/.config 使能、SPI1/I2C/UART2-4/SD/QSPI 板级绑定；`.ioc`/MSP 未修改。
5. 固件 `scons` 链接、map/WCET/栈/DMA 波形、SIL/HIL、目标板。
6. 双槽 latest-value 快照服务与 `fc_os_*` RTOS 适配（未在本阶段交付）。
7. Keil/armcc、IAR 编译。

## 7. 未决项 / 对 D 的依赖

1. `T191-A-ABI` 未 ACCEPTED，`T191-A-PLATFORM` 前置未满足：请 D 先记录 A-ABI 的审查/接单状态；
   若必须前置通过，请将本任务记 `BLOCKED` 并保留本提交为 resume 输入。
2. 09 台账未登记：本阶段只有 Host/ARM 编译证据；请 D 按 Gate B 适用项登记，
   不要把本报告当作 Gate B 或目标平台通过。
3. `platform_asset_scan.json` 的 `DOCUMENT_STATIC` 事实来自当前 `.config`/`.ioc`；
   若其他 Owner 并行修改这些文件，需重跑 inventory。
4. DShot/MPU/TIM2 的实现依赖真实 `.ioc`/MSP/TIM 句柄归属，必须在目标集成阶段与 D 的
   map/波形证据一起复核。

## 8. 回滚

- 无 Git 元数据，`commit=null`；以 `evidence/artifact_sha256.json` 追溯。
- 阶段1只新增目录；阶段2另修改 `board/SConscript`、`libraries/HAL_Drivers/drivers/drv_tim.c`、
  `rtconfig.py`、`applications/Kconfig`（见§10）。回滚删除：
  `applications/services/time/`、`applications/services/raw_ingress/`、
  `applications/platform/rtthread53/`（若其中出现其他 Agent 文件，只删本报告列出的文件）。
- 阶段1服务未接入 SCons/固件链接，删除后对现有构建无影响；阶段2 board 模块已注册到
  `board/SConscript`，回滚时需同时移除该行与 `drv_tim.c` 弱钩子。

## 9. 状态请求

见 `applications/platform/rtthread53/status_request_T191-A-PLATFORM.json`。
要点：`task_id=T191-A-PLATFORM`、`update_sequence=2`；请求 `READY_FOR_ASSIGNMENT→ASSIGNED`
（执行侧确认 prompt/基线），并记录前置 `T191-A-ABI` 未 ACCEPTED；本阶段
`verification_status=IN_PROGRESS`，不请求 ACCEPTED/CLOSED，不修改 HANDOFF/09。

## 10. 阶段2：TIM2/DRDY/MPU/DShot（A-ABI 已 ACCEPTED）

前置 `T191-A-ABI` 已 ACCEPTED/PASS（0x00010904，145 类型）。新增：

- `board/fc_board_platform.{h,c}`：
  - MPU：AXI 512KiB cacheable、D2 128KiB ordered/non-cacheable、外设 device、
    DTCM non-cacheable；随后开 I/D-Cache。
  - TIM2：32-bit free-running 10MHz（PSC 由实际 kernel 计算），CH3 PPS/CH4 DRDY
    输入捕获，UIF 64-bit 扩展、CCxOF 标记、32 项 capture ring；线程侧
    SR1→CNT→SR2 重试快照。DRDY 捕获调用弱钩子 `fc_board_imu_drdy_isr()`。
  - DShot：TIM1 4 通道 + DMA2_Stream2/TIM1_UP 4-word burst（68 words =
    bit1..15 + 2 zero tails），先 preload bit0 再 CEN；TC→Update 以 TIM2 绝对
    raw 阈值关断（`19×N` 覆盖启动+16 位+2 zero tail），异常计入 fault。
- `board/SConscript` 注册新文件；`drv_tim.c` 增加弱
  `fc_board_tim_period_elapsed` 钩子，由 board 的 TIM2/TIM1 更新逻辑覆盖。

Gate B 静态证据：`evidence/gate_b/gate_b_evidence.json`（16 checks PASS），
`python applications/platform/rtthread53/tools/gate_b_check.py`；ARM C99 编译
`fc_board_platform.c` exit 0；DShot 公式在 275MHz/300k 下 N=917、ARR=916、
T0H=344、T1H=688 校验通过。

验收证据包：`evidence/gate_b/GATE_B_EVIDENCE_PACKAGE.md` +
`acceptance_matrix.json`（12 项，PASS_STATIC/NOT_RUN 分列）、`symbols.txt`、
`sizes.txt`、`package_sha256.json`。全局符号确认 `htim1/htim2/hdma_tim1_up`
已定义、board 的 `fc_board_tim_period_elapsed` 为强符号、`fc_board_imu_drdy_isr`
为弱钩子；board 模块 `.text/.data/.bss = 3330/8/1457 B`。`drv_tim.c` 的
`flatency` 已改为 `uint32_t`；`BSP_USING_TIM` 未启用时由 board 提供
`HAL_TIM_PeriodElapsedCallback`，启用时由 drv_tim 调用同一钩子。

仍为 NOT_RUN：DRDY→预建 SPI1 DMA（`drv_spi` async 扩展缺失，仅弱钩子）、
MPU/Cache 目标压力、TIM2/PPS/DRDY 波形、DShot 实际位宽/零尾、完整 scons
链接/map/WCET/栈、SIL/HIL/目标板。CR-C02 入口正文已补到 06/08（安装钩子
`quad_x4_fms_configure_product_geofence`）。

状态请求更新为 revision 4、`update_sequence=7`、`ASSIGNED→IN_PROGRESS→REVIEW`；
不请求 ACCEPTED/CLOSED。
