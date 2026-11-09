# T191-A-PLATFORM gap matrix (independent preparation)

- Baseline: `V1.9.1-GLOBAL-AUDIT-R3`
- Scope: static inventory of existing `board/`, `libraries/HAL_Drivers/`, `.config`, `.ioc`, IT handlers
- Production target validation: **NOT_RUN** (no board build/waveform/cache test in this task)

| ID | Requirement | Evidence | Status | Next action |
|---|---|---|---|---|
| SYS-CLOCK | System clock tree / HSE / PLL | `board/board.c` | **REUSE** | reuse SystemClock_Config; target clock evidence NOT_RUN |
| SYS-MPU | MPU regions (DMA non-cacheable/executable attrs) | `board/fc_board_platform.c, 03 section 6/7` | **REUSE** | static regions added; target MPU/cache stress NOT_RUN |
| SYS-CACHE | D-Cache enable + maintenance | `board/fc_board_platform.c` | **REUSE** | cache enabled after MPU; target stress NOT_RUN |
| TIME-TICK | TIM2 32-bit free-running 10 MHz + UIF 64-bit extension | `board/fc_board_platform.c` | **REUSE** | source implemented; target waveform NOT_RUN |
| TIME-CAP | TIM2 CH4 DRDY / CH3 PPS capture | `board/fc_board_platform.c` | **REUSE** | capture implemented; DRDY pre-armed SPI DMA still NOT_RUN |
| TIME-SVC | Monotonic/PPS/UTC TimeService core | `applications/services/time` | **REUSE** | host-tested; board tick source binding pending |
| RAW-SVC | Raw ingress lease ring (160B slots) | `applications/services/raw_ingress` | **REUSE** | host-tested; DMA publish binding pending |
| SNAP-SVC | Double-slot latest-value snapshot protocol | `03 cross-task snapshot` | **MISSING** | not delivered in this slice |
| OS-ADP | RTOS adapter fc_os_* static task/IPC API | `03 section 5.2` | **MISSING** | not delivered in this slice |
| BUS-SPI | SPI1 IMU bus + async DMA fast path | `drv_spi.c / .ioc` | **ENABLE_NEEDED** | driver exists; Kconfig/board binding not enabled |
| BUS-I2C | I2C1/I2C2 baro/mag | `drv_hard_i2c.c` | **ENABLE_NEEDED** | driver exists; Kconfig/board binding not enabled |
| LINK-UART | USART2/3/4 RC/GCS/GNSS + DMA rings | `drv_usart.c / .ioc` | **ENABLE_NEEDED** | only UART1 console enabled; other links are design inputs |
| OUT-DSHOT | TIM1 UP DMA burst + DShot TC->Update | `board/fc_board_platform.c` | **REUSE** | source implemented; target waveform NOT_RUN |
| OUT-PWM | PWM output fallback | `drv_pwm.c` | **ENABLE_NEEDED** | driver exists; not enabled |
| STO-SD | SDMMC1 IDMA storage | `drv_sdmmc.c` | **ENABLE_NEEDED** | driver exists; not enabled |
| STO-NOR | OCTOSPI1 W25Q64 | `drv_qspi.c` | **ENABLE_NEEDED** | driver exists; not enabled |
| IRQ-TABLE | NVIC vector/priority ownership | `stm32h7xx_it.c` | **STUB** | handlers call HAL with CubeMX handles; peripheral init/priority ownership not verified |

## Factual inputs

- Drivers present: 31 `.c` files under `libraries/HAL_Drivers/drivers`.
- Enabled RT-Thread bus config: SPI=n, I2C=n, PWM=n, SDIO=n, CAN=n.
- `.ioc` peripherals declared: I2C1, I2C2, I2C4, OCTOSPI1, SDMMC1, SPI1, TIM1, TIM2, UART4, USART1, USART2, USART3.
- IRQ handlers present: 28.

## Delivered in this slice (services, host-tested)

- `applications/services/time` - monotonic mapping + PPS discipline + UTC mapping.
- `applications/services/raw_ingress` - 160B raw slot lease ring for M2.

## Not executed

- Kconfig/.config driver enablement, board MPU/D-Cache programming, TIM2/DRIVE/PPS capture,
  DShot TC->Update, SD/NOR storage, and any target build/waveform/cache stress test.
- Every target-dependent row must still pass Gate B/E on the real board.
