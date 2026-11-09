# A-PLATFORM stage-2 Gate B evidence package

- Baseline: `V1.9.1-GLOBAL-AUDIT-R3` / Schema 6.0 / Product-Port `0x00010904`
- Scope: static implementation + ARM Cortex-M7 compile/symbols; target validation **NOT_RUN**
- Verdict: **PASS_STATIC**; recommendation: `ACCEPT_STATIC_WITH_TARGET_NOT_RUN`

## Commands

```text
D:\STM32CubeCLT\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -std=c99 -ffunction-sections -fdata-sections -DSTM32H723xx -DUSE_HAL_DRIVER -Dgcc -Wall -Wextra -Werror -fno-short-enums -ID:\STM32Project\FMT-Firmware-1.2.0\Flight -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\board -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\board\CubeMX_Config\Inc -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\libraries\HAL_Drivers -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\libraries\HAL_Drivers\drivers -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\libraries\HAL_Drivers\drivers\config -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\libraries\HAL_Drivers\CMSIS\Include -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\packages\stm32h7_hal_driver-latest\Inc -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\packages\stm32h7_cmsis_driver-latest\Include -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\packages\CMSIS-Core-latest\Include -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\rt-thread\include -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\rt-thread\components\drivers\include -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\rt-thread\components\finsh -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\rt-thread\libcpu\arm\cortex-m7 -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\applications\contracts -c D:\STM32Project\FMT-Firmware-1.2.0\Flight\board\fc_board_platform.c -o D:\STM32Project\FMT-Firmware-1.2.0\Flight\applications\platform\rtthread53\evidence\gate_b\fc_board_platform.o
D:\STM32CubeCLT\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -std=c99 -ffunction-sections -fdata-sections -DSTM32H723xx -DUSE_HAL_DRIVER -Dgcc -Wall -Wextra -fno-short-enums -ID:\STM32Project\FMT-Firmware-1.2.0\Flight -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\board -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\board\CubeMX_Config\Inc -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\libraries\HAL_Drivers -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\libraries\HAL_Drivers\drivers -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\libraries\HAL_Drivers\drivers\config -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\libraries\HAL_Drivers\CMSIS\Include -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\packages\stm32h7_hal_driver-latest\Inc -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\packages\stm32h7_cmsis_driver-latest\Include -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\packages\CMSIS-Core-latest\Include -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\rt-thread\include -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\rt-thread\components\drivers\include -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\rt-thread\components\finsh -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\rt-thread\libcpu\arm\cortex-m7 -ID:\STM32Project\FMT-Firmware-1.2.0\Flight\applications\contracts -c D:\STM32Project\FMT-Firmware-1.2.0\Flight\libraries\HAL_Drivers\drivers\drv_tim.c -o D:\STM32Project\FMT-Firmware-1.2.0\Flight\applications\platform\rtthread53\evidence\gate_b\drv_tim.o
python applications/platform/rtthread53/tools/gate_b_check.py
```

## Symbols / sizes

- Missing/wrong board symbols: `[]`
- `fc_board_tim_period_elapsed`: board=`T`, drv_tim=`None`
- Board `.text/.data/.bss`: `{'text': 3330, 'data': 8, 'bss': 1457}`
- drv_tim `.text/.data/.bss`: `{'text': 909, 'data': 0, 'bss': 0}`

## Acceptance matrix

| ID | Test ID | Item | Status | Evidence/gap |
|---|---|---|---|---|
| GB-01 | TEST-HW-IRQ-001 | Pin/AF owner (PA2/PA3 TIM2, PE9/11/13/14 TIM1) | **PASS_STATIC** | CubeMX MSP + .ioc; target pin measurement NOT_RUN |
| GB-02 | TEST-HW-DMA-002 | DMAMUX TIM1_UP -> DMA2_Stream2 | **PASS_STATIC** | MSP hdma_tim1_up/DMA_REQUEST_TIM1_UP; target transfer NOT_RUN |
| GB-03 | TEST-HW-DMA-003 | DMA stream uniqueness | **PASS_STATIC** | DMA2_Stream2 only for TIM1_UP; target dump NOT_RUN |
| GB-04 | TEST-HW-IRQ-004 | NVIC mapping (TIM2=1, TIM1_UP=2) | **PASS_STATIC** | MSP priorities; runtime register dump NOT_RUN |
| GB-05 | TEST-HW-IRQ-005 | ISR API policy (no RTOS API in TIM2/TIM1/DMA ISR) | **PASS_STATIC** | source scan of callbacks; target trace NOT_RUN |
| GB-06 | TEST-HW-DMA-006 | MPU/D-Cache (D2 non-cacheable) | **PASS_STATIC** | fc_board_mpu_cache_init; cache stress NOT_RUN |
| GB-07 | TEST-HW-IMU-007 | ADIS burst DRDY capture (TIM2 CH4) | **PASS_STATIC** | capture ring/64-bit snapshot; SPI pre-armed DMA path NOT_RUN |
| GB-08 | TEST-HW-SAFE-013 | DShot TIM1 + DMA burst + TC->Update stop | **PASS_STATIC** | code/静态 checks; target waveform NOT_RUN |
| GB-09 | TEST-RT-003/4/7 | map/WCET/RTA/stack/critical section | **NOT_RUN** | no scons/SCons available; no target build |
| GB-10 | TEST-RT-010/11 | DMA reachability/cache stress | **NOT_RUN** | no target board |
| GB-11 | TEST-HW-GNSS-008 | PPS capture / timestamp accuracy | **NOT_RUN** | CH3 configured; no PPS source/waveform |
| GB-12 | TEST-RT-018 | Worst-case combined load / HIL | **NOT_RUN** | no HIL |

## Limitations (must not be read as PASS)

- No `scons`/SCons in this environment: no firmware link, `.map`, WCET/RTA, stack watermark.
- `drv_tim.c` was corrected (`flatency` -> `uint32_t`) and now compiles clean; its
  `fc_board_tim_period_elapsed` hook is dormant while `BSP_USING_TIM` is disabled, and the
  board module owns `HAL_TIM_PeriodElapsedCallback` in that configuration.
- DRDY SPI fast path is only the weak `fc_board_imu_drdy_isr()` hook; the pre-armed `drv_spi`
  extension is not implemented.
- TIM2 capture/HAL IRQ ordering, DShot bit timing/zero-tail and MPU/cache coherency are
  target-verification items.

## Files

- `board/fc_board_platform.c` — `f0b3f939dafd4ace6a66f98333abd46ed65228142f3ea50edfb4b69a3209a4f6`
- `board/fc_board_platform.h` — `e84d4f4059fe39192fa106c14fa251a1d3caf5a516c157d76d1e64937df82c3d`
- `board/SConscript` — `64e6388bfc18a80f76711f6c892bb18559c057ba40194be101a52fb19b9fb7bc`
- `libraries/HAL_Drivers/drivers/drv_tim.c` — `9ff4c69234a106320b6a1eb17151f7847c5bebcacb445fde06a2895ee830cbee`
- `applications/platform/rtthread53/evidence/gate_b/gate_b_evidence.json` — `f3ee1832258cdf0542b9a5680e24fbd63cddcda6def2fb0dcc7f31bb27e4f172`
- `applications/platform/rtthread53/evidence/gate_b/gate_b_report.md` — `8cb8f54ba8c5f846c45ec9f81408409c2a36682ce3f72a9a0296f07a59ebe3dd`
- `applications/platform/rtthread53/evidence/gate_b/acceptance_matrix.json` — `c83c3e8b638d3c633f007d911d1f25123d3a8f3b2cb37cccd62f8c7ff66234b5`
- `applications/platform/rtthread53/evidence/gate_b/symbols.txt` — `cf28b2176f9318b39945094b5ef895186110e3546235d763f257c3034a6401ff`
- `applications/platform/rtthread53/evidence/gate_b/sizes.txt` — `16bad3c99dd8ea66954cb270f6bc1bb0dfa69275c9e0d4b47c05fcf0fc0211d9`
