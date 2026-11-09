# A-PLATFORM stage 2 Gate B static evidence

- Baseline: `V1.9.1-GLOBAL-AUDIT-R3` / Schema 6.0 / Product-Port `0x00010904`
- Scope: source wiring + ARM compile of `board/fc_board_platform.c`; target waveform/HIL NOT_RUN
- Verdict: **PASS_STATIC**

| Gate B item | Status | Evidence |
|---|---|---|
| BSP clock/source init reuse | **PASS_STATIC** | board.c SystemClock_Config + CubeMX MSP; target readback NOT_RUN |
| IRQ/NVIC mapping | **PASS_STATIC** | MSP TIM2=1, TIM1_UP=2, DMA2_Stream2; target dump NOT_RUN |
| DMA2_Stream2 TIM1_UP mapping | **PASS_STATIC** | MSP hdma_tim1_up + .ioc; target transfer NOT_RUN |
| TIM2 free-running/capture/64-bit extension | **PASS_STATIC** | fc_board_platform.c; target waveform NOT_RUN |
| MPU/D-Cache | **PASS_STATIC** | four regions + D2 non-cacheable + SCB_EnableDCache; cache stress NOT_RUN |
| DShot TC->Update / DMA burst | **PASS_STATIC** | TIM1 4ch + DMA burst + absolute stop gate; waveform NOT_RUN |
| DRDY -> pre-armed SPI1 DMA | **NOT_RUN** | weak fc_board_imu_drdy_isr hook only; drv_spi async extension missing |
| map/WCET/stack/RTA | **NOT_RUN** | no target build |
| HIL / board waveform | **NOT_RUN** | no hardware |

## Checks

| Check | Status |
|---|---|
| board_source_registered | **PASS** |
| handles_defined | **PASS** |
| irq_handlers_exist | **PASS** |
| tim2_priority_1 | **PASS** |
| tim1_up_priority_2 | **PASS** |
| dma2_stream2_tim1_up | **PASS** |
| tim2_32bit_free_running | **PASS** |
| tim2_capture_ch3_ch4 | **PASS** |
| tim2_64bit_retry | **PASS** |
| mpu_d2_noncacheable | **PASS** |
| dshot_dma_burst | **PASS** |
| dshot_stop_threshold | **PASS** |
| drdy_hook_present | **PASS** |
| drv_tim_hook_present | **PASS** |
| dshot_timing_275mhz_300k | **PASS** |
| arm_compile | **PASS** |

## Not proved

- No firmware link/map, WCET/RTA, stack watermark, MPU/cache stress test, TIM2 capture
  waveform, DShot bit timing/zero-tail, DRDY-to-SPI DMA or HIL/flight evidence.
- The DRDY fast path is only a weak hook; the `drv_spi` pre-armed async extension is still
  missing and must be implemented/validated on target before Gate B can close.
