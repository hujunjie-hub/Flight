/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 电调输出硬件配置 —— 【全部为占位值, 硬件方案落地后改本文件】
 *
 * ---------------------------------------------------------------------------
 * !!! 占位声明 (2026-10-03) !!!
 * ---------------------------------------------------------------------------
 * 电机输出引脚尚未在 doc/Flight.xlsx 引脚表 / Flight.ioc 中分配,
 * 以下 TIM4 + PB6..PB9 + DMA1 Stream5 为**随意选定的空闲资源占位**
 * (选点依据: PB6/PB7 因 BMM350 迁 I2C4 腾空, PB8/PB9 未用, TIM4 空闲,
 *  DMA1 Stream5 空闲 —— 见 libraries/HAL_Drivers/drivers/config/h7/
 *  dma_config.h 的板级映射表)。硬件方案落地后:
 *    1. 改本文件的引脚/通道/AF/DMA 宏;
 *    2. 若移出 TIM4, 同步改 DSHOT_TIM / 时钟使能与 DCR 基址计算
 *       (代码按 TIM4 写死的部分 grep "TIM4");
 *    3. 在 Flight.ioc 补对应外设时, 重生成后检查 board/CubeMX_Config/
 *       Src/stm32h7xx_hal_msp.c 不要出现与本驱动冲突的 TIM4/GPIO 段
 *       (同 SPI1 DMA 先例, 见 middleware/README);
 *    4. 板级 board.c 的 NVIC 强表按需补 DMA 流中断行 (当前引擎轮询
 *       DMA 完成态, 未用中断)。
 * ---------------------------------------------------------------------------
 */
#ifndef __DSHOT_HW_H__
#define __DSHOT_HW_H__

/* ---------------- 定时器与引脚 (占位) ---------------- */
#define DSHOT_TIM                TIM4
#define DSHOT_TIM_CLK_ENABLE()   __HAL_RCC_TIM4_CLK_ENABLE()
#define DSHOT_TIM_CLK_HZ         275000000UL /* APB1 定时器时钟 (D2PPRE1=DIV2, 与 TIM2 同域) */

/* 输出通道数与引脚: m0 FR / m1 FL / m2 RR / m3 RL (与 mixer 布局一致) */
#define DSHOT_CH_NUM             4

/* 占位引脚表: {GPIOx, PIN, TIM通道, AF} */
#define DSHOT_PIN_AF             GPIO_AF2_TIM4

/* ---------------- DMA (占位): TIM4_UP -> DMAR 突发装载 CCR1..4 ---------- */
#define DSHOT_DMA_INSTANCE       DMA1_Stream5
#define DSHOT_DMA_RCC            RCC_AHB1ENR_DMA1EN
#define DSHOT_DMA_REQUEST        DMA_REQUEST_TIM4_UP   /* 32, DMAMUX1 */
#define DSHOT_DMA_IRQ            DMA1_Stream5_IRQn
#define DSHOT_DMA_PREEMPT        4      /* 占位: 高于业务线程, 低于传感器 DMA(3) */

/* ---------------- PWM 协议参数 (占位可调) ---------------- */
#define DSHOT_PWM_RATE_HZ        400    /* PWM 更新率 */
#define DSHOT_PWM_US_MIN         1000.0 /* u=0 脉宽 us */
#define DSHOT_PWM_US_MAX         2000.0 /* u=1 脉宽 us */

#endif /* __DSHOT_HW_H__ */
