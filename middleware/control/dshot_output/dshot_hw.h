/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 电调输出硬件配置 —— 2026-10-04 硬件定案 (doc/Flight.xlsx 接口配置页)
 *
 * ---------------------------------------------------------------------------
 * 定案引脚: TIM1 (APB2, 定时器时钟 275MHz, 与占位期 TIM4 同为 275MHz,
 * 时序参数无需改动) 四路输出:
 *   m0 FR -> TIM1_CH1 = PE9
 *   m1 FL -> TIM1_CH2 = PE11
 *   m2 RR -> TIM1_CH3 = PE13 (原 BMP585 INT, 气压计已迁 PE1)
 *   m3 RL -> TIM1_CH4 = PE14
 * Flight.ioc 已收录 TIM1 四通道 PWM (无 DMA/无中断), 重生成后核对
 * stm32h7xx_hal_msp.c 的 HAL_TIM_MspPostInit 段引脚与本表一致即可;
 * TIM1 寄存器级时序 (DShot DMAR 突发/PWM CCR) 仍由 dshot.c 自管,
 * 与 SPI1 DMA 同理: 禁止外部代码再对 TIM1 做 HAL_TIM_Init。
 *
 * DMA: TIM1_UP -> DMA1_Stream5 (DMAMUX1 request 15), 引擎轮询完成态
 * 不用中断, board.c NVIC 强表无需 DMA1_Stream5 行。
 * ---------------------------------------------------------------------------
 */
#ifndef __DSHOT_HW_H__
#define __DSHOT_HW_H__

/* ---------------- 定时器与引脚 ---------------- */
#define DSHOT_TIM                TIM1
#define DSHOT_TIM_CLK_ENABLE()   __HAL_RCC_TIM1_CLK_ENABLE()
#define DSHOT_TIM_CLK_HZ         275000000UL /* APB2 定时器时钟 (D2PPRE2=DIV2, 与原 TIM4 方案同为 275MHz) */

/* 输出通道数与引脚: m0 FR / m1 FL / m2 RR / m3 RL (与 mixer 布局一致) */
#define DSHOT_CH_NUM             4

/* 引脚表: {GPIOx, PIN, TIM通道, AF} (AF1_TIM1) */
#define DSHOT_PIN_AF             GPIO_AF1_TIM1

/* ---------------- DMA: TIM1_UP -> DMAR 突发装载 CCR1..4 ---------------- */
#define DSHOT_DMA_INSTANCE       DMA1_Stream5
#define DSHOT_DMA_RCC            RCC_AHB1ENR_DMA1EN
#define DSHOT_DMA_REQUEST        DMA_REQUEST_TIM1_UP   /* 15, DMAMUX1 */
#define DSHOT_DMA_IRQ            DMA1_Stream5_IRQn     /* 引擎轮询, 实际不使能 */
#define DSHOT_DMA_PREEMPT        4      /* 高于业务线程, 低于传感器 DMA(3) */

/* ---------------- PWM 协议参数 ---------------- */
#define DSHOT_PWM_RATE_HZ        400    /* PWM 更新率 */
#define DSHOT_PWM_US_MIN         1000.0 /* u=0 脉宽 us */
#define DSHOT_PWM_US_MAX         2000.0 /* u=1 脉宽 us */

#endif /* __DSHOT_HW_H__ */
