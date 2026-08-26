/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-01-02     zylx         first version
 * 2019-01-08     SummerGift   clean up the code
 * 2020-05-02     whj4674672   support stm32h7 dma1 and dma2
 * 2026-08-26     Flight       customized DMA for navigation system
 */

#ifndef __DMA_CONFIG_H__
#define __DMA_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Customized DMA Stream Allocation for Navigation System
 *
 * DMA1_Stream0: SPI1 RX (ADIS16505 IMU)
 * DMA1_Stream1: SPI1 TX (ADIS16505 IMU)
 * DMA1_Stream2: USART2 RX (UM982 GNSS, Circular mode)
 * DMA1_Stream3: USART1 RX (Debug Serial)
 * DMA1_Stream4: USART1 TX (Debug Serial)
 */

/* DMA1 stream0 - SPI1 RX (IMU) */
#if defined(BSP_SPI1_RX_USING_DMA) && !defined(SPI1_RX_DMA_INSTANCE)
#define SPI1_DMA_RX_IRQHandler           DMA1_Stream0_IRQHandler
#define SPI1_RX_DMA_RCC                  RCC_AHB1ENR_DMA1EN
#define SPI1_RX_DMA_INSTANCE             DMA1_Stream0
#define SPI1_RX_DMA_IRQ                  DMA1_Stream0_IRQn
#endif

/* DMA1 stream1 - SPI1 TX (IMU) */
#if defined(BSP_SPI1_TX_USING_DMA) && !defined(SPI1_TX_DMA_INSTANCE)
#define SPI1_DMA_TX_IRQHandler           DMA1_Stream1_IRQHandler
#define SPI1_TX_DMA_RCC                  RCC_AHB1ENR_DMA1EN
#define SPI1_TX_DMA_INSTANCE             DMA1_Stream1
#define SPI1_TX_DMA_IRQ                  DMA1_Stream1_IRQn
#endif

/* DMA1 stream2 - USART2 RX (GNSS) */
#if defined(BSP_UART2_RX_USING_DMA) && !defined(UART2_RX_DMA_INSTANCE)
#define UART2_DMA_RX_IRQHandler          DMA1_Stream2_IRQHandler
#define UART2_RX_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define UART2_RX_DMA_INSTANCE            DMA1_Stream2
#define UART2_RX_DMA_REQUEST             DMA_REQUEST_USART2_RX
#define UART2_RX_DMA_IRQ                 DMA1_Stream2_IRQn
#endif

/* DMA1 stream3 - USART1 RX (Debug) */
#if defined(BSP_UART1_RX_USING_DMA) && !defined(UART1_RX_DMA_INSTANCE)
#define UART1_DMA_RX_IRQHandler          DMA1_Stream3_IRQHandler
#define UART1_RX_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define UART1_RX_DMA_INSTANCE            DMA1_Stream3
#define UART1_RX_DMA_REQUEST             DMA_REQUEST_USART1_RX
#define UART1_RX_DMA_IRQ                 DMA1_Stream3_IRQn
#endif

/* DMA1 stream4 - USART1 TX (Debug) */
#if defined(BSP_UART1_TX_USING_DMA) && !defined(UART1_TX_DMA_INSTANCE)
#define UART1_DMA_TX_IRQHandler          DMA1_Stream4_IRQHandler
#define UART1_TX_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define UART1_TX_DMA_INSTANCE            DMA1_Stream4
#define UART1_TX_DMA_REQUEST             DMA_REQUEST_USART1_TX
#define UART1_TX_DMA_IRQ                 DMA1_Stream4_IRQn
#endif

/* DMA2 stream0 - SPI4 TX */
#if defined(BSP_SPI4_TX_USING_DMA) && !defined(SPI4_TX_DMA_INSTANCE)
#define SPI4_DMA_TX_IRQHandler           DMA2_Stream0_IRQHandler
#define SPI4_TX_DMA_RCC                  RCC_AHB1ENR_DMA2EN
#define SPI4_TX_DMA_INSTANCE             DMA2_Stream0
#define SPI4_TX_DMA_IRQ                  DMA2_Stream0_IRQn
#endif

/* DMA2 stream1 - SPI4 RX */
#if defined(BSP_SPI4_RX_USING_DMA) && !defined(SPI4_RX_DMA_INSTANCE)
#define SPI4_DMA_RX_IRQHandler           DMA2_Stream1_IRQHandler
#define SPI4_RX_DMA_RCC                  RCC_AHB1ENR_DMA2EN
#define SPI4_RX_DMA_INSTANCE             DMA2_Stream1
#define SPI4_RX_DMA_IRQ                  DMA2_Stream1_IRQn
#endif

/* DMA2 stream2 - SPI5 RX */
#if defined(BSP_SPI5_RX_USING_DMA) && !defined(SPI5_RX_DMA_INSTANCE)
#define SPI5_DMA_RX_IRQHandler           DMA2_Stream2_IRQHandler
#define SPI5_RX_DMA_RCC                  RCC_AHB1ENR_DMA2EN
#define SPI5_RX_DMA_INSTANCE             DMA2_Stream2
#define SPI5_RX_DMA_IRQ                  DMA2_Stream2_IRQn
#endif

/* DMA2 stream3 - SPI5 TX */
#if defined(BSP_SPI5_TX_USING_DMA) && !defined(SPI5_TX_DMA_INSTANCE)
#define SPI5_DMA_TX_IRQHandler           DMA2_Stream3_IRQHandler
#define SPI5_TX_DMA_RCC                  RCC_AHB1ENR_DMA2EN
#define SPI5_TX_DMA_INSTANCE             DMA2_Stream3
#define SPI5_TX_DMA_IRQ                  DMA2_Stream3_IRQn
#endif

/* DMA2 stream4 - SPI6 RX */
#if defined(BSP_SPI6_RX_USING_DMA) && !defined(SPI6_RX_DMA_INSTANCE)
#define SPI6_DMA_RX_IRQHandler           DMA2_Stream4_IRQHandler
#define SPI6_RX_DMA_RCC                  RCC_AHB1ENR_DMA2EN
#define SPI6_RX_DMA_INSTANCE             DMA2_Stream4
#define SPI6_RX_DMA_IRQ                  DMA2_Stream4_IRQn
#endif

/* DMA2 stream5 - SPI6 TX */
#if defined(BSP_SPI6_TX_USING_DMA) && !defined(SPI6_TX_DMA_INSTANCE)
#define SPI6_DMA_TX_IRQHandler           DMA2_Stream5_IRQHandler
#define SPI6_TX_DMA_RCC                  RCC_AHB1ENR_DMA2EN
#define SPI6_TX_DMA_INSTANCE             DMA2_Stream5
#define SPI6_TX_DMA_IRQ                  DMA2_Stream5_IRQn
#endif

/* DMA2 stream6 - USART2 TX */
#if defined(BSP_UART2_TX_USING_DMA) && !defined(UART2_TX_DMA_INSTANCE)
#define UART2_DMA_TX_IRQHandler          DMA2_Stream6_IRQHandler
#define UART2_TX_DMA_RCC                 RCC_AHB1ENR_DMA2EN
#define UART2_TX_DMA_INSTANCE            DMA2_Stream6
#define UART2_TX_DMA_REQUEST             DMA_REQUEST_USART2_TX
#define UART2_TX_DMA_IRQ                 DMA2_Stream6_IRQn
#endif

/* DMA2 stream7 - QSPI */
#if defined(BSP_QSPI_USING_DMA) && !defined(QSPI_DMA_INSTANCE)
#define QSPI_DMA_IRQHandler              DMA2_Stream7_IRQHandler
#define QSPI_DMA_RCC                     RCC_AHB1ENR_DMA2EN
#define QSPI_DMA_INSTANCE                DMA2_Stream7
#define QSPI_DMA_IRQ                     DMA2_Stream7_IRQn
#endif

#ifdef __cplusplus
}
#endif

#endif /* __DMA_CONFIG_H__ */
