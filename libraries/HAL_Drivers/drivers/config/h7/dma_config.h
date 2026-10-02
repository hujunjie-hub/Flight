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
 * 2026-xx-xx     Flight BSP   align DMA1/DMA2 streams with board/CubeMX_Config/Flight.ioc
 */

#ifndef __DMA_CONFIG_H__
#define __DMA_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * STM32H723 Flight board DMA mapping (see board/CubeMX_Config/Flight.ioc):
 *   DMA1 Stream0 : SPI1_RX   (ADIS16505 DOUT)      IRQ priority 3
 *   DMA1 Stream1 : SPI1_TX   (ADIS16505 DIN)       IRQ priority 3
 *   DMA1 Stream2 : USART2_RX (UM982 GNSS, circ.)   IRQ priority 3
 *   DMA1 Stream3 : USART1_RX (debug console)       IRQ priority 5
 *   DMA1 Stream4 : USART1_TX (debug console)       IRQ priority 5
 *   DMA2 Stream0 : I2C2_RX   (BMP585 barometer)    IRQ priority 6
 *   DMA2 Stream1 : I2C2_TX   (BMP585 barometer)    IRQ priority 6
 */

/* DMA1 stream0 */
#if defined(BSP_SPI1_RX_USING_DMA) && !defined(SPI1_RX_DMA_INSTANCE)
#define SPI1_DMA_RX_IRQHandler           DMA1_Stream0_IRQHandler
#define SPI1_RX_DMA_RCC                  RCC_AHB1ENR_DMA1EN
#define SPI1_RX_DMA_INSTANCE             DMA1_Stream0
#define SPI1_RX_DMA_REQUEST              DMA_REQUEST_SPI1_RX
#define SPI1_RX_DMA_IRQ                  DMA1_Stream0_IRQn
#endif

/* DMA1 stream1 */
#if defined(BSP_SPI1_TX_USING_DMA) && !defined(SPI1_TX_DMA_INSTANCE)
#define SPI1_DMA_TX_IRQHandler           DMA1_Stream1_IRQHandler
#define SPI1_TX_DMA_RCC                  RCC_AHB1ENR_DMA1EN
#define SPI1_TX_DMA_INSTANCE             DMA1_Stream1
#define SPI1_TX_DMA_REQUEST              DMA_REQUEST_SPI1_TX
#define SPI1_TX_DMA_IRQ                  DMA1_Stream1_IRQn
#endif

/* DMA1 stream2 */
#if defined(BSP_UART2_RX_USING_DMA) && !defined(UART2_RX_DMA_INSTANCE)
#define UART2_DMA_RX_IRQHandler          DMA1_Stream2_IRQHandler
#define UART2_RX_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define UART2_RX_DMA_INSTANCE            DMA1_Stream2
#define UART2_RX_DMA_REQUEST             DMA_REQUEST_USART2_RX
#define UART2_RX_DMA_IRQ                 DMA1_Stream2_IRQn
#endif

/* DMA1 stream3 */
#if defined(BSP_UART1_RX_USING_DMA) && !defined(UART1_RX_DMA_INSTANCE)
#define UART1_DMA_RX_IRQHandler          DMA1_Stream3_IRQHandler
#define UART1_RX_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define UART1_RX_DMA_INSTANCE            DMA1_Stream3
#define UART1_RX_DMA_REQUEST             DMA_REQUEST_USART1_RX
#define UART1_RX_DMA_IRQ                 DMA1_Stream3_IRQn
#endif

/* DMA1 stream4 */
#if defined(BSP_UART1_TX_USING_DMA) && !defined(UART1_TX_DMA_INSTANCE)
#define UART1_DMA_TX_IRQHandler          DMA1_Stream4_IRQHandler
#define UART1_TX_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define UART1_TX_DMA_INSTANCE            DMA1_Stream4
#define UART1_TX_DMA_REQUEST             DMA_REQUEST_USART1_TX
#define UART1_TX_DMA_IRQ                 DMA1_Stream4_IRQn
#endif

/* DMA2 stream0 */
#if defined(BSP_I2C2_RX_USING_DMA) && !defined(I2C2_RX_DMA_INSTANCE)
#define I2C2_DMA_RX_IRQHandler           DMA2_Stream0_IRQHandler
#define I2C2_RX_DMA_RCC                  RCC_AHB1ENR_DMA2EN
#define I2C2_RX_DMA_INSTANCE             DMA2_Stream0
#define I2C2_RX_DMA_REQUEST              DMA_REQUEST_I2C2_RX
#define I2C2_RX_DMA_IRQ                  DMA2_Stream0_IRQn
#endif

/* DMA2 stream1 */
#if defined(BSP_I2C2_TX_USING_DMA) && !defined(I2C2_TX_DMA_INSTANCE)
#define I2C2_DMA_TX_IRQHandler           DMA2_Stream1_IRQHandler
#define I2C2_TX_DMA_RCC                  RCC_AHB1ENR_DMA2EN
#define I2C2_TX_DMA_INSTANCE             DMA2_Stream1
#define I2C2_TX_DMA_REQUEST              DMA_REQUEST_I2C2_TX
#define I2C2_TX_DMA_IRQ                  DMA2_Stream1_IRQn
#endif

#ifdef __cplusplus
}
#endif

#endif /* __DMA_CONFIG_H__ */
