/* Flight UART RX DMA buffers and counters shared by the STM32 serial drivers. */
#include <rtthread.h>
#include "drv_usart_dma.h"
#include "board_device.h"

#ifdef RT_USING_SERIAL_V2
static uint8_t g_uart2_rx[BSP_UART2_DMA_PING_BUFSIZE] DMA_BUFFER;
static uint8_t g_uart3_rx[BSP_UART3_DMA_PING_BUFSIZE] DMA_BUFFER;
static uint8_t g_uart4_rx[BSP_UART4_DMA_PING_BUFSIZE] DMA_BUFFER;
#else
static uint8_t g_uart2_rx[2048] DMA_BUFFER;
static uint8_t g_uart3_rx[512] DMA_BUFFER;
static uint8_t g_uart4_rx[2048] DMA_BUFFER;
#endif
static volatile UartStats g_stats[3];

static int link_index(USART_TypeDef *instance)
{
    return instance == USART2 ? 0 : (instance == USART3 ? 1 : (instance == UART4 ? 2 : -1));
}

#ifndef RT_USING_SERIAL_V2
uint32_t board_uart_rx_size(USART_TypeDef *instance)
{
    if (instance == USART2 || instance == UART4)
    {
        return 2048u;
    }
    if (instance == USART3 || instance == USART1)
    {
        return 512u;
    }
    return RT_SERIAL_RB_BUFSZ;
}
#endif

uint8_t *board_uart_rx_buffer(USART_TypeDef *instance, uint32_t size)
{
    if (instance == USART2 && size == sizeof(g_uart2_rx))
    {
        return g_uart2_rx;
    }
    if (instance == USART3 && size == sizeof(g_uart3_rx))
    {
        return g_uart3_rx;
    }
    if (instance == UART4 && size == sizeof(g_uart4_rx))
    {
        return g_uart4_rx;
    }
    return 0;
}

void board_uart_rx_event(USART_TypeDef *instance, uint32_t count, uint32_t lost)
{
    int i = link_index(instance);
    if (i >= 0)
    {
        g_stats[i].rx_bytes += count;
        g_stats[i].overwritten_bytes += lost;
    }
}

void board_uart_error_event(USART_TypeDef *instance, uint32_t flags)
{
    int i = link_index(instance);
    if (i >= 0 && flags)
    {
        g_stats[i].error_events++;
    }
}

int uart_get_stats(USART_TypeDef *instance, UartStats *out)
{
    int i = link_index(instance);
    if (i < 0 || !out)
    {
        return -RT_EINVAL;
    }
    *out = g_stats[i];
    return RT_EOK;
}
