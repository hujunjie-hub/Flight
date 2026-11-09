/* Flight DMA hooks and link statistics for native STM32 UART devices. */
#ifndef DRV_USART_DMA_H
#define DRV_USART_DMA_H

#include <stdint.h>
#ifndef FLIGHT_HOST_TEST
#include <stm32h7xx.h>
#endif

uint8_t *board_uart_rx_buffer(USART_TypeDef *instance, uint32_t size);
#ifndef RT_USING_SERIAL_V2
uint32_t board_uart_rx_size(USART_TypeDef *instance);
#endif
void board_uart_rx_event(USART_TypeDef *instance, uint32_t count, uint32_t lost);
void board_uart_error_event(USART_TypeDef *instance, uint32_t flags);

typedef struct
{
    uint32_t rx_bytes;
    uint32_t overwritten_bytes;
    uint32_t error_events;
} UartStats;

int uart_get_stats(USART_TypeDef *instance, UartStats *out);

#endif /* DRV_USART_DMA_H */
