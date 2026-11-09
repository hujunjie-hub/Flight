#include "um982.h"
#include "board_device.h"
#if defined(BSP_USING_FLIGHT_DRIVERS)
#include <board.h>
#endif

rt_device_t um982_open(void)
{
    struct serial_configure config;
    rt_device_t device = rt_device_find(GNSS_UART_NAME);
    if (!device || rt_device_control(device, RT_SERIAL_CTRL_GET_CONFIG, &config) != RT_EOK)
    {
        return RT_NULL;
    }
#if defined(BSP_USING_FLIGHT_DRIVERS)
    const rt_size_t tx_buffer_size = 2048u;
    if (config.baud_rate != 460800u || config.tx_bufsz != tx_buffer_size)
#else
    if (config.baud_rate != 460800u)
#endif
    {
        config.baud_rate = 460800u;
#if defined(BSP_USING_FLIGHT_DRIVERS)
        config.tx_bufsz = tx_buffer_size;
#endif
        if (rt_device_control(device, RT_DEVICE_CTRL_CONFIG, &config) != RT_EOK)
        {
            return RT_NULL;
        }
    }
#if defined(BSP_USING_FLIGHT_DRIVERS)
    const rt_uint16_t tx_mode = RT_SERIAL_TX_NON_BLOCKING;
#else
    const rt_uint16_t tx_mode = RT_SERIAL_TX_BLOCKING;
#endif
    return rt_device_open(device, RT_DEVICE_OFLAG_RDWR | RT_SERIAL_RX_NON_BLOCKING |
                          tx_mode) == RT_EOK ? device : RT_NULL;
}

bool um982_request_navigation_logs(rt_device_t device)
{
    static const char commands[] = "BESTNAVA 0.1\r\nRECTIMEA 1\r\n";
    return rt_device_write(device, 0, commands, sizeof(commands) - 1u) ==
           (rt_ssize_t)(sizeof(commands) - 1u);
}

#if defined(BSP_USING_FLIGHT_DRIVERS)
/* Fixed board connections: PC1 supply enable and PC0 module status. */
static int gnss_power_init(void)
{
    rt_pin_mode(GNSS_POWER_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(GNSS_POWER_PIN, PIN_HIGH);
    rt_pin_mode(GNSS_STATUS_PIN, PIN_MODE_INPUT);
    return RT_EOK;
}
INIT_DEVICE_EXPORT(gnss_power_init);
#endif
