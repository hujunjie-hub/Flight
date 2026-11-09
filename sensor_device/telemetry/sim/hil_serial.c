#include "hil_serial.h"

#include "board_device.h"
#include <rtdevice.h>
#include <string.h>

#ifndef FLIGHT_SIM_LINK_DEVICE
#define FLIGHT_SIM_LINK_DEVICE TELEMETRY_UART_NAME
#endif
#ifndef FLIGHT_SIM_LINK_BAUD
#define FLIGHT_SIM_LINK_BAUD 921600u
#endif

static rt_device_t serial;

const char *hil_serial_device_name(void)
{
    return FLIGHT_SIM_LINK_DEVICE;
}

bool hil_serial_open(void)
{
    if (serial)
        return true;

    /* ELRS keeps its own receiver in both native and HIL builds. */
    if (strcmp(hil_serial_device_name(), RC_UART_NAME) == 0)
        return false;

    struct serial_configure config;
    rt_device_t device = rt_device_find(hil_serial_device_name());
    if (!device || rt_device_control(device, RT_SERIAL_CTRL_GET_CONFIG, &config) != RT_EOK)
        return false;
    if (config.baud_rate != FLIGHT_SIM_LINK_BAUD)
    {
        config.baud_rate = FLIGHT_SIM_LINK_BAUD;
        if (rt_device_control(device, RT_DEVICE_CTRL_CONFIG, &config) != RT_EOK)
            return false;
    }
    if (rt_device_open(device, RT_DEVICE_OFLAG_RDWR | RT_SERIAL_RX_NON_BLOCKING |
                       RT_SERIAL_TX_NON_BLOCKING) != RT_EOK)
        return false;
    serial = device;
    return true;
}

bool hil_serial_is_open(void)
{
    return serial != RT_NULL;
}

size_t hil_serial_read(uint8_t *buffer, size_t capacity)
{
    if (!serial || !buffer || capacity == 0u)
        return 0u;
    const rt_ssize_t count = rt_device_read(serial, 0, buffer, capacity);
    return count > 0 ? (size_t)count : 0u;
}

size_t hil_serial_write(const uint8_t *data, size_t length)
{
    if (!serial || !data || length == 0u)
        return 0u;
    const rt_ssize_t count = rt_device_write(serial, 0, data, length);
    return count > 0 ? (size_t)count : 0u;
}
