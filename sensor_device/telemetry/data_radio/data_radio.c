#include "data_radio.h"

#include "board_device.h"
#include <rtdevice.h>

static rt_device_t radio;
static volatile bool tx_busy;

static rt_err_t tx_complete(rt_device_t device, void *buffer)
{
    (void)device;
    (void)buffer;
    tx_busy = false;
    return RT_EOK;
}

bool data_radio_open(void)
{
    if (radio)
        return true;

    struct serial_configure config;
    rt_device_t device = rt_device_find(TELEMETRY_UART_NAME);
    if (!device || rt_device_control(device, RT_SERIAL_CTRL_GET_CONFIG, &config) != RT_EOK)
        return false;
    if (config.baud_rate != 115200u)
    {
        config.baud_rate = 115200u;
        if (rt_device_control(device, RT_DEVICE_CTRL_CONFIG, &config) != RT_EOK)
            return false;
    }
    if (rt_device_open(device, RT_DEVICE_OFLAG_RDWR | RT_SERIAL_RX_NON_BLOCKING |
                       RT_SERIAL_TX_NON_BLOCKING) != RT_EOK)
        return false;
    if (rt_device_set_tx_complete(device, tx_complete) != RT_EOK)
    {
        (void)rt_device_close(device);
        return false;
    }
    radio = device;
    return true;
}

void data_radio_close(void)
{
    if (radio)
    {
        (void)rt_device_set_tx_complete(radio, RT_NULL);
        (void)rt_device_close(radio);
        radio = RT_NULL;
        tx_busy = false;
    }
}

size_t data_radio_read(uint8_t *buffer, size_t capacity)
{
    if (!radio || !buffer || capacity == 0u)
        return 0u;
    const rt_ssize_t count = rt_device_read(radio, 0, buffer, capacity);
    return count > 0 ? (size_t)count : 0u;
}

bool data_radio_tx_busy(void)
{
    return tx_busy;
}

size_t data_radio_write(const uint8_t *data, size_t length)
{
    if (!radio || tx_busy || !data || length == 0u)
        return 0u;
    /* Serial V2 copies into the TX FIFO. Wait for completion so each new
     * frame starts with the full FIFO capacity available. */
    tx_busy = true;
    const rt_ssize_t count = rt_device_write(radio, 0, data, length);
    if (count <= 0)
    {
        tx_busy = false;
        return 0u;
    }
    return (size_t)count;
}
