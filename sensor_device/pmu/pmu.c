/* Current monitor on I2C2.
 *
 * Native RT-Thread device "current". The fitted part is not specified
 * yet, so only a bounded register read is exposed through
 * CURRENT_MONITOR_CTRL_READ_REGISTERS. No model, address, scaling or
 * configuration write is guessed.
 */
#include "pmu.h"
#include "board_device.h"
#include <rtdevice.h>

static struct rt_device g_current_dev;
static struct rt_i2c_bus_device *g_current_bus;

static rt_err_t current_init(rt_device_t dev)
{
    (void)dev;
    if (!g_current_bus)
    {
        g_current_bus = rt_i2c_bus_device_find(CURRENT_MONITOR_I2C_BUS_NAME);
    }
    return g_current_bus ? RT_EOK : -RT_EIO;
}

static rt_err_t current_open(rt_device_t dev, rt_uint16_t oflag)
{
    (void)dev;
    (void)oflag;
    return g_current_bus ? RT_EOK : -RT_EIO;
}

static rt_err_t current_close(rt_device_t dev)
{
    (void)dev;
    return RT_EOK;
}

static rt_err_t current_control(rt_device_t dev, int cmd, void *args)
{
    struct current_monitor_request *request = args;

    if (cmd != CURRENT_MONITOR_CTRL_READ_REGISTERS)
    {
        return -RT_ENOSYS;
    }
    if (!dev->ref_count || !g_current_bus)
    {
        return -RT_EIO;
    }
    if (!request || !request->data || !request->length || request->length > 32u ||
        request->address < 0x08u || request->address > 0x77u)
    {
        return -RT_EINVAL;
    }

    uint8_t reg = request->reg;
    struct rt_i2c_msg messages[] = {
        { .addr = request->address, .flags = RT_I2C_WR, .buf = &reg, .len = 1u },
        { .addr = request->address, .flags = RT_I2C_RD, .buf = request->data, .len = request->length },
    };
    return rt_i2c_transfer(g_current_bus, messages, 2u) == 2 ? RT_EOK : -RT_EIO;
}

static const struct rt_device_ops current_ops =
{
    .init = current_init,
    .open = current_open,
    .close = current_close,
    .control = current_control,
};

static int current_register(void)
{
    g_current_dev.type = RT_Device_Class_Miscellaneous;
    g_current_dev.ops = &current_ops;
    return rt_device_register(&g_current_dev, CURRENT_MONITOR_DEVICE_NAME,
                              RT_DEVICE_FLAG_RDONLY);
}
INIT_DEVICE_EXPORT(current_register);
