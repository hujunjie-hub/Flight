/* Current monitor on I2C2, native RT-Thread device.
 *
 * The fitted part is not specified yet, so the device exposes only a bounded
 * register read via rt_device_control. No model, address or scaling is
 * guessed and no configuration write is performed.
 */
#ifndef PMU_H
#define PMU_H

#include <rtthread.h>
#include <stdint.h>

/* RT_NAME_MAX is 12 on this BSP; keep the object name within 11 chars. */
#define CURRENT_MONITOR_DEVICE_NAME "current"

#define CURRENT_MONITOR_CTRL_READ_REGISTERS (RT_DEVICE_CTRL_BASE(Miscellaneous) + 0x10)

struct current_monitor_request
{
    uint8_t address;
    uint8_t reg;
    uint16_t length;
    uint8_t *data;
};

#endif /* PMU_H */
