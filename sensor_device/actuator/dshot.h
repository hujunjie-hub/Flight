/* Four-channel DShot transport on TIM1 + DMA2_Stream2.
 *
 * Registered as the native RT-Thread device "dshot". A frame is submitted with
 * rt_device_write(); scheduled one-shot frames use a driver control command.
 * No arm/authority policy lives here.
 */
#ifndef DSHOT_H
#define DSHOT_H

#include <rtthread.h>
#include <stdint.h>

#define DSHOT_DEVICE_NAME "dshot"

#define DSHOT_CTRL_SCHEDULE    (RT_DEVICE_CTRL_BASE(Miscellaneous) + 0x20)
#define DSHOT_CTRL_STOP        (RT_DEVICE_CTRL_BASE(Miscellaneous) + 0x21)
#define DSHOT_CTRL_GET_STATS   (RT_DEVICE_CTRL_BASE(Miscellaneous) + 0x22)
#define DSHOT_CTRL_GET_ARR     (RT_DEVICE_CTRL_BASE(Miscellaneous) + 0x23)

struct dshot_schedule
{
    uint16_t frame[4];
    uint64_t start_tick;
    uint64_t expires_tick;
};

struct dshot_stats
{
    uint32_t faults;
    uint32_t frames;
    uint32_t arr;
    uint8_t active;
    uint8_t pending;
    uint8_t ready;
    uint8_t reserved;
};

#endif /* DSHOT_H */
