/* TIM2 monotonic time base and capture device.
 *
 * Registered as the native RT-Thread device "timebase"; consumers use
 * rt_device_open/read/control. The TIM2 interrupt handler is the only
 * non-device entry point and is owned by this driver.
 */
#ifndef TIMER_CAPTURE_H
#define TIMER_CAPTURE_H

#include <rtthread.h>
#include <stdint.h>

#define TIMEBASE_DEVICE_NAME "timebase"

#define TIMEBASE_CTRL_GET_TICK_HZ          (RT_DEVICE_CTRL_BASE(Miscellaneous) + 0x10)
#define TIMEBASE_CTRL_READ_CAPTURE         (RT_DEVICE_CTRL_BASE(Miscellaneous) + 0x11)
#define TIMEBASE_CTRL_GET_CAPTURE_LOST     (RT_DEVICE_CTRL_BASE(Miscellaneous) + 0x12)
#define TIMEBASE_CTRL_SET_IMU_CAPTURE      (RT_DEVICE_CTRL_BASE(Miscellaneous) + 0x13)
#define TIMEBASE_CTRL_GET_STATS            (RT_DEVICE_CTRL_BASE(Miscellaneous) + 0x14)
#define TIMEBASE_CTRL_SET_COMPARE          (RT_DEVICE_CTRL_BASE(Miscellaneous) + 0x15)

struct timebase_instant
{
    uint64_t ticks;
    uint32_t epoch;
};

struct timebase_capture
{
    uint64_t tick64;
    uint32_t ccr;
    uint16_t channel;
    uint8_t flags;
    uint8_t reserved;
    uint32_t sequence;
};

struct timebase_stats
{
    uint32_t tick_hz;
    uint32_t capture_lost;
    uint8_t imu_capture_enabled;
    uint8_t reserved[3];
};

/* Device controls copy the callback. Use a null callback to disable it.
 * Callbacks run in the TIM2 ISR and must use bounded, ISR-safe operations. */
struct timebase_imu_capture_config
{
    void (*callback)(const struct timebase_capture *capture);
};

struct timebase_compare
{
    uint8_t channel;
    uint32_t tick;
    void (*callback)(void);
};

#endif /* TIMER_CAPTURE_H */
