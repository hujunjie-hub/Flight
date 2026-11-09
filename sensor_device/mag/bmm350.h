#ifndef BMM350_DEVICE_H
#define BMM350_DEVICE_H

#include <rtthread.h>
#include <stdint.h>

#define MAG_DEVICE_NAME "mag_sensor"

#define MAG_CTRL_READ_COMPENSATED (RT_DEVICE_CTRL_BASE(Sensor) + 0x10)
#define MAG_CTRL_GET_STATS        (RT_DEVICE_CTRL_BASE(Sensor) + 0x11)

struct mag_raw_sample
{
    uint64_t capture_tick;
    uint32_t sequence;
    uint16_t length;
    uint8_t data[12];
};

struct magnetometer_compensated
{
    float xyz_ut[3];
    float temperature_c;
};

struct mag_stats
{
    uint32_t samples;
    uint32_t errors;
};

#endif
