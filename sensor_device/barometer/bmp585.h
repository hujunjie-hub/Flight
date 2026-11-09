#ifndef BMP585_DEVICE_H
#define BMP585_DEVICE_H

#include "../sensor_reading.h"

#ifdef __cplusplus
extern "C" {
#endif

sensor_status_t bmp585_decode_payload(const uint8_t data[6], BarometerReading *reading);

#ifdef __cplusplus
}
#endif

#if !defined(FLIGHT_DRIVER_CODEC_TEST)
#include <rtthread.h>
#include <stdint.h>

#define BAROMETER_DEVICE_NAME "barometer"

#define BAROMETER_CTRL_GET_STATS (RT_DEVICE_CTRL_BASE(Sensor) + 0x10)

struct barometer_raw_sample
{
    uint64_t capture_tick;
    uint32_t sequence;
    uint16_t length;
    uint8_t data[6];
};

struct barometer_stats
{
    uint32_t samples;
    uint32_t errors;
};
#endif

#endif
