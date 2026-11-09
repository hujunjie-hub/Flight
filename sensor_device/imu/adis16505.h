#ifndef ADIS16505_DEVICE_H
#define ADIS16505_DEVICE_H

#include "../sensor_reading.h"

typedef struct {
    bool extended;
    double gyro_lsb_per_dps;
    double accel_lsb_per_mps2;
    double temp_lsb_per_c;
} Adis16505Encoding;

#ifdef __cplusplus
extern "C" {
#endif

Adis16505Encoding adis16505_default_encoding(void);
sensor_status_t adis16505_payload_checksum(const uint8_t *burst, size_t length);
sensor_status_t adis16505_decode_payload(const Adis16505Encoding *encoding,
                                        const uint8_t *burst, size_t length, ImuReading *reading);

#ifdef __cplusplus
}
#endif

#if !defined(FLIGHT_DRIVER_CODEC_TEST)
#include <rtthread.h>
#include <stdint.h>

#define IMU_DEVICE_NAME "imu"

#define IMU_CTRL_GET_STATS (RT_DEVICE_CTRL_BASE(Sensor) + 0x10)

#define IMU_FLAG_VALID           0x01u
#define IMU_FLAG_TRANSPORT_ERROR 0x02u

struct imu_raw_sample
{
    uint64_t capture_tick;
    uint64_t dma_start_tick;
    uint64_t dma_complete_tick;
    uint64_t eot_tick;
    uint32_t sequence;
    uint16_t length;
    uint8_t flags;
    uint8_t reserved;
    uint8_t data[32];
};

struct imu_stats
{
    uint32_t received;
    uint32_t dropped;
    uint32_t errors;
    uint32_t checksum_errors;
    uint32_t raw_dropped;
};
#endif

#endif
