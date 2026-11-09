#ifndef SENSOR_DECODE_FIXTURE_H
#define SENSOR_DECODE_FIXTURE_H

#include "bmp585.h"
#include "adis16505.h"
#include "sample_normalization/sensor_sample.h"

static inline sensor_status_t sensor_imu_decode(const sensor_imu_config_t *config,
                                                 sensor_context_t *context, const uint8_t *frame,
                                                 size_t length, uint64_t capture_us,
                                                 uint64_t publish_us, ImuSample *out)
{
    const Adis16505Encoding encoding = {
        .extended = config->burst32,
        .gyro_lsb_per_dps = config->gyro_lsb_per_dps,
        .accel_lsb_per_mps2 = config->accel_lsb_per_mps2,
        .temp_lsb_per_c = config->temp_lsb_per_c,
    };
    ImuReading reading;
    sensor_status_t status = adis16505_decode_payload(&encoding, frame, length, &reading);
    return status == SENSOR_OK
        ? sensor_imu_build_sample(config, context, &reading, capture_us, publish_us, out)
        : status;
}

/* Recorded fixtures use pressure first. The device decoder receives its
 * native temperature-first register order. */
static inline sensor_status_t sensor_baro_decode(const sensor_baro_config_t *config,
                                                  sensor_context_t *context, const uint8_t *raw,
                                                  uint64_t sample_us, uint64_t publish_us,
                                                  BaroSample *out)
{
    if (!raw)
        return SENSOR_ERR_ARGUMENT;
    const uint8_t data[6] = {raw[3], raw[4], raw[5], raw[0], raw[1], raw[2]};
    BarometerReading reading;
    sensor_status_t status = bmp585_decode_payload(data, &reading);
    return status == SENSOR_OK
        ? sensor_baro_build_sample(config, context, &reading, sample_us, publish_us, out)
        : status;
}

#endif
