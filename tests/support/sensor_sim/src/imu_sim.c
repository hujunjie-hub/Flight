/**
 * @file imu_sim.c
 * @brief ADIS16505-2 raw-burst generator for the independent IMU channel.
 *
 * The encoder is written against the datasheet, not against sensor_sample.c, so
 * that a decoder bug cannot hide behind a matching encoder bug.  The scaling
 * constants are duplicated on purpose and are asserted by the Host test that
 * decodes the generated frames.
 */

#include "imu_sim.h"

#include <math.h>
#include <string.h>

#define IMU_SIM_PI 3.14159265358979323846

static void put_be16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value >> 8);
    dst[1] = (uint8_t)(value & 0xFFu);
}

static int32_t saturate_i32(double value)
{
    if (value > 2147483647.0) {
        return 2147483647;
    }
    if (value < -2147483648.0) {
        return (int32_t)-2147483647 - 1;
    }
    return (int32_t)((value >= 0.0) ? (value + 0.5) : (value - 0.5));
}

void imu_sim_make_static(imu_sim_sample_t *sample, double gravity_mps2, double temperature_c,
                         uint16_t counter)
{
    if (sample == NULL) {
        return;
    }
    memset(sample, 0, sizeof(*sample));
    sample->accel_mps2[2] = -gravity_mps2;
    sample->temperature_c = temperature_c;
    sample->diag_stat = 0u;
    sample->counter = counter;
}

void imu_sim_make_rotation(imu_sim_sample_t *sample, const double body_rate_dps[3],
                           const double specific_force_mps2[3], double temperature_c,
                           uint16_t counter)
{
    int index;
    if ((sample == NULL) || (body_rate_dps == NULL) || (specific_force_mps2 == NULL)) {
        return;
    }
    memset(sample, 0, sizeof(*sample));
    for (index = 0; index < 3; ++index) {
        sample->gyro_dps[index] = body_rate_dps[index];
        sample->accel_mps2[index] = specific_force_mps2[index];
    }
    sample->temperature_c = temperature_c;
    sample->diag_stat = 0u;
    sample->counter = counter;
}

void imu_sim_make_vibrating(imu_sim_sample_t *sample, const double specific_force_mps2[3],
                            double temperature_c, uint32_t index, uint16_t counter)
{
    const double amplitude = 0.02; /* 20 mg vibration, deterministic */
    const double phase = 0.05 * (double)index;
    int axis;

    if ((sample == NULL) || (specific_force_mps2 == NULL)) {
        return;
    }
    memset(sample, 0, sizeof(*sample));
    for (axis = 0; axis < 3; ++axis) {
        sample->accel_mps2[axis] =
            specific_force_mps2[axis] + amplitude * sin(phase + 2.1 * (double)axis);
        sample->gyro_dps[axis] = 0.5 * sin(0.7 * phase + (double)axis);
    }
    sample->temperature_c = temperature_c;
    sample->diag_stat = 0u;
    sample->counter = counter;
}

size_t imu_sim_encode_burst32(const imu_sim_sample_t *sample, uint8_t out[IMU_SIM_BURST32_BYTES])
{
    int32_t gyro_raw[3];
    int32_t accel_raw[3];
    int16_t temperature_raw;
    uint16_t checksum = 0u;
    size_t index;
    int axis;

    if ((sample == NULL) || (out == NULL)) {
        return 0u;
    }

    for (axis = 0; axis < 3; ++axis) {
        gyro_raw[axis] = saturate_i32(sample->gyro_dps[axis] * IMU_SIM_GYRO_LSB_PER_DPS);
        accel_raw[axis] = saturate_i32(sample->accel_mps2[axis] * IMU_SIM_ACCEL_LSB_PER_MPS2);
    }
    temperature_raw = (int16_t)saturate_i32(sample->temperature_c / IMU_SIM_TEMP_LSB_PER_C);

    memset(out, 0, IMU_SIM_BURST32_BYTES);
    put_be16(&out[0], sample->diag_stat);
    for (axis = 0; axis < 3; ++axis) {
        const uint32_t value = (uint32_t)gyro_raw[axis];
        put_be16(&out[2 + axis * 4], (uint16_t)(value & 0xFFFFu));
        put_be16(&out[4 + axis * 4], (uint16_t)(value >> 16));
    }
    for (axis = 0; axis < 3; ++axis) {
        const uint32_t value = (uint32_t)accel_raw[axis];
        put_be16(&out[14 + axis * 4], (uint16_t)(value & 0xFFFFu));
        put_be16(&out[16 + axis * 4], (uint16_t)(value >> 16));
    }
    put_be16(&out[26], (uint16_t)temperature_raw);
    put_be16(&out[28], sample->counter);

    for (index = 0; index < IMU_SIM_BURST32_BYTES - 2u; ++index) {
        checksum = (uint16_t)(checksum + (uint16_t)out[index]);
    }
    put_be16(&out[30], checksum);
    return IMU_SIM_BURST32_BYTES;
}
