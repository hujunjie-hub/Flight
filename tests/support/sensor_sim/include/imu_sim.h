/**
 * @file imu_sim.h
 * @brief Independent ADIS16505-2 IMU simulation channel (Host/SIL only).
 *
 * Owner: Agent B.  Authority: spec 05 §6.2 (Host/SIL samples) and §10 step 3.
 *
 * This module produces *raw device frames* (the same 32-byte burst the real
 * part sends), not standardized samples.  Feeding them through
 * sensor_imu_decode() exercises the real decode/calibration/time path instead of a
 * parallel model, which is the point of an independent channel: the simulator
 * must not share code with the decoder.
 *
 * It is Host/SIL only and must never be linked into the H723 image
 * (spec 01 §SIM_ONLY, spec 09 REQ-SIM-001).
 */

#ifndef IMU_SIM_H
#define IMU_SIM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Physical content of one simulated sample, in the sensor frame. */
typedef struct {
    double gyro_dps[3];      /* angular rate, deg/s */
    double accel_mps2[3];    /* specific force, m/s^2 */
    double temperature_c;
    uint16_t diag_stat;      /* DIAG_STAT word, 0 = healthy */
    uint16_t counter;        /* DATA_CNTR */
} imu_sim_sample_t;

/* ADIS16505-2, 32-bit burst scaling (datasheet Rev. C). */
#define IMU_SIM_GYRO_LSB_PER_DPS 2621440.0
#define IMU_SIM_ACCEL_LSB_PER_MPS2 26756268.0
#define IMU_SIM_TEMP_LSB_PER_C 0.1
#define IMU_SIM_BURST32_BYTES 32u

/* Vehicle at rest, level: specific force = (0, 0, -g) in FRD. */
void imu_sim_make_static(imu_sim_sample_t *sample, double gravity_mps2, double temperature_c,
                         uint16_t counter);

/* Constant body rate and specific force (e.g. a coordinated turn fixture). */
void imu_sim_make_rotation(imu_sim_sample_t *sample, const double body_rate_dps[3],
                           const double specific_force_mps2[3], double temperature_c,
                           uint16_t counter);

/* Encodes the sample as a 32-byte 32-bit burst with the datasheet checksum.
 * Returns the number of bytes written (IMU_SIM_BURST32_BYTES). */
size_t imu_sim_encode_burst32(const imu_sim_sample_t *sample, uint8_t out[IMU_SIM_BURST32_BYTES]);

/* Convenience re-encode of an existing physical scenario at sample k, used by
 * the Host sample generator: adds a deterministic small vibration so two
 * channels with different seeds differ but are reproducible. */
void imu_sim_make_vibrating(imu_sim_sample_t *sample, const double specific_force_mps2[3],
                            double temperature_c, uint32_t index, uint16_t counter);

#ifdef __cplusplus
}
#endif

#endif /* IMU_SIM_H */
