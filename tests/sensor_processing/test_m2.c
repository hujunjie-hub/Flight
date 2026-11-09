/*
 * M2 sensor management Host tests.
 *
 * The frames are built from the datasheet constants (not from the module's own
 * output), so a wrong scale constant or a wrong burst layout fails the test.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "messages/message_ids.h"
#include "sample_normalization/sensor_sample.h"
#include "sensor_decode_fixture.h"

#define TEST_PI 3.14159265358979323846

static int g_checks;
static int g_failures;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        ++g_checks;                                                                                \
        if (!(cond)) {                                                                             \
            ++g_failures;                                                                          \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                 \
        }                                                                                          \
    } while (0)

#define CHECK_NEAR(value, reference, tolerance)                                                    \
    do {                                                                                           \
        ++g_checks;                                                                                \
        const double v_ = (value);                                                                 \
        const double r_ = (reference);                                                             \
        if (!(fabs(v_ - r_) <= (tolerance))) {                                                     \
            ++g_failures;                                                                          \
            printf("FAIL %s:%d: %s = %.9g, expected %.9g (tol %.3g)\n", __FILE__, __LINE__, #value, \
                   v_, r_, (tolerance));                                                           \
        }                                                                                          \
    } while (0)

static void put_be16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value >> 8);
    dst[1] = (uint8_t)(value & 0xFFu);
}

static void finalize_checksum(uint8_t *frame, size_t len)
{
    /* 16-bit big-endian modulo-65536 byte sum, as in the datasheet. */
    uint16_t sum = 0u;
    size_t index;
    for (index = 0; index < len - 2u; ++index) {
        sum = (uint16_t)(sum + (uint16_t)frame[index]);
    }
    put_be16(&frame[len - 2u], sum);
}

/* ADIS16505-2, 16-bit burst, BURST_SEL = 0. */
static void build_imu16(uint8_t frame[20], int16_t gyro[3], int16_t accel[3], int16_t temp,
                        uint16_t diag, uint16_t counter)
{
    memset(frame, 0, 20u);
    put_be16(&frame[0], diag);
    put_be16(&frame[2], (uint16_t)gyro[0]);
    put_be16(&frame[4], (uint16_t)gyro[1]);
    put_be16(&frame[6], (uint16_t)gyro[2]);
    put_be16(&frame[8], (uint16_t)accel[0]);
    put_be16(&frame[10], (uint16_t)accel[1]);
    put_be16(&frame[12], (uint16_t)accel[2]);
    put_be16(&frame[14], (uint16_t)temp);
    put_be16(&frame[16], counter);
    finalize_checksum(frame, 20u);
}

/* ADIS16505-2, 32-bit burst, BURST_SEL = 0. */
static void build_imu32(uint8_t frame[32], int32_t gyro[3], int32_t accel[3], int16_t temp,
                        uint16_t diag, uint16_t counter)
{
    int index;
    memset(frame, 0, 32u);
    put_be16(&frame[0], diag);
    for (index = 0; index < 3; ++index) {
        const uint32_t value = (uint32_t)gyro[index];
        put_be16(&frame[2 + index * 4], (uint16_t)(value & 0xFFFFu));
        put_be16(&frame[4 + index * 4], (uint16_t)(value >> 16));
    }
    for (index = 0; index < 3; ++index) {
        const uint32_t value = (uint32_t)accel[index];
        put_be16(&frame[14 + index * 4], (uint16_t)(value & 0xFFFFu));
        put_be16(&frame[16 + index * 4], (uint16_t)(value >> 16));
    }
    put_be16(&frame[26], (uint16_t)temp);
    put_be16(&frame[28], counter);
    finalize_checksum(frame, 32u);
}

static sensor_imu_config_t imu_config(bool burst32)
{
    sensor_imu_config_t config;
    int i;
    int j;
    memset(&config, 0, sizeof(config));
    config.burst32 = burst32;
    config.gyro_lsb_per_dps = burst32 ? 2621440.0 : 40.0;
    config.accel_lsb_per_mps2 = burst32 ? 26756268.0 : (26756268.0 / 65536.0);
    config.temp_lsb_per_c = 0.1;
    for (i = 0; i < 3; ++i) {
        config.gyro_scale[i] = 1.0f;
        config.accel_scale[i] = 1.0f;
        for (j = 0; j < 3; ++j) {
            config.gyro_misalignment[i][j] = (i == j) ? 1.0f : 0.0f;
            config.accel_misalignment[i][j] = (i == j) ? 1.0f : 0.0f;
            config.install_rotation[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    config.pipeline_delay_us = 0.0;
    config.pipeline_delay_uncertainty_us = 0.0;
    config.calibration_version = 7u;
    return config;
}

static sensor_context_t make_context(void)
{
    sensor_context_t context;
    context.boot_id = 0x1122334455667788ULL;
    context.time_epoch = 3u;
    context.profile_id = 0x00010001u;
    context.sensor_instance_id = 1u;
    context.sequence = 0u;
    return context;
}

static void test_imu_16bit_datasheet_scaling(void)
{
    sensor_imu_config_t config = imu_config(false);
    sensor_context_t context = make_context();
    uint8_t frame[20];
    ImuSample sample;
    int16_t gyro[3] = {4000, -4000, 0}; /* 40 LSB/(deg/s) -> +/-100 deg/s */
    int16_t accel[3] = {4004, 0, -4004};
    const double dps_to_rad = TEST_PI / 180.0;

    build_imu16(frame, gyro, accel, 250, 0u, 42u);
    CHECK(adis16505_payload_checksum(frame, sizeof(frame)) == SENSOR_OK);
    CHECK(sensor_imu_decode(&config, &context, frame, sizeof(frame), 1000000u, 1000200u, &sample) == SENSOR_OK);

    CHECK_NEAR(sample.gyro_frd_radps[0], 100.0 * dps_to_rad, 1e-6);
    CHECK_NEAR(sample.gyro_frd_radps[1], -100.0 * dps_to_rad, 1e-6);
    CHECK_NEAR(sample.accel_frd_mps2[0], 9.807, 0.01); /* 1 g within the scale constant */
    CHECK_NEAR(sample.temperature_c, 25.0, 1e-4);
    CHECK(sample.sample_sequence == 42u);
    CHECK(sample.calibration_version == 7u);
    CHECK((sample.quality_flags & IMU_Q_CALIBRATED) != 0u);
    CHECK(sample.header.schema_major == 6u);
    CHECK(sample.header.message_type == MSG_IMU_SAMPLE);
    CHECK(sample.header.source_id == SOURCE_IMU);
    CHECK(sample.header.frame_id == FRAME_BODY_FRD);
    CHECK(sample.header.boot_id == context.boot_id);
    CHECK(sample.header.sequence == 0u);
    CHECK(sample.header.timestamp_sample_us == 1000000u);
    CHECK(sample.header.timestamp_quality == TIMESTAMP_QUALITY_HARDWARE_CAPTURED);
    CHECK(sample.header.event_time_uncertainty_us == 0u);
    CHECK((sample.header.validity_flags & VALIDITY_PAYLOAD_VALID) != 0u);

    /* A diagnostic status bit must not be reported as calibrated. */
    build_imu16(frame, gyro, accel, 250, 0x0001u, 43u);
    CHECK(sensor_imu_decode(&config, &context, frame, sizeof(frame), 2000000u, 2000200u, &sample) == SENSOR_OK);
    CHECK((sample.quality_flags & IMU_Q_CALIBRATED) == 0u);
    CHECK(sample.header.sequence == 1u); /* context sequence advanced */
}

static void test_imu_32bit_burst_and_saturation_range(void)
{
    sensor_imu_config_t config = imu_config(true);
    sensor_context_t context = make_context();
    uint8_t frame[32];
    ImuSample sample;
    int32_t gyro[3] = {1310720000, 0, -1310720000}; /* 2,621,440 LSB/(deg/s) -> +/-500 deg/s */
    int32_t accel[3] = {0, 26756268, 0};      /* 1 m/s^2 */
    const double dps_to_rad = TEST_PI / 180.0;

    build_imu32(frame, gyro, accel, -125, 0u, 5u);
    CHECK(adis16505_payload_checksum(frame, sizeof(frame)) == SENSOR_OK);
    CHECK(sensor_imu_decode(&config, &context, frame, sizeof(frame), 5000000u, 5000300u, &sample) == SENSOR_OK);
    CHECK_NEAR(sample.gyro_frd_radps[0], 500.0 * dps_to_rad, 1e-4);
    CHECK_NEAR(sample.accel_frd_mps2[1], 1.0, 1e-6);
    CHECK_NEAR(sample.temperature_c, -12.5, 1e-4);

    /* Wrong length for the configured burst format. */
    CHECK(sensor_imu_decode(&config, &context, frame, 20u, 5000000u, 5000300u, &sample) == SENSOR_ERR_LENGTH);
}

static void test_imu_calibration_and_installation(void)
{
    sensor_imu_config_t config = imu_config(false);
    sensor_context_t context = make_context();
    uint8_t frame[20];
    ImuSample sample;
    int16_t gyro[3] = {4000, 0, 0};
    int16_t accel[3] = {4004, 0, 0};
    const double dps_to_rad = TEST_PI / 180.0;

    /* Bias 1 deg/s, scale correction 1.01 -> (100-1)*1.01 = 99.99 deg/s. */
    config.gyro_bias_dps[0] = 1.0f;
    config.gyro_scale[0] = 1.01f;

    /* Installation: rotate +90 deg about body Z, i.e. sensor X -> body Y. */
    config.install_rotation[0][0] = 0.0f;
    config.install_rotation[0][1] = -1.0f;
    config.install_rotation[1][0] = 1.0f;
    config.install_rotation[1][1] = 0.0f;

    config.pipeline_delay_us = 350.0;
    config.pipeline_delay_uncertainty_us = 50.0;

    build_imu16(frame, gyro, accel, 250, 0u, 1u);
    CHECK(sensor_imu_decode(&config, &context, frame, sizeof(frame), 1000000u, 1000400u, &sample) == SENSOR_OK);

    CHECK_NEAR(sample.gyro_frd_radps[1], 99.99 * dps_to_rad, 1e-5);
    CHECK_NEAR(sample.gyro_frd_radps[0], 0.0, 1e-9);
    CHECK_NEAR(sample.accel_frd_mps2[1], 4004 / (26756268.0 / 65536.0), 1e-4);

    /* Pipeline delay is subtracted from the capture time and declared. */
    CHECK(sample.header.timestamp_sample_us == 999650u);
    CHECK(sample.header.timestamp_quality == TIMESTAMP_QUALITY_SENSOR_MODEL_CORRECTED);
    CHECK((sample.header.timestamp_flags & TIMESTAMP_FLAG_PIPELINE_CORRECTED) != 0u);
    CHECK((sample.header.timestamp_flags & TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED) != 0u);
    CHECK(sample.header.event_time_uncertainty_us == 50u);
}

static void test_imu_rejections(void)
{
    sensor_imu_config_t config = imu_config(false);
    sensor_context_t context = make_context();
    uint8_t frame[20];
    ImuSample sample;
    int16_t gyro[3] = {0, 0, 0};
    int16_t accel[3] = {0, 0, 0};

    build_imu16(frame, gyro, accel, 250, 0u, 1u);
    frame[19] ^= 0x01u;
    CHECK(sensor_imu_decode(&config, &context, frame, sizeof(frame), 1000u, 1200u, &sample) ==
          SENSOR_ERR_CHECKSUM);

    CHECK(sensor_imu_decode(&config, &context, NULL, sizeof(frame), 1000u, 1200u, &sample) ==
          SENSOR_ERR_ARGUMENT);

    build_imu16(frame, gyro, accel, 250, 0u, 1u);
    config.gyro_lsb_per_dps = 0.0;
    CHECK(sensor_imu_decode(&config, &context, frame, sizeof(frame), 1000u, 1200u, &sample) ==
          SENSOR_ERR_NOT_CONFIGURED);

    /* Capture time below the correction must not wrap. */
    config = imu_config(false);
    config.pipeline_delay_us = 5000.0;
    CHECK(sensor_imu_decode(&config, &context, frame, sizeof(frame), 1000u, 1200u, &sample) == SENSOR_OK);
    CHECK(sample.header.timestamp_sample_us == 0u);
}

static void test_baro(void)
{
    sensor_baro_config_t config;
    sensor_context_t context = make_context();
    uint8_t raw[6];
    BaroSample sample;
    uint32_t pressure_raw;
    uint32_t temperature_raw;
    double altitude_zero;

    memset(&config, 0, sizeof(config));
    config.p0_pa = 101325.0;
    config.variance_m2 = 0.25;
    config.calibration_version = 2u;

    /* Sea-level reference: exactly p0 -> 0 m. */
    pressure_raw = (uint32_t)(101325.0 * 64.0);
    temperature_raw = (uint32_t)(25.0 * 65536.0);
    raw[0] = (uint8_t)(pressure_raw & 0xFFu);
    raw[1] = (uint8_t)((pressure_raw >> 8) & 0xFFu);
    raw[2] = (uint8_t)((pressure_raw >> 16) & 0xFFu);
    raw[3] = (uint8_t)(temperature_raw & 0xFFu);
    raw[4] = (uint8_t)((temperature_raw >> 8) & 0xFFu);
    raw[5] = (uint8_t)((temperature_raw >> 16) & 0xFFu);

    CHECK(sensor_baro_decode(&config, &context, raw, 3000000u, 3000600u, &sample) == SENSOR_OK);
    CHECK_NEAR(sample.altitude_m, 0.0, 1e-3);
    CHECK_NEAR(sample.variance_m2, 0.25, 1e-6);
    CHECK(sample.quality_flags == (BARO_Q_ALTITUDE_VALID | BARO_Q_VARIANCE_VALID |
                                   BARO_Q_REFERENCE_VALID));
    CHECK(sample.header.message_type == MSG_BARO_SAMPLE);
    CHECK(sample.header.source_id == SOURCE_BARO);
    CHECK(sample.header.timestamp_sample_us == 3000000u);
    CHECK(sample.header.timestamp_quality == TIMESTAMP_QUALITY_HARDWARE_CAPTURED);

    /* 100 kPa gives roughly 111 m in the ICAO troposphere. */
    altitude_zero = sensor_baro_pressure_to_altitude(100000.0, config.p0_pa);
    CHECK(altitude_zero > 100.0 && altitude_zero < 120.0);

    /* Out of the datasheet operating range: reject, do not clamp. */
    pressure_raw = (uint32_t)(20000.0 * 64.0);
    raw[0] = (uint8_t)(pressure_raw & 0xFFu);
    raw[1] = (uint8_t)((pressure_raw >> 8) & 0xFFu);
    raw[2] = (uint8_t)((pressure_raw >> 16) & 0xFFu);
    CHECK(sensor_baro_decode(&config, &context, raw, 4000000u, 4000600u, &sample) == SENSOR_ERR_RANGE);

    CHECK(sensor_baro_decode(&config, &context, NULL, 4000000u, 4000600u, &sample) == SENSOR_ERR_ARGUMENT);
}

static sensor_gnss_config_t gnss_config(void)
{
    sensor_gnss_config_t config;
    config.max_speed_mps = 30.0;
    config.max_altitude_m = 20000.0;
    config.max_variance_m2 = 10000.0;
    config.stale_age_us = 20000u;
    return config;
}

static sensor_gnss_pvt_t gnss_valid_pvt(void)
{
    sensor_gnss_pvt_t pvt;
    int index;
    memset(&pvt, 0, sizeof(pvt));
    pvt.latitude_deg = 30.5;
    pvt.longitude_deg = 114.3;
    pvt.altitude_m = 35.0;
    pvt.velocity_ned_mps[0] = 1.0f;
    pvt.velocity_ned_mps[1] = -0.5f;
    pvt.velocity_ned_mps[2] = 0.1f;
    for (index = 0; index < 3; ++index) {
        pvt.position_variance_m2[index] = 0.25f;
        pvt.velocity_variance_m2ps2[index] = 0.04f;
    }
    pvt.fix_type = GNSS_FIX_RTK_FIXED;
    pvt.satellites = 24u;
    pvt.valid_flags = SENSOR_GNSS_PVT_POSITION_VALID | SENSOR_GNSS_PVT_VELOCITY_VALID |
                      SENSOR_GNSS_PVT_POSITION_COV_VALID | SENSOR_GNSS_PVT_VELOCITY_COV_VALID;
    return pvt;
}

static void test_gnss(void)
{
    const sensor_gnss_config_t config = gnss_config();
    sensor_context_t context = make_context();
    sensor_gnss_state_t state;
    sensor_gnss_pvt_t pvt = gnss_valid_pvt();
    GnssSample sample;

    sensor_gnss_reset(&state);

    CHECK(sensor_gnss_build_sample(&config, &context, &state, &pvt, 1000000u, 1005000u, 9u, &sample) ==
          SENSOR_OK);
    CHECK(sample.quality_flags == (GNSS_Q_POSITION_VALID | GNSS_Q_VELOCITY_VALID |
                                   GNSS_Q_POSITION_COV_VALID | GNSS_Q_VELOCITY_COV_VALID));
    CHECK_NEAR(sample.latitude_deg, 30.5, 1e-12);
    CHECK_NEAR(sample.velocity_ned_mps[1], -0.5, 1e-6);
    CHECK(sample.header.message_type == MSG_GNSS_SAMPLE);
    CHECK(sample.header.source_id == SOURCE_GNSS);
    CHECK(sample.header.source_clock_domain_id == CLOCK_DOMAIN_GNSS_GPS);
    CHECK(sample.header.source_mapping_epoch == 9u);
    CHECK(sample.header.timestamp_quality == TIMESTAMP_QUALITY_EXTERNAL_MAPPED);
    CHECK((sample.header.timestamp_flags & TIMESTAMP_FLAG_EXTERNAL_CLOCK_MAPPED) != 0u);

    /* Position jump: 0.01 deg latitude (~1.1 km) in 100 ms is not a fix. */
    pvt.latitude_deg = 30.51;
    CHECK(sensor_gnss_build_sample(&config, &context, &state, &pvt, 1100000u, 1105000u, 9u, &sample) ==
          SENSOR_ERR_JUMP);
    CHECK(state.latitude_deg == 30.5); /* rejected sample did not move the state */
    pvt = gnss_valid_pvt();

    /* Stale: arrival is 25 ms after the measurement epoch. */
    CHECK(sensor_gnss_build_sample(&config, &context, &state, &pvt, 1200000u, 1225000u, 9u, &sample) ==
          SENSOR_ERR_STALE);

    /* Time going backwards. */
    CHECK(sensor_gnss_build_sample(&config, &context, &state, &pvt, 900000u, 905000u, 9u, &sample) ==
          SENSOR_ERR_STALE);

    /* Range checks. */
    pvt.latitude_deg = 91.0;
    CHECK(sensor_gnss_build_sample(&config, &context, &state, &pvt, 1300000u, 1305000u, 9u, &sample) ==
          SENSOR_ERR_RANGE);
    pvt = gnss_valid_pvt();

    /* Velocity invalid + covariance not trusted: independent flags. */
    pvt.valid_flags = SENSOR_GNSS_PVT_POSITION_VALID | SENSOR_GNSS_PVT_POSITION_COV_VALID;
    pvt.velocity_variance_m2ps2[0] = -1.0f;
    CHECK(sensor_gnss_build_sample(&config, &context, &state, &pvt, 1400000u, 1405000u, 9u, &sample) ==
          SENSOR_OK);
    CHECK((sample.quality_flags & GNSS_Q_POSITION_VALID) != 0u);
    CHECK((sample.quality_flags & GNSS_Q_VELOCITY_VALID) == 0u);
    CHECK((sample.quality_flags & GNSS_Q_VELOCITY_COV_VALID) == 0u);

    /* Mapping epoch 0: time is receive-time estimated, not external mapped. */
    pvt = gnss_valid_pvt();
    sensor_gnss_reset(&state);
    CHECK(sensor_gnss_build_sample(&config, &context, &state, &pvt, 1500000u, 1505000u, 0u, &sample) ==
          SENSOR_OK);
    CHECK(sample.header.timestamp_quality == TIMESTAMP_QUALITY_RECEIVE_TIME_ESTIMATED);
    CHECK(sample.header.source_mapping_epoch == 0u);
}

int main(void)
{
    printf("M2 sensor management tests\n");
    test_imu_16bit_datasheet_scaling();
    test_imu_32bit_burst_and_saturation_range();
    test_imu_calibration_and_installation();
    test_imu_rejections();
    test_baro();
    test_gnss();
    printf("test_m2: %d checks, %d failures\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
