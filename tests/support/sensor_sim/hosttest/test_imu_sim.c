/*
 * Independent IMU channel test and standard-sample generator.
 *
 * Round trip: imu_sim (datasheet encoder) -> raw 32-byte burst -> M2 decoder.
 * The tolerance is the quantization step of the simulated part, so a wrong
 * scale constant on either side makes the test fail.
 *
 * Usage:
 *   test_imu_sim                 # run the round-trip checks only
 *   test_imu_sim <directory>     # also write the public standard samples
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "imu_sim.h"
#include "sample_normalization/sensor_sample.h"
#include "sensor_decode_fixture.h"

#define TEST_PI 3.14159265358979323846
/* Public samples carry float; allow one float ulp on top of quantization. */
#define TEST_FLOAT_ULP 2.0e-6

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

static sensor_imu_config_t sim_matching_config(void)
{
    sensor_imu_config_t config;
    int i;
    int j;
    memset(&config, 0, sizeof(config));
    config.burst32 = true;
    config.gyro_lsb_per_dps = IMU_SIM_GYRO_LSB_PER_DPS;
    config.accel_lsb_per_mps2 = IMU_SIM_ACCEL_LSB_PER_MPS2;
    config.temp_lsb_per_c = IMU_SIM_TEMP_LSB_PER_C;
    for (i = 0; i < 3; ++i) {
        config.gyro_scale[i] = 1.0f;
        config.accel_scale[i] = 1.0f;
        for (j = 0; j < 3; ++j) {
            config.gyro_misalignment[i][j] = (i == j) ? 1.0f : 0.0f;
            config.accel_misalignment[i][j] = (i == j) ? 1.0f : 0.0f;
            config.install_rotation[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    config.calibration_version = 1u;
    return config;
}

static sensor_context_t sim_context(void)
{
    sensor_context_t context;
    context.boot_id = 0x00C0FFEE00000001ULL;
    context.time_epoch = 1u;
    context.profile_id = 0x00010001u;
    context.sensor_instance_id = 10u;
    context.sequence = 0u;
    return context;
}

static void roundtrip_static(void)
{
    const sensor_imu_config_t config = sim_matching_config();
    sensor_context_t context = sim_context();
    imu_sim_sample_t sim;
    uint8_t frame[IMU_SIM_BURST32_BYTES];
    ImuSample sample;
    uint64_t capture_us = 1000000u;

    imu_sim_make_static(&sim, 9.80665, 25.0, 11u);
    CHECK(imu_sim_encode_burst32(&sim, frame) == IMU_SIM_BURST32_BYTES);
    CHECK(adis16505_payload_checksum(frame, sizeof(frame)) == SENSOR_OK);
    CHECK(sensor_imu_decode(&config, &context, frame, sizeof(frame), capture_us, capture_us + 150u,
                        &sample) == SENSOR_OK);

    /* Quantization: half an LSB of each quantity. */
    CHECK_NEAR(sample.gyro_frd_radps[0], 0.0, (0.5 * (TEST_PI / 180.0) / IMU_SIM_GYRO_LSB_PER_DPS + TEST_FLOAT_ULP));
    CHECK_NEAR(sample.accel_frd_mps2[2], -9.80665, (0.5 / IMU_SIM_ACCEL_LSB_PER_MPS2 + TEST_FLOAT_ULP));
    CHECK_NEAR(sample.temperature_c, 25.0, 0.05);
    CHECK(sample.sample_sequence == 11u);
}

static void roundtrip_rotation(void)
{
    const sensor_imu_config_t config = sim_matching_config();
    sensor_context_t context = sim_context();
    imu_sim_sample_t sim;
    uint8_t frame[IMU_SIM_BURST32_BYTES];
    ImuSample sample;
    const double rate_dps[3] = {0.0, 0.0, 90.0};
    const double specific_force[3] = {0.35, -0.20, -9.79};
    int axis;

    imu_sim_make_rotation(&sim, rate_dps, specific_force, -10.0, 12u);
    CHECK(imu_sim_encode_burst32(&sim, frame) == IMU_SIM_BURST32_BYTES);
    CHECK(sensor_imu_decode(&config, &context, frame, sizeof(frame), 2000000u, 2000150u, &sample) == SENSOR_OK);

    for (axis = 0; axis < 3; ++axis) {
        CHECK_NEAR(sample.gyro_frd_radps[axis], rate_dps[axis] * (TEST_PI / 180.0),
                   (0.5 * (TEST_PI / 180.0) / IMU_SIM_GYRO_LSB_PER_DPS + TEST_FLOAT_ULP));
        CHECK_NEAR(sample.accel_frd_mps2[axis], specific_force[axis],
                   (0.5 / IMU_SIM_ACCEL_LSB_PER_MPS2 + TEST_FLOAT_ULP));
    }
    CHECK_NEAR(sample.temperature_c, -10.0, 0.05);
}

static void roundtrip_vibrating(void)
{
    const sensor_imu_config_t config = sim_matching_config();
    sensor_context_t context = sim_context();
    imu_sim_sample_t sim;
    uint8_t frame[IMU_SIM_BURST32_BYTES];
    ImuSample sample;
    const double specific_force[3] = {0.0, 0.0, -9.80665};
    unsigned index;
    double previous = -1.0;
    int changes = 0;

    for (index = 0; index < 50u; ++index) {
        imu_sim_make_vibrating(&sim, specific_force, 30.0, index, (uint16_t)index);
        CHECK(imu_sim_encode_burst32(&sim, frame) == IMU_SIM_BURST32_BYTES);
        CHECK(adis16505_payload_checksum(frame, sizeof(frame)) == SENSOR_OK);
        CHECK(sensor_imu_decode(&config, &context, frame, sizeof(frame), 3000000u + index * 1000u,
                            3000200u + index * 1000u, &sample) == SENSOR_OK);
        CHECK_NEAR(sample.accel_frd_mps2[2], sim.accel_mps2[2], (0.5 / IMU_SIM_ACCEL_LSB_PER_MPS2 + TEST_FLOAT_ULP));
        if (index > 0u) {
            if (sample.accel_frd_mps2[0] != (float)previous) {
                ++changes;
            }
        }
        previous = (double)sample.accel_frd_mps2[0];
    }
    /* The vibration must actually vary; a constant channel would be a bug. */
    CHECK(changes > 10);
}

/* ------------------------------------------------------------------------- */
/* Standard samples                                                          */
/* ------------------------------------------------------------------------- */

static int emit_scenario(FILE *file, const char *scenario, int rotating)
{
    const sensor_imu_config_t config = sim_matching_config();
    sensor_context_t context = sim_context();
    unsigned index;

    fprintf(file, "{\n  \"schema\": \"flight-standard-sample/1\",\n");
    fprintf(file, "  \"channel\": \"adis16505-2-burst32\",\n");
    fprintf(file, "  \"profile_id\": \"0x00010001\",\n");
    fprintf(file, "  \"scenario\": \"%s\",\n", scenario);
    fprintf(file, "  \"note\": \"raw device frames; decode with sensor_imu_decode()\",\n");
    fprintf(file, "  \"samples\": [\n");

    for (index = 0; index < 5u; ++index) {
        imu_sim_sample_t sim;
        uint8_t frame[IMU_SIM_BURST32_BYTES];
        ImuSample decoded;
        size_t byte;
        const double rate_dps[3] = {0.0, 0.0, 90.0};
        const double rotating_force[3] = {0.35, -0.20, -9.79};
        const double static_force[3] = {0.0, 0.0, -9.80665};

        if (rotating) {
            imu_sim_make_rotation(&sim, rate_dps, rotating_force, -10.0, (uint16_t)index);
        } else {
            imu_sim_make_vibrating(&sim, static_force, 25.0, index, (uint16_t)index);
        }
        CHECK(imu_sim_encode_burst32(&sim, frame) == IMU_SIM_BURST32_BYTES);
        CHECK(sensor_imu_decode(&config, &context, frame, sizeof(frame), 1000000u + index * 1000u,
                            1000150u + index * 1000u, &decoded) == SENSOR_OK);

        fprintf(file, "    {\"index\": %u, \"counter\": %u, \"frame_hex\": \"", index,
                (unsigned)sim.counter);
        for (byte = 0; byte < IMU_SIM_BURST32_BYTES; ++byte) {
            fprintf(file, "%02x", (unsigned)frame[byte]);
        }
        fprintf(file, "\", \"gyro_dps\": [%.9g, %.9g, %.9g], \"accel_mps2\": [%.9g, %.9g, %.9g], "
                      "\"temperature_c\": %.9g}%s\n",
                sim.gyro_dps[0], sim.gyro_dps[1], sim.gyro_dps[2], sim.accel_mps2[0],
                sim.accel_mps2[1], sim.accel_mps2[2], sim.temperature_c,
                (index + 1u < 5u) ? "," : "");
    }
    fprintf(file, "  ]\n}\n");
    return 0;
}

static void write_standard_samples(const char *directory)
{
    char path[512];
    FILE *file;

    snprintf(path, sizeof(path), "%s/imu_static_5x200hz.json", directory);
    file = fopen(path, "w");
    if (file == NULL) {
        printf("cannot write %s\n", path);
        ++g_failures;
        return;
    }
    emit_scenario(file, "static-with-vibration-25c", 0);
    fclose(file);
    printf("wrote %s\n", path);

    snprintf(path, sizeof(path), "%s/imu_rotation_5x200hz.json", directory);
    file = fopen(path, "w");
    if (file == NULL) {
        printf("cannot write %s\n", path);
        ++g_failures;
        return;
    }
    emit_scenario(file, "rotate-90dps-10c", 1);
    fclose(file);
    printf("wrote %s\n", path);
}

int main(int argc, char **argv)
{
    printf("IMU simulation channel tests\n");
    roundtrip_static();
    roundtrip_rotation();
    roundtrip_vibrating();

    if (argc > 1) {
        write_standard_samples(argv[1]);
    }

    printf("test_imu_sim: %d checks, %d failures\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
