/* Host test for the HIL sim-link wire codec (applications/tasks/communication/task_sim.c).
 *
 * The file under test is compiled with FLIGHT_SIM_UNIT_TEST so only its pure
 * part (MAVLink v2 framing, HIL decoders, public-sample builders, thrust
 * normalization) is linked here. The expected bytes come from
 * tests/simulation/golden/hil_golden.h, produced by an independent Python
 * implementation of the same dialect.
 *
 * Run: pwsh tests/simulation/run_host_tests.ps1
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hil_golden.h"

#define FLIGHT_SIM_UNIT_TEST 1
#include "communication/task_sim.c"

static unsigned checks;
static unsigned failures;

#define CHECK(condition)                                                                  \
    do                                                                                    \
    {                                                                                     \
        ++checks;                                                                         \
        if (!(condition))                                                                 \
        {                                                                                 \
            ++failures;                                                                   \
            printf("FAIL %u: %s\n", (unsigned)__LINE__, #condition);                      \
        }                                                                                 \
    } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                                           \
    do                                                                                    \
    {                                                                                     \
        double a_ = (double)(actual);                                                      \
        double e_ = (double)(expected);                                                    \
        ++checks;                                                                          \
        if (!(fabs(a_ - e_) <= (double)(tolerance)))                                       \
        {                                                                                  \
            ++failures;                                                                    \
            printf("FAIL %u: %s=%f expected %f\n", (unsigned)__LINE__, #actual, a_, e_);   \
        }                                                                                  \
    } while (0)

/* --- helpers ------------------------------------------------------------- */

static void feed_all(mavlink_hil_rx_t *rx, const unsigned char *frame, size_t length)
{
    (void)mavlink_hil_rx_feed(rx, frame, (uint16_t)length);
}

static int parse_one(const unsigned char *frame, size_t length, mavlink_hil_frame_t *out)
{
    mavlink_hil_rx_t rx;
    memset(&rx, 0, sizeof(rx));
    feed_all(&rx, frame, length);
    return mavlink_hil_rx_next(&rx, out);
}

static void test_crc_check_value(void)
{
    /* CRC-16/MCRF4XX published check value for "123456789". */
    static const char text[] = "123456789";
    uint16_t crc = 0xFFFFu;
    size_t i;
    for (i = 0; i < sizeof(text) - 1u; ++i)
    {
        crc = mavlink_crc_accumulate(crc, (uint8_t)text[i]);
    }
    CHECK(crc == 0x6F91u);
}

static void test_decode_sensor(void)
{
    mavlink_hil_frame_t frame;
    mavlink_hil_sensor_t sensor;
    CHECK(parse_one(HIL_SENSOR_FRAME, sizeof(HIL_SENSOR_FRAME), &frame) == 1);
    CHECK(frame.id == MAVLINK_HIL_MSG_SENSOR);
    CHECK(mavlink_hil_frame_is_sensor(&frame) == 1);
    CHECK(mavlink_hil_frame_is_gps(&frame) == 0);
    CHECK(frame.sequence == 1u);
    CHECK(frame.length == 62u); /* MAVLink2 trimmed the two trailing zero bytes */
    mavlink_hil_decode_sensor(&frame, &sensor);
    CHECK(sensor.sim_time_us == 1234567u);
    CHECK_NEAR(sensor.accel_mps2[0], 0.25, 1e-6);
    CHECK_NEAR(sensor.accel_mps2[1], -0.5, 1e-6);
    CHECK_NEAR(sensor.accel_mps2[2], 9.79, 1e-6);
    CHECK_NEAR(sensor.gyro_radps[0], 0.01, 1e-6);
    CHECK_NEAR(sensor.gyro_radps[2], 0.03, 1e-6);
    CHECK_NEAR(sensor.mag_gauss[0], 0.21, 1e-6);
    CHECK_NEAR(sensor.mag_gauss[2], 0.43, 1e-6);
    CHECK_NEAR(sensor.abs_pressure_hpa, 900.0, 1e-4);
    CHECK_NEAR(sensor.temperature_c, 25.5, 1e-6);
    CHECK(sensor.fields_updated == 0x1FFFu);
}

static void test_decode_gps(void)
{
    mavlink_hil_frame_t frame;
    mavlink_hil_gps_t gps;
    CHECK(parse_one(HIL_GPS_FRAME, sizeof(HIL_GPS_FRAME), &frame) == 1);
    CHECK(frame.id == MAVLINK_HIL_MSG_GPS);
    CHECK(mavlink_hil_frame_is_gps(&frame) == 1);
    CHECK(mavlink_hil_frame_is_sensor(&frame) == 0);
    CHECK(frame.length == 37u);
    mavlink_hil_decode_gps(&frame, &gps);
    CHECK_NEAR(gps.latitude_deg, 47.397742, 1e-9);
    CHECK_NEAR(gps.longitude_deg, 8.545594, 1e-9);
    CHECK_NEAR(gps.altitude_m, 488.0, 1e-4);
    CHECK_NEAR(gps.eph_m, 1.2, 1e-6);
    CHECK_NEAR(gps.epv_m, 2.0, 1e-6);
    CHECK_NEAR(gps.velocity_ned_mps[0], 1.5, 1e-6);
    CHECK_NEAR(gps.velocity_ned_mps[1], -1.5, 1e-6);
    CHECK_NEAR(gps.velocity_ned_mps[2], 0.25, 1e-6);
    CHECK(gps.fix_type == 3u);
    CHECK(gps.satellites == 14u);

    /* A 36-byte payload (no extension) must still decode: the receiver
     * zero-fills what MAVLink2 truncated. */
    CHECK(parse_one(HIL_GPS_TRUNCATED_FRAME, sizeof(HIL_GPS_TRUNCATED_FRAME), &frame) == 1);
    CHECK(frame.length == 36u);
    mavlink_hil_decode_gps(&frame, &gps);
    CHECK(gps.fix_type == 3u);
    CHECK(gps.satellites == 14u);

    /* A zero satellite count is also subject to MAVLink 2 truncation. */
    uint8_t payload[36], trimmed[sizeof(HIL_GPS_FRAME)];
    memcpy(payload, HIL_GPS_TRUNCATED_FRAME + MAVLINK_V2_HEADER_SIZE, sizeof(payload));
    payload[35] = 0u;
    const size_t length = wire_encode(MAVLINK_HIL_MSG_GPS, 124u, payload, sizeof(payload),
                                      0u, trimmed, sizeof(trimmed));
    CHECK(length == MAVLINK_V2_HEADER_SIZE + 35u + MAVLINK_CRC_SIZE);
    CHECK(parse_one(trimmed, length, &frame) == 1);
    CHECK(mavlink_hil_frame_is_gps(&frame));
    mavlink_hil_decode_gps(&frame, &gps);
    CHECK(gps.fix_type == 3u && gps.satellites == 0u);
}

static void test_parser_edges(void)
{
    mavlink_hil_rx_t rx;
    mavlink_hil_frame_t frame;
    unsigned i;

    /* Garbage in front of a frame: resynchronize, keep the frame. */
    memset(&rx, 0, sizeof(rx));
    feed_all(&rx, (const unsigned char *)"\x00\x11\xfd\x99", 4);
    feed_all(&rx, HIL_ACTUATOR_FRAME, sizeof(HIL_ACTUATOR_FRAME));
    CHECK(mavlink_hil_rx_next(&rx, &frame) == 1);
    CHECK(frame.id == MAVLINK_HIL_MSG_ACTUATOR_CONTROLS);
    CHECK(rx.dropped > 0u);

    /* Byte-at-a-time delivery must produce the same frame. */
    memset(&rx, 0, sizeof(rx));
    for (i = 0; i < sizeof(HIL_GPS_FRAME); ++i)
    {
        feed_all(&rx, HIL_GPS_FRAME + i, 1u);
    }
    CHECK(mavlink_hil_rx_next(&rx, &frame) == 1);
    CHECK(frame.id == MAVLINK_HIL_MSG_GPS);
    /* An accepted frame is consumed, never counted as discarded bytes. */
    CHECK(rx.dropped == 0u);
    CHECK(rx.length == 0u);

    /* One compaction keeps an incomplete header after a long noise prefix. */
    uint8_t noise[200];
    memset(noise, 0x55, sizeof(noise));
    memset(&rx, 0, sizeof(rx));
    feed_all(&rx, noise, sizeof(noise));
    feed_all(&rx, HIL_GPS_FRAME, 5u);
    CHECK(mavlink_hil_rx_next(&rx, &frame) == 0);
    CHECK(rx.dropped == sizeof(noise) && rx.length == 5u);
    feed_all(&rx, HIL_GPS_FRAME + 5u, sizeof(HIL_GPS_FRAME) - 5u);
    CHECK(mavlink_hil_rx_next(&rx, &frame) == 1 && frame.id == MAVLINK_HIL_MSG_GPS);
    CHECK(rx.dropped == sizeof(noise) && rx.length == 0u);

    /* A flipped checksum byte is rejected, not decoded. */
    memset(&rx, 0, sizeof(rx));
    feed_all(&rx, HIL_SENSOR_BAD_CRC_FRAME, sizeof(HIL_SENSOR_BAD_CRC_FRAME));
    CHECK(mavlink_hil_rx_next(&rx, &frame) == 0);
    CHECK(rx.dropped > 0u);
    CHECK(rx.length < MAVLINK_V2_HEADER_SIZE + MAVLINK_CRC_SIZE);

    /* An unknown message id cannot be CRC-checked and is skipped. */
    {
        const uint8_t unknown[MAVLINK_V2_HEADER_SIZE] = {MAVLINK_V2_MAGIC, 81u, 0, 0, 0, 1, 1, 200};
        memset(&rx, 0, sizeof(rx));
        feed_all(&rx, unknown, sizeof(unknown));
        feed_all(&rx, HIL_GPS_FRAME, sizeof(HIL_GPS_FRAME));
        CHECK(mavlink_hil_rx_next(&rx, &frame) == 1 && frame.id == MAVLINK_HIL_MSG_GPS);
        CHECK(rx.dropped == sizeof(unknown) && rx.length == 0u);
    }

    /* Even with a valid CRC, a frame requiring an unknown feature is rejected. */
    {
        uint8_t incompatible[sizeof(HIL_GPS_FRAME)];
        const size_t crc_offset = sizeof(incompatible) - MAVLINK_CRC_SIZE;
        memcpy(incompatible, HIL_GPS_FRAME, sizeof(incompatible));
        incompatible[2] = 0x02u;
        uint16_t crc = 0xffffu;
        for (size_t byte = 1; byte < crc_offset; ++byte)
            crc = mavlink_crc_accumulate(crc, incompatible[byte]);
        wire_u16(incompatible + crc_offset, mavlink_crc_accumulate(crc, 124u));
        CHECK(parse_one(incompatible, sizeof(incompatible), &frame) == 0);
    }

    /* Two frames in one buffer, in order. */
    memset(&rx, 0, sizeof(rx));
    feed_all(&rx, HIL_GPS_FRAME, sizeof(HIL_GPS_FRAME));
    feed_all(&rx, HIL_SENSOR_FRAME, sizeof(HIL_SENSOR_FRAME));
    CHECK(mavlink_hil_rx_next(&rx, &frame) == 1);
    CHECK(frame.id == MAVLINK_HIL_MSG_GPS);
    CHECK(mavlink_hil_rx_next(&rx, &frame) == 1);
    CHECK(frame.id == MAVLINK_HIL_MSG_SENSOR);
}

static void dump_mismatch(const char *label, const uint8_t *actual, const unsigned char *expected,
                          size_t length, size_t expected_length)
{
    size_t i;
    printf("%s: actual %u bytes, expected %u bytes\n", label, (unsigned)length,
           (unsigned)expected_length);
    for (i = 0; i < length && i < expected_length; ++i)
    {
        if (actual[i] != expected[i])
        {
            printf("  first difference at %u: %02x != %02x\n", (unsigned)i, actual[i],
                   expected[i]);
            break;
        }
    }
}

static void test_encode_matches_golden(void)
{
    uint8_t out[128];
    float controls[4] = {0.1f, 0.2f, 0.3f, 0.4f};
    size_t length;

    length = mavlink_hil_build_actuator(3000000u, controls, 0x80u, 1u, out, sizeof(out));
    CHECK(length == sizeof(HIL_ACTUATOR_FRAME));
    if (memcmp(out, HIL_ACTUATOR_FRAME, length) != 0)
    {
        dump_mismatch("actuator", out, HIL_ACTUATOR_FRAME, length, sizeof(HIL_ACTUATOR_FRAME));
    }
    CHECK(memcmp(out, HIL_ACTUATOR_FRAME, length) == 0);

    length = mavlink_hil_build_heartbeat(1u, out, sizeof(out));
    CHECK(length == sizeof(HEARTBEAT_FRAME));
    if (memcmp(out, HEARTBEAT_FRAME, length) != 0)
    {
        dump_mismatch("heartbeat", out, HEARTBEAT_FRAME, length, sizeof(HEARTBEAT_FRAME));
    }
    CHECK(memcmp(out, HEARTBEAT_FRAME, length) == 0);

    /* A zeroed demand must still produce a frame (single truncated byte). */
    {
        float zeros[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        length = mavlink_hil_build_actuator(0u, zeros, 0u, 0u, out, sizeof(out));
        CHECK(length > 0u);
        CHECK(out[1] >= 1u);
    }
}

static void test_thrust_normalization(void)
{
    CHECK_NEAR(sim_normalize_thrust(0.0f, 0.0f, 10.0f), 0.0, 1e-6);
    CHECK_NEAR(sim_normalize_thrust(5.0f, 0.0f, 10.0f), 0.5, 1e-6);
    CHECK_NEAR(sim_normalize_thrust(10.0f, 0.0f, 10.0f), 1.0, 1e-6);
    CHECK_NEAR(sim_normalize_thrust(25.0f, 0.0f, 10.0f), 1.0, 1e-6);  /* clamped */
    CHECK_NEAR(sim_normalize_thrust(-5.0f, 0.0f, 10.0f), 0.0, 1e-6);  /* clamped */
    CHECK_NEAR(sim_normalize_thrust(5.0f, 2.0f, 12.0f), 0.3, 1e-6);
    CHECK_NEAR(sim_normalize_thrust(5.0f, 4.0f, 4.0f), 0.0, 1e-6);    /* zero span */
    CHECK_NEAR(sim_normalize_thrust(NAN, 0.0f, 10.0f), 0.0, 1e-6);
}

static void test_sample_builders(void)
{
    mavlink_hil_frame_t frame;
    mavlink_hil_sensor_t sensor;
    mavlink_hil_gps_t gps;
    sim_context_t context;
    ImuSample imu;
    BaroSample baro;
    GnssSample gnss;
    const uint64_t now_us = 10000000u;

    context.boot_id = 0x1122334455667788ull;
    context.time_epoch = 7u;

    CHECK(parse_one(HIL_SENSOR_FRAME, sizeof(HIL_SENSOR_FRAME), &frame) == 1);
    mavlink_hil_decode_sensor(&frame, &sensor);
    sim_make_imu(&sensor, &context, 42u, now_us, &imu);
    CHECK(imu.header.source_id == SOURCE_IMU);
    CHECK(imu.header.frame_id == FRAME_BODY_FRD);
    CHECK(imu.header.message_type == MSG_IMU_SAMPLE);
    CHECK(imu.header.boot_id == context.boot_id);
    CHECK(imu.header.time_epoch == context.time_epoch);
    CHECK(imu.header.validity_flags == (VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID));
    CHECK(imu.header.timestamp_sample_us == now_us - SIM_STAMP_OFFSET_US);
    CHECK(imu.header.timestamp_publish_us == now_us);
    CHECK(imu.header.event_time_uncertainty_us == SIM_STAMP_UNCERTAINTY_US);
    CHECK(imu.header.timestamp_flags == TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED);
    CHECK(imu.header.timestamp_quality == TIMESTAMP_QUALITY_SENSOR_MODEL_CORRECTED);
    CHECK(imu.sensor_instance_id == 1u);
    CHECK(imu.sample_sequence == 42u);
    CHECK(imu.calibration_version == 1u);
    CHECK((imu.quality_flags & (IMU_Q_GYRO_VALID | IMU_Q_ACCEL_VALID | IMU_Q_CALIBRATED)) ==
          (IMU_Q_GYRO_VALID | IMU_Q_ACCEL_VALID | IMU_Q_CALIBRATED));
    CHECK((imu.quality_flags & IMU_Q_TEMP_VALID) != 0u);
    CHECK_NEAR(imu.accel_frd_mps2[2], 9.79, 1e-6);

    CHECK(sim_make_baro(&sensor, &context, 7u, now_us, &baro) == 1);
    CHECK(baro.header.source_id == SOURCE_BARO);
    CHECK(baro.header.frame_id == FRAME_LOCAL_NED);
    CHECK(baro.header.message_type == MSG_BARO_SAMPLE);
    CHECK(baro.quality_flags == (BARO_Q_ALTITUDE_VALID | BARO_Q_VARIANCE_VALID |
                                 BARO_Q_REFERENCE_VALID));
    CHECK_NEAR(baro.altitude_m, sensor_baro_pressure_to_altitude(90000.0, 101325.0), 1e-3);
    CHECK(baro.variance_m2 > 0.0f);
    {
        mavlink_hil_sensor_t broken = sensor;
        broken.abs_pressure_hpa = 0.0f;
        CHECK(sim_make_baro(&broken, &context, 8u, now_us, &baro) == 0);
    }

    CHECK(parse_one(HIL_GPS_FRAME, sizeof(HIL_GPS_FRAME), &frame) == 1);
    mavlink_hil_decode_gps(&frame, &gps);
    CHECK(sim_make_gnss(&gps, &context, 9u, now_us, &gnss) == 1);
    CHECK(gnss.header.source_id == SOURCE_GNSS);
    CHECK(gnss.header.frame_id == FRAME_WGS84_GEODETIC);
    CHECK(gnss.header.message_type == MSG_GNSS_SAMPLE);
    CHECK(gnss.sensor_instance_id == 4u);
    CHECK(gnss.fix_type == GNSS_FIX_SINGLE);
    CHECK(gnss.satellite_count == 14u);
    CHECK((gnss.quality_flags & (GNSS_Q_POSITION_VALID | GNSS_Q_POSITION_COV_VALID |
                                 GNSS_Q_VELOCITY_VALID | GNSS_Q_VELOCITY_COV_VALID)) ==
          (GNSS_Q_POSITION_VALID | GNSS_Q_POSITION_COV_VALID | GNSS_Q_VELOCITY_VALID |
           GNSS_Q_VELOCITY_COV_VALID));
    CHECK_NEAR(gnss.position_variance_m2[0], 1.44, 1e-5);
    CHECK_NEAR(gnss.position_variance_m2[2], 4.0, 1e-5);
    CHECK_NEAR(gnss.velocity_ned_mps[0], 1.5, 1e-6);
    CHECK_NEAR(gnss.latitude_deg, 47.397742, 1e-9);

    /* No fix: no position, and fix 2 (2D) must not claim an altitude fix. */
    gps.fix_type = 0u;
    CHECK(sim_make_gnss(&gps, &context, 10u, now_us, &gnss) == 0);
    gps.fix_type = 2u;
    CHECK(sim_make_gnss(&gps, &context, 11u, now_us, &gnss) == 1);
    CHECK((gnss.quality_flags & GNSS_Q_POSITION_VALID) == 0u);
    CHECK((gnss.quality_flags & GNSS_Q_VELOCITY_VALID) != 0u);

    /* Unknown dilution (65535 cm) must not fabricate a covariance. */
    gps.fix_type = 3u;
    gps.eph_m = 655.35f;
    CHECK(sim_make_gnss(&gps, &context, 12u, now_us, &gnss) == 1);
    CHECK((gnss.quality_flags & GNSS_Q_POSITION_COV_VALID) == 0u);
}

static void test_fmt_state(void)
{
    mavlink_fmt_state_t state = {0};
    uint8_t broken[sizeof(FMT_EXTERNAL_STATE_FRAME)];
    CHECK(FMT_EXTERNAL_STATE_FRAME[1] == 85u);
    CHECK(mavlink_hil_decode_fmt_state(FMT_EXTERNAL_STATE_FRAME,
                                     sizeof(FMT_EXTERNAL_STATE_FRAME), &state));
    CHECK_NEAR(state.quaternion[0], 1.0, 1e-6);
    CHECK_NEAR(state.gyro_radps[1], -0.02, 1e-6);
    CHECK_NEAR(state.specific_force_ned_mps2[2], -9.8, 1e-6);
    CHECK_NEAR(state.velocity_ned_mps[2], 0.25, 1e-6);
    CHECK_NEAR(state.position_ned_m[2], -2.0, 1e-6);
    CHECK_NEAR(state.latitude_deg, 37.6278579, 1e-9);
    CHECK_NEAR(state.longitude_deg, -122.3895167, 1e-9);
    CHECK_NEAR(state.altitude_m, 4.488, 1e-6);
    CHECK(!mavlink_hil_decode_fmt_state(FMT_EXTERNAL_STATE_FRAME,
                                      sizeof(FMT_EXTERNAL_STATE_FRAME) - 1u, &state));
    memcpy(broken, FMT_EXTERNAL_STATE_FRAME, sizeof(broken));
    broken[20] ^= 1u;
    CHECK(!mavlink_hil_decode_fmt_state(broken, sizeof(broken), &state));
}

int main(void)
{
    test_crc_check_value();
    test_decode_sensor();
    test_decode_gps();
    test_fmt_state();
    test_parser_edges();
    test_encode_matches_golden();
    test_thrust_normalization();
    test_sample_builders();

    printf("HIL link codec tests: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
