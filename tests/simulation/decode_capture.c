/* Decode a captured HIL byte stream with the *firmware* codec.
 *
 * `tools/fmt_sim_probe.py --save` writes exactly the bytes a running FMT-SIM
 * sent to a socket; this program replays them through the same functions
 * task_sim.c uses on the target (MAVLink v2 CRC_EXTRA, HIL decoders, M2 sample
 * builders). It proves the firmware accepts the simulator's real frames, not
 * just the golden ones.
 *
 * The feed pattern mirrors the target: read a bounded chunk, drain every
 * complete frame, repeat. One call per capture file would overflow the parser
 * window and silently drop the tail.
 *
 * Build and run (tests/simulation/verify_live.ps1 does both):
 *   gcc -std=c11 -O2 -DFLIGHT_SIM_UNIT_TEST -I middleware/contracts -I middleware/sensor -I sensor_device \
 *       -I middleware/protocol -I applications/tasks -I tests/simulation/golden \
 *       tests/simulation/decode_capture.c middleware/protocol/mavlink/mavlink_frame.c middleware/protocol/mavlink/mavlink_hil.c \
 *       middleware/sensor/sample_normalization/sensor_sample.c middleware/sensor/measurement_filter/sensor_filter.c middleware/sensor/calibration/sensor_calibration.c -lm -o build/validation/sim/decode_capture
 *   build/validation/sim/decode_capture <capture.bin>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FLIGHT_SIM_UNIT_TEST 1
#include "communication/task_sim.c"

#define CAPTURE_LIMIT (4u * 1024u * 1024u)
#define FEED_CHUNK 256u

typedef struct {
    unsigned sensor_frames;
    unsigned gps_frames;
    unsigned actuator_frames;
    unsigned heartbeat_frames;
    unsigned decoded_imu;
    unsigned decoded_gps;
    mavlink_hil_sensor_t last_sensor;
    mavlink_hil_gps_t last_gps;
    sim_context_t context;
} report_t;

static void handle_frame(const mavlink_hil_frame_t *frame, report_t *report)
{
    ImuSample imu;
    GnssSample gnss;

    if (mavlink_hil_frame_is_sensor(frame))
    {
        ++report->sensor_frames;
        mavlink_hil_decode_sensor(frame, &report->last_sensor);
        if (!report->decoded_imu)
        {
            sim_make_imu(&report->last_sensor, &report->context, 1u, 100000u, &imu);
            printf("IMU  acc=(%.3f,%.3f,%.3f) m/s2 gyro=(%.4f,%.4f,%.4f) rad/s "
                   "temp=%.2fC q=0x%x src=%u frame=%u\n",
                   (double)imu.accel_frd_mps2[0], (double)imu.accel_frd_mps2[1],
                   (double)imu.accel_frd_mps2[2], (double)imu.gyro_frd_radps[0],
                   (double)imu.gyro_frd_radps[1], (double)imu.gyro_frd_radps[2],
                   (double)imu.temperature_c, (unsigned)imu.quality_flags,
                   (unsigned)imu.header.source_id, (unsigned)imu.header.frame_id);
            ++report->decoded_imu;
        }
        return;
    }
    if (mavlink_hil_frame_is_gps(frame))
    {
        ++report->gps_frames;
        mavlink_hil_decode_gps(frame, &report->last_gps);
        if (!report->decoded_gps)
        {
            if (sim_make_gnss(&report->last_gps, &report->context, 1u, 200000u, &gnss))
            {
                printf("GNSS lat=%.7f lon=%.7f alt=%.1fm v=(%.2f,%.2f,%.2f) m/s "
                       "fix=%u sats=%u q=0x%x\n",
                       gnss.latitude_deg, gnss.longitude_deg, (double)gnss.altitude_m,
                       (double)gnss.velocity_ned_mps[0], (double)gnss.velocity_ned_mps[1],
                       (double)gnss.velocity_ned_mps[2], (unsigned)gnss.fix_type,
                       (unsigned)gnss.satellite_count, (unsigned)gnss.quality_flags);
            }
            else
            {
                printf("GNSS fix=%u rejected by sim_make_gnss (no usable fix)\n",
                       (unsigned)report->last_gps.fix_type);
            }
            ++report->decoded_gps;
        }
        return;
    }
    if (frame->id == MAVLINK_HIL_MSG_ACTUATOR_CONTROLS)
    {
        ++report->actuator_frames;
        return;
    }
    if (frame->id == MAVLINK_HIL_MSG_HEARTBEAT)
    {
        ++report->heartbeat_frames;
    }
}

int main(int argc, char **argv)
{
    FILE *file;
    static unsigned char data[CAPTURE_LIMIT];
    size_t length;
    size_t offset;
    mavlink_hil_rx_t rx;
    mavlink_hil_frame_t frame;
    report_t report;

    if (argc != 2)
    {
        fprintf(stderr, "usage: %s <capture.bin>\n", argv[0]);
        return 2;
    }
    file = fopen(argv[1], "rb");
    if (!file)
    {
        fprintf(stderr, "cannot open %s\n", argv[1]);
        return 2;
    }
    length = fread(data, 1u, sizeof(data), file);
    fclose(file);
    if (length == 0u)
    {
        fprintf(stderr, "empty capture\n");
        return 2;
    }

    memset(&report, 0, sizeof(report));
    report.context.boot_id = 0x1122334455667788ull;
    report.context.time_epoch = 7u;

    /* Codec smoke check: the same translation unit must still build both
     * uplink frames, so one link proves decode and encode together. */
    {
        uint8_t scratch[128];
        float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        size_t heartbeat = mavlink_hil_build_heartbeat(0u, scratch, sizeof(scratch));
        size_t actuator = mavlink_hil_build_actuator(0u, zero, 0u, 0u, scratch, sizeof(scratch));
        printf("codec build check: HEARTBEAT %u B, HIL_ACTUATOR_CONTROLS %u B\n",
               (unsigned)heartbeat, (unsigned)actuator);
        if (!heartbeat || !actuator)
        {
            return 4;
        }
    }

    memset(&rx, 0, sizeof(rx));
    offset = 0u;
    while (offset < length)
    {
        /* Never offer more than the parser window can hold: mavlink_hil_rx_feed drops
         * the overflow, which would silently cut frames in half. */
        size_t room = (size_t)MAVLINK_HIL_RX_CAPACITY - (size_t)rx.length;
        size_t chunk = length - offset;
        if (chunk > room)
        {
            chunk = room;
        }
        if (chunk > FEED_CHUNK)
        {
            chunk = FEED_CHUNK;
        }
        (void)mavlink_hil_rx_feed(&rx, data + offset, (uint16_t)chunk);
        offset += chunk;
        while (mavlink_hil_rx_next(&rx, &frame))
        {
            handle_frame(&frame, &report);
        }
    }

    printf("capture %s: %u B -> HIL_SENSOR %u, HIL_GPS %u, HIL_ACTUATOR %u, HEARTBEAT %u "
           "(%u undecodable bytes, %u left in the window)\n",
           argv[1], (unsigned)length, report.sensor_frames, report.gps_frames,
           report.actuator_frames, report.heartbeat_frames, (unsigned)rx.dropped,
           (unsigned)rx.length);

    if (report.decoded_imu)
    {
        /* Exercise the two remaining builders so this link covers the whole
         * pure part of task_sim.c. */
        BaroSample baro;
        if (sim_make_baro(&report.last_sensor, &report.context, 2u, 300000u, &baro))
        {
            printf("BARO alt=%.1f m var=%.2f q=0x%x\n", (double)baro.altitude_m,
                   (double)baro.variance_m2, (unsigned)baro.quality_flags);
        }
        printf("thrust norm: 5 N in [2,12] -> %.3f\n",
               (double)sim_normalize_thrust(5.0f, 2.0f, 12.0f));
    }
    return (report.sensor_frames || report.gps_frames) ? 0 : 3;
}
