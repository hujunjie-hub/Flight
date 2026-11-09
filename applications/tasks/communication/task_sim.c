/* HIL sim-link task: FMT-SIM hardware-in-the-loop bridge.
 *
 *   FMT-SIM  <-- UDP 127.0.0.1:14553 -->  communication/sim/fmt_sim_udp_bridge.py
 *            <-- serial <baud> -------->  this task (via hil_link)
 *
 * This task owns the HIL exchange through the communication adapter and
 * protocol codec:
 *
 *   RX  HIL_SENSOR -> M2 standard samples (IMU through sensor_imu_inject,
 *                     barometer through runtime_submit_measurement)
 *       HIL_GPS    -> M2 GnssSample (runtime_submit_measurement)
 *   TX  HIL_ACTUATOR_CONTROLS <- the M6 ActuatorDemand, normalized per motor
 *                              with the M7 thrust-curve limits
 *       HEARTBEAT  <- 1 Hz link keepalive for the simulator
 *
 * Boundaries this task deliberately keeps (spec 01 PLANT, spec 02 section 4.9,
 * spec 05 section 6.2):
 *   - it never writes a NavigationSnapshot, an estimator state, a permit or an
 *     actuator frame: M3 and M7 keep their single owners;
 *   - the communication layer handles framing and the HIL serial device owns
 *     the UART (spec 03 layering);
 *   - the uplink carries the M6 *demand*, not a committed DShot frame. With
 *     output_policy_id 1 (BENCH_INHIBIT) the physical outputs stay inhibited,
 *     so a HIL run cannot spin motors;
 *   - PlantTruth / sim time never enter the estimator: every injected sample
 *     is timestamped with the local monotonic capture time and carries the
 *     normal calibration/quality metadata of an M2 sample.
 *
 * Wire details are MAVLink v2 (magic 0xFD, CRC-16/MCRF4XX over header[1..9] +
 * payload + CRC_EXTRA). The payload lengths and CRC_EXTRA values are the ones
 * in the generated common dialect of this project's upstream baseline
 * (HIL_SENSOR 64/108, HIL_GPS 36..37/124, HIL_ACTUATOR_CONTROLS 81/47,
 * HEARTBEAT 9/50).
 *
 * Build switch: applications/Kconfig CONFIG_FLIGHT_SIM_LINK. When it is off,
 * this file is not compiled and no UART is taken from M1.
 *
 * Host test: tests/simulation links the protocol codec and builds the sample
 * conversion below with -DFLIGHT_SIM_UNIT_TEST.
 */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "base_types/common_base.h"
#include "messages/common_messages.h"
#include "messages/message_ids.h"
#include "product_contract/product_contract.h"
#include "sample_normalization/sensor_sample.h"
#include "mavlink/mavlink_hil.h"

/* ------------------------------------------------------------------------- */
/* Configuration                                                             */
/* ------------------------------------------------------------------------- */

/* The HIL link takes the telemetry UART: the first Flight carrier has no
 * spare UART routed to the console connector, and the same connector is what
 * a bench bridge needs. M1 then stops opening that device (see
 * telemetry_link_release_radio). 921600 carries 250 Hz HIL_SENSOR plus 250 Hz
 * HIL_ACTUATOR_CONTROLS (~115 B/frame, ~58 kB/s) with headroom; 115200 cannot:
 * it would cap the uplink at ~50 Hz. */
/* Injected samples: 1 kHz IMU cadence, 50 Hz barometer, 5 Hz GNSS. Slower
 * rates only decimate; HIL_SENSOR usually arrives at the IMU rate. */
#define SIM_IMU_PERIOD_US 1000u
#define SIM_BARO_PERIOD_US 20000u
#define SIM_GNSS_PERIOD_US 200000u
#define SIM_ACTUATOR_PERIOD_US 4000u
#define SIM_HEARTBEAT_PERIOD_US 1000000u

/* The demand is valid for 5 ms in M7; allow a little slack for a 250 Hz poll
 * but never forward a demand that M6 has already let expire. */
#define SIM_DEMAND_MAX_AGE_US 10000u

/* Local timestamp policy: the bridge adds several milliseconds of transport
 * jitter, so sim time is NOT used as a sample epoch. The sample is stamped
 * slightly before the local publication time, and that bounded 50 us is what
 * the header advertises. header_valid() accepts <= 100 us. */
#define SIM_STAMP_OFFSET_US 50u
#define SIM_STAMP_UNCERTAINTY_US 50u

/* MAV_MODE_FLAG_SAFETY_ARMED. The field says "this demand is live", not that
 * the firmware has armed or that M7 committed a frame. */
#define SIM_HIL_ARMED_MODE 0x80u

/* ------------------------------------------------------------------------- */
/* HIL sample adaptation                                                    */
/* ------------------------------------------------------------------------- */

/* Sea-level reference used for the pressure-altitude conversion, matching the
 * M2 barometer default (sensor_baro_config_t.p0_pa). */
#define SIM_BARO_P0_PA 101325.0

/* ------------------------------------------------------------------------- */
/* Sample construction (host-testable, no RTOS dependency)                 */
/* ------------------------------------------------------------------------- */

/* Identity needed to build public sample headers. */
typedef struct {
    uint64_t boot_id;
    uint32_t time_epoch;
} sim_context_t;

static uint8_t sim_map_fix_type(uint8_t mavlink_fix)
{
    switch (mavlink_fix)
    {
    case 5u: return GNSS_FIX_RTK_FLOAT;
    case 6u: return GNSS_FIX_RTK_FIXED;
    case 4u: return GNSS_FIX_DGPS;
    case 3u: return GNSS_FIX_SINGLE;
    case 2u: return GNSS_FIX_SINGLE;
    default: return GNSS_FIX_NONE;
    }
}

static void sim_header(ContractHeader *header, uint16_t source, uint16_t type, uint32_t sequence,
                       uint64_t sample_us, uint64_t publish_us, uint64_t valid_until_us,
                       uint32_t uncertainty_us, uint32_t frame, uint8_t quality)
{
    memset(header, 0, sizeof(*header));
    header->schema_major = CONTRACT_SCHEMA_MAJOR;
    header->schema_minor = CONTRACT_SCHEMA_MINOR;
    header->message_type = type;
    header->source_id = source;
    header->sequence = sequence;
    header->profile_id = PRODUCT_PROFILE_ID;
    header->frame_id = frame;
    header->boot_id = 0u;
    header->clock_domain_id = CLOCK_DOMAIN_FCU_MONOTONIC;
    header->time_epoch = 0u;
    header->source_clock_domain_id = 0u;
    header->source_mapping_epoch = 0u;
    header->timestamp_sample_us = sample_us;
    header->timestamp_publish_us = publish_us;
    header->valid_until_us = valid_until_us;
    header->event_time_uncertainty_us = uncertainty_us;
    header->timestamp_quality = quality;
    header->timestamp_flags = TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED;
    header->validity_flags = VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID;
}

static void sim_fill_identity(ContractHeader *header, const sim_context_t *context)
{
    header->boot_id = context->boot_id;
    header->time_epoch = context->time_epoch;
}

static void sim_make_imu(const mavlink_hil_sensor_t *sensor, const sim_context_t *context,
                         uint32_t sequence, uint64_t now_us, ImuSample *out)
{
    uint64_t sample_us = now_us > SIM_STAMP_OFFSET_US ? now_us - SIM_STAMP_OFFSET_US : 0u;
    memset(out, 0, sizeof(*out));
    sim_header(&out->header, SOURCE_IMU, MSG_IMU_SAMPLE, sequence, sample_us,
               now_us, now_us + 5000u, SIM_STAMP_UNCERTAINTY_US, FRAME_BODY_FRD,
               TIMESTAMP_QUALITY_SENSOR_MODEL_CORRECTED);
    sim_fill_identity(&out->header, context);
    out->sensor_instance_id = 1u;
    out->sample_sequence = sequence;
    out->calibration_version = 1u;
    out->quality_flags = IMU_Q_GYRO_VALID | IMU_Q_ACCEL_VALID | IMU_Q_CALIBRATED;
    if (isfinite(sensor->temperature_c))
    {
        out->quality_flags |= IMU_Q_TEMP_VALID;
    }
    out->gyro_frd_radps[0] = sensor->gyro_radps[0];
    out->gyro_frd_radps[1] = sensor->gyro_radps[1];
    out->gyro_frd_radps[2] = sensor->gyro_radps[2];
    out->accel_frd_mps2[0] = sensor->accel_mps2[0];
    out->accel_frd_mps2[1] = sensor->accel_mps2[1];
    out->accel_frd_mps2[2] = sensor->accel_mps2[2];
    out->temperature_c = sensor->temperature_c;
}

/* HIL_SENSOR gives both a pressure and a pressure altitude. The public
 * BaroSample is a pressure altitude from the M2 conversion so the estimator
 * sees the same reference as the real sensor; the message's own
 * pressure_alt is only a cross-check. */
static int sim_make_baro(const mavlink_hil_sensor_t *sensor, const sim_context_t *context,
                         uint32_t sequence, uint64_t now_us, BaroSample *out)
{
    double pressure_pa;
    memset(out, 0, sizeof(*out));
    if (!isfinite(sensor->abs_pressure_hpa) || sensor->abs_pressure_hpa <= 0.0f)
    {
        return 0;
    }
    pressure_pa = (double)sensor->abs_pressure_hpa * 100.0;
    {
        uint64_t sample_us = now_us > SIM_STAMP_OFFSET_US ? now_us - SIM_STAMP_OFFSET_US : 0u;
        sim_header(&out->header, SOURCE_BARO, MSG_BARO_SAMPLE, sequence, sample_us,
                   now_us, now_us + 200000u, SIM_STAMP_UNCERTAINTY_US, FRAME_LOCAL_NED,
                   TIMESTAMP_QUALITY_SENSOR_MODEL_CORRECTED);
    }
    sim_fill_identity(&out->header, context);
    out->sensor_instance_id = 3u;
    out->calibration_version = 1u;
    out->quality_flags = BARO_Q_ALTITUDE_VALID | BARO_Q_VARIANCE_VALID | BARO_Q_REFERENCE_VALID;
    out->altitude_m = (float)sensor_baro_pressure_to_altitude(pressure_pa, SIM_BARO_P0_PA);
    /* No barometer noise model is provided by HIL_SENSOR; 0.5 m sigma is the
     * configured M2 floor and is declared as such instead of being invented
     * from the flight state. */
    out->variance_m2 = 0.25f;
    return isfinite(out->altitude_m) ? 1 : 0;
}

/* HIL_GPS carries no velocity accuracy, so a fixed 0.5 m/s sigma is declared.
 * Position/velocity validity stays independent (spec 02 section 1.6). */
static int sim_make_gnss(const mavlink_hil_gps_t *gps, const sim_context_t *context,
                         uint32_t sequence, uint64_t now_us, GnssSample *out)
{
    uint8_t fix;
    int position_ok;
    memset(out, 0, sizeof(*out));

    fix = sim_map_fix_type(gps->fix_type);
    if (fix == GNSS_FIX_NONE)
    {
        return 0;
    }
    position_ok = gps->fix_type >= 3u && isfinite(gps->latitude_deg) &&
                  isfinite(gps->longitude_deg) && isfinite(gps->altitude_m) &&
                  fabs(gps->latitude_deg) < 89.0 && fabs(gps->longitude_deg) <= 180.0 &&
                  gps->eph_m > 0.0f && gps->eph_m < 655.0f && gps->epv_m > 0.0f &&
                  gps->epv_m < 655.0f;

    sim_header(&out->header, SOURCE_GNSS, MSG_GNSS_SAMPLE, sequence,
               now_us > SIM_STAMP_OFFSET_US ? now_us - SIM_STAMP_OFFSET_US : 0u,
               now_us, now_us + 200000u, SIM_STAMP_UNCERTAINTY_US, FRAME_WGS84_GEODETIC,
               TIMESTAMP_QUALITY_SENSOR_MODEL_CORRECTED);
    sim_fill_identity(&out->header, context);
    out->sensor_instance_id = 4u;
    out->sample_sequence = sequence;
    out->calibration_version = 1u;
    out->fix_type = fix;
    out->satellite_count = gps->satellites;
    out->latitude_deg = gps->latitude_deg;
    out->longitude_deg = gps->longitude_deg;
    out->altitude_m = gps->altitude_m;

    if (position_ok)
    {
        out->quality_flags |= GNSS_Q_POSITION_VALID | GNSS_Q_POSITION_COV_VALID;
        out->position_variance_m2[0] = gps->eph_m * gps->eph_m;
        out->position_variance_m2[1] = gps->eph_m * gps->eph_m;
        out->position_variance_m2[2] = gps->epv_m * gps->epv_m;
    }
    if (isfinite(gps->velocity_ned_mps[0]) && isfinite(gps->velocity_ned_mps[1]) &&
        isfinite(gps->velocity_ned_mps[2]))
    {
        out->quality_flags |= GNSS_Q_VELOCITY_VALID | GNSS_Q_VELOCITY_COV_VALID;
        out->velocity_ned_mps[0] = gps->velocity_ned_mps[0];
        out->velocity_ned_mps[1] = gps->velocity_ned_mps[1];
        out->velocity_ned_mps[2] = gps->velocity_ned_mps[2];
        out->velocity_variance_m2ps2[0] = 0.25f;
        out->velocity_variance_m2ps2[1] = 0.25f;
        out->velocity_variance_m2ps2[2] = 0.25f;
    }
    return (out->quality_flags & (GNSS_Q_POSITION_VALID | GNSS_Q_VELOCITY_VALID)) != 0u;
}

/* Motor demand in newtons -> HIL_ACTUATOR_CONTROLS in [0,1], using the same
 * configured range M7 bounds the physical output with. A zero span cannot be
 * normalized, so it reports 0 instead of a fabricated full-scale value. */
static float sim_normalize_thrust(float thrust_n, float min_n, float max_n)
{
    float span;
    float normalized;
    if (!isfinite(thrust_n) || !isfinite(min_n) || !isfinite(max_n))
    {
        return 0.0f;
    }
    span = max_n - min_n;
    if (!(span > 0.0f))
    {
        return 0.0f;
    }
    normalized = (thrust_n - min_n) / span;
    if (normalized < 0.0f)
    {
        return 0.0f;
    }
    if (normalized > 1.0f)
    {
        return 1.0f;
    }
    return normalized;
}

#ifdef FLIGHT_SIM_UNIT_TEST
/* The Host fixture exercises sample conversion without RT-Thread. */
#else

#include <rtthread.h>
#include "output_guard/actuator_guard.h"
#include "sim/hil_link.h"
#include "runtime.h"
#include "runtime_internal.h"
#include "sample_acquisition/sensor_input.h"
#include "systime.h"
#include "task_config.h"
#define LOG_TAG "hil"
#include <ulog.h>

static uint8_t sim_link_announced;

static uint32_t sim_imu_injected;
static uint32_t sim_imu_rejected;
static uint32_t sim_baro_submitted;
static uint32_t sim_baro_rejected;
static uint32_t sim_gnss_submitted;
static uint32_t sim_gnss_rejected;
static uint32_t sim_mag_seen;
static uint32_t sim_demand_fresh;

static uint32_t sim_sequence_imu;
static uint32_t sim_sequence_baro;
static uint32_t sim_sequence_gnss;

static uint8_t sim_have_sensor;
static uint8_t sim_have_gps;
static mavlink_hil_sensor_t sim_sensor;
static mavlink_hil_gps_t sim_gps;
static sim_context_t sim_context;

static uint64_t sim_last_baro_us;
static uint64_t sim_last_gnss_us;

/* Announce once, so a silent wire is distinguishable from a closed one. */
static void sim_announce_link(void)
{
    if (!sim_link_announced)
    {
        sim_link_announced = 1u;
        LOG_I("HIL link %s, external IMU source", hil_link_device_name());
    }
}

static int sim_link_open(void)
{
    if (hil_link_is_open())
    {
        return 1;
    }
    if (hil_link_open() != STATUS_OK)
    {
        return 0;
    }
    /* This bench task owns the external source once it opens the UART. An
     * idle wire publishes no IMU samples. */
    sensor_imu_use_external_source(true);
    sim_announce_link();
    return 1;
}

static void sim_submit_measurement(const SensorMeasurement *measurement, uint32_t *ok_count,
                                   uint32_t *rejected_count)
{
    if (runtime_submit_measurement(measurement) == STATUS_OK)
    {
        ++(*ok_count);
    }
    else
    {
        ++(*rejected_count);
    }
}

static void sim_publish_baro(uint64_t now_us)
{
    SensorMeasurement measurement;
    BaroSample baro;
    if (now_us - sim_last_baro_us < SIM_BARO_PERIOD_US)
    {
        return;
    }
    if (!sim_make_baro(&sim_sensor, &sim_context, ++sim_sequence_baro, now_us, &baro))
    {
        ++sim_baro_rejected;
        return;
    }
    sim_last_baro_us = now_us;
    memset(&measurement, 0, sizeof(measurement));
    measurement.header = baro.header;
    measurement.sensor_instance_id = baro.sensor_instance_id;
    measurement.measurement_kind = PRODUCT_MEASUREMENT_BARO;
    measurement.payload.baro = baro;
    sim_submit_measurement(&measurement, &sim_baro_submitted, &sim_baro_rejected);
}

static void sim_publish_gnss(uint64_t now_us)
{
    SensorMeasurement measurement;
    GnssSample gnss;
    if (now_us - sim_last_gnss_us < SIM_GNSS_PERIOD_US)
    {
        return;
    }
    if (!sim_make_gnss(&sim_gps, &sim_context, ++sim_sequence_gnss, now_us, &gnss))
    {
        ++sim_gnss_rejected;
        return;
    }
    sim_last_gnss_us = now_us;
    memset(&measurement, 0, sizeof(measurement));
    measurement.header = gnss.header;
    measurement.sensor_instance_id = gnss.sensor_instance_id;
    measurement.measurement_kind = PRODUCT_MEASUREMENT_GNSS;
    measurement.payload.gnss = gnss;
    sim_submit_measurement(&measurement, &sim_gnss_submitted, &sim_gnss_rejected);
}

/* One injected IMU sample per local millisecond while data is fresh. The local
 * cadence is what the estimator integrates; sim time is never the epoch. */
static void sim_imu_tick(uint64_t now_us)
{
    ImuSample sample;
    if (!sim_have_sensor)
    {
        return;
    }
    sim_have_sensor = 0u;
    sim_make_imu(&sim_sensor, &sim_context, ++sim_sequence_imu, now_us, &sample);
    if (sensor_imu_inject(&sample, now_us) == STATUS_OK)
    {
        ++sim_imu_injected;
        (void)rt_sem_release(&runtime_data.imu_event);
    }
    else
    {
        ++sim_imu_rejected;
    }
    sim_publish_baro(now_us);
}

static void sim_rx_frame(const mavlink_hil_frame_t *frame)
{
    if (mavlink_hil_frame_is_sensor(frame))
    {
        mavlink_hil_decode_sensor(frame, &sim_sensor);
        sim_have_sensor = 1u;
        /* The magnetometer is decoded for the link statistics only: this
         * product keeps MAG aiding disabled and has no verified BMM350 scale,
         * so publishing a MagSample here would fake calibration evidence. */
        if (sim_sensor.mag_gauss[0] != 0.0f || sim_sensor.mag_gauss[1] != 0.0f ||
            sim_sensor.mag_gauss[2] != 0.0f)
        {
            ++sim_mag_seen;
        }
        return;
    }
    if (mavlink_hil_frame_is_gps(frame))
    {
        mavlink_hil_decode_gps(frame, &sim_gps);
        sim_have_gps = 1u;
        return;
    }
}

static void sim_tx_actuator(uint64_t now_us)
{
    ControlCapability capability;
    ControlHealth health;
    ActuatorDemand demand;
    uint32_t reference = 0u;
    float limits_min[4], limits_max[4];
    float controls[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t mode = 0u;

    if (runtime_get_control(&capability, &health, &demand, &reference) == STATUS_OK &&
        demand.control_sequence != 0u && now_us < demand.demand_expires_at_us &&
        now_us >= demand.header.timestamp_publish_us &&
        now_us - demand.header.timestamp_publish_us <= SIM_DEMAND_MAX_AGE_US &&
        (demand.valid_channel_mask & 15u) == 15u && (demand.demand.valid_mask & 15u) == 15u &&
        actuator_guard_get_thrust_limits(limits_min, limits_max) == STATUS_OK)
    {
        unsigned motor;
        for (motor = 0; motor < 4u; ++motor)
        {
            controls[motor] = sim_normalize_thrust(demand.demand.motor_thrust_n[motor],
                                                   limits_min[motor], limits_max[motor]);
        }
        mode = SIM_HIL_ARMED_MODE;
        ++sim_demand_fresh;
    }
    hil_link_send_actuator(now_us, controls, mode);
}

static void sim_status(void)
{
    const HilLinkStats stats = hil_link_stats();
    rt_kprintf("HIL link %s: rx %u frames / %u bytes, %u discarded bytes, tx %u frames (%u dropped)\n",
               hil_link_is_open() ? hil_link_device_name() : "closed", (unsigned)stats.rx_frames,
               (unsigned)stats.rx_bytes, (unsigned)stats.discarded_bytes, (unsigned)stats.tx_frames,
               (unsigned)stats.tx_dropped);
    rt_kprintf("  imu=%u/%u baro=%u/%u gnss=%u/%u mag-seen=%u fresh-demand=%u imu-fifo-drop=%u\n",
               (unsigned)sim_imu_injected, (unsigned)sim_imu_rejected, (unsigned)sim_baro_submitted,
               (unsigned)sim_baro_rejected, (unsigned)sim_gnss_submitted,
               (unsigned)sim_gnss_rejected, (unsigned)sim_mag_seen, (unsigned)sim_demand_fresh,
               (unsigned)sensor_imu_injected_dropped());
    if (!hil_link_is_open())
    {
        rt_kprintf("  link not open: %s is missing, busy or has another owner\n",
                   hil_link_device_name());
    }
}
MSH_CMD_EXPORT(sim_status, HIL sim-link statistics);

static void task_sim_step(uint64_t now_us, uint64_t *next_imu_us, uint64_t *next_actuator_us,
                          uint64_t *next_heartbeat_us)
{
    if (!hil_link_is_open() && !sim_link_open())
    {
        return;
    }
    hil_link_poll_receive(sim_rx_frame);
    if (now_us >= *next_imu_us)
    {
        sim_imu_tick(now_us);
        *next_imu_us = now_us + SIM_IMU_PERIOD_US;
    }
    if (sim_have_gps)
    {
        sim_have_gps = 0u;
        sim_publish_gnss(now_us);
    }
    if (now_us >= *next_actuator_us)
    {
        sim_tx_actuator(now_us);
        *next_actuator_us = now_us + SIM_ACTUATOR_PERIOD_US;
    }
    if (now_us >= *next_heartbeat_us)
    {
        hil_link_send_heartbeat();
        *next_heartbeat_us = now_us + SIM_HEARTBEAT_PERIOD_US;
    }
}

/* Includes the HIL link framing, sample submission and runtime lock calls. */
static uint8_t task_sim_stack[4096] __attribute__((aligned(8)));

static void task_sim_entry(void *parameter)
{
    rt_tick_t last = rt_tick_get();
    uint64_t next_imu_us = 0u;
    uint64_t next_actuator_us = 0u;
    uint64_t next_heartbeat_us = 0u;
    ClockInstant clock;

    (void)parameter;
    memset(&sim_context, 0, sizeof(sim_context));
    if (!clock_read(&clock))
    {
        sim_context.boot_id = clock.boot_id;
        sim_context.time_epoch = clock.time_epoch;
    }
    while (runtime_running())
    {
        uint64_t now_us = runtime_now_us();
        /* Marked before any blocking read: a stalled wire must not stop the
         * supervisor heartbeat check from seeing a live task. */
        runtime_data_mark_heartbeat(RUNTIME_OWNER_SIM, now_us);
        task_sim_step(now_us, &next_imu_us, &next_actuator_us, &next_heartbeat_us);
        rt_thread_delay_until(&last, rt_tick_from_millisecond(TASK_SIM_PERIOD_MS));
    }
}

const TaskDescriptor task_sim = {
    "sim",
    task_sim_entry,
    task_sim_stack,
    sizeof(task_sim_stack),
    TASK_SIM_PRIORITY,
};

#endif /* FLIGHT_SIM_UNIT_TEST */
