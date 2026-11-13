#include "runtime_internal.h"
#include "calibration/sensor_calibration_service.h"
#include "sample_acquisition/sensor_input.h"
#include "device_adapter/actuator_device.h"
#include "output_guard/actuator_guard.h"
#include "ins_interface.h"
#include "control_interface.h"
#include "fms_interface.h"
#include "guidance_update/guidance.h"
#include "telemetry_link/telemetry_link.h"
#include "parameter_store.h"
#include <math.h>
#include <string.h>
#if defined(FLIGHT_HOST_SIL) && defined(FLIGHT_GROUND_STATION)
void flight_sil_configure_controller(ProductControllerConfig *controller);
#endif
#ifdef FLIGHT_BENCH_TUNING
#include "task_config.h"
#include <finsh.h>
#include <math.h>
#include <stdlib.h>
#if defined(FINSH_THREAD_PRIORITY) && FINSH_THREAD_PRIORITY <= TASK_MPC_PRIORITY
#error "The bench console must run below the control owners"
#endif
#endif
#define LOG_TAG "runtime"
#include <ulog.h>

typedef struct {
    ProductProfile profile;
    ProductEstimatorConfig estimator;
    ProductControllerConfig controller;
    ProductFmsConfig fms;
    ProductGuidanceConfig guidance;
    ProductActuatorConfig actuator;
    sensor_imu_config_t imu;
    uint32_t provisional;
} RuntimeConfig;

static RuntimeConfig config;
static float base_imu_rotation[3][3];
static volatile uint8_t running;
static rt_base_t permit_irq_level;

/* Output policy of this image. BENCH_INHIBIT (1) is the only policy allowed in
 * a flight build. The Host SIL harness (tests/simulation/sil_flight.c) defines
 * FLIGHT_HOST_SIL to use SIM_CONTROLLED_STOP (2), which spec 04 section M7
 * restricts to Host/Plant targets and which never exists as a motor driver
 * here: the SIL replaces the guard driver with a recording stub. */
#if defined(FLIGHT_HOST_SIL) && !defined(FLIGHT_HOST_TEST)
#error "Simulation output policy requires the Host test target"
#endif
#ifdef FLIGHT_HOST_SIL
#define FLIGHT_OUTPUT_POLICY_ID 2u
#define FLIGHT_VERIFIED_INPUT_MASK ((1u << PREARM_ID_COUNT) - 1u)
#else
#define FLIGHT_OUTPUT_POLICY_ID 1u
#define FLIGHT_VERIFIED_INPUT_MASK 0u
#endif

static void permit_lock(void)
{
    permit_irq_level = rt_hw_interrupt_disable();
}

static void permit_unlock(void)
{
    rt_hw_interrupt_enable(permit_irq_level);
}

static void signal_imu_ready(void)
{
    if (running)
    {
        (void)rt_sem_release(&runtime_data.imu_event);
    }
}

static void load_bench_config(RuntimeConfig *out)
{
    memset(out, 0, sizeof(*out));
    out->provisional = 1u;
    out->profile = (ProductProfile){
        .product_profile_id = PRODUCT_PROFILE_ID,
        .profile_version = PRODUCT_CONTRACT_REVISION,
        .profile_hash = PRODUCT_PROFILE_ID,
        .nav_layout_id = PRODUCT_NAV_LAYOUT_ID,
        .motion_reference_layout_id = PRODUCT_MOTION_REFERENCE_LAYOUT_ID,
        .actuator_layout_id = PRODUCT_ACTUATOR_LAYOUT_ID,
        .fms_policy_id = fms_selected_policy_id(),
        .estimator_impl_id = PRODUCT_ESTIMATOR_IMPL_ID,
        .guidance_impl_id = PRODUCT_GUIDANCE_IMPL_ID,
        .controller_impl_id = PRODUCT_CONTROLLER_IMPL_ID,
        .actuator_adapter_id = PRODUCT_ACTUATOR_ADAPTER_ID,
        .control_model_id = PRODUCT_CONTROL_MODEL_ID,
    };
    ins_sim_config(&out->estimator);
    /* Use a provisional local origin until GNSS supplies a valid candidate. */
#ifdef FLIGHT_HOST_SIL
    /* SIL profile: the bench supplies a complete origin so the estimator
     * accepts GNSS observations instead of staging them forever (see
     * KF-GINS bootstrap). The flight image keeps the 1-bit placeholder
     * and must obtain a real origin through M4 authorization. */
    out->estimator.payload.initial_origin_valid_mask = 3u;
#else
    out->estimator.payload.initial_origin_valid_mask = 1u;
#endif
    out->estimator.payload.initial_latitude_deg = 0;
    out->estimator.payload.initial_longitude_deg = 0;
    out->estimator.payload.initial_ellipsoid_height_m = 0;
    out->estimator.payload.aiding_mask &= ~PRODUCT_AIDING_MAG;
#ifdef FLIGHT_HOST_SIL
    /* SIL profile assumption: the bench has a calibrated magnetometer, so MAG
     * aiding is enabled here and nowhere else. Without it NAV_VALID_HEADING can
     * never be published and prearm item COMPASS_HEADING can never PASS. */
    out->estimator.payload.aiding_mask |= PRODUCT_AIDING_MAG;
#endif

    control_sim_config(&out->controller);
#if defined(FLIGHT_HOST_SIL) && defined(FLIGHT_GROUND_STATION)
    flight_sil_configure_controller(&out->controller);
#endif
    fms_default_config(&out->fms);
    out->fms.payload.output_policy_id = FLIGHT_OUTPUT_POLICY_ID;
    out->fms.payload.verified_input_mask = FLIGHT_VERIFIED_INPUT_MASK;
    guidance_default_config(&out->guidance);
    out->guidance.payload.mass_kg = out->controller.payload.mass_kg;
    out->guidance.payload.gravity_mps2 = 9.80665f;
    out->actuator.profile_hash = out->profile.profile_hash;
    out->actuator.config_version = 1u;
    out->actuator.payload.output_policy_id = FLIGHT_OUTPUT_POLICY_ID;
    out->actuator.payload.esc_bit_rate = 300000u;
#ifdef FLIGHT_HOST_SIL
    /* SIM_CONTROLLED_STOP requires the calibration identity and a monotone
     * thrust curve that BENCH_INHIBIT never reads. These numbers are the SIL
     * fixture only: they are sized so a 5 N demand sits mid-curve and the
     * guard's clamp/slew path is exercised. The flight image carries the
     * product calibration instead and never compiles this block. */
    out->actuator.calibration_hash = 1u;
    out->actuator.calibration_set_id = 1u;
    out->actuator.payload.calibration_version = 1u;
    for (unsigned motor = 0; motor < PRODUCT_ACTUATOR_COUNT; ++motor)
    {
        out->actuator.payload.motor_to_timer_channel[motor] = (uint8_t)(motor + 1u);
        out->actuator.payload.thrust_curve_count[motor] = 2u;
        out->actuator.payload.thrust_curve_n[motor][0] = 0.0f;
        out->actuator.payload.thrust_curve_n[motor][1] = 10.0f;
        out->actuator.payload.dshot_curve_code[motor][0] = 48u;
        out->actuator.payload.dshot_curve_code[motor][1] = 2047u;
        out->actuator.payload.min_thrust_n[motor] = 0.0f;
        out->actuator.payload.max_thrust_n[motor] = 10.0f;
        out->actuator.payload.max_slew_nps[motor] = 100.0f;
    }
#endif

    sensor_imu_default_encoding(&out->imu);
    out->imu.pipeline_delay_uncertainty_us = 1u;
    out->imu.calibration_version = 1u;
    for (unsigned axis = 0; axis < 3u; ++axis)
    {
        out->imu.gyro_scale[axis] = out->imu.accel_scale[axis] = 1;
        out->imu.gyro_misalignment[axis][axis] = out->imu.accel_misalignment[axis][axis] = 1;
        out->imu.install_rotation[axis][axis] = 1;
    }
    memcpy(base_imu_rotation, out->imu.install_rotation, sizeof(base_imu_rotation));
}

static int apply_parameter_payload(const ProductControllerConfigPayload *payload)
{
    if (!runtime_running())
        return 2; /* deferred: runtime not started, keep the session value */
    return runtime_apply_controller_parameters(payload) == STATUS_OK ? 0 : 1;
}

static float calibration_value(const char *name)
{
    const int index = parameter_store_find(name);
    return index >= 0 ? parameter_store_value((unsigned)index) : 0.0f;
}

static bool calibration_flag(const char *name)
{
    return calibration_value(name) >= 0.5f;
}

static void store_calibration_value(const char *name, float value)
{
    const int index = parameter_store_find(name);
    if (index >= 0)
        (void)parameter_store_store((unsigned)index, value);
}

static void rotation_multiply(const float a[3][3], const float b[3][3], float out[3][3])
{
    for (unsigned row = 0; row < 3; ++row) {
        for (unsigned column = 0; column < 3; ++column) {
            out[row][column] = a[row][0] * b[0][column] + a[row][1] * b[1][column] +
                               a[row][2] * b[2][column];
        }
    }
}

/* Build the trim that makes the measured gravity vector point along body Z
 * while the vehicle is level.  Angles are the roll/pitch observed during the
 * level calibration, so applying R = Ry(-pitch) * Rx(roll) zeroes them. */
static void level_trim_rotation(float roll_deg, float pitch_deg, float out[3][3])
{
    const float roll = roll_deg * (float)(3.14159265358979323846 / 180.0);
    const float pitch = pitch_deg * (float)(3.14159265358979323846 / 180.0);
    const float cr = cosf(roll), sr = sinf(roll);
    const float cp = cosf(pitch), sp = sinf(pitch);
    const float rx[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, cr, -sr}, {0.0f, sr, cr}};
    const float ry[3][3] = {{cp, 0.0f, -sp}, {0.0f, 1.0f, 0.0f}, {sp, 0.0f, cp}};
    rotation_multiply(ry, rx, out);
}

static void runtime_refresh_calibration(void)
{
    float gyro_bias[3], accel_bias[3], accel_scale[3];
    float mag_offset[3], mag_scale[3];
    float trim[3][3], rotation[3][3];
    static const float identity[3][3] = {{1.0f, 0.0f, 0.0f},
                                         {0.0f, 1.0f, 0.0f},
                                         {0.0f, 0.0f, 1.0f}};

    for (unsigned axis = 0; axis < 3; ++axis) {
        static const char *const gyro_names[3] = {"CAL_GYRO_X", "CAL_GYRO_Y", "CAL_GYRO_Z"};
        static const char *const accel_names[3] = {"CAL_ACC_X", "CAL_ACC_Y", "CAL_ACC_Z"};
        static const char *const accel_scale_names[3] = {"CAL_ACC_SX", "CAL_ACC_SY", "CAL_ACC_SZ"};
        static const char *const mag_names[3] = {"CAL_MAG_X", "CAL_MAG_Y", "CAL_MAG_Z"};
        static const char *const mag_scale_names[3] = {"CAL_MAG_SX", "CAL_MAG_SY", "CAL_MAG_SZ"};
        gyro_bias[axis] = calibration_flag("CAL_GYRO_OK") ? calibration_value(gyro_names[axis]) : 0.0f;
        accel_bias[axis] = calibration_flag("CAL_ACC_OK") ? calibration_value(accel_names[axis]) : 0.0f;
        accel_scale[axis] = calibration_flag("CAL_ACC_OK") ? calibration_value(accel_scale_names[axis]) : 1.0f;
        mag_offset[axis] = calibration_flag("CAL_MAG_OK") ? calibration_value(mag_names[axis]) : 0.0f;
        mag_scale[axis] = calibration_flag("CAL_MAG_OK") ? calibration_value(mag_scale_names[axis]) : 1.0f;
    }
    if (calibration_flag("CAL_LVL_OK"))
        level_trim_rotation(calibration_value("CAL_LVL_ROLL"),
                            calibration_value("CAL_LVL_PITCH"), trim);
    else
        memcpy(trim, identity, sizeof(trim));
    rotation_multiply(trim, base_imu_rotation, rotation);

    rt_enter_critical();
    memcpy(config.imu.gyro_bias_dps, gyro_bias, sizeof(gyro_bias));
    memcpy(config.imu.accel_bias_mps2, accel_bias, sizeof(accel_bias));
    memcpy(config.imu.accel_scale, accel_scale, sizeof(accel_scale));
    memcpy(config.imu.install_rotation, rotation, sizeof(rotation));
    rt_exit_critical();
    sensor_mag_set_calibration(mag_offset, mag_scale);
    sensor_baro_set_reference(calibration_value("CAL_BARO_P0"));
}

static void calibration_complete(const SensorCalibrationStatus *status)
{
    if (!status || status->state != SENSOR_CAL_DONE)
        return;
    /* The service measures the residual after the current calibration was
     * applied.  Accumulate it instead of replacing the previous values, so a
     * repeated calibration converges and a failed run does not erase a good
     * one. */
    switch (status->kind) {
    case SENSOR_CAL_GYRO:
        store_calibration_value("CAL_GYRO_X", calibration_value("CAL_GYRO_X") + status->gyro_bias_dps[0]);
        store_calibration_value("CAL_GYRO_Y", calibration_value("CAL_GYRO_Y") + status->gyro_bias_dps[1]);
        store_calibration_value("CAL_GYRO_Z", calibration_value("CAL_GYRO_Z") + status->gyro_bias_dps[2]);
        store_calibration_value("CAL_GYRO_OK", 1.0f);
        break;
    case SENSOR_CAL_ACCEL:
        store_calibration_value("CAL_ACC_X", calibration_value("CAL_ACC_X") +
                                status->accel_bias_mps2[0] / calibration_value("CAL_ACC_SX"));
        store_calibration_value("CAL_ACC_Y", calibration_value("CAL_ACC_Y") +
                                status->accel_bias_mps2[1] / calibration_value("CAL_ACC_SY"));
        store_calibration_value("CAL_ACC_Z", calibration_value("CAL_ACC_Z") +
                                status->accel_bias_mps2[2] / calibration_value("CAL_ACC_SZ"));
        store_calibration_value("CAL_ACC_SX", calibration_value("CAL_ACC_SX") * status->accel_scale[0]);
        store_calibration_value("CAL_ACC_SY", calibration_value("CAL_ACC_SY") * status->accel_scale[1]);
        store_calibration_value("CAL_ACC_SZ", calibration_value("CAL_ACC_SZ") * status->accel_scale[2]);
        store_calibration_value("CAL_ACC_OK", 1.0f);
        break;
    case SENSOR_CAL_MAG:
        store_calibration_value("CAL_MAG_X", calibration_value("CAL_MAG_X") +
                                status->mag_offset_ut[0] / calibration_value("CAL_MAG_SX"));
        store_calibration_value("CAL_MAG_Y", calibration_value("CAL_MAG_Y") +
                                status->mag_offset_ut[1] / calibration_value("CAL_MAG_SY"));
        store_calibration_value("CAL_MAG_Z", calibration_value("CAL_MAG_Z") +
                                status->mag_offset_ut[2] / calibration_value("CAL_MAG_SZ"));
        store_calibration_value("CAL_MAG_SX", calibration_value("CAL_MAG_SX") * status->mag_scale[0]);
        store_calibration_value("CAL_MAG_SY", calibration_value("CAL_MAG_SY") * status->mag_scale[1]);
        store_calibration_value("CAL_MAG_SZ", calibration_value("CAL_MAG_SZ") * status->mag_scale[2]);
        store_calibration_value("CAL_MAG_OK", 1.0f);
        break;
    case SENSOR_CAL_LEVEL:
        store_calibration_value("CAL_LVL_ROLL", calibration_value("CAL_LVL_ROLL") + status->level_roll_deg);
        store_calibration_value("CAL_LVL_PITCH", calibration_value("CAL_LVL_PITCH") + status->level_pitch_deg);
        store_calibration_value("CAL_LVL_OK", 1.0f);
        break;
    case SENSOR_CAL_BARO:
        store_calibration_value("CAL_BARO_P0", status->baro_p0_pa);
        break;
    case SENSOR_CAL_RC:
        for (unsigned channel = 0; channel < SENSOR_CAL_RC_CHANNELS; ++channel) {
            static const char *const min_names[4] = {"RC_MIN_1", "RC_MIN_2", "RC_MIN_3", "RC_MIN_4"};
            static const char *const max_names[4] = {"RC_MAX_1", "RC_MAX_2", "RC_MAX_3", "RC_MAX_4"};
            static const char *const trim_names[4] = {"RC_TRIM_1", "RC_TRIM_2", "RC_TRIM_3", "RC_TRIM_4"};
            store_calibration_value(min_names[channel], (float)status->rc_min[channel]);
            store_calibration_value(max_names[channel], (float)status->rc_max[channel]);
            store_calibration_value(trim_names[channel], (float)status->rc_trim[channel]);
        }
        store_calibration_value("RC_CAL_OK", 1.0f);
        break;
    default:
        break;
    }
    runtime_refresh_calibration();
}

static void calibration_parameter_changed(unsigned index)
{
    const char *name = parameter_store_name(index);
    if (name && (strncmp(name, "CAL_", 4u) == 0 || strncmp(name, "RC_", 3u) == 0))
        runtime_refresh_calibration();
}

Status runtime_apply_controller_parameters(const ProductControllerConfigPayload *payload)
{
    Status status;
    if (!payload || !running)
    {
        return STATUS_NOT_READY;
    }
    if (!runtime_data_lock(0))
    {
        return STATUS_NOT_READY;
    }
    if (runtime_data.fms.arm_state != ARM_STATE_DISARMED)
    {
        runtime_data_unlock();
        return STATUS_NOT_PERMITTED;
    }
    /* The local console runs below the control owners. Keep the bounded
     * configuration update indivisible to their next scheduler turn. */
    rt_enter_critical();
    status = controller_apply_payload(payload);
    runtime_data_unlock();
    rt_exit_critical();
    return status;
}

bool runtime_bench_inhibit_active(void)
{
    return FLIGHT_OUTPUT_POLICY_ID == 1u && runtime_running();
}

#ifdef FLIGHT_NOR_LOG_ENABLE
bool runtime_nor_writes_allowed(void)
{
    return runtime_bench_inhibit_active();
}
#endif

#ifdef FLIGHT_BENCH_TUNING
static bool bench_parameter_writable(unsigned index)
{
    return parameter_store_writable(index) &&
           strncmp(parameter_store_name(index), "CTL_", 4u) == 0;
}

static bool set_bench_parameter(unsigned index, float value)
{
    return runtime_running() && config.actuator.payload.output_policy_id == 1u &&
           bench_parameter_writable(index) && parameter_store_set(index, value);
}

static void print_bench_parameter(unsigned index)
{
    const float value = parameter_store_value(index);
    const double magnitude = fabs((double)value);
    if (magnitude >= UINT32_MAX)
    {
        rt_kprintf("%s outside display range\n", parameter_store_name(index));
        return;
    }
    const unsigned whole = (unsigned)magnitude;
    const unsigned fractional = (unsigned)((magnitude - whole) * 1000000.0 + 0.5);
    rt_kprintf("%s %s%u.%06u%s\n", parameter_store_name(index), value < 0 ? "-" : "",
               whole + fractional / 1000000u, fractional % 1000000u,
               bench_parameter_writable(index) ? "" : " (read only)");
}

static int flight_param(int argc, char **argv)
{
    if (!runtime_running())
    {
        rt_kprintf("Flight runtime is not ready.\n");
        return -RT_ERROR;
    }
    if (argc == 1)
    {
        for (unsigned index = 0; index < parameter_store_count(); ++index)
        {
            if (bench_parameter_writable(index))
            {
                print_bench_parameter(index);
                rt_thread_mdelay(5);
            }
        }
        return RT_EOK;
    }
    const int index = parameter_store_find(argv[1]);
    if (argc > 3 || index < 0)
    {
        rt_kprintf("flight_param [NAME [VALUE]]\n");
        return -RT_EINVAL;
    }
    if (argc == 3)
    {
        char *end;
        const float value = strtof(argv[2], &end);
        if (end == argv[2] || *end || !set_bench_parameter((unsigned)index, value))
        {
            rt_kprintf("Parameter unchanged. Check the range and bench state.\n");
            print_bench_parameter((unsigned)index);
            return -RT_ERROR;
        }
    }
    print_bench_parameter((unsigned)index);
    return RT_EOK;
}
MSH_CMD_EXPORT(flight_param, Read or adjust controller parameters for this bench session);
#endif

#if defined(FINSH_USING_MSH) && (defined(FLIGHT_LOGGER_ENABLE) || defined(FLIGHT_NOR_LOG_ENABLE))
#include <finsh.h>
static int flight_storage(int argc, char **argv)
{
    RuntimeStats stats;
    (void)argc;
    (void)argv;
    runtime_get_stats(&stats);
    rt_kprintf("SD recording=%u event_drops=%u write_errors=%u\n",
               stats.sd_recording, stats.log_event_drops, stats.sd_write_errors);
    rt_kprintf("NOR recording=%u critical_drops=%u io_errors=%u\n",
               stats.nor_recording, stats.critical_event_drops, stats.nor_io_errors);
    return RT_EOK;
}
MSH_CMD_EXPORT(flight_storage, Show SD and NOR recording status);
#endif

int runtime_initialize(void)
{
    static const ActuatorDriver driver = {
        .lock = permit_lock,
        .unlock = permit_unlock,
        .inhibit = actuator_device_inhibit,
        .commit = actuator_device_commit,
        .poll_apply = actuator_device_poll_apply,
        .fault_count = actuator_device_faults,
    };
    ClockInstant clock;

    if (running)
    {
        return -RT_EBUSY;
    }
    if (clock_read(&clock))
    {
        return -RT_ERROR;
    }
    load_bench_config(&config);
    if (!config.provisional || config.actuator.payload.output_policy_id != FLIGHT_OUTPUT_POLICY_ID ||
        config.fms.payload.output_policy_id != FLIGHT_OUTPUT_POLICY_ID)
    {
        return -RT_ERROR;
    }
    /* The parameter catalog is part of the runtime contract, not a bench
     * convenience: the ground station must be able to read every exposed
     * parameter and apply a disarmed session update on the target as well. */
    parameter_store_init(&config.controller.payload);
    parameter_store_set_apply_hook(apply_parameter_payload);
    parameter_store_set_value_hook(calibration_parameter_changed);
    sensor_calibration_service_init();
    sensor_calibration_service_set_complete(calibration_complete);
    runtime_refresh_calibration();
    if (runtime_data_init(clock.monotonic_us) != RT_EOK)
    {
        return -RT_ERROR;
    }
#ifdef FLIGHT_SIM_LINK
    /* The HIL link takes the telemetry UART before any task polls it; M1 then
     * never opens that device, so only task_sim reads HIL frames from it. */
    telemetry_link_release_radio();
#endif
    if (actuator_device_start() != STATUS_OK)
    {
        goto deinit_data;
    }
    if (sensor_imu_start(signal_imu_ready) != STATUS_OK)
    {
        goto close_actuator;
    }

#define INIT_PORT(call) do {                                              \
    Status result = (call);                                               \
    if (result != STATUS_OK)                                             \
    {                                                                    \
        LOG_E("Flight init %s failed: %d", #call, result);                \
        runtime_stats.last_error = result;                                \
        goto stop_runtime;                                               \
    }                                                                    \
} while (0)
    INIT_PORT(actuator_guard_set_driver(&driver));
    INIT_PORT(estimator_validate(&config.profile));
    INIT_PORT(controller_validate(&config.profile));
    INIT_PORT(fms_policy_validate(&config.profile));
    INIT_PORT(guidance_validate(&config.profile));
    INIT_PORT(actuator_validate(&config.profile));
    INIT_PORT(estimator_init(&config.estimator));
    INIT_PORT(controller_init(&config.controller));
    INIT_PORT(fms_policy_init(&config.fms));
    INIT_PORT(guidance_init(&config.guidance));
    INIT_PORT(actuator_init(&config.actuator));
#undef INIT_PORT
    task_sensor_prepare(&config.imu, &config.estimator.payload, &clock);
    (void)estimator_get_snapshot(&runtime_data.nav);
    (void)estimator_get_capability(&runtime_data.nav_cap);
    (void)actuator_poll(runtime_now_us(), &runtime_data.actuator);
    (void)actuator_get_capability(&runtime_data.actuator_cap);
    running = 1u;
    runtime_stats.started = 1u;
    runtime_stats.provisional_parameters = config.provisional;
    runtime_data_record_event(SOURCE_PLATFORM, 0, 19, 1, clock.monotonic_us);
#ifdef FLIGHT_HOST_SIL
    LOG_I("Flight pipeline running with SIM_CONTROLLED_STOP.");
#else
    LOG_I("Flight pipeline running: BENCH_INHIBIT; provisional parameters, ARM unavailable.");
#endif
    return RT_EOK;

stop_runtime:
    runtime_shutdown();
    return -RT_ERROR;
close_actuator:
    actuator_device_close();
deinit_data:
    runtime_data_deinit();
    return -RT_ERROR;
}

static void flight_status(void) {
  RuntimeStats s; runtime_get_stats(&s);
  rt_kprintf("runtime=%u provisional=%u imu=%u reject=%u nav=%u ctl=%u reject=%u plan=%u mpc=%u\n",
      s.started,s.provisional_parameters,s.imu_batches,s.imu_rejected,s.nav_updates,
      s.control_updates,s.control_rejected,s.planning_updates,s.mpc_updates);
  rt_kprintf("status nav=%d ctl=%d plan=%d queue_overrun=%u M7=%u (output inhibited)\n",
      s.navigation_status,s.control_status,s.planning_status,s.queue_overruns,(unsigned)actuator_guard_state());
}
MSH_CMD_EXPORT(flight_status, Flight pipeline diagnostics);

int runtime_running(void) { return running; }

void runtime_shutdown(void)
{
    running = 0u;
    sensor_imu_stop();
    actuator_device_inhibit();
    (void)actuator_stop();
    (void)guidance_stop();
    (void)fms_policy_stop();
    (void)controller_stop();
    (void)estimator_stop();
    actuator_device_close();
    runtime_stats.started = 0u;
    runtime_data_deinit();
}
