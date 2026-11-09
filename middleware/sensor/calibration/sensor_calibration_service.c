/* Runtime sensor calibration procedures.
 *
 * The service is deliberately transport-free: MAVLink command handling starts
 * a procedure, the sensor/RC owners feed samples, and a completion callback
 * publishes the result as session parameters.  A short spin lock protects the
 * shared state because IMU, magnetometer and RC samples arrive from different
 * task contexts; every critical section only copies or accumulates a handful
 * of floats.
 */
#include "calibration/sensor_calibration_service.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef FLIGHT_HOST_TEST
#include <rtthread.h>
#endif

#define GRAVITY_MPS2 9.80665f
#define DEG_PER_RAD 57.29577951308232f

enum {
    GYRO_DURATION_US = 1500000,
    GYRO_MIN_SAMPLES = 500,
    ACCEL_MIN_SAMPLES = 100,
    ACCEL_TIMEOUT_US = 120000000,
    MAG_DURATION_US = 20000000,
    MAG_MIN_SAMPLES = 200,
    MAG_TIMEOUT_US = 60000000,
    LEVEL_DURATION_US = 1000000,
    LEVEL_MIN_SAMPLES = 200,
    BARO_DURATION_US = 1000000,
    BARO_MIN_SAMPLES = 20,
    RC_TIMEOUT_US = 60000000,
    RC_MIN_SPAN = 300,
    FEED_TIMEOUT_US = 5000000,
};

typedef struct {
    uint8_t kind;
    uint8_t state;
    uint8_t face_mask;
    uint16_t progress_permille;
    uint32_t started_us;
    uint32_t updated_us;
    uint32_t last_feed_us;
    uint32_t sample_count;
    char message[48];

    float gyro_sum[3];
    float gyro_square_sum[3];
    uint32_t gyro_count;

    float accel_sum[3];
    uint32_t accel_count;
    float face_sum[SENSOR_CAL_ACCEL_FACES][3];
    uint32_t face_count[SENSOR_CAL_ACCEL_FACES];

    float mag_min[3];
    float mag_max[3];
    uint32_t mag_count;
    float mag_sum[3], mag_square[3], mag_cross[3];

    float baro_sum;
    uint32_t baro_count;

    uint16_t rc_min[SENSOR_CAL_RC_CHANNELS];
    uint16_t rc_max[SENSOR_CAL_RC_CHANNELS];
    uint8_t rc_seen_mask;
    uint16_t rc_latest[SENSOR_CAL_RC_CHANNELS];
    uint64_t rc_latest_us;

    float gyro_bias_dps[3];
    float accel_bias_mps2[3];
    float accel_scale[3];
    float mag_offset_ut[3];
    float mag_scale[3];
    float level_roll_deg;
    float level_pitch_deg;
    float baro_p0_pa;
    uint16_t rc_cal_min[SENSOR_CAL_RC_CHANNELS];
    uint16_t rc_cal_max[SENSOR_CAL_RC_CHANNELS];
    uint16_t rc_cal_trim[SENSOR_CAL_RC_CHANNELS];
} CalibrationState;

static CalibrationState state;
#ifdef FLIGHT_HOST_TEST
static volatile int state_lock;
#else
static rt_base_t state_lock;
#endif
static SensorCalibrationComplete complete_callback;
static uint32_t last_notified_us;

static void lock_state(void)
{
#ifdef FLIGHT_HOST_TEST
    while (__sync_lock_test_and_set(&state_lock, 1)) {
    }
#else
    state_lock = rt_hw_interrupt_disable();
#endif
}

static void unlock_state(void)
{
#ifdef FLIGHT_HOST_TEST
    __sync_lock_release(&state_lock);
#else
    rt_hw_interrupt_enable(state_lock);
#endif
}

static void set_message(const char *message)
{
    snprintf(state.message, sizeof(state.message), "%s", message);
}

static float clampf(float value, float low, float high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

static unsigned popcount6(uint8_t mask)
{
    unsigned count = 0;
    for (unsigned bit = 0; bit < SENSOR_CAL_ACCEL_FACES; ++bit)
        count += (unsigned)((mask >> bit) & 1u);
    return count;
}

void sensor_calibration_service_init(void)
{
    lock_state();
    memset(&state, 0, sizeof(state));
    state.kind = SENSOR_CAL_NONE;
    state.state = SENSOR_CAL_IDLE;
    last_notified_us = 0;
    state.accel_scale[0] = state.accel_scale[1] = state.accel_scale[2] = 1.0f;
    state.mag_scale[0] = state.mag_scale[1] = state.mag_scale[2] = 1.0f;
    state.baro_p0_pa = 101325.0f;
    unlock_state();
    complete_callback = 0;
}

void sensor_calibration_service_set_complete(SensorCalibrationComplete callback)
{
    complete_callback = callback;
}

static void reset_accumulators(void)
{
    memset(state.gyro_sum, 0, sizeof(state.gyro_sum));
    memset(state.gyro_square_sum, 0, sizeof(state.gyro_square_sum));
    state.gyro_count = 0;
    memset(state.accel_sum, 0, sizeof(state.accel_sum));
    state.accel_count = 0;
    memset(state.face_sum, 0, sizeof(state.face_sum));
    memset(state.face_count, 0, sizeof(state.face_count));
    state.face_mask = 0;
    for (unsigned axis = 0; axis < 3; ++axis) {
        state.mag_min[axis] = 1.0e30f;
        state.mag_max[axis] = -1.0e30f;
    }
    state.mag_count = 0;
    memset(state.mag_sum, 0, sizeof(state.mag_sum));
    memset(state.mag_square, 0, sizeof(state.mag_square));
    memset(state.mag_cross, 0, sizeof(state.mag_cross));
    state.baro_sum = 0.0f;
    state.baro_count = 0;
    for (unsigned channel = 0; channel < SENSOR_CAL_RC_CHANNELS; ++channel) {
        state.rc_min[channel] = UINT16_MAX;
        state.rc_max[channel] = 0;
    }
    state.rc_seen_mask = 0;
    state.sample_count = 0;
    state.progress_permille = 0;
}

bool sensor_calibration_start(uint8_t kind, uint64_t now_us,
                              char *error, unsigned error_capacity)
{
    bool started = false;
    lock_state();
    if (state.state == SENSOR_CAL_RUNNING) {
        if (error && error_capacity)
            snprintf(error, error_capacity, "another calibration is running");
    } else if (kind < SENSOR_CAL_GYRO || kind > SENSOR_CAL_RC) {
        if (error && error_capacity)
            snprintf(error, error_capacity, "unsupported calibration");
    } else {
        reset_accumulators();
        state.kind = kind;
        state.state = SENSOR_CAL_RUNNING;
        state.started_us = (uint32_t)now_us;
        state.updated_us = (uint32_t)now_us;
        state.last_feed_us = (uint32_t)now_us;
        last_notified_us = 0;
        switch (kind) {
        case SENSOR_CAL_GYRO:
            set_message("GYRO CAL: keep the vehicle still");
            break;
        case SENSOR_CAL_ACCEL:
            set_message("ACCEL CAL: place -Z up and capture face 1/6");
            break;
        case SENSOR_CAL_MAG:
            set_message("MAG CAL: rotate the vehicle slowly");
            break;
        case SENSOR_CAL_LEVEL:
            set_message("LEVEL CAL: keep the vehicle level and still");
            break;
        case SENSOR_CAL_BARO:
            set_message("BARO CAL: keep the vehicle still");
            break;
        case SENSOR_CAL_RC:
            set_message("RC CAL: move all sticks to their limits");
            break;
        default:
            break;
        }
        started = true;
    }
    unlock_state();
    return started;
}

void sensor_calibration_cancel(uint64_t now_us)
{
    lock_state();
    if (state.state == SENSOR_CAL_RUNNING) {
        state.state = SENSOR_CAL_FAILED;
        state.updated_us = (uint32_t)now_us;
        set_message("calibration cancelled");
    }
    unlock_state();
}

static void finish_locked(uint8_t result, const char *message, uint64_t now_us)
{
    state.state = result;
    state.progress_permille = result == SENSOR_CAL_DONE ? 1000 : state.progress_permille;
    state.updated_us = (uint32_t)now_us;
    set_message(message);
}

static void complete_gyro_locked(uint64_t now_us)
{
    if (state.gyro_count < GYRO_MIN_SAMPLES) {
        finish_locked(SENSOR_CAL_FAILED, "GYRO CAL: not enough samples", now_us);
        return;
    }
    float mean[3], variance[3];
    for (unsigned axis = 0; axis < 3; ++axis) {
        mean[axis] = state.gyro_sum[axis] / (float)state.gyro_count;
        variance[axis] = state.gyro_square_sum[axis] / (float)state.gyro_count -
                         mean[axis] * mean[axis];
        if (variance[axis] < 0.0f)
            variance[axis] = 0.0f;
        if (sqrtf(variance[axis]) > 3.0f) {
            finish_locked(SENSOR_CAL_FAILED, "GYRO CAL: motion detected", now_us);
            return;
        }
        if (fabsf(mean[axis]) > 20.0f) {
            finish_locked(SENSOR_CAL_FAILED, "GYRO CAL: bias out of range", now_us);
            return;
        }
        state.gyro_bias_dps[axis] = mean[axis];
    }
    finish_locked(SENSOR_CAL_DONE, "GYRO CAL done", now_us);
}

static void complete_accel_locked(uint64_t now_us)
{
    if (state.face_mask != (uint8_t)((1u << SENSOR_CAL_ACCEL_FACES) - 1u)) {
        finish_locked(SENSOR_CAL_FAILED, "ACCEL CAL: not all six faces captured", now_us);
        return;
    }
    /* Faces 0/1 are -Z/+Z up, 2/3 +X/-X, 4/5 +Y/-Y. */
    static const uint8_t plus_face[3] = {2, 4, 1};   /* +X, +Y, +Z up */
    static const uint8_t minus_face[3] = {3, 5, 0};  /* -X, -Y, -Z up */
    for (unsigned axis = 0; axis < 3; ++axis) {
        const float plus = state.face_sum[plus_face[axis]][axis] /
                           (float)state.face_count[plus_face[axis]];
        const float minus = state.face_sum[minus_face[axis]][axis] /
                            (float)state.face_count[minus_face[axis]];
        const float span = plus - minus;
        const float scale = 2.0f * GRAVITY_MPS2 / span;
        if (!isfinite(scale) || scale < 0.8f || scale > 1.2f) {
            finish_locked(SENSOR_CAL_FAILED, "ACCEL CAL: span out of range", now_us);
            return;
        }
        state.accel_bias_mps2[axis] = 0.5f * (plus + minus);
        state.accel_scale[axis] = scale;
        if (fabsf(state.accel_bias_mps2[axis]) > 5.0f) {
            finish_locked(SENSOR_CAL_FAILED, "ACCEL CAL: bias out of range", now_us);
            return;
        }
    }
    finish_locked(SENSOR_CAL_DONE, "ACCEL CAL done", now_us);
}

/* A rotation in one plane can still have large extrema on all three axes.
 * The determinant of the normalised covariance detects that missing axis. */
static bool mag_coverage_ready(void)
{
    const float count = (float)state.mag_count;
    float variance[3], correlation[3];
    for (unsigned i = 0; i < 3; ++i) {
        variance[i] = (state.mag_square[i] - state.mag_sum[i] * state.mag_sum[i] / count) / count;
        if (!(variance[i] > 16.0f))
            return false;
    }
    for (unsigned i = 0; i < 3; ++i) {
        const unsigned j = (i + 1u) % 3u;
        const float covariance = (state.mag_cross[i] - state.mag_sum[i] * state.mag_sum[j] / count) / count;
        correlation[i] = covariance / sqrtf(variance[i] * variance[j]);
    }
    const float xy = correlation[0], yz = correlation[1], zx = correlation[2];
    return 1.0f + 2.0f * xy * yz * zx - xy * xy - yz * yz - zx * zx > 0.05f;
}

static void complete_mag_locked(uint64_t now_us)
{
    float span[3], radius = 0.0f;
    if (state.mag_count < MAG_MIN_SAMPLES) {
        finish_locked(SENSOR_CAL_FAILED, "MAG CAL: not enough samples", now_us);
        return;
    }
    for (unsigned axis = 0; axis < 3; ++axis) {
        span[axis] = state.mag_max[axis] - state.mag_min[axis];
        if (span[axis] < 20.0f) {
            finish_locked(SENSOR_CAL_FAILED, "MAG CAL: rotate through all axes", now_us);
            return;
        }
        radius += span[axis];
    }
    radius /= 6.0f; /* mean half-span */
    for (unsigned axis = 0; axis < 3; ++axis) {
        state.mag_offset_ut[axis] = 0.5f * (state.mag_max[axis] + state.mag_min[axis]);
        state.mag_scale[axis] = clampf(radius / (0.5f * span[axis]), 0.5f, 2.0f);
        if (fabsf(state.mag_offset_ut[axis]) > 500.0f) {
            finish_locked(SENSOR_CAL_FAILED, "MAG CAL: offset out of range", now_us);
            return;
        }
    }
    finish_locked(SENSOR_CAL_DONE, "MAG CAL done", now_us);
}

static void complete_level_locked(uint64_t now_us)
{
    if (state.accel_count < LEVEL_MIN_SAMPLES) {
        finish_locked(SENSOR_CAL_FAILED, "LEVEL CAL: not enough samples", now_us);
        return;
    }
    const float ax = state.accel_sum[0] / (float)state.accel_count;
    const float ay = state.accel_sum[1] / (float)state.accel_count;
    const float az = state.accel_sum[2] / (float)state.accel_count;
    const float roll = atan2f(ay, sqrtf(ax * ax + az * az)) * DEG_PER_RAD;
    const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * DEG_PER_RAD;
    if (az > -0.8f * GRAVITY_MPS2 || fabsf(roll) > 10.0f || fabsf(pitch) > 10.0f) {
        finish_locked(SENSOR_CAL_FAILED, "LEVEL CAL: vehicle is not level", now_us);
        return;
    }
    state.level_roll_deg = roll;
    state.level_pitch_deg = pitch;
    finish_locked(SENSOR_CAL_DONE, "LEVEL CAL done", now_us);
}

static void complete_baro_locked(uint64_t now_us)
{
    if (state.baro_count < BARO_MIN_SAMPLES) {
        finish_locked(SENSOR_CAL_FAILED, "BARO CAL: not enough samples", now_us);
        return;
    }
    const float pressure = state.baro_sum / (float)state.baro_count;
    if (!(pressure > 80000.0f && pressure < 110000.0f)) {
        finish_locked(SENSOR_CAL_FAILED, "BARO CAL: pressure out of range", now_us);
        return;
    }
    state.baro_p0_pa = pressure;
    finish_locked(SENSOR_CAL_DONE, "BARO CAL done", now_us);
}

static bool rc_travel_ready(void)
{
    for (unsigned channel = 0; channel < SENSOR_CAL_RC_CHANNELS; ++channel) {
        if (!(state.rc_seen_mask & (1u << channel)) ||
            state.rc_max[channel] - state.rc_min[channel] < RC_MIN_SPAN) {
            return false;
        }
    }
    return true;
}

bool sensor_calibration_finish_rc(uint64_t now_us, char *error, unsigned error_capacity)
{
    lock_state();
    const bool running = state.kind == SENSOR_CAL_RC && state.state == SENSOR_CAL_RUNNING;
    const bool fresh = (uint32_t)now_us - state.last_feed_us < 500000u;
    bool ready = running && fresh && rc_travel_ready();
    for (unsigned channel = 0; ready && channel < SENSOR_CAL_RC_CHANNELS; ++channel) {
        const unsigned span = state.rc_max[channel] - state.rc_min[channel];
        const unsigned above_min = state.rc_latest[channel] - state.rc_min[channel];
        ready = channel == 2 ? above_min <= span / 10u :
                above_min >= span / 4u && above_min <= span * 3u / 4u;
    }
    if (!ready) {
        if (error && error_capacity)
            snprintf(error, error_capacity, "RC: verify travel, centre and low throttle");
        unlock_state();
        return false;
    }
    for (unsigned channel = 0; channel < SENSOR_CAL_RC_CHANNELS; ++channel) {
        state.rc_cal_min[channel] = state.rc_min[channel];
        state.rc_cal_max[channel] = state.rc_max[channel];
        state.rc_cal_trim[channel] = state.rc_latest[channel];
    }
    /* Throttle low stick is the useful trim reference, not the mechanical centre. */
    state.rc_cal_trim[2] = state.rc_cal_min[2];
    finish_locked(SENSOR_CAL_DONE, "RC CAL done", now_us);
    unlock_state();
    return true;
}

bool sensor_calibration_copy_rc_trims(uint64_t now_us, char *error, unsigned error_capacity)
{
    lock_state();
    bool ready = state.state != SENSOR_CAL_RUNNING && state.rc_latest_us != 0u &&
                 now_us >= state.rc_latest_us && now_us - state.rc_latest_us < 500000u;
    for (unsigned i = 0; ready && i < SENSOR_CAL_RC_CHANNELS; ++i) {
        const int minimum = state.rc_cal_min[i];
        const int span = state.rc_cal_max[i] - minimum;
        const int position = state.rc_latest[i] - minimum;
        ready = span >= RC_MIN_SPAN && (i == 2u ? position >= -span / 20 && position <= span / 10 :
                    position >= span * 2 / 5 && position <= span * 3 / 5);
    }
    if (!ready) {
        if (error && error_capacity)
            snprintf(error, error_capacity, "RC: calibrate, center sticks, lower throttle");
        unlock_state();
        return false;
    }
    for (unsigned i = 0; i < SENSOR_CAL_RC_CHANNELS; ++i)
        if (i != 2u)
            state.rc_cal_trim[i] = state.rc_latest[i];
    state.kind = SENSOR_CAL_RC;
    state.started_us = (uint32_t)now_us;
    finish_locked(SENSOR_CAL_DONE, "RC TRIM copied", now_us);
    unlock_state();
    return true;
}

void sensor_calibration_tick(uint64_t now_us)
{
    SensorCalibrationStatus done;
    bool notify = false;
    lock_state();
    if (state.state == SENSOR_CAL_RUNNING) {
        const uint32_t elapsed = (uint32_t)now_us - state.started_us;
        state.updated_us = (uint32_t)now_us;
        if ((uint32_t)now_us - state.last_feed_us > FEED_TIMEOUT_US) {
            finish_locked(SENSOR_CAL_FAILED, "calibration: sensor data timeout", now_us);
        } else {
            switch (state.kind) {
            case SENSOR_CAL_GYRO:
                state.progress_permille = (uint16_t)((uint64_t)elapsed * 1000u /
                                                     GYRO_DURATION_US);
                if (state.progress_permille > 1000u)
                    state.progress_permille = 1000u;
                if (elapsed >= GYRO_DURATION_US)
                    complete_gyro_locked(now_us);
                break;
            case SENSOR_CAL_ACCEL:
                state.progress_permille = (uint16_t)(popcount6(state.face_mask) * 1000u /
                                                     SENSOR_CAL_ACCEL_FACES);
                if (elapsed > ACCEL_TIMEOUT_US)
                    finish_locked(SENSOR_CAL_FAILED, "ACCEL CAL: timeout", now_us);
                break;
            case SENSOR_CAL_MAG: {
                state.progress_permille = (uint16_t)((uint64_t)elapsed * 1000u /
                                                     MAG_DURATION_US);
                if (state.progress_permille > 1000u)
                    state.progress_permille = 1000u;
                float span[3];
                span[0] = state.mag_max[0] - state.mag_min[0];
                span[1] = state.mag_max[1] - state.mag_min[1];
                span[2] = state.mag_max[2] - state.mag_min[2];
                const bool enough_span = state.mag_count >= MAG_MIN_SAMPLES &&
                                         span[0] > 40.0f && span[1] > 40.0f && span[2] > 40.0f;
                if (elapsed >= MAG_DURATION_US && enough_span && mag_coverage_ready())
                    complete_mag_locked(now_us);
                else if (elapsed > MAG_TIMEOUT_US)
                    finish_locked(SENSOR_CAL_FAILED, "MAG CAL: incomplete three-axis coverage", now_us);
                break;
            }
            case SENSOR_CAL_LEVEL:
                state.progress_permille = (uint16_t)((uint64_t)elapsed * 1000u /
                                                     LEVEL_DURATION_US);
                if (state.progress_permille > 1000u)
                    state.progress_permille = 1000u;
                if (elapsed >= LEVEL_DURATION_US)
                    complete_level_locked(now_us);
                break;
            case SENSOR_CAL_BARO:
                state.progress_permille = (uint16_t)((uint64_t)elapsed * 1000u /
                                                     BARO_DURATION_US);
                if (state.progress_permille > 1000u)
                    state.progress_permille = 1000u;
                if (elapsed >= BARO_DURATION_US)
                    complete_baro_locked(now_us);
                break;
            case SENSOR_CAL_RC:
                state.progress_permille = rc_travel_ready() ? 1000u : 0u;
                if (elapsed >= RC_TIMEOUT_US)
                    finish_locked(SENSOR_CAL_FAILED, "RC CAL: timeout", now_us);
                break;
            default:
                break;
            }
        }
        if (state.state != SENSOR_CAL_RUNNING) {
            memset(&done, 0, sizeof(done));
            done.kind = state.kind;
            done.state = state.state;
            done.face_mask = state.face_mask;
            done.progress_permille = state.progress_permille;
            done.sample_count = state.sample_count;
            done.started_us = state.started_us;
            done.updated_us = state.updated_us;
            memcpy(done.message, state.message, sizeof(done.message));
            memcpy(done.gyro_bias_dps, state.gyro_bias_dps, sizeof(done.gyro_bias_dps));
            memcpy(done.accel_bias_mps2, state.accel_bias_mps2, sizeof(done.accel_bias_mps2));
            memcpy(done.accel_scale, state.accel_scale, sizeof(done.accel_scale));
            memcpy(done.mag_offset_ut, state.mag_offset_ut, sizeof(done.mag_offset_ut));
            memcpy(done.mag_scale, state.mag_scale, sizeof(done.mag_scale));
            done.level_roll_deg = state.level_roll_deg;
            done.level_pitch_deg = state.level_pitch_deg;
            done.baro_p0_pa = state.baro_p0_pa;
            memcpy(done.rc_min, state.rc_cal_min, sizeof(done.rc_min));
            memcpy(done.rc_max, state.rc_cal_max, sizeof(done.rc_max));
            memcpy(done.rc_trim, state.rc_cal_trim, sizeof(done.rc_trim));
            notify = true;
            last_notified_us = state.updated_us;
        }
    } else if ((state.state == SENSOR_CAL_DONE || state.state == SENSOR_CAL_FAILED) &&
               state.updated_us != last_notified_us) {
        /* The accelerometer completes inside capture_accel_face(), not in this
         * tick.  Publish that result on the next tick exactly once. */
        memset(&done, 0, sizeof(done));
        done.kind = state.kind;
        done.state = state.state;
        done.face_mask = state.face_mask;
        done.progress_permille = state.progress_permille;
        done.sample_count = state.sample_count;
        done.started_us = state.started_us;
        done.updated_us = state.updated_us;
        memcpy(done.message, state.message, sizeof(done.message));
        memcpy(done.gyro_bias_dps, state.gyro_bias_dps, sizeof(done.gyro_bias_dps));
        memcpy(done.accel_bias_mps2, state.accel_bias_mps2, sizeof(done.accel_bias_mps2));
        memcpy(done.accel_scale, state.accel_scale, sizeof(done.accel_scale));
        memcpy(done.mag_offset_ut, state.mag_offset_ut, sizeof(done.mag_offset_ut));
        memcpy(done.mag_scale, state.mag_scale, sizeof(done.mag_scale));
        done.level_roll_deg = state.level_roll_deg;
        done.level_pitch_deg = state.level_pitch_deg;
        done.baro_p0_pa = state.baro_p0_pa;
        memcpy(done.rc_min, state.rc_cal_min, sizeof(done.rc_min));
        memcpy(done.rc_max, state.rc_cal_max, sizeof(done.rc_max));
        memcpy(done.rc_trim, state.rc_cal_trim, sizeof(done.rc_trim));
        notify = true;
        last_notified_us = state.updated_us;
    }
    unlock_state();
    if (notify && complete_callback)
        complete_callback(&done);
}

bool sensor_calibration_capture_accel_face(uint8_t face, uint64_t now_us,
                                           char *error, unsigned error_capacity)
{
    bool captured = false;
    lock_state();
    if (state.kind != SENSOR_CAL_ACCEL || state.state != SENSOR_CAL_RUNNING) {
        if (error && error_capacity)
            snprintf(error, error_capacity, "accelerometer calibration is not running");
    } else if (face >= SENSOR_CAL_ACCEL_FACES) {
        if (error && error_capacity)
            snprintf(error, error_capacity, "invalid face");
    } else if (face != popcount6(state.face_mask)) {
        if (error && error_capacity)
            snprintf(error, error_capacity, "capture the next face in order");
    } else if (state.accel_count < ACCEL_MIN_SAMPLES ||
               (uint32_t)now_us - state.last_feed_us > 250000u) {
        if (error && error_capacity)
            snprintf(error, error_capacity, "hold the face still for a moment");
    } else {
        /* Reject a capture when the measured gravity is not dominated by the
         * axis that this face is supposed to expose. */
        static const uint8_t face_axis[SENSOR_CAL_ACCEL_FACES] = {2, 2, 0, 0, 1, 1};
        const uint8_t axis = face_axis[face];
        const float mean = state.accel_sum[axis] / (float)state.accel_count;
        bool other_axis_large = false;
        for (unsigned other = 0; other < 3; ++other) {
            if (other == axis)
                continue;
            const float value = state.accel_sum[other] / (float)state.accel_count;
            if (fabsf(value) > 0.5f * fabsf(mean))
                other_axis_large = true;
        }
        if (fabsf(mean) < 0.5f * GRAVITY_MPS2 || other_axis_large) {
            if (error && error_capacity)
                snprintf(error, error_capacity, "orientation does not match this face");
            unlock_state();
            return false;
        }
        memcpy(state.face_sum[face], state.accel_sum, sizeof(state.face_sum[face]));
        state.face_count[face] = state.accel_count;
        state.face_mask |= (uint8_t)(1u << face);
        state.accel_sum[0] = state.accel_sum[1] = state.accel_sum[2] = 0.0f;
        state.accel_count = 0;
        state.updated_us = (uint32_t)now_us;
        if (state.face_mask == (uint8_t)((1u << SENSOR_CAL_ACCEL_FACES) - 1u))
            complete_accel_locked(now_us);
        else
            snprintf(state.message, sizeof(state.message),
                     "ACCEL CAL: %u/6 faces captured", popcount6(state.face_mask));
        captured = true;
    }
    unlock_state();
    return captured;
}

void sensor_calibration_feed_imu(const ImuSample *sample, uint64_t now_us)
{
    if (!sample)
        return;
    lock_state();
    if (state.state == SENSOR_CAL_RUNNING) {
        state.last_feed_us = (uint32_t)now_us;
        state.sample_count++;
        if (state.kind == SENSOR_CAL_GYRO) {
            for (unsigned axis = 0; axis < 3; ++axis) {
                const float dps = sample->gyro_frd_radps[axis] * DEG_PER_RAD;
                state.gyro_sum[axis] += dps;
                state.gyro_square_sum[axis] += dps * dps;
            }
            state.gyro_count++;
        } else if (state.kind == SENSOR_CAL_ACCEL || state.kind == SENSOR_CAL_LEVEL) {
            const unsigned face = popcount6(state.face_mask);
            static const unsigned axis_for_face[] = {2, 2, 0, 0, 1, 1};
            const unsigned axis = state.kind == SENSOR_CAL_ACCEL ? axis_for_face[face] : 2u;
            const bool negative = state.kind == SENSOR_CAL_LEVEL || face == 0 || face == 3 || face == 5;
            const float sign = negative ? -1.0f : 1.0f;
            const float *a = sample->accel_frd_mps2;
            const float *w = sample->gyro_frd_radps;
            const float accel_squared = a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
            const float gyro_squared = w[0]*w[0] + w[1]*w[1] + w[2]*w[2];
            if (!(sign * a[axis] > 0.75f * GRAVITY_MPS2 &&
                  accel_squared < 1.6f * GRAVITY_MPS2 * GRAVITY_MPS2 &&
                  gyro_squared < 0.04f)) {
                memset(state.accel_sum, 0, sizeof(state.accel_sum));
                state.accel_count = 0;
                unlock_state();
                return;
            }
            const unsigned limit = state.kind == SENSOR_CAL_ACCEL ? ACCEL_MIN_SAMPLES : LEVEL_MIN_SAMPLES;
            if (state.accel_count >= limit) {
                for (unsigned i = 0; i < 3; ++i)
                    state.accel_sum[i] += a[i] - state.accel_sum[i] / (float)limit;
                unlock_state();
                return;
            }
            for (unsigned axis = 0; axis < 3; ++axis)
                state.accel_sum[axis] += sample->accel_frd_mps2[axis];
            state.accel_count++;
        }
    }
    unlock_state();
}

void sensor_calibration_feed_mag(const float ut[3], uint64_t now_us)
{
    if (!ut)
        return;
    lock_state();
    if (state.state == SENSOR_CAL_RUNNING && state.kind == SENSOR_CAL_MAG) {
        state.last_feed_us = (uint32_t)now_us;
        for (unsigned axis = 0; axis < 3; ++axis) {
            state.mag_sum[axis] += ut[axis];
            state.mag_square[axis] += ut[axis] * ut[axis];
            state.mag_cross[axis] += ut[axis] * ut[(axis + 1u) % 3u];
            if (ut[axis] < state.mag_min[axis])
                state.mag_min[axis] = ut[axis];
            if (ut[axis] > state.mag_max[axis])
                state.mag_max[axis] = ut[axis];
        }
        state.mag_count++;
        state.sample_count++;
    }
    unlock_state();
}

void sensor_calibration_feed_baro(float pressure_pa, uint64_t now_us)
{
    lock_state();
    if (state.state == SENSOR_CAL_RUNNING && state.kind == SENSOR_CAL_BARO) {
        state.last_feed_us = (uint32_t)now_us;
        state.baro_sum += pressure_pa;
        state.baro_count++;
        state.sample_count++;
    }
    unlock_state();
}

void sensor_calibration_feed_rc(const uint16_t channels[16], uint64_t now_us)
{
    if (!channels)
        return;
    lock_state();
    memcpy(state.rc_latest, channels, sizeof(state.rc_latest));
    state.rc_latest_us = now_us;
    if (state.state == SENSOR_CAL_RUNNING && state.kind == SENSOR_CAL_RC) {
        state.last_feed_us = (uint32_t)now_us;
        for (unsigned channel = 0; channel < SENSOR_CAL_RC_CHANNELS; ++channel) {
            if (channels[channel] < state.rc_min[channel])
                state.rc_min[channel] = channels[channel];
            if (channels[channel] > state.rc_max[channel])
                state.rc_max[channel] = channels[channel];
        }
        state.rc_seen_mask = 0x0fu;
        state.sample_count++;
    }
    unlock_state();
}

void sensor_calibration_get_status(SensorCalibrationStatus *out)
{
    if (!out)
        return;
    lock_state();
    memset(out, 0, sizeof(*out));
    out->kind = state.kind;
    out->state = state.state;
    out->face_mask = state.face_mask;
    out->progress_permille = state.progress_permille;
    out->sample_count = state.sample_count;
    out->started_us = state.started_us;
    out->updated_us = state.updated_us;
    memcpy(out->message, state.message, sizeof(out->message));
    memcpy(out->gyro_bias_dps, state.gyro_bias_dps, sizeof(out->gyro_bias_dps));
    memcpy(out->accel_bias_mps2, state.accel_bias_mps2, sizeof(out->accel_bias_mps2));
    memcpy(out->accel_scale, state.accel_scale, sizeof(out->accel_scale));
    memcpy(out->mag_offset_ut, state.mag_offset_ut, sizeof(out->mag_offset_ut));
    memcpy(out->mag_scale, state.mag_scale, sizeof(out->mag_scale));
    out->level_roll_deg = state.level_roll_deg;
    out->level_pitch_deg = state.level_pitch_deg;
    out->baro_p0_pa = state.baro_p0_pa;
    memcpy(out->rc_min, state.rc_cal_min, sizeof(out->rc_min));
    memcpy(out->rc_max, state.rc_cal_max, sizeof(out->rc_max));
    memcpy(out->rc_trim, state.rc_cal_trim, sizeof(out->rc_trim));
    unlock_state();
}
