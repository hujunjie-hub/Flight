#ifndef SENSOR_CALIBRATION_SERVICE_H
#define SENSOR_CALIBRATION_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "messages/common_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Calibration kinds are also used by the Flight status message. */
typedef enum {
    SENSOR_CAL_NONE = 0,
    SENSOR_CAL_GYRO = 1,
    SENSOR_CAL_ACCEL = 2,
    SENSOR_CAL_MAG = 3,
    SENSOR_CAL_LEVEL = 4,
    SENSOR_CAL_BARO = 5,
    SENSOR_CAL_RC = 6,
} SensorCalibrationKind;

typedef enum {
    SENSOR_CAL_IDLE = 0,
    SENSOR_CAL_RUNNING = 1,
    SENSOR_CAL_DONE = 2,
    SENSOR_CAL_FAILED = 3,
} SensorCalibrationState;

enum { SENSOR_CAL_ACCEL_FACES = 6, SENSOR_CAL_RC_CHANNELS = 4 };

typedef struct {
    uint8_t kind;
    uint8_t state;
    uint8_t face_mask;
    uint16_t progress_permille;
    uint32_t sample_count;
    uint32_t started_us;
    uint32_t updated_us;
    char message[48];

    float gyro_bias_dps[3];
    float accel_bias_mps2[3];
    float accel_scale[3];
    float mag_offset_ut[3];
    float mag_scale[3];
    float level_roll_deg;   /* measured roll while the vehicle is level */
    float level_pitch_deg;  /* measured pitch while the vehicle is level */
    float baro_p0_pa;
    uint16_t rc_min[SENSOR_CAL_RC_CHANNELS];
    uint16_t rc_max[SENSOR_CAL_RC_CHANNELS];
    uint16_t rc_trim[SENSOR_CAL_RC_CHANNELS];
} SensorCalibrationStatus;

typedef void (*SensorCalibrationComplete)(const SensorCalibrationStatus *status);

void sensor_calibration_service_init(void);
void sensor_calibration_service_set_complete(SensorCalibrationComplete callback);

/* The caller checks vehicle arming state before starting a procedure. */
bool sensor_calibration_start(uint8_t kind, uint64_t now_us,
                              char *error, unsigned error_capacity);

/* Accelerometer six-face workflow: the operator places the vehicle on one
 * face and asks for a capture.  Faces 0..5 are -Z, +Z, +X, -X, +Y, -Y up. */
bool sensor_calibration_capture_accel_face(uint8_t face, uint64_t now_us,
                                           char *error, unsigned error_capacity);

bool sensor_calibration_copy_rc_trims(uint64_t now_us, char *error, unsigned error_capacity);
bool sensor_calibration_finish_rc(uint64_t now_us, char *error, unsigned error_capacity);
void sensor_calibration_cancel(uint64_t now_us);
void sensor_calibration_tick(uint64_t now_us);

void sensor_calibration_feed_imu(const ImuSample *sample, uint64_t now_us);
void sensor_calibration_feed_mag(const float ut[3], uint64_t now_us);
void sensor_calibration_feed_baro(float pressure_pa, uint64_t now_us);
void sensor_calibration_feed_rc(const uint16_t channels[16], uint64_t now_us);

void sensor_calibration_get_status(SensorCalibrationStatus *out);

#ifdef __cplusplus
}
#endif

#endif
