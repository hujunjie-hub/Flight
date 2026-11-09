#ifndef SENSOR_SAMPLE_H
#define SENSOR_SAMPLE_H

#include "base_types/common_base.h"
#include "sensor_reading.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Identity fields the platform owns but every public sample must carry. */
typedef struct {
    uint64_t boot_id;
    uint32_t time_epoch;
    uint32_t profile_id;
    uint32_t sensor_instance_id;
    uint32_t sequence; /* incremented by the module for every emitted sample */
} sensor_context_t;

/* IMU encoding configuration, calibration and timing. */
typedef struct {
    bool burst32;
    double gyro_lsb_per_dps;
    double accel_lsb_per_mps2;
    double temp_lsb_per_c;

    /* Calibration in the sensor frame, applied before the installation
     * rotation: corrected = misalignment * ((raw - bias) * scale).
     * scale = 1 and misalignment = identity mean "no correction". */
    float gyro_bias_dps[3];
    float gyro_scale[3];
    float gyro_misalignment[3][3];
    float accel_bias_mps2[3];
    float accel_scale[3];
    float accel_misalignment[3][3];

    /* Installation rotation: sensor frame -> body FRD (orthonormal). */
    float install_rotation[3][3];

    /* Calibrated physical sample-time correction (spec 05 section 3.1). */
    double pipeline_delay_us;
    double pipeline_delay_uncertainty_us;

    uint32_t calibration_version;
} sensor_imu_config_t;

sensor_status_t sensor_imu_build_sample(const sensor_imu_config_t *config, sensor_context_t *context,
                                        const ImuReading *reading, uint64_t capture_us,
                                        uint64_t publish_us, ImuSample *out);

typedef struct {
    double p0_pa;       /* pressure reference, 101325 Pa default */
    double variance_m2; /* configured sample variance floor */
    uint32_t calibration_version;
} sensor_baro_config_t;

sensor_status_t sensor_baro_build_sample(const sensor_baro_config_t *config, sensor_context_t *context,
                                         const BarometerReading *reading, uint64_t sample_time_us,
                                         uint64_t publish_us, BaroSample *out);

/* Pressure [Pa] -> pressure altitude [m], positive up. */
double sensor_baro_pressure_to_altitude(double pressure_pa, double p0_pa);

/* ------------------------------------------------------------------------- */
/* GNSS PVT                                                                  */
/* ------------------------------------------------------------------------- */

#define SENSOR_GNSS_PVT_POSITION_VALID (1u << 0)
#define SENSOR_GNSS_PVT_VELOCITY_VALID (1u << 1)
#define SENSOR_GNSS_PVT_POSITION_COV_VALID (1u << 2)
#define SENSOR_GNSS_PVT_VELOCITY_COV_VALID (1u << 3)

typedef struct {
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
    float velocity_ned_mps[3];
    float position_variance_m2[3];
    float velocity_variance_m2ps2[3];
    uint8_t fix_type; /* GNSS_FIX_* */
    uint8_t satellites;
    uint32_t valid_flags; /* SENSOR_GNSS_PVT_* */
} sensor_gnss_pvt_t;

typedef struct {
    double max_speed_mps;   /* reject a position step faster than this */
    double max_altitude_m;  /* physical sanity bound */
    double max_variance_m2; /* above this the covariance bit is cleared */
    uint64_t stale_age_us;  /* arrival - sample time above this is stale */
} sensor_gnss_config_t;

typedef struct {
    bool has_previous;
    double latitude_deg;
    double longitude_deg;
    uint64_t sample_time_us;
} sensor_gnss_state_t;

/* 'sample_time_us' is the measurement epoch already mapped to FCU monotonic;
 * 'arrival_us' is the UART/parse completion time used only for the age check. */
sensor_status_t sensor_gnss_build_sample(const sensor_gnss_config_t *config, sensor_context_t *context,
                                 sensor_gnss_state_t *state, const sensor_gnss_pvt_t *pvt,
                                 uint64_t sample_time_us, uint64_t arrival_us, uint32_t mapping_epoch,
                                 GnssSample *out);

void sensor_gnss_reset(sensor_gnss_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAMPLE_H */
