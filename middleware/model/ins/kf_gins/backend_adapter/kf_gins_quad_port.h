/*
 * KF-GINS_QUAD plugin-private C ABI.
 *
 * Scope note: spec 05 §6.1 plans a `kf_gins_port.c` exporting the public
 * EstimatorPort.  The public EstimatorPort header now exists in
 * middleware/contracts/ (Schema 6.0 / Port 0x00010903, task T191-A-ABI), but
 * binding it needs the M2 sample semantics, observation queue and snapshot
 * marshalling owned by T191-B-ADAPTER / T191-B-REPLAY.  This file therefore
 * exports a *plugin-private* C ABI with opaque, caller provided storage and
 * defines no public ICD type;
 * hosttest/test_contract_coexistence.cpp proves the two header sets compile
 * together without conflict.
 *
 * No heap: the caller supplies statically allocated storage whose size and
 * alignment are reported by kfg_quad_sizeof()/kfg_quad_alignof().
 */

#ifndef KFGINS_QUAD_PORT_H
#define KFGINS_QUAD_PORT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KFG_QUAD_ABI_VERSION 0x00010000u

/* Mirrored status codes; they are stable inside the plugin only. */
enum {
    KFG_QUAD_OK = 0,
    KFG_QUAD_ERR_ARGUMENT = 1,
    KFG_QUAD_ERR_NOT_INITIALIZED = 2,
    KFG_QUAD_ERR_NON_FINITE = 3,
    KFG_QUAD_ERR_NOT_POSITIVE_DEFINITE = 4,
    KFG_QUAD_ERR_NIS_REJECTED = 5,
    KFG_QUAD_ERR_TIME_DISCONTINUITY = 6,
    KFG_QUAD_ERR_COVARIANCE_DEGRADED = 7
};

typedef struct kfg_quad_config {
    double gyro_arw[3];      /* rad/s/sqrt(Hz) */
    double accel_vrw[3];     /* m/s^2/sqrt(Hz) */
    double gyro_bias_std[3]; /* rad/s */
    double accel_bias_std[3];/* m/s^2 */
    double gyro_scale_std[3];
    double accel_scale_std[3];
    double corr_time_s;      /* first-order Gauss-Markov correlation time */
    double baro_bias_walk;   /* m^2/s process noise density of b_h */
    double antlever_body_m[3]; /* IMU centre -> GNSS antenna, body FRD */
    int32_t nis_gate_enable;
    double baro_origin_height_m;
} kfg_quad_config_t;

typedef struct kfg_quad_init_state {
    double pos_llh[3];  /* rad, rad, m */
    double vel_ned[3];  /* m/s */
    double euler_rpy[3];/* rad */
    double gyro_bias[3];
    double accel_bias[3];
    double gyro_scale[3];
    double accel_scale[3];
    double baro_bias_m;

    double pos_std_m[3];
    double vel_std_mps[3];
    double euler_std_rad[3];
    double gyro_bias_std[3];
    double accel_bias_std[3];
    double gyro_scale_std[3];
    double accel_scale_std[3];
    double baro_bias_std_m;
} kfg_quad_init_state_t;

typedef struct kfg_quad_increment {
    double dt_s;
    double dtheta_rad[3];
    double dvel_mps[3];
} kfg_quad_increment_t;

typedef struct kfg_quad_gnss_position {
    double blh[3];   /* antenna phase centre, rad, rad, m */
    double std_m[3]; /* local NED */
} kfg_quad_gnss_position_t;

typedef struct kfg_quad_gnss_velocity {
    double vel_ned[3];
    double std_mps[3];
    double omega_body_radps[3]; /* raw body rate at the measurement epoch */
} kfg_quad_gnss_velocity_t;

typedef struct kfg_quad_baro {
    double height_m; /* h_baro - h_baro0 */
    double std_m;
} kfg_quad_baro_t;

typedef struct kfg_quad_nav {
    double pos_llh[3];
    double vel_ned[3];
    double euler_rpy[3];
    double quat_wxyz[4];
    double gyro_bias[3];
    double accel_bias[3];
    double gyro_scale[3];
    double accel_scale[3];
    double baro_bias_m;
} kfg_quad_nav_t;

typedef struct kfg_quad_diagnostics {
    uint32_t propagate_count;
    uint32_t gnss_pos_updates;
    uint32_t gnss_vel_updates;
    uint32_t baro_updates;
    uint32_t nis_rejected;
    uint32_t nonfinite_events;
    uint32_t spd_failures;
    double last_nis;
} kfg_quad_diagnostics_t;

/* Storage requirements (no heap, no dynamic allocation). */
size_t kfg_quad_sizeof(void);
size_t kfg_quad_alignof(void);
uint32_t kfg_quad_abi_version(void);

/*
 * Constructs the estimator in 'storage'.  Returns KFG_QUAD_OK on success.
 * The storage must be aligned to kfg_quad_alignof() and at least
 * kfg_quad_sizeof() bytes; it is owned by the caller for the object lifetime.
 */
int kfg_quad_init(void *storage, size_t storage_bytes, const kfg_quad_config_t *config,
                  const kfg_quad_init_state_t *init);

int kfg_quad_propagate(void *handle, const kfg_quad_increment_t *increment);
int kfg_quad_update_gnss_position(void *handle, const kfg_quad_gnss_position_t *observation);
int kfg_quad_update_gnss_velocity(void *handle, const kfg_quad_gnss_velocity_t *observation);
int kfg_quad_update_baro(void *handle, const kfg_quad_baro_t *observation);

int kfg_quad_get_nav(const void *handle, kfg_quad_nav_t *nav);
/* Copies the full 22x22 covariance in row-major order into 'out' (484 doubles). */
int kfg_quad_copy_covariance(const void *handle, double *out);
int kfg_quad_get_diagnostics(const void *handle, kfg_quad_diagnostics_t *diagnostics);

#ifdef __cplusplus
}
#endif

#endif /* KFGINS_QUAD_PORT_H */
