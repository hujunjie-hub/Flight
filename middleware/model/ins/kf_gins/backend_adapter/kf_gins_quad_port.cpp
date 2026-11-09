/*
 * KF-GINS_QUAD plugin-private C ABI implementation.
 *
 * Thin marshalling only: unit/frame conversion between the C structs and the
 * fixed-dimension core, status mapping and lifetime checks.  All numerical
 * work stays in trim/.
 */

#include "backend_adapter/kf_gins_quad_port.h"

#include <cmath>
#include <cstring>
#include <new>

#include "kfgins/quad_core.hpp"
#include "kfgins/quad_types.hpp"

namespace {

constexpr std::uint32_t kHandleMagic = 0x4B464751u; /* "KFGQ" */

struct Handle {
    std::uint32_t magic;
    std::uint32_t abi;
    kfg::QuadCore core;
};

kfg::Vec3 to_vec3(const double *values) {
    return kfg::vec3(values[0], values[1], values[2]);
}

void from_vec3(const kfg::Vec3 &value, double *out) {
    out[0] = value(0, 0);
    out[1] = value(1, 0);
    out[2] = value(2, 0);
}

int to_status(kfg::Status status) {
    switch (status) {
    case kfg::kOk:
        return KFG_QUAD_OK;
    case kfg::kErrArgument:
        return KFG_QUAD_ERR_ARGUMENT;
    case kfg::kErrNotInitialized:
        return KFG_QUAD_ERR_NOT_INITIALIZED;
    case kfg::kErrNonFinite:
        return KFG_QUAD_ERR_NON_FINITE;
    case kfg::kErrNotPositiveDefinite:
        return KFG_QUAD_ERR_NOT_POSITIVE_DEFINITE;
    case kfg::kErrNisRejected:
        return KFG_QUAD_ERR_NIS_REJECTED;
    case kfg::kErrTimeDiscontinuity:
        return KFG_QUAD_ERR_TIME_DISCONTINUITY;
    case kfg::kErrCovarianceDegraded:
        return KFG_QUAD_ERR_COVARIANCE_DEGRADED;
    }
    return KFG_QUAD_ERR_ARGUMENT;
}

Handle *to_handle(void *storage) {
    if (storage == nullptr) {
        return nullptr;
    }
    Handle *handle = static_cast<Handle *>(storage);
    if (handle->magic != kHandleMagic || handle->abi != KFG_QUAD_ABI_VERSION) {
        return nullptr;
    }
    return handle;
}

const Handle *to_handle(const void *storage) {
    return to_handle(const_cast<void *>(storage));
}

} // namespace

extern "C" {

size_t kfg_quad_sizeof(void) { return sizeof(Handle); }

size_t kfg_quad_alignof(void) { return alignof(Handle); }

uint32_t kfg_quad_abi_version(void) { return KFG_QUAD_ABI_VERSION; }

int kfg_quad_init(void *storage, size_t storage_bytes, const kfg_quad_config_t *config,
                  const kfg_quad_init_state_t *init) {
    if (storage == nullptr || config == nullptr || init == nullptr) {
        return KFG_QUAD_ERR_ARGUMENT;
    }
    if (storage_bytes < sizeof(Handle)) {
        return KFG_QUAD_ERR_ARGUMENT;
    }
    if ((reinterpret_cast<std::uintptr_t>(storage) % alignof(Handle)) != 0u) {
        return KFG_QUAD_ERR_ARGUMENT;
    }

    kfg::QuadConfig core_config;
    core_config.imunoise.gyr_arw = to_vec3(config->gyro_arw);
    core_config.imunoise.acc_vrw = to_vec3(config->accel_vrw);
    core_config.imunoise.gyrbias_std = to_vec3(config->gyro_bias_std);
    core_config.imunoise.accbias_std = to_vec3(config->accel_bias_std);
    core_config.imunoise.gyrscale_std = to_vec3(config->gyro_scale_std);
    core_config.imunoise.accscale_std = to_vec3(config->accel_scale_std);
    core_config.imunoise.corr_time = config->corr_time_s;
    core_config.imunoise.baro_bias_walk = config->baro_bias_walk;
    core_config.antlever = to_vec3(config->antlever_body_m);
    core_config.nis_gate_enable = (config->nis_gate_enable != 0);
    core_config.baro_origin_height_m = config->baro_origin_height_m;

    kfg::QuadInitState core_init;
    core_init.nominal.pos = to_vec3(init->pos_llh);
    core_init.nominal.vel = to_vec3(init->vel_ned);
    core_init.nominal.euler = to_vec3(init->euler_rpy);
    core_init.nominal.cbn = kfg::eulerToMatrix(core_init.nominal.euler);
    core_init.nominal.qbn = kfg::matrixToQuat(core_init.nominal.cbn);
    core_init.nominal.gyrbias = to_vec3(init->gyro_bias);
    core_init.nominal.accbias = to_vec3(init->accel_bias);
    core_init.nominal.gyrscale = to_vec3(init->gyro_scale);
    core_init.nominal.accscale = to_vec3(init->accel_scale);
    core_init.nominal.baro_bias = init->baro_bias_m;

    core_init.pos_std = to_vec3(init->pos_std_m);
    core_init.vel_std = to_vec3(init->vel_std_mps);
    core_init.euler_std = to_vec3(init->euler_std_rad);
    core_init.gyrbias_std = to_vec3(init->gyro_bias_std);
    core_init.accbias_std = to_vec3(init->accel_bias_std);
    core_init.gyrscale_std = to_vec3(init->gyro_scale_std);
    core_init.accscale_std = to_vec3(init->accel_scale_std);
    core_init.baro_bias_std = init->baro_bias_std_m;

    Handle *handle = new (storage) Handle();
    handle->magic = kHandleMagic;
    handle->abi = KFG_QUAD_ABI_VERSION;

    const kfg::Status status = handle->core.initialize(core_config, core_init);
    if (status != kfg::kOk) {
        handle->magic = 0u;
        return to_status(status);
    }
    return KFG_QUAD_OK;
}

int kfg_quad_propagate(void *storage, const kfg_quad_increment_t *increment) {
    Handle *handle = to_handle(storage);
    if (handle == nullptr || increment == nullptr) {
        return KFG_QUAD_ERR_ARGUMENT;
    }
    kfg::ImuIncrement core_increment;
    core_increment.dt = increment->dt_s;
    core_increment.dtheta = to_vec3(increment->dtheta_rad);
    core_increment.dvel = to_vec3(increment->dvel_mps);
    return to_status(handle->core.propagate(core_increment));
}

int kfg_quad_update_gnss_position(void *storage, const kfg_quad_gnss_position_t *observation) {
    Handle *handle = to_handle(storage);
    if (handle == nullptr || observation == nullptr) {
        return KFG_QUAD_ERR_ARGUMENT;
    }
    kfg::GnssPositionObs core_observation;
    core_observation.blh = to_vec3(observation->blh);
    core_observation.std = to_vec3(observation->std_m);
    return to_status(handle->core.updateGnssPosition(core_observation));
}

int kfg_quad_update_gnss_velocity(void *storage, const kfg_quad_gnss_velocity_t *observation) {
    Handle *handle = to_handle(storage);
    if (handle == nullptr || observation == nullptr) {
        return KFG_QUAD_ERR_ARGUMENT;
    }
    kfg::GnssVelocityObs core_observation;
    core_observation.vel_ned = to_vec3(observation->vel_ned);
    core_observation.std = to_vec3(observation->std_mps);
    core_observation.omega_ib = to_vec3(observation->omega_body_radps);
    return to_status(handle->core.updateGnssVelocity(core_observation));
}

int kfg_quad_update_baro(void *storage, const kfg_quad_baro_t *observation) {
    Handle *handle = to_handle(storage);
    if (handle == nullptr || observation == nullptr) {
        return KFG_QUAD_ERR_ARGUMENT;
    }
    kfg::BaroObs core_observation;
    core_observation.height = observation->height_m;
    core_observation.std = observation->std_m;
    return to_status(handle->core.updateBaro(core_observation));
}

int kfg_quad_get_nav(const void *storage, kfg_quad_nav_t *nav) {
    const Handle *handle = to_handle(storage);
    if (handle == nullptr || nav == nullptr) {
        return KFG_QUAD_ERR_ARGUMENT;
    }
    const kfg::QuadNominal &nominal = handle->core.nominal();
    from_vec3(nominal.pos, nav->pos_llh);
    from_vec3(nominal.vel, nav->vel_ned);
    from_vec3(nominal.euler, nav->euler_rpy);
    nav->quat_wxyz[0] = nominal.qbn.w;
    nav->quat_wxyz[1] = nominal.qbn.x;
    nav->quat_wxyz[2] = nominal.qbn.y;
    nav->quat_wxyz[3] = nominal.qbn.z;
    from_vec3(nominal.gyrbias, nav->gyro_bias);
    from_vec3(nominal.accbias, nav->accel_bias);
    from_vec3(nominal.gyrscale, nav->gyro_scale);
    from_vec3(nominal.accscale, nav->accel_scale);
    nav->baro_bias_m = nominal.baro_bias;
    return KFG_QUAD_OK;
}

int kfg_quad_copy_covariance(const void *storage, double *out) {
    const Handle *handle = to_handle(storage);
    if (handle == nullptr || out == nullptr) {
        return KFG_QUAD_ERR_ARGUMENT;
    }
    handle->core.copyCovariance(out);
    return KFG_QUAD_OK;
}

int kfg_quad_get_diagnostics(const void *storage, kfg_quad_diagnostics_t *diagnostics) {
    const Handle *handle = to_handle(storage);
    if (handle == nullptr || diagnostics == nullptr) {
        return KFG_QUAD_ERR_ARGUMENT;
    }
    const kfg::QuadDiagnostics &source = handle->core.diagnostics();
    diagnostics->propagate_count = source.propagate_count;
    diagnostics->gnss_pos_updates = source.gnss_pos_updates;
    diagnostics->gnss_vel_updates = source.gnss_vel_updates;
    diagnostics->baro_updates = source.baro_updates;
    diagnostics->nis_rejected = source.nis_rejected;
    diagnostics->nonfinite_events = source.nonfinite_events;
    diagnostics->spd_failures = source.spd_failures;
    diagnostics->last_nis = source.last_nis;
    return KFG_QUAD_OK;
}

} // extern "C"
