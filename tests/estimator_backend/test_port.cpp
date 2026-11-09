/*
 * KF-GINS_QUAD plugin C ABI and IMU adapter test.
 *
 * Verifies the statically allocated handle, the no-heap contract, the C ABI
 * lifetime checks and the increment builder rules of spec 05 §3.1 on the Host.
 */

#include <cstdio>
#include <cstring>

#include "backend_adapter/kf_gins_adapter.hpp"
#include "backend_adapter/kf_gins_quad_port.h"
#include "test_support.hpp"

namespace {

kfg_quad_config_t make_config() {
    kfg_quad_config_t config;
    std::memset(&config, 0, sizeof(config));
    for (int i = 0; i < 3; ++i) {
        config.gyro_arw[i] = 0.0015;
        config.accel_vrw[i] = 0.025;
        config.gyro_bias_std[i] = 5.0e-4;
        config.accel_bias_std[i] = 0.02;
        config.gyro_scale_std[i] = 1.0e-4;
        config.accel_scale_std[i] = 1.0e-4;
    }
    config.corr_time_s = 3600.0;
    config.baro_bias_walk = 1.0e-4;
    config.antlever_body_m[0] = 0.12;
    config.antlever_body_m[1] = -0.05;
    config.antlever_body_m[2] = 0.21;
    config.nis_gate_enable = 1;
    config.baro_origin_height_m = 30.0;
    return config;
}

kfg_quad_init_state_t make_init() {
    kfg_quad_init_state_t init;
    std::memset(&init, 0, sizeof(init));
    init.pos_llh[0] = 0.532325;
    init.pos_llh[1] = 1.994911;
    init.pos_llh[2] = 35.0;
    init.euler_rpy[0] = 0.01;
    init.euler_rpy[1] = -0.02;
    init.euler_rpy[2] = 0.4;
    init.gyro_bias[0] = 1.0e-4;
    init.accel_bias[1] = -0.02;
    for (int i = 0; i < 3; ++i) {
        init.pos_std_m[i] = 1.0;
        init.vel_std_mps[i] = 0.1;
        init.euler_std_rad[i] = 0.02;
        init.gyro_bias_std[i] = 5.0e-4;
        init.accel_bias_std[i] = 0.02;
        init.gyro_scale_std[i] = 1.0e-4;
        init.accel_scale_std[i] = 1.0e-4;
    }
    init.baro_bias_std_m = 1.5;
    return init;
}

/* Statically sized storage; the 14 KiB DTCM active arena of spec 07 §6 is the
 * target budget and is asserted below. */
alignas(16) unsigned char g_storage[16 * 1024];

void test_storage_requirements() {
    const size_t required = kfg_quad_sizeof();
    std::printf("kfg_quad_sizeof  = %zu bytes\n", required);
    std::printf("kfg_quad_alignof = %zu bytes\n", kfg_quad_alignof());
    CHECK(kfg_quad_alignof() <= 16u);
    CHECK(required <= 14u * 1024u);
    CHECK(kfg_quad_abi_version() == KFG_QUAD_ABI_VERSION);
}

void test_lifecycle_and_null_checks() {
    const kfg_quad_config_t config = make_config();
    const kfg_quad_init_state_t init = make_init();

    CHECK(kfg_quad_init(nullptr, sizeof(g_storage), &config, &init) == KFG_QUAD_ERR_ARGUMENT);
    CHECK(kfg_quad_init(g_storage, 8, &config, &init) == KFG_QUAD_ERR_ARGUMENT);
    CHECK(kfg_quad_init(g_storage, sizeof(g_storage), nullptr, &init) == KFG_QUAD_ERR_ARGUMENT);

    CHECK(kfg_quad_init(g_storage, sizeof(g_storage), &config, &init) == KFG_QUAD_OK);

    kfg_quad_nav_t nav;
    CHECK(kfg_quad_get_nav(g_storage, &nav) == KFG_QUAD_OK);
    CHECK_NEAR(nav.pos_llh[2], 35.0, 1.0e-12);

    /* An uninitialised or corrupted handle must be rejected, not executed. */
    alignas(16) unsigned char untouched[16 * 1024];
    std::memset(untouched, 0, sizeof(untouched));
    CHECK(kfg_quad_get_nav(untouched, &nav) == KFG_QUAD_ERR_ARGUMENT);

    kfg_quad_increment_t increment;
    std::memset(&increment, 0, sizeof(increment));
    increment.dt_s = 0.001;
    increment.dvel_mps[2] = -0.0098;
    CHECK(kfg_quad_propagate(untouched, &increment) == KFG_QUAD_ERR_ARGUMENT);
    CHECK(kfg_quad_propagate(g_storage, &increment) == KFG_QUAD_OK);

    kfg_quad_diagnostics_t diagnostics;
    CHECK(kfg_quad_get_diagnostics(g_storage, &diagnostics) == KFG_QUAD_OK);
    CHECK(diagnostics.propagate_count == 1u);
}

void test_observations_through_abi() {
    const kfg_quad_config_t config = make_config();
    const kfg_quad_init_state_t init = make_init();
    CHECK(kfg_quad_init(g_storage, sizeof(g_storage), &config, &init) == KFG_QUAD_OK);

    for (int i = 0; i < 100; ++i) {
        kfg_quad_increment_t increment;
        std::memset(&increment, 0, sizeof(increment));
        increment.dt_s = 0.001;
        increment.dvel_mps[2] = -9.80665 * 0.001;
        CHECK(kfg_quad_propagate(g_storage, &increment) == KFG_QUAD_OK);
    }

    kfg_quad_baro_t baro;
    baro.height_m = 2.0; /* relative pressure height above the origin */
    baro.std_m = 0.3;
    CHECK(kfg_quad_update_baro(g_storage, &baro) == KFG_QUAD_OK);

    kfg_quad_gnss_position_t position;
    position.blh[0] = init.pos_llh[0];
    position.blh[1] = init.pos_llh[1];
    position.blh[2] = init.pos_llh[2];
    position.std_m[0] = 0.5;
    position.std_m[1] = 0.5;
    position.std_m[2] = 1.0;
    CHECK(kfg_quad_update_gnss_position(g_storage, &position) == KFG_QUAD_OK);

    kfg_quad_gnss_velocity_t velocity;
    std::memset(&velocity, 0, sizeof(velocity));
    velocity.std_mps[0] = 0.1;
    velocity.std_mps[1] = 0.1;
    velocity.std_mps[2] = 0.1;
    CHECK(kfg_quad_update_gnss_velocity(g_storage, &velocity) == KFG_QUAD_OK);

    double covariance[22 * 22];
    CHECK(kfg_quad_copy_covariance(g_storage, covariance) == KFG_QUAD_OK);
    for (int i = 0; i < 22; ++i) {
        CHECK(covariance[i * 22 + i] > 0.0);
        CHECK(std::isfinite(covariance[i * 22 + i]));
    }

    kfg_quad_diagnostics_t diagnostics;
    CHECK(kfg_quad_get_diagnostics(g_storage, &diagnostics) == KFG_QUAD_OK);
    CHECK(diagnostics.gnss_pos_updates == 1u);
    CHECK(diagnostics.gnss_vel_updates == 1u);
    CHECK(diagnostics.baro_updates == 1u);
}

void test_increment_builder() {
    kfg::ImuIncrementBuilder builder;
    kfg::ImuIncrement increment;

    kfg::ImuSample sample;
    sample.gyro_radps = kfg::vec3(0.0, 0.0, 0.0);
    sample.accel_mps2 = kfg::vec3(0.0, 0.0, -9.80665);
    sample.sample_time_us = 1000;
    sample.sequence = 1;

    CHECK(builder.push(sample, increment) == kfg::kAdapterSeed);
    CHECK(!builder.has_seed() ? false : true);

    /* Second sample 1 ms later: a valid increment. */
    sample.sample_time_us = 2000;
    sample.sequence = 2;
    CHECK(builder.push(sample, increment) == kfg::kAdapterIncrement);
    CHECK_NEAR(increment.dt, 0.001, 1.0e-15);
    CHECK_NEAR(increment.dvel(2, 0), -9.80665 * 0.001, 1.0e-15);

    /* Duplicate sequence number. */
    CHECK(builder.push(sample, increment) == kfg::kAdapterSequenceError);
    CHECK(builder.sequence_error_count() == 1u);

    /* Too-long gap. */
    kfg::ImuSample gap = sample;
    gap.sample_time_us = 2000 + 2000;
    gap.sequence = 3;
    CHECK(builder.push(gap, increment) == kfg::kAdapterGap);
    CHECK(builder.gap_count() == 1u);

    /* The rejected gap sample did not move the seed, so a 1 ms step from the
       last accepted sample is still continuous. */
    kfg::ImuSample uniform = sample;
    uniform.sample_time_us = 3000;
    uniform.sequence = 4;
    CHECK(builder.push(uniform, increment) == kfg::kAdapterIncrement);

    /* Non-uniform sampling: +5 % relative to the previous interval. */
    kfg::ImuSample jitter = sample;
    jitter.sample_time_us = 3000 + 1050;
    jitter.sequence = 5;
    CHECK(builder.push(jitter, increment) == kfg::kAdapterGap);

    /* Non-finite sample. */
    kfg::ImuSample bad = sample;
    bad.sample_time_us = 4050;
    bad.sequence = 6;
    bad.accel_mps2(1, 0) = NAN;
    CHECK(builder.push(bad, increment) == kfg::kAdapterBadSample);
}

} // namespace

int main() {
    std::printf("KF-GINS_QUAD plugin C ABI / adapter test\n");
    test_storage_requirements();
    test_lifecycle_and_null_checks();
    test_observations_through_abi();
    test_increment_builder();
    return hosttest::summary("test_port");
}
