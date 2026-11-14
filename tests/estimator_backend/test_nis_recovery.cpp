/*
 * FLIGHT-29 reproduction: the NIS gate has no recovery path.
 *
 * This is a mechanism test, not a judgement: it records what the production
 * estimator does today so that a fix can be measured against it.
 *
 * Sequence:
 *   1. initialise and settle the filter;
 *   2. feed a GNSS position fix that agrees with the filter -> accepted;
 *   3. feed a fix displaced by 40 m while the filter state stays put. The
 *      innovation is far outside the declared covariance, so the NIS gate
 *      rejects it;
 *   4. repeat the displaced fix many times. Because nothing inflates the
 *      covariance or resets the estimate, every repeat is rejected as well:
 *      gnss_pos_updates never advances again and the position never moves
 *      toward the (correct) fix.
 *
 * The assertions below therefore check the mechanism (gate fires, counter
 * advances for the acceptance, last_nis above the chi-square threshold,
 * no recovery) and print the numbers that a Stage B fix must change.
 */

#include <cmath>
#include <cstdio>
#include <cstring>

#include "backend_adapter/kf_gins_quad_port.h"
#include "test_support.hpp"

namespace {

alignas(16) unsigned char g_storage[16 * 1024];

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6378137.0;
/* chi-square 99.5 % thresholds used by quad_core.cpp */
constexpr double kNisThresholdDof3 = 11.3448667;

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
        init.pos_std_m[i] = 0.5;
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

void settle(int steps) {
    for (int i = 0; i < steps; ++i) {
        kfg_quad_increment_t increment;
        std::memset(&increment, 0, sizeof(increment));
        increment.dt_s = 0.001;
        increment.dvel_mps[2] = -9.80665 * 0.001;
        (void)kfg_quad_propagate(g_storage, &increment);
    }
}

kfg_quad_gnss_position_t make_fix(const kfg_quad_init_state_t &init, double north_m) {
    kfg_quad_gnss_position_t position;
    std::memset(&position, 0, sizeof(position));
    position.blh[0] = init.pos_llh[0] + north_m / kEarthRadiusM * 180.0 / kPi;
    position.blh[1] = init.pos_llh[1];
    position.blh[2] = init.pos_llh[2];
    position.std_m[0] = 0.2;
    position.std_m[1] = 0.2;
    position.std_m[2] = 0.4;
    return position;
}

int north_mm(const kfg_quad_nav_t &nav, const kfg_quad_init_state_t &init) {
    const double dlat = (nav.pos_llh[0] - init.pos_llh[0]) * kPi / 180.0;
    return static_cast<int>(std::lround(dlat * kEarthRadiusM * 1000.0));
}

} // namespace

int main() {
    const kfg_quad_config_t config = make_config();
    const kfg_quad_init_state_t init = make_init();

    CHECK(kfg_quad_init(g_storage, sizeof(g_storage), &config, &init) == KFG_QUAD_OK);
    settle(200);

    /* A fix that agrees with the filter is accepted. */
    kfg_quad_gnss_position_t nominal = make_fix(init, 0.0);
    CHECK(kfg_quad_update_gnss_position(g_storage, &nominal) == KFG_QUAD_OK);

    kfg_quad_diagnostics_t diagnostics;
    CHECK(kfg_quad_get_diagnostics(g_storage, &diagnostics) == KFG_QUAD_OK);
    const unsigned accepted_before = diagnostics.gnss_pos_updates;
    const unsigned rejected_before = diagnostics.nis_rejected;
    CHECK(accepted_before == 1u);
    CHECK(rejected_before == 0u);

    /* The same fix, displaced by 40 m: the filter cannot believe it. */
    kfg_quad_gnss_position_t displaced = make_fix(init, 40.0);
    const int first = kfg_quad_update_gnss_position(g_storage, &displaced);
    CHECK(kfg_quad_get_diagnostics(g_storage, &diagnostics) == KFG_QUAD_OK);
    const double first_nis = diagnostics.last_nis;
    const unsigned rejected_after_first = diagnostics.nis_rejected;

    std::printf("first displaced fix: result=%d nis=%.6f nis_rejected %u -> %u\n",
                first, first_nis, rejected_before,
                rejected_after_first);

    /* The gate fired: the update was refused and the NIS is above the dof-3
     * threshold, so this is a gate decision, not a numerical failure. */
    CHECK(first != KFG_QUAD_OK);
    CHECK(rejected_after_first == rejected_before + 1u);
    CHECK(first_nis > kNisThresholdDof3);

    /* Repeat the correct fix twenty times over twenty seconds. A filter with a
     * recovery path would eventually accept it again; today it never does. */
    int recovered = 0;
    for (int i = 0; i < 20; ++i) {
        settle(1000);
        if (kfg_quad_update_gnss_position(g_storage, &displaced) == KFG_QUAD_OK)
            ++recovered;
    }
    CHECK(kfg_quad_get_diagnostics(g_storage, &diagnostics) == KFG_QUAD_OK);
    kfg_quad_nav_t nav;
    CHECK(kfg_quad_get_nav(g_storage, &nav) == KFG_QUAD_OK);

    std::printf("after 20 displaced fixes: accepted=%u rejected=%u recovered=%d "
                "north_error=%d mm last_nis=%.6f\n",
                diagnostics.gnss_pos_updates, diagnostics.nis_rejected, recovered,
                north_mm(nav, init), diagnostics.last_nis);

    /* Mechanism assertions, i.e. today's behaviour. When Stage B adds a bounded
     * recovery path, `recovered` and `gnss_pos_updates` are the lines that must
     * change; the gate counters below must not. */
    CHECK(diagnostics.gnss_pos_updates == accepted_before);
    CHECK(diagnostics.nis_rejected == rejected_before + 21u);
    CHECK(recovered == 0);
    /* Rejecting a 40 m outlier is correct. What is missing is any way out of
     * the state where every future fix is also rejected: last_nis remains far
     * above the 11.34 threshold after 20 s of covariance growth. */
    CHECK(std::abs(north_mm(nav, init)) < 2000);
    CHECK(diagnostics.last_nis > kNisThresholdDof3);

    return hosttest::summary("test_nis_recovery");
}
