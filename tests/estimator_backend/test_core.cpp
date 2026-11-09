/*
 * KF-GINS_QUAD core unit tests.
 *
 * Covers the 22-state extension invariants, the numeric failure containment
 * required by spec 05 §5.4/§6.1.1 and the NIS gate, all on the Host build.
 */

#include <cstdio>

#include "kfgins/quad_core.hpp"
#include "kfgins/quad_models.hpp"
#include "test_support.hpp"

using namespace kfg;

namespace {

QuadConfig make_config() {
    QuadConfig cfg;
    cfg.imunoise.gyr_arw = vec3(0.0015, 0.0015, 0.0015);
    cfg.imunoise.acc_vrw = vec3(0.025, 0.025, 0.025);
    cfg.imunoise.gyrbias_std = vec3(0.0005, 0.0005, 0.0005);
    cfg.imunoise.accbias_std = vec3(0.01, 0.01, 0.01);
    cfg.imunoise.gyrscale_std = vec3(1.0e-4, 1.0e-4, 1.0e-4);
    cfg.imunoise.accscale_std = vec3(1.0e-4, 1.0e-4, 1.0e-4);
    cfg.imunoise.corr_time = 3600.0;
    cfg.imunoise.baro_bias_walk = 1.0e-4;
    cfg.antlever = vec3(0.12, -0.05, 0.21);
    cfg.nis_gate_enable = true;
    cfg.baro_origin_height_m = 30.0;
    return cfg;
}

QuadInitState make_init() {
    QuadInitState init;
    init.nominal.pos = vec3(30.5 * kD2R, 114.3 * kD2R, 35.0);
    init.nominal.vel = vec3(0.0, 0.0, 0.0);
    init.nominal.euler = vec3(0.01, -0.02, 0.4);
    init.nominal.cbn = eulerToMatrix(init.nominal.euler);
    init.nominal.qbn = matrixToQuat(init.nominal.cbn);
    init.nominal.gyrbias = vec3(1.0e-4, -2.0e-4, 3.0e-4);
    init.nominal.accbias = vec3(0.01, -0.02, 0.03);
    init.nominal.gyrscale = vec3(1.0e-5, 0.0, -1.0e-5);
    init.nominal.accscale = vec3(0.0, 2.0e-5, 0.0);
    init.nominal.baro_bias = 0.0;

    init.pos_std = vec3(1.0, 1.0, 1.5);
    init.vel_std = vec3(0.1, 0.1, 0.1);
    init.euler_std = vec3(0.02, 0.02, 0.1);
    init.gyrbias_std = vec3(5.0e-4, 5.0e-4, 5.0e-4);
    init.accbias_std = vec3(0.02, 0.02, 0.02);
    init.gyrscale_std = vec3(1.0e-4, 1.0e-4, 1.0e-4);
    init.accscale_std = vec3(1.0e-4, 1.0e-4, 1.0e-4);
    init.baro_bias_std = 1.5;
    return init;
}

/*
 * Raw IMU increment of a vehicle at rest whose sensor carries the bias/scale
 * that the filter currently estimates, so that the filter's own compensation
 * recovers the true specific force f_b = C_bn^T * (0, 0, -gamma).
 */
ImuIncrement static_increment(const QuadNominal &nominal, double dt) {
    const Vec3 f_nav = vec3(0.0, 0.0, -gravity(nominal.pos));
    const Vec3 f_body = matvecTransposed(nominal.cbn, f_nav);

    ImuIncrement increment;
    increment.dt = dt;
    for (int i = 0; i < 3; ++i) {
        increment.dtheta(i, 0) = nominal.gyrbias(i, 0) * (1.0 + nominal.gyrscale(i, 0)) * dt;
        increment.dvel(i, 0) = (f_body(i, 0) + nominal.accbias(i, 0)) * (1.0 + nominal.accscale(i, 0)) * dt;
    }
    return increment;
}

void test_initialization_validation() {
    QuadCore core;
    QuadConfig cfg = make_config();
    QuadInitState init = make_init();

    CHECK(core.initialize(cfg, init) == kOk);
    CHECK(core.initialized());
    CHECK_NEAR(core.covariance()(kIdxP, kIdxP), 1.0, 1.0e-12);
    CHECK_NEAR(core.covariance()(kIdxBh, kIdxBh), 2.25, 1.0e-12);

    QuadConfig bad_corr = cfg;
    bad_corr.imunoise.corr_time = 0.0;
    CHECK(core.initialize(bad_corr, init) == kErrArgument);

    QuadConfig bad_lever = cfg;
    bad_lever.antlever = vec3(NAN, 0.0, 0.0);
    CHECK(core.initialize(bad_lever, init) == kErrArgument);

    QuadInitState bad_std = init;
    bad_std.baro_bias_std = 0.0;
    CHECK(core.initialize(cfg, bad_std) == kErrArgument);
    CHECK_NEAR(core.covariance()(kIdxP, kIdxP), 1.0, 1.0e-12);
}

void test_propagation_invariants() {
    QuadCore core;
    const QuadConfig cfg = make_config();
    const QuadInitState init = make_init();
    CHECK(core.initialize(cfg, init) == kOk);

    for (int i = 0; i < 1000; ++i) {
        CHECK(core.propagate(static_increment(core.nominal(), 0.001)) == kOk);
    }

    CHECK(core.diagnostics().propagate_count == 1000u);

    /* At rest the specific force cancels gravity: velocity and height stay. */
    CHECK_NEAR(core.nominal().vel(0, 0), 0.0, 1.0e-6);
    CHECK_NEAR(core.nominal().vel(1, 0), 0.0, 1.0e-6);
    CHECK_NEAR(core.nominal().vel(2, 0), 0.0, 1.0e-6);
    CHECK_NEAR(core.nominal().pos(2, 0), 35.0, 1.0e-6);

    /* 22nd state stays decoupled from the 21 core states while no barometer
       update has been applied. */
    for (int j = 0; j < kIdxBh; ++j) {
        CHECK_NEAR(core.covariance()(kIdxBh, j), 0.0, 1.0e-18);
        CHECK_NEAR(core.covariance()(j, kIdxBh), 0.0, 1.0e-18);
    }
    /* Random walk only: P(21,21) = sigma^2 + q_bh * t. */
    CHECK_NEAR(core.covariance()(kIdxBh, kIdxBh), 2.25 + 1.0e-4 * 1.0, 1.0e-9);

    /* Covariance remains symmetric and positive definite. */
    for (int r = 0; r < kRank22; ++r) {
        for (int c = r + 1; c < kRank22; ++c) {
            CHECK_NEAR(core.covariance()(r, c), core.covariance()(c, r), 1.0e-15);
        }
        CHECK(core.covariance()(r, r) > 0.0);
    }
    CHECK(isPositiveDefinite(core.covariance()));
}

void test_dt_guard() {
    QuadCore core;
    const QuadConfig cfg = make_config();
    const QuadInitState init = make_init();
    CHECK(core.initialize(cfg, init) == kOk);

    ImuIncrement zero_dt = static_increment(core.nominal(), 0.001);
    zero_dt.dt = 0.0;
    CHECK(core.propagate(zero_dt) == kErrArgument);

    ImuIncrement negative_dt = static_increment(core.nominal(), 0.001);
    negative_dt.dt = -0.001;
    CHECK(core.propagate(negative_dt) == kErrArgument);

    ImuIncrement nan_increment = static_increment(core.nominal(), 0.001);
    nan_increment.dvel(0, 0) = NAN;
    CHECK(core.propagate(nan_increment) == kErrArgument);
    CHECK(core.diagnostics().propagate_count == 0u);
}

void test_baro_update() {
    QuadCore core;
    QuadConfig cfg = make_config();
    const QuadInitState init = make_init();
    CHECK(core.initialize(cfg, init) == kOk);

    for (int i = 0; i < 100; ++i) {
        CHECK(core.propagate(static_increment(core.nominal(), 0.001)) == kOk);
    }

    /* The true relative height is 5 m above the origin; the filter starts at
       0 m with b_h = 0, so the update has to move the estimate upwards. */
    const double true_relative_height = 5.0;
    BaroObs obs;
    obs.height = true_relative_height;
    obs.std = 0.2;

    const double prediction_before = baroPrediction(core.nominal(), cfg);
    CHECK(core.updateBaro(obs) == kOk);
    CHECK(core.diagnostics().baro_updates == 1u);
    CHECK(std::fabs(baroPrediction(core.nominal(), cfg) - true_relative_height) <
          std::fabs(prediction_before - true_relative_height));

    /* Height/bias cross covariance must be produced by the update, not
       cleared (spec 05 §2.3). */
    CHECK(std::fabs(core.covariance()(kIdxP + 2, kIdxBh)) > 1.0e-9);
    CHECK_NEAR(core.covariance()(kIdxP + 2, kIdxBh), core.covariance()(kIdxBh, kIdxP + 2), 1.0e-15);

    /* Repeated measurements converge towards the truth and drive the height
       uncertainty down. */
    for (int i = 0; i < 200; ++i) {
        CHECK(core.propagate(static_increment(core.nominal(), 0.001)) == kOk);
        CHECK(core.updateBaro(obs) == kOk);
    }
    CHECK_NEAR(baroPrediction(core.nominal(), cfg), true_relative_height, 0.02);
    CHECK(core.covariance()(kIdxP + 2, kIdxP + 2) < init.pos_std(2, 0) * init.pos_std(2, 0));
}

void test_gnss_position_update() {
    QuadCore core;
    const QuadConfig cfg = make_config();
    const QuadInitState init = make_init();
    CHECK(core.initialize(cfg, init) == kOk);

    for (int i = 0; i < 100; ++i) {
        CHECK(core.propagate(static_increment(core.nominal(), 0.001)) == kOk);
    }

    /* Antenna position of the initial state plus a 2 m north offset. */
    GnssPositionObs obs;
    obs.blh = gnssPositionPrediction(core.nominal(), cfg);
    obs.blh(0, 0) += 2.0 / (WGS84_RA + obs.blh(2, 0));
    obs.std = vec3(0.3, 0.3, 0.5);

    const double std_before = std::sqrt(core.covariance()(kIdxP, kIdxP));
    CHECK(core.updateGnssPosition(obs) == kOk);
    CHECK(core.diagnostics().gnss_pos_updates == 1u);
    CHECK(core.nominal().pos(0, 0) > init.nominal.pos(0, 0));
    CHECK(std::sqrt(core.covariance()(kIdxP, kIdxP)) < std_before);
    CHECK(isPositiveDefinite(core.covariance()));
}

void test_gnss_velocity_update() {
    QuadCore core;
    const QuadConfig cfg = make_config();
    const QuadInitState init = make_init();
    CHECK(core.initialize(cfg, init) == kOk);

    for (int i = 0; i < 100; ++i) {
        CHECK(core.propagate(static_increment(core.nominal(), 0.001)) == kOk);
    }

    GnssVelocityObs obs;
    obs.vel_ned = vec3(core.nominal().vel(0, 0) + 0.20, core.nominal().vel(1, 0) - 0.10,
                       core.nominal().vel(2, 0) + 0.05);
    obs.std = vec3(0.05, 0.05, 0.05);
    obs.omega_ib = vec3(0.0, 0.0, 0.0);

    CHECK(core.updateGnssVelocity(obs) == kOk);
    CHECK(core.diagnostics().gnss_vel_updates == 1u);
    CHECK(core.nominal().vel(0, 0) > 0.1);
    CHECK(core.nominal().vel(1, 0) < -0.05);
}

void test_nis_gate_and_failure_atomicity() {
    QuadCore core;
    const QuadConfig cfg = make_config();
    const QuadInitState init = make_init();
    CHECK(core.initialize(cfg, init) == kOk);

    for (int i = 0; i < 100; ++i) {
        CHECK(core.propagate(static_increment(core.nominal(), 0.001)) == kOk);
    }

    BaroObs outlier;
    outlier.height = 500.0; /* far outside any plausible innovation */
    outlier.std = 0.2;

    const double height_before = core.nominal().pos(2, 0);
    const double bias_before = core.nominal().baro_bias;
    double covariance_before[kRank22 * kRank22];
    core.copyCovariance(covariance_before);

    CHECK(core.updateBaro(outlier) == kErrNisRejected);
    CHECK(core.diagnostics().nis_rejected == 1u);
    CHECK(core.diagnostics().baro_updates == 0u);
    CHECK_NEAR(core.nominal().pos(2, 0), height_before, 0.0);
    CHECK_NEAR(core.nominal().baro_bias, bias_before, 0.0);

    double covariance_after[kRank22 * kRank22];
    core.copyCovariance(covariance_after);
    for (int i = 0; i < kRank22 * kRank22; ++i) {
        CHECK_NEAR(covariance_after[i], covariance_before[i], 0.0);
    }

    /* A normal measurement is still accepted afterwards. */
    BaroObs good;
    good.height = 0.0;
    good.std = 0.5;
    CHECK(core.updateBaro(good) == kOk);
}

void test_update_input_validation() {
    QuadCore core;
    const QuadConfig cfg = make_config();
    const QuadInitState init = make_init();
    CHECK(core.initialize(cfg, init) == kOk);

    GnssPositionObs bad_pos;
    bad_pos.blh = vec3(0.5, 2.0, 35.0);
    bad_pos.std = vec3(-1.0, 1.0, 1.0);
    CHECK(core.updateGnssPosition(bad_pos) == kErrArgument);

    BaroObs bad_baro;
    bad_baro.height = 0.0;
    bad_baro.std = 0.0;
    CHECK(core.updateBaro(bad_baro) == kErrArgument);

    GnssVelocityObs bad_vel;
    bad_vel.vel_ned = vec3(0.0, 0.0, 0.0);
    bad_vel.std = vec3(1.0, 1.0, 1.0);
    bad_vel.omega_ib = vec3(NAN, 0.0, 0.0);
    CHECK(core.updateGnssVelocity(bad_vel) == kErrArgument);

    CHECK(core.diagnostics().gnss_pos_updates == 0u);
    CHECK(core.diagnostics().baro_updates == 0u);
    CHECK(core.diagnostics().gnss_vel_updates == 0u);
}

void test_uninitialized_use() {
    QuadCore core;
    CHECK(core.propagate(static_increment(make_init().nominal, 0.001)) == kErrNotInitialized);

    BaroObs baro;
    baro.height = 0.0;
    baro.std = 1.0;
    CHECK(core.updateBaro(baro) == kErrNotInitialized);

    GnssPositionObs position;
    position.blh = vec3(0.5, 2.0, 35.0);
    position.std = vec3(1.0, 1.0, 1.0);
    CHECK(core.updateGnssPosition(position) == kErrNotInitialized);
}

} // namespace

int main() {
    std::printf("KF-GINS_QUAD core unit tests\n");
    test_initialization_validation();
    test_propagation_invariants();
    test_dt_guard();
    test_baro_update();
    test_gnss_position_update();
    test_gnss_velocity_update();
    test_nis_gate_and_failure_atomicity();
    test_update_input_validation();
    test_uninitialized_use();
    return hosttest::summary("test_core");
}
