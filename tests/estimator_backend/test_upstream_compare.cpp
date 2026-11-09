/*
 * Pristine-upstream numerical comparison.
 *
 * Builds the locked upstream KF-GINS sources (src/kf-gins/gi_engine.cpp and
 * insmech.cpp, commit 8291a93e49de513fe9d21f819500d39082ded611) unmodified
 * against the upstream ThirdParty Eigen 3.3.9 headers, feeds exactly the same
 * IMU increments and GNSS position measurements to both engines, and compares
 * the resulting navigation state and covariance.
 *
 * The 22nd state of the QUAD core is held inert here (q_bh = 0, sigma_bh
 * tiny), so the 21x21 block must reproduce the upstream 21-state filter.  A
 * mismatch means the fixed-dimension trim changed the algorithm, which is
 * exactly what this test is meant to detect.
 *
 * Not part of the firmware build: requires the upstream checkout.
 */

#include <cmath>\n#include <cstdlib>\n#include <cstring>
#include <cstdio>
#include <iostream>
#include <sstream>

#include <Eigen/Dense>

#include "common/types.h"
#include "kf-gins/gi_engine.h"

#include "kfgins/quad_core.hpp"
#include "test_support.hpp"

using namespace kfg;

namespace {

struct IMUIncrement {
    double dt;
    Vec3 dtheta;
    Vec3 dvel;
};

QuadInitState make_quad_init() {
    QuadInitState init;
    init.nominal.pos = vec3(30.5 * kD2R, 114.3 * kD2R, 35.0);
    init.nominal.vel = vec3(0.0, 0.0, 0.0);
    init.nominal.euler = vec3(0.02, -0.03, 0.7);
    init.nominal.cbn = eulerToMatrix(init.nominal.euler);
    init.nominal.qbn = matrixToQuat(init.nominal.cbn);
    init.nominal.gyrbias = vec3(2.0e-4, -1.0e-4, 3.0e-4);
    init.nominal.accbias = vec3(0.01, -0.02, 0.03);
    init.nominal.gyrscale = vec3(0.0, 0.0, 0.0);
    init.nominal.accscale = vec3(0.0, 0.0, 0.0);
    init.nominal.baro_bias = 0.0;

    init.pos_std = vec3(2.0, 2.0, 3.0);
    init.vel_std = vec3(0.2, 0.2, 0.2);
    init.euler_std = vec3(0.02, 0.02, 0.1);
    init.gyrbias_std = vec3(5.0e-4, 5.0e-4, 5.0e-4);
    init.accbias_std = vec3(0.02, 0.02, 0.02);
    init.gyrscale_std = vec3(1.0e-4, 1.0e-4, 1.0e-4);
    init.accscale_std = vec3(1.0e-4, 1.0e-4, 1.0e-4);
    init.baro_bias_std = 1.0e-6;
    return init;
}

QuadConfig make_quad_config() {
    QuadConfig cfg;
    cfg.imunoise.gyr_arw = vec3(0.0015, 0.0015, 0.0015);
    cfg.imunoise.acc_vrw = vec3(0.025, 0.025, 0.025);
    cfg.imunoise.gyrbias_std = vec3(5.0e-4, 5.0e-4, 5.0e-4);
    cfg.imunoise.accbias_std = vec3(0.02, 0.02, 0.02);
    cfg.imunoise.gyrscale_std = vec3(1.0e-4, 1.0e-4, 1.0e-4);
    cfg.imunoise.accscale_std = vec3(1.0e-4, 1.0e-4, 1.0e-4);
    cfg.imunoise.corr_time = 3600.0;
    cfg.imunoise.baro_bias_walk = 0.0;
    cfg.antlever = vec3(0.12, -0.05, 0.21);
    cfg.nis_gate_enable = false; /* isolate numerics from the NIS gate */
    cfg.baro_origin_height_m = 0.0;
    return cfg;
}

GINSOptions make_upstream_options(const QuadInitState &init, const QuadConfig &cfg) {
    GINSOptions opt;
    opt.initstate.pos = Eigen::Vector3d(init.nominal.pos(0, 0), init.nominal.pos(1, 0), init.nominal.pos(2, 0));
    opt.initstate.vel = Eigen::Vector3d(init.nominal.vel(0, 0), init.nominal.vel(1, 0), init.nominal.vel(2, 0));
    opt.initstate.euler =
        Eigen::Vector3d(init.nominal.euler(0, 0), init.nominal.euler(1, 0), init.nominal.euler(2, 0));
    opt.initstate.imuerror.gyrbias =
        Eigen::Vector3d(init.nominal.gyrbias(0, 0), init.nominal.gyrbias(1, 0), init.nominal.gyrbias(2, 0));
    opt.initstate.imuerror.accbias =
        Eigen::Vector3d(init.nominal.accbias(0, 0), init.nominal.accbias(1, 0), init.nominal.accbias(2, 0));
    opt.initstate.imuerror.gyrscale =
        Eigen::Vector3d(init.nominal.gyrscale(0, 0), init.nominal.gyrscale(1, 0), init.nominal.gyrscale(2, 0));
    opt.initstate.imuerror.accscale =
        Eigen::Vector3d(init.nominal.accscale(0, 0), init.nominal.accscale(1, 0), init.nominal.accscale(2, 0));

    opt.initstate_std.pos = Eigen::Vector3d(init.pos_std(0, 0), init.pos_std(1, 0), init.pos_std(2, 0));
    opt.initstate_std.vel = Eigen::Vector3d(init.vel_std(0, 0), init.vel_std(1, 0), init.vel_std(2, 0));
    opt.initstate_std.euler = Eigen::Vector3d(init.euler_std(0, 0), init.euler_std(1, 0), init.euler_std(2, 0));
    opt.initstate_std.imuerror.gyrbias =
        Eigen::Vector3d(init.gyrbias_std(0, 0), init.gyrbias_std(1, 0), init.gyrbias_std(2, 0));
    opt.initstate_std.imuerror.accbias =
        Eigen::Vector3d(init.accbias_std(0, 0), init.accbias_std(1, 0), init.accbias_std(2, 0));
    opt.initstate_std.imuerror.gyrscale =
        Eigen::Vector3d(init.gyrscale_std(0, 0), init.gyrscale_std(1, 0), init.gyrscale_std(2, 0));
    opt.initstate_std.imuerror.accscale =
        Eigen::Vector3d(init.accscale_std(0, 0), init.accscale_std(1, 0), init.accscale_std(2, 0));

    opt.imunoise.gyr_arw =
        Eigen::Vector3d(cfg.imunoise.gyr_arw(0, 0), cfg.imunoise.gyr_arw(1, 0), cfg.imunoise.gyr_arw(2, 0));
    opt.imunoise.acc_vrw =
        Eigen::Vector3d(cfg.imunoise.acc_vrw(0, 0), cfg.imunoise.acc_vrw(1, 0), cfg.imunoise.acc_vrw(2, 0));
    opt.imunoise.gyrbias_std = Eigen::Vector3d(cfg.imunoise.gyrbias_std(0, 0), cfg.imunoise.gyrbias_std(1, 0),
                                               cfg.imunoise.gyrbias_std(2, 0));
    opt.imunoise.accbias_std = Eigen::Vector3d(cfg.imunoise.accbias_std(0, 0), cfg.imunoise.accbias_std(1, 0),
                                               cfg.imunoise.accbias_std(2, 0));
    opt.imunoise.gyrscale_std = Eigen::Vector3d(cfg.imunoise.gyrscale_std(0, 0), cfg.imunoise.gyrscale_std(1, 0),
                                                cfg.imunoise.gyrscale_std(2, 0));
    opt.imunoise.accscale_std = Eigen::Vector3d(cfg.imunoise.accscale_std(0, 0), cfg.imunoise.accscale_std(1, 0),
                                                cfg.imunoise.accscale_std(2, 0));
    opt.imunoise.corr_time = cfg.imunoise.corr_time;
    opt.antlever = Eigen::Vector3d(cfg.antlever(0, 0), cfg.antlever(1, 0), cfg.antlever(2, 0));
    return opt;
}

} // namespace

int main(int argc, char **argv) {
    const bool use_gnss = !(argc > 1 && std::strcmp(argv[1], "--no-gnss") == 0);

    const QuadInitState init = make_quad_init();
    const QuadConfig config = make_quad_config();

    /* Silence the upstream GINSOptions::print_options() console dump. */
    std::ostringstream sink;
    std::streambuf *saved = std::cout.rdbuf(sink.rdbuf());
    GINSOptions options = make_upstream_options(init, config);
    GIEngine upstream(options);
    std::cout.rdbuf(saved);

    QuadCore quad;
    CHECK(quad.initialize(config, init) == kOk);

    const double dt = 0.005;
    const int epochs = 400;          /* 2 s at 200 Hz */
    const int gnss_period = 20;      /* 10 Hz GNSS position */
    const double gnss_sigma = 0.5;   /* m per axis */

    /* Constant body rate and specific force: a slow, smooth trajectory that
     * exercises the coning/sculling terms and the lever arm. */
    const Vec3 omega_body = vec3(0.02, -0.01, 0.05);
    const Vec3 accel_body = vec3(0.10, -0.20, -9.79);

    double max_pos_diff = 0.0;
    double max_vel_diff = 0.0;
    double max_att_diff = 0.0;
    double max_bias_diff = 0.0;
    double max_cov_diff = 0.0;

    for (int epoch = 0; epoch < epochs; ++epoch) {
        const double time = (epoch + 1) * dt;

        IMUIncrement increment;
        increment.dt = dt;
        increment.dtheta = omega_body * dt;
        increment.dvel = accel_body * dt;

        IMU upstream_imu;
        upstream_imu.time = time;
        upstream_imu.dt = dt;
        upstream_imu.dtheta = Eigen::Vector3d(increment.dtheta(0, 0), increment.dtheta(1, 0),
                                              increment.dtheta(2, 0));
        upstream_imu.dvel = Eigen::Vector3d(increment.dvel(0, 0), increment.dvel(1, 0), increment.dvel(2, 0));
        upstream_imu.odovel = 0.0;

        const bool gnss_epoch = use_gnss && (((epoch + 1) % gnss_period) == 0);

        /* Identical measurement for both engines: a fixed point near the
         * initial antenna position with a deterministic offset pattern. */
        const double phase = 0.1 * static_cast<double>(epoch);
        GNSS upstream_gnss;
        Vec3 gnss_blh = init.nominal.pos;
        gnss_blh(0, 0) += gnss_sigma * std::sin(phase) / (WGS84_RA + init.nominal.pos(2, 0));
        gnss_blh(1, 0) += gnss_sigma * std::cos(phase) / ((WGS84_RA + init.nominal.pos(2, 0)) *
                                                          std::cos(init.nominal.pos(0, 0)));
        gnss_blh(2, 0) += 0.1 * std::sin(0.5 * phase);
        upstream_gnss.time = time;
        upstream_gnss.blh = Eigen::Vector3d(gnss_blh(0, 0), gnss_blh(1, 0), gnss_blh(2, 0));
        upstream_gnss.std = Eigen::Vector3d(gnss_sigma, gnss_sigma, 0.5);
        upstream_gnss.isvalid = false;

        if (gnss_epoch) {
            upstream.addGnssData(upstream_gnss);
        }
        upstream.addImuData(upstream_imu);

        CHECK(quad.propagate(ImuIncrement{dt, increment.dtheta, increment.dvel}) == kOk);

        if (gnss_epoch) {
            GnssPositionObs obs;
            obs.blh = gnss_blh;
            obs.std = vec3(gnss_sigma, gnss_sigma, 0.5);
            CHECK(quad.updateGnssPosition(obs) == kOk);
        }

        /* The upstream engine has to run in the same order. */
        upstream.newImuProcess();

        const NavState reference = upstream.getNavState();
        const QuadNominal &actual = quad.nominal();

        const Mat3 dr = earthDR(init.nominal.pos);
        const Vec3 pos_delta =
            matvec(dr, vec3(actual.pos(0, 0) - reference.pos(0),
                            actual.pos(1, 0) - reference.pos(1),
                            actual.pos(2, 0) - reference.pos(2)));
        const double pos_diff = std::max(std::fabs(pos_delta(0, 0)),
                                         std::max(std::fabs(pos_delta(1, 0)), std::fabs(pos_delta(2, 0))));
        const double vel_diff =
            std::max(std::fabs(actual.vel(0, 0) - reference.vel(0)),
                     std::max(std::fabs(actual.vel(1, 0) - reference.vel(1)),
                              std::fabs(actual.vel(2, 0) - reference.vel(2))));
        const double att_diff =
            std::max(std::fabs(actual.euler(0, 0) - reference.euler(0)),
                     std::max(std::fabs(actual.euler(1, 0) - reference.euler(1)),
                              std::fabs(actual.euler(2, 0) - reference.euler(2))));
        const double bias_diff =
            std::max(std::fabs(actual.gyrbias(0, 0) - reference.imuerror.gyrbias(0)),
                     std::max(std::fabs(actual.accbias(2, 0) - reference.imuerror.accbias(2)),
                              std::fabs(actual.gyrscale(1, 0) - reference.imuerror.gyrscale(1))));

        if (std::getenv("KFG_VERBOSE") != nullptr) {
            std::printf("epoch %3d%s dpos=%.3e dvel=%.3e datt=%.3e\n", epoch,
                        gnss_epoch ? " (gnss)" : "       ", pos_diff, vel_diff, att_diff);
        }

        max_pos_diff = std::max(max_pos_diff, pos_diff);
        max_vel_diff = std::max(max_vel_diff, vel_diff);
        max_att_diff = std::max(max_att_diff, att_diff);
        max_bias_diff = std::max(max_bias_diff, bias_diff);

        const Eigen::MatrixXd reference_cov = upstream.getCovariance();
        for (int r = 0; r < 21; ++r) {
            for (int c = 0; c < 21; ++c) {
                const double cov_diff = std::fabs(quad.covariance()(r, c) - reference_cov(r, c));
                max_cov_diff = std::max(max_cov_diff, cov_diff);
            }
        }
    }

    std::printf("pristine upstream comparison: %d epochs, %d GNSS position updates\n", epochs,
                epochs / gnss_period);
    std::printf("  max |dpos|   = %.3e m\n", max_pos_diff);
    std::printf("  max |dvel|   = %.3e m/s\n", max_vel_diff);
    std::printf("  max |datt|   = %.3e rad\n", max_att_diff);
    std::printf("  max |dbias|  = %.3e\n", max_bias_diff);
    std::printf("  max |dP|     = %.3e\n", max_cov_diff);

    /*
     * The fixed-dimension trim must be algebraically identical to the
     * upstream 21-state filter.  The residual is floating-point reassociation
     * (different matrix traversal order, a different quaternion normalisation
     * path, Cholesky solve instead of S.inverse()) amplified by the feedback
     * loop of the filter itself.  A genuine algorithm change (state order,
     * sign convention, Jacobian, gain) shows up at 1e-2 or larger, i.e. five
     * orders of magnitude above these bounds.
     */
    CHECK_NEAR(max_pos_diff, 0.0, 1.0e-6);
    CHECK_NEAR(max_vel_diff, 0.0, 1.0e-6);
    CHECK_NEAR(max_att_diff, 0.0, 1.0e-7);
    CHECK_NEAR(max_bias_diff, 0.0, 1.0e-9);
    CHECK_NEAR(max_cov_diff, 0.0, 1.0e-8);

    return hosttest::summary("test_upstream_compare");
}
