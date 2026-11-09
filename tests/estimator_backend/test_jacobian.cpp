/*
 * Finite-difference verification of the analytic observation Jacobians.
 *
 * spec 05 §2.3 requires: perturb the error state with the same retraction the
 * filter uses, form h(x_hat) - h(retract(x_hat, delta_x)), and compare with
 * the analytic H.  The prediction functions are the very same ones the filter
 * calls, so this test also protects against a model/Jacobian mismatch.
 */

#include <cstdio>

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

QuadNominal make_nominal() {
    QuadNominal n;
    n.pos = vec3(30.5 * kD2R, 114.3 * kD2R, 35.0);
    n.vel = vec3(3.2, -1.7, 0.4);
    n.euler = vec3(0.05, -0.12, 1.1);
    n.cbn = eulerToMatrix(n.euler);
    n.qbn = matrixToQuat(n.cbn);
    n.gyrbias = vec3(1.0e-4, -2.0e-4, 3.0e-4);
    n.accbias = vec3(0.02, -0.03, 0.05);
    n.gyrscale = vec3(2.0e-5, -3.0e-5, 1.0e-5);
    n.accscale = vec3(-1.0e-5, 4.0e-5, -2.0e-5);
    n.baro_bias = 1.5;
    return n;
}

/*
 * Step sizes are chosen at the geometric mean of the two error sources of a
 * forward difference on a geodetic state: the O(eps) truncation term and the
 * round-off floor of representing a sub-micrometre displacement in radians
 * (about 6.4e6 m * 2^-52 per ulp).  The resulting floor is visible in the
 * printed max error and is the reason the attitude columns of the position
 * Jacobian are checked with a 2e-3 relative tolerance; every other column is
 * checked tightly.
 */
double epsilon_for(int index) {
    if (index < 3) {
        return 1.0e-2; /* position, m */
    }
    if (index < 6) {
        return 1.0e-6; /* velocity, m/s */
    }
    if (index < 9) {
        return 1.0e-5; /* attitude, rad */
    }
    if (index < 12) {
        return 1.0e-8; /* gyro bias */
    }
    if (index < 15) {
        return 1.0e-6; /* accel bias */
    }
    if (index < 21) {
        return 1.0e-8; /* scale factors */
    }
    return 1.0e-2; /* baro bias, m */
}

void perturbed_state(const QuadNominal &nominal, int index, double epsilon, QuadNominal &out) {
    Vec22 d;
    setZero(d);
    d(index, 0) = epsilon;
    retract(nominal, d, out);
}

/* GNSS position: residual is Dr * (predicted_blh - true_blh). */
void test_gnss_position_jacobian() {
    const QuadConfig cfg = make_config();
    const QuadNominal nominal = make_nominal();

    Mat3x22 analytic;
    gnssPositionJacobian(nominal, cfg, analytic);
    const Mat3 dr = earthDR(nominal.pos);
    const Vec3 base = gnssPositionPrediction(nominal, cfg);

    double max_error = 0.0;
    for (int i = 0; i < kRank22; ++i) {
        QuadNominal perturbed;
        perturbed_state(nominal, i, epsilon_for(i), perturbed);
        const Vec3 shifted = gnssPositionPrediction(perturbed, cfg);
        const Vec3 diff = matvec(dr, base - shifted);
        for (int r = 0; r < 3; ++r) {
            const double numeric = diff(r, 0) / epsilon_for(i);
            const double error = std::fabs(numeric - analytic(r, i));
            if (error > max_error) {
                max_error = error;
            }
            const bool attitude_column = (i >= kIdxPhi) && (i < kIdxPhi + 3);
            const double tolerance =
                (attitude_column ? 2.0e-3 : 2.0e-5) * (1.0 + std::fabs(analytic(r, i)));
            CHECK_NEAR(numeric, analytic(r, i), tolerance);
        }
    }
    std::printf("gnss_position Jacobian: max |analytic - numeric| = %.3e\n", max_error);
}

/* GNSS velocity: residual is predicted_antenna_velocity - measured. */
void test_gnss_velocity_jacobian(double scale_magnitude) {
    const QuadConfig cfg = make_config();
    QuadNominal nominal = make_nominal();
    nominal.gyrscale = vec3(scale_magnitude, -scale_magnitude, 0.5 * scale_magnitude);
    const Vec3 omega = vec3(0.3, -0.2, 0.5);

    Mat3x22 analytic;
    gnssVelocityJacobian(nominal, cfg, omega, analytic);
    const Vec3 base = gnssVelocityPrediction(nominal, cfg, omega);

    double max_error = 0.0;
    int worst_index = -1;
    for (int i = 0; i < kRank22; ++i) {
        QuadNominal perturbed;
        perturbed_state(nominal, i, epsilon_for(i), perturbed);
        const Vec3 shifted = gnssVelocityPrediction(perturbed, cfg, omega);
        const Vec3 diff = base - shifted;
        for (int r = 0; r < 3; ++r) {
            const double numeric = diff(r, 0) / epsilon_for(i);
            const double error = std::fabs(numeric - analytic(r, i));
            if (error > max_error) {
                max_error = error;
                worst_index = i;
            }
        }
    }

    /* Scale-factor columns are checked with a relative tolerance because
     * 05 §7 defines S = diag(omega/(1+s)) while the exact derivative of
     * 1/(1+s) carries a second (1+s) factor; see the report for the O(s^2)
     * difference.  With s = 0 both definitions coincide exactly. */
    for (int i = 0; i < kRank22; ++i) {
        QuadNominal perturbed;
        perturbed_state(nominal, i, epsilon_for(i), perturbed);
        const Vec3 shifted = gnssVelocityPrediction(perturbed, cfg, omega);
        const Vec3 diff = base - shifted;
        for (int r = 0; r < 3; ++r) {
            const double numeric = diff(r, 0) / epsilon_for(i);
            const double tolerance = 2.0e-4 * (1.0 + std::fabs(analytic(r, i)));
            CHECK_NEAR(numeric, analytic(r, i), tolerance);
        }
    }

    std::printf("gnss_velocity Jacobian (|s| = %.1e): max |analytic - numeric| = %.3e at index %d\n",
                scale_magnitude, max_error, worst_index);
}

void test_baro_jacobian() {
    const QuadConfig cfg = make_config();
    const QuadNominal nominal = make_nominal();

    Mat<1, 22> analytic;
    baroJacobian(analytic);
    const double base = baroPrediction(nominal, cfg);

    double max_error = 0.0;
    for (int i = 0; i < kRank22; ++i) {
        QuadNominal perturbed;
        perturbed_state(nominal, i, epsilon_for(i), perturbed);
        const double numeric = (base - baroPrediction(perturbed, cfg)) / epsilon_for(i);
        const double error = std::fabs(numeric - analytic(0, i));
        if (error > max_error) {
            max_error = error;
        }
        CHECK_NEAR(numeric, analytic(0, i), 1.0e-6);
    }
    std::printf("baro Jacobian: max |analytic - numeric| = %.3e\n", max_error);
}

} // namespace

int main() {
    std::printf("KF-GINS_QUAD analytic Jacobian verification (spec 05 §2.3/§7)\n");
    test_gnss_position_jacobian();
    test_gnss_velocity_jacobian(0.0);
    test_gnss_velocity_jacobian(1.0e-3);
    test_baro_jacobian();
    return hosttest::summary("test_jacobian");
}
