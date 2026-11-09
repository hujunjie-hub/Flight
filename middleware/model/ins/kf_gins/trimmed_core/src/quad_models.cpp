/*
 * KF-GINS_QUAD observation models.
 *
 * See quad_models.hpp for the contract.  All derivatives below follow the
 * error-state retraction of quad_types.hpp and the residual convention
 * "predicted - measured".
 */

#include "kfgins/quad_models.hpp"

namespace kfg {

void inertialRate(const Vec3 &blh, const Vec3 &vel_ned, Vec3 &wie_n, Vec3 &wen_n) {
    double rm, rn;
    meridianPrimeVerticalRadius(blh(0, 0), rm, rn);
    const double h = blh(2, 0);
    const double lat = blh(0, 0);

    wie_n(0, 0) = WGS84_WIE * std::cos(lat);
    wie_n(1, 0) = 0.0;
    wie_n(2, 0) = -WGS84_WIE * std::sin(lat);

    wen_n(0, 0) = vel_ned(1, 0) / (rn + h);
    wen_n(1, 0) = -vel_ned(0, 0) / (rm + h);
    wen_n(2, 0) = -vel_ned(1, 0) * std::tan(lat) / (rn + h);
}

Vec3 gnssPositionPrediction(const QuadNominal &nominal, const QuadConfig &config) {
    const Mat3 dri = earthDRi(nominal.pos);
    return nominal.pos + matvec(dri, matvec(nominal.cbn, config.antlever));
}

void gnssPositionJacobian(const QuadNominal &nominal, const QuadConfig &config, Mat3x22 &h) {
    setZero(h);
    /* Local-NED residual: dz = delta_p + skew(C*l) * delta_phi. */
    for (int i = 0; i < 3; ++i) {
        h(i, kIdxP + i) = 1.0;
    }
    const Vec3 lever_nav = matvec(nominal.cbn, config.antlever);
    const Mat3 lever_skew = skew(lever_nav);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            h(r, kIdxPhi + c) = lever_skew(r, c);
        }
    }
}

/* Body rate with the current bias/scale estimates removed (upstream
 * GIEngine::imuCompensate applied to an angular rate). */
static Vec3 compensateRate(const QuadNominal &nominal, const Vec3 &omega_meas) {
    Vec3 omega;
    for (int i = 0; i < 3; ++i) {
        double scale = 1.0 + nominal.gyrscale(i, 0);
        if (std::fabs(scale) < 1.0e-6) {
            scale = (scale < 0.0) ? -1.0e-6 : 1.0e-6;
        }
        omega(i, 0) = (omega_meas(i, 0) - nominal.gyrbias(i, 0)) / scale;
    }
    return omega;
}

Vec3 gnssVelocityPrediction(const QuadNominal &nominal, const QuadConfig &config, const Vec3 &omega_meas) {
    const Vec3 omega = compensateRate(nominal, omega_meas);
    const Vec3 lever_nav = matvec(nominal.cbn, config.antlever);
    const Vec3 rotational = matvec(nominal.cbn, cross(omega, config.antlever));

    Vec3 wie_n, wen_n;
    inertialRate(nominal.pos, nominal.vel, wie_n, wen_n);

    const Vec3 omega_in = vec3(wie_n(0, 0) + wen_n(0, 0), wie_n(1, 0) + wen_n(1, 0),
                               wie_n(2, 0) + wen_n(2, 0));
    const Vec3 navigation = cross(omega_in, lever_nav);

    return vec3(nominal.vel(0, 0) + rotational(0, 0) - navigation(0, 0),
                nominal.vel(1, 0) + rotational(1, 0) - navigation(1, 0),
                nominal.vel(2, 0) + rotational(2, 0) - navigation(2, 0));
}

void gnssVelocityJacobian(const QuadNominal &nominal, const QuadConfig &config, const Vec3 &omega_meas,
                          Mat3x22 &h) {
    setZero(h);

    const Vec3 omega = compensateRate(nominal, omega_meas);

    const double lat = nominal.pos(0, 0);
    const double hgt = nominal.pos(2, 0);
    double rm, rn;
    meridianPrimeVerticalRadius(lat, rm, rn);
    const double rmh = rm + hgt;
    const double rnh = rn + hgt;
    const double coslat = std::cos(lat);

    const Vec3 lever_nav = matvec(nominal.cbn, config.antlever);
    const Mat3 lever_skew = skew(lever_nav);

    Vec3 wie_n, wen_n;
    inertialRate(nominal.pos, nominal.vel, wie_n, wen_n);
    const Vec3 omega_in = vec3(wie_n(0, 0) + wen_n(0, 0), wie_n(1, 0) + wen_n(1, 0),
                               wie_n(2, 0) + wen_n(2, 0));
    const Mat3 omega_in_skew = skew(omega_in);

    /* d(omega_in)/d(local NED position), rows = nav axes. */
    Mat3 omega_p;
    setZero(omega_p);
    {
        const Vec3 dphi = vec3(-WGS84_WIE * std::sin(lat), 0.0,
                               -WGS84_WIE * std::cos(lat) -
                                   nominal.vel(1, 0) / (rnh * coslat * coslat));
        const Vec3 dh = vec3(-nominal.vel(1, 0) / (rnh * rnh), nominal.vel(0, 0) / (rmh * rmh),
                             nominal.vel(1, 0) * std::tan(lat) / (rnh * rnh));
        for (int i = 0; i < 3; ++i) {
            omega_p(i, 0) = dphi(i, 0) / rmh;
            omega_p(i, 1) = 0.0;
            omega_p(i, 2) = -dh(i, 0);
        }
    }

    /* d(omega_in)/d(NED velocity). */
    Mat3 omega_v;
    setZero(omega_v);
    omega_v(0, 1) = 1.0 / rnh;
    omega_v(1, 0) = -1.0 / rmh;
    omega_v(2, 1) = -std::tan(lat) / rnh;

    const Mat3 lever_omega_p = matmul3(lever_skew, omega_p);
    const Mat3 lever_omega_v = matmul3(lever_skew, omega_v);

    /* H_p and H_v. */
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            h(r, kIdxP + c) = lever_omega_p(r, c);
            h(r, kIdxV + c) = ((r == c) ? 1.0 : 0.0) + lever_omega_v(r, c);
        }
    }

    /* H_phi = skew(W) - skew(omega_in) * skew(L) with W = C (omega x lever). */
    const Vec3 w_vec = matvec(nominal.cbn, cross(omega, config.antlever));
    const Mat3 w_skew = skew(w_vec);
    const Mat3 phi_term = matmul3(omega_in_skew, lever_skew);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            h(r, kIdxPhi + c) = w_skew(r, c) - phi_term(r, c);
        }
    }

    /*
     * H_bg = -[L x] * C * B and H_sg = -[L x] * C * S with
     * B = diag(1/(1+s_g)) and S = diag(omega/(1+s_g)) (spec 05 §7 writes it
     * as -C*skew(l)*B with the body-frame lever arm l).  The cross product is
     * taken with the body-frame lever arm and then rotated, so the nav-frame
     * skew(L) multiplies C from the left.  A guard keeps 1/(1+s) finite for
     * corrupt scale states.
     */
    Mat3 b_diag;
    Mat3 s_diag;
    setZero(b_diag);
    setZero(s_diag);
    for (int i = 0; i < 3; ++i) {
        double scale = 1.0 + nominal.gyrscale(i, 0);
        if (std::fabs(scale) < 1.0e-6) {
            scale = (scale < 0.0) ? -1.0e-6 : 1.0e-6;
        }
        b_diag(i, i) = 1.0 / scale;
        s_diag(i, i) = omega(i, 0) / scale;
    }

    const Mat3 h_bg = matmul3(lever_skew, matmul3(nominal.cbn, b_diag));
    const Mat3 h_sg = matmul3(lever_skew, matmul3(nominal.cbn, s_diag));
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            h(r, kIdxBg + c) = -h_bg(r, c);
            h(r, kIdxSg + c) = -h_sg(r, c);
        }
    }
}

double baroPrediction(const QuadNominal &nominal, const QuadConfig &config) {
    return (nominal.pos(2, 0) - config.baro_origin_height_m) + nominal.baro_bias;
}

void baroJacobian(Mat<1, 22> &h) {
    setZero(h);
    h(0, kIdxP + 2) = -1.0;
    h(0, kIdxBh) = -1.0;
}

} // namespace kfg
