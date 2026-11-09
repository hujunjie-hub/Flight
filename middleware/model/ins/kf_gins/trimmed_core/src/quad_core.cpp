/*
 * KF-GINS_QUAD trimmed 22-state core implementation.
 *
 * Functions tagged [UPSTREAM] are line-by-line ports of the locked upstream
 * file named in the comment; functions tagged [QUAD] are project extensions
 * whose design is specified by spec 05 (22-state extension, GNSS velocity,
 * barometer, NIS gating, numerical failure containment).
 */

#include "kfgins/quad_core.hpp"

#include <cmath>
#include <cstring>

#include "kfgins/quad_models.hpp"

namespace kfg {
namespace {

constexpr double kMinDt = 1.0e-6;
constexpr double kMaxDt = 0.5;

Mat3 identity3() {
    Mat3 m;
    setEye(m);
    return m;
}

Mat3 diagonalOf(const Vec3 &v) { return diag(v); }

/* [UPSTREAM] insmech.cpp::velUpdate */
void velUpdate(const QuadNominal &pre, QuadNominal &cur, const ImuIncrement &imu_pre,
               const ImuIncrement &imu_cur) {
    double rm = 0.0, rn = 0.0;
    meridianPrimeVerticalRadius(pre.pos(0, 0), rm, rn);

    Vec3 wie_n, wen_n;
    inertialRate(pre.pos, pre.vel, wie_n, wen_n);
    const double g = gravity(pre.pos);

    const Vec3 temp1 = cross(imu_cur.dtheta, imu_cur.dvel) * 0.5;
    const Vec3 temp2 = cross(imu_pre.dtheta, imu_cur.dvel) / 12.0;
    const Vec3 temp3 = cross(imu_pre.dvel, imu_cur.dtheta) / 12.0;

    const Vec3 d_vfb = imu_cur.dvel + temp1 + temp2 + temp3;

    Vec3 omega_mid;
    for (int i = 0; i < 3; ++i) {
        omega_mid(i, 0) = (wie_n(i, 0) + wen_n(i, 0)) * imu_cur.dt * 0.5;
    }
    const Mat3 cnn_first = identity3() - skew(omega_mid);
    const Vec3 d_vfn_first = matvec(cnn_first, matvec(pre.cbn, d_vfb));

    Vec3 gl = vec3(0.0, 0.0, g);
    const Vec3 coriolis_first = cross(wie_n * 2.0 + wen_n, pre.vel);
    Vec3 d_vgn;
    for (int i = 0; i < 3; ++i) {
        d_vgn(i, 0) = (gl(i, 0) - coriolis_first(i, 0)) * imu_cur.dt;
    }

    Vec3 midvel;
    for (int i = 0; i < 3; ++i) {
        midvel(i, 0) = pre.vel(i, 0) + (d_vfn_first(i, 0) + d_vgn(i, 0)) * 0.5;
    }

    Quat qnn = rotvecToQuat(omega_mid);
    Quat qee = rotvecToQuat(vec3(0.0, 0.0, -WGS84_WIE * imu_cur.dt * 0.5));
    Quat qne = quatNormalized(quatMultiply(quatMultiply(qee, earthQne(pre.pos)), qnn));

    const double mid_height = pre.pos(2, 0) - midvel(2, 0) * imu_cur.dt * 0.5;
    const Vec3 midpos = earthBlh(qne, mid_height);

    meridianPrimeVerticalRadius(midpos(0, 0), rm, rn);
    inertialRate(midpos, midvel, wie_n, wen_n);

    for (int i = 0; i < 3; ++i) {
        omega_mid(i, 0) = (wie_n(i, 0) + wen_n(i, 0)) * imu_cur.dt * 0.5;
    }
    const Mat3 cnn_second = identity3() - skew(omega_mid);
    const Vec3 d_vfn = matvec(cnn_second, matvec(pre.cbn, d_vfb));

    gl = vec3(0.0, 0.0, gravity(midpos));
    const Vec3 coriolis_second = cross(wie_n * 2.0 + wen_n, midvel);
    for (int i = 0; i < 3; ++i) {
        d_vgn(i, 0) = (gl(i, 0) - coriolis_second(i, 0)) * imu_cur.dt;
    }

    for (int i = 0; i < 3; ++i) {
        cur.vel(i, 0) = pre.vel(i, 0) + d_vfn(i, 0) + d_vgn(i, 0);
    }
}

/* [UPSTREAM] insmech.cpp::posUpdate */
void posUpdate(const QuadNominal &pre, QuadNominal &cur, const ImuIncrement &imu_cur) {
    Vec3 midvel;
    for (int i = 0; i < 3; ++i) {
        midvel(i, 0) = (cur.vel(i, 0) + pre.vel(i, 0)) * 0.5;
    }

    const Vec3 delta_ned = midvel * (imu_cur.dt * 0.5);
    Vec3 midpos = pre.pos + matvec(earthDRi(pre.pos), delta_ned);

    double rm = 0.0, rn = 0.0;
    meridianPrimeVerticalRadius(midpos(0, 0), rm, rn);

    Vec3 wie_n, wen_n;
    inertialRate(midpos, midvel, wie_n, wen_n);

    Vec3 temp1;
    for (int i = 0; i < 3; ++i) {
        temp1(i, 0) = (wie_n(i, 0) + wen_n(i, 0)) * imu_cur.dt;
    }
    const Quat qnn = rotvecToQuat(temp1);
    const Quat qee = rotvecToQuat(vec3(0.0, 0.0, -WGS84_WIE * imu_cur.dt));

    Quat qne = quatNormalized(quatMultiply(quatMultiply(qee, earthQne(pre.pos)), qnn));
    cur.pos(2, 0) = pre.pos(2, 0) - midvel(2, 0) * imu_cur.dt;
    cur.pos = earthBlh(qne, cur.pos(2, 0));
}

/* [UPSTREAM] insmech.cpp::attUpdate */
void attUpdate(const QuadNominal &pre, QuadNominal &cur, const ImuIncrement &imu_pre,
               const ImuIncrement &imu_cur) {
    Vec3 midvel;
    for (int i = 0; i < 3; ++i) {
        midvel(i, 0) = (pre.vel(i, 0) + cur.vel(i, 0)) * 0.5;
    }

    const Quat qne_pre = earthQne(pre.pos);
    const Quat qne_cur = earthQne(cur.pos);
    const Quat qee_full = rotvecToQuat(vec3(0.0, 0.0, -WGS84_WIE * imu_cur.dt));

    const Quat qne_delta = quatNormalized(
        quatMultiply(quatMultiply(quatConjugate(qne_pre), quatConjugate(qee_full)), qne_cur));
    const Vec3 temp1 = quatToRotvec(qne_delta);

    const Quat qee_half = rotvecToQuat(vec3(0.0, 0.0, -WGS84_WIE * imu_cur.dt * 0.5));
    const Quat qne_mid = quatNormalized(quatMultiply(
        quatMultiply(qee_half, qne_pre), rotvecToQuat(temp1 * 0.5)));

    const double mid_height = (cur.pos(2, 0) + pre.pos(2, 0)) * 0.5;
    const Vec3 midpos = earthBlh(qne_mid, mid_height);

    Vec3 wie_n, wen_n;
    inertialRate(midpos, midvel, wie_n, wen_n);

    Vec3 n_rotation;
    for (int i = 0; i < 3; ++i) {
        n_rotation(i, 0) = -(wie_n(i, 0) + wen_n(i, 0)) * imu_cur.dt;
    }
    const Quat qnn = rotvecToQuat(n_rotation);

    Vec3 coning;
    const Vec3 cross_term = cross(imu_pre.dtheta, imu_cur.dtheta) / 12.0;
    for (int i = 0; i < 3; ++i) {
        coning(i, 0) = imu_cur.dtheta(i, 0) + cross_term(i, 0);
    }
    const Quat qbb = rotvecToQuat(coning);

    cur.qbn = quatNormalized(quatMultiply(quatMultiply(qnn, pre.qbn), qbb));
    cur.cbn = quatToMatrix(cur.qbn);
    cur.euler = matrixToEuler(cur.cbn);
}

/* [UPSTREAM] insmech.cpp::insMech */
void insMech(const QuadNominal &pre, QuadNominal &cur, const ImuIncrement &imu_pre,
             const ImuIncrement &imu_cur) {
    velUpdate(pre, cur, imu_pre, imu_cur);
    posUpdate(pre, cur, imu_cur);
    attUpdate(pre, cur, imu_pre, imu_cur);
}

} // namespace

QuadCore::QuadCore() : has_previous_increment_(false), initialized_(false) {
    setZero(covariance_);
    setZero(dx_);
    setZero(workspace_.phi);
    setZero(workspace_.scratch);
    setZero(workspace_.k);
    setZero(workspace_.ks);
    setZero(workspace_.hp);
    setZero(workspace_.s);
    setZero(workspace_.l);
    previous_increment_.dt = 0.0;
    setZero(previous_increment_.dtheta);
    setZero(previous_increment_.dvel);
    diagnostics_.propagate_count = 0;
    diagnostics_.gnss_pos_updates = 0;
    diagnostics_.gnss_vel_updates = 0;
    diagnostics_.baro_updates = 0;
    diagnostics_.nis_rejected = 0;
    diagnostics_.nonfinite_events = 0;
    diagnostics_.spd_failures = 0;
    diagnostics_.last_nis = 0.0;
}

Status QuadCore::initialize(const QuadConfig &config, const QuadInitState &init) {
    const ImuNoise &noise = config.imunoise;
    for(double tau:noise.correlation_time_s)
        if(!std::isfinite(tau)||tau<0)return kErrArgument;
    if (!(noise.corr_time > 0.0) || !(noise.baro_bias_walk >= 0.0) || !(init.baro_bias_std > 0.0)) {
        return kErrArgument;
    }
    if (!allFiniteVec(noise.gyr_arw) || !allFiniteVec(noise.acc_vrw) || !allFiniteVec(noise.gyrbias_std) ||
        !allFiniteVec(noise.accbias_std) || !allFiniteVec(noise.gyrscale_std) ||
        !allFiniteVec(noise.accscale_std)) {
        return kErrArgument;
    }
    if (!allFiniteVec(config.antlever) || !allFiniteVec(init.pos_std) || !allFiniteVec(init.vel_std) ||
        !allFiniteVec(init.euler_std) || !allFiniteVec(init.gyrbias_std) ||
        !allFiniteVec(init.accbias_std) || !allFiniteVec(init.gyrscale_std) ||
        !allFiniteVec(init.accscale_std)) {
        return kErrArgument;
    }
    for (int i = 0; i < 3; ++i) {
        if (!(init.pos_std(i, 0) > 0.0) || !(init.vel_std(i, 0) > 0.0) || !(init.euler_std(i, 0) > 0.0)) {
            return kErrArgument;
        }
    }
    if (!std::isfinite(config.baro_origin_height_m) || !std::isfinite(init.nominal.baro_bias)) {
        return kErrArgument;
    }

    config_ = config;
    nominal_ = init.nominal;
    nominal_.cbn = quatToMatrix(nominal_.qbn);
    nominal_.euler = matrixToEuler(nominal_.cbn);

    setZero(covariance_);
    for (int i = 0; i < 3; ++i) {
        covariance_(kIdxP + i, kIdxP + i) = init.pos_std(i, 0) * init.pos_std(i, 0);
        covariance_(kIdxV + i, kIdxV + i) = init.vel_std(i, 0) * init.vel_std(i, 0);
        covariance_(kIdxPhi + i, kIdxPhi + i) = init.euler_std(i, 0) * init.euler_std(i, 0);
        covariance_(kIdxBg + i, kIdxBg + i) = init.gyrbias_std(i, 0) * init.gyrbias_std(i, 0);
        covariance_(kIdxBa + i, kIdxBa + i) = init.accbias_std(i, 0) * init.accbias_std(i, 0);
        covariance_(kIdxSg + i, kIdxSg + i) = init.gyrscale_std(i, 0) * init.gyrscale_std(i, 0);
        covariance_(kIdxSa + i, kIdxSa + i) = init.accscale_std(i, 0) * init.accscale_std(i, 0);
    }
    covariance_(kIdxBh, kIdxBh) = init.baro_bias_std * init.baro_bias_std;

    setZero(dx_);
    previous_increment_.dt = 0.0;
    setZero(previous_increment_.dtheta);
    setZero(previous_increment_.dvel);
    has_previous_increment_ = false;
    initialized_ = true;
    return kOk;
}

Status QuadCore::propagate(const ImuIncrement &increment) {
    if (!initialized_) {
        return kErrNotInitialized;
    }
    if (!(increment.dt > kMinDt) || !(increment.dt < kMaxDt) || !allFiniteVec(increment.dtheta) ||
        !allFiniteVec(increment.dvel)) {
        return kErrArgument;
    }

    /* [UPSTREAM] GIEngine::imuCompensate on the incoming increment. */
    ImuIncrement compensated;
    compensated.dt = increment.dt;
    for (int i = 0; i < 3; ++i) {
        const double gs = 1.0 + nominal_.gyrscale(i, 0);
        const double as = 1.0 + nominal_.accscale(i, 0);
        if (std::fabs(gs) < 1.0e-6 || std::fabs(as) < 1.0e-6) {
            return kErrArgument;
        }
        compensated.dtheta(i, 0) = (increment.dtheta(i, 0) - nominal_.gyrbias(i, 0) * increment.dt) / gs;
        compensated.dvel(i, 0) = (increment.dvel(i, 0) - nominal_.accbias(i, 0) * increment.dt) / as;
    }

    const QuadNominal pre = nominal_;
    QuadNominal cur = nominal_;
    insMech(pre, cur, previous_increment_, compensated);

    buildPhi(pre, compensated);
    const Status covariance_status = propagateCovariance(pre, compensated);

    /* [UPSTREAM] EKFPredict also propagates the error state. */
    Vec22 dx_next;
    for (int i = 0; i < kRank22; ++i) {
        double sum = 0.0;
        for (int k = 0; k < kRank22; ++k) {
            sum += workspace_.phi(i, k) * dx_(k, 0);
        }
        dx_next(i, 0) = sum;
    }

    if (covariance_status != kOk) {
        ++diagnostics_.nonfinite_events;
        return covariance_status;
    }

    nominal_ = cur;
    dx_ = dx_next;
    previous_increment_ = compensated;
    has_previous_increment_ = true;
    ++diagnostics_.propagate_count;
    return kOk;
}

/*
 * [UPSTREAM] GIEngine::insPropagation state-transition part, extended by
 * F(21,:) = F(:,21) = 0 (spec 05 §2.2).
 */
void QuadCore::buildPhi(const QuadNominal &pre, const ImuIncrement &compensated) {
    const double dt = compensated.dt;
    const Vec3 accel = compensated.dvel * (1.0 / dt);
    const Vec3 omega = compensated.dtheta * (1.0 / dt);

    double rm = 0.0, rn = 0.0;
    meridianPrimeVerticalRadius(pre.pos(0, 0), rm, rn);
    const double rmh = rm + pre.pos(2, 0);
    const double rnh = rn + pre.pos(2, 0);
    const double lat = pre.pos(0, 0);
    const double g = gravity(pre.pos);

    Vec3 wie_n, wen_n;
    inertialRate(pre.pos, pre.vel, wie_n, wen_n);

    setEye(workspace_.phi);
    auto add_block = [&](int row, int col, const Mat3 &block) {
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                workspace_.phi(row + r, col + c) += block(r, c) * dt;
            }
        }
    };

    const double vn = pre.vel(0, 0);
    const double ve = pre.vel(1, 0);
    const double vd = pre.vel(2, 0);
    const double tan_lat = std::tan(lat);
    const double cos_lat = std::cos(lat);

    Mat3 block;
    setZero(block);
    block(0, 0) = -vd / rmh;
    block(0, 2) = vn / rmh;
    block(1, 0) = ve * tan_lat / rnh;
    block(1, 1) = -(vd + vn * tan_lat) / rnh;
    block(1, 2) = ve / rnh;
    add_block(kIdxP, kIdxP, block);
    add_block(kIdxP, kIdxV, identity3());

    setZero(block);
    block(0, 0) = -2.0 * ve * WGS84_WIE * cos_lat / rmh -
                  (ve * ve) / (rmh * rnh * cos_lat * cos_lat);
    block(0, 2) = vn * vd / (rmh * rmh) - (ve * ve) * tan_lat / (rnh * rnh);
    block(1, 0) = 2.0 * WGS84_WIE * (vn * cos_lat - vd * std::sin(lat)) / rmh +
                  vn * ve / (rmh * rnh * cos_lat * cos_lat);
    block(1, 2) = (ve * vd + vn * ve * tan_lat) / (rnh * rnh);
    block(2, 0) = 2.0 * WGS84_WIE * ve * std::sin(lat) / rmh;
    block(2, 2) = -(ve * ve) / (rnh * rnh) - (vn * vn) / (rmh * rmh) +
                  2.0 * g / (std::sqrt(rm * rn) + pre.pos(2, 0));
    add_block(kIdxV, kIdxP, block);

    setZero(block);
    block(0, 0) = vd / rmh;
    block(0, 1) = -2.0 * (WGS84_WIE * std::sin(lat) + ve * tan_lat / rnh);
    block(0, 2) = vn / rmh;
    block(1, 0) = 2.0 * WGS84_WIE * std::sin(lat) + ve * tan_lat / rnh;
    block(1, 1) = (vd + vn * tan_lat) / rnh;
    block(1, 2) = 2.0 * WGS84_WIE * cos_lat + ve / rnh;
    block(2, 0) = -2.0 * vn / rmh;
    block(2, 1) = -2.0 * (WGS84_WIE * cos_lat + ve / rnh);
    add_block(kIdxV, kIdxV, block);

    add_block(kIdxV, kIdxPhi, skew(matvec(pre.cbn, accel)));
    add_block(kIdxV, kIdxBa, pre.cbn);
    add_block(kIdxV, kIdxSa, matmul3(pre.cbn, diagonalOf(accel)));

    setZero(block);
    block(0, 0) = -WGS84_WIE * std::sin(lat) / rmh;
    block(0, 2) = ve / (rnh * rnh);
    block(1, 2) = -vn / (rmh * rmh);
    block(2, 0) = -WGS84_WIE * cos_lat / rmh - ve / (rmh * rnh * cos_lat * cos_lat);
    block(2, 2) = -ve * tan_lat / (rnh * rnh);
    add_block(kIdxPhi, kIdxP, block);

    setZero(block);
    block(0, 1) = 1.0 / rnh;
    block(1, 0) = -1.0 / rmh;
    block(2, 1) = -tan_lat / rnh;
    add_block(kIdxPhi, kIdxV, block);

    Vec3 omega_in;
    for (int i = 0; i < 3; ++i) {
        omega_in(i, 0) = wie_n(i, 0) + wen_n(i, 0);
    }
    Mat3 neg_skew;
    const Mat3 skew_in = skew(omega_in);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            neg_skew(r, c) = -skew_in(r, c);
        }
    }
    add_block(kIdxPhi, kIdxPhi, neg_skew);

    Mat3 neg_cbn;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            neg_cbn(r, c) = -pre.cbn(r, c);
        }
    }
    add_block(kIdxPhi, kIdxBg, neg_cbn);
    add_block(kIdxPhi, kIdxSg, matmul3(neg_cbn, diagonalOf(omega)));

    const int offset[4]={kIdxBg,kIdxBa,kIdxSg,kIdxSa};
    for(int group=0;group<4;++group) {
        setZero(block);
        const double configured=config_.imunoise.correlation_time_s[group];
        const double tau=configured>0?configured:config_.imunoise.corr_time;
        for(int i=0;i<3;++i)block(i,i)=-1.0/tau;
        add_block(offset[group],offset[group],block);
    }
}

/*
 * [UPSTREAM] GIEngine::insPropagation noise part, extended to 22 states with
 * G(21, 18) = 1 and Qc19(18, 18) = q_bh (spec 05 §2.2).
 *   Q0  = G Qc G^T dt      (block diagonal)
 *   Qd  = (Phi Q0 Phi^T + Q0) / 2
 */
Status QuadCore::propagateCovariance(const QuadNominal &pre, const ImuIncrement &compensated) {
    const double dt = compensated.dt;
    Mat22 &phi = workspace_.phi;
    Mat22 &work = workspace_.scratch;
    const Mat22 &cov = covariance_;

    /* work = Phi * P (column by column). */
    for (int j = 0; j < kRank22; ++j) {
        for (int i = 0; i < kRank22; ++i) {
            double sum = 0.0;
            for (int k = 0; k < kRank22; ++k) {
                sum += phi(i, k) * cov(k, j);
            }
            work(i, j) = sum;
        }
    }

    /* work = work * Phi^T, row by row (row i is read and then overwritten). */
    for (int i = 0; i < kRank22; ++i) {
        double row[kRank22];
        for (int j = 0; j < kRank22; ++j) {
            row[j] = work(i, j);
        }
        for (int k = 0; k < kRank22; ++k) {
            double sum = 0.0;
            for (int j = 0; j < kRank22; ++j) {
                sum += row[j] * phi(k, j);
            }
            work(i, k) = sum;
        }
    }

    const ImuNoise &noise = config_.imunoise;
    const Vec3 vrw_sq = hadamard(noise.acc_vrw, noise.acc_vrw);
    const Vec3 arw_sq = hadamard(noise.gyr_arw, noise.gyr_arw);

    Mat3 blocks[7];
    int offsets[7];
    int count = 0;

    blocks[count] = matmul3(matmul3(pre.cbn, diag(vrw_sq)), transpose3(pre.cbn));
    offsets[count++] = kIdxV;
    blocks[count] = matmul3(matmul3(pre.cbn, diag(arw_sq)), transpose3(pre.cbn));
    offsets[count++] = kIdxPhi;

    double gm[4];
    for(int group=0;group<4;++group)
        gm[group]=2.0/(noise.correlation_time_s[group]>0?noise.correlation_time_s[group]:noise.corr_time);
    blocks[count] = diag(hadamard(noise.gyrbias_std, noise.gyrbias_std) * gm[0]);
    offsets[count++] = kIdxBg;
    blocks[count] = diag(hadamard(noise.accbias_std, noise.accbias_std) * gm[1]);
    offsets[count++] = kIdxBa;
    blocks[count] = diag(hadamard(noise.gyrscale_std, noise.gyrscale_std) * gm[2]);
    offsets[count++] = kIdxSg;
    blocks[count] = diag(hadamard(noise.accscale_std, noise.accscale_std) * gm[3]);
    offsets[count++] = kIdxSa;

    for (int b = 0; b < count; ++b) {
        Mat3 q0 = blocks[b];
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                q0(r, c) *= dt;
            }
        }
        const int offset = offsets[b];

        /* t = Phi[:, offset:offset+3] * q0  (22x3), stored in the gain buffer. */
        for (int i = 0; i < kRank22; ++i) {
            for (int c = 0; c < 3; ++c) {
                double sum = 0.0;
                for (int k = 0; k < 3; ++k) {
                    sum += phi(i, offset + k) * q0(k, c);
                }
                workspace_.k(i, c) = sum;
            }
        }

        /* work += 0.5 * t * Phi[:, offset:offset+3]^T */
        for (int i = 0; i < kRank22; ++i) {
            for (int j = 0; j < kRank22; ++j) {
                double sum = 0.0;
                for (int c = 0; c < 3; ++c) {
                    sum += workspace_.k(i, c) * phi(j, offset + c);
                }
                work(i, j) += 0.5 * sum;
            }
        }

        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                work(offset + r, offset + c) += 0.5 * q0(r, c);
            }
        }
    }

    /* Barometer bias random walk: Phi(21,21) = 1, so Qd(21,21) = q_bh * dt. */
    work(kIdxBh, kIdxBh) += noise.baro_bias_walk * dt;

    symmetrize(work);

    if (!allFinite(work)) {
        return kErrNonFinite;
    }
    for (int i = 0; i < kRank22; ++i) {
        if (!(work(i, i) > 0.0)) {
            return kErrCovarianceDegraded;
        }
    }

    copy(work, covariance_);
    return kOk;
}

Status QuadCore::updateGnssPosition(const GnssPositionObs &obs) {
    if (!initialized_) {
        return kErrNotInitialized;
    }
    if (!allFiniteVec(obs.blh) || !allFiniteVec(obs.std)) {
        return kErrArgument;
    }
    for (int i = 0; i < 3; ++i) {
        if (!(obs.std(i, 0) > 0.0)) {
            return kErrArgument;
        }
    }

    const Vec3 predicted = gnssPositionPrediction(nominal_, config_);
    /* Same linearisation point as the locked upstream gnssUpdate(): the
     * current nominal position, not the measurement position. */
    const Mat3 dr = earthDR(nominal_.pos);
    const Vec3 delta = predicted - obs.blh;
    const Vec3 dz = matvec(dr, delta);

    Mat3x22 h;
    gnssPositionJacobian(nominal_, config_, h);

    Mat3 r;
    setZero(r);
    for (int i = 0; i < 3; ++i) {
        r(i, i) = obs.std(i, 0) * obs.std(i, 0);
    }

    return applyUpdate(3, &dz(0, 0), &h(0, 0), &r(0, 0), 3, kUpdateGnssPosition);
}

Status QuadCore::updateGnssVelocity(const GnssVelocityObs &obs) {
    if (!initialized_) {
        return kErrNotInitialized;
    }
    if (!allFiniteVec(obs.vel_ned) || !allFiniteVec(obs.std) || !allFiniteVec(obs.omega_ib)) {
        return kErrArgument;
    }
    for (int i = 0; i < 3; ++i) {
        if (!(obs.std(i, 0) > 0.0)) {
            return kErrArgument;
        }
    }

    const Vec3 predicted = gnssVelocityPrediction(nominal_, config_, obs.omega_ib);
    const Vec3 dz = predicted - obs.vel_ned;

    Mat3x22 h;
    gnssVelocityJacobian(nominal_, config_, obs.omega_ib, h);

    Mat3 r;
    setZero(r);
    for (int i = 0; i < 3; ++i) {
        r(i, i) = obs.std(i, 0) * obs.std(i, 0);
    }

    return applyUpdate(3, &dz(0, 0), &h(0, 0), &r(0, 0), 3, kUpdateGnssVelocity);
}

Status QuadCore::updateBaro(const BaroObs &obs) {
    if (!initialized_) {
        return kErrNotInitialized;
    }
    if (!std::isfinite(obs.height) || !(obs.std > 0.0)) {
        return kErrArgument;
    }

    const double predicted = baroPrediction(nominal_, config_);
    const double dz = predicted - obs.height;

    Mat<1, 22> h;
    baroJacobian(h);

    const double r = obs.std * obs.std;

    return applyUpdate(1, &dz, &h(0, 0), &r, 1, kUpdateBaro);
}

Status QuadCore::updateHeading(double heading, double variance) {
    if(!initialized_) return kErrNotInitialized;
    if(!std::isfinite(heading)||!std::isfinite(variance)||variance<=0 ||
       std::fabs(std::cos(nominal_.euler(1,0)))<.17) return kErrArgument;
    const double yaw=nominal_.euler(2,0), pitch=nominal_.euler(1,0);
    const double dz=std::remainder(yaw-heading,2*kPi);
    Mat<1,22> h;setZero(h);
    h(0,kIdxPhi)=-std::tan(pitch)*std::cos(yaw);
    h(0,kIdxPhi+1)=-std::tan(pitch)*std::sin(yaw);
    h(0,kIdxPhi+2)=-1;
    return applyUpdate(1,&dz,&h(0,0),&variance,1,kUpdateHeading);
}

/*
 * Common EKF update: S = H P H^T + R, NIS gate, K = P H^T S^-1 through a
 * Cholesky solve, Joseph covariance update, covariance validation, error
 * feedback and reset.  'm' is 1 or 3; the system is padded to 3x3 with the
 * identity so that a single 3x3 Cholesky path is used.
 */
Status QuadCore::applyUpdate(int m, const double *dz, const double *h, const double *r_noise, int dof,
                             UpdateKind kind) {
    if (m != 1 && m != 3) {
        return kErrArgument;
    }
    for (int i = 0; i < m; ++i) {
        if (!std::isfinite(dz[i])) {
            return kErrArgument;
        }
    }

    /* S = H P H^T + R, padded to 3x3. */
    for (int i = 0; i < kRank22; ++i) {
        for (int c = 0; c < m; ++c) {
            double sum = 0.0;
            for (int k = 0; k < kRank22; ++k) {
                sum += covariance_(i, k) * h[c * kRank22 + k];
            }
            workspace_.k(i, c) = sum;
        }
        for (int c = m; c < 3; ++c) {
            workspace_.k(i, c) = 0.0;
        }
    }

    setZero(workspace_.s);
    for (int i = 0; i < m; ++i) {
        for (int c = 0; c < m; ++c) {
            double sum = 0.0;
            for (int k = 0; k < kRank22; ++k) {
                sum += h[i * kRank22 + k] * workspace_.k(k, c);
            }
            workspace_.s(i, c) = sum + r_noise[i * m + c];
        }
    }
    for (int i = m; i < 3; ++i) {
        workspace_.s(i, i) = 1.0;
    }
    symmetrize(workspace_.s);

    if (!allFinite(workspace_.s) || !cholesky(workspace_.s, workspace_.l)) {
        ++diagnostics_.spd_failures;
        return kErrNotPositiveDefinite;
    }

    /* NIS with the same Cholesky factor. */
    double nis = 0.0;
    if (m == 1) {
        nis = dz[0] * dz[0] / workspace_.s(0, 0);
    } else {
        Mat<3, 1> rhs;
        Mat<3, 1> sol;
        for (int i = 0; i < 3; ++i) {
            rhs(i, 0) = (i < m) ? dz[i] : 0.0;
        }
        choleskySolve(workspace_.l, rhs, sol);
        for (int i = 0; i < m; ++i) {
            nis += dz[i] * sol(i, 0);
        }
    }
    diagnostics_.last_nis = nis;

    const double threshold = (dof == 1) ? kNisThresholdDof1 : kNisThresholdDof3;
    if (config_.nis_gate_enable && nis > threshold) {
        ++diagnostics_.nis_rejected;
        return kErrNisRejected;
    }

    /*
     * K = P H^T S^-1.  P H^T is already in workspace_.k; S^-1 is applied
     * through m Cholesky solves of the unit vectors, so no 22x22 inverse and
     * no general matrix inversion is ever formed.
     */
    Mat3 s_inv;
    setZero(s_inv);
    for (int c = 0; c < m; ++c) {
        Mat<3, 1> rhs;
        Mat<3, 1> sol;
        setZero(rhs);
        rhs(c, 0) = 1.0;
        choleskySolve(workspace_.l, rhs, sol);
        for (int r = 0; r < 3; ++r) {
            s_inv(r, c) = sol(r, 0);
        }
    }
    for (int c = 0; c < m; ++c) {
        for (int i = 0; i < kRank22; ++i) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += workspace_.k(i, k) * s_inv(k, c);
            }
            workspace_.ks(i, c) = sum;
        }
    }
    /* The buffers are reused across updates of different dimension: the
     * columns beyond this observation's dimension must not carry stale gains
     * into the K S K^T product. */
    for (int c = m; c < 3; ++c) {
        for (int i = 0; i < kRank22; ++i) {
            workspace_.ks(i, c) = 0.0;
        }
    }

    /* Backup of the previous covariance for failure atomicity. */
    copy(covariance_, workspace_.scratch);

    /* KHP = K * (H P), stored in workspace_.phi. */
    for (int c = 0; c < m; ++c) {
        for (int j = 0; j < kRank22; ++j) {
            double sum = 0.0;
            for (int k = 0; k < kRank22; ++k) {
                sum += h[c * kRank22 + k] * covariance_(k, j);
            }
            workspace_.hp(c, j) = sum;
        }
    }
    for (int i = 0; i < kRank22; ++i) {
        for (int j = 0; j < kRank22; ++j) {
            double sum = 0.0;
            for (int c = 0; c < m; ++c) {
                sum += workspace_.ks(i, c) * workspace_.hp(c, j);
            }
            workspace_.phi(i, j) = sum;
        }
    }

    /* P = P - KHP - (KHP)^T */
    for (int i = 0; i < kRank22; ++i) {
        for (int j = 0; j < kRank22; ++j) {
            covariance_(i, j) -= workspace_.phi(i, j) + workspace_.phi(j, i);
        }
    }

    /* P += K S K^T ; P H^T is no longer needed, so its buffer is reused. */
    for (int i = 0; i < kRank22; ++i) {
        for (int c = 0; c < 3; ++c) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += workspace_.ks(i, k) * workspace_.s(k, c);
            }
            workspace_.k(i, c) = sum;
        }
    }
    for (int i = 0; i < kRank22; ++i) {
        for (int j = 0; j < kRank22; ++j) {
            double sum = 0.0;
            for (int c = 0; c < 3; ++c) {
                sum += workspace_.k(i, c) * workspace_.ks(j, c);
            }
            covariance_(i, j) += sum;
        }
    }

    symmetrize(covariance_);

    if (!allFinite(covariance_) || !cholesky(covariance_,workspace_.phi)) {
        copy(workspace_.scratch, covariance_);
        ++diagnostics_.nonfinite_events;
        return kErrCovarianceDegraded;
    }

    /* dx = K * dz (dx is zero after every feedback, kept for fidelity). */
    Vec22 dx = dx_;
    for (int i = 0; i < kRank22; ++i) {
        double sum = 0.0;
        for (int c = 0; c < m; ++c) {
            sum += workspace_.ks(i, c) * dz[c];
        }
        dx(i, 0) += sum;
    }
    for (int i = 0; i < kRank22; ++i) {
        if (!std::isfinite(dx(i, 0))) {
            copy(workspace_.scratch, covariance_);
            ++diagnostics_.nonfinite_events;
            return kErrNonFinite;
        }
    }

    QuadNominal updated;
    retract(nominal_, dx, updated);
    nominal_ = updated;
    setZero(dx_);

    switch (kind) {
    case kUpdateGnssPosition:
        ++diagnostics_.gnss_pos_updates;
        break;
    case kUpdateGnssVelocity:
        ++diagnostics_.gnss_vel_updates;
        break;
    case kUpdateBaro:
        ++diagnostics_.baro_updates;
        break;
    case kUpdateHeading:
        break;
    }
    return kOk;
}

void QuadCore::copyCovariance(double *out) const {
    for (int i = 0; i < kRank22 * kRank22; ++i) {
        out[i] = covariance_.d[i];
    }
}

Status QuadCore::setCovarianceForTest(const Mat22 &cov) {
    if (!allFinite(cov)) {
        return kErrNonFinite;
    }
    covariance_ = cov;
    return kOk;
}

static void unpackCovarianceUpper(const double (&upper)[kCovarianceUpperCount], Mat22 &out) {
    unsigned index = 0;
    for (int row = 0; row < kRank22; ++row)
        for (int col = row; col < kRank22; ++col)
            out(row, col) = out(col, row) = upper[index++];
}

bool QuadCore::validateInitialCovariance(const double (&upper)[kCovarianceUpperCount]) {
    unpackCovarianceUpper(upper, workspace_.scratch);
    return allFinite(workspace_.scratch) && cholesky(workspace_.scratch, workspace_.phi);
}

Status QuadCore::setInitialCovariance(const double (&upper)[kCovarianceUpperCount],
                                      double heading_variance_floor) {
    unpackCovarianceUpper(upper, workspace_.scratch);
    const int yaw_index = kIdxPhi + 2;
    if (workspace_.scratch(yaw_index, yaw_index) < heading_variance_floor)
        workspace_.scratch(yaw_index, yaw_index) = heading_variance_floor;
    if (!allFinite(workspace_.scratch))
        return kErrNonFinite;
    covariance_ = workspace_.scratch;
    return kOk;
}

Status QuadCore::saveCheckpoint(QuadCheckpoint *out, std::uint32_t fusion_revision,
                                std::uint32_t origin_id, std::uint32_t state_epoch,
                                std::uint64_t sample_time_us, std::uint32_t imu_sequence) const {
    if (out == nullptr) {
        return kErrArgument;
    }
    if (!allFinite(covariance_) || !allFiniteVec(nominal_.pos) || !allFiniteVec(nominal_.vel) ||
        !allFiniteVec(nominal_.gyrbias) || !allFiniteVec(nominal_.accbias) ||
        !allFiniteVec(nominal_.gyrscale) || !allFiniteVec(nominal_.accscale) ||
        !std::isfinite(nominal_.baro_bias)) {
        return kErrNonFinite;
    }

    std::memset(out, 0, sizeof(*out));
    out->magic = kQuadCheckpointMagic;
    out->version = kQuadCheckpointVersion;
    out->flags = 0u;
    out->fusion_revision = fusion_revision;
    out->imu_sequence = imu_sequence;
    out->origin_id = origin_id;
    out->state_epoch = state_epoch;
    out->sample_time_us = sample_time_us;

    out->previous_dt = has_previous_increment_ ? previous_increment_.dt : 0.0;
    for (int i = 0; i < 3; ++i) {
        out->previous_dtheta[i] = previous_increment_.dtheta(i, 0);
        out->previous_dvel[i] = previous_increment_.dvel(i, 0);
    }

    int k = 0;
    for (int i = 0; i < 3; ++i) {
        out->nominal[k++] = nominal_.pos(i, 0);
    }
    for (int i = 0; i < 3; ++i) {
        out->nominal[k++] = nominal_.vel(i, 0);
    }
    out->nominal[k++] = nominal_.qbn.w;
    out->nominal[k++] = nominal_.qbn.x;
    out->nominal[k++] = nominal_.qbn.y;
    out->nominal[k++] = nominal_.qbn.z;
    for (int i = 0; i < 3; ++i) {
        out->nominal[k++] = nominal_.gyrbias(i, 0);
    }
    for (int i = 0; i < 3; ++i) {
        out->nominal[k++] = nominal_.accbias(i, 0);
    }
    for (int i = 0; i < 3; ++i) {
        out->nominal[k++] = nominal_.gyrscale(i, 0);
    }
    for (int i = 0; i < 3; ++i) {
        out->nominal[k++] = nominal_.accscale(i, 0);
    }
    out->nominal[k++] = nominal_.baro_bias;

    int index = 0;
    for (int row = 0; row < kRank22; ++row) {
        for (int col = row; col < kRank22; ++col) {
            out->covariance_upper[index++] = covariance_(row, col);
        }
    }
    return kOk;
}

Status QuadCore::restoreCheckpoint(const QuadCheckpoint &in) {
    if (in.magic != kQuadCheckpointMagic || in.version != kQuadCheckpointVersion) {
        return kErrArgument;
    }

    QuadNominal nominal = nominal_;
    int k = 0;
    for (int i = 0; i < 3; ++i) {
        nominal.pos(i, 0) = in.nominal[k++];
    }
    for (int i = 0; i < 3; ++i) {
        nominal.vel(i, 0) = in.nominal[k++];
    }
    nominal.qbn = Quat{in.nominal[k], in.nominal[k + 1], in.nominal[k + 2], in.nominal[k + 3]};
    k += 4;
    nominal.qbn = quatNormalized(nominal.qbn);
    nominal.cbn = quatToMatrix(nominal.qbn);
    nominal.euler = matrixToEuler(nominal.cbn);
    for (int i = 0; i < 3; ++i) {
        nominal.gyrbias(i, 0) = in.nominal[k++];
    }
    for (int i = 0; i < 3; ++i) {
        nominal.accbias(i, 0) = in.nominal[k++];
    }
    for (int i = 0; i < 3; ++i) {
        nominal.gyrscale(i, 0) = in.nominal[k++];
    }
    for (int i = 0; i < 3; ++i) {
        nominal.accscale(i, 0) = in.nominal[k++];
    }
    nominal.baro_bias = in.nominal[k++];

    Mat22 &covariance = workspace_.scratch;
    unpackCovarianceUpper(in.covariance_upper, covariance);

    if (!allFinite(covariance) || !allFiniteVec(nominal.pos) || !allFiniteVec(nominal.vel) ||
        !std::isfinite(nominal.baro_bias) || !cholesky(covariance,workspace_.phi)) {
        return kErrNonFinite;
    }

    nominal_ = nominal;
    covariance_ = covariance;
    setZero(dx_);
    previous_increment_.dt = in.previous_dt;
    for (int i = 0; i < 3; ++i) {
        previous_increment_.dtheta(i, 0) = in.previous_dtheta[i];
        previous_increment_.dvel(i, 0) = in.previous_dvel[i];
    }
    has_previous_increment_ = (in.previous_dt > 0.0);
    initialized_ = true;
    return kOk;
}

} // namespace kfg
