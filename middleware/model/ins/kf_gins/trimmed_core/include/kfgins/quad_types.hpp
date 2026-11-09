/*
 * KF-GINS_QUAD types: state layout, configuration, nominal state and the
 * error-state retraction used by both the filter and the Jacobian tests.
 *
 * State layout is frozen by spec 05 §2.2 and matches the locked upstream
 * StateID order (src/kf-gins/gi_engine.h) with the QUAD extension appended:
 *
 *   delta_x[0..3)  : position error, local NED metres
 *   delta_x[3..6)  : velocity error, NED m/s
 *   delta_x[6..9)  : Phi-angle attitude error, rad
 *   delta_x[9..12) : gyro bias error, rad/s
 *   delta_x[12..15): accelerometer bias error, m/s^2
 *   delta_x[15..18): gyro scale-factor error, dimensionless
 *   delta_x[18..21): accelerometer scale-factor error, dimensionless
 *   delta_x[21]    : barometer height-bias error, m
 *
 *   noise[0..3) VRW, [3..6) ARW, [6..9) BGSTD, [9..12) BASTD,
 *   [12..15) SGSTD, [15..18) SASTD, [18] w_bh
 *
 * These indices are plugin-private (spec 05 §2.1) and must never be exported
 * through the public EstimatorPort.
 */

#ifndef KFGINS_QUAD_TYPES_HPP
#define KFGINS_QUAD_TYPES_HPP

#include <cstdint>

#include "quad_earth.hpp"
#include "quad_math.hpp"

namespace kfg {

/* State indices (0-based, frozen). */
enum StateIndex {
    kIdxP = 0,
    kIdxV = 3,
    kIdxPhi = 6,
    kIdxBg = 9,
    kIdxBa = 12,
    kIdxSg = 15,
    kIdxSa = 18,
    kIdxBh = 21
};

/* Process-noise indices (0-based, frozen). */
enum NoiseIndex {
    kIdxVrw = 0,
    kIdxArw = 3,
    kIdxBgStd = 6,
    kIdxBaStd = 9,
    kIdxSgStd = 12,
    kIdxSaStd = 15,
    kIdxWBh = 18
};

/* NIS chi-square thresholds, 99 % confidence (spec 05 §5.3). */
constexpr double kNisThresholdDof1 = 6.6348966;
constexpr double kNisThresholdDof3 = 11.3448667;
constexpr double kNisThresholdDof6 = 16.8118938;

/* Result codes.  No operation throws; every failure is a return value. */
enum Status : int {
    kOk = 0,
    kErrArgument = 1,
    kErrNotInitialized = 2,
    kErrNonFinite = 3,
    kErrNotPositiveDefinite = 4,
    kErrNisRejected = 5,
    kErrTimeDiscontinuity = 6,
    kErrCovarianceDegraded = 7
};

struct ImuNoise {
    Vec3 gyr_arw;      /* rad/s/sqrt(Hz)  (VRW order in upstream: acc first) */
    Vec3 acc_vrw;      /* m/s^2/sqrt(Hz) */
    Vec3 gyrbias_std;  /* rad/s          */
    Vec3 accbias_std;  /* m/s^2          */
    Vec3 gyrscale_std; /* dimensionless  */
    Vec3 accscale_std; /* dimensionless  */
    double corr_time;  /* s, first-order Gauss-Markov correlation time */
    double baro_bias_walk; /* m/s/sqrt(Hz)-equivalent PSD of b_h random walk */
    /* Project extension; zero preserves the upstream single-time-constant
       regression interface. Order: gyro bias, accel bias, gyro scale, accel scale. */
    double correlation_time_s[4] = {};
};

struct QuadConfig {
    ImuNoise imunoise;
    Vec3 antlever;        /* IMU centre -> GNSS antenna phase centre, body FRD (m) */
    bool nis_gate_enable; /* false only for pristine-upstream regression runs */
    /*
     * Ellipsoid height of the local M3 origin, latched when the origin is
     * established.  The caller computes the barometer innovation against
     * h_baro - h_baro0; this field is only the vertical datum of the nominal
     * geodetic height so that the two terms are not silently mixed.
     */
    double baro_origin_height_m;
};

struct QuadNominal {
    Vec3 pos;   /* geodetic latitude/longitude [rad], ellipsoid height [m] */
    Vec3 vel;   /* local NED [m/s] */
    Mat3 cbn;   /* body FRD -> nav NED */
    Quat qbn;
    Vec3 euler; /* ZYX roll/pitch/yaw [rad] */
    Vec3 gyrbias;
    Vec3 accbias;
    Vec3 gyrscale;
    Vec3 accscale;
    double baro_bias; /* b_h [m] */
};

struct QuadInitState {
    QuadNominal nominal;
    /* One-sigma initial uncertainty per group. */
    Vec3 pos_std;   /* m, local NED (converted through Dr_inv on init) */
    Vec3 vel_std;   /* m/s */
    Vec3 euler_std; /* rad */
    Vec3 gyrbias_std;
    Vec3 accbias_std;
    Vec3 gyrscale_std;
    Vec3 accscale_std;
    double baro_bias_std; /* m */
};

/* IMU increment over one propagation interval (already scaled to seconds). */
struct ImuIncrement {
    double dt;    /* s */
    Vec3 dtheta;  /* rad, body */
    Vec3 dvel;    /* m/s, body (specific force integral) */
};

struct GnssPositionObs {
    Vec3 blh; /* antenna phase centre, geodetic [rad,rad,m] */
    Vec3 std; /* m, local NED */
};

struct GnssVelocityObs {
    Vec3 vel_ned;   /* antenna phase centre velocity, NED [m/s] */
    Vec3 std;       /* m/s */
    Vec3 omega_ib;  /* body angular rate used by the lever-arm term [rad/s] */
};

struct BaroObs {
    double height; /* pressure altitude at sample time [m] */
    double std;    /* m */
};

struct QuadDiagnostics {
    std::uint32_t propagate_count;
    std::uint32_t gnss_pos_updates;
    std::uint32_t gnss_vel_updates;
    std::uint32_t baro_updates;
    std::uint32_t nis_rejected;
    std::uint32_t nonfinite_events;
    std::uint32_t spd_failures;
    double last_nis;
};

/* Diagonal 3x3 from a vector (upstream `.asDiagonal()`). */
inline Mat3 diag(const Vec3 &v) {
    Mat3 m;
    setZero(m);
    m(0, 0) = v(0, 0);
    m(1, 1) = v(1, 0);
    m(2, 2) = v(2, 0);
    return m;
}

/* Element-wise product. */
inline Vec3 hadamard(const Vec3 &a, const Vec3 &b) {
    Vec3 r;
    r(0, 0) = a(0, 0) * b(0, 0);
    r(1, 0) = a(1, 0) * b(1, 0);
    r(2, 0) = a(2, 0) * b(2, 0);
    return r;
}

inline Vec3 vec3(double a, double b, double c) {
    Vec3 v;
    v(0, 0) = a;
    v(1, 0) = b;
    v(2, 0) = c;
    return v;
}

/*
 * Error-state retraction, identical to the feedback rule of the locked
 * upstream gi_engine.cpp::stateFeedback():
 *   position : pos -= Dr_inv * delta_p
 *   velocity : vel -= delta_v
 *   attitude : q   = Exp(delta_phi) * q      (left multiplication)
 *   biases   : b   += delta_b, s += delta_s
 *   baro bias: b_h += delta_bh
 *
 * This is the single definition used by the filter and by the finite
 * difference Jacobian verification; tests must not re-implement it.
 */
inline void retract(const QuadNominal &in, const Vec22 &dx, QuadNominal &out) {
    QuadNominal nominal = in;

    const Vec3 dp = vec3(dx(kIdxP + 0, 0), dx(kIdxP + 1, 0), dx(kIdxP + 2, 0));
    const Vec3 dv = vec3(dx(kIdxV + 0, 0), dx(kIdxV + 1, 0), dx(kIdxV + 2, 0));
    const Vec3 dphi = vec3(dx(kIdxPhi + 0, 0), dx(kIdxPhi + 1, 0), dx(kIdxPhi + 2, 0));

    const Mat3 dri = earthDRi(nominal.pos);
    const Vec3 dllh = matvec(dri, dp);
    nominal.pos(0, 0) -= dllh(0, 0);
    nominal.pos(1, 0) -= dllh(1, 0);
    nominal.pos(2, 0) -= dllh(2, 0);

    nominal.vel(0, 0) -= dv(0, 0);
    nominal.vel(1, 0) -= dv(1, 0);
    nominal.vel(2, 0) -= dv(2, 0);

    const Quat dq = rotvecToQuat(dphi);
    nominal.qbn = quatNormalized(quatMultiply(dq, nominal.qbn));
    nominal.cbn = quatToMatrix(nominal.qbn);
    nominal.euler = matrixToEuler(nominal.cbn);

    for (int i = 0; i < 3; ++i) {
        nominal.gyrbias(i, 0) += dx(kIdxBg + i, 0);
        nominal.accbias(i, 0) += dx(kIdxBa + i, 0);
        nominal.gyrscale(i, 0) += dx(kIdxSg + i, 0);
        nominal.accscale(i, 0) += dx(kIdxSa + i, 0);
    }
    nominal.baro_bias += dx(kIdxBh, 0);

    out = nominal;
}

} // namespace kfg

#endif // KFGINS_QUAD_TYPES_HPP
