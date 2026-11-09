/*
 * KF-GINS_QUAD earth/rotation helpers (fixed-size, header only).
 *
 * Ported from the locked upstream files src/common/earth.h,
 * src/common/rotation.h and src/common/angle.h
 * (i2Nav-WHU/KF-GINS, commit 8291a93e49de513fe9d21f819500d39082ded611,
 * GPL-3.0).  The mathematical content is unchanged; the Eigen::Matrix3d /
 * Eigen::Quaterniond dependency is replaced by the fixed-size types of
 * quad_math.hpp and the console warning of Rotation::matrix2euler() is
 * removed (firmware must not print).
 */

#ifndef KFGINS_QUAD_EARTH_HPP
#define KFGINS_QUAD_EARTH_HPP

#include <cmath>

#include "quad_math.hpp"

namespace kfg {

/* WGS-84 ellipsoid parameters (unchanged from upstream common/earth.h). */
constexpr double WGS84_WIE = 7.2921151467E-5;
constexpr double WGS84_F = 0.0033528106647474805;
constexpr double WGS84_RA = 6378137.0000000000;
constexpr double WGS84_RB = 6356752.3142451793;
constexpr double WGS84_GM0 = 398600441800000.00;
constexpr double WGS84_E1 = 0.0066943799901413156;
constexpr double WGS84_E2 = 0.0067394967422764341;

constexpr double kPi = 3.14159265358979323846;
constexpr double kD2R = kPi / 180.0;
constexpr double kR2D = 180.0 / kPi;

/* Normal gravity (upstream Earth::gravity). */
inline double gravity(const Vec3 &blh) {
    const double sinphi = std::sin(blh(0, 0));
    const double sin2 = sinphi * sinphi;
    const double sin4 = sin2 * sin2;

    const double gamma_a = 9.7803267715;
    const double gamma_0 = gamma_a * (1 + 0.0052790414 * sin2 + 0.0000232718 * sin4 +
                                      0.0000001262 * sin2 * sin4 + 0.0000000007 * sin4 * sin4);
    const double gamma = gamma_0 - (3.0877e-6 - 4.3e-9 * sin2) * blh(2, 0) +
                         0.72e-12 * blh(2, 0) * blh(2, 0);
    return gamma;
}

/* Meridian radius [0] and prime vertical radius [1] (upstream
 * Earth::meridianPrimeVerticalRadius). */
inline void meridianPrimeVerticalRadius(double lat, double &rm, double &rn) {
    double tmp = std::sin(lat);
    tmp *= tmp;
    tmp = 1 - WGS84_E1 * tmp;
    const double sqrttmp = std::sqrt(tmp);
    rm = WGS84_RA * (1 - WGS84_E1) / (sqrttmp * tmp);
    rn = WGS84_RA / sqrttmp;
}

/* n-frame to e-frame rotation matrix (upstream Earth::cne). */
inline Mat3 cne(const Vec3 &blh) {
    const double sinlat = std::sin(blh(0, 0));
    const double sinlon = std::sin(blh(1, 0));
    const double coslat = std::cos(blh(0, 0));
    const double coslon = std::cos(blh(1, 0));

    Mat3 dcm;
    dcm(0, 0) = -sinlat * coslon;
    dcm(0, 1) = -sinlon;
    dcm(0, 2) = -coslat * coslon;
    dcm(1, 0) = -sinlat * sinlon;
    dcm(1, 1) = coslon;
    dcm(1, 2) = -coslat * sinlon;
    dcm(2, 0) = coslat;
    dcm(2, 1) = 0.0;
    dcm(2, 2) = -sinlat;
    return dcm;
}

struct Quat {
    double w;
    double x;
    double y;
    double z;
};

inline Quat quatIdentity() { return Quat{1.0, 0.0, 0.0, 0.0}; }

inline Quat quatNormalized(const Quat &q) {
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!(n > 0.0)) {
        return quatIdentity();
    }
    const double inv = 1.0 / n;
    return Quat{q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

/* Conjugate; for the unit quaternions used here this equals the inverse. */
inline Quat quatConjugate(const Quat &q) { return Quat{q.w, -q.x, -q.y, -q.z}; }

inline Quat quatMultiply(const Quat &a, const Quat &b) {
    Quat r;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return r;
}

inline Mat3 quatToMatrix(const Quat &qin) {
    const Quat q = quatNormalized(qin);
    const double w = q.w, x = q.x, y = q.y, z = q.z;
    Mat3 m;
    m(0, 0) = 1 - 2 * (y * y + z * z);
    m(0, 1) = 2 * (x * y - w * z);
    m(0, 2) = 2 * (x * z + w * y);
    m(1, 0) = 2 * (x * y + w * z);
    m(1, 1) = 1 - 2 * (x * x + z * z);
    m(1, 2) = 2 * (y * z - w * x);
    m(2, 0) = 2 * (x * z - w * y);
    m(2, 1) = 2 * (y * z + w * x);
    m(2, 2) = 1 - 2 * (x * x + y * y);
    return m;
}

inline Quat matrixToQuat(const Mat3 &m) {
    /* Shepperd's method; equivalent to Eigen::Quaterniond(matrix). */
    const double trace = m(0, 0) + m(1, 1) + m(2, 2);
    Quat q;
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q.w = 0.25 * s;
        q.x = (m(2, 1) - m(1, 2)) / s;
        q.y = (m(0, 2) - m(2, 0)) / s;
        q.z = (m(1, 0) - m(0, 1)) / s;
    } else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
        const double s = std::sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2.0;
        q.w = (m(2, 1) - m(1, 2)) / s;
        q.x = 0.25 * s;
        q.y = (m(0, 1) + m(1, 0)) / s;
        q.z = (m(0, 2) + m(2, 0)) / s;
    } else if (m(1, 1) > m(2, 2)) {
        const double s = std::sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2.0;
        q.w = (m(0, 2) - m(2, 0)) / s;
        q.x = (m(0, 1) + m(1, 0)) / s;
        q.y = 0.25 * s;
        q.z = (m(1, 2) + m(2, 1)) / s;
    } else {
        const double s = std::sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2.0;
        q.w = (m(1, 0) - m(0, 1)) / s;
        q.x = (m(0, 2) + m(2, 0)) / s;
        q.y = (m(1, 2) + m(2, 1)) / s;
        q.z = 0.25 * s;
    }
    return quatNormalized(q);
}

/* Rotation vector -> quaternion (upstream Rotation::rotvec2quaternion). */
inline Quat rotvecToQuat(const Vec3 &rotvec) {
    const double angle = norm(rotvec);
    if (angle < 1.0e-12) {
        return quatIdentity();
    }
    const double half = 0.5 * angle;
    const double s = std::sin(half) / angle;
    return Quat{std::cos(half), rotvec(0, 0) * s, rotvec(1, 0) * s, rotvec(2, 0) * s};
}

/* Quaternion -> rotation vector (upstream Rotation::quaternion2vector). */
inline Vec3 quatToRotvec(const Quat &q) {
    const Quat qn = quatNormalized(q);
    if (qn.w < 0.0) {
        /* keep the shortest rotation, matching Eigen::AngleAxis semantics */
        return quatToRotvec(Quat{-qn.w, -qn.x, -qn.y, -qn.z});
    }
    const double vnorm = std::sqrt(qn.x * qn.x + qn.y * qn.y + qn.z * qn.z);
    Vec3 r;
    if (vnorm < 1.0e-12) {
        r(0, 0) = 0.0;
        r(1, 0) = 0.0;
        r(2, 0) = 0.0;
        return r;
    }
    const double angle = 2.0 * std::atan2(vnorm, qn.w);
    const double s = angle / vnorm;
    r(0, 0) = qn.x * s;
    r(1, 0) = qn.y * s;
    r(2, 0) = qn.z * s;
    return r;
}

/* ZYX Euler (roll, pitch, yaw) from C_b^n (upstream Rotation::matrix2euler).
 * The upstream console warning is removed; at gimbal lock roll is set to 0 as
 * upstream does. */
inline Vec3 matrixToEuler(const Mat3 &dcm) {
    Vec3 euler;
    euler(1, 0) = std::atan(-dcm(2, 0) / std::sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));

    if (dcm(2, 0) <= -0.999) {
        euler(0, 0) = 0.0;
        euler(2, 0) = std::atan2((dcm(1, 2) - dcm(0, 1)), (dcm(0, 2) + dcm(1, 1)));
    } else if (dcm(2, 0) >= 0.999) {
        euler(0, 0) = 0.0;
        euler(2, 0) = kPi + std::atan2((dcm(1, 2) + dcm(0, 1)), (dcm(0, 2) - dcm(1, 1)));
    } else {
        euler(0, 0) = std::atan2(dcm(2, 1), dcm(2, 2));
        euler(2, 0) = std::atan2(dcm(1, 0), dcm(0, 0));
    }

    if (euler(2, 0) < 0.0) {
        euler(2, 0) = 2.0 * kPi + euler(2, 0);
    }
    return euler;
}

/* RPY -> C_b^n, ZYX order (upstream Rotation::euler2matrix). */
inline Mat3 eulerToMatrix(const Vec3 &euler) {
    const double cr = std::cos(euler(0, 0)), sr = std::sin(euler(0, 0));
    const double cp = std::cos(euler(1, 0)), sp = std::sin(euler(1, 0));
    const double cy = std::cos(euler(2, 0)), sy = std::sin(euler(2, 0));

    Mat3 m;
    m(0, 0) = cy * cp;
    m(0, 1) = cy * sp * sr - sy * cr;
    m(0, 2) = cy * sp * cr + sy * sr;
    m(1, 0) = sy * cp;
    m(1, 1) = sy * sp * sr + cy * cr;
    m(1, 2) = sy * sp * cr - cy * sr;
    m(2, 0) = -sp;
    m(2, 1) = cp * sr;
    m(2, 2) = cp * cr;
    return m;
}

inline Quat eulerToQuat(const Vec3 &euler) { return matrixToQuat(eulerToMatrix(euler)); }

/* n-frame orientation at a geodetic position (upstream Earth::qne). */
inline Quat earthQne(const Vec3 &blh) {
    const double coslon = std::cos(blh(1, 0) * 0.5);
    const double sinlon = std::sin(blh(1, 0) * 0.5);
    const double coslat = std::cos(-kPi * 0.25 - blh(0, 0) * 0.5);
    const double sinlat = std::sin(-kPi * 0.25 - blh(0, 0) * 0.5);

    Quat quat;
    quat.w = coslat * coslon;
    quat.x = -sinlat * sinlon;
    quat.y = sinlat * coslon;
    quat.z = coslat * sinlon;
    return quatNormalized(quat);
}

/* Latitude/longitude from the n->e quaternion plus height (upstream
 * Earth::blh). */
inline Vec3 earthBlh(const Quat &qne, double height) {
    Vec3 blh;
    blh(0, 0) = -2.0 * std::atan(qne.y / qne.w) - kPi * 0.5;
    blh(1, 0) = 2.0 * std::atan2(qne.z, qne.w);
    blh(2, 0) = height;
    return blh;
}

/* Local NED metres -> geodetic increments (upstream Earth::DRi). */
inline Mat3 earthDRi(const Vec3 &blh) {
    double rm, rn;
    meridianPrimeVerticalRadius(blh(0, 0), rm, rn);
    Mat3 dri;
    setZero(dri);
    dri(0, 0) = 1.0 / (rm + blh(2, 0));
    dri(1, 1) = 1.0 / ((rn + blh(2, 0)) * std::cos(blh(0, 0)));
    dri(2, 2) = -1.0;
    return dri;
}

/* Geodetic increments -> local NED metres (upstream Earth::DR). */
inline Mat3 earthDR(const Vec3 &blh) {
    double rm, rn;
    meridianPrimeVerticalRadius(blh(0, 0), rm, rn);
    Mat3 dr;
    setZero(dr);
    dr(0, 0) = rm + blh(2, 0);
    dr(1, 1) = (rn + blh(2, 0)) * std::cos(blh(0, 0));
    dr(2, 2) = -1.0;
    return dr;
}

} // namespace kfg

#endif // KFGINS_QUAD_EARTH_HPP
