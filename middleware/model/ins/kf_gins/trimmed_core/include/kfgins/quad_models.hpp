/*
 * KF-GINS_QUAD observation models (prediction + analytic Jacobian).
 *
 * Single source of truth: the filter calls these functions for its update and
 * the host Jacobian test calls the very same functions inside a finite
 * difference check, so the analytic H cannot silently diverge from the model.
 *
 * Residual convention (spec 05 §2.3): innovation = predicted - measured,
 * dx = K * innovation, and the error-state retraction of quad_types.hpp is
 * used unchanged.
 */

#ifndef KFGINS_QUAD_MODELS_HPP
#define KFGINS_QUAD_MODELS_HPP

#include "quad_math.hpp"
#include "quad_types.hpp"

namespace kfg {

/* Earth rotation + transport rate at the nominal state (nav frame). */
void inertialRate(const Vec3 &blh, const Vec3 &vel_ned, Vec3 &wie_n, Vec3 &wen_n);

/* Antenna phase-centre geodetic position: p_ant = p_imu + Dr_inv * C * lever. */
Vec3 gnssPositionPrediction(const QuadNominal &nominal, const QuadConfig &config);

/* Jacobian of the local-NED position residual (3x22). */
void gnssPositionJacobian(const QuadNominal &nominal, const QuadConfig &config, Mat3x22 &h);

/*
 * Antenna velocity:
 *   omega_true = (omega_meas - b_g) / (1 + s_g)
 *   v_ant = v + C (omega_true x lever) - omega_in x (C * lever)
 * (spec 05 §3.2, including the navigation-frame rotation term).  The body
 * rate is passed raw; the bias/scale compensation happens inside the model so
 * that the bias and scale-factor Jacobian columns are well defined.
 */
Vec3 gnssVelocityPrediction(const QuadNominal &nominal, const QuadConfig &config, const Vec3 &omega_meas);

/* Jacobian of the antenna velocity residual (3x22), spec 05 §7. */
void gnssVelocityJacobian(const QuadNominal &nominal, const QuadConfig &config, const Vec3 &omega_meas,
                          Mat3x22 &h);

/* Barometer relative height prediction: h_b = (h - h_origin) + b_h. */
double baroPrediction(const QuadNominal &nominal, const QuadConfig &config);

/* Scalar barometer Jacobian H_b[0,2] = -1, H_b[0,21] = -1. */
void baroJacobian(Mat<1, 22> &h);

} // namespace kfg

#endif // KFGINS_QUAD_MODELS_HPP
