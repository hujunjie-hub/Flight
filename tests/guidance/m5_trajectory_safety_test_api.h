#ifndef M5_TRAJECTORY_SAFETY_TEST_API_H
#define M5_TRAJECTORY_SAFETY_TEST_API_H

#include <stdint.h>

/* Test-only adapter for the private M5-06 Hermite -> Bernstein -> bounds path. */
uint8_t m5_trajectory_safety_test_curve_bounds(
    const float start_position[3],
    const float start_velocity[3],
    const float start_acceleration[3],
    const float end_position[3],
    const float end_velocity[3],
    const float end_acceleration[3],
    float duration_s,
    float u0,
    float u1,
    float position_low[3],
    float position_high[3],
    float *speed_bound,
    float *acceleration_bound);

/* Continuous third-derivative bound over [u0,u1] plus the consumer gamma_2
   tail, per axis. */
uint8_t m5_trajectory_safety_test_curve_jerk(
    const float start_position[3],
    const float start_velocity[3],
    const float start_acceleration[3],
    const float end_position[3],
    const float end_velocity[3],
    const float end_acceleration[3],
    float duration_s,
    float u0,
    float u1,
    float jerk_axis_bound[3]);

/* Separate bounds for the Bernstein original interval, the power-basis
   gamma_2 tail, and the consumer binary32 polynomial. */
uint8_t m5_trajectory_safety_test_curve_jerk_components(
    const float start_position[3],
    const float start_velocity[3],
    const float start_acceleration[3],
    const float end_position[3],
    const float end_velocity[3],
    const float end_acceleration[3],
    float duration_s,
    float u0,
    float u1,
    float bernstein_bound[3],
    float tail_bound[3],
    float consumer_bound[3],
    float total_bound[3]);

uint8_t m5_trajectory_safety_test_curve_eval(
    const float start_position[3],
    const float start_velocity[3],
    const float start_acceleration[3],
    const float end_position[3],
    const float end_velocity[3],
    const float end_acceleration[3],
    float duration_s,
    float u0,
    float u1,
    float local_u,
    float position[3],
    float velocity[3],
    float acceleration[3]);

/* MR-009: bound on |binary32 consumer curve - binary64 validator curve| over
   the sub-interval [u0, u1]. Norms over the three axes. */
uint8_t m5_trajectory_safety_test_curve_envelope(
    const float start_position[3],
    const float start_velocity[3],
    const float start_acceleration[3],
    const float end_position[3],
    const float end_velocity[3],
    const float end_acceleration[3],
    float duration_s,
    float u0,
    float u1,
    float *position_error_m,
    float *velocity_error_mps,
    float *acceleration_error_mps2);

uint8_t m5_trajectory_safety_test_rounding_bounds(
    double value, float *lower, float *upper);
float m5_trajectory_safety_test_norm_lower_bound(
    double x, double y, double z);
float m5_trajectory_safety_test_norm_upper_bound(
    float x, float y, float z);

uint8_t m5_trajectory_safety_test_predict_dynamic_position(
    const float position_ned_m[3],
    const float velocity_ned_mps[3],
    double local_mid_offset_us,
    int64_t sample_offset_us,
    float predicted_position[3],
    float *position_error_m);

uint8_t m5_trajectory_safety_test_point_aabb_distance_lower(
    const float point[3],
    const float center[3],
    float half_extent,
    float point_rounding_error,
    float *distance_lower);

uint8_t m5_trajectory_safety_test_dynamic_relative_sweep_bound(
    float curve_speed_bound,
    float obstacle_speed_bound,
    float velocity_std_mps,
    float relative_speed_bound,
    float relative_acceleration_bound,
    float half_duration_s,
    float *sweep_bound);

/* operation: 0 add, 1 subtract, 2 multiply, 3 divide */
uint8_t m5_trajectory_safety_test_interval_op(
    uint8_t operation,
    double a_low, double a_high,
    double b_low, double b_high,
    double *result_low, double *result_high);

/* S06-108: production locator outputs the active segment, local_us and
   global_us directly; Python does not recompute them. */
uint8_t m5_trajectory_safety_test_active_time(
    uint32_t step_us, uint32_t interval, uint32_t offset_us, uint64_t base_us,
    uint32_t *active_interval, uint64_t *local_us, uint64_t *global_us,
    float *local_seconds);

/* S06-103: drive the real M6 control_sample_reference() with a unit-velocity
   linear reference so its x output is exactly the binary32 time s_c. */
uint8_t m5_trajectory_safety_test_consumer_time(
    uint32_t step_us, uint32_t offset_us, float *seconds);

/* S06-113: use the production local-time conversion and the production C
   gamma_2 predicate with an injected error. mode 0 normal, 1 wrong local time,
   2 +4*gamma_2, 3 wrong segment duration. */
uint8_t m5_trajectory_safety_test_time_conversion(
    uint32_t step_us, uint32_t interval, uint32_t offset_us, uint8_t mode,
    float *seconds, uint8_t *within_gamma2, double *true_seconds,
    double *actual_error);

/* S06-103 extension: arbitrary interval, base time and endpoint states.
   `seconds` is measured separately with a unit-velocity companion reference,
   so the time error and the curve-evaluation error can be checked separately. */
uint8_t m5_trajectory_safety_test_consumer_interval_sample(
    uint32_t step_us, uint32_t interval, uint32_t offset_us, uint64_t base_us,
    const float start_position[3], const float start_velocity[3],
    const float start_acceleration[3], const float end_position[3],
    const float end_velocity[3], const float end_acceleration[3],
    float position[3], float *seconds);

/* S06-103: the local time the real consumer uses for a given time, obtained
   through control_reference_locate_segment() - the same function production
   code calls, so no formula is duplicated in the test. */
uint8_t m5_trajectory_safety_test_consumer_local_seconds(
    uint32_t step_us, uint32_t interval, uint32_t offset_us, uint64_t base_us,
    float *seconds);

/* Actual binary32 coefficients produced by the shared production function. */
uint8_t m5_trajectory_safety_test_consumer_coefficients(
    uint32_t step_us, uint32_t interval, const float start_position[3],
    const float start_velocity[3], const float start_acceleration[3],
    const float end_position[3], const float end_velocity[3],
    const float end_acceleration[3], float coefficients[18]);

/* Full consumer output (position, velocity, acceleration) plus the local time. */
uint8_t m5_trajectory_safety_test_consumer_full_sample(
    uint32_t step_us, uint32_t interval, uint32_t offset_us, uint64_t base_us,
    const float start_position[3], const float start_velocity[3],
    const float start_acceleration[3], const float end_position[3],
    const float end_velocity[3], const float end_acceleration[3],
    float position[3], float velocity[3], float acceleration[3], float *seconds);

/* Active segment for a given time: the real endpoints (18 floats), the
   actual binary32 coefficients (18 floats) and the local time. */
uint8_t m5_trajectory_safety_test_active_segment(
    uint32_t step_us, uint32_t interval, uint32_t offset_us, uint64_t base_us,
    const float start_position[3], const float start_velocity[3],
    const float start_acceleration[3], const float end_position[3],
    const float end_velocity[3], const float end_acceleration[3],
    float endpoints[18], float coefficients[18], float *seconds,
    uint32_t *active_interval);

/* Real product envelope limits (SafetyEnvelope + ProductSafetyEnvelopePayload). */
uint8_t m5_trajectory_safety_test_product_limits(
    float *max_linear_speed_mps, float *max_linear_accel_mps2,
    float max_velocity_ned_mps[3], float max_acceleration_ned_mps2[3],
    float max_jerk_ned_mps3[3]);

/* Extended product identity used by the Python oracles. */
uint8_t m5_trajectory_safety_test_product_envelope(
    float *max_linear_speed_mps, float *max_linear_accel_mps2,
    float max_velocity_ned_mps[3], float max_acceleration_ned_mps2[3],
    float max_jerk_ned_mps3[3], uint32_t *valid_mask,
    uint32_t *safety_envelope_id, uint32_t *config_version,
    uint32_t *parameter_hash, uint32_t *profile_hash, uint32_t *profile_id);

#endif /* M5_TRAJECTORY_SAFETY_TEST_API_H */
