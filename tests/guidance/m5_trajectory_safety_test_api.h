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

#endif /* M5_TRAJECTORY_SAFETY_TEST_API_H */
