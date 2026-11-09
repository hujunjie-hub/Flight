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

#endif /* M5_TRAJECTORY_SAFETY_TEST_API_H */
