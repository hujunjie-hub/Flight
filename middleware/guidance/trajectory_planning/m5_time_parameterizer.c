/*
 * M5-04 bounded trajectory time parameterizer.
 *
 * Segment duration is selected on the 200 ms lattice. Every candidate quintic
 * is checked with outward-rounded Bernstein bounds for velocity, acceleration
 * and jerk; no sampled-point check is treated as a proof.
 */
#if defined(FLIGHT_M5_HOST_PLANNER)

#include <math.h>
#include <string.h>
#include "trajectory_planning/m5_time_parameterizer.h"

#define M5_TIME_TWO_PI 6.28318530717958647692f
#define M5_TIME_BIN_S ((double)M5_TIME_PARAMETERIZER_BIN_US * 1e-6)
#define M5_TIME_EPS 1e-5f

typedef struct {
  float velocity[3];
  float acceleration[3];
  float jerk[3];
  float max_linear_velocity;
  float max_linear_acceleration;
  float max_yaw_rate;
} M5TimeLimits;

static uint8_t finite3(const float value[3])
{
  return (uint8_t)(isfinite(value[0]) && isfinite(value[1]) &&
                   isfinite(value[2]));
}

static float absmax(float lower, float upper)
{
  return fmaxf(fabsf(lower), fabsf(upper));
}

static uint8_t quintic_finite(const GuidanceQuintic *q)
{
  unsigned i;
  if (q == NULL) return 0u;
  for (i = 0u; i < 6u; ++i)
    if (!isfinite(q->c[i])) return 0u;
  return 1u;
}

static uint8_t duration_to_bin_us(double required_s, uint32_t *duration_us)
{
  double bins;
  if (duration_us == NULL || !isfinite(required_s) || required_s <= 0.0)
    return 0u;
  bins = ceil(required_s / M5_TIME_BIN_S - 1e-12);
  if (!isfinite(bins) || bins < 1.0 ||
      bins > (double)GUIDANCE_MAX_TIME_BINS) return 0u;
  *duration_us = (uint32_t)bins * M5_TIME_PARAMETERIZER_BIN_US;
  return 1u;
}

static M5TimeParameterizerResult validate_inputs(
    const M5AstarPath *path, const ProductGuidanceConfig *config,
    const SafetyEnvelope *envelope, const NavigationSnapshot *navigation,
    uint64_t now_us, M5TimeLimits *limits, uint32_t *source_duration_us)
{
  const ProductGuidanceConfigPayload *cfg;
  const ProductSafetyEnvelopePayload *safe;
  const M5KinodynamicPrimitive *primitives;
  uint8_t primitive_count = 0u;
  uint32_t total = 0u;
  unsigned axis, knot, segment;

  if (path == NULL || config == NULL || envelope == NULL ||
      navigation == NULL || limits == NULL || source_duration_us == NULL ||
      now_us == 0u) return M5_TIME_PARAMETERIZER_INVALID_INPUT;
  if (path->segment_count == 0u) return M5_TIME_PARAMETERIZER_INVALID_INPUT;
  if (path->segment_count > M5_TIME_PARAMETERIZER_MAX_SEGMENTS)
    return M5_TIME_PARAMETERIZER_CAPACITY_EXCEEDED;
  if (path->map_revision == 0u || path->offset_us[0] != 0u ||
      !isfinite(path->total_cost_s) || path->total_cost_s <= 0.0f)
    return M5_TIME_PARAMETERIZER_INVALID_INPUT;

  if (navigation->header.message_type != MSG_NAVIGATION_SNAPSHOT ||
      navigation->header.profile_id != PRODUCT_PROFILE_ID ||
      navigation->header.boot_id == 0u || navigation->header.time_epoch == 0u ||
      navigation->header.timestamp_sample_us > now_us ||
      now_us - navigation->header.timestamp_sample_us > 5000u ||
      navigation->header.valid_until_us <= now_us ||
      (navigation->header.validity_flags &
       (VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID)) !=
       (VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID) ||
      !guidance_header_ok(&envelope->header, &navigation->header,
                          MSG_SAFETY_ENVELOPE, now_us))
    return M5_TIME_PARAMETERIZER_INPUT_STALE;

  if (config->profile_hash != PRODUCT_PROFILE_ID ||
      config->config_version == 0u || config->parameter_hash == 0u ||
      (config->payload.obstacle_source_mode != 1u &&
       config->payload.obstacle_source_mode != 2u))
    return M5_TIME_PARAMETERIZER_INVALID_INPUT;

  cfg = &config->payload;
  safe = &envelope->vehicle_extension;
  if ((safe->valid_mask & 0x7u) != 0x7u ||
      !isfinite(envelope->max_linear_speed_mps) ||
      envelope->max_linear_speed_mps <= 0.0f ||
      !isfinite(envelope->max_linear_accel_mps2) ||
      envelope->max_linear_accel_mps2 <= 0.0f ||
      !isfinite(envelope->max_yaw_rate_radps) ||
      envelope->max_yaw_rate_radps <= 0.0f ||
      !isfinite(envelope->max_angular_rate_radps) ||
      envelope->max_angular_rate_radps <= 0.0f)
    return M5_TIME_PARAMETERIZER_INVALID_INPUT;

  limits->max_linear_velocity = envelope->max_linear_speed_mps;
  limits->max_linear_acceleration = envelope->max_linear_accel_mps2;
  limits->max_yaw_rate = fminf(envelope->max_yaw_rate_radps,
                               envelope->max_angular_rate_radps);
  for (axis = 0u; axis < 3u; ++axis) {
    if (!isfinite(cfg->max_velocity_ned_mps[axis]) ||
        cfg->max_velocity_ned_mps[axis] <= 0.0f ||
        !isfinite(cfg->max_acceleration_ned_mps2[axis]) ||
        cfg->max_acceleration_ned_mps2[axis] <= 0.0f ||
        !isfinite(cfg->max_jerk_ned_mps3[axis]) ||
        cfg->max_jerk_ned_mps3[axis] <= 0.0f ||
        !isfinite(safe->max_velocity_ned_mps[axis]) ||
        safe->max_velocity_ned_mps[axis] <= 0.0f ||
        !isfinite(safe->max_acceleration_ned_mps2[axis]) ||
        safe->max_acceleration_ned_mps2[axis] <= 0.0f ||
        !isfinite(safe->max_jerk_ned_mps3[axis]) ||
        safe->max_jerk_ned_mps3[axis] <= 0.0f)
      return M5_TIME_PARAMETERIZER_INVALID_INPUT;
    limits->velocity[axis] = fminf(cfg->max_velocity_ned_mps[axis],
                                   safe->max_velocity_ned_mps[axis]);
    limits->acceleration[axis] =
        fminf(cfg->max_acceleration_ned_mps2[axis],
              safe->max_acceleration_ned_mps2[axis]);
    limits->jerk[axis] = fminf(cfg->max_jerk_ned_mps3[axis],
                               safe->max_jerk_ned_mps3[axis]);
  }

  primitives = m5_kinodynamic_primitive_table(&primitive_count);
  if (primitives == NULL || primitive_count != 60u ||
      !m5_kinodynamic_primitive_table_validate())
    return M5_TIME_PARAMETERIZER_INVALID_INPUT;

  for (knot = 0u; knot <= path->segment_count; ++knot) {
    if (!finite3(path->position_ned_m[knot]) ||
        !finite3(path->velocity_ned_mps[knot]) ||
        !finite3(path->acceleration_ned_mps2[knot]) ||
        !isfinite(path->yaw_rad[knot]) ||
        !isfinite(path->yaw_rate_radps[knot]))
      return M5_TIME_PARAMETERIZER_INVALID_INPUT;
  }
  for (segment = 0u; segment < path->segment_count; ++segment) {
    uint32_t duration;
    if (path->offset_us[segment + 1u] <= path->offset_us[segment])
      return M5_TIME_PARAMETERIZER_INVALID_INPUT;
    duration = path->offset_us[segment + 1u] - path->offset_us[segment];
    if (duration < M5_TIME_PARAMETERIZER_BIN_US ||
        duration % M5_TIME_PARAMETERIZER_BIN_US != 0u ||
        duration > M5_TIME_PARAMETERIZER_MAX_TOTAL_US ||
        total > M5_TIME_PARAMETERIZER_MAX_TOTAL_US - duration ||
        path->primitive_index[segment] >= primitive_count)
      return M5_TIME_PARAMETERIZER_CAPACITY_EXCEEDED;
    if (primitives[path->primitive_index[segment]].duration_us != duration)
      return M5_TIME_PARAMETERIZER_INVALID_INPUT;
    total += duration;
  }
  if (path->offset_us[path->segment_count] != total ||
      total > M5_TIME_PARAMETERIZER_MAX_TOTAL_US ||
      fabsf(path->total_cost_s - (float)total * 1e-6f) > 1e-4f)
    return M5_TIME_PARAMETERIZER_INVALID_INPUT;
  *source_duration_us = total;
  return M5_TIME_PARAMETERIZER_OK;
}

static double initial_duration_s(const M5AstarPath *path, unsigned segment,
                                 const M5TimeLimits *limits)
{
  double required = M5_TIME_BIN_S;
  double distance_squared = 0.0;
  unsigned axis;
  for (axis = 0u; axis < 3u; ++axis) {
    double distance = fabs((double)path->position_ned_m[segment + 1u][axis] -
                          path->position_ned_m[segment][axis]);
    double velocity_time = distance / limits->velocity[axis];
    double acceleration_time = sqrt(distance / limits->acceleration[axis]);
    double jerk_time = cbrt(distance / limits->jerk[axis]);
    distance_squared += distance * distance;
    required = fmax(required, velocity_time);
    required = fmax(required, acceleration_time);
    required = fmax(required, jerk_time);
  }
  {
    double distance = sqrt(distance_squared);
    required = fmax(required, distance / limits->max_linear_velocity);
    required = fmax(required,
        sqrt(distance / limits->max_linear_acceleration));
  }
  {
    double yaw_delta = fabs((double)remainderf(
        path->yaw_rad[segment + 1u] - path->yaw_rad[segment],
        M5_TIME_TWO_PI));
    required = fmax(required, yaw_delta / limits->max_yaw_rate);
  }
  return required;
}

static uint8_t segment_bounds_ok(const GuidanceQuintic position[3],
                                 const GuidanceQuintic *yaw, float duration_s,
                                 const M5TimeLimits *limits)
{
  float velocity_squared = 0.0f, acceleration_squared = 0.0f;
  unsigned axis, derivative;
  for (axis = 0u; axis < 3u; ++axis) {
    for (derivative = 1u; derivative <= 3u; ++derivative) {
      float lower, upper, bound;
      float limit = (derivative == 1u) ? limits->velocity[axis] :
                    (derivative == 2u) ? limits->acceleration[axis] :
                                         limits->jerk[axis];
      if (!guidance_quintic_bounds(&position[axis], 0.0f, duration_s,
                                   (uint8_t)derivative, &lower, &upper))
        return 0u;
      bound = absmax(lower, upper);
      if (!isfinite(bound) || bound > limit + M5_TIME_EPS) return 0u;
      if (derivative == 1u) velocity_squared += bound * bound;
      if (derivative == 2u) acceleration_squared += bound * bound;
    }
  }
  if (sqrtf(velocity_squared) >
          limits->max_linear_velocity + M5_TIME_EPS ||
      sqrtf(acceleration_squared) >
          limits->max_linear_acceleration + M5_TIME_EPS)
    return 0u;
  {
    float lower, upper;
    if (!guidance_quintic_bounds(yaw, 0.0f, duration_s, 1u, &lower, &upper) ||
        absmax(lower, upper) > limits->max_yaw_rate + M5_TIME_EPS)
      return 0u;
  }
  return 1u;
}

static uint8_t build_segment(const M5AstarPath *path, unsigned segment,
                             uint32_t duration_us, float yaw_start,
                             const M5TimeLimits *limits,
                             GuidanceQuintic position[3],
                             GuidanceQuintic *yaw, float *yaw_end)
{
  float duration_s = (float)duration_us * 1e-6f;
  float delta = remainderf(path->yaw_rad[segment + 1u] -
                           path->yaw_rad[segment], M5_TIME_TWO_PI);
  unsigned axis;
  if (!isfinite(duration_s) || duration_s <= 0.0f || !isfinite(delta))
    return 0u;
  for (axis = 0u; axis < 3u; ++axis) {
    guidance_quintic_boundary(
        path->position_ned_m[segment][axis],
        path->velocity_ned_mps[segment][axis],
        path->acceleration_ned_mps2[segment][axis],
        path->position_ned_m[segment + 1u][axis],
        path->velocity_ned_mps[segment + 1u][axis],
        path->acceleration_ned_mps2[segment + 1u][axis],
        duration_s, &position[axis]);
    if (!quintic_finite(&position[axis])) return 0u;
  }
  *yaw_end = yaw_start + delta;
  guidance_quintic_boundary(yaw_start, path->yaw_rate_radps[segment], 0.0f,
      *yaw_end, path->yaw_rate_radps[segment + 1u], 0.0f, duration_s, yaw);
  return (uint8_t)(quintic_finite(yaw) &&
      segment_bounds_ok(position, yaw, duration_s, limits));
}

M5TimeParameterizerResult m5_time_parameterizer_build(
    const M5AstarPath *path, const ProductGuidanceConfig *config,
    const SafetyEnvelope *envelope, const NavigationSnapshot *navigation,
    uint64_t now_us, M5TimedTrajectory *output)
{
  M5TimedTrajectory candidate;
  M5TimeLimits limits;
  uint32_t source_duration_us = 0u;
  uint32_t total_duration_us = 0u;
  float yaw_start;
  M5TimeParameterizerResult result;
  unsigned segment;

  if (output != NULL) memset(output, 0, sizeof(*output));
  if (output == NULL) return M5_TIME_PARAMETERIZER_INVALID_INPUT;
  result = validate_inputs(path, config, envelope, navigation, now_us,
                           &limits, &source_duration_us);
  if (result != M5_TIME_PARAMETERIZER_OK) return result;

  (void)source_duration_us; /* Input offsets prove this is a complete A* path. */
  memset(&candidate, 0, sizeof(candidate));
  candidate.segment_count = path->segment_count;
  candidate.map_revision = path->map_revision;
  yaw_start = path->yaw_rad[0];

  for (segment = 0u; segment < path->segment_count; ++segment) {
    double required_s = initial_duration_s(path, segment, &limits);
    uint32_t duration_us = 0u;
    uint8_t accepted = 0u;
    unsigned attempt;

    if (!duration_to_bin_us(required_s, &duration_us))
      return M5_TIME_PARAMETERIZER_NO_FEASIBLE_DURATION;

    for (attempt = 0u; attempt < M5_TIME_PARAMETERIZER_MAX_ATTEMPTS; ++attempt) {
      float yaw_end = yaw_start;
      if (duration_us > M5_TIME_PARAMETERIZER_MAX_TOTAL_US ||
          total_duration_us > M5_TIME_PARAMETERIZER_MAX_TOTAL_US - duration_us)
        return M5_TIME_PARAMETERIZER_CAPACITY_EXCEEDED;
      if (build_segment(path, segment, duration_us, yaw_start, &limits,
                        candidate.position[segment], &candidate.yaw[segment],
                        &yaw_end)) {
        candidate.segment_duration_us[segment] = duration_us;
        total_duration_us += duration_us;
        yaw_start = yaw_end;
        accepted = 1u;
        break;
      }
      {
        double next_required = (double)duration_us * 1e-6 * 1.25;
        uint32_t next_duration_us;
        if (!duration_to_bin_us(next_required, &next_duration_us) ||
            next_duration_us <= duration_us) break;
        duration_us = next_duration_us;
      }
    }
    if (accepted == 0u) return M5_TIME_PARAMETERIZER_NO_FEASIBLE_DURATION;
  }

  candidate.total_duration_us = total_duration_us;
  *output = candidate;
  return M5_TIME_PARAMETERIZER_OK;
}

#endif /* FLIGHT_M5_HOST_PLANNER */
