/*
 * M5-05 bounded Host/SIL time parameterization.
 *
 * Each segment is rebuilt from the same endpoint p/v/a boundary states. Its
 * duration grows on the 200 ms lattice until Bernstein v/a/jerk bounds pass.
 */
#if defined(FLIGHT_M5_HOST_PLANNER)
#include <math.h>
#include <string.h>
#include "trajectory_planning/m5_time_scaler.h"

#define M5_TIME_BIN_S 0.2f
#define M5_TIME_BOUND_TOL 1e-4f

static uint8_t finite3(const float value[3])
{
  return (uint8_t)(isfinite(value[0]) && isfinite(value[1]) && isfinite(value[2]));
}

static uint8_t navigation_valid(const NavigationSnapshot *nav, uint64_t now_us)
{
  const ContractHeader *h;
  if (nav == NULL) return 0u;
  h = &nav->header;
  return (uint8_t)(h->message_type == MSG_NAVIGATION_SNAPSHOT &&
      h->profile_id == PRODUCT_PROFILE_ID &&
      h->schema_major == CONTRACT_SCHEMA_MAJOR &&
      h->schema_minor == CONTRACT_SCHEMA_MINOR &&
      h->frame_id == FRAME_LOCAL_NED &&
      h->boot_id != 0u && h->time_epoch != 0u &&
      h->clock_domain_id == CLOCK_DOMAIN_FCU_MONOTONIC &&
      (h->validity_flags & (VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID)) ==
          (VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID) &&
      h->timestamp_sample_us <= h->timestamp_publish_us &&
      h->timestamp_publish_us <= now_us && h->valid_until_us > now_us &&
      now_us - h->timestamp_sample_us <= 5000u &&
      nav->origin_id != 0u && nav->state_epoch != 0u &&
      finite3(nav->core.position_ned_m));
}

static uint8_t config_and_limits_valid(const ProductGuidanceConfig *config,
                                       const SafetyEnvelope *envelope,
                                       const NavigationSnapshot *nav,
                                       uint64_t now_us,
                                       float velocity_limit[3],
                                       float acceleration_limit[3],
                                       float jerk_limit[3])
{
  unsigned axis;
  const uint32_t required_mask = PRODUCT_ENVELOPE_VALID_VELOCITY |
      PRODUCT_ENVELOPE_VALID_ACCELERATION | PRODUCT_ENVELOPE_VALID_JERK;
  if (config == NULL || envelope == NULL || nav == NULL ||
      config->profile_hash != PRODUCT_PROFILE_ID ||
      config->config_version == 0u || config->parameter_hash == 0u ||
      !guidance_header_ok(&envelope->header, &nav->header,
                          MSG_SAFETY_ENVELOPE, now_us) ||
      (envelope->vehicle_extension.valid_mask & required_mask) != required_mask)
    return 0u;

  for (axis = 0u; axis < 3u; ++axis) {
    float cfg_v = config->payload.max_velocity_ned_mps[axis];
    float cfg_a = config->payload.max_acceleration_ned_mps2[axis];
    float cfg_j = config->payload.max_jerk_ned_mps3[axis];
    float env_v = envelope->vehicle_extension.max_velocity_ned_mps[axis];
    float env_a = envelope->vehicle_extension.max_acceleration_ned_mps2[axis];
    float env_j = envelope->vehicle_extension.max_jerk_ned_mps3[axis];
    if (!isfinite(cfg_v) || !isfinite(cfg_a) || !isfinite(cfg_j) ||
        !isfinite(env_v) || !isfinite(env_a) || !isfinite(env_j) ||
        cfg_v <= 0.0f || cfg_a <= 0.0f || cfg_j <= 0.0f ||
        env_v <= 0.0f || env_a <= 0.0f || env_j <= 0.0f) return 0u;
    velocity_limit[axis] = fminf(cfg_v, env_v);
    acceleration_limit[axis] = fminf(cfg_a, env_a);
    jerk_limit[axis] = fminf(cfg_j, env_j);
  }
  return 1u;
}

static uint8_t validate_astar_path(const M5AstarPath *path)
{
  const M5KinodynamicPrimitive *primitives;
  uint8_t primitive_count = 0u;
  uint32_t total = 0u;
  unsigned i;
  if (path == NULL || path->segment_count == 0u ||
      path->segment_count > M5_ASTAR_MAX_PATH_SEGMENTS ||
      path->map_revision == 0u || path->offset_us[0] != 0u ||
      !isfinite(path->total_cost_s) || path->total_cost_s <= 0.0f)
    return 0u;
  primitives = m5_kinodynamic_primitive_table(&primitive_count);
  if (primitives == NULL || primitive_count != 60u ||
      !m5_kinodynamic_primitive_table_validate()) return 0u;

  for (i = 0u; i <= path->segment_count; ++i) {
    if (!finite3(path->position_ned_m[i]) ||
        !finite3(path->velocity_ned_mps[i]) ||
        !finite3(path->acceleration_ned_mps2[i]) ||
        !isfinite(path->yaw_rad[i]) || !isfinite(path->yaw_rate_radps[i]))
      return 0u;
  }
  for (i = 0u; i < path->segment_count; ++i) {
    uint32_t duration;
    if (path->primitive_index[i] >= primitive_count ||
        path->offset_us[i + 1u] <= path->offset_us[i]) return 0u;
    duration = path->offset_us[i + 1u] - path->offset_us[i];
    if (duration != primitives[path->primitive_index[i]].duration_us ||
        total > GUIDANCE_MAX_DURATION_US - duration) return 0u;
    total += duration;
  }
  return (uint8_t)(path->offset_us[path->segment_count] == total &&
      total <= GUIDANCE_MAX_DURATION_US &&
      fabsf(path->total_cost_s - (float)total * 1e-6f) <= 1e-4f);
}

static uint32_t round_up_time_ticks(float required_s)
{
  float ticks_f = ceilf(required_s / M5_TIME_BIN_S - 1e-6f);
  if (!isfinite(ticks_f) || ticks_f < 1.0f) return 1u;
  if (ticks_f > 31.0f) return 32u;
  return (uint32_t)ticks_f;
}

static uint8_t segment_bounds_ok(const M5AstarPath *path, unsigned segment,
                                 float duration_s, const float v_limit[3],
                                 const float a_limit[3], const float j_limit[3])
{
  unsigned axis, derivative;
  for (axis = 0u; axis < 3u; ++axis) {
    GuidanceQuintic q;
    float lower, upper;
    const float p0 = path->position_ned_m[segment][axis];
    const float v0 = path->velocity_ned_mps[segment][axis];
    const float a0 = path->acceleration_ned_mps2[segment][axis];
    const float p1 = path->position_ned_m[segment + 1u][axis];
    const float v1 = path->velocity_ned_mps[segment + 1u][axis];
    const float a1 = path->acceleration_ned_mps2[segment + 1u][axis];
    const float limits[3] = {v_limit[axis], a_limit[axis], j_limit[axis]};
    guidance_quintic_boundary(p0, v0, a0, p1, v1, a1, duration_s, &q);
    for (derivative = 1u; derivative <= 3u; ++derivative) {
      if (!guidance_quintic_bounds(&q, 0.0f, duration_s,
                                   (uint8_t)derivative, &lower, &upper) ||
          !isfinite(lower) || !isfinite(upper) ||
          lower < -limits[derivative - 1u] - M5_TIME_BOUND_TOL ||
          upper > limits[derivative - 1u] + M5_TIME_BOUND_TOL)
        return 0u;
    }
  }
  return 1u;
}

M5TimeScalerResult m5_time_scaler_build(
    const M5AstarPath *path, const ProductGuidanceConfig *config,
    const SafetyEnvelope *envelope, const NavigationSnapshot *navigation,
    uint64_t now_us, M5TimeScaledPath *output)
{
  M5TimeScaledPath candidate;
  float velocity_limit[3], acceleration_limit[3], jerk_limit[3];
  uint32_t total_duration_us = 0u;
  unsigned segment, axis;
  if (output != NULL) memset(output, 0, sizeof(*output));
  if (output == NULL || path == NULL || config == NULL || envelope == NULL ||
      navigation == NULL) return M5_TIME_SCALER_INVALID_INPUT;
  if (!navigation_valid(navigation, now_us))
    return M5_TIME_SCALER_INPUT_STALE;
  if (!guidance_header_ok(&envelope->header, &navigation->header,
                          MSG_SAFETY_ENVELOPE, now_us))
    return M5_TIME_SCALER_INPUT_STALE;
  if (!config_and_limits_valid(config, envelope, navigation, now_us,
                               velocity_limit, acceleration_limit, jerk_limit) ||
      !validate_astar_path(path))
    return M5_TIME_SCALER_INVALID_INPUT;

  memset(&candidate, 0, sizeof(candidate));
  candidate.path = *path;
  candidate.path.offset_us[0] = 0u;

  for (segment = 0u; segment < path->segment_count; ++segment) {
    float required_s = M5_TIME_BIN_S;
    uint32_t ticks;
    uint32_t accepted_duration_us = 0u;
    uint8_t accepted_attempt = 0u;
    for (axis = 0u; axis < 3u; ++axis) {
      float distance = fabsf(path->position_ned_m[segment + 1u][axis] -
                             path->position_ned_m[segment][axis]);
      float velocity_time, acceleration_time, jerk_time;
      if (!isfinite(distance)) return M5_TIME_SCALER_INVALID_INPUT;
      velocity_time = distance / velocity_limit[axis];
      acceleration_time = sqrtf(distance / acceleration_limit[axis]);
      jerk_time = cbrtf(distance / jerk_limit[axis]);
      required_s = fmaxf(required_s,
          fmaxf(velocity_time, fmaxf(acceleration_time, jerk_time)));
    }
    if (!isfinite(required_s)) return M5_TIME_SCALER_INVALID_INPUT;
    ticks = round_up_time_ticks(required_s);

    for (uint8_t attempt = 1u; attempt <= M5_TIME_SCALER_MAX_ATTEMPTS; ++attempt) {
      uint32_t duration_us;
      float duration_s;
      if (ticks > (GUIDANCE_MAX_DURATION_US / GUIDANCE_TIME_BIN_US))
        return M5_TIME_SCALER_CAPACITY_EXCEEDED;
      duration_us = ticks * GUIDANCE_TIME_BIN_US;
      duration_s = (float)duration_us * 1e-6f;
      if (segment_bounds_ok(path, segment, duration_s, velocity_limit,
                            acceleration_limit, jerk_limit)) {
        accepted_duration_us = duration_us;
        accepted_attempt = attempt;
        break;
      }
      if (attempt == M5_TIME_SCALER_MAX_ATTEMPTS)
        return M5_TIME_SCALER_INFEASIBLE;
      {
        uint32_t next_ticks = round_up_time_ticks(duration_s * 1.25f);
        if (next_ticks <= ticks) next_ticks = ticks + 1u;
        ticks = next_ticks;
      }
    }
    if (accepted_duration_us == 0u ||
        total_duration_us > GUIDANCE_MAX_DURATION_US - accepted_duration_us)
      return M5_TIME_SCALER_CAPACITY_EXCEEDED;
    total_duration_us += accepted_duration_us;
    candidate.attempts[segment] = accepted_attempt;
    candidate.path.offset_us[segment + 1u] = total_duration_us;
  }

  candidate.path.total_cost_s = (float)total_duration_us * 1e-6f;
  *output = candidate;
  return M5_TIME_SCALER_OK;
}

#endif /* FLIGHT_M5_HOST_PLANNER */
