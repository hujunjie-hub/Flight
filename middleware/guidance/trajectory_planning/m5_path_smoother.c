/*
 * M5-05 bounded Host/SIL minimum-jerk path smoother.
 *
 * Consumes the exact M5-04 timed quintic candidate. It changes only position
 * quintics; yaw and segment timing remain those produced by M5-04. Any result,
 * including fallback, remains a candidate for independent M5-06 validation.
 */
#if defined(FLIGHT_M5_HOST_PLANNER)
#include <math.h>
#include <string.h>
#include "trajectory_planning/m5_path_smoother.h"

#define M5_SMOOTHER_TWO_PI 6.28318530717958647692f
#define M5_SMOOTHER_POSITION_TOL 1e-3f
#define M5_SMOOTHER_VELOCITY_TOL 1e-3f
#define M5_SMOOTHER_ACCELERATION_TOL 1e-2f
#define M5_SMOOTHER_YAW_TOL 1e-3f

static uint8_t finite3(const float value[3])
{
  return (uint8_t)(isfinite(value[0]) && isfinite(value[1]) && isfinite(value[2]));
}

static uint8_t quintic_finite(const GuidanceQuintic *q)
{
  unsigned i;
  if (q == NULL) return 0u;
  for (i = 0u; i < 6u; ++i)
    if (!isfinite(q->c[i])) return 0u;
  return 1u;
}

static uint8_t near_value(float actual, float expected, float tolerance)
{
  return (uint8_t)(isfinite(actual) && isfinite(expected) &&
                   fabsf(actual - expected) <= tolerance);
}

static M5SmootherResult validate_path(const M5AstarPath *path,
                                      const M5TimedTrajectory *timed,
                                      const M5LocalMap *map,
                                      const NavigationSnapshot *navigation,
                                      uint64_t now_us,
                                      uint32_t *total_duration_us)
{
  uint8_t primitive_count = 0u;
  const M5KinodynamicPrimitive *primitives;
  uint32_t source_total = 0u, timed_total = 0u;
  float yaw_start;
  unsigned i, axis;

  if (path == NULL || timed == NULL || map == NULL || navigation == NULL ||
      total_duration_us == NULL || now_us == 0u)
    return M5_SMOOTHER_INVALID_INPUT;
  if (path->segment_count == 0u || timed->segment_count == 0u)
    return M5_SMOOTHER_INVALID_INPUT;
  if (path->segment_count > M5_SMOOTHER_MAX_SEGMENTS ||
      timed->segment_count > M5_SMOOTHER_MAX_SEGMENTS)
    return M5_SMOOTHER_CAPACITY_EXCEEDED;
  if (path->segment_count != timed->segment_count ||
      path->map_revision == 0u || path->map_revision != timed->map_revision ||
      path->map_revision != map->map_revision ||
      !m5_local_map_is_ready(map, navigation, now_us))
    return M5_SMOOTHER_INPUT_STALE;
  if (path->offset_us[0] != 0u || !isfinite(path->total_cost_s) ||
      path->total_cost_s <= 0.0f || timed->total_duration_us == 0u ||
      timed->total_duration_us > GUIDANCE_MAX_DURATION_US)
    return M5_SMOOTHER_INVALID_INPUT;

  primitives = m5_kinodynamic_primitive_table(&primitive_count);
  if (primitives == NULL || primitive_count != 60u ||
      !m5_kinodynamic_primitive_table_validate())
    return M5_SMOOTHER_INVALID_INPUT;

  for (i = 0u; i <= path->segment_count; ++i) {
    uint16_t voxel;
    uint8_t cell_state;
    if (!finite3(path->position_ned_m[i]) ||
        !finite3(path->velocity_ned_mps[i]) ||
        !finite3(path->acceleration_ned_mps2[i]) ||
        !isfinite(path->yaw_rad[i]) || !isfinite(path->yaw_rate_radps[i]))
      return M5_SMOOTHER_INVALID_INPUT;
    if (!m5_local_map_world_to_voxel(map, path->position_ned_m[i], &voxel) ||
        !m5_local_map_voxel_state(map, voxel, now_us, &cell_state) ||
        cell_state != M5_MAP_CELL_FREE_CONFIRMED)
      return M5_SMOOTHER_INPUT_STALE;
  }

  for (i = 0u; i < path->segment_count; ++i) {
    uint32_t source_duration, duration_us;
    float duration_s, yaw_end;
    if (path->offset_us[i + 1u] <= path->offset_us[i])
      return M5_SMOOTHER_INVALID_INPUT;
    source_duration = path->offset_us[i + 1u] - path->offset_us[i];
    if (source_duration < GUIDANCE_TIME_BIN_US ||
        source_duration % GUIDANCE_TIME_BIN_US != 0u ||
        source_duration > GUIDANCE_MAX_DURATION_US ||
        source_total > GUIDANCE_MAX_DURATION_US - source_duration ||
        path->primitive_index[i] >= primitive_count)
      return M5_SMOOTHER_CAPACITY_EXCEEDED;
    if (primitives[path->primitive_index[i]].duration_us != source_duration)
      return M5_SMOOTHER_INVALID_INPUT;
    source_total += source_duration;

    duration_us = timed->segment_duration_us[i];
    if (duration_us < GUIDANCE_TIME_BIN_US ||
        duration_us % GUIDANCE_TIME_BIN_US != 0u ||
        duration_us > GUIDANCE_MAX_DURATION_US ||
        timed_total > GUIDANCE_MAX_DURATION_US - duration_us)
      return M5_SMOOTHER_CAPACITY_EXCEEDED;
    duration_s = (float)duration_us * 1e-6f;
    for (axis = 0u; axis < 3u; ++axis) {
      const GuidanceQuintic *q = &timed->position[i][axis];
      if (!quintic_finite(q) ||
          !near_value(guidance_quintic_eval(q, 0.0f, 0u),
                      path->position_ned_m[i][axis], M5_SMOOTHER_POSITION_TOL) ||
          !near_value(guidance_quintic_eval(q, 0.0f, 1u),
                      path->velocity_ned_mps[i][axis], M5_SMOOTHER_VELOCITY_TOL) ||
          !near_value(guidance_quintic_eval(q, 0.0f, 2u),
                      path->acceleration_ned_mps2[i][axis], M5_SMOOTHER_ACCELERATION_TOL) ||
          !near_value(guidance_quintic_eval(q, duration_s, 0u),
                      path->position_ned_m[i + 1u][axis], M5_SMOOTHER_POSITION_TOL) ||
          !near_value(guidance_quintic_eval(q, duration_s, 1u),
                      path->velocity_ned_mps[i + 1u][axis], M5_SMOOTHER_VELOCITY_TOL) ||
          !near_value(guidance_quintic_eval(q, duration_s, 2u),
                      path->acceleration_ned_mps2[i + 1u][axis],
                      M5_SMOOTHER_ACCELERATION_TOL))
        return M5_SMOOTHER_INVALID_INPUT;
    }

    yaw_end = yaw_start + remainderf(path->yaw_rad[i + 1u] -
                                     path->yaw_rad[i], M5_SMOOTHER_TWO_PI);
    if (!quintic_finite(&timed->yaw[i]) ||
        !near_value(guidance_quintic_eval(&timed->yaw[i], 0.0f, 0u),
                    yaw_start, M5_SMOOTHER_YAW_TOL) ||
        !near_value(guidance_quintic_eval(&timed->yaw[i], 0.0f, 1u),
                    path->yaw_rate_radps[i], M5_SMOOTHER_YAW_TOL) ||
        !near_value(guidance_quintic_eval(&timed->yaw[i], duration_s, 0u),
                    yaw_end, M5_SMOOTHER_YAW_TOL) ||
        !near_value(guidance_quintic_eval(&timed->yaw[i], duration_s, 1u),
                    path->yaw_rate_radps[i + 1u], M5_SMOOTHER_YAW_TOL))
      return M5_SMOOTHER_INVALID_INPUT;
    yaw_start = yaw_end;
    timed_total += duration_us;
  }

  if (path->offset_us[path->segment_count] != source_total ||
      source_total > GUIDANCE_MAX_DURATION_US ||
      fabsf(path->total_cost_s - (float)source_total * 1e-6f) > 1e-4f ||
      timed_total != timed->total_duration_us)
    return M5_SMOOTHER_INVALID_INPUT;
  *total_duration_us = timed_total;
  return M5_SMOOTHER_OK;
}

M5SmootherResult m5_path_smoother_build(
    const M5AstarPath *path, const M5TimedTrajectory *timed,
    const M5LocalMap *map, const NavigationSnapshot *navigation,
    uint64_t now_us, GuidanceMinimumJerkWorkspace *workspace,
    M5SmoothedTrajectory *output)
{
  M5SmoothedTrajectory candidate;
  float duration_s[M5_SMOOTHER_MAX_SEGMENTS] = {0.0f};
  double initial_cost = 0.0, optimized_cost = 0.0;
  uint32_t total_duration_us = 0u;
  M5SmootherResult validation;
  uint8_t optimized;
  unsigned segment;

  if (output != NULL) memset(output, 0, sizeof(*output));
  if (workspace == NULL || output == NULL)
    return M5_SMOOTHER_INVALID_INPUT;
  validation = validate_path(path, timed, map, navigation, now_us,
                             &total_duration_us);
  if (validation != M5_SMOOTHER_OK) return validation;

  memset(&candidate, 0, sizeof(candidate));
  candidate.segment_count = path->segment_count;
  candidate.map_revision = timed->map_revision;
  candidate.total_duration_us = total_duration_us;
  for (segment = 0u; segment < path->segment_count; ++segment) {
    candidate.segment_duration_us[segment] = timed->segment_duration_us[segment];
    duration_s[segment] = (float)timed->segment_duration_us[segment] * 1e-6f;
    candidate.yaw[segment] = timed->yaw[segment];
  }

  optimized = guidance_minimum_jerk_optimize(
      path->position_ned_m, duration_s, path->segment_count,
      path->velocity_ned_mps[0], path->acceleration_ned_mps2[0],
      path->velocity_ned_mps[path->segment_count],
      path->acceleration_ned_mps2[path->segment_count],
      workspace, candidate.position, &initial_cost, &optimized_cost);

  if (optimized != 0u) {
    candidate.fallback_used = 0u;
    candidate.initial_cost = initial_cost;
    candidate.optimized_cost = optimized_cost;
  } else {
    memcpy(candidate.position, timed->position, sizeof(candidate.position));
    candidate.fallback_used = 1u;
  }

  for (segment = 0u; segment < candidate.segment_count; ++segment) {
    unsigned axis;
    if (!quintic_finite(&candidate.yaw[segment])) return M5_SMOOTHER_INVALID_INPUT;
    for (axis = 0u; axis < 3u; ++axis)
      if (!quintic_finite(&candidate.position[segment][axis]))
        return M5_SMOOTHER_INVALID_INPUT;
  }
  *output = candidate;
  return (candidate.fallback_used != 0u) ? M5_SMOOTHER_FALLBACK : M5_SMOOTHER_OK;
}

#endif /* FLIGHT_M5_HOST_PLANNER */
