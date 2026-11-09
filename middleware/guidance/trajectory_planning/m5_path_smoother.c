/*
 * M5-05 bounded Host/SIL minimum-jerk path smoother.
 *
 * Consumes the M5-04 time-scaled A* path and returns position/yaw quintic
 * candidates. Any result, including fallback, requires independent M5-06
 * validation before it can be published to M6.
 */
#if defined(FLIGHT_M5_HOST_PLANNER)
#include <math.h>
#include <string.h>
#include "trajectory_planning/m5_path_smoother.h"

#define M5_SMOOTHER_TWO_PI 6.28318530717958647692f

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

static M5SmootherResult validate_path(const M5TimeScaledPath *timed_path,
                                      const M5LocalMap *map,
                                      const NavigationSnapshot *navigation,
                                      uint64_t now_us,
                                      uint32_t *total_duration_us)
{
  const M5AstarPath *path;
  uint8_t primitive_count = 0u;
  const M5KinodynamicPrimitive *primitives;
  uint32_t total = 0u;
  unsigned i;
  if (timed_path == NULL || map == NULL || navigation == NULL ||
      total_duration_us == NULL || now_us == 0u)
    return M5_SMOOTHER_INVALID_INPUT;
  path = &timed_path->path;
  if (path->segment_count == 0u) return M5_SMOOTHER_INVALID_INPUT;
  if (path->segment_count > M5_SMOOTHER_MAX_SEGMENTS)
    return M5_SMOOTHER_CAPACITY_EXCEEDED;
  if (path->map_revision == 0u || path->map_revision != map->map_revision ||
      !m5_local_map_is_ready(map, navigation, now_us))
    return M5_SMOOTHER_INPUT_STALE;
  if (path->offset_us[0] != 0u || !isfinite(path->total_cost_s) ||
      path->total_cost_s <= 0.0f)
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
    uint32_t duration;
    if (path->offset_us[i + 1u] <= path->offset_us[i])
      return M5_SMOOTHER_INVALID_INPUT;
    duration = path->offset_us[i + 1u] - path->offset_us[i];
    if (duration < GUIDANCE_TIME_BIN_US ||
        duration % GUIDANCE_TIME_BIN_US != 0u ||
        duration > GUIDANCE_MAX_DURATION_US ||
        total > GUIDANCE_MAX_DURATION_US - duration)
      return M5_SMOOTHER_CAPACITY_EXCEEDED;
    if (path->primitive_index[i] >= primitive_count ||
        timed_path->attempts[i] == 0u ||
        timed_path->attempts[i] > M5_TIME_SCALER_MAX_ATTEMPTS)
      return M5_SMOOTHER_INVALID_INPUT;
    total += duration;
  }
  if (path->offset_us[path->segment_count] != total ||
      total > GUIDANCE_MAX_DURATION_US ||
      fabsf(path->total_cost_s - (float)total * 1e-6f) > 1e-4f)
    return M5_SMOOTHER_INVALID_INPUT;
  *total_duration_us = total;
  return M5_SMOOTHER_OK;
}

static uint8_t build_initial_position_quintics(const M5AstarPath *path,
                                                const float duration_s[M5_SMOOTHER_MAX_SEGMENTS],
                                                M5SmoothedTrajectory *candidate)
{
  unsigned segment, axis;
  for (segment = 0u; segment < path->segment_count; ++segment) {
    for (axis = 0u; axis < 3u; ++axis) {
      guidance_quintic_boundary(
          path->position_ned_m[segment][axis],
          path->velocity_ned_mps[segment][axis],
          path->acceleration_ned_mps2[segment][axis],
          path->position_ned_m[segment + 1u][axis],
          path->velocity_ned_mps[segment + 1u][axis],
          path->acceleration_ned_mps2[segment + 1u][axis],
          duration_s[segment], &candidate->position[segment][axis]);
      if (!quintic_finite(&candidate->position[segment][axis])) return 0u;
    }
  }
  return 1u;
}

static uint8_t build_yaw_quintics(const M5AstarPath *path,
                                  const float duration_s[M5_SMOOTHER_MAX_SEGMENTS],
                                  M5SmoothedTrajectory *candidate)
{
  float yaw_start = path->yaw_rad[0];
  unsigned segment;
  for (segment = 0u; segment < path->segment_count; ++segment) {
    float delta = remainderf(path->yaw_rad[segment + 1u] -
                             path->yaw_rad[segment], M5_SMOOTHER_TWO_PI);
    float yaw_end = yaw_start + delta;
    guidance_quintic_boundary(yaw_start, path->yaw_rate_radps[segment], 0.0f,
        yaw_end, path->yaw_rate_radps[segment + 1u], 0.0f,
        duration_s[segment], &candidate->yaw[segment]);
    if (!quintic_finite(&candidate->yaw[segment])) return 0u;
    yaw_start = yaw_end;
  }
  return 1u;
}

M5SmootherResult m5_path_smoother_build(
    const M5TimeScaledPath *timed_path, const M5LocalMap *map,
    const NavigationSnapshot *navigation, uint64_t now_us,
    GuidanceMinimumJerkWorkspace *workspace, M5SmoothedTrajectory *output)
{
  const M5AstarPath *path;
  M5SmoothedTrajectory candidate;
  float duration_s[M5_SMOOTHER_MAX_SEGMENTS] = {0.0f};
  double initial_cost = 0.0, optimized_cost = 0.0;
  uint32_t total_duration_us = 0u;
  M5SmootherResult validation;
  uint8_t optimized;
  unsigned segment;

  if (output != NULL) memset(output, 0, sizeof(*output));
  if (workspace == NULL || output == NULL || timed_path == NULL)
    return M5_SMOOTHER_INVALID_INPUT;
  path = &timed_path->path;
  validation = validate_path(timed_path, map, navigation, now_us,
                             &total_duration_us);
  if (validation != M5_SMOOTHER_OK) return validation;

  memset(&candidate, 0, sizeof(candidate));
  candidate.segment_count = path->segment_count;
  candidate.map_revision = path->map_revision;
  candidate.total_duration_us = total_duration_us;
  for (segment = 0u; segment < path->segment_count; ++segment) {
    candidate.segment_duration_us[segment] =
        path->offset_us[segment + 1u] - path->offset_us[segment];
    duration_s[segment] = (float)candidate.segment_duration_us[segment] * 1e-6f;
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
    if (!build_initial_position_quintics(path, duration_s, &candidate))
      return M5_SMOOTHER_INVALID_INPUT;
    candidate.fallback_used = 1u;
  }
  if (!build_yaw_quintics(path, duration_s, &candidate))
    return M5_SMOOTHER_INVALID_INPUT;

  *output = candidate;
  return (candidate.fallback_used != 0u) ? M5_SMOOTHER_FALLBACK : M5_SMOOTHER_OK;
}

#endif /* FLIGHT_M5_HOST_PLANNER */
