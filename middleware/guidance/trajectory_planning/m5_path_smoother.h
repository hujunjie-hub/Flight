/*
 * M5-04 Host/SIL path smoothing.
 *
 * Consumes a complete M5-03 path and returns fixed-duration minimum-jerk
 * quintics. Output is a candidate only; M5-06 must validate it before use.
 */
#ifndef M5_PATH_SMOOTHER_H
#define M5_PATH_SMOOTHER_H

#if !defined(FLIGHT_M5_HOST_PLANNER)
#error "M5 path smoother is Host/SIL only; H723/NONE does not compile this module"
#endif

#include "trajectory_planning/m5_kinodynamic_astar.h"

#define M5_SMOOTHER_MAX_SEGMENTS M5_ASTAR_MAX_PATH_SEGMENTS

typedef enum {
  M5_SMOOTHER_OK = 0u,
  M5_SMOOTHER_FALLBACK = 1u,
  M5_SMOOTHER_INPUT_STALE = 2u,
  M5_SMOOTHER_INVALID_INPUT = 3u,
  M5_SMOOTHER_CAPACITY_EXCEEDED = 4u
} M5SmootherResult;

typedef struct {
  uint8_t segment_count;
  uint8_t fallback_used;
  uint16_t reserved0;
  uint32_t map_revision;
  uint32_t total_duration_us;
  uint32_t segment_duration_us[M5_SMOOTHER_MAX_SEGMENTS];
  double initial_cost;
  double optimized_cost;
  GuidanceQuintic position[M5_SMOOTHER_MAX_SEGMENTS][3];
  GuidanceQuintic yaw[M5_SMOOTHER_MAX_SEGMENTS];
} M5SmoothedTrajectory;

CONTRACT_STATIC_ASSERT(sizeof(M5SmoothedTrajectory) <= 2048u,
                       "M5 smoothed path output exceeds fixed 2 KiB budget");

M5SmootherResult m5_path_smoother_build(
    const M5AstarPath *path, const M5LocalMap *map,
    const NavigationSnapshot *navigation, uint64_t now_us,
    GuidanceMinimumJerkWorkspace *workspace, M5SmoothedTrajectory *output);

#endif /* M5_PATH_SMOOTHER_H */
