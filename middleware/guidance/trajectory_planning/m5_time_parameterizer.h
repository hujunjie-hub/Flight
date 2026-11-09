/*
 * M5-04 bounded trajectory time parameterizer.
 *
 * Converts an A* geometric/state path into fixed-duration quintic segments.
 * It uses outward-rounded Bernstein bounds; output remains a candidate and
 * must pass M5-06 before publication.
 */
#ifndef M5_TIME_PARAMETERIZER_H
#define M5_TIME_PARAMETERIZER_H

#if !defined(FLIGHT_M5_HOST_PLANNER)
#error "M5 time parameterizer is Host/SIL only in the advanced planner profile"
#endif

#include "trajectory_planning/m5_kinodynamic_astar.h"

#define M5_TIME_PARAMETERIZER_MAX_SEGMENTS M5_ASTAR_MAX_PATH_SEGMENTS
#define M5_TIME_PARAMETERIZER_BIN_US GUIDANCE_TIME_BIN_US
#define M5_TIME_PARAMETERIZER_MAX_TOTAL_US GUIDANCE_MAX_DURATION_US
#define M5_TIME_PARAMETERIZER_MAX_ATTEMPTS 16u

typedef enum {
  M5_TIME_PARAMETERIZER_OK = 0u,
  M5_TIME_PARAMETERIZER_NO_FEASIBLE_DURATION = 1u,
  M5_TIME_PARAMETERIZER_INPUT_STALE = 2u,
  M5_TIME_PARAMETERIZER_INVALID_INPUT = 3u,
  M5_TIME_PARAMETERIZER_CAPACITY_EXCEEDED = 4u
} M5TimeParameterizerResult;

typedef struct {
  uint8_t segment_count;
  uint8_t reserved0;
  uint16_t reserved1;
  uint32_t map_revision;
  uint32_t total_duration_us;
  uint32_t segment_duration_us[M5_TIME_PARAMETERIZER_MAX_SEGMENTS];
  GuidanceQuintic position[M5_TIME_PARAMETERIZER_MAX_SEGMENTS][3];
  GuidanceQuintic yaw[M5_TIME_PARAMETERIZER_MAX_SEGMENTS];
} M5TimedTrajectory;

CONTRACT_STATIC_ASSERT(sizeof(M5TimedTrajectory) <= 2048u,
                       "M5 timed trajectory exceeds fixed 2 KiB budget");

M5TimeParameterizerResult m5_time_parameterizer_build(
    const M5AstarPath *path, const ProductGuidanceConfig *config,
    const SafetyEnvelope *envelope, const NavigationSnapshot *navigation,
    uint64_t now_us, M5TimedTrajectory *output);

#endif /* M5_TIME_PARAMETERIZER_H */
