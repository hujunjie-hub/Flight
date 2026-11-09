/*
 * M5-05 bounded time parameterization for Host/SIL planning.
 *
 * Converts discrete A* primitive durations into Bernstein-bounded segment
 * durations on the 200 ms lattice. H723/NONE does not compile this module.
 */
#ifndef M5_TIME_SCALER_H
#define M5_TIME_SCALER_H

#if !defined(FLIGHT_M5_HOST_PLANNER)
#error "M5 time scaler is Host/SIL only; H723/NONE does not compile this module"
#endif

#include "trajectory_planning/m5_kinodynamic_astar.h"

#define M5_TIME_SCALER_MAX_ATTEMPTS 16u

typedef enum {
  M5_TIME_SCALER_OK = 0u,
  M5_TIME_SCALER_INVALID_INPUT = 1u,
  M5_TIME_SCALER_INFEASIBLE = 2u,
  M5_TIME_SCALER_CAPACITY_EXCEEDED = 3u,
  M5_TIME_SCALER_INPUT_STALE = 4u
} M5TimeScalerResult;

typedef struct {
  M5AstarPath path;
  uint8_t attempts[M5_ASTAR_MAX_PATH_SEGMENTS];
  uint8_t reserved[8];
} M5TimeScaledPath;

CONTRACT_STATIC_ASSERT(sizeof(M5TimeScaledPath) <= 768u,
                       "M5 time-scaled path exceeds fixed 768 byte budget");

M5TimeScalerResult m5_time_scaler_build(
    const M5AstarPath *path, const ProductGuidanceConfig *guidance_config,
    const SafetyEnvelope *safety_envelope, const NavigationSnapshot *navigation,
    uint64_t now_us, M5TimeScaledPath *output);

#endif /* M5_TIME_SCALER_H */
