/*
 * M5-06 independent continuous trajectory safety validator.
 *
 * Host/SIL only. The input is the exact anchor+24 preview that M6 will
 * consume, not the M5-05 optimizer's internal coefficients. A successful
 * result is evidence for M5-07 gating; it is not an actuation permit.
 */
#ifndef M5_TRAJECTORY_SAFETY_H
#define M5_TRAJECTORY_SAFETY_H

#if !defined(FLIGHT_M5_HOST_PLANNER)
#error "M5-06 Host/SIL validator is not part of the H723/NONE production profile"
#endif

#include "trajectory_planning/m5_local_map.h"

#define M5_SAFETY_INTERVAL_COUNT PRODUCT_PREVIEW_COUNT
#define M5_SAFETY_MAX_DEPTH 5u
#define M5_SAFETY_MAX_WORK_UNITS 256u
#define M5_SAFETY_STACK_CAPACITY (M5_SAFETY_INTERVAL_COUNT + M5_SAFETY_MAX_DEPTH)
#define M5_SAFETY_MAX_CELL_CHECKS 8192u
#define M5_SAFETY_MAX_RESPONSE_DELAY_US 200000u
#define M5_SAFETY_MAX_DYNAMIC_ACCEL_MPS2 20.0f

typedef enum {
  M5_SAFETY_PASS = 0u,
  M5_SAFETY_UNKNOWN = 1u,
  M5_SAFETY_STALE = 2u,
  M5_SAFETY_INVALID = 3u,
  M5_SAFETY_STATIC_COLLISION = 4u,
  M5_SAFETY_DYNAMIC_COLLISION = 5u,
  M5_SAFETY_GEOFENCE_VIOLATION = 6u,
  M5_SAFETY_DYNAMICS_VIOLATION = 7u,
  M5_SAFETY_CLEARANCE_UNPROVEN = 8u,
  M5_SAFETY_BUDGET_EXCEEDED = 9u,
  M5_SAFETY_CAPACITY_EXCEEDED = 10u,
  M5_SAFETY_NUMERIC_FAILURE = 11u
} M5TrajectorySafetyStatus;

typedef enum {
  M5_SAFETY_REASON_NONE = 0u,
  M5_SAFETY_REASON_INPUT_INVALID = 1u,
  M5_SAFETY_REASON_NAV_INVALID = 2u,
  M5_SAFETY_REASON_EPOCH_MISMATCH = 3u,
  M5_SAFETY_REASON_REFERENCE_INVALID = 4u,
  M5_SAFETY_REASON_REFERENCE_STALE = 5u,
  M5_SAFETY_REASON_MAP_INVALID = 6u,
  M5_SAFETY_REASON_MAP_STALE = 7u,
  M5_SAFETY_REASON_MAP_UNKNOWN = 8u,
  M5_SAFETY_REASON_MAP_CELL_INVALID = 9u,
  M5_SAFETY_REASON_STATIC_COLLISION = 10u,
  M5_SAFETY_REASON_DYNAMIC_STALE = 11u,
  M5_SAFETY_REASON_DYNAMIC_INVALID = 12u,
  M5_SAFETY_REASON_DYNAMIC_COLLISION = 13u,
  M5_SAFETY_REASON_GEOFENCE_INVALID = 14u,
  M5_SAFETY_REASON_GEOFENCE_VIOLATION = 15u,
  M5_SAFETY_REASON_DYNAMICS = 16u,
  M5_SAFETY_REASON_CLEARANCE_UNPROVEN = 17u,
  M5_SAFETY_REASON_BUDGET = 18u,
  M5_SAFETY_REASON_CAPACITY = 19u,
  M5_SAFETY_REASON_NUMERIC = 20u
} M5TrajectorySafetyReason;

typedef struct {
  const MotionReference *candidate_reference;
  const M5LocalMap *map;
  const NavigationSnapshot *navigation;
  const VehicleDirective *directive;
  const SafetyEnvelope *safety_envelope;
  const ProductGeofenceSnapshot *geofences;
  uint16_t geofence_count;
  uint16_t reserved0;
  uint32_t expected_map_revision; /* copied from the M5-05 candidate */
  uint32_t response_delay_us;
  float max_dynamic_obstacle_accel_mps2; /* product-approved upper bound */
  uint64_t evaluation_time_us;
} M5TrajectorySafetyInput;

typedef struct {
  uint8_t status;
  uint8_t passed;
  uint16_t reason_code;
  uint32_t reference_id;
  uint32_t reference_epoch;
  uint32_t state_epoch;
  uint32_t origin_id;
  uint32_t map_revision;
  uint32_t first_violation_offset_us;
  uint16_t work_units;
  uint16_t cell_checks;
  uint16_t track_checks;
  uint16_t geofence_checks;
  uint64_t verified_at_us;
  uint64_t valid_until_us;
} M5TrajectorySafetyResult;

typedef struct {
  uint8_t interval;
  uint8_t depth;
  uint16_t reserved;
  float u0;
  float u1;
} M5TrajectorySafetyWorkItem;

typedef struct {
  M5TrajectorySafetyWorkItem stack[M5_SAFETY_STACK_CAPACITY];
} M5TrajectorySafetyWorkspace;

CONTRACT_STATIC_ASSERT(sizeof(M5TrajectorySafetyWorkspace) <= 512u,
                       "M5-06 work stack exceeds fixed 512-byte budget");

M5TrajectorySafetyStatus m5_trajectory_safety_validate(
    const M5TrajectorySafetyInput *input,
    M5TrajectorySafetyWorkspace *workspace,
    M5TrajectorySafetyResult *result);

#endif /* M5_TRAJECTORY_SAFETY_H */
