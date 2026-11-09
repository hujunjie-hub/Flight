/*
 * M5 Guidance: minimum-jerk reference generation, preview resampling,
 * dynamics/geofence/time validation and the READY/ACTIVE reference lifecycle.
 *
 * First-version H723 production scope: OBSTACLE_SOURCE_MODE NONE only.
 * Missions that require obstacle avoidance are rejected and H723 does not
 * compile or allocate the map, A-star, time-scaler or optimizer arenas.
 * Host/SIL-only M5-02 through M5-05 kernels are separate behind FLIGHT_M5_HOST_PLANNER;
 * they do not imply M5-06
 * collision proof, real-sensor assurance, H723 timing or flight release.
 */
#ifndef GUIDANCE_INTERNAL_H
#define GUIDANCE_INTERNAL_H

#include "ports/port_api.h"
#include "messages/message_ids.h"
#include "geofence_geometry/geofence_geometry.h"

#define GUIDANCE_PREVIEW_US PRODUCT_PREVIEW_STEP_US
#define GUIDANCE_PREVIEW_POINTS PRODUCT_PREVIEW_COUNT
#define GUIDANCE_REF_TTL_US 200000u
#define GUIDANCE_VALIDATE_SUBSAMPLES 8u
#define GUIDANCE_TIME_BIN_US 200000u
#define GUIDANCE_MAX_TIME_BINS 31u
#define GUIDANCE_MAX_DURATION_US (GUIDANCE_TIME_BIN_US * GUIDANCE_MAX_TIME_BINS)
#define GUIDANCE_MAX_DURATION_S ((float)GUIDANCE_MAX_DURATION_US * 1e-6f)
CONTRACT_STATIC_ASSERT(GUIDANCE_MAX_DURATION_US == 6200000u,
                       "M5 planning horizon must remain 6.2 seconds");
#define GUIDANCE_MAX_DURATION_ATTEMPTS 16u
#define GUIDANCE_VALIDATION_WORK_UNITS 256u
#define GUIDANCE_VALIDATION_MAX_DEPTH 5u

#define GUIDANCE_MIN_JERK_MAX_SEGMENTS 8u
#define GUIDANCE_MIN_JERK_MAX_VARIABLES (2u * (GUIDANCE_MIN_JERK_MAX_SEGMENTS - 1u))

/* GuidanceFeasibility.violated_constraint_id (02 §4) */
enum {
  GUIDANCE_PLAN_CONSTRAINT_NONE = 0,
  GUIDANCE_PLAN_CONSTRAINT_DYNAMICS = 1,
  GUIDANCE_PLAN_CONSTRAINT_SPEED = 2,
  GUIDANCE_PLAN_CONSTRAINT_ACCELERATION = 3,
  GUIDANCE_PLAN_CONSTRAINT_JERK = 4,
  GUIDANCE_PLAN_CONSTRAINT_TILT = 5,
  GUIDANCE_PLAN_CONSTRAINT_THRUST = 6,
  GUIDANCE_PLAN_CONSTRAINT_GEOFENCE = 7,
  GUIDANCE_PLAN_CONSTRAINT_COLLISION = 8,
  GUIDANCE_PLAN_CONSTRAINT_TIME = 9,
  GUIDANCE_PLAN_CONSTRAINT_CAPACITY = 10
};

/* GuidanceReasonCode subset used by M5 (02 §9). */
#define GUIDANCE_PLAN_REASON_NONE 0x0000u
#define GUIDANCE_PLAN_REASON_INPUT_STALE 0x0601u
#define GUIDANCE_PLAN_REASON_EPOCH_MISMATCH 0x0602u
#define GUIDANCE_PLAN_REASON_OBSTACLE_INVALID 0x0604u
#define GUIDANCE_PLAN_REASON_OBSTACLE_UNKNOWN 0x0605u
#define GUIDANCE_PLAN_REASON_GEOFENCE_INVALID 0x0606u
#define GUIDANCE_PLAN_REASON_GEOFENCE_VIOLATION 0x0607u
#define GUIDANCE_PLAN_REASON_CLEARANCE_UNPROVEN 0x060Au
#define GUIDANCE_PLAN_REASON_BUDGET_EXCEEDED 0x060Cu
#define GUIDANCE_PLAN_REASON_NUMERIC 0x060Du
#define GUIDANCE_PLAN_REASON_ENVELOPE_INVALID 0x060Eu
#define GUIDANCE_PLAN_REASON_REFERENCE_EXPIRED 0x060Fu

typedef struct {
  double c[6]; /* retain boundary accuracy when reconstructing 20 ms intervals */
} GuidanceQuintic;

/*
 * Caller-owned fixed workspace for the Host/SIL minimum-jerk kernel.
 * H723/NONE does not instantiate this object. H is stored as three lower
 * off-diagonals plus the diagonal (block-tridiagonal, scalar bandwidth 3).
 */
typedef struct {
  double hessian[GUIDANCE_MIN_JERK_MAX_VARIABLES][4];
  double gradient[GUIDANCE_MIN_JERK_MAX_VARIABLES];
  double solution[GUIDANCE_MIN_JERK_MAX_VARIABLES];
  double trial[4];
  double plus[4];
  double knot_velocity[GUIDANCE_MIN_JERK_MAX_SEGMENTS + 1u];
  double knot_acceleration[GUIDANCE_MIN_JERK_MAX_SEGMENTS + 1u];
  uint8_t active_local[4];
  uint8_t global_index[4];
  GuidanceQuintic candidate[GUIDANCE_MIN_JERK_MAX_SEGMENTS][3];
} GuidanceMinimumJerkWorkspace;
CONTRACT_STATIC_ASSERT(sizeof(GuidanceMinimumJerkWorkspace) <= 2304u,
                       "Host/SIL minimum-jerk workspace exceeds fixed memory budget");

typedef struct {
  uint8_t valid;
  uint16_t kind;             /* GuidanceReferenceKind */
  uint32_t authority_epoch;
  uint32_t directive_epoch;
  uint32_t state_epoch;
  uint32_t origin_id;
  uint64_t boot_id;
  uint32_t time_epoch;
  uint32_t mission_epoch;
  uint32_t current_item_id;
  uint64_t execution_token;
  float target_ned[3];
  float target_yaw_rad;
} GuidanceRefContext;

typedef struct {
  uint8_t initialized;
  ProductGuidanceConfig config;
  GuidanceRefContext ctx;
  uint32_t ref_id;
  uint32_t ref_epoch;
  uint32_t id_next;
  uint32_t epoch_next;
  uint8_t active;
  ModuleHealth health;
  uint32_t reject_count;
  uint32_t validation_failures;
  uint64_t last_state_time_us;
  uint64_t last_evaluation_time_us;
  uint64_t plan_start_us;
  float plan_duration_s;
  GuidanceQuintic plan[4];          /* N/E/D and unwrapped yaw */
  uint8_t plan_valid;
  uint8_t landing_stage;
  uint8_t pilot_valid;
  float pilot_yaw_rad;
  float pilot_height_up_m;
  float pilot_vertical_up_mps;
  uint32_t sequence;
  uint32_t violation_offset_us;
} GuidanceState;

void guidance_quintic_solve(float p0, float v0, float a0, float pf, float tf, GuidanceQuintic *q);
float guidance_quintic_eval(const GuidanceQuintic *q, float t, uint8_t derivative);
float guidance_quintic_jerk(const GuidanceQuintic *q, float t);
void guidance_quintic_boundary(float p0, float v0, float a0, float p1, float v1, float a1,
                          float duration, GuidanceQuintic *out);
uint8_t guidance_minimum_jerk_optimize(
    const float knot_position[GUIDANCE_MIN_JERK_MAX_SEGMENTS + 1u][3],
    const float segment_duration_s[GUIDANCE_MIN_JERK_MAX_SEGMENTS],
    uint8_t segment_count,
    const float start_velocity[3], const float start_acceleration[3],
    const float end_velocity[3], const float end_acceleration[3],
    GuidanceMinimumJerkWorkspace *workspace,
    GuidanceQuintic output[GUIDANCE_MIN_JERK_MAX_SEGMENTS][3],
    double *initial_cost, double *optimized_cost);
uint8_t guidance_quintic_bounds(const GuidanceQuintic *q, float t0, float t1, uint8_t derivative,
                           float *lower, float *upper);
uint8_t guidance_validate_reference(const GuidanceInput *in, const ProductGuidanceConfig *config,
                                const MotionReference *ref, uint16_t *reason,
                                uint16_t *constraint, uint32_t *violation_offset_us);
uint8_t guidance_curve_dynamics(const GuidanceQuintic q[4], float t0, float t1,
                           const SafetyEnvelope *envelope,
                           const ProductGuidanceConfig *config, uint16_t *constraint);
uint8_t guidance_landing_speed_ok(const GuidanceInput *in, const GuidanceQuintic *down,
                                   float t0, float t1);
uint8_t guidance_has_landing_plane(const GuidanceInput *in);
uint8_t guidance_plan_fences(const GuidanceInput *in, const ProductGuidanceConfig *config,
                         const GuidanceQuintic q[4], float duration);
uint8_t guidance_header_ok(const ContractHeader *h, const ContractHeader *nav, uint16_t type, uint64_t now);
uint64_t guidance_deadline(const GuidanceInput *in, uint16_t kind);
Status guidance_prepare_input(const GuidanceInput *in, const GuidanceState *state, uint16_t kind);
uint8_t guidance_home_ok(const GuidanceInput *in);
uint8_t guidance_handoff_acceleration(const GuidanceState *state, const GuidanceInput *in,
                                       const GuidanceRefContext *next, float acceleration[3]);
Status guidance_build_trajectory(GuidanceState *state, const GuidanceInput *in, GuidanceOutput *out,
                             const float *handoff_acceleration, uint16_t *reason, uint16_t *constraint);
Status guidance_build_pilot(GuidanceState *state, const GuidanceInput *in, GuidanceOutput *out);
Status guidance_build_vertical_land(GuidanceState *state, const GuidanceInput *in, GuidanceOutput *out);
void guidance_publish_header(const GuidanceInput *in, GuidanceState *state, ContractHeader *header,
                          uint16_t type, uint16_t kind);
void guidance_quintic_hermite_eval(float p0, float v0, float a0, float p1, float v1, float a1,
                              float h, float s, float *p, float *v);
float guidance_wrap_pi(float angle);
float guidance_yaw_from_quat(const float q_wxyz[4]);
void guidance_quat_from_rpy(float roll, float pitch, float yaw, float q_wxyz[4]);

uint8_t guidance_geofence_point_ok(const ProductGeofenceSnapshot *fences, uint16_t count,
                              const float p_ned[3], float home_down_m);

#endif /* GUIDANCE_INTERNAL_H */
