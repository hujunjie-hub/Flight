/*
 * M4 FMS core internals (QUAD_X4 first version).
 *
 * Scope of this file: state owned by the eight M4 logical submodules, all
 * updated by one serialized, non-blocking entry point (fms_policy_update).
 * No dynamic allocation, no recursion, no floating-point traps, fixed
 * capacity. Other modules never write these states; they only submit requests
 * or read snapshots (06 §7.1, 02 §21).
 */
#ifndef FMS_INTERNAL_H
#define FMS_INTERNAL_H

#include "fms_contract/fms_contract.h"
#include "flight_policy/fms_policy.h"

#define FMS_MAX_WAYPOINTS PRODUCT_MAX_MISSION_WAYPOINTS
#define FMS_MAX_GEOFENCES PRODUCT_MAX_GEOFENCE_POLYGONS
#define FMS_MAX_VERTICES PRODUCT_MAX_GEOFENCE_VERTICES
#define FMS_MAX_LINKS PRODUCT_MAX_LINK_INSTANCES
#define FMS_MAX_ACKS PRODUCT_MAX_PENDING_OPERATION_REQUESTS
#define FMS_MAX_TRACKERS 4u
#define FMS_DEDUP_CAPACITY 32u
#define FMS_DEDUP_TTL_US 60000000ull
#define FMS_SEQ_SLOTS 16u

/* Fixed policy constants that must not be diluted into runtime tuning. */
#define FMS_PERMIT_LEASE_US 25000ull        /* 07: PERMIT_LEASE_US <= 25 ms */
#define FMS_ARM_REFERENCE_WINDOW_US 100000ull /* 06: absolute 100 ms */
#define FMS_MISSION_ID_MAX 0xFFFFFFFEu
#define FMS_RETURN_ARRIVE_M 0.5f
#define FMS_RETURN_PREPARE_TIMEOUT_US 6000000ull
#define FMS_RETURN_CONTROL_BIND_US 80000ull
#define FMS_TAKEOFF_ALT_TOL_M 0.2f
#define FMS_TAKEOFF_VZ_TOL_MPS 0.2f
#define FMS_TAKEOFF_STABLE_US 1000000ull
#define FMS_PREARM_MAX_DETAIL 15u

/* AuthorityStatus.flags */
enum {
  FMS_AUTH_FLAG_TAKEOVER_LATCHED = 1u << 0,
  FMS_AUTH_FLAG_FAILSAFE_ACTIVE = 1u << 1,
  FMS_AUTH_FLAG_RECOVERY_PENDING = 1u << 2
};

typedef struct {
  uint64_t boot_id;
  uint32_t time_epoch;
  uint32_t profile_id;
  uint64_t now_us;
  uint64_t last_update_us;
  uint8_t bound;
} FmsClock;

typedef struct {
  uint8_t arm_state;
  uint8_t flight_mode;
  uint8_t failsafe_level;
  uint8_t terminal_latched;
  uint8_t critical_energy_latched;
  uint8_t control_link_established;
  uint8_t mode_hysteresis_entered;
  uint32_t authority_epoch;
  uint32_t directive_epoch;
  uint32_t state_epoch;
  uint32_t origin_id;
  uint32_t active_policy_id;
  uint64_t arming_start_us;
  uint64_t mode_since_us;
  uint64_t arm_stable_us;
  uint64_t nav_ready_since_us[10];
  uint32_t nav_candidate_mask;
  uint32_t nav_ready_mask;
} FmsFlight;

typedef struct {
  uint8_t owner;               /* Authority */
  uint16_t source_id;          /* logical source id: 1 RC, 2 GCS, 3 Companion */
  uint8_t latched;
  uint32_t epoch;
  uint32_t last_mode_bits;
  uint64_t mapped_mode_since_us;
  uint8_t mapped_mode_pending;
  uint64_t pilot_since_us;
  uint64_t release_since_us;
} FmsAuthority;

typedef struct {
  uint8_t state;               /* MissionState */
  uint32_t mission_id;
  uint32_t revision;
  uint32_t hash;
  uint32_t mission_epoch;
  uint32_t origin_id;
  uint32_t state_epoch_at_commit;
  uint16_t wp_count;
  uint16_t current_index;
  uint16_t result_code;
  uint64_t token;
  uint64_t token_next;
  uint8_t arrival_active;
  uint64_t arrival_since_us;
  uint64_t expires_at_us;
  float progress;
  ProductMissionPayload active;
} FmsMission;

typedef struct {
  uint8_t active;              /* permit currently published/valid */
  uint8_t revoked_pending;     /* publish REVOKED once, then invalid */
  uint8_t scope;               /* PERMIT_SCOPE_* */
  uint8_t reference_ready;     /* READY or ACTIVE guidance accepted */
  uint16_t output_policy_id;
  uint32_t epoch;
  uint32_t directive_epoch;
  uint32_t authority_epoch;
  uint32_t state_epoch;
  uint32_t reference_epoch;
  uint32_t reference_id;
  uint32_t revoked_watermark;
  uint64_t expires_at_us;
  uint64_t reference_rebind_until_us;
  uint64_t binding_since_us;
  uint16_t reason_code;
} FmsPermit;

typedef struct {
  uint8_t state;               /* LandedState */
  uint8_t candidate;
  uint32_t revision;
  uint8_t evidence_mask;
  float confidence;
  uint32_t stable_ms;
  uint64_t since_us;
  uint64_t candidate_since_us;
} FmsLand;

typedef struct {
  uint64_t sample_us, since_us;
  float velocity_filtered_ned_mps[3];
  float acceleration_ned_mps2[3];
} FmsMotion;

typedef struct {
  uint8_t rc_present, gcs_present, cmp_present;
  uint8_t rc_state, gcs_state, cmp_state;
  uint8_t rc_failsafe;
  uint8_t rc_fresh, gcs_fresh, cmp_fresh;
  uint64_t rc_last_frame_us;
  uint32_t rc_recovery_ms;
} FmsLinks;

typedef struct {
  uint8_t valid;
  ProductHomeTargetSnapshot snap;
} FmsHome;

typedef struct {
  uint32_t fence_count;
  uint32_t revision;
  ProductGeofenceSnapshot fences[FMS_MAX_GEOFENCES];
} FmsGeofence;

typedef struct {
  uint64_t seen_us;
  uint32_t request_id;  /* Zero denotes an unused entry. */
  uint32_t session;
  uint16_t source;
  uint16_t ack_state;
  uint16_t reason;
  uint16_t command_code;
  ProductOperationPayload payload;
} FmsDedupEntry;

enum { FMS_TRACKER_NONE = 0, FMS_TRACKER_TAKEOFF = 1, FMS_TRACKER_LAND = 2, FMS_TRACKER_RETURN = 3 };
enum { FMS_RETURN_WAIT_CONTROL_BINDING = 1, FMS_RETURN_WAIT_MISSION_HANDOFF = 2 };

typedef struct {
  uint8_t kind;
  uint8_t active;
  uint8_t preparing_return;
  uint16_t source;
  uint32_t request_id;
  uint32_t session;
  uint32_t home_token;
  uint64_t started_us;
  uint64_t deadline_us;
  uint64_t last_progress_us;
  uint64_t stable_since_us;
  float target_altitude_m;
  float target_position_ned_m[3];
  float target_yaw_rad;
  uint32_t directive_epoch;
} FmsTracker;

typedef struct {
  uint8_t prearm_state;        /* PreArmState */
  uint8_t item_state[FMS_PREARM_MAX_DETAIL];
  uint32_t check_epoch;
  uint32_t failed_mask;
  uint32_t warning_mask;
  uint16_t primary_reason;
  uint64_t checked_at_us;
  uint8_t valid;
} FmsPrearm;

/* Per-cycle validated input view (no pointer escapes the update call). */
typedef struct {
  const NavigationSnapshot *nav;
  const NavigationCapability *cap;
  const OperatorInput *op;
  const CommunicationLinkStatus *links[FMS_MAX_LINKS];
  const MissionUploadCandidate *candidate;
  const GuidanceFeasibility *feas;
  const GuidanceStatus *guidance;
  const ControlCapability *control;
  const ActuatorCapability *act_cap;
  const ActuatorStatus *act_status;
  const FaultSummary *faults;
  const ProductFmsInputExtension *ext;
  uint16_t link_count;
  uint8_t nav_ok, cap_ok, op_ok, candidate_ok, feas_ok, guidance_ok;
  uint8_t control_ok, act_ok, faults_ok, ext_ok;
  uint8_t rc_state, rc_failsafe, gcs_state, cmp_state;
  uint8_t rc_fresh, gcs_fresh, cmp_fresh;
  uint8_t rc_seen, gcs_seen, cmp_seen;
  uint8_t ground_contact;
  uint8_t energy_state;
  uint8_t geofence_state;
  uint8_t ext_mask_valid;
  float battery_fraction;
  uint64_t now_us;
} FmsInputs;

typedef struct {
  uint16_t ack_state;
  uint16_t reason_code;
} FmsCmdResult;

typedef struct {
  uint8_t initialized;
  const FmsPolicy *policy;
  ProductFmsConfig config;
  FmsClock clock;
  FmsFlight flight;
  FmsAuthority authority;
  FmsMission mission;
  FmsPermit permit;
  FmsLand land;
  FmsMotion motion;
  FmsLinks links;
  FmsHome home;
  FmsGeofence geofence;
  FmsPrearm prearm;
  FmsDedupEntry dedup[FMS_DEDUP_CAPACITY];
  FmsTracker trackers[FMS_MAX_TRACKERS];
  uint32_t seq[FMS_SEQ_SLOTS];
  uint32_t decision_id;
  uint16_t last_decision_code;
  uint16_t last_decision_reason;
  uint32_t last_decision_request;
  ModuleHealth health;
  uint32_t prearm_evaluations;
  uint32_t command_rejects;
  uint32_t permit_revocations;
  uint32_t mission_completions;
  uint32_t epoch_failures;
} FmsEngine;

/* ---- shared helpers (fms.c) ---- */
uint64_t fms_add_us(uint64_t base, uint64_t delta);
uint8_t fms_u64_le(uint64_t a, uint64_t b); /* a <= b without overflow */
uint32_t fms_next_sequence(FmsEngine *e, uint16_t message_type);
void fms_fill_header(FmsEngine *e, ContractHeader *h, uint16_t message_type,
                    uint32_t frame_id, uint64_t sample_us, uint64_t ttl_us);
void fms_clear_header(ContractHeader *h);
void fms_zero_output(FmsOutputView *out);
uint8_t fms_finite(float v);
uint8_t fms_finite3(const float v[3]);
uint8_t fms_finite4(const float v[4]);
void fms_record_decision(FmsEngine *e, uint16_t code, uint16_t reason, uint32_t request_id);
void fms_ack(FmsEngine *e, FmsOutputView *out, const OperationRequest *req,
            uint16_t ack_state, uint16_t reason_code);
uint8_t fms_capable_for_mode(const FmsEngine *e, const FmsInputs *in, uint8_t mode, uint8_t entering);
uint32_t fms_mode_required_nav(uint8_t mode);
uint8_t fms_mode_is_auto(uint8_t mode);
uint8_t fms_mode_is_pilot(uint8_t mode);

/* ---- engine (fms.c) ---- */
Status fms_engine_init(FmsEngine *e, const ProductFmsConfig *config);
Status fms_engine_init_configured(FmsEngine *e, const ProductFmsConfig *config,
                                  uint8_t preserve_geofence);
Status fms_engine_update(FmsEngine *e, const FmsInputView *in, FmsOutputView *out);
void fms_engine_stop(FmsEngine *e);
Status fms_engine_set_geofence(FmsEngine *e, const ProductGeofenceSnapshot *fence);

/* ---- mission (mission_lifecycle.c) ---- */
void fms_mission_init(FmsEngine *e);
void fms_mission_stage(FmsEngine *e, const FmsInputs *in, FmsCmdResult *r);
void fms_mission_commit(FmsEngine *e, const FmsInputs *in, const OperationRequest *req, FmsCmdResult *r);
void fms_mission_start(FmsEngine *e, const FmsInputs *in, const OperationRequest *req, FmsCmdResult *r);
void fms_mission_pause(FmsEngine *e, const FmsInputs *in, const OperationRequest *req, FmsCmdResult *r);
void fms_mission_resume(FmsEngine *e, const FmsInputs *in, const OperationRequest *req, FmsCmdResult *r);
void fms_mission_abort(FmsEngine *e, const FmsInputs *in, const OperationRequest *req, FmsCmdResult *r);
void fms_mission_clear(FmsEngine *e, const FmsInputs *in, const OperationRequest *req, FmsCmdResult *r);
void fms_mission_tick(FmsEngine *e, const FmsInputs *in);
void fms_mission_on_pilot_takeover(FmsEngine *e);
void fms_mission_on_failsafe(FmsEngine *e);
void fms_mission_suspend(FmsEngine *e, uint16_t reason);
uint8_t fms_mission_running(const FmsEngine *e);

/* ---- authority/mode (operator_authority.c) ---- */
void fms_authority_init(FmsEngine *e);
void fms_mode_capability_tick(FmsEngine *e, const FmsInputs *in);
void fms_authority_update(FmsEngine *e, const FmsInputs *in);
void fms_authority_set(FmsEngine *e, uint8_t owner, uint16_t source_id, uint16_t reason);
uint8_t fms_authority_pilot(const FmsEngine *e);
uint8_t fms_mode_transition(FmsEngine *e, uint8_t mode, const FmsInputs *in,
                           uint8_t internal_request);
void fms_failsafe_update(FmsEngine *e, const FmsInputs *in);

/* ---- permit (actuation_permit.c) ---- */
void fms_permit_init(FmsEngine *e);
void fms_permit_tick(FmsEngine *e, const FmsInputs *in);
void fms_permit_revoke(FmsEngine *e, uint16_t reason);
void fms_permit_fill(FmsEngine *e, ActuationPermit *out);

/* ---- support submodules (fms_safety.c) ---- */
void fms_land_init(FmsEngine *e);
void fms_land_tick(FmsEngine *e, const FmsInputs *in);
void fms_motion_tick(FmsEngine *e, const FmsInputs *in);
const float *fms_motion_acceleration(const FmsEngine *e, const FmsInputs *in);
void fms_home_tick(FmsEngine *e, const FmsInputs *in);
void fms_home_capture(FmsEngine *e, const FmsInputs *in, uint8_t source, uint16_t *reason);
void fms_geofence_init(FmsEngine *e);
Status fms_geofence_configure(FmsEngine *e, const ProductGeofenceSnapshot *fence);
uint8_t fms_geofence_contains(const FmsGeofence *g, const float p_ned[3], float home_down_m);
void fms_prearm_evaluate(FmsEngine *e, const FmsInputs *in);
uint8_t fms_prearm_ready_for_arm(const FmsEngine *e, const FmsInputs *in);

#endif /* FMS_INTERNAL_H */
