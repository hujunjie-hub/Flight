/*
 * Flight public contract - common message layer.
 *
 * Authority: spec/02_接口契约与公共数据规范.md (V1.9.1-GLOBAL-AUDIT-R3),
 *            section 1.5 to 1.21 and section 3.
 *
 * Layer position:
 *   common_base.h -> product_contract.h -> common_messages.h -> port_api.h
 *
 * Every structure here contains a ContractHeader (except the specified
 * single-message aggregates) and is registered in
 * spec/contracts/message_registry.json.  Message type constants live in
 * message_ids.h and are append-only.
 *
 * Pointers are never part of a message; they appear only in the Port
 * aggregates of port_types.h and are valid only for the synchronous call.
 */
#ifndef COMMON_MESSAGES_H
#define COMMON_MESSAGES_H

#include "base_types/contract_version.h"
#include "base_types/common_base.h"
#include "product_contract/product_contract.h"
#include "ports/control_model_types.h"

EXTERN_C_BEGIN

/* ------------------------------------------------------------------------- */
/* M1 envelopes and external odometry (spec 02 sections 1.5)                  */
/* ------------------------------------------------------------------------- */

typedef struct {
  ContractHeader header;
  uint32_t request_id;
  uint32_t source_session_id; /* M1 authenticated logical-source session, nonzero */
  uint16_t command_code;       /* meaning owned by selected ProductContract */
  uint16_t authority_class;
  uint16_t payload_schema_id;
  uint16_t reserved;
  uint64_t execution_deadline_us;
  ProductOperationPayload payload;
} OperationRequest;

typedef struct {
  ContractHeader header;
  uint32_t input_layout_id;
  uint32_t valid_axis_mask;
  ProductOperatorInput payload;
} OperatorInput;

typedef struct {
  ContractHeader header;
  uint32_t source_origin_id;
  uint32_t reset_counter;
  uint32_t tracking_status;
  float position_ned_m[3];
  float velocity_ned_mps[3];
  float q_nb_wxyz[4];
  float position_variance_m2[3];
  float velocity_variance_m2ps2[3];
  float attitude_variance_rad2[3];
  uint32_t time_sync_uncertainty_us;
} ExternalOdometry;

/* ------------------------------------------------------------------------- */
/* M2 tagged union (spec 02 sections 1.6)                                     */
/* ------------------------------------------------------------------------- */

typedef struct {
  ContractHeader header;
  uint32_t sensor_instance_id;
  uint16_t measurement_kind;
  uint16_t reserved;
  ProductSensorMeasurement payload; /* tagged union of enabled strong types */
} SensorMeasurement;

/* ------------------------------------------------------------------------- */
/* M3 navigation state (spec 02 sections 1.7)                                 */
/* ------------------------------------------------------------------------- */

typedef struct {
  ContractHeader header;
  uint32_t state_epoch;
  uint32_t origin_id;
  NavStateCore core;
  ProductNavExtension vehicle_extension;
} NavigationSnapshot;

typedef struct {
  ContractHeader header;
  uint32_t state_epoch;
  uint32_t origin_id;
  uint32_t capability_mask;
  uint32_t degraded_mask;
  uint8_t solution_state;
  uint8_t reserved[3];
  float horizontal_position_std_m;
  float vertical_position_std_m;
  float velocity_std_mps;
  float heading_std_rad;
  uint32_t active_source_mask;
  uint32_t rejected_source_mask;
} NavigationCapability;

/* NavigationCapability.solution_state (spec 02 section 4). */
#define NAV_SOLUTION_INITIALIZING 0u
#define NAV_SOLUTION_VALID        1u
#define NAV_SOLUTION_DEGRADED     2u
#define NAV_SOLUTION_INVALID      3u

typedef struct {
  ContractHeader header;
  uint32_t old_state_epoch;
  uint32_t new_state_epoch;
  uint32_t old_origin_id;
  uint32_t new_origin_id;
  uint16_t reset_type;
  uint16_t reserved;
  float position_delta_m[3];
  float velocity_delta_mps[3];
  float attitude_delta_rad[3];
} StateResetEvent;

typedef struct {
  ContractHeader header;
  uint16_t reset_type;
  uint16_t initial_source;
  uint32_t requested_origin_id;
  uint32_t valid_initial_state_mask;
  NavStateCore initial_state;
} EstimatorResetRequest;

/* Reset type / initial source (spec 02 section 4). */
#define RESET_TYPE_INITIALIZE        1u
#define RESET_TYPE_ORIGIN_REBUILD    2u
#define RESET_TYPE_RECOVER_AFTER_GAP 3u
#define RESET_SOURCE_PROVIDED_STATE  0u
#define RESET_SOURCE_STATIC_IMU_LOCAL 1u
#define RESET_SOURCE_GNSS_ASSISTED   2u

typedef struct {
  ContractHeader header;
  uint32_t safety_envelope_id;
  uint32_t envelope_epoch;
  float max_linear_speed_mps;
  float max_linear_accel_mps2;
  float max_angular_rate_radps;
  float max_yaw_rate_radps;
  ProductSafetyEnvelopePayload vehicle_extension;
} SafetyEnvelope;

typedef struct {
  ContractHeader header;
  uint32_t sensor_instance_id;
  uint32_t policy_epoch;
  uint16_t use_state;       /* 0=NORMAL, 1=DEGRADED, 2=REJECT */
  uint16_t reason_code;
  uint32_t degraded_noise_profile_id;
  uint32_t evaluation_flags; /* bit0 ALLOW_SHADOW_EVALUATION; other bits zero */
  uint64_t effective_time_us;
  uint64_t policy_expires_at_us;
} SensorUsePolicy;

/* SensorUsePolicy.use_state (spec 02 section 1.7). */
#define SENSOR_USE_NORMAL   0u
#define SENSOR_USE_DEGRADED 1u
#define SENSOR_USE_REJECT   2u

#define SENSOR_USE_EVAL_ALLOW_SHADOW (1u << 0)

typedef enum {
  SENSOR_CONSISTENCY_REASON_NONE = 0,
  SENSOR_CONSISTENCY_REASON_POLICY_DISABLED = 1,
  SENSOR_CONSISTENCY_REASON_TIME_NOT_READY = 2,
  SENSOR_CONSISTENCY_REASON_CALIBRATION_NOT_READY = 3,
  SENSOR_CONSISTENCY_REASON_INSUFFICIENT_SAMPLES = 4,
  SENSOR_CONSISTENCY_REASON_NUMERIC_FAILURE = 5,
  SENSOR_CONSISTENCY_REASON_INCONSISTENT = 6,
  SENSOR_CONSISTENCY_REASON_EVIDENCE_TIMEOUT = 7,
  SENSOR_CONSISTENCY_REASON_VERSION_MISMATCH = 8
} SensorConsistencyReason;

typedef struct {
  ContractHeader header;
  uint32_t sensor_instance_id;
  uint32_t policy_epoch;
  uint32_t state_epoch;
  uint32_t calibration_version;
  uint32_t window_duration_us;
  uint16_t evaluated_count;
  uint16_t accepted_count;
  uint16_t rejected_count;
  uint16_t probation_pass_count;
  uint32_t evidence_flags; /* TIME_READY/CALIBRATION_READY/NUMERIC_READY/CONSISTENCY_READY */
  uint16_t reason_code;
  uint16_t evaluation_mode; /* 0=UNAVAILABLE, 1=NORMAL, 2=SHADOW_ONLY */
  uint64_t last_evaluated_sample_time_us;
  uint64_t evidence_expires_at_us;
} SensorConsistencyEvidence;

#define SENSOR_EVIDENCE_TIME_READY        (1u << 0)
#define SENSOR_EVIDENCE_CALIBRATION_READY (1u << 1)
#define SENSOR_EVIDENCE_NUMERIC_READY     (1u << 2)
#define SENSOR_EVIDENCE_CONSISTENCY_READY (1u << 3)

/* ------------------------------------------------------------------------- */
/* M4 vehicle operation policy (spec 02 sections 1.8)                         */
/* ------------------------------------------------------------------------- */

typedef enum {
  LANDED_STATE_UNKNOWN = 0,
  LANDED_STATE_LANDED = 1,
  LANDED_STATE_IN_AIR = 2,
  LANDED_STATE_TAKEOFF = 3,
  LANDED_STATE_LANDING = 4
} LandedState;

typedef struct {
  ContractHeader header;
  uint32_t state_revision;
  uint16_t state;
  uint16_t evidence_mask;
  float confidence_0_to_1;
  uint32_t stable_duration_ms;
  uint64_t state_since_us;
} LandedStateSnapshot;

/* LandedStateSnapshot.evidence_mask: command evidence, never ESC feedback. */
#define LANDED_EVIDENCE_GROUND_CONTACT     (1u << 0)
#define LANDED_EVIDENCE_VERTICAL_STABLE    (1u << 1)
#define LANDED_EVIDENCE_LOW_THRUST_COMMAND (1u << 2)

typedef struct {
  ContractHeader header;
  uint32_t directive_epoch;
  uint32_t authority_epoch;
  uint32_t state_epoch;
  uint32_t policy_id;
  uint16_t operation_code;       /* selected ProductContract */
  uint16_t guidance_mode_code;   /* selected ProductContract */
  uint16_t reason_code;
  uint16_t reserved;
  uint64_t directive_expires_at_us;
  uint32_t required_nav_capabilities;
  uint32_t safety_envelope_id;
  ProductDirectivePayload payload;
} VehicleDirective;

/* VehicleDirective.operation_code (spec 02 section 4). */
#define PRODUCT_OP_NONE        0u
#define PRODUCT_OP_PILOT       1u
#define PRODUCT_OP_HOLD        2u
#define PRODUCT_OP_MISSION     3u
#define PRODUCT_OP_RETURN      4u
#define PRODUCT_OP_LAND        5u
#define PRODUCT_OP_SAFE_ACTION 6u

/* VehicleDirective.guidance_mode_code (spec 02 section 4). */
#define PRODUCT_GUIDANCE_MODE_NONE       0u
#define PRODUCT_GUIDANCE_MODE_RATE       1u
#define PRODUCT_GUIDANCE_MODE_ATTITUDE   2u
#define PRODUCT_GUIDANCE_MODE_ALT_HOLD   3u
#define PRODUCT_GUIDANCE_MODE_POS_HOLD   4u
#define PRODUCT_GUIDANCE_MODE_TRAJECTORY 5u

typedef struct {
  ContractHeader header;
  uint32_t permit_epoch;
  uint32_t directive_epoch;
  uint32_t authority_epoch;
  uint32_t state_epoch;
  uint32_t reference_epoch;
  uint32_t permit_flags;
  uint64_t permit_expires_at_us;
  uint16_t reason_code;
  uint16_t output_policy_id;
} ActuationPermit;

#define PERMIT_SCOPE_IDLE_ONLY      (1u << 0)
#define PERMIT_SCOPE_CONTROL_ACTIVE (1u << 1)
#define PERMIT_FLAG_REVOKED         (1u << 2)

/* ------------------------------------------------------------------------- */
/* M5 guidance (spec 02 sections 1.9)                                         */
/* ------------------------------------------------------------------------- */

typedef struct {
  uint32_t offset_us;
  uint32_t valid_mask;
  ProductMotionPoint point;
} MotionReferencePoint;

/* Uniform sample spacing supported by the trajectory producer and consumer. */
#define PRODUCT_PREVIEW_STEP_US 20000u
#define PRODUCT_ROUTE_PREVIEW_STEP_US 40000u

typedef struct {
  ContractHeader header;
  uint32_t reference_id;
  uint32_t reference_epoch;
  uint32_t state_epoch;
  uint32_t origin_id;
  uint32_t authority_epoch;
  uint32_t directive_epoch;
  uint32_t mission_epoch;
  uint32_t current_item_id;
  uint64_t execution_token;       /* 0 only for non-mission references */
  uint16_t point_count;          /* exactly 24 future points for QUAD */
  uint16_t reference_kind;
  uint32_t reserved;
  MotionReferencePoint anchor;   /* offset_us == 0, p/v/a/yaw/yaw-rate at t0 */
  MotionReferencePoint points[PRODUCT_PREVIEW_COUNT];
} MotionReference;

/* ------------------------------------------------------------------------- */
/* M6 control (spec 02 sections 1.10)                                         */
/* ------------------------------------------------------------------------- */

typedef struct {
  ContractHeader header;
  uint32_t actuator_layout_id;
  uint32_t valid_channel_mask;
  uint32_t permit_epoch;
  uint32_t directive_epoch;
  uint32_t authority_epoch;
  uint32_t state_epoch;
  uint32_t reference_epoch;
  uint32_t control_sequence;
  uint64_t demand_expires_at_us;
  ProductActuatorDemand demand;  /* physical units, product typed */
} ActuatorDemand;

/* Receipt is public command-commit evidence, not measured actuator feedback. */
typedef struct {
  ContractHeader header;
  uint32_t applied_control_sequence;
  uint32_t permit_epoch;
  uint32_t directive_epoch;
  uint32_t authority_epoch;
  uint32_t state_epoch;
  uint32_t reference_epoch;
  uint64_t commit_time_us;
  uint64_t estimated_apply_time_us;
  uint64_t receipt_expires_at_us;
  uint32_t limit_flags;
  uint16_t output_policy_id;
  uint16_t result_code; /* COMMITTED / REJECTED / DRIVER_FAULT */
  ProductActuatorDemand committed_demand;
} ActuatorCommitReceipt;

#define ACTUATOR_RECEIPT_COMMITTED    1u
#define ACTUATOR_RECEIPT_REJECTED     2u
#define ACTUATOR_RECEIPT_DRIVER_FAULT 3u

typedef struct {
  ContractHeader header;
  uint32_t actuator_layout_id;
  uint32_t available_mask;
  uint32_t fault_mask;
  uint32_t feedback_valid_mask;
  ProductActuatorFeedback feedback;
} ActuatorStatus;

typedef struct {
  ContractHeader header;
  uint32_t actuator_layout_id;
  uint32_t available_mask;
  uint32_t controllable_dof_mask;
  uint32_t degraded_dof_mask;
  ProductActuatorCapability vehicle_extension;
} ActuatorCapability;

/* Control DOF mask (spec 02 section 4). */
#define CONTROL_DOF_TOTAL_THRUST (1u << 0)
#define CONTROL_DOF_ROLL_TORQUE  (1u << 1)
#define CONTROL_DOF_PITCH_TORQUE (1u << 2)
#define CONTROL_DOF_YAW_TORQUE   (1u << 3)

/* ControlCapability.reason_code stable registry; layout unchanged. */
typedef enum {
  CONTROL_REASON_NONE = 0x0000,
  CONTROL_REASON_ALLOCATOR_INFEASIBLE = 0x0701,
  CONTROL_REASON_ALLOCATOR_RANK_DEFICIENT = 0x0702,
  CONTROL_REASON_ALLOCATOR_NUMERIC_FAILURE = 0x0703,
  CONTROL_REASON_ACTUATOR_LIMITED = 0x0704,
  CONTROL_REASON_MPC_HOVER_OUTSIDE_APPROX_DOMAIN = 0x0705
} ControlReasonCode;

typedef struct {
  ContractHeader header;
  uint32_t control_layout_id;
  uint32_t controllable_dof_mask;
  uint32_t degraded_dof_mask;
  uint8_t health_state;
  uint8_t control_level;
  uint16_t reason_code;
} ControlCapability;

/* Common module health_state (spec 02 section 4). */
#define MODULE_HEALTH_UNKNOWN      0u
#define MODULE_HEALTH_INITIALIZING 1u
#define MODULE_HEALTH_HEALTHY      2u
#define MODULE_HEALTH_DEGRADED     3u
#define MODULE_HEALTH_FAILED       4u
#define MODULE_HEALTH_STOPPED      5u

/* ------------------------------------------------------------------------- */
/* Health summaries and evidence (spec 02 sections 1.11, 1.18)                */
/* ------------------------------------------------------------------------- */

typedef struct {
  ContractHeader header;
  uint32_t active_fault_count;
  uint32_t latched_fault_count;
  uint32_t fault_domain_mask;
  uint32_t highest_severity_fault_id;
  uint8_t highest_severity;
  uint8_t reserved[3];
} FaultSummary;

typedef struct {
  ContractHeader header;
  uint32_t mission_id;
  uint32_t current_item_id;
  uint32_t mission_epoch;
  uint32_t mission_revision;
  uint64_t execution_token;
  uint16_t mission_state;
  uint16_t result_code;
  float progress_0_to_1;
} MissionStatus;

/* MissionStatus.result_code (spec 02 section 4). */
#define MISSION_RESULT_NONE     0u
#define MISSION_RESULT_SUCCESS  1u
#define MISSION_RESULT_REJECTED 2u
#define MISSION_RESULT_ABORTED  3u
#define MISSION_RESULT_TIMEOUT  4u
#define MISSION_RESULT_FAILED   5u

typedef struct {
  ContractHeader header;
  uint32_t reference_id;
  uint32_t reference_epoch;
  uint8_t feasible;
  uint8_t severity;
  uint16_t reason_code;
  uint32_t violated_constraint_id;
  uint32_t earliest_violation_offset_us;
} GuidanceFeasibility;

/* GuidanceFeasibility.violated_constraint_id (spec 02 section 4). */
#define GUIDANCE_CONSTRAINT_NONE         0u
#define GUIDANCE_CONSTRAINT_DYNAMICS     1u
#define GUIDANCE_CONSTRAINT_SPEED        2u
#define GUIDANCE_CONSTRAINT_ACCELERATION 3u
#define GUIDANCE_CONSTRAINT_JERK         4u
#define GUIDANCE_CONSTRAINT_TILT         5u
#define GUIDANCE_CONSTRAINT_THRUST       6u
#define GUIDANCE_CONSTRAINT_GEOFENCE     7u
#define GUIDANCE_CONSTRAINT_COLLISION    8u
#define GUIDANCE_CONSTRAINT_TIME         9u
#define GUIDANCE_CONSTRAINT_CAPACITY     10u

typedef struct {
  ContractHeader header;
  uint32_t request_id;
  uint32_t source_session_id;
  uint16_t request_source_id;
  uint16_t reserved_source;
  uint16_t ack_state;
  uint16_t reason_code;
  uint64_t completion_time_us;
} CommandAck;

/* CommandAck.ack_state (spec 02 section 4). */
#define COMMAND_ACK_REJECTED   0u
#define COMMAND_ACK_ACCEPTED   1u
#define COMMAND_ACK_IN_PROGRESS 2u
#define COMMAND_ACK_COMPLETED  3u
#define COMMAND_ACK_FAILED     4u
#define COMMAND_ACK_CANCELLED  5u

typedef struct {
  ContractHeader header;
  uint32_t decision_id;
  uint32_t request_id;
  uint32_t policy_id;
  uint32_t evidence_mask;
  uint16_t decision_code;
  uint16_t reason_code;
} DecisionEvent;

/* DecisionEvent.decision_code (spec 02 section 4). */
#define DECISION_NO_CHANGE       0u
#define DECISION_ACCEPT          1u
#define DECISION_REJECT          2u
#define DECISION_FORCE_SAFE_ACTION 3u

typedef enum {
  GUIDANCE_STATE_INACTIVE = 0,
  GUIDANCE_STATE_READY = 1,
  GUIDANCE_STATE_ACTIVE = 2,
  GUIDANCE_STATE_HOLDING = 3,
  GUIDANCE_STATE_REJECTED = 4,
  GUIDANCE_STATE_EXPIRED = 5
} GuidanceRunState;

typedef enum {
  GUIDANCE_REFERENCE_NONE = 0,
  GUIDANCE_REFERENCE_TRAJECTORY = 1,
  GUIDANCE_REFERENCE_PILOT_RATE = 2,
  GUIDANCE_REFERENCE_PILOT_ATTITUDE = 3,
  GUIDANCE_REFERENCE_ALTITUDE_PILOT = 4,
  GUIDANCE_REFERENCE_ALTITUDE_AUTOMATIC = 5
} GuidanceReferenceKind;

/* GuidanceStatus.reason_code stable registry; values unique in this Product
 * Contract. */
typedef enum {
  GUIDANCE_REASON_NONE = 0x0000,
  M5_VALIDATION_INPUT_STALE = 0x0601,
  M5_VALIDATION_EPOCH_MISMATCH = 0x0602,
  M5_VALIDATION_FRAME_ORIGIN_MISMATCH = 0x0603,
  M5_VALIDATION_OBSTACLE_INVALID = 0x0604,
  M5_VALIDATION_OBSTACLE_UNKNOWN = 0x0605,
  M5_VALIDATION_GEOFENCE_INVALID = 0x0606,
  M5_VALIDATION_GEOFENCE_VIOLATION = 0x0607,
  M5_VALIDATION_STATIC_COLLISION = 0x0608,
  M5_VALIDATION_DYNAMIC_COLLISION = 0x0609,
  M5_VALIDATION_CLEARANCE_UNPROVEN = 0x060A,
  M5_VALIDATION_UNCERTAINTY_UNBOUNDED = 0x060B,
  M5_VALIDATION_INTERVAL_BUDGET_EXCEEDED = 0x060C,
  M5_VALIDATION_NUMERIC_INVALID = 0x060D,
  M5_VALIDATION_VEHICLE_ENVELOPE_INVALID = 0x060E,
  M5_VALIDATION_REFERENCE_EXPIRED = 0x060F
} GuidanceReasonCode;

typedef struct {
  ContractHeader header;
  uint32_t reference_id;
  uint32_t reference_epoch;
  uint32_t state_epoch;
  uint32_t origin_id;
  uint32_t authority_epoch;
  uint32_t directive_epoch;
  uint32_t mission_epoch;
  uint32_t current_item_id;
  uint64_t execution_token;
  uint16_t guidance_state;
  uint16_t reference_kind;
  uint16_t reason_code;
  uint16_t item_result; /* 0=NONE,1=TRACKING,2=SEGMENT_FINISHED,3=REJECTED */
  uint64_t reference_expires_at_us;
} GuidanceStatus;

/* GuidanceStatus.item_result (spec 02 section 1.11). */
#define GUIDANCE_ITEM_RESULT_NONE             0u
#define GUIDANCE_ITEM_RESULT_TRACKING         1u
#define GUIDANCE_ITEM_RESULT_SEGMENT_FINISHED 2u
#define GUIDANCE_ITEM_RESULT_REJECTED         3u

/* ------------------------------------------------------------------------- */
/* M4 FMS state / pre-arm / pilot reference (spec 02 sections 1.11)           */
/* ------------------------------------------------------------------------- */

typedef struct {
  ContractHeader header;
  uint8_t arm_state;
  uint8_t flight_mode;
  uint8_t authority;
  uint8_t failsafe_level;
  uint8_t mission_state;
  uint8_t reserved[3];
  uint32_t authority_epoch;
  uint32_t directive_epoch;
  uint32_t permit_epoch;
  uint32_t reference_epoch;
  uint32_t state_epoch;
  uint32_t active_mission_id;
  uint32_t active_policy_id;
} FmsStateSnapshot;

typedef enum {
  PREARM_NOT_RUN = 0,
  PREARM_RUNNING = 1,
  PREARM_READY = 2,
  PREARM_REJECT = 3
} PreArmState;

typedef struct {
  ContractHeader header;
  uint32_t check_epoch;
  uint32_t failed_checks_mask;
  uint16_t state;
  uint16_t primary_reason_code;
  uint16_t failed_count;
  uint16_t warning_count;
  uint64_t checked_at_us;
} PreArmCheckResult;

typedef enum {
  PILOT_REFERENCE_NONE = 0,
  PILOT_REFERENCE_RATE = 1,
  PILOT_REFERENCE_ATTITUDE = 2,
  PILOT_REFERENCE_ALTITUDE_ATTITUDE = 3
} PilotReferenceKind;

typedef struct {
  ContractHeader header;
  uint32_t reference_id;
  uint32_t reference_epoch;
  uint32_t state_epoch;
  uint32_t origin_id;
  uint32_t authority_epoch;
  uint32_t directive_epoch;
  uint16_t reference_kind;
  uint16_t valid_mask;
  ProductPilotControlReferencePayload vehicle_extension;
} PilotControlReference;

/* ------------------------------------------------------------------------- */
/* Common module health (spec 02 sections 1.11)                               */
/* ------------------------------------------------------------------------- */

typedef struct {
  ContractHeader header;
  uint32_t component_id;
  uint32_t last_error_code;
  uint32_t consecutive_error_count;
  uint64_t last_success_time_us;
  uint64_t last_heartbeat_time_us;
  uint8_t health_state;
  uint8_t reserved[7];
} ModuleHealth;

typedef ModuleHealth FmsHealth;
typedef ModuleHealth GuidanceHealth;
typedef ModuleHealth ActuatorHealth;

typedef struct {
  ModuleHealth common;
  uint64_t last_imu_sample_us;
  uint32_t propagation_age_us;
  uint32_t rejected_measurement_count;
  uint32_t covariance_fault_count;
} EstimatorHealth;

typedef struct {
  ModuleHealth common;
  uint32_t solver_status;
  uint32_t deadline_miss_count;
  uint64_t last_valid_command_us;
  uint32_t max_input_age_us;
  uint32_t allocation_failure_count;
} ControlHealth;

/* ControlHealth.solver_status normalization (spec 02 section 4). */
#define CONTROL_SOLVER_NOT_RUN           0u
#define CONTROL_SOLVER_SOLVED            1u
#define CONTROL_SOLVER_SOLVED_INACCURATE 2u
#define CONTROL_SOLVER_MAX_ITER          3u
#define CONTROL_SOLVER_PRIMAL_INFEASIBLE 4u
#define CONTROL_SOLVER_DUAL_INFEASIBLE   5u
#define CONTROL_SOLVER_NUMERICAL_FAILURE 6u
#define CONTROL_SOLVER_DEADLINE_EXCEEDED 7u
#define CONTROL_SOLVER_INVALID_INPUT     8u

/* ------------------------------------------------------------------------- */
/* M1 link status and navigation origin (spec 02 sections 1.16, 1.16.1)       */
/* ------------------------------------------------------------------------- */

enum {
  LINK_STATE_UNKNOWN = 0, LINK_STATE_INITIALIZING, LINK_STATE_HEALTHY,
  LINK_STATE_DEGRADED, LINK_STATE_LOST, LINK_STATE_RECOVERING
};
enum {
  LINK_FLAG_RECEIVER_FAILSAFE = 1u << 0,
  LINK_FLAG_INTERFERENCE_SUSPECTED = 1u << 1,
  LINK_FLAG_TIME_UNCERTAIN = 1u << 2,
  LINK_FLAG_AUTHENTICATION_ERRORS = 1u << 3
};

/* Additional CommunicationLinkStatus validity bits (spec 02 section 1.4.3). */
#define VALIDITY_RSSI_VALID (1u << 8)
#define VALIDITY_LQ_VALID   (1u << 9)
#define VALIDITY_LOSS_VALID (1u << 10)

/* CommunicationLinkStatus.link_kind (spec 02 section 4). */
#define LINK_KIND_RC        1u
#define LINK_KIND_GCS       2u
#define LINK_KIND_COMPANION 3u
#define LINK_KIND_OTHER     4u

/* ContractHeader.validity_flags carries RSSI/LQ/packet-loss field validity. */
typedef struct {
  ContractHeader header;
  uint32_t link_instance_id;
  uint16_t link_kind;          /* RC / GCS / COMPANION / OTHER: Product Contract */
  uint16_t link_state;         /* UNKNOWN/INIT/HEALTHY/DEGRADED/LOST/RECOVERING */
  uint32_t reason_code;
  uint32_t valid_frame_count;
  uint32_t sequence_gap_count;
  uint32_t crc_error_count;
  uint32_t packet_loss_ppm;
  int16_t rssi_dbm;
  uint16_t link_quality_permille;
  uint64_t last_valid_frame_us;
  uint64_t last_rx_us;
  uint32_t recovery_stable_ms;
  uint32_t flags;              /* receiver failsafe / interference suspected / time uncertain */
} CommunicationLinkStatus;

typedef struct {
  ContractHeader header;
  uint32_t origin_id;
  uint32_t state_epoch;
  uint32_t valid_mask; /* bit0 LOCAL, bit1 WGS84_ANCHOR, bit2 GEOID_SEPARATION */
  uint32_t reserved;
  double latitude_deg;
  double longitude_deg;
  double ellipsoid_height_m;
  float geoid_separation_m; /* ellipsoid height minus AMSL */
  float local_radius_limit_m;
} NavigationOriginSnapshot;

/* ------------------------------------------------------------------------- */
/* Authority and operator input ownership (spec 02 sections 1.17)             */
/* ------------------------------------------------------------------------- */

typedef struct {
  ContractHeader header;
  uint32_t authority_epoch;
  uint16_t selected_source_id;
  uint16_t authority_class;
  uint32_t input_layout_id;
  ProductOperatorInput payload;
} AuthorizedOperatorInput;

typedef struct {
  ContractHeader header;
  uint32_t authority_epoch;
  uint16_t authority_owner;     /* Product Contract-defined source */
  uint16_t operation_mode;
  uint32_t selected_source_id;
  uint32_t flags;               /* takeover latched / failsafe active / recovery pending */
  uint16_t reason_code;
  uint16_t reserved;
} AuthorityStatus;

/* ------------------------------------------------------------------------- */
/* Fault evidence and mission upload (spec 02 sections 1.18, 1.21)            */
/* ------------------------------------------------------------------------- */

typedef struct {
  ContractHeader header;
  uint32_t event_id;
  uint16_t fault_domain;        /* PLATFORM/SENSOR/NAV/MOTION/CONTROL/ACTUATOR/COMM/ENERGY */
  uint16_t evidence_class;      /* DIRECT/CORROBORATING/MODEL_BASED/UNKNOWN */
  uint32_t source_component_id;
  uint32_t source_instance_id;
  uint32_t reason_code;
  uint32_t related_sequence;
  uint64_t first_seen_us;
  uint64_t last_seen_us;
  uint32_t occurrence_count;
  uint8_t severity;
  uint8_t active;
  uint16_t reserved;
} FaultEvidenceEvent;

/* Fault domain / evidence class / severity (spec 02 section 4). */
#define FAULT_DOMAIN_PLATFORM 0u
#define FAULT_DOMAIN_SENSOR   1u
#define FAULT_DOMAIN_NAV      2u
#define FAULT_DOMAIN_MOTION   3u
#define FAULT_DOMAIN_CONTROL  4u
#define FAULT_DOMAIN_ACTUATOR 5u
#define FAULT_DOMAIN_COMM     6u
#define FAULT_DOMAIN_ENERGY   7u

#define FAULT_EVIDENCE_UNKNOWN      0u
#define FAULT_EVIDENCE_DIRECT       1u
#define FAULT_EVIDENCE_CORROBORATING 2u
#define FAULT_EVIDENCE_MODEL_BASED  3u

#define FAULT_SEVERITY_INFO     0u
#define FAULT_SEVERITY_WARNING  1u
#define FAULT_SEVERITY_ERROR    2u
#define FAULT_SEVERITY_CRITICAL 3u

typedef struct {
  ContractHeader header;
  uint32_t transaction_id;
  uint32_t source_session_id;
  uint32_t state_epoch;
  uint32_t origin_id;
  uint32_t mission_hash;
  ProductMissionPayload mission;
} MissionUploadCandidate;

/* ------------------------------------------------------------------------- */
/* Control model view and environment constraint (spec 02 sections 4.5)       */
/* ------------------------------------------------------------------------- */

typedef struct {
  ContractHeader header;
  ControlModelDescriptor descriptor;
  ControlLinearModel linear_model;
  ControlModelBounds bounds;
  uint32_t model_epoch;
  uint32_t operating_point_id;
} ControlModelView;

typedef struct {
  ContractHeader header;
  uint16_t constraint_kind;
  uint16_t severity;
  uint32_t constraint_id;
  uint64_t constraint_expires_at_us;
  ProductEnvironmentConstraint payload;
} EnvironmentConstraint;

EXTERN_C_END

#endif /* COMMON_MESSAGES_H */
