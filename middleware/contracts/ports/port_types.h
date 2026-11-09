/*
 * Flight public contract - Port aggregate input/output types.
 *
 * Authority: spec/02_接口契约与公共数据规范.md (V1.9.1-GLOBAL-AUDIT-R3),
 *            section 4.5 "Port 输入/输出聚合类型" and section 4.8.
 *
 * These aggregates belong to the Port ABI.  Every pointer is valid only for
 * the duration of the synchronous Port call; implementations must copy any
 * history they need into their own fixed storage.  Inputs are read-only.
 *
 * FmsInputView/FmsOutputView, GuidanceInput/GuidanceOutput and
 * ControllerInput/ControllerOutput are large objects: the caller must place
 * them in the fixed arena defined in spec 07, never on a small thread stack.
 */
#ifndef PORT_TYPES_H
#define PORT_TYPES_H

#include "base_types/contract_version.h"
#include "messages/common_messages.h"
#include "ports/control_model_types.h"

EXTERN_C_BEGIN

/* ------------------------------------------------------------------------- */
/* Product profile identity (built by the profile resolver)                   */
/* ------------------------------------------------------------------------- */

typedef struct {
  uint32_t product_profile_id;
  uint32_t profile_version;
  uint32_t profile_hash;
  uint32_t nav_layout_id;
  uint32_t motion_reference_layout_id;
  uint32_t actuator_layout_id;
  uint32_t model_manifest_hash;
  uint32_t fms_policy_id;
  uint32_t estimator_impl_id;
  uint32_t guidance_impl_id;
  uint32_t controller_impl_id;
  uint32_t control_model_id;
  uint32_t actuator_adapter_id;
} ProductProfile;

/* ------------------------------------------------------------------------- */
/* M4 FMS policy views (spec 02 sections 1.21)                                */
/* ------------------------------------------------------------------------- */

enum {
  FMS_INPUT_HAS_REQUEST = 1u << 0,
  FMS_INPUT_HAS_OPERATOR = 1u << 1,
  FMS_INPUT_HAS_LINKS = 1u << 2,
  FMS_INPUT_HAS_NAV = 1u << 3,
  FMS_INPUT_HAS_NAV_CAPABILITY = 1u << 4,
  FMS_INPUT_HAS_MISSION_CANDIDATE = 1u << 5,
  FMS_INPUT_HAS_GUIDANCE = 1u << 6,
  FMS_INPUT_HAS_CONTROL = 1u << 7,
  FMS_INPUT_HAS_ACTUATOR = 1u << 8,
  FMS_INPUT_HAS_FAULT_SUMMARY = 1u << 9,
  FMS_INPUT_HAS_PRODUCT_EXTENSION = 1u << 10,
  FMS_INPUT_HAS_GUIDANCE_STATUS = 1u << 11
};

typedef struct {
  const OperationRequest *pending_requests;          /* fixed-capacity array view */
  uint16_t pending_request_count;                    /* <= PRODUCT_MAX_PENDING_OPERATION_REQUESTS */
  uint16_t reserved0;
  const OperatorInput *operator_input;                /* nullable only when not fresh */
  const CommunicationLinkStatus *link_statuses;
  uint16_t link_status_count;
  uint16_t reserved1;
  const NavigationSnapshot *navigation;
  const NavigationCapability *navigation_capability;
  const MissionUploadCandidate *mission_candidate;
  const GuidanceFeasibility *guidance_feasibility;
  const GuidanceStatus *guidance_status;
  const ControlCapability *control_capability;
  const ActuatorCapability *actuator_capability;
  const ActuatorStatus *actuator_status;
  const FaultSummary *fault_summary;
  uint32_t presence_mask;
  ProductFmsInputExtension product_extension;
  uint64_t evaluation_time_us;
} FmsInputView;

typedef struct {
  VehicleDirective directive;
  SafetyEnvelope safety_envelope;
  ActuationPermit actuation_permit;
  AuthorizedOperatorInput authorized_operator_input;
  AuthorityStatus authority_status;
  FmsStateSnapshot state_snapshot;
  MissionStatus mission_status;
  PreArmCheckResult prearm_result;
  LandedStateSnapshot landed_state;
  AuthorizedMissionSnapshot authorized_mission;
  ProductGeofenceSnapshot geofences[PRODUCT_MAX_GEOFENCE_POLYGONS];
  uint16_t geofence_count;
  uint16_t reserved_geofence;
  CommandAck command_results[PRODUCT_MAX_PENDING_OPERATION_REQUESTS];
  uint16_t command_result_count;
  uint16_t reserved;
  DecisionEvent decision_event;
} FmsOutputView;

/* Legacy source names are aliases only; no duplicate field layout is permitted. */
typedef FmsInputView FmsInputSnapshot;
typedef FmsOutputView FmsOutput;

/* ------------------------------------------------------------------------- */
/* M5 guidance Port                                                          */
/* ------------------------------------------------------------------------- */

typedef struct {
  const NavigationSnapshot *navigation;
  const NavigationCapability *navigation_capability;
  const VehicleDirective *directive;
  const SafetyEnvelope *safety_envelope;
  const LandedStateSnapshot *landed_state;
  const AuthorizedOperatorInput *authorized_operator_input;
  const AuthorityStatus *authority_status;
  const AuthorizedMissionSnapshot *authorized_mission;
  const ActuationPermit *active_permit; /* nullable only before first commit/disarmed */
  const ProductGeofenceSnapshot *geofences;
  uint16_t geofence_count;
  uint16_t observation_count;
  const ProductObstacleObservation *observations; /* <= profile bound, immutable during call */
  const EnvironmentConstraint *environment;
  uint64_t control_time_us;    /* navigation State Time; reference anchor */
  uint64_t evaluation_time_us; /* atomic clock read for TTL and deadlines */
} GuidanceInput;

typedef struct {
  MotionReference reference;
  PilotControlReference pilot_control_reference;
  GuidanceStatus guidance_status;
  GuidanceFeasibility feasibility;
} GuidanceOutput;

/* ------------------------------------------------------------------------- */
/* M6 controller Port                                                        */
/* ------------------------------------------------------------------------- */

typedef struct {
  const NavigationSnapshot *navigation;
  const MotionReference *reference;
  const PilotControlReference *pilot_control_reference;
  const GuidanceStatus *guidance_status;
  const VehicleDirective *directive;
  const SafetyEnvelope *safety_envelope;
  const AuthorityStatus *authority_status;
  const ActuationPermit *actuation_permit; /* required for every emitted demand */
  const ActuatorStatus *actuator_status;
  const ActuatorCapability *actuator_capability;
  const ActuatorCommitReceipt *actuator_commit_receipt; /* nullable, no physical feedback claim */
  const ControlModelView *control_model_view;
  uint64_t control_time_us;     /* selected State Time */
  uint64_t evaluation_time_us;  /* actual TimeService now for age/TTL/deadline */
} ControllerInput;

typedef struct {
  ActuatorDemand actuator_demand;
  ControlCapability capability;
  ControlHealth health;
} ControllerOutput;

/* ------------------------------------------------------------------------- */
/* PlantSimulationPort (Host/SIL/HIL only; never linked into H723 firmware)   */
/* ------------------------------------------------------------------------- */

typedef struct {
  uint32_t product_profile_id;
  uint32_t plant_model_id;
  uint32_t plant_model_version;
  uint32_t parameter_hash;
  float fixed_step_s;
} PlantSimConfig;

typedef struct {
  ProductPlantInitialState state;
} PlantInitialCondition;

typedef struct {
  uint64_t sample_time_us;
  ProductPlantTruth truth;
} PlantTruth;

typedef struct {
  ProductEnvironmentState state;
} EnvironmentInput;

typedef struct {
  uint16_t imu_count;
  uint16_t measurement_count;
  ImuSample imu_samples[PRODUCT_MAX_SIM_IMU_SAMPLES];
  SensorMeasurement measurements[PRODUCT_MAX_SIM_MEASUREMENTS];
} SimulatedMeasurementBatch;

EXTERN_C_END

#endif /* PORT_TYPES_H */
