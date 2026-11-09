/*
 * Flight product contract (Schema 6.0 / Product-Port 0x00010906).
 *
 * Authority: spec/02_接口契约与公共数据规范.md (V1.9.1-GLOBAL-AUDIT-R3),
 *            section 4.2 (QUAD_X4 normative types), section 4.5 (config
 *            payloads), section "QUAD_X4 操作员输入和 FMS 专用扩展" and
 *            section 4 "QUAD字段编码与数值解释补全".
 *
 * All capacities below are compile-time ABI constants.  They participate in
 * the ABI hash; changing one requires a Product Contract revision change,
 * an ICD CR, and regeneration of abi_lock.h.
 */
#ifndef PRODUCT_CONTRACT_H
#define PRODUCT_CONTRACT_H

#include "base_types/contract_version.h"
#include "base_types/common_base.h"

EXTERN_C_BEGIN

/* ------------------------------------------------------------------------- */
/* Profile identity, layout IDs and capacities                                */
/* ------------------------------------------------------------------------- */

#define PRODUCT_PROFILE_ID 0x00010001u
#define PRODUCT_CONTRACT_REVISION 0x00010906u /* native IMU acquisition boundary */
#define PRODUCT_ESTIMATOR_IMPL_ID 0x1201u
#define PRODUCT_GUIDANCE_IMPL_ID 0x1301u
#define PRODUCT_CONTROLLER_IMPL_ID 0x1401u
#define PRODUCT_CONTROL_MODEL_ID 0x1501u
#define PRODUCT_ACTUATOR_ADAPTER_ID 0x1601u

#define PRODUCT_PREVIEW_COUNT 24u
#define PRODUCT_MAX_MISSION_WAYPOINTS 32u
#define PRODUCT_MAX_GEOFENCE_POLYGONS 4u
#define PRODUCT_MAX_GEOFENCE_VERTICES 32u
#define PRODUCT_CONTROL_NX 6
#define PRODUCT_CONTROL_NU 3
#define PRODUCT_ACTUATOR_COUNT 4
#define PRODUCT_MAX_PENDING_OPERATION_REQUESTS 4u
#define PRODUCT_MAX_GUIDANCE_OBSERVATIONS_PER_UPDATE 8u
#define PRODUCT_MAX_LINK_INSTANCES 4u
#define PRODUCT_MAX_SIM_MEASUREMENTS 16
#define PRODUCT_MAX_SIM_IMU_SAMPLES 4u

#define PRODUCT_MISSION_REQUIRES_OBSTACLE_AVOIDANCE (1u << 0)

/* First-version layout identifiers (spec 02 section 4). */
#define PRODUCT_OPERATOR_INPUT_LAYOUT_ID        0x1001u
#define PRODUCT_NAV_LAYOUT_ID                   0x1002u
#define PRODUCT_MOTION_REFERENCE_LAYOUT_ID      0x1003u
#define PRODUCT_ACTUATOR_LAYOUT_ID              0x1004u
#define PRODUCT_CONTROL_MODEL_STATE_LAYOUT_ID   0x2001u
#define PRODUCT_CONTROL_MODEL_INPUT_LAYOUT_ID   0x2002u

/* QUAD_X4 operation codes used by OperationRequest.command_code (spec 02
 * section 1.5 / 06 command table).  0 and every other value are rejected. */
#define PRODUCT_CMD_ARM                1u
#define PRODUCT_CMD_DISARM             2u
#define PRODUCT_CMD_TAKEOFF            3u
#define PRODUCT_CMD_LAND               4u
#define PRODUCT_CMD_RETURN             5u
#define PRODUCT_CMD_SET_MODE           6u
#define PRODUCT_CMD_MISSION_COMMIT     7u
#define PRODUCT_CMD_MISSION_START      8u
#define PRODUCT_CMD_MISSION_PAUSE      9u
#define PRODUCT_CMD_MISSION_RESUME     10u
#define PRODUCT_CMD_MISSION_ABORT      11u
#define PRODUCT_CMD_MISSION_CLEAR      12u
#define PRODUCT_CMD_SET_HOME           13u
#define PRODUCT_CMD_SETTINGS_AUTHORIZE 14u

/* ProductOperationPayload.valid_mask (spec 02 section 1.4.3). */
#define PRODUCT_OPERATION_VALID_TARGET_NED          (1u << 0)
#define PRODUCT_OPERATION_VALID_TARGET_YAW          (1u << 1)
#define PRODUCT_OPERATION_VALID_ALTITUDE_ABOVE_HOME (1u << 2)
#define PRODUCT_OPERATION_VALID_REQUESTED_MODE      (1u << 3)
#define PRODUCT_OPERATION_VALID_MISSION_ID          (1u << 4)
#define PRODUCT_OPERATION_VALID_MISSION_TRANSACTION (1u << 5)
#define PRODUCT_OPERATION_VALID_SETTINGS_TRANSACTION (1u << 6)
#define PRODUCT_OPERATION_VALID_PREARM_EPOCH        (1u << 7)
/* CR-V191-C01: current mission_epoch; 0 = not applicable, bit must be 0. */
#define PRODUCT_OPERATION_VALID_MISSION_EPOCH       (1u << 8)

/* ------------------------------------------------------------------------- */
/* Mission, environment and navigation extension                              */
/* ------------------------------------------------------------------------- */

typedef struct {
  uint32_t waypoint_id;
  float position_ned_m[3];
  float yaw_rad;
  float acceptance_radius_m;
  float speed_limit_mps;
  uint32_t flags;
} ProductMissionWaypoint;

typedef struct {
  uint32_t mission_id;
  uint32_t mission_revision;
  uint32_t mission_hash;
  uint16_t waypoint_count;
  uint16_t flags;
  ProductMissionWaypoint waypoints[PRODUCT_MAX_MISSION_WAYPOINTS];
} ProductMissionPayload;

typedef struct {
  uint32_t obstacle_count;
  float min_clearance_m;
  float max_wind_mps;
} ProductEnvironmentConstraint;

typedef struct {
  uint32_t valid_mask;
  float baro_altitude_m;
  float terrain_range_m;
  float wind_ned_mps[3];
} ProductNavExtension;

/* ProductNavExtension.valid_mask (spec 02 section 1.4.3). */
#define PRODUCT_NAV_EXT_BARO_ALTITUDE_UP (1u << 0)
#define PRODUCT_NAV_EXT_TERRAIN_RANGE    (1u << 1)
#define PRODUCT_NAV_EXT_WIND             (1u << 2)

typedef struct {
  uint32_t mission_id;
  float target_ned_m[3];
  float target_yaw_rad;
  float requested_altitude_m;
  uint32_t valid_mask;
  uint16_t requested_mode;
  uint16_t operation_flags;
  uint32_t settings_transaction_id;
  uint32_t prearm_check_epoch;     /* required for Arm; other operations zero */
  uint32_t mission_transaction_id; /* links staged upload, otherwise zero */
  uint32_t mission_epoch;          /* CR-V191-C01: current epoch for START/PAUSE/RESUME/ABORT/CLEAR; 0 otherwise */
} ProductOperationPayload;

/* ------------------------------------------------------------------------- */
/* Home and geofence snapshots                                                */
/* ------------------------------------------------------------------------- */

typedef enum {
  PRODUCT_HOME_INVALID = 0,
  PRODUCT_HOME_VALID = 1
} ProductHomeValidity;

typedef enum {
  PRODUCT_HOME_QUALITY_GOOD = 0,
  PRODUCT_HOME_QUALITY_DEGRADED = 1
} ProductHomeQuality;

typedef enum {
  PRODUCT_HOME_CAPTURE_AUTO_ARM = 1,
  PRODUCT_HOME_CAPTURE_OPERATOR = 2
} ProductHomeCaptureSource;

typedef struct {
  uint32_t home_id;
  uint32_t home_revision;
  uint32_t frame_id;             /* FRAME_LOCAL_NED */
  uint32_t origin_id;
  uint32_t state_epoch;
  uint8_t validity;
  uint8_t quality;
  uint8_t capture_source;
  uint8_t reserved0;
  uint64_t captured_at_us;
  uint64_t home_expires_at_us;
  float position_ned_m[3];
  float position_std_m[3];
} ProductHomeTargetSnapshot;

typedef enum {
  PRODUCT_GEOFENCE_INVALID = 0,
  PRODUCT_GEOFENCE_VALID = 1
} ProductGeofenceValidity;

typedef enum {
  PRODUCT_GEOFENCE_KEEP_IN = 1,
  PRODUCT_GEOFENCE_KEEP_OUT = 2
} ProductGeofenceKind;

typedef struct {
  ContractHeader header;
  uint32_t fence_id;
  uint32_t fence_revision;
  uint32_t fence_hash;
  uint32_t frame_id;             /* FRAME_LOCAL_NED */
  uint32_t origin_id;
  uint32_t state_epoch;
  uint8_t validity;
  uint8_t kind;
  uint8_t vertex_count;
  uint8_t reserved0;
  float vertices_ne_m[PRODUCT_MAX_GEOFENCE_VERTICES][2];
  float min_height_above_home_m; /* positive up */
  float max_height_above_home_m; /* positive up */
  float horizontal_margin_m;
  float vertical_margin_m;
} ProductGeofenceSnapshot;

/* CR-V191-C02: compile-time geofence configuration.  Generated into the
 * Product Manifest/Profile .rodata and loaded by M4 at fms_policy_init; there
 * is no runtime SET_GEOFENCE command and Companion cannot modify fences. */
typedef struct {
  uint32_t fence_id;
  uint8_t kind;              /* ProductGeofenceKind */
  uint8_t vertex_count;
  uint16_t reserved0;
  float vertices_ne_m[PRODUCT_MAX_GEOFENCE_VERTICES][2];
  float min_height_above_home_m; /* positive up */
  float max_height_above_home_m; /* positive up */
  float horizontal_margin_m;
  float vertical_margin_m;
} ProductGeofenceDefinition;

typedef struct {
  uint32_t config_version;
  uint32_t fence_count;
  uint32_t geofence_hash;
  ProductGeofenceDefinition fences[PRODUCT_MAX_GEOFENCE_POLYGONS];
} ProductGeofenceConfigPayload;

typedef struct {
  uint32_t profile_hash;
  uint32_t config_version;
  uint32_t geofence_hash;
  ProductGeofenceConfigPayload payload;
} ProductGeofenceConfig;

typedef struct {
  ContractHeader header;
  uint32_t mission_epoch;
  uint32_t authority_epoch;
  uint32_t directive_epoch;
  uint32_t state_epoch;
  uint32_t origin_id;
  uint32_t mission_id;
  uint32_t mission_revision;
  uint32_t mission_hash;
  uint64_t mission_expires_at_us;
  uint32_t current_item_id;
  uint16_t current_item_index;
  uint16_t reserved;
  uint64_t execution_token;
  ProductMissionPayload mission;
} AuthorizedMissionSnapshot;

/* ------------------------------------------------------------------------- */
/* Directive, reference points and actuator payloads                          */
/* ------------------------------------------------------------------------- */

typedef struct {
  uint8_t flight_phase;
  uint8_t failsafe_policy;
  uint16_t reserved;
  float max_tilt_rad;
  float min_height_above_home_m; /* positive up */
  float max_height_above_home_m; /* positive up */
  ProductHomeTargetSnapshot home;
  uint32_t target_valid_mask;
  float target_position_ned_m[3];
  float target_yaw_rad;
} ProductDirectivePayload;

#define PRODUCT_DIRECTIVE_TARGET_POSITION (1u << 0)
#define PRODUCT_DIRECTIVE_TARGET_YAW      (1u << 1)

typedef struct {
  uint32_t valid_mask;
  float position_ned_m[3];
  float velocity_ned_mps[3];
  float acceleration_ned_mps2[3];
  float yaw_rad;
  float yaw_rate_radps;
} ProductMotionPoint;

/* ProductMotionPoint.valid_mask (spec 02 section 1.4.3). */
#define PRODUCT_POINT_VALID_POSITION    (1u << 0)
#define PRODUCT_POINT_VALID_VELOCITY    (1u << 1)
#define PRODUCT_POINT_VALID_ACCELERATION (1u << 2)
#define PRODUCT_POINT_VALID_YAW         (1u << 3)
#define PRODUCT_POINT_VALID_YAW_RATE    (1u << 4)

typedef struct {
  uint32_t valid_mask;
  float motor_thrust_n[4];
} ProductActuatorDemand;

typedef struct {
  uint32_t valid_mask;
  float motor_rpm[4];
  float motor_thrust_est_n[4];
} ProductActuatorFeedback;

typedef struct {
  float min_thrust_n[4];
  float max_thrust_n[4];
  float max_slew_nps[4];
} ProductActuatorCapability;

typedef struct {
  uint32_t valid_mask;
  float max_velocity_ned_mps[3];
  float max_acceleration_ned_mps2[3];
  float max_jerk_ned_mps3[3];
  float max_tilt_rad;
  float min_total_thrust_n;
  float max_total_thrust_n;
  float max_body_rate_frd_radps[3];
  float min_height_above_home_m;
  float max_height_above_home_m;
} ProductSafetyEnvelopePayload;

/* ProductSafetyEnvelopePayload.valid_mask (spec 02 section 1.4.3). */
#define PRODUCT_ENVELOPE_VALID_VELOCITY     (1u << 0)
#define PRODUCT_ENVELOPE_VALID_ACCELERATION (1u << 1)
#define PRODUCT_ENVELOPE_VALID_JERK         (1u << 2)
#define PRODUCT_ENVELOPE_VALID_TILT         (1u << 3)
#define PRODUCT_ENVELOPE_VALID_THRUST       (1u << 4)
#define PRODUCT_ENVELOPE_VALID_BODY_RATE    (1u << 5)
#define PRODUCT_ENVELOPE_VALID_HEIGHT       (1u << 6)

typedef struct {
  uint32_t valid_mask;
  float body_rate_frd_radps[3];   /* RATE reference */
  float q_nb_desired_wxyz[4];     /* ATTITUDE reference */
  float yaw_rate_ff_radps;
  float collective_throttle_norm; /* [0,1], pilot intent, not motor output */
  float altitude_above_origin_m;  /* positive Up, ALTITUDE_ATTITUDE only */
  float vertical_velocity_up_mps; /* ALTITUDE_ATTITUDE only */
} ProductPilotControlReferencePayload;

/* ProductPilotControlReferencePayload.valid_mask (spec 02 section 1.4.3). */
#define PRODUCT_PILOT_REF_VALID_RATE          (1u << 0)
#define PRODUCT_PILOT_REF_VALID_ATTITUDE      (1u << 1)
#define PRODUCT_PILOT_REF_VALID_YAW_FF        (1u << 2)
#define PRODUCT_PILOT_REF_VALID_THROTTLE      (1u << 3)
#define PRODUCT_PILOT_REF_VALID_ALTITUDE_UP   (1u << 4)
#define PRODUCT_PILOT_REF_VALID_VERTICAL_UP   (1u << 5)

/* ------------------------------------------------------------------------- */
/* Obstacle observation (M2 -> M5 environment input)                          */
/* ------------------------------------------------------------------------- */

typedef enum {
  PRODUCT_OBS_OCCUPIED_PRIMITIVE = 1,
  PRODUCT_OBS_FREE_SPACE_RAY = 2
} ProductObstacleObservationKind;

typedef enum {
  PRODUCT_OBS_SHAPE_POINT = 1,
  PRODUCT_OBS_SHAPE_SPHERE = 2,
  PRODUCT_OBS_SHAPE_AABB = 3
} ProductObstacleShape;

typedef enum {
  PRODUCT_OBS_SOURCE_NORMAL = 0,
  PRODUCT_OBS_SOURCE_DEGRADED = 1,
  PRODUCT_OBS_SOURCE_REJECT = 2
} ProductObstacleSourceHealth;

typedef struct {
  ContractHeader header;
  uint32_t observation_id;
  uint32_t sensor_instance_id;
  uint32_t source_health_epoch;
  uint32_t origin_id;
  uint32_t state_epoch;
  uint16_t observation_kind;
  uint16_t shape_kind;
  uint8_t source_health;
  uint8_t reserved0[3];
  uint32_t quality_flags;
  float obstacle_center_ned_m[3];
  float obstacle_half_extent_m[3];
  float ray_start_ned_m[3];
  float ray_end_ned_m[3];
  float obstacle_velocity_ned_mps[3];
  float position_std_m[3];
  float velocity_std_mps[3];
  float confidence_0_to_1;
} ProductObstacleObservation;

/* ------------------------------------------------------------------------- */
/* Strongly typed tagged union of enabled asynchronous measurements           */
/* ------------------------------------------------------------------------- */

typedef union {
  GnssSample gnss;
  BaroSample baro;
  MagSample mag;
  RangeSample range;
  FlowSample flow;
} ProductSensorMeasurement;

/* SensorMeasurement.measurement_kind (spec 02 section 4). */
#define PRODUCT_MEASUREMENT_GNSS  1u
#define PRODUCT_MEASUREMENT_BARO  2u
#define PRODUCT_MEASUREMENT_MAG   3u
#define PRODUCT_MEASUREMENT_RANGE 4u
#define PRODUCT_MEASUREMENT_FLOW  5u

/* ------------------------------------------------------------------------- */
/* Host/SIL/HIL plant truth types (never linked into H723 firmware)           */
/* ------------------------------------------------------------------------- */

typedef struct {
  float position_ned_m[3];
  float velocity_ned_mps[3];
  float q_nb_wxyz[4];
  float body_rate_frd_radps[3];
  float motor_omega_radps[4];
} ProductPlantInitialState;

typedef struct {
  float position_ned_m[3];
  float velocity_ned_mps[3];
  float q_nb_wxyz[4];
  float body_rate_frd_radps[3];
  float motor_omega_radps[4];
} ProductPlantTruth;

typedef struct {
  float wind_ned_mps[3];
  float air_density_kgpm3;
} ProductEnvironmentState;

/* ------------------------------------------------------------------------- */
/* Operator input and FMS product facts (QUAD only)                           */
/* ------------------------------------------------------------------------- */

typedef struct {
  float roll_norm;       /* [-1, +1], calibrated */
  float pitch_norm;      /* [-1, +1], calibrated */
  float yaw_norm;        /* [-1, +1], calibrated */
  float throttle_norm;   /* QUAD exactly [0,1]; no alternative scale */
  uint32_t mode_switch_bits;
  uint32_t aux_switch_bits;
} ProductOperatorInput;

/* OperatorInput.valid_axes_mask (spec 02 section 4). */
#define PRODUCT_AXIS_VALID_ROLL     (1u << 0)
#define PRODUCT_AXIS_VALID_PITCH    (1u << 1)
#define PRODUCT_AXIS_VALID_YAW      (1u << 2)
#define PRODUCT_AXIS_VALID_THROTTLE (1u << 3)

/* OperatorInput.mode_switch_bits: low 4 bits request FlightMode, bit 4 is an
 * explicit takeover edge; bit 5 marks a mapped RC mode position. */
#define PRODUCT_MODE_SWITCH_MASK 0x0fu
#define PRODUCT_MODE_TAKEOVER_EDGE (1u << 4)
#define PRODUCT_MODE_SWITCH_MAPPED (1u << 5)

typedef struct {
  ContractHeader header;
  uint32_t valid_mask;
  uint8_t ground_contact_state; /* 0=UNKNOWN,1=CONTACT,2=NO_CONTACT; only verified source */
  uint8_t landing_zone_valid;
  uint8_t geofence_state;
  uint8_t energy_state;
  float battery_remaining_fraction;
  float estimated_return_energy_fraction;
  float estimated_landing_energy_fraction;
  float wind_ned_mps[3];
  float wind_uncertainty_mps;
} ProductFmsInputExtension;

/* ProductFmsInputExtension.valid_mask (spec 02 section 1.4.3). */
#define PRODUCT_FMS_EXT_GROUND_CONTACT  (1u << 0)
#define PRODUCT_FMS_EXT_LANDING_ZONE    (1u << 1)
#define PRODUCT_FMS_EXT_GEOFENCE_STATE  (1u << 2)
#define PRODUCT_FMS_EXT_ENERGY_STATE    (1u << 3)
#define PRODUCT_FMS_EXT_BATTERY_FRACTION (1u << 4)
#define PRODUCT_FMS_EXT_RETURN_ENERGY   (1u << 5)
#define PRODUCT_FMS_EXT_LANDING_ENERGY  (1u << 6)
#define PRODUCT_FMS_EXT_WIND            (1u << 7)

/* ------------------------------------------------------------------------- */
/* M2 read-only installation/calibration configuration (generated .rodata)    */
/* ------------------------------------------------------------------------- */

typedef struct {
  uint32_t config_version;
  uint32_t calibration_version;
  double imu_to_frd_rotation[9]; /* row-major proper SO(3) */
  double mag_to_frd_rotation[9];
  double imu_bias_gyro_radps[3];
  double imu_bias_accel_mps2[3];
  double imu_scale_gyro[3]; /* fractional correction, denominator 1+s */
  double imu_scale_accel[3];
  double mag_hard_iron_ut[3];
  double mag_soft_iron_matrix[9];
  float imu_axis_delay_us[6]; /* gyro xyz then accel xyz, physical group delay */
  float imu_delay_uncertainty_us[6];
  float baro_delay_us;
  float baro_delay_uncertainty_us;
  float mag_delay_us;
  float mag_delay_uncertainty_us;
  uint32_t verified_sensor_mask;
  uint32_t reserved;
} QuadSensorProcessingConfig;

/* ------------------------------------------------------------------------- */
/* Port configuration payloads (immutable .rodata; validation precedes use)   */
/* ------------------------------------------------------------------------- */

typedef struct {
  uint32_t safety_policy_id;
  uint32_t output_policy_id;
  uint32_t verified_input_mask;
  uint32_t rc_lost_timeout_us;
  uint32_t rc_recovery_dwell_us;
  uint32_t landed_dwell_us;
  uint32_t first_reference_timeout_us;
  uint32_t nav_enter_dwell_us;
  float attitude_std_limit_rad;
  float heading_std_limit_rad;
  float horizontal_position_std_limit_m;
  float vertical_position_std_limit_m;
  float horizontal_velocity_std_limit_mps;
  float vertical_velocity_std_limit_mps;
  float nav_enter_ratio; /* 0 < ratio < 1; exit at limit, enter below ratio*limit */
  uint32_t geofence_required;
} ProductFmsConfigPayload;

typedef struct {
  double initial_covariance_upper[253]; /* row-major upper triangle, 22x22 */
  double gyro_noise_density_radps_per_sqrt_hz[3];
  double accel_noise_density_mps2_per_sqrt_hz[3];
  double gyro_bias_std_radps[3];
  double accel_bias_std_mps2[3];
  double gyro_scale_std[3];
  double accel_scale_std[3];
  double bias_correlation_time_s[2]; /* gyro, accel; both strictly positive */
  double scale_correlation_time_s[2];
  double baro_bias_rw_psd_m2ps;
  double gnss_position_floor_m[3];
  double gnss_velocity_floor_mps[3];
  double baro_floor_m;
  double mag_heading_floor_rad;
  double antenna_lever_frd_m[3];
  double imu_to_vehicle_reference_frd_m[3]; /* IMU center -> CG, derived from vehicle model */
  double magnetic_declination_rad;
  double magnetic_declination_variance_rad2;
  double min_horizontal_field_ut;
  double initial_latitude_deg;
  double initial_longitude_deg;
  double initial_ellipsoid_height_m;
  float local_radius_limit_m;
  uint32_t initial_origin_valid_mask; /* bit0 LOCAL, bit1 WGS84; no geoid claim */
  uint32_t aiding_mask; /* GNSS_POSITION, GNSS_VELOCITY, BARO, MAG; see registry */
  uint32_t calibration_version;
} ProductEstimatorConfigPayload;

/* ProductEstimatorConfigPayload.aiding_mask (spec 02 section 1.4.3). */
#define PRODUCT_AIDING_GNSS_POSITION (1u << 0)
#define PRODUCT_AIDING_GNSS_VELOCITY (1u << 1)
#define PRODUCT_AIDING_BARO          (1u << 2)
#define PRODUCT_AIDING_MAG           (1u << 3)

typedef struct {
  float max_velocity_ned_mps[3];
  float max_acceleration_ned_mps2[3];
  float max_jerk_ned_mps3[3];
  float vehicle_radius_m;
  float clearance_margin_m;
  float uncertainty_inflation_factor;
  uint32_t obstacle_source_mode; /* 0 NONE,1 STATIC_MAP,2 SIMULATION; ONBOARD unsupported */
  uint32_t planning_deadline_us;
  uint32_t primitive_table_version;
  uint32_t max_dynamic_prediction_us;
  float mass_kg;              /* same model data as M6/Host Plant, never inferred */
  float gravity_mps2;
} ProductGuidanceConfigPayload;

typedef struct {
  float mass_kg;
  float inertia_frd_kgm2[9]; /* row-major symmetric positive definite */
  float motor_position_frd_m[4][3];
  float motor_yaw_torque_ratio_m[4];
  float so3_gain_per_s[3];
  float rate_kp_nm_per_radps[3];
  float rate_ki_nm_per_rad[3];
  float rate_kd_nm_per_radps2[3];
  float rate_kaw_per_s[3];
  float rate_d_cutoff_hz;
  float rate_integral_limit_nm[3];
  float fallback_kp_per_s2[3];
  float fallback_kd_per_s[3];
  float mpc_q_diag[6], mpc_qf_diag[6], mpc_r_diag[3], mpc_s_diag[3];
  float actuation_preview_delay_s;
  float manual_hover_throttle_norm;
  float mpc_seed_position_error_limit_m;
  float mpc_seed_velocity_error_limit_mps;
} ProductControllerConfigPayload;

typedef struct {
  uint32_t output_policy_id;
  uint32_t esc_bit_rate; /* 300000 or 600000; default first profile 300000 */
  uint32_t calibration_version;
  uint8_t motor_to_timer_channel[4]; /* permutation 1..4 */
  uint16_t thrust_curve_count[4]; /* each 2..16; strictly monotonic */
  float thrust_curve_n[4][16];
  uint16_t dshot_curve_code[4][16]; /* throttle range 48..2047 only */
  float min_thrust_n[4], max_thrust_n[4], max_slew_nps[4];
} ProductActuatorConfigPayload;

/* The product contract defines these payload structs with its own
 * typed parameters. */
typedef struct {
  uint32_t profile_hash;
  uint32_t parameter_set_id;
  uint32_t parameter_hash;
  uint32_t config_version;
  ProductFmsConfigPayload payload;
} ProductFmsConfig;

typedef struct {
  uint32_t profile_hash;
  uint32_t parameter_set_id;
  uint32_t parameter_hash;
  uint32_t config_version;
  ProductEstimatorConfigPayload payload;
} ProductEstimatorConfig;

typedef struct {
  uint32_t profile_hash;
  uint32_t parameter_set_id;
  uint32_t parameter_hash;
  uint32_t config_version;
  ProductGuidanceConfigPayload payload;
} ProductGuidanceConfig;

typedef struct {
  uint32_t profile_hash;
  uint32_t parameter_set_id;
  uint32_t parameter_hash;
  uint32_t config_version;
  ProductControllerConfigPayload payload;
} ProductControllerConfig;

typedef struct {
  uint32_t profile_hash;
  uint32_t calibration_set_id;
  uint32_t calibration_hash;
  uint32_t config_version;
  ProductActuatorConfigPayload payload;
} ProductActuatorConfig;

/* ------------------------------------------------------------------------- */
/* Product contract invariants                                                */
/* ------------------------------------------------------------------------- */

CONTRACT_STATIC_ASSERT(PRODUCT_CONTRACT_REVISION == PORT_API_REVISION,
                 "product contract and Port revision must be released together");
CONTRACT_STATIC_ASSERT(PRODUCT_PREVIEW_COUNT == 24u,
                 "QUAD MotionReference preview depth changed");
CONTRACT_STATIC_ASSERT(PRODUCT_CONTROL_NX * PRODUCT_CONTROL_NX == 36,
                 "control model state dimension changed");
CONTRACT_STATIC_ASSERT(PRODUCT_ACTUATOR_COUNT == 4,
                 "QUAD actuator count changed");

EXTERN_C_END

#endif /* PRODUCT_CONTRACT_H */
