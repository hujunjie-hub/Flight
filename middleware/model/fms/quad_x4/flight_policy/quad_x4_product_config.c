/*
 * QUAD_X4 product policy and SafetyEnvelope source.
 *
 * This translation unit owns the product-owned safety values and the one
 * function that turns them into the SafetyEnvelope consumed by M5/M6. It has no
 * dependency on the FMS state machine, so the Host/SIL M5-06 validator tests
 * can link the real production builder instead of maintaining a second copy of
 * the limits.
 *
 * Evidence status: every threshold below is a first-version policy constant
 * owned by C. Values that still need airframe evidence are marked
 * SOURCE_EVIDENCE_PENDING in the delivery report; none of them claims flight
 * approval, and PreArm fails closed when the matching verified_input bit is
 * absent.
 */
#include <string.h>
#include "fms_contract/fms_model.h"
#include "flight_policy/fms_policy.h"

const FmsPolicy fms_quad_x4_policy = {
  .fms_policy_id = QUAD_X4_FMS_POLICY_ID,
  .safety_policy_id = 0x00010001u,
  .output_policy_id = 2u, /* SIM_CONTROLLED_STOP; real policies >=0x100 need registration */
  .required_arm_mask = (1u << PREARM_ID_PLATFORM_READY) | (1u << PREARM_ID_IMU_READY) |
                       (1u << PREARM_ID_ESTIMATOR_ATTITUDE) |
                       (1u << PREARM_ID_ESTIMATOR_POSITION) | (1u << PREARM_ID_RC_INPUT) |
                       (1u << PREARM_ID_ACTUATOR_OUTPUT_READY) |
                       (1u << PREARM_ID_BATTERY_ENERGY) |
                       (1u << PREARM_ID_GNSS_HOME_ORIGIN) |
                       (1u << PREARM_ID_COMPASS_HEADING) | (1u << PREARM_ID_CONTROL_MODEL) |
                       (1u << PREARM_ID_CONTROL_SOLVER) | (1u << PREARM_ID_SAFETY_ENVELOPE) |
                       (1u << PREARM_ID_STORAGE_EVENT_LOG),
  .required_position_mask = (1u << PREARM_ID_GEOFENCE),
  .geofence_required = 0u,
  .directive_ttl_us = 100000u,
  .status_ttl_us = 200000u,
  .mission_ttl_us = 500000u,
  .home_ttl_us = 5000000u,
  .landed_ttl_us = 200000u,
  .command_ttl_us = 1000000u,
  .staging_timeout_us = 10000000u,
  .long_op_deadline_us = 120000000u,
  .arrival_dwell_us = 200000u,
  .arrival_max_speed_mps = 0.5f,
  .battery_arm_min_fraction = 0.30f,
  .home_max_position_std_m = 2.0f,
  .geofence_horizontal_margin_m = 1.0f,
  .geofence_vertical_margin_m = 0.5f,
  .default_max_tilt_rad = 0.5f,
  .default_max_speed_mps = 8.0f,
  .default_max_accel_mps2 = 4.0f,
  .default_max_yaw_rate_radps = 2.0f,
  .default_min_height_above_home_m = 0.3f,
  .default_max_height_above_home_m = 50.0f,
  .default_min_total_thrust_n = 4.0f,
  .default_max_total_thrust_n = 20.0f
};

Status fms_product_build_safety_envelope(const FmsPolicy *policy,
                                        const ProductFmsConfig *config,
                                        uint32_t envelope_epoch,
                                        SafetyEnvelope *out)
{
  ProductSafetyEnvelopePayload *ve;

  if (policy == NULL || config == NULL || out == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  memset(out, 0, sizeof(*out));
  ve = &out->vehicle_extension;
  ve->valid_mask = 0x7Fu; /* VELOCITY|ACCELERATION|JERK|TILT|THRUST|BODY_RATE|HEIGHT */
  ve->max_velocity_ned_mps[0] = policy->default_max_speed_mps;
  ve->max_velocity_ned_mps[1] = policy->default_max_speed_mps;
  ve->max_velocity_ned_mps[2] = policy->default_max_speed_mps * 0.5f;
  ve->max_acceleration_ned_mps2[0] = policy->default_max_accel_mps2;
  ve->max_acceleration_ned_mps2[1] = policy->default_max_accel_mps2;
  ve->max_acceleration_ned_mps2[2] = policy->default_max_accel_mps2 * 0.5f;
  ve->max_jerk_ned_mps3[0] = policy->default_max_accel_mps2 * 4.0f;
  ve->max_jerk_ned_mps3[1] = policy->default_max_accel_mps2 * 4.0f;
  ve->max_jerk_ned_mps3[2] = policy->default_max_accel_mps2 * 2.0f;
  ve->max_tilt_rad = policy->default_max_tilt_rad;
  ve->min_total_thrust_n = policy->default_min_total_thrust_n;
  ve->max_total_thrust_n = config->payload.output_policy_id == 2u
                              ? 30.0f : policy->default_max_total_thrust_n;
  ve->max_body_rate_frd_radps[0] = policy->default_max_yaw_rate_radps;
  ve->max_body_rate_frd_radps[1] = policy->default_max_yaw_rate_radps;
  ve->max_body_rate_frd_radps[2] = policy->default_max_yaw_rate_radps;
  ve->min_height_above_home_m = policy->default_min_height_above_home_m;
  ve->max_height_above_home_m = policy->default_max_height_above_home_m;
  out->safety_envelope_id = 1u;
  out->envelope_epoch = envelope_epoch;
  out->max_linear_speed_mps = policy->default_max_speed_mps;
  out->max_linear_accel_mps2 = policy->default_max_accel_mps2;
  out->max_angular_rate_radps = policy->default_max_yaw_rate_radps;
  out->max_yaw_rate_radps = policy->default_max_yaw_rate_radps;
  out->header.schema_major = CONTRACT_SCHEMA_MAJOR;
  out->header.schema_minor = CONTRACT_SCHEMA_MINOR;
  out->header.message_type = MSG_SAFETY_ENVELOPE;
  out->header.source_id = 21u;
  out->header.profile_id = PRODUCT_PROFILE_ID;
  out->header.frame_id = FRAME_LOCAL_NED;
  return STATUS_OK;
}

void fms_model_default_config(ProductFmsConfig *config)
{
  if (config == NULL) {
    return;
  }
  memset(config, 0, sizeof(*config));
  config->profile_hash = 0x00010001u;
  config->parameter_set_id = 1u;
  config->parameter_hash = 0x00000001u;
  config->config_version = 1u;
  config->payload.safety_policy_id = 0x00010001u;
  config->payload.output_policy_id = 2u;
  config->payload.verified_input_mask = (1u << PREARM_ID_COUNT) - 1u;
  config->payload.rc_lost_timeout_us = 300000u;
  config->payload.rc_recovery_dwell_us = 300000u;
  config->payload.landed_dwell_us = 2000000u;
  config->payload.first_reference_timeout_us = 100000u;
  config->payload.nav_enter_dwell_us = 200000u;
  config->payload.attitude_std_limit_rad = 0.15f;
  config->payload.heading_std_limit_rad = 0.35f;
  config->payload.horizontal_position_std_limit_m = 1.0f;
  config->payload.vertical_position_std_limit_m = 1.0f;
  config->payload.horizontal_velocity_std_limit_mps = 0.5f;
  config->payload.vertical_velocity_std_limit_mps = 0.5f;
  config->payload.nav_enter_ratio = 0.8f;
  config->payload.geofence_required = 0u;
}
