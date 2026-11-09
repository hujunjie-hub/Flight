/*
 * Product-owned FMS policy constants (QUAD_X4 first version).
 *
 * These are controlled safety-policy values, not ordinary engineering tuning:
 * changing them requires CR + independent safety review (08/06). Values whose
 * flight evidence is not yet available are marked and must be treated as
 * fail-closed by the caller; no value here claims flight approval.
 */
#ifndef FMS_POLICY_H
#define FMS_POLICY_H

#include "fms_contract/fms_contract.h"

/* Profile-bound FMS implementation id (02 §4.3 allow-list). */
#define QUAD_X4_FMS_POLICY_ID 0x1101u

typedef struct {
  uint32_t fms_policy_id;        /* 0x1101 QUAD_X4 AERIAL_FMS */
  uint32_t safety_policy_id;     /* product safety policy id */
  uint32_t output_policy_id;     /* M7 output policy; 0 = not allowed to output */
  uint32_t required_arm_mask;    /* PreArm bit0..14 required for ARM */
  uint32_t required_position_mask; /* extra required bits for position modes */
  uint32_t geofence_required;    /* 0/1 */

  uint32_t directive_ttl_us;
  uint32_t status_ttl_us;
  uint32_t mission_ttl_us;
  uint32_t home_ttl_us;
  uint32_t landed_ttl_us;
  uint32_t command_ttl_us;
  uint32_t staging_timeout_us;
  uint32_t long_op_deadline_us;

  uint32_t arrival_dwell_us;
  float arrival_max_speed_mps;
  float battery_arm_min_fraction;
  float home_max_position_std_m;
  float geofence_horizontal_margin_m;
  float geofence_vertical_margin_m;

  float default_max_tilt_rad;
  float default_max_speed_mps;
  float default_max_accel_mps2;
  float default_max_yaw_rate_radps;
  float default_min_height_above_home_m;
  float default_max_height_above_home_m;
  float default_min_total_thrust_n;
  float default_max_total_thrust_n;
} FmsPolicy;

/* Provided by the selected product binding (middleware/model/fms). */
extern const FmsPolicy fms_quad_x4_policy;

/* Product binding hooks used by the core. */
Status fms_product_validate_profile(const ProductProfile *profile);
Status fms_product_validate_config(const ProductFmsConfig *config);
Status fms_product_build_safety_envelope(const FmsPolicy *policy,
                                        const ProductFmsConfig *config,
                                        uint32_t envelope_epoch,
                                        SafetyEnvelope *out);
uint32_t fms_product_mode_required_nav(uint8_t flight_mode);
uint32_t fms_product_mode_prearm_mask(const FmsPolicy *policy, uint8_t flight_mode);

#endif /* FMS_POLICY_H */
