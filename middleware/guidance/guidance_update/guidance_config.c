/* Default guidance parameters for runtime initialization and Host fixtures. */
#include <string.h>
#include "guidance_update/guidance.h"

void guidance_default_config(ProductGuidanceConfig *config)
{
  if (config == NULL) {
    return;
  }
  memset(config, 0, sizeof(*config));
  config->profile_hash = PRODUCT_PROFILE_ID;
  config->parameter_set_id = 1u;
  config->parameter_hash = 0x00000001u;
  config->config_version = 1u;
  config->payload.max_velocity_ned_mps[0] = 4.0f;
  config->payload.max_velocity_ned_mps[1] = 4.0f;
  config->payload.max_velocity_ned_mps[2] = 1.5f;
  config->payload.max_acceleration_ned_mps2[0] = 2.5f;
  config->payload.max_acceleration_ned_mps2[1] = 2.5f;
  config->payload.max_acceleration_ned_mps2[2] = 1.0f;
  config->payload.max_jerk_ned_mps3[0] = 10.0f;
  config->payload.max_jerk_ned_mps3[1] = 10.0f;
  config->payload.max_jerk_ned_mps3[2] = 5.0f;
  config->payload.vehicle_radius_m = 0.25f;
  config->payload.clearance_margin_m = 0.3f;
  config->payload.uncertainty_inflation_factor = 2.0f;
  config->payload.obstacle_source_mode = 0u; /* NONE: only non-avoidance missions */
  config->payload.planning_deadline_us = 20000u;
  config->payload.primitive_table_version = 1u;
  config->payload.max_dynamic_prediction_us = 1000000u;
  /* Host fixture defaults. Product integration supplies the common model. */
  config->payload.mass_kg = 1.0f;
  config->payload.gravity_mps2 = 9.80665f;
}
