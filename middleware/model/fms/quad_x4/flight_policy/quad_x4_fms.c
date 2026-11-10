/*
 * QUAD_X4 AERIAL_FMS: Port implementation and product policy tables.
 *
 * Evidence status: every threshold below is a first-version policy constant
 * owned by C. Values that still need airframe evidence are marked
 * SOURCE_EVIDENCE_PENDING in the delivery report; none of them claims flight
 * approval, and PreArm fails closed when the matching verified_input bit is
 * absent.
 */
#include <string.h>
#include <math.h>
#include "contract_checks/algorithm_checks.h"
#include "flight_state_machine/fms_internal.h"
#include "fms_contract/fms_model.h"
#ifdef FLIGHT_HOST_SIL
#include "geofence_geometry/geofence_geometry.h"
#endif

uint32_t fms_model_policy_id(void)
{
  return fms_quad_x4_policy.fms_policy_id;
}

static FmsEngine g_fms;
static uint8_t g_initialized = 0u;

Status fms_product_validate_profile(const ProductProfile *profile)
{
  if (profile == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  if (profile->product_profile_id != PRODUCT_PROFILE_ID ||
      profile->nav_layout_id != PRODUCT_NAV_LAYOUT_ID ||
      profile->motion_reference_layout_id != PRODUCT_MOTION_REFERENCE_LAYOUT_ID ||
      profile->actuator_layout_id != PRODUCT_ACTUATOR_LAYOUT_ID) {
    return STATUS_PROFILE_MISMATCH;
  }
  if (profile->fms_policy_id != QUAD_X4_FMS_POLICY_ID) {
    return STATUS_PROFILE_MISMATCH;
  }
  return STATUS_OK;
}

static uint8_t fms_mask_only_bits(uint32_t mask, uint32_t allowed)
{
  return (uint8_t)((mask & ~allowed) == 0u);
}

Status fms_product_validate_config(const ProductFmsConfig *config)
{
  const ProductFmsConfigPayload *p;

  if (config == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  if (config->config_version == 0u || config->parameter_hash == 0u ||
      config->profile_hash == 0u) {
    return STATUS_INVALID_ARGUMENT;
  }
  p = &config->payload;
  if (p->safety_policy_id == 0u) {
    return STATUS_PROFILE_MISMATCH;
  }
  /* 0 = no output allowed; 1/2 are bench/simulation policies. Real strategies
   * (>=0x100) need a registered verification record that this build cannot
   * prove, so they are rejected rather than silently accepted. */
  if (p->output_policy_id > 2u) {
    return STATUS_UNSUPPORTED;
  }
  if (!fms_mask_only_bits(p->verified_input_mask, (1u << PREARM_ID_COUNT) - 1u)) {
    return STATUS_OUT_OF_RANGE;
  }
  if (p->rc_lost_timeout_us < 50000u || p->rc_lost_timeout_us > 2000000u) {
    return STATUS_OUT_OF_RANGE;
  }
  if (p->rc_recovery_dwell_us > 5000000u) {
    return STATUS_OUT_OF_RANGE;
  }
  if (p->landed_dwell_us < 500000u || p->landed_dwell_us > 10000000u) {
    return STATUS_OUT_OF_RANGE;
  }
  if (p->first_reference_timeout_us < 50000u || p->first_reference_timeout_us > 200000u) {
    return STATUS_OUT_OF_RANGE;
  }
  if (p->nav_enter_dwell_us > 5000000u) {
    return STATUS_OUT_OF_RANGE;
  }
  if (!fms_finite(p->attitude_std_limit_rad) || p->attitude_std_limit_rad <= 0.0f ||
      !fms_finite(p->heading_std_limit_rad) || p->heading_std_limit_rad <= 0.0f ||
      !fms_finite(p->horizontal_position_std_limit_m) ||
      p->horizontal_position_std_limit_m <= 0.0f ||
      !fms_finite(p->vertical_position_std_limit_m) ||
      p->vertical_position_std_limit_m <= 0.0f ||
      !fms_finite(p->horizontal_velocity_std_limit_mps) ||
      p->horizontal_velocity_std_limit_mps <= 0.0f ||
      !fms_finite(p->vertical_velocity_std_limit_mps) ||
      p->vertical_velocity_std_limit_mps <= 0.0f) {
    return STATUS_NUMERIC_ERROR;
  }
  if (!fms_finite(p->nav_enter_ratio) || p->nav_enter_ratio <= 0.0f ||
      p->nav_enter_ratio >= 1.0f) {
    return STATUS_NUMERIC_ERROR;
  }
  if (p->geofence_required > 1u) {
    return STATUS_OUT_OF_RANGE;
  }
  return STATUS_OK;
}

uint32_t fms_product_mode_required_nav(uint8_t flight_mode)
{
  switch (flight_mode) {
  case FLIGHT_MODE_MANUAL_RATE:
    return NAV_VALID_BODY_RATE;
  case FLIGHT_MODE_MANUAL_ATTITUDE:
    return NAV_VALID_ATTITUDE | NAV_VALID_BODY_RATE;
  case FLIGHT_MODE_ALT_HOLD:
    return NAV_VALID_ATTITUDE | NAV_VALID_BODY_RATE | NAV_VALID_VERTICAL_POSITION |
           NAV_VALID_VERTICAL_VELOCITY;
  case FLIGHT_MODE_POS_HOLD:
  case FLIGHT_MODE_AUTO_MISSION:
  case FLIGHT_MODE_AUTO_RETURN:
    return NAV_VALID_ATTITUDE | NAV_VALID_BODY_RATE | NAV_VALID_HEADING |
           NAV_VALID_HORIZONTAL_POSITION | NAV_VALID_VERTICAL_POSITION |
           NAV_VALID_HORIZONTAL_VELOCITY | NAV_VALID_VERTICAL_VELOCITY;
  case FLIGHT_MODE_AUTO_LAND:
  case FLIGHT_MODE_EMERGENCY_LAND:
    return NAV_VALID_ATTITUDE | NAV_VALID_BODY_RATE | NAV_VALID_VERTICAL_POSITION |
           NAV_VALID_VERTICAL_VELOCITY;
  default:
    return 0u;
  }
}

uint32_t fms_product_mode_prearm_mask(const FmsPolicy *policy, uint8_t flight_mode)
{
  uint32_t mask = policy->required_arm_mask;
  if (flight_mode == FLIGHT_MODE_POS_HOLD || flight_mode == FLIGHT_MODE_AUTO_MISSION ||
      flight_mode == FLIGHT_MODE_AUTO_RETURN) {
    mask |= policy->required_position_mask;
  }
  return mask;
}


/* Compile-time geometry is installed directly into M4's owned storage. */
static uint8_t g_geo_configured;

#ifdef FLIGHT_HOST_SIL
/* Called between ticks by the single-threaded SIL scheduler. The board build
 * still installs its product fence table before FMS initialization. */
Status fms_sil_replace_geofences(const ProductGeofenceSnapshot *fences, uint16_t count)
{
  if (!g_initialized || g_fms.config.payload.output_policy_id != 2u ||
      g_fms.flight.arm_state != ARM_STATE_DISARMED)
    return STATUS_NOT_PERMITTED;
  if (count > FMS_MAX_GEOFENCES || (count && !fences))
    return STATUS_INVALID_ARGUMENT;
  for (uint16_t index = 0; index < count; ++index)
    if (!fence_geometry_valid(&fences[index]))
      return STATUS_INVALID_ARGUMENT;
  FmsGeofence next = {.fence_count = count, .revision = g_fms.geofence.revision + 1u};
  for (uint16_t index = 0; index < count; ++index) {
    next.fences[index] = fences[index];
    next.fences[index].fence_revision = next.revision;
  }
  if (count) {
    const float home_down = g_fms.home.valid ? g_fms.home.snap.position_ned_m[2] : 0.0f;
    for (uint16_t index = 0; index < g_fms.mission.wp_count; ++index)
      if (!fms_geofence_contains(&next, g_fms.mission.active.waypoints[index].position_ned_m, home_down))
        return STATUS_NOT_PERMITTED;
  }
  g_fms.geofence = next;
  g_fms.config.payload.geofence_required = count != 0;
  return STATUS_OK;
}
#endif

static void fms_geofence_from_definition(const ProductGeofenceDefinition *def,
                                        const ProductGeofenceConfig *config,
                                        ProductGeofenceSnapshot *out);

/* CR-V191-C02: compile-time product geofence configuration (Product
 * Manifest/Profile). Installed before fms_model_init(); no runtime command. */
Status fms_model_configure_product_geofence(const ProductGeofenceConfig *config)
{
  if (g_initialized != 0u) {
    return STATUS_NOT_PERMITTED;
  }
  if (config == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  if (config->profile_hash != PRODUCT_PROFILE_ID) {
    return STATUS_PROFILE_MISMATCH;
  }
  if (config->config_version == 0u || config->geofence_hash == 0u ||
      config->config_version != config->payload.config_version ||
      config->geofence_hash != config->payload.geofence_hash) {
    return STATUS_INVALID_ARGUMENT;
  }
  if (config->payload.fence_count > PRODUCT_MAX_GEOFENCE_POLYGONS) {
    return STATUS_OUT_OF_RANGE;
  }
  g_geo_configured = 0u;
  fms_geofence_init(&g_fms);
  {
    uint32_t i;
    for (i = 0; i < config->payload.fence_count; ++i) {
      ProductGeofenceSnapshot snapshot;
      Status st;
      fms_geofence_from_definition(&config->payload.fences[i], config, &snapshot);
      st = fms_geofence_configure(&g_fms, &snapshot);
      if (st != STATUS_OK) {
        fms_geofence_init(&g_fms);
        return st;
      }
    }
  }
  g_geo_configured = 1u;
  return STATUS_OK;
}

static void fms_geofence_from_definition(const ProductGeofenceDefinition *def,
                                        const ProductGeofenceConfig *config,
                                        ProductGeofenceSnapshot *out)
{
  memset(out, 0, sizeof(*out));
  out->fence_id = def->fence_id;
  out->fence_revision = config->config_version;
  out->fence_hash = config->geofence_hash;
  out->frame_id = FRAME_LOCAL_NED;
  out->validity = PRODUCT_GEOFENCE_VALID;
  out->kind = def->kind;
  out->vertex_count = def->vertex_count;
  memcpy(out->vertices_ne_m, def->vertices_ne_m, sizeof(out->vertices_ne_m));
  out->min_height_above_home_m = def->min_height_above_home_m;
  out->max_height_above_home_m = def->max_height_above_home_m;
  out->horizontal_margin_m = def->horizontal_margin_m;
  out->vertical_margin_m = def->vertical_margin_m;
}

/* ------------------------------------------------------------------ */
/* FmsPolicyPort                                                      */
/* ------------------------------------------------------------------ */

Status fms_model_validate(const ProductProfile *profile)
{
  Status st = fms_product_validate_profile(profile);
  if (st != STATUS_OK) {
    return st;
  }
  return STATUS_OK;
}

Status fms_model_init(const ProductFmsConfig *config)
{
  Status st;
  uint8_t configured;
  if (g_initialized != 0u) {
    return STATUS_NOT_PERMITTED;
  }
  configured = g_geo_configured;
  g_geo_configured = 0u;
  st = fms_product_validate_config(config);
  if (st != STATUS_OK) {
    return st;
  }
  st = fms_engine_init_configured(&g_fms, config, configured);
  if (st != STATUS_OK) {
    return st;
  }
  if (config->payload.geofence_required != 0u &&
      (!configured || g_fms.geofence.fence_count == 0u)) {
    fms_engine_stop(&g_fms);
    return STATUS_PROFILE_MISMATCH; /* fail closed: a required fence is missing */
  }
  g_initialized = 1u;
  return STATUS_OK;
}

Status fms_model_update(const FmsInputView *input, FmsOutputView *output)
{
  if (g_initialized == 0u) {
    return STATUS_NOT_READY;
  }
  return fms_engine_update(&g_fms, input, output);
}

Status fms_model_get_health(FmsHealth *output)
{
  if (output == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  memset(output, 0, sizeof(*output));
  if (g_initialized == 0u) {
    return STATUS_NOT_READY;
  }
  *output = g_fms.health;
  fms_fill_header(&g_fms, &output->header, MSG_MODULE_HEALTH, FRAME_LOCAL_NED,
                 g_fms.clock.now_us, fms_quad_x4_policy.status_ttl_us);
  return STATUS_OK;
}
Status fms_model_authorize_origin_reset(const ContractHeader *candidate,
    uint32_t next_origin,EstimatorResetRequest *out)
{
  uint64_t now=g_fms.clock.now_us;
  if(!candidate || !out) return STATUS_INVALID_ARGUMENT;
  memset(out,0,sizeof(*out));
  if(!g_initialized || !g_fms.clock.bound) return STATUS_NOT_READY;
  if(g_fms.flight.arm_state!=ARM_STATE_DISARMED || g_fms.permit.active ||
     !next_origin || g_fms.flight.origin_id==UINT32_MAX || next_origin!=g_fms.flight.origin_id+1)
    return STATUS_NOT_PERMITTED;
  if(!header_valid(candidate,now) || candidate->source_id!=SOURCE_GNSS ||
     candidate->frame_id!=FRAME_WGS84_GEODETIC || candidate->boot_id!=g_fms.clock.boot_id ||
     candidate->time_epoch!=g_fms.clock.time_epoch || now-candidate->timestamp_sample_us>20000)
    return STATUS_STALE_DATA;
  fms_fill_header(&g_fms,&out->header,MSG_ESTIMATOR_RESET_REQUEST,FRAME_LOCAL_NED,
      candidate->timestamp_sample_us,20000);
  if(out->header.valid_until_us>candidate->valid_until_us) out->header.valid_until_us=candidate->valid_until_us;
  out->reset_type=RESET_TYPE_ORIGIN_REBUILD;out->initial_source=RESET_SOURCE_GNSS_ASSISTED;
  out->requested_origin_id=next_origin;
  fms_record_decision(&g_fms,DECISION_ACCEPT,13u,0u);
  return STATUS_OK;
}

Status fms_model_stop(void)
{
  fms_engine_stop(&g_fms);
  g_initialized = 0u;
  g_geo_configured = 0u;
  return STATUS_OK;
}
