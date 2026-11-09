/*
 * M5 GuidancePort: reference construction, validation and lifecycle.
 *
 * Every update is bounded and allocation-free. A reference is published as
 * anchor + exactly 24 future points at 20 ms spacing (t0..t0+480 ms); the
 * published points are validated with the same fifth-order Hermite
 * interpolation a consumer uses, against the analytic quintic.
 */
#include <math.h>
#include <string.h>
#include "guidance_update/guidance_internal.h"

static GuidanceState g_m5;

static uint8_t guidance_finite(float v)
{
  return (uint8_t)((v == v) && (v != INFINITY) && (v != -INFINITY));
}

static void guidance_zero_output(GuidanceOutput *out) { memset(out, 0, sizeof(*out)); }

static uint8_t guidance_directive_mode(const VehicleDirective *dir, uint16_t *kind)
{
  switch (dir->operation_code) {
  case PRODUCT_OP_PILOT:
    switch (dir->guidance_mode_code) {
    case PRODUCT_GUIDANCE_MODE_RATE:
      *kind = GUIDANCE_REFERENCE_PILOT_RATE;
      return 1u;
    case PRODUCT_GUIDANCE_MODE_ATTITUDE:
      *kind = GUIDANCE_REFERENCE_PILOT_ATTITUDE;
      return 1u;
    case PRODUCT_GUIDANCE_MODE_ALT_HOLD:
      *kind = GUIDANCE_REFERENCE_ALTITUDE_PILOT;
      return 1u;
    case PRODUCT_GUIDANCE_MODE_POS_HOLD:
      *kind = GUIDANCE_REFERENCE_TRAJECTORY;
      return 1u;
    default:
      return 0u;
    }
  case PRODUCT_OP_HOLD:
  case PRODUCT_OP_MISSION:
  case PRODUCT_OP_RETURN:
    *kind = GUIDANCE_REFERENCE_TRAJECTORY;
    return 1u;
  case PRODUCT_OP_LAND:
  case PRODUCT_OP_SAFE_ACTION:
    *kind = dir->guidance_mode_code == PRODUCT_GUIDANCE_MODE_ALT_HOLD ?
        GUIDANCE_REFERENCE_ALTITUDE_AUTOMATIC : GUIDANCE_REFERENCE_TRAJECTORY;
    return 1u;
  default:
    return 0u;
  }
}

static void guidance_epochs_from_input(const GuidanceInput *in, uint16_t kind,
                                 GuidanceRefContext *ctx)
{
  memset(ctx, 0, sizeof(*ctx));
  ctx->valid = 1u;
  ctx->kind = kind;
  ctx->authority_epoch = in->authority_status->authority_epoch;
  ctx->directive_epoch = in->directive->directive_epoch;
  ctx->state_epoch = in->navigation->state_epoch;
  ctx->origin_id = in->navigation->origin_id;
  ctx->boot_id = in->navigation->header.boot_id;
  ctx->time_epoch = in->navigation->header.time_epoch;
  if (kind == GUIDANCE_REFERENCE_TRAJECTORY && in->directive->operation_code ==
                                                    PRODUCT_OP_MISSION &&
      in->authorized_mission != NULL) {
    ctx->mission_epoch = in->authorized_mission->mission_epoch;
    ctx->current_item_id = in->authorized_mission->current_item_id;
    ctx->execution_token = in->authorized_mission->execution_token;
  }
}

static uint8_t guidance_context_same(const GuidanceRefContext *a, const GuidanceRefContext *b)
{
  if (a->boot_id != b->boot_id || a->time_epoch != b->time_epoch || a->kind != b->kind || a->authority_epoch != b->authority_epoch ||
      a->directive_epoch != b->directive_epoch || a->state_epoch != b->state_epoch ||
      a->origin_id != b->origin_id || a->mission_epoch != b->mission_epoch ||
      a->current_item_id != b->current_item_id ||
      a->execution_token != b->execution_token) {
    return 0u;
  }
  return 1u;
}

static void guidance_new_reference(GuidanceState *m5)
{
  m5->id_next = (m5->id_next == 0u) ? 1u : m5->id_next;
  m5->epoch_next = (m5->epoch_next == 0u) ? 1u : m5->epoch_next;
  m5->ref_id = m5->id_next++;
  m5->ref_epoch = m5->epoch_next++;
  m5->active = 0u;
  m5->plan_valid = 0u;
  m5->landing_stage = 0u;
  m5->pilot_valid = 0u;
}

/* ------------------------------------------------------------------ */
/* update                                                             */
/* ------------------------------------------------------------------ */

static void guidance_fill_status(GuidanceState *m5, const GuidanceInput *in, GuidanceOutput *out,
                           uint16_t state, uint16_t kind, uint16_t reason,
                           uint16_t item_result)
{
  GuidanceStatus *st = &out->guidance_status;
  guidance_publish_header(in, m5, &st->header, MSG_GUIDANCE_STATUS, kind);
  st->reference_id = m5->ref_id;
  st->reference_epoch = m5->ref_epoch;
  st->state_epoch = in->navigation->state_epoch;
  st->origin_id = in->navigation->origin_id;
  st->authority_epoch = in->authority_status->authority_epoch;
  st->directive_epoch = in->directive->directive_epoch;
  if (kind == GUIDANCE_REFERENCE_TRAJECTORY && in->authorized_mission != NULL &&
      in->directive->operation_code == PRODUCT_OP_MISSION) {
    st->mission_epoch = in->authorized_mission->mission_epoch;
    st->current_item_id = in->authorized_mission->current_item_id;
    st->execution_token = in->authorized_mission->execution_token;
  }
  st->guidance_state = state;
  st->reference_kind = kind;
  st->reason_code = reason;
  st->item_result = item_result;
  st->reference_expires_at_us = guidance_deadline(in, kind);
}

static void guidance_fill_feasibility(const GuidanceInput *in, GuidanceState *m5,
                                GuidanceOutput *out, uint8_t feasible,
                                uint16_t reason, uint16_t constraint)
{
  GuidanceFeasibility *f = &out->feasibility;
  guidance_publish_header(in, m5, &f->header, MSG_GUIDANCE_FEASIBILITY, m5->ctx.kind);
  f->reference_id = m5->ref_id;
  f->reference_epoch = m5->ref_epoch;
  f->feasible = feasible;
  f->severity = (uint8_t)(feasible ? 0u : 2u);
  f->reason_code = reason;
  f->violated_constraint_id = constraint;
  f->earliest_violation_offset_us = m5->violation_offset_us;
}

static uint8_t guidance_permit_matches(const GuidanceInput *in, uint32_t reference_epoch,
                                 uint32_t authority_epoch, uint32_t directive_epoch)
{
  const ActuationPermit *p = in->active_permit;
  return (uint8_t)(p != NULL && guidance_header_ok(&p->header, &in->navigation->header,
      MSG_ACTUATION_PERMIT, in->evaluation_time_us) && p->permit_epoch != 0u &&
      p->permit_flags == PERMIT_SCOPE_CONTROL_ACTIVE &&
      p->reference_epoch == reference_epoch && p->authority_epoch == authority_epoch &&
      p->directive_epoch == directive_epoch && p->state_epoch == in->navigation->state_epoch &&
      p->output_policy_id == in->directive->policy_id &&
      p->permit_expires_at_us > in->evaluation_time_us);
}

Status guidance_validate(const ProductProfile *profile)
{
  if (profile == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  if (profile->product_profile_id != PRODUCT_PROFILE_ID ||
      profile->guidance_impl_id != PRODUCT_GUIDANCE_IMPL_ID) {
    return STATUS_PROFILE_MISMATCH;
  }
  return STATUS_OK;
}

Status guidance_init(const ProductGuidanceConfig *config)
{
  const ProductGuidanceConfigPayload *p;
  if (config == NULL || g_m5.initialized != 0u) {
    return (config == NULL) ? STATUS_INVALID_ARGUMENT : STATUS_NOT_PERMITTED;
  }
  p = &config->payload;
  if (config->profile_hash != PRODUCT_PROFILE_ID || config->config_version == 0u ||
      config->parameter_hash == 0u || !guidance_finite(p->mass_kg) || p->mass_kg <= 0 ||
      !guidance_finite(p->gravity_mps2) || p->gravity_mps2 <= 0 ||
      !guidance_finite(p->vehicle_radius_m) || p->vehicle_radius_m <= 0 ||
      !guidance_finite(p->clearance_margin_m) || p->clearance_margin_m < 0 ||
      !guidance_finite(p->uncertainty_inflation_factor) || p->uncertainty_inflation_factor < 0 ||
      p->planning_deadline_us == 0 || p->primitive_table_version != 1u) return STATUS_INVALID_ARGUMENT;
  if (p->obstacle_source_mode != 0u) return STATUS_UNSUPPORTED;
  {
    unsigned i;
    for (i = 0; i < 3; ++i)
      if (!guidance_finite(p->max_velocity_ned_mps[i]) || p->max_velocity_ned_mps[i] <= 0 ||
          !guidance_finite(p->max_acceleration_ned_mps2[i]) || p->max_acceleration_ned_mps2[i] <= 0 ||
          !guidance_finite(p->max_jerk_ned_mps3[i]) || p->max_jerk_ned_mps3[i] <= 0)
        return STATUS_INVALID_ARGUMENT;
  }
  memset(&g_m5, 0, sizeof(g_m5));
  g_m5.config = *config;
  g_m5.id_next = 1u;
  g_m5.epoch_next = 1u;
  g_m5.health.component_id = PRODUCT_GUIDANCE_IMPL_ID;
  g_m5.health.health_state = 2u;
  g_m5.initialized = 1u;
  return STATUS_OK;
}

Status guidance_stop(void)
{
  memset(&g_m5, 0, sizeof(g_m5));
  return STATUS_OK;
}

Status guidance_update(const GuidanceInput *in, GuidanceOutput *out)
{
  GuidanceRefContext ctx;
  float handoff_acceleration[3];
  const float *seed_acceleration = NULL;
  uint16_t kind = 0u;
  Status st;

  if (in == NULL || out == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  guidance_zero_output(out);
  if (g_m5.initialized == 0u) {
    return STATUS_NOT_READY;
  }
  if (in->directive != NULL) (void)guidance_directive_mode(in->directive, &kind);
  st = guidance_prepare_input(in, &g_m5, kind);
  if (st != STATUS_OK) {
    g_m5.ctx.valid = g_m5.plan_valid = g_m5.pilot_valid = g_m5.active = 0u;
    g_m5.health.last_error_code = (uint32_t)st;
    g_m5.health.consecutive_error_count++;
    return st;
  }

  if (!guidance_directive_mode(in->directive, &kind)) {
    guidance_fill_status(&g_m5, in, out, GUIDANCE_STATE_INACTIVE, GUIDANCE_REFERENCE_NONE,
                   GUIDANCE_PLAN_REASON_NONE, 0u);
    guidance_fill_feasibility(in, &g_m5, out, 0u, GUIDANCE_PLAN_REASON_INPUT_STALE, GUIDANCE_PLAN_CONSTRAINT_TIME);
    g_m5.active = g_m5.ctx.valid = g_m5.plan_valid = g_m5.pilot_valid = 0u;
    return STATUS_OK;
  }

  guidance_epochs_from_input(in, kind, &ctx);
  if (g_m5.ctx.valid == 0u || !guidance_context_same(&g_m5.ctx, &ctx)) {
    if (g_m5.id_next == 0u || g_m5.epoch_next == 0u) return STATUS_OUT_OF_RANGE;
    if (guidance_handoff_acceleration(&g_m5, in, &ctx, handoff_acceleration))
      seed_acceleration = handoff_acceleration;
    guidance_new_reference(&g_m5);
    g_m5.ctx = ctx;
    memcpy(g_m5.ctx.target_ned, in->navigation->core.position_ned_m, sizeof(ctx.target_ned));
    g_m5.ctx.target_yaw_rad = guidance_yaw_from_quat(in->navigation->core.q_nb_wxyz);
  }
  g_m5.violation_offset_us = 0;
  if (kind == GUIDANCE_REFERENCE_TRAJECTORY) {
    uint16_t reason = GUIDANCE_PLAN_REASON_NONE, constraint = GUIDANCE_PLAN_CONSTRAINT_NONE;
    st = guidance_build_trajectory(&g_m5, in, out, seed_acceleration, &reason, &constraint);
    if (st != STATUS_OK) {
      g_m5.reject_count++;
      g_m5.plan_valid = g_m5.ctx.valid = 0u;
      g_m5.health.health_state = 3u;
      g_m5.health.last_error_code = reason;
      g_m5.health.consecutive_error_count++;
      memset(&out->reference, 0, sizeof(out->reference));
      guidance_fill_status(&g_m5, in, out, GUIDANCE_STATE_REJECTED, kind, reason,
                     GUIDANCE_ITEM_RESULT_REJECTED);
      guidance_fill_feasibility(in, &g_m5, out, 0u, reason, constraint);
      g_m5.active = 0u;
      return STATUS_OK;
    }
    g_m5.active = guidance_permit_matches(in, g_m5.ref_epoch, in->authority_status->authority_epoch, in->directive->directive_epoch);
    guidance_fill_status(&g_m5, in, out, g_m5.active ? GUIDANCE_STATE_ACTIVE : GUIDANCE_STATE_READY,
                   kind, GUIDANCE_PLAN_REASON_NONE, GUIDANCE_ITEM_RESULT_TRACKING);
    guidance_fill_feasibility(in, &g_m5, out, 1u, GUIDANCE_PLAN_REASON_NONE, GUIDANCE_PLAN_CONSTRAINT_NONE);
  } else {
    st = kind == GUIDANCE_REFERENCE_ALTITUDE_AUTOMATIC ? guidance_build_vertical_land(&g_m5, in, out) :
        guidance_build_pilot(&g_m5, in, out);
    if (st != STATUS_OK) {
      g_m5.reject_count++;
      guidance_fill_status(&g_m5, in, out, GUIDANCE_STATE_REJECTED, kind,
                     (st == STATUS_NUMERIC_ERROR) ? GUIDANCE_PLAN_REASON_NUMERIC
                                                  : GUIDANCE_PLAN_REASON_ENVELOPE_INVALID,
                     GUIDANCE_ITEM_RESULT_REJECTED);
      guidance_fill_feasibility(in, &g_m5, out, 0u, GUIDANCE_PLAN_REASON_ENVELOPE_INVALID,
                          GUIDANCE_PLAN_CONSTRAINT_DYNAMICS);
      g_m5.active = 0u;
      return STATUS_OK;
    }
    g_m5.active = guidance_permit_matches(in, g_m5.ref_epoch, in->authority_status->authority_epoch, in->directive->directive_epoch);
    guidance_fill_status(&g_m5, in, out, g_m5.active ? GUIDANCE_STATE_ACTIVE : GUIDANCE_STATE_READY,
                   kind, GUIDANCE_PLAN_REASON_NONE, GUIDANCE_ITEM_RESULT_TRACKING);
    guidance_fill_feasibility(in, &g_m5, out, 1u, GUIDANCE_PLAN_REASON_NONE, GUIDANCE_PLAN_CONSTRAINT_NONE);
  }

  g_m5.last_state_time_us = in->control_time_us;
  g_m5.last_evaluation_time_us = in->evaluation_time_us;
  guidance_publish_header(in, &g_m5, &g_m5.health.header, MSG_MODULE_HEALTH, kind);
  g_m5.health.last_error_code = 0;
  g_m5.health.consecutive_error_count = 0;
  g_m5.health.last_success_time_us = in->evaluation_time_us;
  g_m5.health.last_heartbeat_time_us = in->evaluation_time_us;
  g_m5.health.health_state = 2u;
  return STATUS_OK;
}

Status guidance_get_health(GuidanceHealth *out)
{
  if (out == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  memset(out, 0, sizeof(*out));
  if (g_m5.initialized == 0u) {
    return STATUS_NOT_READY;
  }
  *out = g_m5.health;
  out->header.message_type = MSG_MODULE_HEALTH;
  out->header.source_id = SOURCE_M5;
  out->header.schema_major = CONTRACT_SCHEMA_MAJOR;
  out->header.schema_minor = CONTRACT_SCHEMA_MINOR;

  return STATUS_OK;
}
