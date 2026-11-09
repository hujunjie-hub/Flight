/*
 * M4-08 FailsafePermitManager (permit half): ActuationPermit issuance,
 * renewal, revocation watermark and the READY -> Permit -> ACTIVE handshake.
 *
 * Handshake rule (02 §9, 00 §4.6): a READY candidate reference plus a matching
 * Feasibility is enough for M4 to sign the CONTROL_ACTIVE permit; M4 never
 * waits for the later ACTIVE confirmation. Renewal with unchanged semantics
 * keeps permit_epoch; new/revoked/scope/policy/reference binding changes bump
 * it. Revocation is published immediately and forms an irreversible watermark.
 */
#include <string.h>
#include "flight_state_machine/fms_internal.h"

void fms_permit_init(FmsEngine *e)
{
  memset(&e->permit, 0, sizeof(e->permit));
  e->permit.epoch = 0u;
  e->permit.output_policy_id = (uint16_t)e->config.payload.output_policy_id;
}

static uint32_t fms_permit_next_epoch(FmsEngine *e)
{
  uint32_t epoch = e->permit.epoch + 1u;
  if (epoch == 0u) {
    epoch = 1u;
  }
  e->permit.epoch = epoch;
  return epoch;
}

void fms_permit_revoke(FmsEngine *e, uint16_t reason)
{
  if (e->permit.active == 0u) {
    return; /* nothing to revoke; a revoked permit is already published/pending */
  }
  if (e->config.payload.output_policy_id == 2u &&
      e->permit.scope == PERMIT_SCOPE_CONTROL_ACTIVE &&
      (reason == 12u || reason == 20u || ((reason == 10u || reason == 11u || reason == 21u) &&
                        (e->permit.directive_epoch != e->flight.directive_epoch ||
                         e->permit.authority_epoch != e->authority.epoch)))) {
    if (e->permit.reference_rebind_until_us == 0u) {
      e->permit.reference_rebind_until_us = fms_add_us(e->clock.now_us, 2u * FMS_PERMIT_LEASE_US);
    }
  } else {
    e->permit.reference_rebind_until_us = 0u;
  }
  fms_permit_next_epoch(e);
  if (e->permit.epoch > e->permit.revoked_watermark) {
    e->permit.revoked_watermark = e->permit.epoch;
  }
  e->permit.active = 0u;
  e->permit.revoked_pending = 1u;
  e->permit.reference_ready = 0u;
  e->permit.reason_code = reason;
  e->permit_revocations++;
}

static uint8_t fms_reference_kind_matches(uint8_t mode, uint16_t kind)
{
  switch (mode) {
  case FLIGHT_MODE_MANUAL_RATE:
    return (uint8_t)(kind == GUIDANCE_REFERENCE_PILOT_RATE);
  case FLIGHT_MODE_MANUAL_ATTITUDE:
    return (uint8_t)(kind == GUIDANCE_REFERENCE_PILOT_ATTITUDE);
  case FLIGHT_MODE_ALT_HOLD:
    return (uint8_t)(kind == GUIDANCE_REFERENCE_ALTITUDE_PILOT);
  case FLIGHT_MODE_POS_HOLD:
  case FLIGHT_MODE_AUTO_MISSION:
  case FLIGHT_MODE_AUTO_RETURN:
    return (uint8_t)(kind == GUIDANCE_REFERENCE_TRAJECTORY);
  case FLIGHT_MODE_AUTO_LAND:
  case FLIGHT_MODE_EMERGENCY_LAND:
    return (uint8_t)(kind == GUIDANCE_REFERENCE_TRAJECTORY || kind == GUIDANCE_REFERENCE_ALTITUDE_AUTOMATIC);
  default:
    return 0u;
  }
}

/* Validate the M5 candidate bundle. Returns 1 and fills reference_epoch when
 * this cycle may be signed; 0 otherwise. */
static uint8_t fms_guidance_context(const FmsEngine *e, const FmsInputs *in,
                                   uint32_t *reference_epoch, uint32_t *reference_id,
                                   uint64_t *reference_expires_us)
{
  const GuidanceStatus *g;
  const GuidanceFeasibility *f;

  if (in->guidance_ok == 0u || in->feas_ok == 0u || in->nav_ok == 0u) {
    return 0u;
  }
  g = in->guidance;
  f = in->feas;

  if (g->guidance_state != GUIDANCE_STATE_READY &&
      g->guidance_state != GUIDANCE_STATE_ACTIVE) {
    return 0u;
  }
  if (f->feasible != 1u || f->reason_code != 0u) {
    return 0u;
  }
  if (f->reference_id != g->reference_id || f->reference_epoch != g->reference_epoch) {
    return 0u;
  }
  if (g->reference_expires_at_us == 0u || g->reference_expires_at_us <= in->now_us) {
    return 0u;
  }
  if (g->state_epoch != in->nav->state_epoch || g->origin_id != in->nav->origin_id) {
    return 0u;
  }
  if (g->authority_epoch != e->authority.epoch ||
      g->directive_epoch != e->flight.directive_epoch) {
    return 0u;
  }
  if (!fms_reference_kind_matches(e->flight.flight_mode, g->reference_kind)) {
    return 0u;
  }

  if (g->reference_kind == GUIDANCE_REFERENCE_TRAJECTORY) {
    if (e->mission.state == MISSION_STATE_RUNNING) {
      if (g->execution_token != e->mission.token || g->execution_token == 0u) {
        return 0u;
      }
      if (g->mission_epoch != e->mission.mission_epoch) {
        return 0u;
      }
      if (g->current_item_id !=
          e->mission.active.waypoints[e->mission.current_index].waypoint_id) {
        return 0u;
      }
    } else if (g->execution_token != 0u) {
      return 0u; /* no active mission execution may carry a stale token */
    }
  } else {
    if (g->execution_token != 0u) {
      return 0u;
    }
    if (g->reference_kind != GUIDANCE_REFERENCE_ALTITUDE_AUTOMATIC && e->authority.owner != AUTHORITY_PILOT) {
      return 0u; /* pilot references need pilot authority */
    }
  }

  *reference_epoch = g->reference_epoch;
  *reference_id = g->reference_id;
  *reference_expires_us = g->reference_expires_at_us;
  return 1u;
}

static void fms_permit_sign(FmsEngine *e, const FmsInputs *in, uint8_t scope,
                           uint32_t reference_epoch, uint32_t reference_id,
                           uint64_t reference_expires_us)
{
  uint64_t now = in->now_us;
  uint64_t lease_expires = fms_add_us(now, FMS_PERMIT_LEASE_US);
  uint64_t expires = lease_expires;
  uint8_t binding_changed;

  if (in->nav_ok == 0u) {
    return; /* no navigation state that a permit could be bound to */
  }
  if (e->permit.revoked_pending != 0u) {
    return; /* a revocation must be observable for one cycle before any new permit */
  }

  if (reference_expires_us != 0u && reference_expires_us < expires) {
    expires = reference_expires_us;
  }
  if (expires <= now) {
    fms_permit_revoke(e, 10u);
    return;
  }

  binding_changed = (uint8_t)(e->permit.active == 0u ||
                              e->permit.scope != scope ||
                              e->permit.reference_epoch != reference_epoch ||
                              e->permit.reference_id != reference_id ||
                              e->permit.directive_epoch != e->flight.directive_epoch ||
                              e->permit.authority_epoch != e->authority.epoch ||
                              e->permit.state_epoch != in->nav->state_epoch ||
                              e->permit.output_policy_id !=
                                  (uint16_t)e->config.payload.output_policy_id);
  if (binding_changed != 0u) {
    fms_permit_next_epoch(e);
    e->permit.binding_since_us = now;
    e->permit.active = 1u;
    e->permit.revoked_pending = 0u;
    e->permit.scope = scope;
    e->permit.reference_epoch = reference_epoch;
    e->permit.reference_id = reference_id;
    e->permit.directive_epoch = e->flight.directive_epoch;
    e->permit.authority_epoch = e->authority.epoch;
    e->permit.state_epoch = in->nav->state_epoch;
    e->permit.output_policy_id = (uint16_t)e->config.payload.output_policy_id;
    e->permit.reason_code = 0u;
  }
  if (scope == PERMIT_SCOPE_CONTROL_ACTIVE) {
    e->flight.control_link_established = 1u;
  }
  e->permit.expires_at_us = expires;
}

static void fms_permit_ground_disarm(FmsEngine *e)
{
  fms_permit_revoke(e, 11u);
  e->authority.owner = AUTHORITY_NONE;
  e->authority.latched = 0u;
  e->authority.source_id = 0u;
  e->flight.flight_mode = FLIGHT_MODE_NONE;
  e->flight.arm_state = ARM_STATE_DISARMED;
  e->flight.directive_epoch++;
  if (e->flight.directive_epoch == 0u) {
    e->flight.directive_epoch = 1u;
  }
  if (e->mission.state == MISSION_STATE_RUNNING || e->mission.state == MISSION_STATE_PAUSED) {
    e->mission.state = MISSION_STATE_ABORTED;
    e->mission.result_code = MISSION_RESULT_ABORTED;
    e->mission.token = 0u;
  }
  fms_record_decision(e, DECISION_FORCE_SAFE_ACTION, 6u, 0u);
}

void fms_permit_tick(FmsEngine *e, const FmsInputs *in)
{
  uint32_t reference_epoch = 0u;
  uint32_t reference_id = 0u;
  uint64_t reference_expires = 0u;
  uint64_t now = in->now_us;
  uint8_t control_ready;

  if (e->flight.arm_state == ARM_STATE_DISARMED) {
    fms_permit_revoke(e, 2u);
    return;
  }
  if (e->flight.failsafe_level == FAILSAFE_TERMINAL) {
    fms_permit_revoke(e, 18u);
    return;
  }

  if (e->flight.control_link_established == 0u) {
    uint64_t window_us = e->config.payload.first_reference_timeout_us;
    if (window_us == 0u || window_us > 200000u) {
      window_us = FMS_ARM_REFERENCE_WINDOW_US;
    }
    if (now - e->flight.arming_start_us > window_us) {
      /* Absolute first-reference window elapsed: no motion, controlled ground stop. */
      if (e->land.state == LANDED_STATE_LANDED) {
        fms_permit_ground_disarm(e);
      } else {
        fms_permit_revoke(e, 21u);
        if (e->flight.failsafe_level < FAILSAFE_ACTION_REQUIRED) {
          e->flight.failsafe_level = FAILSAFE_ACTION_REQUIRED;
          fms_record_decision(e, DECISION_FORCE_SAFE_ACTION, 21u, 0u);
        }
      }
      return;
    }
  }

  control_ready = (uint8_t)(e->flight.flight_mode != FLIGHT_MODE_NONE &&
                            fms_guidance_context(e, in, &reference_epoch, &reference_id,
                                                &reference_expires));
  if (control_ready != 0u) {
    fms_permit_sign(e, in, PERMIT_SCOPE_CONTROL_ACTIVE, reference_epoch, reference_id,
                   reference_expires);
    return;
  }

  if (e->flight.control_link_established == 0u) {
    if (e->land.state == LANDED_STATE_LANDED &&
        e->land.stable_ms >= e->config.payload.landed_dwell_us / 1000u) {
      fms_permit_sign(e, in, PERMIT_SCOPE_IDLE_ONLY, 0u, 0u, 0u);
      return;
    }
    fms_permit_revoke(e, 2u);
    return;
  }

  /* Control link was established and is now unusable: revoke, no silent reuse. */
  fms_permit_revoke(e, 10u);
  if (e->land.state == LANDED_STATE_LANDED &&
      e->land.stable_ms >= e->config.payload.landed_dwell_us / 1000u &&
      e->flight.flight_mode == FLIGHT_MODE_POS_HOLD) {
    fms_permit_sign(e, in, PERMIT_SCOPE_IDLE_ONLY, 0u, 0u, 0u);
  }
}

void fms_permit_fill(FmsEngine *e, ActuationPermit *out)
{
  memset(out, 0, sizeof(*out));
  if (e->permit.revoked_pending != 0u) {
    fms_fill_header(e, &out->header, MSG_ACTUATION_PERMIT, FRAME_LOCAL_NED,
                   e->clock.now_us, FMS_PERMIT_LEASE_US);
    out->permit_epoch = e->permit.epoch;
    out->directive_epoch = e->permit.directive_epoch;
    out->authority_epoch = e->permit.authority_epoch;
    out->state_epoch = e->permit.state_epoch;
    out->reference_epoch = e->permit.reference_epoch;
    out->permit_flags = PERMIT_FLAG_REVOKED;
    out->permit_expires_at_us = e->clock.now_us;
    out->reason_code = e->permit.reason_code;
    out->output_policy_id = e->permit.output_policy_id;
    e->permit.revoked_pending = 0u; /* published once, then invalid */
    return;
  }
  if (e->permit.active == 0u) {
    return;
  }
  fms_fill_header(e, &out->header, MSG_ACTUATION_PERMIT, FRAME_LOCAL_NED,
                 e->clock.now_us, FMS_PERMIT_LEASE_US);
  out->permit_epoch = e->permit.epoch;
  out->directive_epoch = e->permit.directive_epoch;
  out->authority_epoch = e->permit.authority_epoch;
  out->state_epoch = e->permit.state_epoch;
  out->reference_epoch = e->permit.reference_epoch;
  out->permit_flags = (uint32_t)e->permit.scope;
  out->permit_expires_at_us = e->permit.expires_at_us;
  out->reason_code = 0u;
  out->output_policy_id = e->permit.output_policy_id;
}
