/*
 * M4-02 ModeManager and M4-03 AuthorityManager.
 *
 * Authority is only granted by an explicit, authenticated edge with fresh RC;
 * stick centring, link recovery and mode-switch changes never grant or release
 * authority by themselves (00 §5.2/§5.3, 06 §7.3).
 */
#include <string.h>
#include "flight_state_machine/fms_internal.h"

void fms_authority_init(FmsEngine *e)
{
  memset(&e->authority, 0, sizeof(e->authority));
  e->authority.owner = AUTHORITY_NONE;
  e->authority.epoch = 1u;
  e->authority.last_mode_bits = 0u;
  e->flight.authority_epoch = 1u;
}

void fms_authority_set(FmsEngine *e, uint8_t owner, uint16_t source_id, uint16_t reason)
{
  if (owner > AUTHORITY_GCS_SUPERVISED) {
    return;
  }
  if (e->authority.owner == owner && e->authority.source_id == source_id) {
    return; /* link recovery / repeat does not bump authority_epoch */
  }
  e->authority.owner = owner;
  e->authority.source_id = source_id;
  e->authority.epoch++;
  if (e->authority.epoch == 0u) {
    e->authority.epoch = 1u;
  }
  e->flight.authority_epoch = e->authority.epoch;
  e->flight.directive_epoch++;
  if (e->flight.directive_epoch == 0u) {
    e->flight.directive_epoch = 1u;
  }
  fms_record_decision(e, DECISION_ACCEPT, reason, 0u);
}

uint8_t fms_authority_pilot(const FmsEngine *e)
{
  return (uint8_t)(e->authority.owner == AUTHORITY_PILOT);
}

/* ------------------------------------------------------------------ */
/* mode transitions                                                   */
/* ------------------------------------------------------------------ */

static uint8_t fms_mode_legal(const FmsEngine *e, uint8_t mode, uint8_t internal_request)
{
  if (mode > FLIGHT_MODE_EMERGENCY_LAND) {
    return 0u;
  }
  if (e->flight.arm_state == ARM_STATE_DISARMED) {
    return (uint8_t)(mode == FLIGHT_MODE_NONE);
  }
  if (mode == FLIGHT_MODE_NONE) {
    return 0u; /* ARMED requires a non-NONE mode (00 §5.2) */
  }
  if (mode == FLIGHT_MODE_AUTO_MISSION &&
      e->mission.state != MISSION_STATE_RUNNING &&
      !(internal_request != 0u && (e->mission.state == MISSION_STATE_LOADED ||
                                   e->mission.state == MISSION_STATE_PAUSED))) {
    return 0u;
  }
  if (mode == FLIGHT_MODE_AUTO_RETURN &&
      e->home.snap.validity != PRODUCT_HOME_VALID) {
    return 0u;
  }
  return 1u;
}

uint8_t fms_mode_transition(FmsEngine *e, uint8_t mode, const FmsInputs *in,
                           uint8_t internal_request)
{
  if (!fms_mode_legal(e, mode, internal_request)) {
    return 0u;
  }
  if (internal_request == 0u && fms_mode_is_pilot(mode) &&
      e->authority.owner != AUTHORITY_PILOT) {
    return 0u; /* a GCS/mission request may not select a pilot mode */
  }
  if (mode != FLIGHT_MODE_NONE && in != NULL &&
      !fms_capable_for_mode(e, in, mode, 1u)) {
    return 0u;
  }
  if (mode != e->flight.flight_mode) {
    e->flight.flight_mode = mode;
    e->flight.mode_since_us = (in != NULL) ? in->now_us : e->clock.now_us;
    e->flight.directive_epoch++;
    if (e->flight.directive_epoch == 0u) {
      e->flight.directive_epoch = 1u;
    }
  }
  return 1u;
}

/* ------------------------------------------------------------------ */
/* authority arbitration                                              */
/* ------------------------------------------------------------------ */

void fms_authority_update(FmsEngine *e, const FmsInputs *in)
{
  uint32_t bits = 0u;
  uint8_t pilot_mode = 0u;
  uint8_t edge = 0u;

  if (in->op_ok != 0u) {
    bits = in->op->payload.mode_switch_bits;
    pilot_mode = (uint8_t)(bits & PRODUCT_MODE_SWITCH_MASK);
    edge = (uint8_t)(((bits & PRODUCT_MODE_TAKEOVER_EDGE) != 0u) &&
                     ((e->authority.last_mode_bits & PRODUCT_MODE_TAKEOVER_EDGE) == 0u));
  }

  if (e->flight.arm_state != ARM_STATE_ARMED) {
    e->authority.last_mode_bits = bits;
    e->authority.mapped_mode_pending = 0u;
    return;
  }

  if (edge != 0u && !e->flight.critical_energy_latched && !e->flight.terminal_latched &&
      in->rc_fresh != 0u && in->op_ok != 0u &&
      pilot_mode >= FLIGHT_MODE_MANUAL_RATE && pilot_mode <= FLIGHT_MODE_POS_HOLD) {
    if (fms_capable_for_mode(e, in, pilot_mode, 1u)) {
      e->authority.latched = 1u;
      e->authority.pilot_since_us = in->now_us;
      fms_authority_set(e, AUTHORITY_PILOT, 1u, 0u);
      fms_mission_on_pilot_takeover(e);
      fms_permit_revoke(e, 11u);
      (void)fms_mode_transition(e, pilot_mode, in, 1u);
    }
  } else if (e->authority.latched != 0u && edge == 0u && in->rc_fresh != 0u &&
             in->op_ok != 0u && pilot_mode >= FLIGHT_MODE_AUTO_MISSION) {
    /* Explicit release: switch deliberately returned to an automatic position. */
    e->authority.latched = 0u;
    fms_authority_set(e, (e->mission.state == MISSION_STATE_RUNNING) ? AUTHORITY_MISSION
                                                                    : AUTHORITY_NONE,
                     0u, 0u);
  }

  /* A mapped switch can change mode only after explicit pilot takeover.
   * Use a stable position for 150 ms and require an edge after link acquisition. */
  if (e->authority.owner != AUTHORITY_PILOT || in->rc_fresh == 0u ||
      e->flight.failsafe_level >= FAILSAFE_ACTION_REQUIRED ||
      e->flight.critical_energy_latched || e->flight.terminal_latched ||
      (bits & e->authority.last_mode_bits & PRODUCT_MODE_SWITCH_MAPPED) == 0u ||
      pilot_mode < FLIGHT_MODE_MANUAL_RATE || pilot_mode > FLIGHT_MODE_POS_HOLD) {
    e->authority.mapped_mode_pending = 0u;
  } else if (pilot_mode != (e->authority.last_mode_bits & PRODUCT_MODE_SWITCH_MASK)) {
    e->authority.mapped_mode_pending = pilot_mode;
    e->authority.mapped_mode_since_us = in->now_us;
  } else if (e->authority.mapped_mode_pending == pilot_mode &&
             in->now_us - e->authority.mapped_mode_since_us >= 150000u) {
    e->authority.mapped_mode_pending = 0u;
    (void)fms_mode_transition(e, pilot_mode, in, 1u);
  }

  if (in->rc_fresh == 0u && e->authority.owner == AUTHORITY_PILOT) {
    /* Pilot input is not authoritative without a fresh authenticated link. */
    fms_authority_set(e, AUTHORITY_NONE, 0u, 0u);
  }

  e->authority.last_mode_bits = in->rc_fresh ? bits : bits & ~PRODUCT_MODE_SWITCH_MAPPED;
}

/* ------------------------------------------------------------------ */
/* failsafe level and forced safe action                              */
/* ------------------------------------------------------------------ */

static uint8_t fms_nav_attitude_invalid(const FmsInputs *in)
{
  if (in->nav_ok == 0u || in->cap_ok == 0u) {
    return 1u;
  }
  if (((in->cap->capability_mask & in->nav->core.valid_mask) & NAV_VALID_ATTITUDE) == 0u) {
    return 1u;
  }
  if (in->cap->solution_state == 3u) {
    return 1u;
  }
  return 0u;
}

void fms_failsafe_update(FmsEngine *e, const FmsInputs *in)
{
  uint8_t level = FAILSAFE_NORMAL;
  uint8_t rc_lost;

  if (e->flight.terminal_latched != 0u) {
    e->flight.failsafe_level = FAILSAFE_TERMINAL;
    fms_permit_revoke(e, 18u);
    return;
  }

  rc_lost = (uint8_t)(in->rc_seen == 0u || in->rc_fresh == 0u);
  const uint8_t guidance_rejected = (uint8_t)(in->guidance_ok && in->feas_ok && in->nav_ok &&
      in->guidance->guidance_state == GUIDANCE_STATE_REJECTED && in->feas->feasible == 0u &&
      in->guidance->authority_epoch == e->authority.epoch &&
      in->guidance->directive_epoch == e->flight.directive_epoch &&
      in->guidance->state_epoch == in->nav->state_epoch && in->guidance->origin_id == in->nav->origin_id &&
      in->guidance->reference_epoch == in->feas->reference_epoch &&
      in->guidance->reference_id == in->feas->reference_id);
  const uint8_t return_rejected = (uint8_t)(guidance_rejected &&
      e->flight.flight_mode == FLIGHT_MODE_AUTO_RETURN);
  const uint8_t mission_rejected = (uint8_t)(guidance_rejected &&
      e->mission.state == MISSION_STATE_RUNNING);

  if (e->flight.arm_state == ARM_STATE_ARMED) {
    if (in->control_ok != 0u && e->permit.active != 0u &&
        e->permit.scope == PERMIT_SCOPE_CONTROL_ACTIVE &&
        in->control->header.timestamp_publish_us > e->permit.binding_since_us) {
      e->permit.reference_rebind_until_us = 0u;
    }
    if (in->faults_ok != 0u && in->faults->highest_severity >= 3u) {
      level = FAILSAFE_TERMINAL;
    } else if (e->land.state != LANDED_STATE_LANDED && fms_nav_attitude_invalid(in)) {
      level = FAILSAFE_TERMINAL; /* NAV-06: no attitude estimate, no promise of flight */
    } else if (in->act_ok == 0u || in->act_status->fault_mask != 0u ||
               (in->act_cap->available_mask & 0x0Fu) != 0x0Fu ||
               (in->control_ok != 0u && (in->control->health_state >= 4u ||
                                         (in->control->controllable_dof_mask & 0x0Fu) != 0x0Fu)) ||
               (in->control_ok == 0u && in->now_us >= e->permit.reference_rebind_until_us &&
                                        (e->flight.control_link_established != 0u ||
                                         e->land.state != LANDED_STATE_LANDED))) {
      level = FAILSAFE_TERMINAL;
    } else if (rc_lost != 0u || e->flight.critical_energy_latched) {
      level = FAILSAFE_ACTION_REQUIRED;
    } else if (!fms_capable_for_mode(e, in, e->flight.flight_mode, 0u) ||
               return_rejected || mission_rejected) {
      level = FAILSAFE_ACTION_REQUIRED;
    } else if (in->rc_state == LINK_STATE_DEGRADED || in->energy_state == ENERGY_LOW ||
               in->cap_ok == 0u || in->cap->solution_state == 2u) {
      level = FAILSAFE_DEGRADED;
    }
  } else if (e->flight.arm_state == ARM_STATE_DISARMED) {
    level = FAILSAFE_NORMAL;
  }

  if (level == FAILSAFE_TERMINAL) {
    e->flight.terminal_latched = 1u;
    e->flight.failsafe_level = FAILSAFE_TERMINAL;
    fms_permit_revoke(e, 18u);
    fms_record_decision(e, DECISION_FORCE_SAFE_ACTION, 18u, 0u);
    return;
  }

  if (level == FAILSAFE_ACTION_REQUIRED &&
      e->flight.failsafe_level < FAILSAFE_ACTION_REQUIRED) {
    fms_record_decision(e, DECISION_FORCE_SAFE_ACTION, 0u, 0u);
  }

  e->flight.failsafe_level = level;

  if (level == FAILSAFE_ACTION_REQUIRED && e->flight.arm_state == ARM_STATE_ARMED) {
    if (rc_lost != 0u && e->authority.owner == AUTHORITY_PILOT) {
      fms_authority_set(e, AUTHORITY_NONE, 0u, 0u);
    }
    if ((e->flight.flight_mode == FLIGHT_MODE_AUTO_LAND ||
         e->flight.flight_mode == FLIGHT_MODE_EMERGENCY_LAND) &&
        fms_capable_for_mode(e, in, e->flight.flight_mode, 0u)) {
      return; /* A feasible descent already satisfies the forced action. */
    }
    if (rc_lost || e->flight.critical_energy_latched ||
        !fms_capable_for_mode(e, in, e->flight.flight_mode, 0u) ||
        return_rejected || mission_rejected) {
      if (!e->flight.critical_energy_latched && !return_rejected && rc_lost &&
          fms_capable_for_mode(e, in, FLIGHT_MODE_AUTO_RETURN, 1u)) {
        (void)fms_mode_transition(e, FLIGHT_MODE_AUTO_RETURN, in, 1u);
      } else if (fms_capable_for_mode(e, in, FLIGHT_MODE_AUTO_LAND, 1u)) {
        (void)fms_mode_transition(e, FLIGHT_MODE_AUTO_LAND, in, 1u);
      } else if (in->rc_fresh && e->authority.owner == AUTHORITY_PILOT &&
                 fms_capable_for_mode(e, in, FLIGHT_MODE_MANUAL_ATTITUDE, 1u)) {
        (void)fms_mode_transition(e, FLIGHT_MODE_MANUAL_ATTITUDE, in, 1u);
      } else {
        e->flight.terminal_latched = 1u;
        e->flight.failsafe_level = FAILSAFE_TERMINAL;
        fms_permit_revoke(e, 18u);
      }
      fms_mission_on_failsafe(e);
    }
  }
}
