/*
 * M4-01 MissionManager: unique owner of mission storage, current item,
 * execution token, MissionState and MissionStatus (02 §8/§21, 06 §7.1).
 *
 * CR-V191-C01: MISSION_START/PAUSE/RESUME/ABORT/CLEAR carry the current
 * mission_epoch in ProductOperationPayload (valid bit 8). The epoch is
 * mandatory and non-zero for those five commands; a missing bit, zero epoch or
 * a value that differs from the current M4 mission_epoch is rejected with
 * reason EPOCH (12). Non-mission commands must leave the field zero and the
 * valid bit clear. mission_id plus the authenticated source/session still bind
 * the command, but epoch replay is now rejected rather than silently accepted.
 */
#include <string.h>
#include <math.h>
#include "flight_state_machine/fms_internal.h"

static void fms_mission_clear_data(FmsEngine *e)
{
  memset(&e->mission.active, 0, sizeof(e->mission.active));
  e->mission.mission_id = 0u;
  e->mission.revision = 0u;
  e->mission.hash = 0u;
  e->mission.wp_count = 0u;
  e->mission.current_index = 0u;
  e->mission.token = 0u;
  e->mission.arrival_active = 0u;
  e->mission.progress = 0.0f;
  e->mission.origin_id = 0u;
  e->mission.state_epoch_at_commit = 0u;
  e->mission.result_code = MISSION_RESULT_NONE;
}

void fms_mission_init(FmsEngine *e)
{
  memset(&e->mission, 0, sizeof(e->mission));
  e->mission.state = MISSION_STATE_IDLE;
  e->mission.token_next = 1u;
  e->mission.result_code = MISSION_RESULT_NONE;
  fms_mission_clear_data(e);
}

static uint8_t fms_mission_new_token(FmsEngine *e)
{
  uint64_t next = e->mission.token_next;
  if (next == 0u) {
    next = 1u;
  }
  if (next == UINT64_MAX) {
    return 0u; /* exhausted: refuse new execution (02 §4.3) */
  }
  e->mission.token = next;
  e->mission.token_next = next + 1u;
  return 1u;
}

static void fms_mission_epoch_next(FmsEngine *e)
{
  uint32_t epoch = e->mission.mission_epoch + 1u;
  if (epoch == 0u) {
    epoch = 1u;
  }
  e->mission.mission_epoch = epoch;
}

uint8_t fms_mission_running(const FmsEngine *e)
{
  return (uint8_t)(e->mission.state == MISSION_STATE_RUNNING);
}

static Status fms_mission_validate_payload(FmsEngine *e, const FmsInputs *in,
                                          const ProductMissionPayload *m,
                                          uint32_t hash)
{
  uint16_t i, j;
  float max_speed = e->policy->default_max_speed_mps;
  float min_h = e->policy->default_min_height_above_home_m;
  float max_h = e->policy->default_max_height_above_home_m;

  if (hash == 0u || m->mission_id == 0u) {
    return STATUS_INVALID_ARGUMENT;
  }
  if (m->waypoint_count == 0u || m->waypoint_count > FMS_MAX_WAYPOINTS) {
    return STATUS_OUT_OF_RANGE;
  }
  if ((m->flags & ~PRODUCT_MISSION_REQUIRES_OBSTACLE_AVOIDANCE) != 0u) {
    return STATUS_INVALID_ARGUMENT;
  }
  if (m->flags & PRODUCT_MISSION_REQUIRES_OBSTACLE_AVOIDANCE) return STATUS_UNSUPPORTED;
  for (i = 0u; i < m->waypoint_count; ++i) {
    const ProductMissionWaypoint *wp = &m->waypoints[i];
    if (wp->waypoint_id == 0u || wp->flags != 0u) {
      return STATUS_INVALID_ARGUMENT;
    }
    for (j = 0u; j < i; ++j) {
      if (m->waypoints[j].waypoint_id == wp->waypoint_id) {
        return STATUS_INVALID_ARGUMENT;
      }
    }
    if (!fms_finite3(wp->position_ned_m) || !fms_finite(wp->acceptance_radius_m) ||
        !fms_finite(wp->speed_limit_mps) || !fms_finite(wp->yaw_rad)) {
      return STATUS_NUMERIC_ERROR;
    }
    if (wp->acceptance_radius_m <= 0.0f || wp->acceptance_radius_m > 100.0f ||
        wp->speed_limit_mps <= 0.0f || wp->speed_limit_mps > max_speed) {
      return STATUS_OUT_OF_RANGE;
    }
    if (in->nav_ok != 0u) {
      float home_d = e->home.valid ? e->home.snap.position_ned_m[2] : in->nav->core.position_ned_m[2];
      float h = home_d - wp->position_ned_m[2];
      if (h < min_h || h > max_h) {
        return STATUS_OUT_OF_RANGE;
      }
    }
    if (e->config.payload.geofence_required != 0u) {
      float p[3];
      p[0] = wp->position_ned_m[0];
      p[1] = wp->position_ned_m[1];
      p[2] = wp->position_ned_m[2];
      if (e->geofence.fence_count == 0u || !fms_geofence_contains(&e->geofence, p, e->home.valid ? e->home.snap.position_ned_m[2] : 0.0f)) {
        return STATUS_PROFILE_MISMATCH;
      }
    }
  }
  return STATUS_OK;
}

void fms_mission_commit(FmsEngine *e, const FmsInputs *in, const OperationRequest *req,
                       FmsCmdResult *r)
{
  const MissionUploadCandidate *cand = in->candidate;
  Status st;

  r->ack_state = COMMAND_ACK_REJECTED;
  r->reason_code = 1u;

  if (e->flight.arm_state != ARM_STATE_DISARMED) {
    r->reason_code = 11u;
    return;
  }
  if (e->mission.state == MISSION_STATE_RUNNING || e->mission.state == MISSION_STATE_PAUSED) {
    r->reason_code = 11u;
    return;
  }
  if (cand == NULL || in->candidate_ok == 0u) {
    r->reason_code = 2u;
    return;
  }
  if ((req->payload.valid_mask & (PRODUCT_OPERATION_VALID_MISSION_ID | PRODUCT_OPERATION_VALID_MISSION_TRANSACTION)) !=
      (PRODUCT_OPERATION_VALID_MISSION_ID | PRODUCT_OPERATION_VALID_MISSION_TRANSACTION)) {
    r->reason_code = 1u;
    return;
  }
  if (req->payload.mission_id != cand->mission.mission_id ||
      req->payload.mission_transaction_id != cand->transaction_id) {
    r->reason_code = 12u; /* transaction/id mismatch */
    return;
  }
  if (req->source_session_id != cand->source_session_id ||
      req->header.source_id != cand->header.source_id) {
    r->reason_code = 15u; /* source/session mismatch */
    return;
  }
  if (in->now_us - cand->header.timestamp_publish_us > e->policy->staging_timeout_us) {
    r->reason_code = 10u; /* stale staging */
    return;
  }
  if (in->nav_ok == 0u) {
    r->reason_code = 2u;
    return;
  }
  if (cand->state_epoch != in->nav->state_epoch || cand->origin_id != in->nav->origin_id ||
      cand->origin_id == 0u) {
    r->reason_code = 13u; /* FRAME_ORIGIN */
    return;
  }

  st = fms_mission_validate_payload(e, in, &cand->mission, cand->mission_hash);
  if (st != STATUS_OK) {
    r->reason_code = (st == STATUS_NUMERIC_ERROR) ? 5u : 3u;
    return;
  }

  memset(&e->mission.active, 0, sizeof(e->mission.active));
  e->mission.active = cand->mission;
  e->mission.mission_id = cand->mission.mission_id;
  e->mission.revision = cand->mission.mission_revision;
  e->mission.hash = cand->mission_hash;
  e->mission.wp_count = cand->mission.waypoint_count;
  e->mission.current_index = 0u;
  e->mission.origin_id = cand->origin_id;
  e->mission.state_epoch_at_commit = cand->state_epoch;
  e->mission.state = MISSION_STATE_LOADED;
  e->mission.result_code = MISSION_RESULT_NONE;
  e->mission.progress = 0.0f;
  e->mission.token = 0u;
  e->mission.expires_at_us = fms_add_us(in->now_us, e->policy->mission_ttl_us);
  e->mission.arrival_active = 0u;
  fms_mission_epoch_next(e);

  r->ack_state = COMMAND_ACK_COMPLETED;
  r->reason_code = 0u;
  fms_record_decision(e, DECISION_ACCEPT, 0u, req->request_id);
}

static uint8_t fms_mission_command_matches(const FmsEngine *e, const OperationRequest *req,
                                          uint16_t *reason)
{
  if ((req->payload.valid_mask & PRODUCT_OPERATION_VALID_MISSION_ID) == 0u) {
    *reason = 1u;
    return 0u;
  }
  if (e->mission.mission_id == 0u || req->payload.mission_id != e->mission.mission_id) {
    *reason = 12u;
    return 0u;
  }
  /* CR-V191-C01: mission commands must carry the current mission_epoch. */
  if ((req->payload.valid_mask & PRODUCT_OPERATION_VALID_MISSION_EPOCH) == 0u ||
      req->payload.mission_epoch == 0u) {
    *reason = 12u; /* EPOCH: field missing */
    return 0u;
  }
  if (req->payload.mission_epoch != e->mission.mission_epoch) {
    *reason = 12u; /* EPOCH: stale/replayed context */
    return 0u;
  }
  return 1u;
}

void fms_mission_start(FmsEngine *e, const FmsInputs *in, const OperationRequest *req,
                      FmsCmdResult *r)
{
  uint16_t reason = 0u;

  r->ack_state = COMMAND_ACK_REJECTED;
  r->reason_code = 1u;
  if (!fms_mission_command_matches(e, req, &reason)) {
    r->reason_code = reason;
    return;
  }
  if (e->mission.state != MISSION_STATE_LOADED) {
    r->reason_code = 11u;
    return;
  }
  if (e->flight.arm_state != ARM_STATE_ARMED) {
    r->reason_code = 2u;
    return;
  }
  if (e->authority.latched != 0u) {
    r->reason_code = 11u;
    return;
  }
  if (e->flight.failsafe_level == FAILSAFE_TERMINAL) {
    r->reason_code = 11u;
    return;
  }
  if (!fms_capable_for_mode(e, in, FLIGHT_MODE_AUTO_MISSION, 1u)) {
    r->reason_code = 2u;
    return;
  }
  if (e->home.snap.validity != PRODUCT_HOME_VALID ||
      e->home.snap.origin_id != in->nav->origin_id ||
      e->mission.origin_id != in->nav->origin_id ||
      e->mission.state_epoch_at_commit != in->nav->state_epoch) {
    r->reason_code = 13u;
    return;
  }
  if (in->now_us >= e->mission.expires_at_us) {
    r->reason_code = 10u;
    return;
  }
  if (!fms_mode_transition(e, FLIGHT_MODE_AUTO_MISSION, in, 1u)) {
    r->reason_code = 2u;
    return;
  }
  if (!fms_mission_new_token(e)) {
    r->reason_code = 9u;
    return;
  }
  e->mission.state = MISSION_STATE_RUNNING;
  e->mission.current_index = 0u;
  e->mission.result_code = MISSION_RESULT_NONE;
  e->mission.arrival_active = 0u;
  e->mission.expires_at_us = fms_add_us(in->now_us, e->policy->mission_ttl_us);
  fms_mission_epoch_next(e);
  fms_authority_set(e, AUTHORITY_MISSION, 0u, 0u);
  fms_record_decision(e, DECISION_ACCEPT, 0u, req->request_id);

  r->ack_state = COMMAND_ACK_COMPLETED;
  r->reason_code = 0u;
}

void fms_mission_pause(FmsEngine *e, const FmsInputs *in, const OperationRequest *req,
                      FmsCmdResult *r)
{
  uint16_t reason = 0u;

  r->ack_state = COMMAND_ACK_REJECTED;
  r->reason_code = 1u;
  if (!fms_mission_command_matches(e, req, &reason)) {
    r->reason_code = reason;
    return;
  }
  if (e->mission.state != MISSION_STATE_RUNNING) {
    r->reason_code = 11u;
    return;
  }
  e->mission.state = MISSION_STATE_PAUSED;
  e->mission.arrival_active = 0u;
  e->mission.token = 0u;
  fms_mission_epoch_next(e);
  fms_permit_revoke(e, 20u);
  (void)fms_mode_transition(e, FLIGHT_MODE_POS_HOLD, in, 1u);
  fms_record_decision(e, DECISION_ACCEPT, 0u, req->request_id);
  r->ack_state = COMMAND_ACK_COMPLETED;
  r->reason_code = 0u;
}

void fms_mission_resume(FmsEngine *e, const FmsInputs *in, const OperationRequest *req,
                       FmsCmdResult *r)
{
  uint16_t reason = 0u;

  r->ack_state = COMMAND_ACK_REJECTED;
  r->reason_code = 1u;
  if (!fms_mission_command_matches(e, req, &reason)) {
    r->reason_code = reason;
    return;
  }
  if (e->mission.state != MISSION_STATE_PAUSED) {
    r->reason_code = 11u;
    return;
  }
  if (e->flight.arm_state != ARM_STATE_ARMED || e->authority.latched != 0u ||
      e->flight.failsafe_level >= FAILSAFE_ACTION_REQUIRED) {
    r->reason_code = 11u;
    return;
  }
  if (!fms_capable_for_mode(e, in, FLIGHT_MODE_AUTO_MISSION, 1u) ||
      e->home.snap.validity != PRODUCT_HOME_VALID) {
    r->reason_code = 2u;
    return;
  }
  if (e->mission.origin_id != in->nav->origin_id ||
      e->mission.state_epoch_at_commit != in->nav->state_epoch) {
    r->reason_code = 13u; /* The stored waypoints belong to the old frame. */
    return;
  }
  if (in->now_us >= e->mission.expires_at_us) {
    r->reason_code = 10u;
    return;
  }
  if (!fms_mode_transition(e, FLIGHT_MODE_AUTO_MISSION, in, 1u)) {
    r->reason_code = 2u;
    return;
  }
  if (!fms_mission_new_token(e)) {
    r->reason_code = 9u;
    return;
  }
  e->mission.state = MISSION_STATE_RUNNING;
  e->mission.arrival_active = 0u;
  e->mission.expires_at_us = fms_add_us(in->now_us, e->policy->mission_ttl_us);
  fms_mission_epoch_next(e);
  fms_authority_set(e, AUTHORITY_MISSION, 0u, 0u);
  fms_record_decision(e, DECISION_ACCEPT, 0u, req->request_id);
  r->ack_state = COMMAND_ACK_COMPLETED;
  r->reason_code = 0u;
}

void fms_mission_abort(FmsEngine *e, const FmsInputs *in, const OperationRequest *req,
                      FmsCmdResult *r)
{
  uint16_t reason = 0u;

  r->ack_state = COMMAND_ACK_REJECTED;
  r->reason_code = 1u;
  if (!fms_mission_command_matches(e, req, &reason)) {
    r->reason_code = reason;
    return;
  }
  if (e->mission.state != MISSION_STATE_RUNNING && e->mission.state != MISSION_STATE_PAUSED) {
    r->reason_code = 11u;
    return;
  }
  e->mission.state = MISSION_STATE_ABORTED;
  e->mission.result_code = MISSION_RESULT_ABORTED;
  e->mission.token = 0u;
  e->mission.arrival_active = 0u;
  fms_mission_epoch_next(e);
  fms_permit_revoke(e, 20u);
  (void)fms_mode_transition(e, FLIGHT_MODE_POS_HOLD, in, 1u);
  fms_authority_set(e, AUTHORITY_NONE, 0u, 0u);
  fms_record_decision(e, DECISION_ACCEPT, 0u, req->request_id);
  r->ack_state = COMMAND_ACK_COMPLETED;
  r->reason_code = 0u;
}

void fms_mission_clear(FmsEngine *e, const FmsInputs *in, const OperationRequest *req,
                      FmsCmdResult *r)
{
  uint16_t reason = 0u;

  (void)in;
  r->ack_state = COMMAND_ACK_REJECTED;
  r->reason_code = 1u;
  if (e->mission.state == MISSION_STATE_IDLE && e->mission.mission_id == 0u &&
      req->payload.valid_mask == 0u && req->payload.mission_id == 0u &&
      req->payload.mission_epoch == 0u) {
    if (e->flight.arm_state != ARM_STATE_DISARMED) {
      r->reason_code = 11u;
      return;
    }
    fms_record_decision(e, DECISION_ACCEPT, 0u, req->request_id);
    r->ack_state = COMMAND_ACK_COMPLETED;
    r->reason_code = 0u;
    return;
  }
  if (!fms_mission_command_matches(e, req, &reason)) {
    r->reason_code = reason;
    return;
  }
  if (e->mission.state == MISSION_STATE_RUNNING || e->mission.state == MISSION_STATE_PAUSED) {
    r->reason_code = 11u;
    return;
  }
  fms_mission_clear_data(e);
  e->mission.state = MISSION_STATE_IDLE;
  fms_mission_epoch_next(e);
  fms_record_decision(e, DECISION_ACCEPT, 0u, req->request_id);
  r->ack_state = COMMAND_ACK_COMPLETED;
  r->reason_code = 0u;
}

static void fms_mission_complete(FmsEngine *e, const FmsInputs *in, uint16_t result)
{
  e->mission.state = MISSION_STATE_COMPLETED;
  e->mission.result_code = result;
  e->mission.token = 0u;
  e->mission.progress = (result == MISSION_RESULT_SUCCESS) ? 1.0f : e->mission.progress;
  e->mission.arrival_active = 0u;
  fms_mission_epoch_next(e);
  fms_permit_revoke(e, 20u);
  if (result == MISSION_RESULT_SUCCESS) {
    e->mission_completions++;
  }
  /* Mission over: hold position; operator must request LAND/RETURN explicitly. */
  (void)fms_mode_transition(e, FLIGHT_MODE_POS_HOLD, in, 1u);
  fms_authority_set(e, AUTHORITY_NONE, 0u, 0u);
}

static void fms_mission_advance(FmsEngine *e, const FmsInputs *in)
{
  e->mission.current_index++;
  e->mission.arrival_active = 0u;
  if (e->mission.current_index >= e->mission.wp_count) {
    fms_mission_complete(e, in, MISSION_RESULT_SUCCESS);
    return;
  }
  e->mission.progress = (float)e->mission.current_index / (float)e->mission.wp_count;
  if (!fms_mission_new_token(e)) {
    fms_mission_complete(e, in, MISSION_RESULT_FAILED);
    return;
  }
  e->mission.expires_at_us = fms_add_us(in->now_us, e->policy->mission_ttl_us);
  fms_permit_revoke(e, 12u); /* new item: old reference binding is stale */
}

void fms_mission_tick(FmsEngine *e, const FmsInputs *in)
{
  uint16_t idx;
  const ProductMissionWaypoint *wp;
  float dx, dy, dz, d2, vx, vy, vz, speed;

  if (e->mission.state == MISSION_STATE_LOADED || e->mission.state == MISSION_STATE_PAUSED) {
    /* Stored waypoints stay available while loaded or paused. Paused missions
     * publish no execution token, and resume rechecks current authority and frame. */
    e->mission.expires_at_us = fms_add_us(in->now_us, e->policy->mission_ttl_us);
    return;
  }
  if (e->mission.state != MISSION_STATE_RUNNING) {
    return;
  }
  /* M4 is the producer of the authorized snapshot: its own liveness must not
   * be judged by the TTL it republishes every cycle. Consumers use the TTL to
   * reject a stale snapshot; M4 refreshes it here. */
  e->mission.expires_at_us = fms_add_us(in->now_us, e->policy->mission_ttl_us);
  if (in->nav_ok == 0u) {
    return; /* hold progress; failsafe path handles capability loss */
  }
  idx = e->mission.current_index;
  if (idx >= e->mission.wp_count) {
    fms_mission_complete(e, in, MISSION_RESULT_FAILED);
    return;
  }
  wp = &e->mission.active.waypoints[idx];
  dx = in->nav->core.position_ned_m[0] - wp->position_ned_m[0];
  dy = in->nav->core.position_ned_m[1] - wp->position_ned_m[1];
  dz = in->nav->core.position_ned_m[2] - wp->position_ned_m[2];
  d2 = dx * dx + dy * dy + dz * dz;
  vx = in->nav->core.velocity_ned_mps[0];
  vy = in->nav->core.velocity_ned_mps[1];
  vz = in->nav->core.velocity_ned_mps[2];
  speed = sqrtf(vx * vx + vy * vy + vz * vz);

  if (d2 <= wp->acceptance_radius_m * wp->acceptance_radius_m &&
      speed <= e->policy->arrival_max_speed_mps) {
    if (e->mission.arrival_active == 0u) {
      e->mission.arrival_active = 1u;
      e->mission.arrival_since_us = in->now_us;
    } else if (in->now_us - e->mission.arrival_since_us >= e->policy->arrival_dwell_us) {
      fms_mission_advance(e, in);
    }
  } else {
    e->mission.arrival_active = 0u;
  }
}

void fms_mission_on_pilot_takeover(FmsEngine *e)
{
  if (e->mission.state != MISSION_STATE_RUNNING) {
    return;
  }
  e->mission.state = MISSION_STATE_PAUSED;
  e->mission.arrival_active = 0u;
  e->mission.token = 0u;
  fms_mission_epoch_next(e);
  fms_permit_revoke(e, 11u);
}

void fms_mission_suspend(FmsEngine *e, uint16_t reason)
{
  if (e->mission.state != MISSION_STATE_RUNNING) {
    return;
  }
  e->mission.state = MISSION_STATE_PAUSED;
  e->mission.arrival_active = 0u;
  e->mission.token = 0u;
  fms_mission_epoch_next(e);
  fms_permit_revoke(e, reason);
  if (e->authority.owner == AUTHORITY_MISSION) {
    fms_authority_set(e, AUTHORITY_NONE, 0u, 0u);
  }
}

void fms_mission_on_failsafe(FmsEngine *e)
{
  fms_mission_suspend(e, 21u);
}
