/*
 * M4 engine: single serialized update entry, input validation, command
 * dispatch, long-operation trackers and snapshot publication.
 */
#include <string.h>
#include <stddef.h>
#include <math.h>
#include "flight_state_machine/fms_internal.h"

/* ------------------------------------------------------------------ */
/* small bounded helpers                                              */
/* ------------------------------------------------------------------ */

uint64_t fms_add_us(uint64_t base, uint64_t delta)
{
  if (UINT64_MAX - base < delta) {
    return UINT64_MAX;
  }
  return base + delta;
}

uint8_t fms_u64_le(uint64_t a, uint64_t b) { return (uint8_t)(a <= b); }

uint8_t fms_finite(float v)
{
  return (uint8_t)((v == v) && (v != INFINITY) && (v != -INFINITY));
}

uint8_t fms_finite3(const float v[3])
{
  return (uint8_t)(fms_finite(v[0]) && fms_finite(v[1]) && fms_finite(v[2]));
}

uint8_t fms_finite4(const float v[4])
{
  return (uint8_t)(fms_finite(v[0]) && fms_finite(v[1]) && fms_finite(v[2]) && fms_finite(v[3]));
}

static uint8_t fms_seq_slot(uint16_t message_type)
{
  switch (message_type) {
  case MSG_VEHICLE_DIRECTIVE: return 0;
  case MSG_SAFETY_ENVELOPE: return 1;
  case MSG_ACTUATION_PERMIT: return 2;
  case MSG_AUTHORIZED_OPERATOR_INPUT: return 3;
  case MSG_AUTHORITY_STATUS: return 4;
  case MSG_FMS_STATE_SNAPSHOT: return 5;
  case MSG_MISSION_STATUS: return 6;
  case MSG_PRE_ARM_CHECK_RESULT: return 7;
  case MSG_LANDED_STATE_SNAPSHOT: return 8;
  case MSG_AUTHORIZED_MISSION_SNAPSHOT: return 9;
  case MSG_PRODUCT_GEOFENCE_SNAPSHOT: return 10;
  case MSG_COMMAND_ACK: return 11;
  case MSG_DECISION_EVENT: return 12;
  case MSG_MODULE_HEALTH: return 13;
  default: return 15;
  }
}

uint32_t fms_next_sequence(FmsEngine *e, uint16_t message_type)
{
  uint8_t slot = fms_seq_slot(message_type);
  uint32_t value = e->seq[slot] + 1u;
  if (value == 0u) {
    value = 1u;
  }
  e->seq[slot] = value;
  return value;
}

void fms_clear_header(ContractHeader *h) { memset(h, 0, sizeof(*h)); }

void fms_fill_header(FmsEngine *e, ContractHeader *h, uint16_t message_type,
                    uint32_t frame_id, uint64_t sample_us, uint64_t ttl_us)
{
  memset(h, 0, sizeof(*h));
  h->schema_major = CONTRACT_SCHEMA_MAJOR;
  h->schema_minor = CONTRACT_SCHEMA_MINOR;
  h->message_type = message_type;
  h->source_id = SOURCE_M4;
  h->sequence = fms_next_sequence(e, message_type);
  h->profile_id = PRODUCT_PROFILE_ID;
  h->frame_id = frame_id;
  h->boot_id = e->clock.boot_id;
  h->clock_domain_id = CLOCK_DOMAIN_FCU_MONOTONIC;
  h->time_epoch = e->clock.time_epoch;
  h->source_clock_domain_id = 0u;
  h->source_mapping_epoch = 0u;
  h->timestamp_sample_us = sample_us;
  h->timestamp_publish_us = e->clock.now_us;
  h->valid_until_us = fms_add_us(e->clock.now_us, ttl_us);
  h->event_time_uncertainty_us = 0u;
  /* These are decisions made on the native FCU clock, not sensor events. */
  h->timestamp_quality = TIMESTAMP_QUALITY_HARDWARE_CAPTURED;
  h->timestamp_flags = TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED;
  h->validity_flags = VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID;
}

void fms_zero_output(FmsOutputView *out) { memset(out, 0, sizeof(*out)); }

void fms_record_decision(FmsEngine *e, uint16_t code, uint16_t reason, uint32_t request_id)
{
  e->last_decision_code = code;
  e->last_decision_reason = reason;
  e->last_decision_request = request_id;
  e->decision_id++;
}

static void fms_ack_raw(FmsEngine *e, FmsOutputView *out, uint32_t request_id,
                       uint32_t session, uint16_t source, uint16_t ack_state,
                       uint16_t reason)
{
  CommandAck *ack;
  if (out->command_result_count >= FMS_MAX_ACKS) {
    return;
  }
  ack = &out->command_results[out->command_result_count++];
  fms_fill_header(e, &ack->header, MSG_COMMAND_ACK, FRAME_LOCAL_NED,
                 e->clock.now_us, 0u);
  ack->request_id = request_id;
  ack->source_session_id = session;
  ack->request_source_id = source;
  ack->ack_state = ack_state;
  ack->reason_code = reason;
  ack->completion_time_us = e->clock.now_us;
}

void fms_ack(FmsEngine *e, FmsOutputView *out, const OperationRequest *req,
            uint16_t ack_state, uint16_t reason_code)
{
  fms_ack_raw(e, out, req->request_id, req->source_session_id,
             req->header.source_id, ack_state, reason_code);
}

/* ------------------------------------------------------------------ */
/* input validation                                                   */
/* ------------------------------------------------------------------ */

static uint8_t fms_header_identity_ok(const FmsEngine *e, const ContractHeader *h)
{
  if (h->schema_major != CONTRACT_SCHEMA_MAJOR ||
      h->schema_minor != CONTRACT_SCHEMA_MINOR) {
    return 0;
  }
  if (h->profile_id != PRODUCT_PROFILE_ID) {
    return 0;
  }
  if (h->boot_id == 0u || h->boot_id != e->clock.boot_id) {
    return 0;
  }
  if (h->time_epoch != e->clock.time_epoch) {
    return 0;
  }
  return 1;
}

static uint8_t fms_header_fresh(const ContractHeader *h, uint64_t now)
{
  if ((h->validity_flags & (VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID)) !=
      (VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID) ||
      h->clock_domain_id != CLOCK_DOMAIN_FCU_MONOTONIC ||
      h->timestamp_sample_us > h->timestamp_publish_us ||
      h->valid_until_us == 0u || h->valid_until_us <= now) {
    return 0;
  }
  if (h->timestamp_publish_us > now) {
    return 0;
  }
  return 1;
}

static uint8_t fms_valid_fresh(const FmsEngine *e, const ContractHeader *h,
                              uint64_t now, uint16_t expected_type)
{
  if (h->message_type != expected_type) {
    return 0;
  }
  if (!fms_header_identity_ok(e, h)) {
    return 0;
  }
  return fms_header_fresh(h, now);
}

static const ContractHeader *fms_probe_header(const FmsInputView *iv)
{
  if (iv->navigation != NULL) {
    return &iv->navigation->header;
  }
  if (iv->mission_candidate != NULL) {
    return &iv->mission_candidate->header;
  }
  if (iv->guidance_status != NULL) {
    return &iv->guidance_status->header;
  }
  if (iv->operator_input != NULL) {
    return &iv->operator_input->header;
  }
  if (iv->link_status_count > 0u && iv->link_statuses != NULL) {
    return &iv->link_statuses[0].header;
  }
  if (iv->pending_request_count > 0u && iv->pending_requests != NULL) {
    return &iv->pending_requests[0].header;
  }
  return NULL;
}

static void fms_bind_clock(FmsEngine *e, const FmsInputView *iv)
{
  const ContractHeader *h = fms_probe_header(iv);

  if (h == NULL) {
    return;
  }
  if (h->schema_major != CONTRACT_SCHEMA_MAJOR ||
      h->schema_minor != CONTRACT_SCHEMA_MINOR ||
      h->profile_id != PRODUCT_PROFILE_ID || h->boot_id == 0u) {
    return;
  }
  e->clock.boot_id = h->boot_id;
  e->clock.time_epoch = h->time_epoch;
  e->clock.profile_id = h->profile_id;
  e->clock.bound = 1u;
}

static void fms_collect_inputs(FmsEngine *e, const FmsInputView *iv, FmsInputs *in)
{
  uint16_t i;

  memset(in, 0, sizeof(*in));
  in->now_us = iv->evaluation_time_us;

  if ((iv->presence_mask & FMS_INPUT_HAS_NAV) != 0u && iv->navigation != NULL &&
      fms_valid_fresh(e, &iv->navigation->header, in->now_us, MSG_NAVIGATION_SNAPSHOT)) {
    in->nav = iv->navigation;
    in->nav_ok = 1u;
  }
  if ((iv->presence_mask & FMS_INPUT_HAS_NAV_CAPABILITY) != 0u &&
      iv->navigation_capability != NULL &&
      fms_valid_fresh(e, &iv->navigation_capability->header, in->now_us,
                     MSG_NAVIGATION_CAPABILITY)) {
    in->cap = iv->navigation_capability;
    in->cap_ok = 1u;
  }
  if ((iv->presence_mask & FMS_INPUT_HAS_OPERATOR) != 0u && iv->operator_input != NULL &&
      fms_valid_fresh(e, &iv->operator_input->header, in->now_us, MSG_OPERATOR_INPUT)) {
    in->op = iv->operator_input;
    in->op_ok = 1u;
  }
  if ((iv->presence_mask & FMS_INPUT_HAS_MISSION_CANDIDATE) != 0u &&
      iv->mission_candidate != NULL &&
      fms_valid_fresh(e, &iv->mission_candidate->header, in->now_us,
                     MSG_MISSION_UPLOAD_CANDIDATE)) {
    in->candidate = iv->mission_candidate;
    in->candidate_ok = 1u;
  }
  if ((iv->presence_mask & FMS_INPUT_HAS_GUIDANCE) != 0u &&
      iv->guidance_feasibility != NULL &&
      fms_valid_fresh(e, &iv->guidance_feasibility->header, in->now_us,
                     MSG_GUIDANCE_FEASIBILITY)) {
    in->feas = iv->guidance_feasibility;
    in->feas_ok = 1u;
  }
  if ((iv->presence_mask & FMS_INPUT_HAS_GUIDANCE_STATUS) != 0u &&
      iv->guidance_status != NULL &&
      fms_valid_fresh(e, &iv->guidance_status->header, in->now_us, MSG_GUIDANCE_STATUS)) {
    in->guidance = iv->guidance_status;
    in->guidance_ok = 1u;
  }
  if ((iv->presence_mask & FMS_INPUT_HAS_CONTROL) != 0u && iv->control_capability != NULL &&
      fms_valid_fresh(e, &iv->control_capability->header, in->now_us,
                     MSG_CONTROL_CAPABILITY)) {
    in->control = iv->control_capability;
    in->control_ok = 1u;
  }
  if ((iv->presence_mask & FMS_INPUT_HAS_ACTUATOR) != 0u &&
      iv->actuator_capability != NULL && iv->actuator_status != NULL &&
      fms_valid_fresh(e, &iv->actuator_capability->header, in->now_us,
                     MSG_ACTUATOR_CAPABILITY) &&
      fms_valid_fresh(e, &iv->actuator_status->header, in->now_us, MSG_ACTUATOR_STATUS)) {
    in->act_cap = iv->actuator_capability;
    in->act_status = iv->actuator_status;
    in->act_ok = 1u;
  }
  if ((iv->presence_mask & FMS_INPUT_HAS_FAULT_SUMMARY) != 0u && iv->fault_summary != NULL &&
      fms_valid_fresh(e, &iv->fault_summary->header, in->now_us, MSG_FAULT_SUMMARY)) {
    in->faults = iv->fault_summary;
    in->faults_ok = 1u;
  }
  if ((iv->presence_mask & FMS_INPUT_HAS_PRODUCT_EXTENSION) != 0u &&
      (iv->product_extension.header.validity_flags & VALIDITY_PAYLOAD_VALID) != 0u &&
      fms_valid_fresh(e, &iv->product_extension.header, in->now_us,
                     MSG_PRODUCT_FMS_INPUT_EXTENSION)) {
    in->ext = &iv->product_extension;
    in->ext_ok = 1u;
    in->ext_mask_valid = (uint8_t)iv->product_extension.valid_mask;
    if ((iv->product_extension.valid_mask & PRODUCT_FMS_EXT_GROUND_CONTACT) != 0u) {
      in->ground_contact = iv->product_extension.ground_contact_state;
    }
    if ((iv->product_extension.valid_mask & PRODUCT_FMS_EXT_ENERGY_STATE) != 0u) {
      in->energy_state = iv->product_extension.energy_state;
    }
    if ((iv->product_extension.valid_mask & PRODUCT_FMS_EXT_BATTERY_FRACTION) != 0u) {
      in->battery_fraction = iv->product_extension.battery_remaining_fraction;
    }
    if ((iv->product_extension.valid_mask & PRODUCT_FMS_EXT_GEOFENCE_STATE) != 0u) {
      in->geofence_state = iv->product_extension.geofence_state;
    }
  }

  if (iv->link_status_count > FMS_MAX_LINKS) {
    in->link_count = 0u;
  } else {
    in->link_count = iv->link_status_count;
    for (i = 0u; i < in->link_count; ++i) {
      in->links[i] = &iv->link_statuses[i];
    }
  }
}

static void fms_classify_links(FmsEngine *e, FmsInputs *in)
{
  uint16_t i;
  uint64_t timeout = e->config.payload.rc_lost_timeout_us;
  uint64_t now = in->now_us;

  for (i = 0u; i < in->link_count; ++i) {
    const CommunicationLinkStatus *l = in->links[i];
    uint8_t fresh = fms_valid_fresh(e, &l->header, now, MSG_COMMUNICATION_LINK_STATUS);
    if (!fresh) {
      continue;
    }
    if (l->link_kind == LINK_KIND_RC) {
      in->rc_seen = 1u;
      in->rc_state = (uint8_t)l->link_state;
      in->rc_failsafe = (uint8_t)((l->flags & LINK_FLAG_RECEIVER_FAILSAFE) != 0u);
      in->rc_fresh = (uint8_t)((l->link_state == LINK_STATE_HEALTHY ||
                                l->link_state == LINK_STATE_DEGRADED ||
                                l->link_state == LINK_STATE_RECOVERING) &&
                               (l->last_valid_frame_us != 0u) &&
                               (l->last_valid_frame_us <= now) &&
                               (now - l->last_valid_frame_us <= timeout));
      if (in->rc_failsafe) {
        in->rc_fresh = 0u;
      }
    } else if (l->link_kind == LINK_KIND_GCS) {
      in->gcs_seen = 1u;
      in->gcs_state = (uint8_t)l->link_state;
      in->gcs_fresh = (uint8_t)(l->link_state == LINK_STATE_HEALTHY ||
                                l->link_state == LINK_STATE_DEGRADED);
    } else if (l->link_kind == LINK_KIND_COMPANION) {
      in->cmp_seen = 1u;
      in->cmp_state = (uint8_t)l->link_state;
      in->cmp_fresh = (uint8_t)(l->link_state == LINK_STATE_HEALTHY ||
                                l->link_state == LINK_STATE_DEGRADED);
    }
  }

  e->links.rc_present = in->rc_seen;
  e->links.gcs_present = in->gcs_seen;
  e->links.cmp_present = in->cmp_seen;
  e->links.rc_state = in->rc_state;
  e->links.gcs_state = in->gcs_state;
  e->links.cmp_state = in->cmp_state;
  e->links.rc_failsafe = in->rc_failsafe;
  e->links.rc_fresh = in->rc_fresh;
  e->links.gcs_fresh = in->gcs_fresh;
  e->links.cmp_fresh = in->cmp_fresh;
}

/* ------------------------------------------------------------------ */
/* command handling                                                   */
/* ------------------------------------------------------------------ */

enum {
  FMS_DEDUP_NEW = 0,
  FMS_DEDUP_REPLAY = 1,
  FMS_DEDUP_CONFLICT = 2,
  FMS_DEDUP_FULL = 3
};

static uint8_t fms_dedup_lookup(FmsEngine *e, const OperationRequest *req, uint64_t now,
                               FmsDedupEntry **out)
{
  uint16_t i;
  FmsDedupEntry *free_slot = NULL;

  *out = NULL;
  for (i = 0u; i < FMS_DEDUP_CAPACITY; ++i) {
    FmsDedupEntry *d = &e->dedup[i];
    if (d->used == 0u) {
      if (free_slot == NULL) {
        free_slot = d;
      }
      continue;
    }
    if (now > d->seen_us && (now - d->seen_us) > FMS_DEDUP_TTL_US) {
      d->used = 0u;
      if (free_slot == NULL) {
        free_slot = d;
      }
      continue;
    }
    if (d->request_id == req->request_id && d->session == req->source_session_id &&
        d->source == req->header.source_id) {
      if (d->command_code != req->command_code ||
          memcmp(&d->payload, &req->payload, sizeof(d->payload)) != 0) {
        return FMS_DEDUP_CONFLICT;
      }
      *out = d;
      return FMS_DEDUP_REPLAY;
    }
  }
  if (free_slot != NULL) {
    *out = free_slot;
    return FMS_DEDUP_NEW;
  }
  return FMS_DEDUP_FULL;
}

static void fms_handle_request(FmsEngine *e, const FmsInputs *in,
                              const OperationRequest *req, FmsCmdResult *result)
{
  FmsDedupEntry *slot;
  FmsCmdResult r;

  r.ack_state = COMMAND_ACK_REJECTED;
  r.reason_code = 1u; /* INVALID_ARGUMENT */

  if (req->source_session_id == 0u || req->request_id == 0u) {
    *result = r;
    e->command_rejects++;
    return;
  }
  if (!fms_header_identity_ok(e, &req->header)) {
    result->ack_state = COMMAND_ACK_REJECTED;
    result->reason_code = 12u; /* EPOCH */
    e->epoch_failures++;
    return;
  }
  if (req->header.message_type != MSG_OPERATION_REQUEST ||
      !fms_header_fresh(&req->header, in->now_us)) {
    *result = r;
    e->command_rejects++;
    return;
  }

  slot = NULL;
  switch (fms_dedup_lookup(e, req, in->now_us, &slot)) {
  case FMS_DEDUP_CONFLICT:
    result->ack_state = COMMAND_ACK_REJECTED;
    result->reason_code = 16u; /* REPLAY: same key, different payload */
    e->command_rejects++;
    return;
  case FMS_DEDUP_FULL:
    result->ack_state = COMMAND_ACK_REJECTED;
    result->reason_code = 9u; /* CAPACITY */
    e->command_rejects++;
    return;
  case FMS_DEDUP_REPLAY:
    /* Replay of an already processed idempotent request: same answer, no side effect. */
    result->ack_state = slot->ack_state;
    result->reason_code = slot->reason;
    return;
  default:
    break;
  }
  if (req->execution_deadline_us == 0u || req->execution_deadline_us <= in->now_us) {
    result->ack_state = COMMAND_ACK_REJECTED;
    result->reason_code = 6u; /* DEADLINE */
    e->command_rejects++;
    return;
  }

  switch (req->command_code) {
  case PRODUCT_CMD_ARM:
  case PRODUCT_CMD_DISARM:
  case PRODUCT_CMD_TAKEOFF:
  case PRODUCT_CMD_LAND:
  case PRODUCT_CMD_RETURN:
  case PRODUCT_CMD_SET_MODE:
  case PRODUCT_CMD_MISSION_COMMIT:
  case PRODUCT_CMD_MISSION_START:
  case PRODUCT_CMD_MISSION_PAUSE:
  case PRODUCT_CMD_MISSION_RESUME:
  case PRODUCT_CMD_MISSION_ABORT:
  case PRODUCT_CMD_MISSION_CLEAR:
  case PRODUCT_CMD_SET_HOME:
    break;
  default:
    result->ack_state = COMMAND_ACK_REJECTED;
    result->reason_code = 4u; /* UNSUPPORTED */
    e->command_rejects++;
    return;
  }

  {
    /* CR-V191-C01: mission_epoch is applicable only to the five mission
     * lifecycle commands (0 = not applicable, valid bit must be 0). */
    uint8_t epoch_cmd = (uint8_t)(req->command_code == PRODUCT_CMD_MISSION_START ||
                                  req->command_code == PRODUCT_CMD_MISSION_PAUSE ||
                                  req->command_code == PRODUCT_CMD_MISSION_RESUME ||
                                  req->command_code == PRODUCT_CMD_MISSION_ABORT ||
                                  req->command_code == PRODUCT_CMD_MISSION_CLEAR);
    if (epoch_cmd == 0u &&
        ((req->payload.valid_mask & PRODUCT_OPERATION_VALID_MISSION_EPOCH) != 0u ||
         req->payload.mission_epoch != 0u)) {
      result->ack_state = COMMAND_ACK_REJECTED;
      result->reason_code = 1u; /* INVALID_ARGUMENT: field not applicable */
      e->command_rejects++;
      return;
    }
  }

  if (slot != NULL) {
    slot->used = 1u;
    slot->request_id = req->request_id;
    slot->session = req->source_session_id;
    slot->source = req->header.source_id;
    slot->command_code = req->command_code;
    slot->payload = req->payload;
    slot->seen_us = in->now_us;
    slot->ack_state = COMMAND_ACK_REJECTED;
    slot->reason = 1u;
  }

  if ((e->flight.terminal_latched || e->flight.critical_energy_latched) &&
      (req->command_code == PRODUCT_CMD_TAKEOFF || req->command_code == PRODUCT_CMD_RETURN ||
       req->command_code == PRODUCT_CMD_SET_MODE || req->command_code == PRODUCT_CMD_MISSION_START ||
       req->command_code == PRODUCT_CMD_MISSION_RESUME)) {
    r.reason_code = 11u;
  } else if (e->authority.latched != 0u && req->header.source_id != SOURCE_RC &&
      (req->command_code == PRODUCT_CMD_TAKEOFF || req->command_code == PRODUCT_CMD_LAND ||
       req->command_code == PRODUCT_CMD_RETURN)) {
    r.reason_code = 11u; /* Ordinary remote motion cannot override pilot takeover. */
  } else switch (req->command_code) {
  case PRODUCT_CMD_ARM: {
    uint8_t mode;
    if (e->flight.arm_state != ARM_STATE_DISARMED || e->flight.terminal_latched != 0u ||
        e->flight.critical_energy_latched != 0u) {
      r.ack_state = COMMAND_ACK_REJECTED;
      r.reason_code = 11u; /* NOT_PERMITTED */
      break;
    }
    if ((req->payload.valid_mask & PRODUCT_OPERATION_VALID_PREARM_EPOCH) == 0u) {
      r.reason_code = 1u;
      break;
    }
    if (e->config.payload.output_policy_id == 0u) {
      r.reason_code = 19u; /* POLICY_UNVERIFIED */
      break;
    }
    if (req->payload.prearm_check_epoch != e->prearm.check_epoch ||
        e->prearm.prearm_state != PREARM_READY ||
        !fms_prearm_ready_for_arm(e, in)) {
      r.ack_state = COMMAND_ACK_REJECTED;
      r.reason_code = 2u; /* NOT_READY */
      fms_record_decision(e, DECISION_REJECT, r.reason_code, req->request_id);
      break;
    }
    for (mode = FLIGHT_MODE_POS_HOLD; mode != FLIGHT_MODE_NONE; --mode) {
      if (fms_capable_for_mode(e, in, mode, 1u)) break;
    }
    if (mode == FLIGHT_MODE_NONE) {
      r.reason_code = 2u;
      break;
    }
    e->flight.arm_state = ARM_STATE_ARMING;
    e->flight.arming_start_us = in->now_us;
    e->flight.arm_stable_us = in->now_us;
    {
      uint16_t home_reason = 0u;
      if (e->home.valid == 0u || e->home.snap.capture_source != PRODUCT_HOME_CAPTURE_OPERATOR) {
        fms_home_capture(e, in, PRODUCT_HOME_CAPTURE_AUTO_ARM, &home_reason);
      }
      if (e->home.snap.validity != PRODUCT_HOME_VALID) {
        e->flight.arm_state = ARM_STATE_DISARMED;
        r.ack_state = COMMAND_ACK_REJECTED;
        r.reason_code = (home_reason != 0u) ? home_reason : 2u;
        break;
      }
    }
    if (in->act_ok && in->act_cap->available_mask == 0x0Fu &&
        in->act_status->fault_mask == 0u) {
      e->flight.arm_state = ARM_STATE_ARMED;
      e->flight.control_link_established = 0u;
      e->permit.reference_rebind_until_us = 0u;
      (void)fms_mode_transition(e, mode, in, 1u);
      r.ack_state = COMMAND_ACK_COMPLETED;
      r.reason_code = 0u;
      fms_record_decision(e, DECISION_ACCEPT, 0u, req->request_id);
    } else {
      e->flight.arm_state = ARM_STATE_DISARMED;
      r.ack_state = COMMAND_ACK_REJECTED;
      r.reason_code = 2u;
    }
    break;
  }

  case PRODUCT_CMD_DISARM:
    if (e->flight.arm_state != ARM_STATE_ARMED) {
      r.reason_code = 11u;
      break;
    }
    if (e->land.state != LANDED_STATE_LANDED ||
        e->land.stable_ms < e->config.payload.landed_dwell_us / 1000u) {
      r.reason_code = 11u; /* airborne or unproven ground contact */
      break;
    }
    e->flight.arm_state = ARM_STATE_DISARMING;
    fms_permit_revoke(e, 20u /* CANCELLED */);
    e->authority.owner = AUTHORITY_NONE;
    e->authority.latched = 0u;
    e->authority.source_id = 0u;
    e->flight.flight_mode = FLIGHT_MODE_NONE;
    e->flight.control_link_established = 0u;
    if (e->mission.state == MISSION_STATE_RUNNING ||
        e->mission.state == MISSION_STATE_PAUSED) {
      e->mission.state = MISSION_STATE_ABORTED;
      e->mission.result_code = MISSION_RESULT_ABORTED;
      e->mission.token = 0u;
    }
    e->flight.arm_state = ARM_STATE_DISARMED;
    r.ack_state = COMMAND_ACK_COMPLETED;
    r.reason_code = 0u;
    fms_record_decision(e, DECISION_ACCEPT, 0u, req->request_id);
    break;

  case PRODUCT_CMD_TAKEOFF: {
    FmsTracker *t = NULL;
    uint8_t i;
    if (e->flight.arm_state != ARM_STATE_ARMED) {
      r.reason_code = 2u;
      break;
    }
    if (e->home.snap.validity != PRODUCT_HOME_VALID) {
      r.reason_code = 2u;
      break;
    }
    if ((req->payload.valid_mask & PRODUCT_OPERATION_VALID_ALTITUDE_ABOVE_HOME) == 0u ||
        !fms_finite(req->payload.requested_altitude_m) ||
        req->payload.requested_altitude_m <= 0.0f ||
        req->payload.requested_altitude_m < e->config.payload.vertical_position_std_limit_m ||
        req->payload.requested_altitude_m > e->policy->default_max_height_above_home_m) {
      r.reason_code = 3u; /* OUT_OF_RANGE */
      break;
    }
    if (!fms_capable_for_mode(e, in, FLIGHT_MODE_POS_HOLD, 1u) ||
        e->land.state != LANDED_STATE_LANDED) { r.reason_code = 5u; break; }
    for (i = 0u; i < FMS_MAX_TRACKERS; ++i) {
      if (e->trackers[i].active == 0u) {
        t = &e->trackers[i];
        break;
      }
    }
    if (t == NULL) {
      r.reason_code = 9u; /* CAPACITY */
      break;
    }
    memset(t, 0, sizeof(*t));
    t->kind = FMS_TRACKER_TAKEOFF;
    t->active = 1u;
    t->source = req->header.source_id;
    t->request_id = req->request_id;
    t->session = req->source_session_id;
    t->started_us = in->now_us;
    t->deadline_us = fms_add_us(in->now_us, e->policy->long_op_deadline_us);
    t->last_progress_us = in->now_us;
    t->target_altitude_m = req->payload.requested_altitude_m;
    memcpy(t->target_position_ned_m, in->nav->core.position_ned_m, sizeof(t->target_position_ned_m));
    t->target_position_ned_m[2] = e->home.snap.position_ned_m[2] - t->target_altitude_m;
    t->target_yaw_rad = atan2f(2*(in->nav->core.q_nb_wxyz[0]*in->nav->core.q_nb_wxyz[3]+
        in->nav->core.q_nb_wxyz[1]*in->nav->core.q_nb_wxyz[2]),
        1-2*(in->nav->core.q_nb_wxyz[2]*in->nav->core.q_nb_wxyz[2]+
             in->nav->core.q_nb_wxyz[3]*in->nav->core.q_nb_wxyz[3]));
    (void)fms_mode_transition(e, FLIGHT_MODE_POS_HOLD, in, 1u);
    e->flight.directive_epoch++; /* target change even when the mode stays POS_HOLD */
    t->directive_epoch = e->flight.directive_epoch;
    fms_permit_revoke(e, 12u);
    r.ack_state = COMMAND_ACK_ACCEPTED;
    r.reason_code = 0u;
    fms_record_decision(e, DECISION_ACCEPT, 0u, req->request_id);
    break;
  }

  case PRODUCT_CMD_LAND: {
    FmsTracker *t = NULL;
    uint8_t i;
    if (e->flight.arm_state != ARM_STATE_ARMED) {
      r.reason_code = 2u;
      break;
    }
    if (e->land.state == LANDED_STATE_LANDED) {
      r.ack_state = COMMAND_ACK_COMPLETED;
      r.reason_code = 0u;
      break;
    }
    if (e->flight.terminal_latched != 0u ||
        !fms_capable_for_mode(e, in, FLIGHT_MODE_AUTO_LAND, 1u)) {
      r.reason_code = 11u;
      break;
    }
    for (i = 0u; i < FMS_MAX_TRACKERS; ++i) {
      if (e->trackers[i].active == 0u) {
        t = &e->trackers[i];
        break;
      }
    }
    if (t == NULL) {
      r.reason_code = 9u;
      break;
    }
    memset(t, 0, sizeof(*t));
    t->kind = FMS_TRACKER_LAND;
    t->active = 1u;
    t->source = req->header.source_id;
    t->request_id = req->request_id;
    t->session = req->source_session_id;
    t->started_us = in->now_us;
    t->deadline_us = fms_add_us(in->now_us, e->policy->long_op_deadline_us);
    t->last_progress_us = in->now_us;
    (void)fms_mode_transition(e, FLIGHT_MODE_AUTO_LAND, in, 1u);
    fms_mission_on_failsafe(e); /* A landing cannot retain a waypoint execution token. */
    r.ack_state = COMMAND_ACK_ACCEPTED;
    r.reason_code = 0u;
    break;
  }

  case PRODUCT_CMD_RETURN: {
    FmsTracker *t = NULL;
    uint8_t i;
    const uint8_t returning_from_mission = e->mission.state == MISSION_STATE_RUNNING &&
                                           e->flight.flight_mode == FLIGHT_MODE_AUTO_MISSION;
    if (e->flight.arm_state != ARM_STATE_ARMED ||
        e->flight.failsafe_level >= FAILSAFE_ACTION_REQUIRED ||
        e->home.snap.validity != PRODUCT_HOME_VALID ||
        !fms_capable_for_mode(e, in, FLIGHT_MODE_AUTO_RETURN, 1u)) {
      r.reason_code = 11u;
      break;
    }
    for (i = 0u; i < FMS_MAX_TRACKERS; ++i) {
      if (e->trackers[i].active != 0u && e->trackers[i].kind == FMS_TRACKER_RETURN) {
        t = NULL;
        break;
      }
      if (t == NULL && e->trackers[i].active == 0u) {
        t = &e->trackers[i];
      }
    }
    if (t == NULL) {
      r.reason_code = 9u;
      break;
    }
    memset(t, 0, sizeof(*t));
    t->kind = FMS_TRACKER_RETURN;
    t->active = 1u;
    t->source = req->header.source_id;
    t->request_id = req->request_id;
    t->session = req->source_session_id;
    t->started_us = in->now_us;
    t->deadline_us = fms_add_us(in->now_us, e->policy->long_op_deadline_us);
    t->last_progress_us = in->now_us;
    t->home_token = e->home.snap.home_id;
    if (returning_from_mission) {
      t->preparing_return = FMS_RETURN_WAIT_MISSION_HANDOFF;
    } else if (e->mission.state == MISSION_STATE_COMPLETED &&
               e->mission.result_code == MISSION_RESULT_SUCCESS &&
               e->flight.flight_mode == FLIGHT_MODE_POS_HOLD) {
      t->preparing_return = FMS_RETURN_STABILIZE_HOLD;
    }
    if (t->preparing_return == 0u) {
      (void)fms_mode_transition(e, FLIGHT_MODE_AUTO_RETURN, in, 1u);
    }
    t->directive_epoch = e->flight.directive_epoch;
    r.ack_state = COMMAND_ACK_ACCEPTED;
    r.reason_code = 0u;
    break;
  }

  case PRODUCT_CMD_SET_MODE:
    if (e->flight.arm_state != ARM_STATE_ARMED) {
      r.reason_code = 2u;
      break;
    }
    if ((req->payload.valid_mask & PRODUCT_OPERATION_VALID_REQUESTED_MODE) == 0u ||
        req->payload.requested_mode > FLIGHT_MODE_EMERGENCY_LAND) {
      r.reason_code = 1u;
      break;
    }
    if (e->authority.latched != 0u && req->header.source_id != 1u) {
      r.reason_code = 11u; /* GCS may not override a latched pilot */
      break;
    }
    if (!fms_mode_transition(e, (uint8_t)req->payload.requested_mode, in, 0u)) {
      r.reason_code = 2u;
      break;
    }
    r.ack_state = COMMAND_ACK_COMPLETED;
    r.reason_code = 0u;
    break;

  case PRODUCT_CMD_MISSION_COMMIT:
    fms_mission_commit(e, in, req, &r);
    break;
  case PRODUCT_CMD_MISSION_START:
    fms_mission_start(e, in, req, &r);
    break;
  case PRODUCT_CMD_MISSION_PAUSE:
    fms_mission_pause(e, in, req, &r);
    break;
  case PRODUCT_CMD_MISSION_RESUME:
    fms_mission_resume(e, in, req, &r);
    break;
  case PRODUCT_CMD_MISSION_ABORT:
    fms_mission_abort(e, in, req, &r);
    break;
  case PRODUCT_CMD_MISSION_CLEAR:
    fms_mission_clear(e, in, req, &r);
    break;

  case PRODUCT_CMD_SET_HOME:
    if (e->flight.arm_state != ARM_STATE_DISARMED) {
      r.reason_code = 11u;
      break;
    }
    if ((req->payload.valid_mask & PRODUCT_OPERATION_VALID_TARGET_NED) == 0u ||
        !fms_finite3(req->payload.target_ned_m)) {
      r.reason_code = 1u;
      break;
    }
    if (!in->nav_ok || in->nav->origin_id == 0u) {
      r.reason_code = 13u; /* FRAME_ORIGIN */
      break;
    }
    if (!fms_capable_for_mode(e, in, FLIGHT_MODE_POS_HOLD, 0u) ||
        !fms_geofence_contains(&e->geofence, req->payload.target_ned_m,
                               req->payload.target_ned_m[2])) {
      r.reason_code = 11u;
      break;
    }
    memset(&e->home.snap, 0, sizeof(e->home.snap));
    e->home.snap.home_id = req->request_id;
    e->home.snap.home_revision = 1u;
    e->home.snap.frame_id = FRAME_LOCAL_NED;
    e->home.snap.origin_id = in->nav->origin_id;
    e->home.snap.state_epoch = in->nav->state_epoch;
    e->home.snap.validity = PRODUCT_HOME_VALID;
    e->home.snap.quality = 0u;
    e->home.snap.capture_source = PRODUCT_HOME_CAPTURE_OPERATOR;
    e->home.snap.captured_at_us = in->now_us;
    e->home.snap.home_expires_at_us = fms_add_us(in->now_us, e->policy->home_ttl_us);
    e->home.snap.position_ned_m[0] = req->payload.target_ned_m[0];
    e->home.snap.position_ned_m[1] = req->payload.target_ned_m[1];
    e->home.snap.position_ned_m[2] = req->payload.target_ned_m[2];
    memcpy(e->home.snap.position_std_m, in->nav->core.position_std_m,
           sizeof(e->home.snap.position_std_m));
    e->home.valid = 1u;
    r.ack_state = COMMAND_ACK_COMPLETED;
    r.reason_code = 0u;
    break;

  default:
    r.reason_code = 4u;
    break;
  }

  *result = r;
  if (slot != NULL) {
    slot->ack_state = r.ack_state;
    slot->reason = r.reason_code;
  }
  if (r.ack_state != COMMAND_ACK_COMPLETED) {
    if (r.ack_state == COMMAND_ACK_REJECTED) {
      e->command_rejects++;
    }
  } else {
    fms_record_decision(e, DECISION_ACCEPT, 0u, req->request_id);
  }
}

static void fms_process_requests(FmsEngine *e, const FmsInputs *in, const FmsInputView *iv,
                                FmsOutputView *out)
{
  uint16_t i;
  uint16_t count = iv->pending_request_count;

  if (count > FMS_MAX_ACKS) {
    count = FMS_MAX_ACKS;
  }
  for (i = 0u; i < count; ++i) {
    FmsCmdResult r;
    fms_handle_request(e, in, &iv->pending_requests[i], &r);
    fms_ack(e, out, &iv->pending_requests[i], r.ack_state, r.reason_code);
  }
}

/* ------------------------------------------------------------------ */
/* long-operation trackers                                            */
/* ------------------------------------------------------------------ */

static void fms_tracker_ack(FmsEngine *e, FmsOutputView *out, const FmsTracker *t,
                            uint16_t state, uint16_t reason)
{
  for (unsigned i = 0; i < FMS_DEDUP_CAPACITY; ++i) {
    FmsDedupEntry *entry = &e->dedup[i];
    if (entry->used != 0u && entry->request_id == t->request_id &&
        entry->session == t->session && entry->source == t->source) {
      entry->ack_state = state;
      entry->reason = reason;
      entry->seen_us = e->clock.now_us;
      break;
    }
  }
  fms_ack_raw(e, out, t->request_id, t->session, t->source, state, reason);
}

static uint8_t fms_return_control_is_bound(const FmsEngine *e, const FmsInputs *in)
{
  const uint32_t required = NAV_VALID_HORIZONTAL_VELOCITY | NAV_VALID_VERTICAL_VELOCITY |
                            NAV_VALID_ACCELERATION;
  if (!in->nav_ok || !in->cap_ok || !in->control_ok || !in->act_ok ||
      (in->nav->core.valid_mask & in->cap->capability_mask & required) != required ||
      in->act_status->fault_mask != 0u || !e->permit.active ||
      e->permit.scope != PERMIT_SCOPE_CONTROL_ACTIVE ||
      e->permit.directive_epoch != e->flight.directive_epoch ||
      e->permit.authority_epoch != e->authority.epoch ||
      e->permit.expires_at_us <= in->now_us ||
      in->control->header.source_id != SOURCE_M6 ||
      in->control->header.timestamp_sample_us < e->permit.binding_since_us)
    return 0u;
  return 1u;
}

static uint8_t fms_return_hold_is_stable(const FmsEngine *e, const FmsInputs *in)
{
  if (!fms_return_control_is_bound(e, in)) return 0u;
  const float *v = in->nav->core.velocity_ned_mps;
  const float *a = fms_motion_acceleration(e, in);
  return a != NULL && hypotf(hypotf(v[0], v[1]), v[2]) < 0.15f &&
         hypotf(hypotf(a[0], a[1]), a[2]) < 0.4f;
}

static void fms_tracker_tick(FmsEngine *e, const FmsInputs *in, FmsOutputView *out)
{
  uint8_t i;
  for (i = 0u; i < FMS_MAX_TRACKERS; ++i) {
    FmsTracker *t = &e->trackers[i];
    uint8_t complete = 0u;
    uint16_t reason = 0u;

    if (t->active == 0u) {
      continue;
    }
    if (e->flight.arm_state != ARM_STATE_ARMED || e->flight.failsafe_level == FAILSAFE_TERMINAL ||
        (t->kind == FMS_TRACKER_TAKEOFF && t->directive_epoch != e->flight.directive_epoch)) {
      t->active = 0;
      fms_tracker_ack(e, out, t, COMMAND_ACK_FAILED, 12u);
      continue;
    }
    if ((t->kind == FMS_TRACKER_LAND && e->flight.flight_mode != FLIGHT_MODE_AUTO_LAND) ||
        (t->kind == FMS_TRACKER_RETURN && t->preparing_return == 0u &&
         e->flight.flight_mode != FLIGHT_MODE_AUTO_RETURN)) {
      t->active = 0u;
      fms_tracker_ack(e, out, t, COMMAND_ACK_CANCELLED, 12u);
      continue;
    }
    if (in->now_us >= t->deadline_us) {
      t->active = 0u;
      fms_tracker_ack(e, out, t, COMMAND_ACK_FAILED, 6u);
      continue;
    }

    if (t->kind == FMS_TRACKER_TAKEOFF) {
      if (in->nav_ok && in->nav->core.valid_mask != 0u) {
        float height = -in->nav->core.position_ned_m[2];
        float home_h = -e->home.snap.position_ned_m[2];
        float rel = height - home_h;
        float vz = in->nav->core.velocity_ned_mps[2];
        if (fabsf(rel-t->target_altitude_m) <= FMS_TAKEOFF_ALT_TOL_M &&
            fms_finite(vz) && fabsf(vz) <= FMS_TAKEOFF_VZ_TOL_MPS) {
          if (t->stable_since_us == 0u) {
            t->stable_since_us = in->now_us;
          }
          if (in->now_us - t->stable_since_us >= FMS_TAKEOFF_STABLE_US) {
            complete = 1u;
          }
        } else {
          t->stable_since_us = 0u;
        }
      }
    } else if (t->kind == FMS_TRACKER_LAND) {
      if (e->land.state == LANDED_STATE_LANDED &&
          e->land.stable_ms >= e->config.payload.landed_dwell_us / 1000u) {
        complete = 1u;
      }
    } else if (t->kind == FMS_TRACKER_RETURN) {
      if (t->preparing_return == FMS_RETURN_WAIT_MISSION_HANDOFF) {
        const uint8_t running = e->mission.state == MISSION_STATE_RUNNING &&
                                e->flight.flight_mode == FLIGHT_MODE_AUTO_MISSION &&
                                e->flight.directive_epoch == t->directive_epoch;
        const uint8_t completed = e->mission.state == MISSION_STATE_COMPLETED &&
                                  e->mission.result_code == MISSION_RESULT_SUCCESS &&
                                  e->flight.flight_mode == FLIGHT_MODE_POS_HOLD;
        if (e->flight.failsafe_level >= FAILSAFE_ACTION_REQUIRED || e->authority.latched ||
            (!running && !completed) || !e->home.valid || e->home.snap.home_id != t->home_token) {
          t->active = 0u;
          fms_tracker_ack(e, out, t, COMMAND_ACK_CANCELLED, 12u);
          continue;
        }
        if (in->now_us - t->started_us >= FMS_RETURN_PREPARE_TIMEOUT_US) {
          t->active = 0u;
          fms_tracker_ack(e, out, t, COMMAND_ACK_FAILED, 6u);
          continue;
        }
        if (completed ||
            (fms_return_control_is_bound(e, in) &&
             in->control->header.timestamp_sample_us - e->permit.binding_since_us >= 80000u &&
             fms_mode_transition(e, FLIGHT_MODE_POS_HOLD, in, 1u))) {
          fms_mission_suspend(e, 20u);
          t->preparing_return = FMS_RETURN_STABILIZE_HOLD;
          t->directive_epoch = e->flight.directive_epoch;
        }
      }
      if (t->preparing_return == FMS_RETURN_STABILIZE_HOLD) {
        if (e->flight.failsafe_level >= FAILSAFE_ACTION_REQUIRED || e->authority.latched ||
            e->flight.flight_mode != FLIGHT_MODE_POS_HOLD ||
            t->directive_epoch != e->flight.directive_epoch ||
            e->home.snap.validity != PRODUCT_HOME_VALID || e->home.snap.home_id != t->home_token) {
          t->active = 0u;
          fms_tracker_ack(e, out, t, COMMAND_ACK_CANCELLED, 12u);
          continue;
        }
        if (in->now_us - t->started_us >= FMS_RETURN_PREPARE_TIMEOUT_US) {
          t->active = 0u;
          fms_tracker_ack(e, out, t, COMMAND_ACK_FAILED, 6u);
          continue;
        }
        if (!fms_return_hold_is_stable(e, in)) {
          t->stable_since_us = 0u;
        } else if (t->stable_since_us == 0u) {
          t->stable_since_us = in->now_us;
        } else if (in->now_us - t->stable_since_us >= FMS_RETURN_STABLE_US &&
                   fms_mode_transition(e, FLIGHT_MODE_AUTO_RETURN, in, 1u)) {
          t->preparing_return = 0u;
          fms_permit_revoke(e, 12u);
        }
      } else if (e->flight.flight_mode == FLIGHT_MODE_AUTO_RETURN &&
                 e->home.snap.validity == PRODUCT_HOME_VALID &&
                 e->home.snap.home_id == t->home_token && in->nav_ok) {
        float dx = in->nav->core.position_ned_m[0] - e->home.snap.position_ned_m[0];
        float dy = in->nav->core.position_ned_m[1] - e->home.snap.position_ned_m[1];
        float d2 = dx * dx + dy * dy;
        if (d2 <= FMS_RETURN_ARRIVE_M * FMS_RETURN_ARRIVE_M &&
            e->land.state == LANDED_STATE_LANDED) {
          complete = 1u;
        }
      }
    }

    if (complete != 0u) {
      t->active = 0u;
      fms_tracker_ack(e, out, t, COMMAND_ACK_COMPLETED, reason);
    } else if (in->now_us - t->last_progress_us >= 1000000u) {
      t->last_progress_us = in->now_us;
      fms_tracker_ack(e, out, t, COMMAND_ACK_IN_PROGRESS, 0u);
    }
  }
}

/* ------------------------------------------------------------------ */
/* mode capability gate                                               */
/* ------------------------------------------------------------------ */

uint32_t fms_mode_required_nav(uint8_t mode)
{
  return fms_product_mode_required_nav(mode);
}

uint8_t fms_mode_is_pilot(uint8_t mode)
{
  return (uint8_t)(mode == FLIGHT_MODE_MANUAL_RATE || mode == FLIGHT_MODE_MANUAL_ATTITUDE ||
                   mode == FLIGHT_MODE_ALT_HOLD || mode == FLIGHT_MODE_POS_HOLD);
}

uint8_t fms_mode_is_auto(uint8_t mode)
{
  return (uint8_t)(mode == FLIGHT_MODE_AUTO_MISSION || mode == FLIGHT_MODE_AUTO_RETURN ||
                   mode == FLIGHT_MODE_AUTO_LAND || mode == FLIGHT_MODE_EMERGENCY_LAND);
}

static uint8_t fms_navigation_capable(const FmsEngine *e, const FmsInputs *in,
                                      uint8_t mode, uint8_t entering)
{
  const NavigationCapability *c;
  const NavigationSnapshot *n;
  uint32_t required = fms_mode_required_nav(mode);
  float ratio = entering ? e->config.payload.nav_enter_ratio : 1.0f;
  unsigned i;
  if (mode == FLIGHT_MODE_NONE) return 1u;
  if (mode > FLIGHT_MODE_EMERGENCY_LAND) return 0u;
  if (!in->nav_ok || !in->cap_ok) return 0u;
  c = in->cap; n = in->nav;
  if ((c->solution_state != 1u && c->solution_state != 2u) ||
      n->state_epoch != c->state_epoch || n->origin_id != c->origin_id || !c->origin_id ||
      (c->capability_mask & required) != required || (n->core.valid_mask & required) != required) return 0u;
  if (required & NAV_VALID_ATTITUDE) {
    float norm = 0;
    for (i = 0; i < 4; ++i) norm += n->core.q_nb_wxyz[i]*n->core.q_nb_wxyz[i];
    if (!fms_finite(norm) || fabsf(norm-1.0f) > 0.01f) return 0u;
    for (i = 0; i < 3; ++i)
      if (!fms_finite(n->core.attitude_std_rad[i]) || n->core.attitude_std_rad[i] < 0 ||
          n->core.attitude_std_rad[i] > e->config.payload.attitude_std_limit_rad*ratio) return 0u;
  }
  if ((required & NAV_VALID_HEADING) && (!fms_finite(c->heading_std_rad) ||
      c->heading_std_rad < 0 || c->heading_std_rad > e->config.payload.heading_std_limit_rad*ratio)) return 0u;
  if ((required & NAV_VALID_HORIZONTAL_POSITION) && (!fms_finite(c->horizontal_position_std_m) ||
      c->horizontal_position_std_m < 0 || c->horizontal_position_std_m > e->config.payload.horizontal_position_std_limit_m*ratio)) return 0u;
  if ((required & NAV_VALID_VERTICAL_POSITION) && (!fms_finite(c->vertical_position_std_m) ||
      c->vertical_position_std_m < 0 || c->vertical_position_std_m > e->config.payload.vertical_position_std_limit_m*ratio)) return 0u;
  for (i = 0; i < 3; ++i) {
    uint32_t pos = i < 2 ? NAV_VALID_HORIZONTAL_POSITION : NAV_VALID_VERTICAL_POSITION;
    uint32_t vel = i < 2 ? NAV_VALID_HORIZONTAL_VELOCITY : NAV_VALID_VERTICAL_VELOCITY;
    float limit = i < 2 ? e->config.payload.horizontal_velocity_std_limit_mps :
                         e->config.payload.vertical_velocity_std_limit_mps;
    if ((required & pos) && !fms_finite(n->core.position_ned_m[i])) return 0u;
    if ((required & vel) && (!fms_finite(n->core.velocity_ned_mps[i]) ||
        !fms_finite(c->velocity_std_mps) || c->velocity_std_mps < 0 ||
        c->velocity_std_mps > limit*ratio || !fms_finite(n->core.velocity_std_mps[i]) ||
        n->core.velocity_std_mps[i] < 0 || n->core.velocity_std_mps[i] > limit*ratio)) return 0u;
    if ((required & NAV_VALID_BODY_RATE) && !fms_finite(n->core.body_rate_frd_radps[i])) return 0u;
  }
  return 1u;
}

void fms_mode_capability_tick(FmsEngine *e, const FmsInputs *in)
{
  uint8_t mode;
  for (mode = 1; mode <= FLIGHT_MODE_EMERGENCY_LAND; ++mode) {
    uint32_t bit = 1u << mode;
    if (!fms_navigation_capable(e, in, mode, 1u)) {
      e->flight.nav_candidate_mask &= ~bit; e->flight.nav_ready_mask &= ~bit;
    } else {
      if (!(e->flight.nav_candidate_mask & bit)) {
        e->flight.nav_candidate_mask |= bit; e->flight.nav_ready_since_us[mode] = in->now_us;
      }
      if (in->now_us-e->flight.nav_ready_since_us[mode] >= e->config.payload.nav_enter_dwell_us)
        e->flight.nav_ready_mask |= bit;
    }
  }
}

uint8_t fms_capable_for_mode(const FmsEngine *e, const FmsInputs *in, uint8_t mode, uint8_t entering)
{
  if (!fms_navigation_capable(e, in, mode, entering)) return 0u;
  if (mode != FLIGHT_MODE_NONE && entering && !(e->flight.nav_ready_mask & (1u << mode))) return 0u;
  if (mode == FLIGHT_MODE_AUTO_RETURN && (e->home.snap.validity != PRODUCT_HOME_VALID ||
      e->home.snap.origin_id != in->nav->origin_id || e->home.snap.state_epoch != in->nav->state_epoch)) return 0u;
  return 1u;
}

/* ------------------------------------------------------------------ */
/* publication                                                        */
/* ------------------------------------------------------------------ */

static uint16_t fms_directive_operation(uint8_t mode, uint8_t failsafe)
{
  if (failsafe == FAILSAFE_TERMINAL) {
    return PRODUCT_OP_SAFE_ACTION;
  }
  if (mode == FLIGHT_MODE_AUTO_MISSION) {
    return PRODUCT_OP_MISSION;
  }
  if (mode == FLIGHT_MODE_AUTO_RETURN) {
    return PRODUCT_OP_RETURN;
  }
  if (mode == FLIGHT_MODE_AUTO_LAND || mode == FLIGHT_MODE_EMERGENCY_LAND) {
    return PRODUCT_OP_LAND;
  }
  if (fms_mode_is_pilot(mode)) {
    return PRODUCT_OP_PILOT;
  }
  if (mode == FLIGHT_MODE_NONE) {
    return PRODUCT_OP_NONE;
  }
  return PRODUCT_OP_HOLD;
}

static uint16_t fms_guidance_mode(uint8_t mode)
{
  switch (mode) {
  case FLIGHT_MODE_MANUAL_RATE: return PRODUCT_GUIDANCE_MODE_RATE;
  case FLIGHT_MODE_MANUAL_ATTITUDE: return PRODUCT_GUIDANCE_MODE_ATTITUDE;
  case FLIGHT_MODE_ALT_HOLD: return PRODUCT_GUIDANCE_MODE_ALT_HOLD;
  case FLIGHT_MODE_POS_HOLD: return PRODUCT_GUIDANCE_MODE_POS_HOLD;
  default: return PRODUCT_GUIDANCE_MODE_TRAJECTORY;
  }
}

static void fms_publish(FmsEngine *e, const FmsInputs *in, FmsOutputView *out)
{
  uint64_t now = in->now_us;
  MissionStatus *ms = &out->mission_status;
  VehicleDirective *dir = &out->directive;
  AuthorityStatus *as = &out->authority_status;
  FmsStateSnapshot *st = &out->state_snapshot;
  uint16_t output_policy = (uint16_t)e->config.payload.output_policy_id;

  fms_fill_header(e, &dir->header, MSG_VEHICLE_DIRECTIVE, FRAME_LOCAL_NED, now,
                 e->policy->directive_ttl_us);
  dir->directive_epoch = e->flight.directive_epoch;
  dir->authority_epoch = e->authority.epoch;
  dir->state_epoch = e->flight.state_epoch;
  dir->policy_id = output_policy;
  dir->operation_code = fms_directive_operation(e->flight.flight_mode, e->flight.failsafe_level);
  if (e->flight.flight_mode == FLIGHT_MODE_POS_HOLD && e->authority.owner != AUTHORITY_PILOT &&
      e->flight.failsafe_level != FAILSAFE_TERMINAL) dir->operation_code = PRODUCT_OP_HOLD;
  dir->guidance_mode_code = fms_guidance_mode(e->flight.flight_mode);
  if ((e->flight.flight_mode == FLIGHT_MODE_AUTO_LAND || e->flight.flight_mode == FLIGHT_MODE_EMERGENCY_LAND) &&
      (!fms_capable_for_mode(e, in, FLIGHT_MODE_POS_HOLD, 0u) || !e->home.valid))
    dir->guidance_mode_code = PRODUCT_GUIDANCE_MODE_ALT_HOLD;
  dir->reason_code = 0u;
  dir->directive_expires_at_us = fms_add_us(now, e->policy->directive_ttl_us);
  dir->required_nav_capabilities = fms_mode_required_nav(e->flight.flight_mode);
  dir->safety_envelope_id = 1u;
  dir->payload.flight_phase = (uint8_t)(e->flight.failsafe_level == FAILSAFE_TERMINAL
                                            ? FLIGHT_PHASE_EMERGENCY
                                            : (e->flight.flight_mode == FLIGHT_MODE_AUTO_LAND ||
                                                       e->flight.flight_mode == FLIGHT_MODE_EMERGENCY_LAND
                                                   ? FLIGHT_PHASE_LANDING
                                                   : (e->land.state == LANDED_STATE_LANDED
                                                          ? FLIGHT_PHASE_GROUND
                                                          : FLIGHT_PHASE_CRUISE)));
  dir->payload.failsafe_policy = (uint8_t)output_policy;
  dir->payload.max_tilt_rad = e->policy->default_max_tilt_rad;
  dir->payload.min_height_above_home_m = e->policy->default_min_height_above_home_m;
  dir->payload.max_height_above_home_m = e->policy->default_max_height_above_home_m;
  dir->payload.home = e->home.snap;
  {
    uint8_t i;
    for (i = 0; i < FMS_MAX_TRACKERS; ++i) {
      const FmsTracker *t = &e->trackers[i];
      if (t->active && t->kind == FMS_TRACKER_TAKEOFF && t->directive_epoch == e->flight.directive_epoch) {
        dir->operation_code = PRODUCT_OP_HOLD; dir->guidance_mode_code = PRODUCT_GUIDANCE_MODE_TRAJECTORY;
        dir->payload.flight_phase = FLIGHT_PHASE_TAKEOFF;
        dir->payload.target_valid_mask = PRODUCT_DIRECTIVE_TARGET_POSITION | PRODUCT_DIRECTIVE_TARGET_YAW;
        memcpy(dir->payload.target_position_ned_m, t->target_position_ned_m, sizeof(t->target_position_ned_m));
        dir->payload.target_yaw_rad = t->target_yaw_rad;
        break;
      }
    }
  }

  if (fms_product_build_safety_envelope(e->policy, &e->config, 1u,
                                       &out->safety_envelope) == STATUS_OK) {
    /* The product hook owns the payload; identity and TTL come from M4. */
    fms_fill_header(e, &out->safety_envelope.header, MSG_SAFETY_ENVELOPE,
                   FRAME_LOCAL_NED, now, e->policy->directive_ttl_us);
  } else {
    memset(&out->safety_envelope, 0, sizeof(out->safety_envelope));
  }

  fms_permit_fill(e, &out->actuation_permit);

  if (e->authority.owner == AUTHORITY_PILOT && in->op_ok && in->rc_fresh) {
    AuthorizedOperatorInput *ao = &out->authorized_operator_input;
    fms_fill_header(e, &ao->header, MSG_AUTHORIZED_OPERATOR_INPUT, FRAME_LOCAL_NED,
                   now, e->policy->status_ttl_us);
    ao->authority_epoch = e->authority.epoch;
    ao->selected_source_id = e->authority.source_id;
    ao->authority_class = AUTHORITY_PILOT;
    ao->input_layout_id = PRODUCT_OPERATOR_INPUT_LAYOUT_ID;
    ao->payload = in->op->payload;
  }

  fms_fill_header(e, &as->header, MSG_AUTHORITY_STATUS, FRAME_LOCAL_NED, now,
                 e->policy->status_ttl_us);
  as->authority_epoch = e->authority.epoch;
  as->authority_owner = e->authority.owner;
  as->operation_mode = e->flight.flight_mode;
  as->selected_source_id = e->authority.source_id;
  as->flags = (uint32_t)((e->authority.latched != 0u ? FMS_AUTH_FLAG_TAKEOVER_LATCHED : 0u) |
                         (e->flight.failsafe_level >= FAILSAFE_ACTION_REQUIRED
                              ? FMS_AUTH_FLAG_FAILSAFE_ACTIVE
                              : 0u));
  as->reason_code = 0u;

  fms_fill_header(e, &st->header, MSG_FMS_STATE_SNAPSHOT, FRAME_LOCAL_NED, now,
                 e->policy->status_ttl_us);
  st->arm_state = e->flight.arm_state;
  st->flight_mode = e->flight.flight_mode;
  st->authority = e->authority.owner;
  st->failsafe_level = e->flight.failsafe_level;
  st->mission_state = e->mission.state;
  st->authority_epoch = e->authority.epoch;
  st->directive_epoch = e->flight.directive_epoch;
  st->permit_epoch = e->permit.epoch;
  st->reference_epoch = e->permit.reference_epoch;
  st->state_epoch = e->flight.state_epoch;
  st->active_mission_id = (e->mission.state == MISSION_STATE_IDLE) ? 0u : e->mission.mission_id;
  st->active_policy_id = e->policy->fms_policy_id;

  fms_fill_header(e, &ms->header, MSG_MISSION_STATUS, FRAME_LOCAL_NED, now,
                 e->policy->status_ttl_us);
  ms->mission_id = (e->mission.state == MISSION_STATE_IDLE) ? 0u : e->mission.mission_id;
  ms->current_item_id = (e->mission.state == MISSION_STATE_RUNNING ||
                         e->mission.state == MISSION_STATE_PAUSED)
                            ? e->mission.active.waypoints[e->mission.current_index].waypoint_id
                            : 0u;
  ms->mission_epoch = e->mission.mission_epoch;
  ms->mission_revision = e->mission.revision;
  ms->execution_token = (e->mission.state == MISSION_STATE_RUNNING) ? e->mission.token : 0u;
  ms->mission_state = e->mission.state;
  ms->result_code = e->mission.result_code;
  ms->progress_0_to_1 = e->mission.progress;

  fms_fill_header(e, &out->prearm_result.header, MSG_PRE_ARM_CHECK_RESULT,
                 FRAME_LOCAL_NED, now, e->policy->status_ttl_us);
  out->prearm_result.check_epoch = e->prearm.check_epoch;
  out->prearm_result.failed_checks_mask = e->prearm.failed_mask;
  out->prearm_result.state = e->prearm.prearm_state;
  out->prearm_result.primary_reason_code = e->prearm.primary_reason;
  out->prearm_result.failed_count = 0u;
  out->prearm_result.warning_count = 0u;
  {
    uint8_t i;
    for (i = 0u; i < PREARM_ID_COUNT; ++i) {
      if (e->prearm.item_state[i] == PREARM_ITEM_FAIL) {
        out->prearm_result.failed_count++;
      } else if (e->prearm.item_state[i] == PREARM_ITEM_WARNING) {
        out->prearm_result.warning_count++;
      }
    }
  }
  out->prearm_result.checked_at_us = e->prearm.checked_at_us;

  fms_fill_header(e, &out->landed_state.header, MSG_LANDED_STATE_SNAPSHOT,
                 FRAME_LOCAL_NED, now, e->policy->landed_ttl_us);
  out->landed_state.state_revision = e->land.revision;
  out->landed_state.state = e->land.state;
  out->landed_state.evidence_mask = e->land.evidence_mask;
  out->landed_state.confidence_0_to_1 = e->land.confidence;
  out->landed_state.stable_duration_ms = e->land.stable_ms;
  out->landed_state.state_since_us = e->land.since_us;

  if (e->mission.state == MISSION_STATE_RUNNING) {
    AuthorizedMissionSnapshot *am = &out->authorized_mission;
    fms_fill_header(e, &am->header, MSG_AUTHORIZED_MISSION_SNAPSHOT, FRAME_LOCAL_NED,
                   now, e->policy->mission_ttl_us);
    am->mission_epoch = e->mission.mission_epoch;
    am->authority_epoch = e->authority.epoch;
    am->directive_epoch = e->flight.directive_epoch;
    am->state_epoch = e->flight.state_epoch;
    am->origin_id = e->mission.origin_id;
    am->mission_id = e->mission.mission_id;
    am->mission_revision = e->mission.revision;
    am->mission_hash = e->mission.hash;
    am->mission_expires_at_us = e->mission.expires_at_us;
    am->current_item_id = e->mission.active.waypoints[e->mission.current_index].waypoint_id;
    am->current_item_index = e->mission.current_index;
    am->execution_token = e->mission.token;
    am->mission = e->mission.active;
  }

  out->geofence_count = (uint16_t)e->geofence.fence_count;
  {
    uint16_t i;
    for (i = 0u; i < out->geofence_count && i < FMS_MAX_GEOFENCES; ++i) {
      out->geofences[i] = e->geofence.fences[i];
      fms_fill_header(e, &out->geofences[i].header, MSG_PRODUCT_GEOFENCE_SNAPSHOT,
                     FRAME_LOCAL_NED, now, e->policy->status_ttl_us);
    }
  }

  fms_fill_header(e, &out->decision_event.header, MSG_DECISION_EVENT, FRAME_LOCAL_NED,
                 now, e->policy->status_ttl_us);
  out->decision_event.decision_id = e->decision_id;
  out->decision_event.request_id = e->last_decision_request;
  out->decision_event.policy_id = e->policy->fms_policy_id;
  out->decision_event.evidence_mask = 0u;
  out->decision_event.decision_code = e->last_decision_code;
  out->decision_event.reason_code = e->last_decision_reason;
}

/* ------------------------------------------------------------------ */
/* engine lifecycle                                                   */
/* ------------------------------------------------------------------ */

Status fms_engine_init(FmsEngine *e, const ProductFmsConfig *config)
{
  return fms_engine_init_configured(e, config, 0u);
}

Status fms_engine_init_configured(FmsEngine *e, const ProductFmsConfig *config,
                                  uint8_t preserve_geofence)
{
  if (e == NULL || config == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  if (config->config_version == 0u || config->parameter_hash == 0u) {
    return STATUS_INVALID_ARGUMENT;
  }
  if (preserve_geofence) {
    size_t end = offsetof(FmsEngine, geofence) + sizeof(e->geofence);
    memset(e, 0, offsetof(FmsEngine, geofence));
    memset((uint8_t *)e + end, 0, sizeof(*e) - end);
  } else {
    memset(e, 0, sizeof(*e));
  }
  e->policy = &fms_quad_x4_policy;
  e->config = *config;
  fms_mission_init(e);
  fms_authority_init(e);
  fms_permit_init(e);
  fms_land_init(e);
  if (!preserve_geofence) fms_geofence_init(e);
  e->flight.arm_state = ARM_STATE_DISARMED;
  e->flight.flight_mode = FLIGHT_MODE_NONE;
  e->flight.failsafe_level = FAILSAFE_NORMAL;
  e->flight.authority_epoch = e->authority.epoch;
  e->flight.directive_epoch = 1u;
  e->flight.active_policy_id = e->policy->fms_policy_id;
  e->prearm.prearm_state = PREARM_NOT_RUN;
  e->health.component_id = e->policy->fms_policy_id;
  e->health.health_state = 2u; /* HEALTHY */
  e->initialized = 1u;
  return STATUS_OK;
}

void fms_engine_stop(FmsEngine *e)
{
  if (e == NULL) {
    return;
  }
  memset(e, 0, sizeof(*e));
}

Status fms_engine_set_geofence(FmsEngine *e, const ProductGeofenceSnapshot *fence)
{
  if (e == NULL || fence == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  if (e->initialized == 0u) {
    return STATUS_NOT_READY;
  }
  if (e->flight.arm_state != ARM_STATE_DISARMED) {
    return STATUS_NOT_PERMITTED; /* CR-V191-C02: configuration is init-time only */
  }
  return fms_geofence_configure(e, fence);
}

Status fms_engine_update(FmsEngine *e, const FmsInputView *iv, FmsOutputView *out)
{
  FmsInputs in;

  if (e == NULL || iv == NULL || out == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  fms_zero_output(out);
  if (e->initialized == 0u) {
    return STATUS_NOT_READY;
  }
  if (iv->evaluation_time_us == 0u) {
    e->health.last_error_code = (uint32_t)STATUS_INVALID_ARGUMENT;
    e->health.consecutive_error_count++;
    return STATUS_INVALID_ARGUMENT;
  }
  if (iv->pending_request_count > FMS_MAX_ACKS ||
      iv->link_status_count > FMS_MAX_LINKS) {
    e->health.last_error_code = (uint32_t)STATUS_BUFFER_FULL;
    e->health.consecutive_error_count++;
    return STATUS_BUFFER_FULL;
  }
  if ((iv->pending_request_count > 0u && iv->pending_requests == NULL) ||
      (iv->link_status_count > 0u && iv->link_statuses == NULL)) {
    e->health.last_error_code = (uint32_t)STATUS_INVALID_ARGUMENT;
    e->health.consecutive_error_count++;
    return STATUS_INVALID_ARGUMENT;
  }
  if (iv->evaluation_time_us < e->clock.now_us &&
      (fms_probe_header(iv) == NULL || (fms_probe_header(iv)->boot_id == e->clock.boot_id &&
       fms_probe_header(iv)->time_epoch == e->clock.time_epoch))) {
    e->health.last_error_code = (uint32_t)STATUS_STALE_DATA;
    e->health.consecutive_error_count++;
    return STATUS_STALE_DATA;
  }

  e->clock.now_us = iv->evaluation_time_us;
  if (e->clock.bound == 0u) {
    fms_bind_clock(e, iv);
    if (e->clock.bound == 0u) {
      e->health.last_error_code = (uint32_t)STATUS_NOT_READY;
      e->health.consecutive_error_count++;
      return STATUS_NOT_READY;
    }
  } else {
    const ContractHeader *probe = fms_probe_header(iv);
    if (probe != NULL && probe->boot_id != 0u && probe->boot_id != e->clock.boot_id) {
      if (e->flight.arm_state != ARM_STATE_DISARMED) {
        fms_permit_revoke(e, 14u);
        e->health.last_error_code = (uint32_t)STATUS_STALE_DATA;
        e->health.consecutive_error_count++;
        return STATUS_STALE_DATA; /* cross-boot data never resumes a live flight */
      }
      {
        ProductFmsConfig cfg = e->config;
        (void)fms_engine_init_configured(e, &cfg, 1u);
        e->clock.now_us = iv->evaluation_time_us;
        fms_bind_clock(e, iv);
      }
    } else if (probe != NULL && probe->time_epoch != e->clock.time_epoch) {
      if (e->flight.arm_state != ARM_STATE_DISARMED) {
        fms_permit_revoke(e, 14u);
        e->health.last_error_code = (uint32_t)STATUS_STALE_DATA;
        e->health.consecutive_error_count++;
        return STATUS_STALE_DATA;
      }
      /* Ground clock discontinuity: nothing old may survive the mapping change. */
      e->clock.time_epoch = probe->time_epoch;
      e->motion.sample_us = 0u;
      e->flight.nav_candidate_mask = e->flight.nav_ready_mask = 0;
      e->prearm.valid = 0;
      fms_mission_on_failsafe(e);
      e->home.valid = 0u;
      e->home.snap.validity = PRODUCT_HOME_INVALID;
      fms_permit_revoke(e, 14u);
      fms_record_decision(e, DECISION_FORCE_SAFE_ACTION, 14u, 0u);
    }
  }

  fms_collect_inputs(e, iv, &in);
  fms_classify_links(e, &in);

  if (in.nav_ok != 0u) {
    uint16_t i;
    for (i = 0; i < e->geofence.fence_count; ++i) {
      ProductGeofenceSnapshot *f = &e->geofence.fences[i];
      if (f->origin_id == 0u && f->state_epoch == 0u) {
        f->origin_id = in.nav->origin_id; f->state_epoch = in.nav->state_epoch;
      } else if (f->origin_id != in.nav->origin_id || f->state_epoch != in.nav->state_epoch) {
        f->validity = PRODUCT_GEOFENCE_INVALID;
      }
    }
    if (e->flight.state_epoch != in.nav->state_epoch || e->flight.origin_id != in.nav->origin_id) {
      e->motion.sample_us = 0u;
      e->flight.nav_candidate_mask = e->flight.nav_ready_mask = 0;
      fms_mission_on_failsafe(e); fms_permit_revoke(e, 14u);
    }
    e->flight.state_epoch = in.nav->state_epoch;
    e->flight.origin_id = in.nav->origin_id;
  }

  fms_motion_tick(e, &in);
  fms_land_tick(e, &in);
  fms_home_tick(e, &in);
  fms_mode_capability_tick(e, &in);
  if (e->flight.arm_state == ARM_STATE_DISARMED) {
    e->flight.critical_energy_latched = 0u;
  } else if (in.energy_state == ENERGY_CRITICAL) {
    e->flight.critical_energy_latched = 1u;
  }
  fms_authority_update(e, &in);
  fms_process_requests(e, &in, iv, out);
  fms_failsafe_update(e, &in);
  fms_mission_tick(e, &in);
  fms_permit_tick(e, &in);
  fms_tracker_tick(e, &in, out);
  /* PreArm is evaluated after this cycle's state changes and published with a
   * new check_epoch; an ARM request must carry the epoch of the last published
   * result, and a result older than one second is rejected. */
  fms_prearm_evaluate(e, &in);
  fms_publish(e, &in, out);

  e->health.last_success_time_us = in.now_us;
  e->health.last_heartbeat_time_us = in.now_us;
  e->health.health_state = (e->flight.failsafe_level >= FAILSAFE_ACTION_REQUIRED) ? 3u : 2u;
  return STATUS_OK;
}
