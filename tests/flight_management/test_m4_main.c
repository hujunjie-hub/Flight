/*
 * Host tests for T191-C-FMS (M4/FMS).
 *
 * The tests drive the real FmsPolicyPort implementation with contract-typed
 * inputs. They are Host fixtures only: no target timing, WCET, DMA or flight
 * claim is made here (see the delivery report for NOT_RUN items).
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ports/port_api.h"
#include "fms_interface.h"
#include "flight_state_machine/fms_internal.h"
#ifdef M4_M5_INTEGRATION
#include "guidance_update/guidance.h"
#endif

_Static_assert(sizeof(FmsDedupEntry) <= 72u, "FMS dedup entry exceeds its static allocation");

#define T_BOOT 0x1122334455667788ull
#define T_EPOCH 3u
#define T_ORIGIN 42u
#define T_STATE_EPOCH 7u
#define T_SESSION 77u
#define T_MISSION_ID 100u
#define T_TRANSACTION 55u
#define T_HASH 0x00ABCDEFu
#define STEP_US 10000u

static int g_failures;
static int g_checks;

#define CHECK(cond)                                                            \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                   \
      g_failures++;                                                            \
    }                                                                          \
  } while (0)

typedef struct {
  uint64_t now;
  NavigationSnapshot nav;
  NavigationCapability cap;
  OperatorInput op;
  CommunicationLinkStatus rc;
  CommunicationLinkStatus gcs;
  ControlCapability control;
  ActuatorCapability act_cap;
  ActuatorStatus act_status;
  FaultSummary faults;
  ProductFmsInputExtension ext;
  MissionUploadCandidate cand;
  GuidanceStatus guidance;
  GuidanceFeasibility feas;
  uint64_t guidance_ttl_us;
  uint8_t guidance_valid;
  uint8_t omit_control;
  uint8_t omit_nav;
  uint32_t control_sample_lag_us;
  uint8_t has_candidate;
  OperationRequest req;
  uint8_t has_req;
  uint32_t next_request_id;
  int32_t epoch_override; /* -1 = current, -2 = omit, >=0 = forced */
  FmsOutputView out;
} Fx;

static uint32_t g_seq;

static void fill_header(ContractHeader *h, uint16_t type, uint16_t source, uint64_t now,
                        uint64_t ttl, uint64_t sample)
{
  memset(h, 0, sizeof(*h));
  h->schema_major = CONTRACT_SCHEMA_MAJOR;
  h->schema_minor = CONTRACT_SCHEMA_MINOR;
  h->message_type = type;
  h->source_id = source;
  h->sequence = ++g_seq;
  h->profile_id = PRODUCT_PROFILE_ID;
  h->frame_id = FRAME_LOCAL_NED;
  h->boot_id = T_BOOT;
  h->clock_domain_id = CLOCK_DOMAIN_FCU_MONOTONIC;
  h->time_epoch = T_EPOCH;
  h->timestamp_sample_us = sample;
  h->timestamp_publish_us = now;
  h->valid_until_us = now + ttl;
  h->timestamp_flags = TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED;
  h->validity_flags = VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID;
}

static void fx_defaults(Fx *fx)
{
  memset(fx, 0, sizeof(*fx));
  fx->now = 1000000u;
  fx->next_request_id = 1000u;
  fx->epoch_override = -1;

  fx->nav.state_epoch = T_STATE_EPOCH;
  fx->nav.origin_id = T_ORIGIN;
  fx->nav.core.valid_mask = 0xFFu;
  fx->nav.core.q_nb_wxyz[0] = 1.0f;
  fx->nav.core.position_std_m[0] = 0.2f;
  fx->nav.core.position_std_m[1] = 0.2f;
  fx->nav.core.position_std_m[2] = 0.3f;
  fx->nav.core.velocity_std_mps[0] = 0.05f;
  fx->nav.core.velocity_std_mps[1] = 0.05f;
  fx->nav.core.velocity_std_mps[2] = 0.05f;
  fx->nav.core.attitude_std_rad[0] = 0.01f;
  fx->nav.core.attitude_std_rad[1] = 0.01f;
  fx->nav.core.attitude_std_rad[2] = 0.02f;

  fx->cap.state_epoch = T_STATE_EPOCH;
  fx->cap.origin_id = T_ORIGIN;
  fx->cap.capability_mask = 0xFFu;
  fx->cap.solution_state = 1u;
  fx->cap.horizontal_position_std_m = 0.3f;
  fx->cap.vertical_position_std_m = 0.4f;
  fx->cap.velocity_std_mps = 0.1f;
  fx->cap.heading_std_rad = 0.05f;

  fx->op.input_layout_id = PRODUCT_OPERATOR_INPUT_LAYOUT_ID;
  fx->op.valid_axis_mask = 0x0Fu;
  fx->op.payload.throttle_norm = 0.0f;

  fx->rc.link_kind = LINK_KIND_RC;
  fx->rc.link_instance_id = LINK_KIND_RC;
  fx->rc.link_state = LINK_STATE_HEALTHY;
  fx->rc.flags = 0u;
  fx->gcs.link_kind = LINK_KIND_GCS;
  fx->gcs.link_instance_id = LINK_KIND_GCS;
  fx->gcs.link_state = LINK_STATE_HEALTHY;

  fx->control.control_layout_id = 1u;
  fx->control.controllable_dof_mask = 0x0Fu;
  fx->control.health_state = 2u;

  fx->act_cap.actuator_layout_id = PRODUCT_ACTUATOR_LAYOUT_ID;
  fx->act_cap.available_mask = 0x0Fu;
  fx->act_cap.controllable_dof_mask = 0x0Fu;
  fx->act_status.actuator_layout_id = PRODUCT_ACTUATOR_LAYOUT_ID;
  fx->act_status.available_mask = 0x0Fu;
  fx->act_status.fault_mask = 0u;

  fx->faults.highest_severity = 0u;

  fx->ext.valid_mask = 0xFFu;
  fx->ext.ground_contact_state = GROUND_CONTACT_CONTACT;
  fx->ext.energy_state = ENERGY_NORMAL;
  fx->ext.battery_remaining_fraction = 1.0f;
  fx->ext.landing_zone_valid = 1u;
  fx->ext.geofence_state = 1u;
}

static void fx_fill_headers(Fx *fx)
{
  uint64_t now = fx->now;
  fill_header(&fx->nav.header, MSG_NAVIGATION_SNAPSHOT, 20u, now, 50000u, now);
  fill_header(&fx->cap.header, MSG_NAVIGATION_CAPABILITY, 20u, now, 50000u, now);
  fill_header(&fx->op.header, MSG_OPERATOR_INPUT, 1u, now, 100000u, now);
  fill_header(&fx->rc.header, MSG_COMMUNICATION_LINK_STATUS, 1u, now, 200000u, now);
  fill_header(&fx->gcs.header, MSG_COMMUNICATION_LINK_STATUS, 2u, now, 200000u, now);
  fill_header(&fx->control.header, MSG_CONTROL_CAPABILITY, 23u, now, 100000u, now);
  if (fx->control_sample_lag_us != 0u && now >= fx->control_sample_lag_us) {
    fx->control.header.timestamp_sample_us = now - fx->control_sample_lag_us;
  }
  fill_header(&fx->act_cap.header, MSG_ACTUATOR_CAPABILITY, 24u, now, 100000u, now);
  fill_header(&fx->act_status.header, MSG_ACTUATOR_STATUS, 24u, now, 50000u, now);
  fill_header(&fx->faults.header, MSG_FAULT_SUMMARY, 25u, now, 200000u, now);
  fill_header(&fx->ext.header, MSG_PRODUCT_FMS_INPUT_EXTENSION, 25u, now, 100000u, now);
  fx->rc.last_valid_frame_us = now;
  fx->rc.last_rx_us = now;
  fx->gcs.last_valid_frame_us = now;
  fx->gcs.last_rx_us = now;

  if (fx->has_candidate != 0u) {
    fill_header(&fx->cand.header, MSG_MISSION_UPLOAD_CANDIDATE, 2u, now, 1000000u, now);
  }
  if (fx->guidance_valid != 0u) {
    fill_header(&fx->guidance.header, MSG_GUIDANCE_STATUS, 22u, now, 100000u, now);
    fill_header(&fx->feas.header, MSG_GUIDANCE_FEASIBILITY, 22u, now, 100000u, now);
    fx->guidance.reference_expires_at_us = now + fx->guidance_ttl_us;
    fx->feas.reference_id = fx->guidance.reference_id;
    fx->feas.reference_epoch = fx->guidance.reference_epoch;
    fx->feas.feasible = (uint8_t)(fx->guidance.guidance_state != GUIDANCE_STATE_REJECTED);
    fx->feas.reason_code = fx->feas.feasible ? 0u : 2u;
  }
  if (fx->has_req != 0u) {
    fill_header(&fx->req.header, MSG_OPERATION_REQUEST, 2u, now, 200000u, now);
    fx->req.execution_deadline_us = now + 1000000u;
  }
}

static void fx_view(Fx *fx, FmsInputView *iv)
{
  memset(iv, 0, sizeof(*iv));
  iv->pending_requests = fx->has_req ? &fx->req : NULL;
  iv->pending_request_count = fx->has_req ? 1u : 0u;
  iv->operator_input = &fx->op;
  iv->link_statuses = &fx->rc;
  iv->link_status_count = 2u;
  iv->navigation = (fx->omit_nav != 0u) ? NULL : &fx->nav;
  iv->navigation_capability = &fx->cap;
  iv->mission_candidate = fx->has_candidate ? &fx->cand : NULL;
  iv->guidance_feasibility = fx->guidance_valid ? &fx->feas : NULL;
  iv->guidance_status = fx->guidance_valid ? &fx->guidance : NULL;
  iv->control_capability = &fx->control;
  iv->actuator_capability = &fx->act_cap;
  iv->actuator_status = &fx->act_status;
  iv->fault_summary = &fx->faults;
  iv->product_extension = fx->ext;
  iv->presence_mask = FMS_INPUT_HAS_OPERATOR | FMS_INPUT_HAS_LINKS |
                      ((fx->omit_nav != 0u) ? 0u : FMS_INPUT_HAS_NAV) |
                      FMS_INPUT_HAS_NAV_CAPABILITY |
                      FMS_INPUT_HAS_ACTUATOR | FMS_INPUT_HAS_FAULT_SUMMARY |
                      FMS_INPUT_HAS_PRODUCT_EXTENSION;
  if (fx->omit_control == 0u) {
    iv->presence_mask |= FMS_INPUT_HAS_CONTROL;
  }
  if (fx->has_req != 0u) {
    iv->presence_mask |= FMS_INPUT_HAS_REQUEST;
  }
  if (fx->has_candidate != 0u) {
    iv->presence_mask |= FMS_INPUT_HAS_MISSION_CANDIDATE;
  }
  if (fx->guidance_valid != 0u) {
    iv->presence_mask |= FMS_INPUT_HAS_GUIDANCE | FMS_INPUT_HAS_GUIDANCE_STATUS;
  }
  iv->evaluation_time_us = fx->now;
}

static Status fx_step(Fx *fx)
{
  FmsInputView iv;
  Status st;
  fx_fill_headers(fx);
  fx_view(fx, &iv);
  st = fms_policy_update(&iv, &fx->out);
  fx->now += STEP_US;
  fx->has_req = 0u;
  return st;
}

static void fx_run(Fx *fx, uint32_t duration_us)
{
  uint32_t elapsed = 0u;
  while (elapsed < duration_us) {
    (void)fx_step(fx);
    elapsed += STEP_US;
  }
}

static const CommandAck *fx_find_ack(const Fx *fx, uint32_t request_id)
{
  uint16_t i;
  for (i = 0u; i < fx->out.command_result_count; ++i) {
    if (fx->out.command_results[i].request_id == request_id) {
      return &fx->out.command_results[i];
    }
  }
  return NULL;
}

static uint16_t fx_send(Fx *fx, uint16_t command, uint32_t request_id, uint16_t valid_mask,
                        uint32_t mission_id, uint32_t transaction, uint32_t mode,
                        uint32_t prearm_epoch, float altitude)
{
  const CommandAck *ack;

  memset(&fx->req, 0, sizeof(fx->req));
  fx->req.request_id = request_id;
  fx->req.source_session_id = T_SESSION;
  fx->req.command_code = command;
  fx->req.payload_schema_id = 1u;
  fx->req.payload.mission_id = mission_id;
  fx->req.payload.mission_transaction_id = transaction;
  fx->req.payload.requested_mode = (uint16_t)mode;
  fx->req.payload.prearm_check_epoch = prearm_epoch;
  fx->req.payload.requested_altitude_m = altitude;
  fx->req.payload.valid_mask = valid_mask;
  if (command == PRODUCT_CMD_MISSION_START || command == PRODUCT_CMD_MISSION_PAUSE ||
      command == PRODUCT_CMD_MISSION_RESUME || command == PRODUCT_CMD_MISSION_ABORT ||
      command == PRODUCT_CMD_MISSION_CLEAR) {
    if (fx->epoch_override != -2) {
      fx->req.payload.valid_mask |= PRODUCT_OPERATION_VALID_MISSION_EPOCH;
      fx->req.payload.mission_epoch =
          (fx->epoch_override >= 0) ? (uint32_t)fx->epoch_override
                                    : fx->out.mission_status.mission_epoch;
    }
  }
  fx->epoch_override = -1;
  fx->has_req = 1u;
  (void)fx_step(fx);
  ack = fx_find_ack(fx, request_id);
  if (ack == NULL) {
    printf("FAIL no ack for request %u cmd %u\n", request_id, command);
    g_failures++;
    return 0xFFFFu;
  }
  return ack->ack_state;
}

static uint16_t fx_send_simple(Fx *fx, uint16_t command)
{
  uint32_t id = fx->next_request_id++;
  return fx_send(fx, command, id, 0u, 0u, 0u, 0u, 0u, 0.0f);
}

static void fx_arm(Fx *fx)
{
  uint32_t id;
  uint16_t state;

  fx_run(fx, 4300000u); /* LANDED declaration dwell + stable dwell */
  CHECK(fx->out.landed_state.state == LANDED_STATE_LANDED);
  CHECK(fx->out.prearm_result.state == PREARM_READY);
  id = fx->next_request_id++;
  state = fx_send(fx, PRODUCT_CMD_ARM, id, PRODUCT_OPERATION_VALID_PREARM_EPOCH, 0u, 0u, 0u,
                  fx->out.prearm_result.check_epoch, 0.0f);
  CHECK(state == COMMAND_ACK_COMPLETED);
  CHECK(fx->out.state_snapshot.arm_state == ARM_STATE_ARMED);
  CHECK(fx->out.actuation_permit.permit_flags == PERMIT_SCOPE_IDLE_ONLY);
}

static void fx_airborne(Fx *fx)
{
  fx->ext.ground_contact_state = GROUND_CONTACT_NO_CONTACT;
  fx->nav.core.position_ned_m[2] = -5.0f;
  fx_run(fx, 400000u);
  CHECK(fx->out.landed_state.state == LANDED_STATE_IN_AIR);
}

static void fx_make_mission(Fx *fx, uint16_t waypoints)
{
  ProductMissionPayload *m = &fx->cand.mission;
  uint16_t i;

  memset(&fx->cand, 0, sizeof(fx->cand));
  fx->cand.transaction_id = T_TRANSACTION;
  fx->cand.source_session_id = T_SESSION;
  fx->cand.state_epoch = fx->nav.state_epoch;
  fx->cand.origin_id = fx->nav.origin_id;
  fx->cand.mission_hash = T_HASH;
  m->mission_id = T_MISSION_ID;
  m->mission_revision = 1u;
  m->mission_hash = T_HASH;
  m->waypoint_count = waypoints;
  for (i = 0u; i < waypoints; ++i) {
    m->waypoints[i].waypoint_id = (uint32_t)(i + 1u);
    m->waypoints[i].position_ned_m[0] = 10.0f * (float)(i + 1u);
    m->waypoints[i].position_ned_m[1] = 0.0f;
    m->waypoints[i].position_ned_m[2] = -5.0f;
    m->waypoints[i].acceptance_radius_m = 1.5f;
    m->waypoints[i].speed_limit_mps = 2.0f;
  }
  fx->has_candidate = 1u;
}

static void fx_commit(Fx *fx)
{
  uint16_t state = fx_send(fx, PRODUCT_CMD_MISSION_COMMIT, fx->next_request_id++,
                           PRODUCT_OPERATION_VALID_MISSION_ID | PRODUCT_OPERATION_VALID_MISSION_TRANSACTION,
                           T_MISSION_ID, T_TRANSACTION, 0u, 0u, 0.0f);
  CHECK(state == COMMAND_ACK_COMPLETED);
  CHECK(fx->out.mission_status.mission_state == MISSION_STATE_LOADED);
}

static void fx_start_mission(Fx *fx)
{
  uint16_t state = fx_send(fx, PRODUCT_CMD_MISSION_START, fx->next_request_id++,
                           PRODUCT_OPERATION_VALID_MISSION_ID, T_MISSION_ID, 0u, 0u, 0u, 0.0f);
  CHECK(state == COMMAND_ACK_COMPLETED);
  CHECK(fx->out.mission_status.mission_state == MISSION_STATE_RUNNING);
  CHECK(fx->out.state_snapshot.flight_mode == FLIGHT_MODE_AUTO_MISSION);
  CHECK(fx->out.mission_status.execution_token != 0u);
}

static void fx_make_guidance(Fx *fx, uint16_t ref_epoch, uint16_t state, uint16_t kind)
{
  const FmsStateSnapshot *st = &fx->out.state_snapshot;
  const MissionStatus *ms = &fx->out.mission_status;

  memset(&fx->guidance, 0, sizeof(fx->guidance));
  memset(&fx->feas, 0, sizeof(fx->feas));
  fx->guidance.reference_id = 700u + ref_epoch;
  fx->guidance.reference_epoch = ref_epoch;
  fx->guidance.state_epoch = fx->nav.state_epoch;
  fx->guidance.origin_id = fx->nav.origin_id;
  fx->guidance.authority_epoch = st->authority_epoch;
  fx->guidance.directive_epoch = st->directive_epoch;
  fx->guidance.mission_epoch = ms->mission_epoch;
  fx->guidance.current_item_id = ms->current_item_id;
  fx->guidance.execution_token = ms->execution_token;
  fx->guidance.guidance_state = state;
  fx->guidance.reference_kind = kind;
  fx->guidance_ttl_us = 100000u;
  fx->guidance_valid = 1u;
}

static void fx_control_active(Fx *fx, uint16_t ref_epoch)
{
  fx_make_guidance(fx, ref_epoch, GUIDANCE_STATE_READY, GUIDANCE_REFERENCE_TRAJECTORY);
  (void)fx_step(fx);
  CHECK((fx->out.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE) != 0u);
}

/* ------------------------------------------------------------------ */
/* tests                                                              */
/* ------------------------------------------------------------------ */

static int fx_init(Fx *fx)
{
  ProductFmsConfig cfg;
  ProductProfile profile;

  fx_defaults(fx);
  fms_default_config(&cfg);
  memset(&profile, 0, sizeof(profile));
  profile.product_profile_id = PRODUCT_PROFILE_ID;
  profile.nav_layout_id = PRODUCT_NAV_LAYOUT_ID;
  profile.motion_reference_layout_id = PRODUCT_MOTION_REFERENCE_LAYOUT_ID;
  profile.actuator_layout_id = PRODUCT_ACTUATOR_LAYOUT_ID;
  profile.fms_policy_id = fms_selected_policy_id();
  if (fms_policy_validate(&profile) != STATUS_OK) {
    return 0;
  }
  /* CR-V191-C02: product fences are init-time configuration; clear state from
   * any previous test before installing this test's (empty) table. */
  {
    ProductGeofenceConfig geo;
    memset(&geo, 0, sizeof(geo));
    geo.profile_hash = PRODUCT_PROFILE_ID;
    geo.config_version = 1u;
    geo.geofence_hash = 1u;
    geo.payload.config_version = 1u;
    geo.payload.geofence_hash = 1u;
    geo.payload.fence_count = 0u;
    if (fms_configure_product_geofence(&geo) != STATUS_OK) {
      return 0;
    }
  }
  if (fms_policy_init(&cfg) != STATUS_OK) {
    return 0;
  }
  return 1;
}

static void test_arm_requires_fresh_prearm_epoch(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx_run(&fx, 4300000u);
  CHECK(fx.out.prearm_result.state == PREARM_READY);
  {
    uint32_t id = fx.next_request_id++;
    uint16_t state = fx_send(&fx, PRODUCT_CMD_ARM, id, PRODUCT_OPERATION_VALID_PREARM_EPOCH, 0, 0, 0,
                             fx.out.prearm_result.check_epoch - 1u, 0.0f);
    CHECK(state == COMMAND_ACK_REJECTED);
    CHECK(fx.out.state_snapshot.arm_state == ARM_STATE_DISARMED);
  }
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_arm_rechecks_current_inputs(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx_run(&fx, 4300000u);

  for (unsigned fault = 0; fault < 5; ++fault) {
    CHECK(fx.out.prearm_result.state == PREARM_READY);
    switch (fault) {
    case 0: fx.ext.energy_state = ENERGY_CRITICAL; break;
    case 1: fx.rc.link_state = LINK_STATE_LOST; break;
    case 2: fx.faults.highest_severity = 3u; break;
    case 3: fx.nav.core.valid_mask &= ~NAV_VALID_ATTITUDE; break;
    case 4: fx.ext.battery_remaining_fraction = INFINITY; break;
    }
    CHECK(fx_send(&fx, PRODUCT_CMD_ARM, fx.next_request_id++, PRODUCT_OPERATION_VALID_PREARM_EPOCH,
                  0, 0, 0, fx.out.prearm_result.check_epoch, 0.0f) == COMMAND_ACK_REJECTED);
    CHECK(fx.out.state_snapshot.arm_state == ARM_STATE_DISARMED);
    CHECK(fx.out.prearm_result.state == PREARM_REJECT);
    CHECK(fx.out.actuation_permit.permit_flags == 0u);
    fx.ext.energy_state = ENERGY_NORMAL;
    fx.rc.link_state = LINK_STATE_HEALTHY;
    fx.faults.highest_severity = 0u;
    fx.nav.core.valid_mask |= NAV_VALID_ATTITUDE;
    fx.ext.battery_remaining_fraction = 0.9f;
    CHECK(fx_step(&fx) == STATUS_OK);
  }
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_idle_lease_renewal_bounded(void)
{
  Fx fx;
  uint32_t epoch;
  uint64_t expiry;
  uint64_t emitted_at;

  CHECK(fx_init(&fx));
  fx_arm(&fx);
  CHECK(fx.out.actuation_permit.permit_flags == PERMIT_SCOPE_IDLE_ONLY);
  CHECK(fx.out.actuation_permit.reference_epoch == 0u);
  epoch = fx.out.actuation_permit.permit_epoch;
  expiry = fx.out.actuation_permit.permit_expires_at_us;
  emitted_at = fx.now - STEP_US;
  CHECK(expiry >= emitted_at);
  CHECK(expiry <= emitted_at + 25000u);
  (void)fx_step(&fx);
  CHECK(fx.out.actuation_permit.permit_epoch == epoch); /* renewal keeps epoch */
  CHECK(fx.out.actuation_permit.permit_expires_at_us > expiry);
  CHECK(fx.out.actuation_permit.permit_expires_at_us <= fx.now - STEP_US + 25000u);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_control_active_ready_handshake(void)
{
  Fx fx;

  CHECK(fx_init(&fx));
  fx_arm(&fx);
  fx_make_guidance(&fx, 21u, GUIDANCE_STATE_READY, GUIDANCE_REFERENCE_TRAJECTORY);
  (void)fx_step(&fx);
  CHECK(fx.out.actuation_permit.permit_flags == PERMIT_SCOPE_CONTROL_ACTIVE);
  CHECK(fx.out.actuation_permit.reference_epoch == 21u);
  CHECK(fx.out.actuation_permit.permit_expires_at_us > fx.now - STEP_US);
  {
    uint32_t epoch = fx.out.actuation_permit.permit_epoch;
    fx.guidance.guidance_state = GUIDANCE_STATE_ACTIVE;
    (void)fx_step(&fx);
    CHECK(fx.out.actuation_permit.permit_epoch == epoch); /* ACTIVE is not required in between */
  }
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_permit_revocation_is_immediate_and_watermarked(void)
{
  Fx fx;
  uint32_t epoch;

  CHECK(fx_init(&fx));
  fx_arm(&fx);
  fx_make_guidance(&fx, 31u, GUIDANCE_STATE_READY, GUIDANCE_REFERENCE_TRAJECTORY);
  (void)fx_step(&fx);
  CHECK((fx.out.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE) != 0u);
  epoch = fx.out.actuation_permit.permit_epoch;

  fx.guidance_valid = 0u;
  (void)fx_step(&fx);
  CHECK((fx.out.actuation_permit.permit_flags & PERMIT_FLAG_REVOKED) != 0u);
  CHECK(fx.out.actuation_permit.permit_epoch > epoch);

  (void)fx_step(&fx);
  CHECK(fx.out.actuation_permit.header.valid_until_us == 0u ||
        (fx.out.actuation_permit.permit_flags & PERMIT_SCOPE_IDLE_ONLY) != 0u);
  CHECK(fx.out.actuation_permit.permit_epoch > epoch);

  fx_make_guidance(&fx, 32u, GUIDANCE_STATE_READY, GUIDANCE_REFERENCE_TRAJECTORY);
  (void)fx_step(&fx);
  CHECK((fx.out.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE) != 0u);
  CHECK(fx.out.actuation_permit.permit_epoch > epoch + 1u);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_guidance_epoch_mismatch_has_no_permit(void)
{
  Fx fx;

  CHECK(fx_init(&fx));
  fx_arm(&fx);
  fx_make_guidance(&fx, 41u, GUIDANCE_STATE_READY, GUIDANCE_REFERENCE_TRAJECTORY);
  fx.guidance.authority_epoch += 5u;
  (void)fx_step(&fx);
  CHECK((fx.out.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE) == 0u);

  fx_make_guidance(&fx, 42u, GUIDANCE_STATE_READY, GUIDANCE_REFERENCE_TRAJECTORY);
  fx.guidance_ttl_us = 0u; /* reference expires at evaluation time */
  (void)fx_step(&fx);
  CHECK((fx.out.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE) == 0u);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_mission_commit_validates_source_and_geometry(void)
{
  Fx fx;
  uint16_t state;

  CHECK(fx_init(&fx));
  fx_make_mission(&fx, 2u);
  fx.cand.source_session_id = T_SESSION + 1u;
  state = fx_send(&fx, PRODUCT_CMD_MISSION_COMMIT, fx.next_request_id++,
                  PRODUCT_OPERATION_VALID_MISSION_ID | PRODUCT_OPERATION_VALID_MISSION_TRANSACTION, T_MISSION_ID,
                  T_TRANSACTION, 0, 0, 0.0f);
  CHECK(state == COMMAND_ACK_REJECTED);
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_IDLE);

  fx_make_mission(&fx, 0u); /* invalid empty mission */
  state = fx_send(&fx, PRODUCT_CMD_MISSION_COMMIT, fx.next_request_id++,
                  PRODUCT_OPERATION_VALID_MISSION_ID | PRODUCT_OPERATION_VALID_MISSION_TRANSACTION, T_MISSION_ID,
                  T_TRANSACTION, 0, 0, 0.0f);
  CHECK(state == COMMAND_ACK_REJECTED);

  fx_make_mission(&fx, 2u);
  fx.cand.mission.waypoints[1].waypoint_id = fx.cand.mission.waypoints[0].waypoint_id;
  state = fx_send(&fx, PRODUCT_CMD_MISSION_COMMIT, fx.next_request_id++,
                  PRODUCT_OPERATION_VALID_MISSION_ID | PRODUCT_OPERATION_VALID_MISSION_TRANSACTION, T_MISSION_ID,
                  T_TRANSACTION, 0, 0, 0.0f);
  CHECK(state == COMMAND_ACK_REJECTED);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_mission_start_requires_arm(void)
{
  Fx fx;
  uint16_t state;

  CHECK(fx_init(&fx));
  fx_make_mission(&fx, 2u);
  fx_commit(&fx);
  state = fx_send(&fx, PRODUCT_CMD_MISSION_START, fx.next_request_id++, PRODUCT_OPERATION_VALID_MISSION_ID,
                  T_MISSION_ID, 0, 0, 0, 0.0f);
  CHECK(state == COMMAND_ACK_REJECTED);
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_LOADED);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_two_waypoint_mission_token_and_completion(void)
{
  Fx fx;
  uint32_t first_epoch;
  uint64_t first_token;

  CHECK(fx_init(&fx));
  fx_make_mission(&fx, 2u);
  fx_commit(&fx);
  fx_arm(&fx);
  fx_control_active(&fx, 51u);
  fx_airborne(&fx);
  fx_start_mission(&fx);

  first_epoch = fx.out.mission_status.mission_epoch;
  first_token = fx.out.mission_status.execution_token;
  CHECK(first_token != 0u);
  CHECK(fx.out.mission_status.current_item_id == 1u);
  CHECK(fx.out.mission_status.progress_0_to_1 == 0.0f);

  /* Authorize the first segment from a READY candidate. */
  fx_make_guidance(&fx, 61u, GUIDANCE_STATE_READY, GUIDANCE_REFERENCE_TRAJECTORY);
  (void)fx_step(&fx);
  CHECK((fx.out.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE) != 0u);
  CHECK(fx.out.actuation_permit.reference_epoch == 61u);
  CHECK(fx.out.authorized_mission.execution_token == first_token);
  CHECK(fx.out.authorized_mission.current_item_index == 0u);

  /* Not inside the acceptance radius: no advance. */
  fx.nav.core.position_ned_m[0] = 5.0f;
  fx_run(&fx, 300000u);
  CHECK(fx.out.mission_status.current_item_id == 1u);

  /* Inside radius and slow for the dwell: advance to waypoint 2, new token. */
  fx.nav.core.position_ned_m[0] = 10.0f;
  fx_run(&fx, 300000u);
  CHECK(fx.out.mission_status.current_item_id == 2u);
  CHECK(fx.out.mission_status.execution_token != first_token);
  CHECK(fx.out.mission_status.mission_epoch == first_epoch); /* item advance keeps mission epoch */
  CHECK(fx.out.mission_status.progress_0_to_1 > 0.4f);

  /* Old token cannot authorize the new segment. */
  {
    uint64_t new_token = fx.out.mission_status.execution_token;
    fx_make_guidance(&fx, 62u, GUIDANCE_STATE_READY, GUIDANCE_REFERENCE_TRAJECTORY);
    fx.guidance.execution_token = first_token; /* stale pre-advance token */
    (void)fx_step(&fx);
    CHECK((fx.out.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE) == 0u);
    fx_make_guidance(&fx, 63u, GUIDANCE_STATE_READY, GUIDANCE_REFERENCE_TRAJECTORY);
    (void)fx_step(&fx);
    CHECK((fx.out.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE) != 0u);
    CHECK(fx.out.authorized_mission.execution_token == new_token);
    CHECK(fx.out.authorized_mission.current_item_id == 2u);
  }

  /* Second waypoint: mission completes and the token is withdrawn. */
  fx.nav.core.position_ned_m[0] = 20.0f;
  fx_run(&fx, 300000u);
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_COMPLETED);
  CHECK(fx.out.mission_status.result_code == MISSION_RESULT_SUCCESS);
  CHECK(fx.out.mission_status.execution_token == 0u);
  CHECK(fx.out.mission_status.progress_0_to_1 == 1.0f);
  CHECK(fx.out.actuation_permit.header.valid_until_us == 0u ||
        (fx.out.actuation_permit.permit_flags & PERMIT_FLAG_REVOKED) != 0u);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_pause_resume_boundaries(void)
{
  Fx fx;
  uint64_t token;
  uint32_t epoch;
  uint16_t state;

  CHECK(fx_init(&fx));
  fx_make_mission(&fx, 2u);
  fx_commit(&fx);
  fx_arm(&fx);
  fx_control_active(&fx, 71u);
  fx_airborne(&fx);
  fx_start_mission(&fx);
  token = fx.out.mission_status.execution_token;
  epoch = fx.out.mission_status.mission_epoch;

  state = fx_send(&fx, PRODUCT_CMD_MISSION_PAUSE, fx.next_request_id++, PRODUCT_OPERATION_VALID_MISSION_ID,
                  T_MISSION_ID, 0, 0, 0, 0.0f);
  CHECK(state == COMMAND_ACK_COMPLETED);
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_PAUSED);
  CHECK(fx.out.mission_status.execution_token == 0u);
  CHECK(fx.out.mission_status.mission_epoch > epoch);
  CHECK(fx.out.authorized_mission.header.valid_until_us == 0u);

  /* Pausing again is rejected; progress stays frozen. */
  state = fx_send(&fx, PRODUCT_CMD_MISSION_PAUSE, fx.next_request_id++, PRODUCT_OPERATION_VALID_MISSION_ID,
                  T_MISSION_ID, 0, 0, 0, 0.0f);
  CHECK(state == COMMAND_ACK_REJECTED);
  CHECK(fx.out.mission_status.current_item_id == 1u);

  fx_run(&fx, 1200000u); /* Longer than the authorized snapshot TTL. */
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_PAUSED);
  CHECK(fx.out.authorized_mission.header.valid_until_us == 0u);
  state = fx_send(&fx, PRODUCT_CMD_MISSION_RESUME, fx.next_request_id++, PRODUCT_OPERATION_VALID_MISSION_ID,
                  T_MISSION_ID, 0, 0, 0, 0.0f);
  CHECK(state == COMMAND_ACK_COMPLETED);
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_RUNNING);
  CHECK(fx.out.mission_status.execution_token != token);
  CHECK(fx.out.mission_status.mission_epoch > epoch + 1u);
  CHECK(fx.out.authorized_mission.current_item_index == 0u);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_abort_withdraws_token(void)
{
  Fx fx;
  uint16_t state;

  CHECK(fx_init(&fx));
  fx_make_mission(&fx, 2u);
  fx_commit(&fx);
  fx_arm(&fx);
  fx_control_active(&fx, 81u);
  fx_airborne(&fx);
  fx_start_mission(&fx);
  state = fx_send(&fx, PRODUCT_CMD_MISSION_ABORT, fx.next_request_id++, PRODUCT_OPERATION_VALID_MISSION_ID,
                  T_MISSION_ID, 0, 0, 0, 0.0f);
  CHECK(state == COMMAND_ACK_COMPLETED);
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_ABORTED);
  CHECK(fx.out.mission_status.result_code == MISSION_RESULT_ABORTED);
  CHECK(fx.out.mission_status.execution_token == 0u);
  CHECK(fx.out.authorized_mission.header.valid_until_us == 0u);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_disarm_rejected_in_air(void)
{
  Fx fx;
  uint16_t state;

  CHECK(fx_init(&fx));
  fx_arm(&fx);
  fx_control_active(&fx, 82u);
  fx_airborne(&fx);
  state = fx_send_simple(&fx, PRODUCT_CMD_DISARM);
  CHECK(state == COMMAND_ACK_REJECTED);
  CHECK(fx.out.state_snapshot.arm_state == ARM_STATE_ARMED);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_pilot_takeover_latches_and_pauses(void)
{
  Fx fx;
  uint32_t id;

  CHECK(fx_init(&fx));
  fx_make_mission(&fx, 2u);
  fx_commit(&fx);
  fx_arm(&fx);
  fx_control_active(&fx, 83u);
  fx_airborne(&fx);
  fx_start_mission(&fx);

  fx.op.payload.mode_switch_bits = (FLIGHT_MODE_ALT_HOLD | 0x10u);
  id = fx.next_request_id++;
  (void)fx_send(&fx, PRODUCT_CMD_SET_MODE, id, PRODUCT_OPERATION_VALID_REQUESTED_MODE, 0, 0, FLIGHT_MODE_ALT_HOLD,
                0, 0.0f);
  (void)fx_step(&fx);
  CHECK(fx.out.state_snapshot.authority == AUTHORITY_PILOT);
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_PAUSED);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_ALT_HOLD);

  /* Stick centring and a repeated mode word never release the latch. */
  fx.op.payload.mode_switch_bits = FLIGHT_MODE_ALT_HOLD;
  fx_run(&fx, 100000u);
  CHECK(fx.out.state_snapshot.authority == AUTHORITY_PILOT);

  /* GCS mode requests are rejected while the pilot latch is held. */
  {
    uint16_t ack = fx_send(&fx, PRODUCT_CMD_SET_MODE, fx.next_request_id++, PRODUCT_OPERATION_VALID_REQUESTED_MODE,
                           0, 0, FLIGHT_MODE_POS_HOLD, 0, 0.0f);
    CHECK(ack == COMMAND_ACK_REJECTED);
  }
  CHECK(fx_send_simple(&fx, PRODUCT_CMD_LAND) == COMMAND_ACK_REJECTED);
  CHECK(fx_send_simple(&fx, PRODUCT_CMD_RETURN) == COMMAND_ACK_REJECTED);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_ALT_HOLD);
  fx.op.payload.mode_switch_bits = PRODUCT_MODE_SWITCH_MAPPED | FLIGHT_MODE_POS_HOLD;
  fx_run(&fx, 200000u);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_ALT_HOLD); /* Initial position. */
  fx.op.payload.mode_switch_bits = PRODUCT_MODE_SWITCH_MAPPED | FLIGHT_MODE_ALT_HOLD;
  fx_run(&fx, 200000u);
  fx.op.payload.mode_switch_bits = PRODUCT_MODE_SWITCH_MAPPED | FLIGHT_MODE_POS_HOLD;
  fx_run(&fx, 50000u);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_ALT_HOLD); /* Debounce. */
  fx_run(&fx, 180000u);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_POS_HOLD);
  CHECK(fx.out.state_snapshot.authority == AUTHORITY_PILOT);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_rc_loss_forces_return_and_pauses_mission(void)
{
  Fx fx;

  CHECK(fx_init(&fx));
  fx_make_mission(&fx, 2u);
  fx_commit(&fx);
  fx_arm(&fx);
  fx_control_active(&fx, 84u);
  fx_airborne(&fx);
  fx_start_mission(&fx);

  fx.rc.link_state = LINK_STATE_LOST;
  fx.rc.last_valid_frame_us = fx.now - 1000000u;
  fx_run(&fx, 200000u);
  CHECK(fx.out.state_snapshot.failsafe_level >= FAILSAFE_ACTION_REQUIRED);
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_PAUSED);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_AUTO_RETURN);
  CHECK(fx.out.state_snapshot.authority == AUTHORITY_NONE);

  /* Link recovery alone does not restore authority or resume the mission. */
  fx.rc.link_state = LINK_STATE_HEALTHY;
  fx_run(&fx, 300000u);
  CHECK(fx.out.state_snapshot.authority != AUTHORITY_PILOT);
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_PAUSED);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_rejected_return_falls_back_to_land(void)
{
  Fx fx;
  CHECK(fx_init(&fx)); fx_arm(&fx); fx_control_active(&fx, 84u); fx_airborne(&fx);
  const uint32_t previous_directive = fx.out.state_snapshot.directive_epoch;
  CHECK(fx_send_simple(&fx, PRODUCT_CMD_RETURN) == COMMAND_ACK_ACCEPTED);

  fx.rc.link_state = LINK_STATE_LOST;
  fx_make_guidance(&fx, 85u, GUIDANCE_STATE_REJECTED, GUIDANCE_REFERENCE_TRAJECTORY);
  const uint32_t return_directive = fx.guidance.directive_epoch;
  fx.guidance.directive_epoch = previous_directive;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_AUTO_RETURN);

  fx.guidance.directive_epoch = return_directive;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_AUTO_LAND);
  CHECK(fx.out.state_snapshot.failsafe_level == FAILSAFE_ACTION_REQUIRED);
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_AUTO_LAND);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_home_renewal_requires_position_evidence(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx_arm(&fx);
  fx_control_active(&fx, 84u);
  fx_airborne(&fx);

  const ProductHomeTargetSnapshot home = fx.out.directive.payload.home;
  fx.nav.core.position_ned_m[0] = 3.0f;
  fx_run(&fx, 5500000u);
  CHECK(fx.now > home.home_expires_at_us);
  CHECK(fx.out.directive.payload.home.validity == PRODUCT_HOME_VALID);
  CHECK(fx.out.directive.payload.home.home_id == home.home_id);
  CHECK(fx.out.directive.payload.home.captured_at_us == home.captured_at_us);
  CHECK(memcmp(fx.out.directive.payload.home.position_ned_m, home.position_ned_m,
               sizeof(home.position_ned_m)) == 0);
  CHECK(fx_send_simple(&fx, PRODUCT_CMD_RETURN) == COMMAND_ACK_ACCEPTED);

  fx.cap.capability_mask &= ~NAV_VALID_HORIZONTAL_POSITION;
  fx_run(&fx, 5500000u);
  CHECK(fx.out.directive.payload.home.validity == PRODUCT_HOME_INVALID);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_operator_home_survives_arm(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx_run(&fx, 4300000u);
  const uint32_t id = fx.next_request_id++;
  fx.req.request_id = id;
  fx.req.source_session_id = T_SESSION;
  fx.req.command_code = PRODUCT_CMD_SET_HOME;
  fx.req.payload.valid_mask = PRODUCT_OPERATION_VALID_TARGET_NED;
  fx.req.payload.target_ned_m[0] = 2.0f;
  fx.has_req = 1u;
  CHECK(fx_step(&fx) == STATUS_OK);
  const CommandAck *ack = fx_find_ack(&fx, id);
  CHECK(ack && ack->ack_state == COMMAND_ACK_COMPLETED);
  const uint64_t expires = fx.out.directive.payload.home.home_expires_at_us;
  fx_run(&fx, 5500000u);
  CHECK(fx.now > expires);
  fx_arm(&fx);
  CHECK(fx.out.directive.payload.home.home_id == id);
  CHECK(fx.out.directive.payload.home.capture_source == PRODUCT_HOME_CAPTURE_OPERATOR);
  CHECK(fx.out.directive.payload.home.position_ned_m[0] == 2.0f);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_clear_empty_mission_requires_disarm(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  const uint32_t epoch = fx.out.mission_status.mission_epoch;
  fx.epoch_override = -2;
  CHECK(fx_send_simple(&fx, PRODUCT_CMD_MISSION_CLEAR) == COMMAND_ACK_COMPLETED);
  CHECK(fx.out.mission_status.mission_epoch == epoch);
  fx_arm(&fx);
  fx.epoch_override = -2;
  CHECK(fx_send_simple(&fx, PRODUCT_CMD_MISSION_CLEAR) == COMMAND_ACK_REJECTED);
  CHECK(fx.out.mission_status.mission_epoch == epoch);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void fx_complete_single_waypoint_mission(Fx *fx)
{
  CHECK(fx_init(fx));
  fx_make_mission(fx, 1u);
  fx_commit(fx);
  fx_arm(fx);
  fx_control_active(fx, 80u);
  fx_airborne(fx);
  fx_start_mission(fx);
  memcpy(fx->nav.core.position_ned_m, fx->cand.mission.waypoints[0].position_ned_m,
         sizeof(fx->nav.core.position_ned_m));
  fx_run(fx, 260000u);
  CHECK(fx->out.mission_status.result_code == MISSION_RESULT_SUCCESS);
  CHECK(fx->out.state_snapshot.flight_mode == FLIGHT_MODE_POS_HOLD);
}

static void test_return_pauses_running_mission(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx_make_mission(&fx, 3u);
  fx_commit(&fx);
  fx_arm(&fx);
  fx_control_active(&fx, 80u);
  fx_airborne(&fx);
  fx_start_mission(&fx);
  fx.control_sample_lag_us = 20000u; /* M6 publish follows a slightly older nav sample. */
  CHECK(fx.out.authorized_mission.execution_token != 0u);
  const uint32_t epoch = fx.out.mission_status.mission_epoch;
  fx.nav.core.velocity_ned_mps[0] = 0.3f;
  CHECK(fx_send_simple(&fx, PRODUCT_CMD_RETURN) == COMMAND_ACK_ACCEPTED);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_AUTO_MISSION);
  fx_control_active(&fx, 81u);
  fx_run(&fx, 120000u);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_POS_HOLD);
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_PAUSED);
  CHECK(fx.out.mission_status.mission_epoch != epoch);
  CHECK(fx.out.authorized_mission.execution_token == 0u);

  /* A new M6 binding for the POS_HOLD directive is sufficient to hand off.
   * The return trajectory owns deceleration; M4 does not wait for a stationary
   * hover or a low instantaneous acceleration estimate. */
  fx_control_active(&fx, 82u);
  fx_run(&fx, 1200000u);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_AUTO_RETURN);

  memset(fx.nav.core.position_ned_m, 0, sizeof(fx.nav.core.position_ned_m));
  memset(fx.nav.core.velocity_ned_mps, 0, sizeof(fx.nav.core.velocity_ned_mps));
  memset(fx.nav.core.acceleration_ned_mps2, 0, sizeof(fx.nav.core.acceleration_ned_mps2));
  fx.ext.ground_contact_state = GROUND_CONTACT_CONTACT;
  fx_run(&fx, 2200000u);
  CHECK(fx.out.landed_state.state == LANDED_STATE_LANDED);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_post_mission_return_starts_after_control_binding(void)
{
  Fx fx;
  FmsInputView input;
  const uint32_t request_id = 3200u;
  fx_complete_single_waypoint_mission(&fx);
  fx.control_sample_lag_us = 20000u; /* Bind on M6 processing time, not sensor capture time. */
  fx_control_active(&fx, 81u);
  fx_run(&fx, 100000u); /* The same directive has a continuous 80 ms M6 binding. */

  /* Deliberately non-stationary: the bounded M5 return trajectory must own
   * deceleration instead of waiting for a separate hover-stability gate. */
  fx.nav.core.velocity_ned_mps[0] = 0.3f;
  fx.nav.core.acceleration_ned_mps2[2] = 2.0f;
  CHECK(fx_send(&fx, PRODUCT_CMD_RETURN, request_id, 0, 0, 0, 0, 0, 0.0f) == COMMAND_ACK_ACCEPTED);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_AUTO_RETURN);
  CHECK(fx.out.actuation_permit.permit_flags & PERMIT_FLAG_REVOKED);
  const uint64_t deadline = fx.req.execution_deadline_us;

  fx_control_active(&fx, 82u);
  memset(fx.nav.core.position_ned_m, 0, sizeof(fx.nav.core.position_ned_m));
  memset(fx.nav.core.velocity_ned_mps, 0, sizeof(fx.nav.core.velocity_ned_mps));
  memset(fx.nav.core.acceleration_ned_mps2, 0, sizeof(fx.nav.core.acceleration_ned_mps2));
  fx.ext.ground_contact_state = GROUND_CONTACT_CONTACT;
  fx_run(&fx, 2200000u);
  CHECK(fx.out.landed_state.state == LANDED_STATE_LANDED);
  fx.has_req = 1u;
  fx_fill_headers(&fx);
  fx.req.execution_deadline_us = deadline; /* The retransmission did not change the request. */
  fx_view(&fx, &input);
  CHECK(fms_policy_update(&input, &fx.out) == STATUS_OK);
  const CommandAck *ack = fx_find_ack(&fx, request_id);
  CHECK(ack != NULL && ack->ack_state == COMMAND_ACK_COMPLETED);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_post_mission_return_missing_control_cancels_safely(void)
{
  Fx fx;
  const uint32_t request_id = 3201u;
  fx_complete_single_waypoint_mission(&fx);
  fx_control_active(&fx, 81u);
  fx_run(&fx, 100000u);
  fx.omit_control = 1u; /* No current M6 capability may authorize AUTO_RETURN. */
  CHECK(fx_send(&fx, PRODUCT_CMD_RETURN, request_id, 0, 0, 0, 0, 0, 0.0f) == COMMAND_ACK_ACCEPTED);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_POS_HOLD);
  fx_run(&fx, FMS_RETURN_PREPARE_TIMEOUT_US + 100000u);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_POS_HOLD);
  CHECK(fx.out.state_snapshot.failsafe_level >= FAILSAFE_ACTION_REQUIRED);
  CHECK((fx.out.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE) == 0u);
  CHECK(fx_send(&fx, PRODUCT_CMD_RETURN, request_id, 0, 0, 0, 0, 0, 0.0f) == COMMAND_ACK_FAILED);
  const CommandAck *ack = fx_find_ack(&fx, request_id);
  CHECK(ack != NULL && ack->reason_code == 12u);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_duplicate_request_replays_ack(void)
{
  Fx fx;
  uint32_t id;
  uint16_t first, second;
  uint32_t epoch_after_first;
  uint32_t prearm_epoch;

  CHECK(fx_init(&fx));
  fx_run(&fx, 4300000u);
  prearm_epoch = fx.out.prearm_result.check_epoch;
  id = fx.next_request_id++;
  first = fx_send(&fx, PRODUCT_CMD_ARM, id, PRODUCT_OPERATION_VALID_PREARM_EPOCH, 0, 0, 0, prearm_epoch, 0.0f);
  CHECK(first == COMMAND_ACK_COMPLETED);
  epoch_after_first = fx.out.state_snapshot.authority_epoch;
  /* Byte-identical retransmission with the same idempotency key. */
  second = fx_send(&fx, PRODUCT_CMD_ARM, id, PRODUCT_OPERATION_VALID_PREARM_EPOCH, 0, 0, 0, prearm_epoch, 0.0f);
  CHECK(second == first);
  CHECK(fx.out.state_snapshot.authority_epoch == epoch_after_first);
  id = fx.next_request_id++;
  CHECK(fx_send(&fx, PRODUCT_CMD_LAND, id, 0, 0, 0, 0, 0, 0.0f) == COMMAND_ACK_COMPLETED);
  CHECK(fx_send(&fx, PRODUCT_CMD_RETURN, id, 0, 0, 0, 0, 0, 0.0f) == COMMAND_ACK_REJECTED);
  const CommandAck *ack = fx_find_ack(&fx, id);
  CHECK(ack != NULL && ack->reason_code == 16u);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_dedup_full_and_expiry(void)
{
  Fx fx;
  const CommandAck *ack;
  uint32_t first_id;
  uint32_t overflow_id;

  CHECK(fx_init(&fx));
  first_id = fx.next_request_id;
  for (unsigned i = 0u; i < FMS_DEDUP_CAPACITY; ++i) {
    const uint32_t id = fx.next_request_id++;
    CHECK(fx_send(&fx, PRODUCT_CMD_LAND, id, 0, 0, 0, 0, 0, 0.0f) == COMMAND_ACK_REJECTED);
    ack = fx_find_ack(&fx, id);
    CHECK(ack != NULL && ack->reason_code == 2u); /* Disarmed. */
  }

  overflow_id = fx.next_request_id++;
  CHECK(fx_send(&fx, PRODUCT_CMD_LAND, overflow_id, 0, 0, 0, 0, 0, 0.0f) == COMMAND_ACK_REJECTED);
  ack = fx_find_ack(&fx, overflow_id);
  CHECK(ack != NULL && ack->reason_code == 9u); /* Capacity. */
  CHECK(fx_send(&fx, PRODUCT_CMD_LAND, first_id, 0, 0, 0, 0, 0, 0.0f) == COMMAND_ACK_REJECTED);
  ack = fx_find_ack(&fx, first_id);
  CHECK(ack != NULL && ack->reason_code == 2u); /* Cached rejection. */

  fx_run(&fx, (uint32_t)FMS_DEDUP_TTL_US);
  CHECK(fx_send(&fx, PRODUCT_CMD_LAND, overflow_id, 0, 0, 0, 0, 0, 0.0f) == COMMAND_ACK_REJECTED);
  ack = fx_find_ack(&fx, overflow_id);
  CHECK(ack != NULL && ack->reason_code == 2u); /* Capacity reclaimed. */
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_geofence_required_rejects_outside_waypoint(void)
{
  Fx fx;
  ProductGeofenceConfig geo;
  ProductFmsConfig cfg;
  uint16_t state;

  fx_defaults(&fx);
  memset(&geo, 0, sizeof(geo));
  geo.profile_hash = PRODUCT_PROFILE_ID;
  geo.config_version = 2u;
  geo.geofence_hash = 0x1234u;
  geo.payload.config_version = 2u;
  geo.payload.geofence_hash = 0x1234u;
  geo.payload.fence_count = 1u;
  geo.payload.fences[0].fence_id = 1u;
  geo.payload.fences[0].kind = PRODUCT_GEOFENCE_KEEP_IN;
  geo.payload.fences[0].vertex_count = 3u;
  geo.payload.fences[0].vertices_ne_m[0][0] = -2.0f;
  geo.payload.fences[0].vertices_ne_m[0][1] = -2.0f;
  geo.payload.fences[0].vertices_ne_m[1][0] = 2.0f;
  geo.payload.fences[0].vertices_ne_m[1][1] = -2.0f;
  geo.payload.fences[0].vertices_ne_m[2][0] = 0.0f;
  geo.payload.fences[0].vertices_ne_m[2][1] = 2.0f;
  geo.payload.fences[0].min_height_above_home_m = 0.0f;
  geo.payload.fences[0].max_height_above_home_m = 20.0f;

  /* CR-V191-C02: compile-time product configuration, installed before init. */
  CHECK(fms_configure_product_geofence(&geo) == STATUS_OK);
  fms_default_config(&cfg);
  cfg.payload.geofence_required = 1u;
  CHECK(fms_policy_init(&cfg) == STATUS_OK);
  memset(&geo, 0, sizeof(geo)); /* the borrowed init view can now expire */
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.geofence_count == 1u);
  CHECK(fx.out.geofences[0].fence_revision == 2u);
  CHECK(fx.out.geofences[0].origin_id == T_ORIGIN);
  CHECK(fx.out.geofences[0].state_epoch == T_STATE_EPOCH);

  fx_make_mission(&fx, 2u); /* waypoints at x=10/20 are outside the fence */
  state = fx_send(&fx, PRODUCT_CMD_MISSION_COMMIT, fx.next_request_id++,
                  PRODUCT_OPERATION_VALID_MISSION_ID | PRODUCT_OPERATION_VALID_MISSION_TRANSACTION, T_MISSION_ID,
                  T_TRANSACTION, 0, 0, 0.0f);
  CHECK(state == COMMAND_ACK_REJECTED);
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_IDLE);
  CHECK(fms_policy_stop() == STATUS_OK);

  /* A required fence with no compile-time configuration fails closed. */
  CHECK(fms_policy_init(&cfg) == STATUS_PROFILE_MISMATCH);
  memset(&geo, 0, sizeof(geo));
  geo.profile_hash = PRODUCT_PROFILE_ID;
  geo.config_version = 1u;
  geo.geofence_hash = 1u;
  geo.payload.config_version = 1u;
  geo.payload.geofence_hash = 1u;
  geo.payload.fence_count = 0u;
  CHECK(fms_configure_product_geofence(&geo) == STATUS_OK);
  CHECK(fms_policy_init(&cfg) == STATUS_PROFILE_MISMATCH);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_mission_epoch_required_and_not_stale(void)
{
  Fx fx;
  uint32_t epoch;
  uint16_t state;

  CHECK(fx_init(&fx));
  fx_make_mission(&fx, 2u);
  fx_commit(&fx);
  fx_arm(&fx);
  fx_control_active(&fx, 91u);
  fx_airborne(&fx);
  fx_start_mission(&fx);
  epoch = fx.out.mission_status.mission_epoch;

  /* Stale epoch is rejected and does not change state (CR-V191-C01). */
  fx.epoch_override = (int32_t)epoch - 1;
  state = fx_send(&fx, PRODUCT_CMD_MISSION_PAUSE, fx.next_request_id++,
                  PRODUCT_OPERATION_VALID_MISSION_ID, T_MISSION_ID, 0, 0, 0, 0.0f);
  CHECK(state == COMMAND_ACK_REJECTED);
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_RUNNING);
  CHECK(fx.out.mission_status.mission_epoch == epoch);

  /* Missing mission_epoch is rejected as well. */
  fx.epoch_override = -2;
  state = fx_send(&fx, PRODUCT_CMD_MISSION_PAUSE, fx.next_request_id++,
                  PRODUCT_OPERATION_VALID_MISSION_ID, T_MISSION_ID, 0, 0, 0, 0.0f);
  CHECK(state == COMMAND_ACK_REJECTED);
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_RUNNING);

  /* Current epoch is accepted. */
  state = fx_send(&fx, PRODUCT_CMD_MISSION_PAUSE, fx.next_request_id++,
                  PRODUCT_OPERATION_VALID_MISSION_ID, T_MISSION_ID, 0, 0, 0, 0.0f);
  CHECK(state == COMMAND_ACK_COMPLETED);
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_PAUSED);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_first_reference_window_expires_to_ground_stop(void)
{
  Fx fx;
  GroundStopCertificate certificate;
  uint32_t id;

  CHECK(fx_init(&fx));
  fx_run(&fx, 4300000u);
  id = fx.next_request_id++;
  CHECK(fx_send(&fx, PRODUCT_CMD_ARM, id, PRODUCT_OPERATION_VALID_PREARM_EPOCH, 0, 0, 0,
                fx.out.prearm_result.check_epoch, 0.0f) == COMMAND_ACK_COMPLETED);
  fx_run(&fx, 90000u);
  fx.now += STEP_US;
  fx_make_guidance(&fx, 91u, GUIDANCE_STATE_READY, GUIDANCE_REFERENCE_TRAJECTORY);
  CHECK(fx_step(&fx) == STATUS_OK); /* The first candidate arrives after the absolute deadline. */
  CHECK(fx.out.state_snapshot.arm_state == ARM_STATE_DISARMED);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_NONE);
  CHECK(fx.out.landed_state.state == LANDED_STATE_LANDED);
  CHECK(fx.out.actuation_permit.permit_flags & PERMIT_FLAG_REVOKED);
  CHECK(fx.out.actuation_permit.reason_code ==
        FMS_PERMIT_REVOKE_REASON_FIRST_REFERENCE_GROUND_DISARM);
  CHECK(fx.out.decision_event.decision_code == DECISION_FORCE_SAFE_ACTION);
  CHECK(fx.out.decision_event.reason_code ==
        FMS_DECISION_REASON_FIRST_REFERENCE_GROUND_DISARM);
  CHECK(fms_build_ground_stop_certificate(&fx.out, fx.now - STEP_US, &certificate) ==
        STATUS_OK);
  CHECK(certificate.header.message_type == MSG_GROUND_STOP_CERTIFICATE);
  CHECK(certificate.lifecycle == ACTUATOR_GROUND_STOP_LIFECYCLE_LANDED);
  CHECK(certificate.authority_epoch == fx.out.actuation_permit.authority_epoch);
  CHECK(certificate.directive_epoch == fx.out.actuation_permit.directive_epoch);
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.actuation_permit.header.valid_until_us == 0u);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_nav_loss_and_recovery_dwell(void)
{
  Fx fx;
  CHECK(fx_init(&fx)); fx_make_mission(&fx, 2); fx_commit(&fx);
  fx_arm(&fx); fx_control_active(&fx, 91); fx_airborne(&fx); fx_start_mission(&fx);
  fx.cap.capability_mask &= ~NAV_VALID_HORIZONTAL_POSITION;
  fx.nav.core.valid_mask &= ~NAV_VALID_HORIZONTAL_POSITION;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_PAUSED);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_AUTO_LAND);
  fx.cap.capability_mask |= NAV_VALID_HORIZONTAL_POSITION;
  fx.nav.core.valid_mask |= NAV_VALID_HORIZONTAL_POSITION;
  CHECK(fx_send(&fx, PRODUCT_CMD_MISSION_RESUME, fx.next_request_id++,
      PRODUCT_OPERATION_VALID_MISSION_ID, T_MISSION_ID, 0, 0, 0, 0) == COMMAND_ACK_REJECTED);
  fx_run(&fx, 220000);
  CHECK(fx_send(&fx, PRODUCT_CMD_MISSION_RESUME, fx.next_request_id++,
      PRODUCT_OPERATION_VALID_MISSION_ID, T_MISSION_ID, 0, 0, 0, 0) == COMMAND_ACK_COMPLETED);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_critical_energy_forces_land(void)
{
  Fx fx;
  CHECK(fx_init(&fx)); fx_make_mission(&fx, 2); fx_commit(&fx);
  fx_arm(&fx); fx_control_active(&fx, 92); fx_airborne(&fx); fx_start_mission(&fx);
  fx.ext.energy_state = ENERGY_CRITICAL;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_AUTO_LAND);
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_PAUSED);
  CHECK(fx.out.state_snapshot.failsafe_level == FAILSAFE_ACTION_REQUIRED);
  fx.ext.energy_state = ENERGY_UNKNOWN; /* The energy publisher went stale. */
  fx.rc.link_state = LINK_STATE_LOST;
  fx_run(&fx, 250000u);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_AUTO_LAND);
  CHECK(fx.out.state_snapshot.failsafe_level == FAILSAFE_ACTION_REQUIRED);
  fx.ext.energy_state = ENERGY_NORMAL; /* The load relaxed and voltage recovered. */
  fx.rc.link_state = LINK_STATE_HEALTHY;
  fx.op.payload.mode_switch_bits = FLIGHT_MODE_ALT_HOLD | 0x10u;
  CHECK(fx_send(&fx, PRODUCT_CMD_SET_MODE, fx.next_request_id++, PRODUCT_OPERATION_VALID_REQUESTED_MODE,
                0, 0, FLIGHT_MODE_POS_HOLD, 0, 0.0f) == COMMAND_ACK_REJECTED);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_AUTO_LAND);
  CHECK(fx.out.state_snapshot.authority != AUTHORITY_PILOT);
  CHECK(fx.out.state_snapshot.failsafe_level == FAILSAFE_ACTION_REQUIRED);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_forced_land_cancels_return(void)
{
  Fx fx;
  CHECK(fx_init(&fx)); fx_arm(&fx); fx_control_active(&fx, 92); fx_airborne(&fx);
  const uint32_t return_id = fx.next_request_id++;
  CHECK(fx_send(&fx, PRODUCT_CMD_RETURN, return_id, 0, 0, 0, 0, 0, 0.0f) == COMMAND_ACK_ACCEPTED);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_AUTO_RETURN);

  fx.ext.energy_state = ENERGY_CRITICAL;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_AUTO_LAND);
  const CommandAck *ack = fx_find_ack(&fx, return_id);
  CHECK(ack != NULL && ack->ack_state == COMMAND_ACK_CANCELLED);
  CHECK(fx_send(&fx, PRODUCT_CMD_RETURN, return_id, 0, 0, 0, 0, 0, 0.0f) == COMMAND_ACK_CANCELLED);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_safety_handoff_missing_control_expires(void)
{
  Fx fx;
  CHECK(fx_init(&fx)); fx_make_mission(&fx, 2); fx_commit(&fx);
  fx_arm(&fx); fx_control_active(&fx, 92); fx_airborne(&fx); fx_start_mission(&fx);
  fx_control_active(&fx, 93);

  fx.ext.energy_state = ENERGY_CRITICAL;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.actuation_permit.permit_flags & PERMIT_FLAG_REVOKED);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_AUTO_LAND);

  fx.omit_control = 1u;
  fx_run(&fx, 40000u);
  CHECK(fx.out.state_snapshot.failsafe_level == FAILSAFE_ACTION_REQUIRED);
  CHECK(fx.out.actuation_permit.permit_flags == 0u);
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.state_snapshot.failsafe_level == FAILSAFE_TERMINAL);
  CHECK(fx.out.actuation_permit.permit_flags == 0u);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_actuator_loss_terminal_and_latched(void)
{
  Fx fx;
  CHECK(fx_init(&fx)); fx_arm(&fx); fx_control_active(&fx, 93);
  fx.act_status.fault_mask = 1u; /* The fault arrives before the capability mask changes. */
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.state_snapshot.failsafe_level == FAILSAFE_TERMINAL);
  CHECK(fx.out.actuation_permit.permit_flags & PERMIT_FLAG_REVOKED);
  fx.act_status.fault_mask = 0u; fx_run(&fx, 50000);
  CHECK(fx.out.state_snapshot.failsafe_level == FAILSAFE_TERMINAL);
  CHECK(!(fx.out.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE));
  CHECK(fx_send_simple(&fx, PRODUCT_CMD_DISARM) == COMMAND_ACK_COMPLETED);
  CHECK(fx_send(&fx, PRODUCT_CMD_ARM, fx.next_request_id++, PRODUCT_OPERATION_VALID_PREARM_EPOCH,
                0, 0, 0, fx.out.prearm_result.check_epoch, 0.0f) == COMMAND_ACK_REJECTED);
  CHECK(fx.out.state_snapshot.arm_state == ARM_STATE_DISARMED);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_contact_candidate_does_not_authorize_disarm(void)
{
  Fx fx;
  CHECK(fx_init(&fx)); fx_arm(&fx); fx_control_active(&fx, 94);
  fx.nav.core.velocity_ned_mps[0] = 0.25f;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.landed_state.state == LANDED_STATE_UNKNOWN);
  CHECK((fx.out.landed_state.evidence_mask & LANDED_EVIDENCE_GROUND_CONTACT) != 0u);
  CHECK(fx.out.landed_state.stable_duration_ms == 0u);
  CHECK(fx_send_simple(&fx, PRODUCT_CMD_DISARM) == COMMAND_ACK_REJECTED);
  CHECK(fx.out.state_snapshot.arm_state == ARM_STATE_ARMED);

  /* Once the measured motion is stable for the full dwell, normal DISARM is
   * permitted; the contact hint itself never bypasses this authorization. */
  fx.nav.core.velocity_ned_mps[0] = 0.0f;
  fx_run(&fx, 2600000u);
  CHECK(fx.out.landed_state.state == LANDED_STATE_LANDED);
  CHECK((fx.out.landed_state.evidence_mask &
         (LANDED_EVIDENCE_GROUND_CONTACT | LANDED_EVIDENCE_VERTICAL_STABLE)) ==
        (LANDED_EVIDENCE_GROUND_CONTACT | LANDED_EVIDENCE_VERTICAL_STABLE));
  CHECK(fx_send_simple(&fx, PRODUCT_CMD_DISARM) == COMMAND_ACK_COMPLETED);
  CHECK(fx.out.state_snapshot.arm_state == ARM_STATE_DISARMED);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_contact_candidate_requires_valid_navigation(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx.omit_nav = 1u; /* Null navigation pointer with the presence bit cleared. */
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.landed_state.state == LANDED_STATE_UNKNOWN);
  CHECK((fx.out.landed_state.evidence_mask & LANDED_EVIDENCE_GROUND_CONTACT) == 0u);
  CHECK(fx.out.state_snapshot.arm_state == ARM_STATE_DISARMED);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_lost_contact_evidence_clears_landed(void)
{
  Fx fx;
  CHECK(fx_init(&fx)); fx_arm(&fx); fx_control_active(&fx, 94);
  fx.nav.core.velocity_ned_mps[0] = 0.25f;
  CHECK(fx_send_simple(&fx, PRODUCT_CMD_DISARM) == COMMAND_ACK_REJECTED);
  CHECK(fx.out.landed_state.state == LANDED_STATE_UNKNOWN);
  fx.nav.core.velocity_ned_mps[0] = 0.0f;
  for (unsigned i = 0; i < 2200000u / STEP_US; ++i) {
    fx.nav.core.acceleration_ned_mps2[2] = (i % 2u) ? 3.0f : -3.0f;
    CHECK(fx_step(&fx) == STATUS_OK);
  }
  CHECK(fx.out.landed_state.state == LANDED_STATE_LANDED);
  fx.nav.core.acceleration_ned_mps2[2] = 7.0f;
  CHECK(fx_send_simple(&fx, PRODUCT_CMD_DISARM) == COMMAND_ACK_REJECTED);
  CHECK(fx.out.landed_state.state == LANDED_STATE_UNKNOWN);
  fx.nav.core.acceleration_ned_mps2[2] = 0.0f;
  fx_run(&fx, 2350000u);
  CHECK(fx.out.landed_state.state == LANDED_STATE_LANDED);
  fx.now += 110000u;
  CHECK(fx_send_simple(&fx, PRODUCT_CMD_DISARM) == COMMAND_ACK_REJECTED);
  CHECK(fx.out.landed_state.state == LANDED_STATE_UNKNOWN);
  fx_run(&fx, 2350000u);
  CHECK(fx.out.landed_state.state == LANDED_STATE_LANDED);
  fx.ext.ground_contact_state = GROUND_CONTACT_NO_CONTACT;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.landed_state.state == LANDED_STATE_UNKNOWN);
  CHECK(fx_send_simple(&fx, PRODUCT_CMD_DISARM) == COMMAND_ACK_REJECTED);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_mission_arrival_requires_height_and_vertical_speed(void)
{
  Fx fx;
  CHECK(fx_init(&fx)); fx_make_mission(&fx, 2); fx_commit(&fx);
  fx_arm(&fx); fx_control_active(&fx, 95); fx_airborne(&fx); fx_start_mission(&fx);
  fx.nav.core.position_ned_m[0] = 10; fx.nav.core.position_ned_m[2] = 0;
  fx_run(&fx, 300000); CHECK(fx.out.mission_status.current_item_id == 1);
  fx.nav.core.position_ned_m[2] = -5; fx.nav.core.velocity_ned_mps[2] = 2;
  fx_run(&fx, 300000); CHECK(fx.out.mission_status.current_item_id == 1);
  fx.nav.core.velocity_ned_mps[2] = 0;
  fx_run(&fx, 300000); CHECK(fx.out.mission_status.current_item_id == 2);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_takeoff_publishes_explicit_home_relative_target(void)
{
  Fx fx;
  CHECK(fx_init(&fx)); fx.nav.core.position_ned_m[2] = 10;
  fx_arm(&fx); fx_control_active(&fx, 96);
  CHECK(fx_send(&fx, PRODUCT_CMD_TAKEOFF, fx.next_request_id++,
      PRODUCT_OPERATION_VALID_ALTITUDE_ABOVE_HOME, 0, 0, 0, 0, 2) == COMMAND_ACK_ACCEPTED);
  CHECK(fx.out.directive.payload.flight_phase == FLIGHT_PHASE_TAKEOFF);
  CHECK(fx.out.directive.operation_code == PRODUCT_OP_HOLD);
  CHECK(fx.out.directive.payload.target_valid_mask & PRODUCT_DIRECTIVE_TARGET_POSITION);
  CHECK(fx.out.directive.payload.target_position_ned_m[2] == 8);
  CHECK(fms_policy_stop() == STATUS_OK);
}

#ifdef M4_M5_INTEGRATION
/* These tests pass production snapshots directly between the two real Ports.
 * No manufactured GuidanceStatus, feasibility or Permit is used. */
static Status real_guidance_step(Fx *fx, GuidanceOutput *out)
{
  GuidanceInput input;
  memset(&input, 0, sizeof(input));
  input.navigation = &fx->nav; input.navigation_capability = &fx->cap;
  input.directive = &fx->out.directive; input.safety_envelope = &fx->out.safety_envelope;
  input.authority_status = &fx->out.authority_status;
  input.authorized_mission = &fx->out.authorized_mission;
  input.authorized_operator_input = &fx->out.authorized_operator_input;
  input.active_permit = &fx->out.actuation_permit;
  input.landed_state = &fx->out.landed_state;
  input.geofences = fx->out.geofences; input.geofence_count = fx->out.geofence_count;
  input.control_time_us = fx->nav.header.timestamp_sample_us;
  input.evaluation_time_us = fx->out.directive.header.timestamp_publish_us;
  return guidance_update(&input, out);
}

static void deliver_guidance(Fx *fx, const GuidanceOutput *out)
{
  FmsInputView input;
  fx_fill_headers(fx); fx_view(fx, &input);
  input.guidance_status = &out->guidance_status;
  input.guidance_feasibility = &out->feasibility;
  input.presence_mask |= FMS_INPUT_HAS_GUIDANCE | FMS_INPUT_HAS_GUIDANCE_STATUS;
  CHECK(fms_policy_update(&input, &fx->out) == STATUS_OK);
  fx->now += STEP_US;
}

static void test_real_m4_m5_ready_permit_active_takeoff(void)
{
  Fx fx;
  ProductGuidanceConfig cfg;
  GuidanceOutput g;
  uint32_t permit_epoch;
  unsigned i;
  CHECK(fx_init(&fx)); guidance_default_config(&cfg);
  CHECK(guidance_init(&cfg) == STATUS_OK); fx_arm(&fx);
  CHECK(real_guidance_step(&fx, &g) == STATUS_OK);
  CHECK(g.guidance_status.guidance_state == GUIDANCE_STATE_READY);
  CHECK(g.feasibility.feasible == 1);
  deliver_guidance(&fx, &g);
  CHECK(fx.out.actuation_permit.permit_flags == PERMIT_SCOPE_CONTROL_ACTIVE);
  permit_epoch = fx.out.actuation_permit.permit_epoch;
  CHECK(real_guidance_step(&fx, &g) == STATUS_OK);
  CHECK(g.guidance_status.guidance_state == GUIDANCE_STATE_ACTIVE);
  for (i = 0; i < 5; ++i) { deliver_guidance(&fx, &g); CHECK(real_guidance_step(&fx, &g) == STATUS_OK); }
  CHECK(fx.out.actuation_permit.permit_epoch == permit_epoch);
  CHECK(fx_send(&fx, PRODUCT_CMD_TAKEOFF, fx.next_request_id++,
      PRODUCT_OPERATION_VALID_ALTITUDE_ABOVE_HOME, 0, 0, 0, 0, 2) == COMMAND_ACK_ACCEPTED);
  CHECK(real_guidance_step(&fx, &g) == STATUS_OK);
  CHECK(g.guidance_status.guidance_state == GUIDANCE_STATE_READY);
  CHECK(g.reference.points[23].point.position_ned_m[2] < 0);
  deliver_guidance(&fx, &g);
  CHECK(fx.out.actuation_permit.permit_flags == PERMIT_SCOPE_CONTROL_ACTIVE);
  CHECK(real_guidance_step(&fx, &g) == STATUS_OK);
  CHECK(g.guidance_status.guidance_state == GUIDANCE_STATE_ACTIVE);
  CHECK(guidance_stop() == STATUS_OK); CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_real_mission_and_vertical_only_fallback(void)
{
  Fx fx;
  ProductGuidanceConfig cfg;
  GuidanceOutput g;
  uint64_t token;
  CHECK(fx_init(&fx)); fx_make_mission(&fx, 2);
  fx.cand.mission.waypoints[0].position_ned_m[0] = 3;
  fx.cand.mission.waypoints[0].position_ned_m[2] = -1;
  fx.cand.mission.waypoints[1].position_ned_m[0] = 4;
  fx.cand.mission.waypoints[1].position_ned_m[2] = -1;
  fx_commit(&fx); guidance_default_config(&cfg);
  CHECK(guidance_init(&cfg) == STATUS_OK); fx_arm(&fx);
  CHECK(real_guidance_step(&fx, &g) == STATUS_OK); deliver_guidance(&fx, &g);
  fx_airborne(&fx); fx.nav.core.position_ned_m[2] = -1; fx_start_mission(&fx);
  token = fx.out.authorized_mission.execution_token;
  CHECK(real_guidance_step(&fx, &g) == STATUS_OK);
  CHECK(g.guidance_status.guidance_state == GUIDANCE_STATE_READY);
  CHECK(g.reference.execution_token == token); CHECK(g.feasibility.feasible);
  deliver_guidance(&fx, &g); CHECK(real_guidance_step(&fx, &g) == STATUS_OK);
  CHECK(g.guidance_status.guidance_state == GUIDANCE_STATE_ACTIVE);
  fx.cap.capability_mask &= ~NAV_VALID_HORIZONTAL_POSITION;
  fx.nav.core.valid_mask &= ~(NAV_VALID_HORIZONTAL_POSITION | NAV_VALID_HORIZONTAL_VELOCITY);
  fx.nav.core.position_ned_m[0] = fx.nav.core.velocity_ned_mps[0] = NAN;
  deliver_guidance(&fx, &g);
  CHECK(fx.out.state_snapshot.flight_mode == FLIGHT_MODE_AUTO_LAND);
  CHECK(fx.out.mission_status.mission_state == MISSION_STATE_PAUSED);
  CHECK(real_guidance_step(&fx, &g) == STATUS_OK);
  CHECK(g.guidance_status.reference_kind == GUIDANCE_REFERENCE_ALTITUDE_AUTOMATIC);
  CHECK(g.pilot_control_reference.reference_kind == PILOT_REFERENCE_ALTITUDE_ATTITUDE);
  CHECK(g.reference.header.validity_flags == 0); CHECK(g.feasibility.feasible);
  fx.omit_control = 1u; /* No new M6 evidence while the old permit is revoked. */
  deliver_guidance(&fx, &g); CHECK(real_guidance_step(&fx, &g) == STATUS_OK);
  CHECK(g.guidance_status.guidance_state == GUIDANCE_STATE_ACTIVE);
  CHECK(fx.out.state_snapshot.failsafe_level != FAILSAFE_TERMINAL);
  CHECK(fx.out.actuation_permit.permit_flags == PERMIT_SCOPE_CONTROL_ACTIVE);
  CHECK(guidance_stop() == STATUS_OK); CHECK(fms_policy_stop() == STATUS_OK);
}
#endif

/* ------------------------------------------------------------------ */
/* FMAT-0008: ARM initial control reference race                      */
/* ------------------------------------------------------------------ */

/* ARM hands control over before the first Control reference exists, so the
 * first Control capability frames legitimately report a partial DOF mask.
 * Landing evidence is deliberately left UNKNOWN to mirror the Host MIL, where
 * the ground stop cannot rescue the ARM frame. */
static void fx_arm_with_partial_control(Fx *fx)
{
  fx->ext.ground_contact_state = GROUND_CONTACT_UNKNOWN;
  fx_run(fx, 4300000u);
  CHECK(fx->out.prearm_result.state == PREARM_READY);
  fx->control.controllable_dof_mask = 0x0u;
  CHECK(fx_send(fx, PRODUCT_CMD_ARM, fx->next_request_id++,
                PRODUCT_OPERATION_VALID_PREARM_EPOCH, 0u, 0u, 0u,
                fx->out.prearm_result.check_epoch, 0.0f) == COMMAND_ACK_COMPLETED);
  CHECK(fx->out.state_snapshot.arm_state == ARM_STATE_ARMED);
}

static void test_arm_first_reference_partial_dof_no_terminal(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx_arm_with_partial_control(&fx);
  /* The ARM frame itself must not latch TERMINAL on a zero DOF mask. */
  CHECK(fx.out.state_snapshot.failsafe_level != FAILSAFE_TERMINAL);
  CHECK((fx.out.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE) == 0u);
  fx_run(&fx, 50000u); /* Still inside the absolute first-reference window. */
  CHECK(fx.out.state_snapshot.failsafe_level != FAILSAFE_TERMINAL);
  CHECK((fx.out.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE) == 0u);
  CHECK(fx.out.state_snapshot.arm_state == ARM_STATE_ARMED);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_arm_first_reference_in_window_signs_permit(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx_arm_with_partial_control(&fx);
  fx_run(&fx, 20000u);
  /* The first real Control reference lands inside the configured window. */
  fx.control.controllable_dof_mask = 0x0Fu;
  fx_control_active(&fx, 71u);
  CHECK(fx.out.state_snapshot.failsafe_level != FAILSAFE_TERMINAL);
  CHECK(fx.out.state_snapshot.arm_state == ARM_STATE_ARMED);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_arm_first_reference_window_expiry_inhibits_outputs(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx_arm_with_partial_control(&fx);
  fx_run(&fx, 150000u); /* Past the absolute 100 ms window. */
  CHECK(fx.out.state_snapshot.failsafe_level >= FAILSAFE_ACTION_REQUIRED);
  CHECK((fx.out.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE) == 0u);
  /* A reference that arrives after the deadline may not resurrect a permit. */
  fx.control.controllable_dof_mask = 0x0Fu;
  fx_make_guidance(&fx, 72u, GUIDANCE_STATE_READY, GUIDANCE_REFERENCE_TRAJECTORY);
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK((fx.out.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE) == 0u);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_established_control_link_loss_keeps_terminal(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx_arm(&fx);
  fx_control_active(&fx, 73u);
  fx_airborne(&fx);
  /* Once the link is up a partial DOF mask is a real capability loss, not
   * initialisation: the window must never soften the established rule. */
  fx.control.controllable_dof_mask = 0x03u;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.state_snapshot.failsafe_level == FAILSAFE_TERMINAL);
  CHECK((fx.out.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE) == 0u);
  fx_run(&fx, 50000u);
  CHECK(fx.out.state_snapshot.failsafe_level == FAILSAFE_TERMINAL);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_expired_reference_cannot_sign_in_window(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx_arm(&fx);
  fx_make_guidance(&fx, 74u, GUIDANCE_STATE_READY, GUIDANCE_REFERENCE_TRAJECTORY);
  fx.guidance_ttl_us = 0u; /* Expires at the same instant it is published. */
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK((fx.out.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE) == 0u);
  /* Same reference, valid TTL: the rejection was freshness, not hidden state. */
  fx.guidance_ttl_us = 100000u;
  fx_control_active(&fx, 74u);
  CHECK(fms_policy_stop() == STATUS_OK);
}

static void test_permit_rebind_keeps_revoked_watermark(void)
{
  Fx fx;
  uint32_t active_epoch, revoked_epoch;

  CHECK(fx_init(&fx));
  fx_arm(&fx);
  fx_control_active(&fx, 75u);
  active_epoch = fx.out.actuation_permit.permit_epoch;
  CHECK(active_epoch != 0u);

  fx.feas.feasible = 0u;
  fx.feas.reason_code = 2u;
  fx.guidance.guidance_state = GUIDANCE_STATE_REJECTED;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.actuation_permit.permit_flags & PERMIT_FLAG_REVOKED);
  revoked_epoch = fx.out.actuation_permit.permit_epoch;
  CHECK(revoked_epoch > active_epoch);
  CHECK(fx_step(&fx) == STATUS_OK); /* Revocation is published exactly once. */

  /* Rebinding with a fresh reference must move forward, never reuse a watermark. */
  fx_control_active(&fx, 76u);
  CHECK((fx.out.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE) != 0u);
  CHECK(fx.out.actuation_permit.permit_epoch > revoked_epoch);
  CHECK(fms_policy_stop() == STATUS_OK);
}

int main(void)
{
  struct {
    const char *name;
    void (*fn)(void);
  } tests[] = {
    {"arm_requires_fresh_prearm_epoch", test_arm_requires_fresh_prearm_epoch},
    {"arm_rechecks_current_inputs", test_arm_rechecks_current_inputs},
    {"idle_lease_renewal_bounded", test_idle_lease_renewal_bounded},
    {"control_active_ready_handshake", test_control_active_ready_handshake},
    {"permit_revocation_watermark", test_permit_revocation_is_immediate_and_watermarked},
    {"guidance_epoch_mismatch", test_guidance_epoch_mismatch_has_no_permit},
    {"mission_commit_validation", test_mission_commit_validates_source_and_geometry},
    {"mission_start_requires_arm", test_mission_start_requires_arm},
    {"two_waypoint_mission", test_two_waypoint_mission_token_and_completion},
    {"pause_resume", test_pause_resume_boundaries},
    {"abort_withdraws_token", test_abort_withdraws_token},
    {"disarm_in_air_rejected", test_disarm_rejected_in_air},
    {"pilot_takeover_latch", test_pilot_takeover_latches_and_pauses},
    {"rc_loss_return", test_rc_loss_forces_return_and_pauses_mission},
    {"rejected_return_falls_back_to_land", test_rejected_return_falls_back_to_land},
    {"home_renewal_requires_position", test_home_renewal_requires_position_evidence},
    {"operator_home_survives_arm", test_operator_home_survives_arm},
    {"clear_empty_mission_requires_disarm", test_clear_empty_mission_requires_disarm},
    {"return_pauses_running_mission", test_return_pauses_running_mission},
    {"post_mission_return_starts_after_control_binding", test_post_mission_return_starts_after_control_binding},
    {"post_mission_return_missing_control_cancels_safely", test_post_mission_return_missing_control_cancels_safely},
    {"duplicate_request_replay", test_duplicate_request_replays_ack},
    {"dedup_full_and_expiry", test_dedup_full_and_expiry},
    {"geofence_required", test_geofence_required_rejects_outside_waypoint},
    {"mission_epoch_binding", test_mission_epoch_required_and_not_stale},
    {"first_reference_window", test_first_reference_window_expires_to_ground_stop},
    {"nav_loss_recovery_dwell", test_nav_loss_and_recovery_dwell},
    {"critical_energy_lands", test_critical_energy_forces_land},
    {"forced_land_cancels_return", test_forced_land_cancels_return},
    {"safety_handoff_missing_control_expires", test_safety_handoff_missing_control_expires},
    {"actuator_loss_terminal", test_actuator_loss_terminal_and_latched},
    {"contact_candidate_not_disarm_authority", test_contact_candidate_does_not_authorize_disarm},
    {"contact_candidate_requires_valid_navigation", test_contact_candidate_requires_valid_navigation},
    {"lost_contact_clears_landed", test_lost_contact_evidence_clears_landed},
    {"mission_arrival_3d", test_mission_arrival_requires_height_and_vertical_speed},
    {"takeoff_explicit_target", test_takeoff_publishes_explicit_home_relative_target},
    {"arm_first_reference_partial_dof_no_terminal", test_arm_first_reference_partial_dof_no_terminal},
    {"arm_first_reference_in_window_signs_permit", test_arm_first_reference_in_window_signs_permit},
    {"arm_first_reference_window_expiry_inhibits_outputs", test_arm_first_reference_window_expiry_inhibits_outputs},
    {"established_link_loss_keeps_terminal", test_established_control_link_loss_keeps_terminal},
    {"expired_reference_cannot_sign_in_window", test_expired_reference_cannot_sign_in_window},
    {"permit_rebind_keeps_revoked_watermark", test_permit_rebind_keeps_revoked_watermark},
#ifdef M4_M5_INTEGRATION
    {"real_m4_m5_handshake_takeoff", test_real_m4_m5_ready_permit_active_takeoff},
    {"real_mission_vertical_fallback", test_real_mission_and_vertical_only_fallback},
#endif
  };
  size_t i;

  for (i = 0u; i < sizeof(tests) / sizeof(tests[0]); ++i) {
    int before = g_failures;
    printf("RUN  %s\n", tests[i].name);
    tests[i].fn();
    printf("%-4s %s\n", (g_failures == before) ? "PASS" : "FAIL", tests[i].name);
  }
  printf("\nchecks=%d failures=%d\n", g_checks, g_failures);
  return (g_failures == 0) ? 0 : 1;
}
