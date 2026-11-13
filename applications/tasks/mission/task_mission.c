/* Planning task: FMS/guidance owner.
 * Wakes on the planning event (validated origin, GNSS fix) or after 20 ms. */
#include <rtthread.h>
#include "task_config.h"
#include "runtime_internal.h"
#include "fms_interface.h"
#include <math.h>
#include <string.h>

static ControlBundle plan_bundle;
static FmsOutputView fms_output;
static MissionUploadCandidate plan_mission;
#ifdef FLIGHT_HOST_SIL
extern uint8_t flight_sil_ground_contact(void);
#endif

static void task_mission_step(void) {
  FmsInputView input;
  GuidanceInput gin;
  OperationRequest requests[PRODUCT_MAX_PENDING_OPERATION_REQUESTS];
  OperatorInput op;
  NavigationCapability nav_cap;
  ControlCapability control;
  uint32_t count;
  uint8_t have_op, have_mission;
  ContractHeader origin;
  uint32_t next_origin;
  uint64_t now;
#ifdef FLIGHT_HOST_SIL
  CommunicationLinkStatus sim_rc = {0};
#endif
  /* Wake promptly for the first validated GNSS fix: a 20 ms periodic
   * phase can otherwise consistently miss its 20 ms measurement TTL. */

  now=runtime_now_us();
  runtime_data_mark_heartbeat(RUNTIME_OWNER_MISSION,now);
  memset(&input,0,sizeof(input));
  runtime_data_lock(RT_WAITING_FOREVER);
  plan_bundle.nav = runtime_data.nav;
  nav_cap = runtime_data.nav_cap;
  plan_bundle.actuator=runtime_data.actuator; plan_bundle.actuator_cap=runtime_data.actuator_cap;
  plan_bundle.receipt=runtime_data.receipt; control=runtime_data.control_cap;
  origin=runtime_data.origin_candidate; next_origin=runtime_data.next_origin_id;
  count=runtime_data_take_requests(requests);
  op=runtime_data.operator_input; have_op=runtime_data.have_operator;
  have_mission=runtime_data.have_mission; if(have_mission) { plan_mission=runtime_data.mission; runtime_data.have_mission=0; }
  runtime_data_unlock();
  input.navigation = &plan_bundle.nav;
  input.navigation_capability = &nav_cap;
  input.actuator_status=&plan_bundle.actuator; input.actuator_capability=&plan_bundle.actuator_cap;
  /* FMS consumes the previous guidance result before M5 refreshes it. */
  input.control_capability = &control;
  input.guidance_status = &plan_bundle.guidance.guidance_status;
  input.guidance_feasibility = &plan_bundle.guidance.feasibility;
  input.pending_requests=requests; input.pending_request_count=(uint16_t)count;
  input.operator_input=have_op ? &op : 0; input.mission_candidate=have_mission ? &plan_mission : 0;
  input.presence_mask=FMS_INPUT_HAS_NAV|FMS_INPUT_HAS_NAV_CAPABILITY|FMS_INPUT_HAS_ACTUATOR|
    FMS_INPUT_HAS_CONTROL|FMS_INPUT_HAS_GUIDANCE|FMS_INPUT_HAS_GUIDANCE_STATUS;
#ifdef FLIGHT_HOST_SIL
  if (have_op) {
    sim_rc.header = op.header;
    sim_rc.header.message_type = MSG_COMMUNICATION_LINK_STATUS;
    sim_rc.link_kind = LINK_KIND_RC;
    sim_rc.link_state = LINK_STATE_HEALTHY;
    sim_rc.last_valid_frame_us = op.header.timestamp_sample_us;
    input.link_statuses = &sim_rc;
    input.link_status_count = 1;
    input.presence_mask |= FMS_INPUT_HAS_LINKS;
  }
  /* SIL profile assumption: the bench supplies the energy/battery evidence that
   * the flight composition does not route into FMS yet. The values are the
   * fixture's, not a measurement. */
  memset(&input.product_extension,0,sizeof(input.product_extension));
  input.product_extension.header.schema_major=CONTRACT_SCHEMA_MAJOR;
  input.product_extension.header.schema_minor=CONTRACT_SCHEMA_MINOR;
  input.product_extension.header.message_type=MSG_PRODUCT_FMS_INPUT_EXTENSION;
  input.product_extension.header.source_id=SOURCE_COMPANION;
  input.product_extension.header.profile_id=PRODUCT_PROFILE_ID;
  input.product_extension.header.boot_id=plan_bundle.nav.header.boot_id;
  input.product_extension.header.time_epoch=plan_bundle.nav.header.time_epoch;
  input.product_extension.header.validity_flags=VALIDITY_PAYLOAD_VALID|VALIDITY_TIME_VALID;
  input.product_extension.header.timestamp_sample_us=now;
  input.product_extension.header.timestamp_publish_us=now;
  input.product_extension.header.valid_until_us=now+200000u;
  input.product_extension.header.timestamp_quality=TIMESTAMP_QUALITY_HARDWARE_CAPTURED;
  input.product_extension.header.timestamp_flags=TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED;
  input.product_extension.header.event_time_uncertainty_us=50u;
  input.product_extension.valid_mask=PRODUCT_FMS_EXT_ENERGY_STATE|PRODUCT_FMS_EXT_BATTERY_FRACTION|
    PRODUCT_FMS_EXT_GROUND_CONTACT;
  input.product_extension.ground_contact_state=flight_sil_ground_contact();
  input.product_extension.energy_state=ENERGY_NORMAL;
  input.product_extension.battery_remaining_fraction=0.9f;
  input.presence_mask|=FMS_INPUT_HAS_PRODUCT_EXTENSION;
#endif
  if(count) input.presence_mask|=FMS_INPUT_HAS_REQUEST;
  if(have_op) input.presence_mask|=FMS_INPUT_HAS_OPERATOR;
  if(have_mission) input.presence_mask|=FMS_INPUT_HAS_MISSION_CANDIDATE;
  input.evaluation_time_us=now;
  runtime_stats.planning_status=fms_policy_update(&input,&fms_output); runtime_stats.planning_updates++;
  /* Revoke/renew directly before doing any more planning work. */
  if(!runtime_stats.planning_status) (void)actuator_update_permit(&fms_output.actuation_permit,runtime_now_us());
  /* Translate only a validated M4 ground-disarm decision plus its matching
   * revoked permit. Keep the short-lived certificate in the bundle long enough
   * for the 1 ms actuator owner to consume it; never synthesize it while airborne. */
  {
    GroundStopCertificate certificate;
    const uint64_t certificate_now = runtime_now_us();
    if (!runtime_stats.planning_status &&
        fms_build_ground_stop_certificate(&fms_output, certificate_now, &certificate) == STATUS_OK) {
      plan_bundle.ground_stop_certificate = certificate;
    } else if (plan_bundle.ground_stop_certificate.certificate_expires_at_us <= certificate_now) {
      memset(&plan_bundle.ground_stop_certificate, 0, sizeof(plan_bundle.ground_stop_certificate));
    }
  }
  if(!runtime_stats.planning_status && next_origin) {
    EstimatorResetRequest reset;
    if(!fms_authorize_origin_reset(&origin,next_origin,&reset)) {
      runtime_data_lock(RT_WAITING_FOREVER);
      if(!runtime_data.have_origin_reset) { runtime_data.origin_reset=reset; runtime_data.have_origin_reset=1; }
      runtime_data_unlock(); (void)rt_sem_release(&runtime_data.imu_event);
    }
  }
  memset(&gin,0,sizeof(gin));
  gin.navigation = &plan_bundle.nav;
  gin.navigation_capability = &nav_cap;
  gin.directive=&fms_output.directive; gin.safety_envelope=&fms_output.safety_envelope;
  gin.landed_state=&fms_output.landed_state; gin.authorized_operator_input=&fms_output.authorized_operator_input;
  gin.authority_status=&fms_output.authority_status; gin.authorized_mission=&fms_output.authorized_mission;
  gin.active_permit=&fms_output.actuation_permit; gin.geofences=fms_output.geofences;
  gin.geofence_count=fms_output.geofence_count; gin.control_time_us=plan_bundle.nav.header.timestamp_sample_us;
  gin.evaluation_time_us=runtime_now_us();
  if (!runtime_stats.planning_status) {
    (void)guidance_update(&gin, &plan_bundle.guidance);
  } else {
    memset(&plan_bundle.guidance, 0, sizeof(plan_bundle.guidance));
  }
  plan_bundle.directive = fms_output.directive;
  plan_bundle.envelope=fms_output.safety_envelope; plan_bundle.authority=fms_output.authority_status;
  plan_bundle.permit=fms_output.actuation_permit;
  {
    OperatingPoint point; memset(&point,0,sizeof(point));
    memcpy(point.state.state,plan_bundle.nav.core.position_ned_m,3*sizeof(float));
    memcpy(point.state.state+3,plan_bundle.nav.core.velocity_ned_mps,3*sizeof(float));
    point.operating_speed_mps = hypotf(hypotf(point.state.state[3], point.state.state[4]),
                                       point.state.state[5]);
#ifdef FLIGHT_HOST_SIL
    point.operating_voltage_v = 16.0f; /* Simulation supply, no battery device. */
#endif
    if (control_model_get_view(&point,.02f,&plan_bundle.envelope,&plan_bundle.model) != STATUS_OK)
      memset(&plan_bundle.model,0,sizeof(plan_bundle.model));
  }
  runtime_data_lock(RT_WAITING_FOREVER); runtime_data.bundle=plan_bundle; runtime_data.bundle_ready=1;
  ++runtime_data.bundle_revision;
  {
    int changed=runtime_data.fms.arm_state!=fms_output.state_snapshot.arm_state ||
      runtime_data.fms.authority_epoch!=fms_output.state_snapshot.authority_epoch ||
      runtime_data.fms.directive_epoch!=fms_output.state_snapshot.directive_epoch ||
      runtime_data.fms.failsafe_level!=fms_output.state_snapshot.failsafe_level;
    runtime_data.fms=fms_output.state_snapshot; runtime_data.prearm=fms_output.prearm_result; runtime_data_unlock();
    runtime_data_set_event_epochs(&fms_output.state_snapshot);
    if(changed) runtime_data_record_event(SOURCE_M4,0,fms_output.decision_event.reason_code,
        fms_output.state_snapshot.failsafe_level ? 1 : 0,runtime_now_us());
  }
}

static uint8_t task_mission_stack[8192] __attribute__((aligned(8)));

static void task_mission_entry(void *parameter)
{
    (void)parameter;
    while (runtime_running()) {
        (void)rt_sem_take(&runtime_data.plan_event,
                          rt_tick_from_millisecond(TASK_MISSION_PERIOD_MS));
        task_mission_step();
    }
}

const TaskDescriptor task_mission = {
    "mission",
    task_mission_entry,
    task_mission_stack,
    sizeof(task_mission_stack),
    TASK_MISSION_PRIORITY,
};
