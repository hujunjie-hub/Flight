#include "task_comm.h"
#include "runtime_internal.h"
#include "systime.h"
#include "fms_contract/fms_contract.h"
#include "mavlink/mavlink_action.h"
#include "mavlink/mavlink_frame.h"
#include "mavlink/mavlink_rtcm.h"
#include "calibration/sensor_calibration_service.h"
#include "parameter_store.h"
#include "sample_acquisition/sensor_input.h"
#include "telemetry_link/telemetry_link.h"
#include <rtthread.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

enum {
  MAV_ACCEPTED = 0, MAV_DENIED = 2, MAV_UNSUPPORTED = 3,
  MAV_CMD_PREFLIGHT_CALIBRATION = 241, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246,
  MAV_CMD_USER_2 = 31011,
  MAV_CMD_SET_MESSAGE_INTERVAL = 511, MAV_CMD_REQUEST_MESSAGE = 512,
  MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520,
  BENCH_POLICY_REASON = 19, EXTENDED_STATE_DEFAULT_US = 1000000
};
/* UINT16_MAX means "no PARAM_REQUEST_LIST received yet". */
static uint16_t next_parameter = UINT16_MAX;
static uint32_t extended_state_period_us = EXTENDED_STATE_DEFAULT_US;
static uint64_t reboot_at_us;

static uint16_t gcs_read_u16(const uint8_t *p)
{
  return (uint16_t)(p[0] | (uint16_t)p[1] << 8);
}

static bool targets_vehicle(const uint8_t *p)
{
  return (p[0] == 0 || p[0] == 1) && (p[1] == 0 || p[1] == 1);
}

static void gcs_status_text(uint8_t severity, const char *text)
{
  uint8_t p[51];
  size_t length = strlen(text);
  memset(p, 0, sizeof(p));
  p[0] = severity;
  if (length > 50u)
    length = 50u;
  memcpy(p + 1, text, length);
  telemetry_link_send(STATUSTEXT, 83, p, sizeof(p));
}

static void send_parameter_by_index(unsigned index)
{
  uint8_t p[25] = {0};
  const int ordinal = parameter_store_exposed_ordinal(index);
  if (index >= parameter_store_count() || ordinal < 0)
    return;
  wire_float(p, parameter_store_value(index));
  wire_u16(p + 4, (uint16_t)parameter_store_exposed_count());
  wire_u16(p + 6, (uint16_t)ordinal);
  memcpy(p + 8, parameter_store_name(index), 16);
  p[24] = 9; /* MAV_PARAM_TYPE_REAL32 */
  telemetry_link_send(PARAM_VALUE, 220, p, sizeof(p));
}

/* MAVLink parameter indexes address the exposed tuning list, not the internal
 * store that also carries calibration values. */
static void send_parameter(unsigned ordinal)
{
  const unsigned index = parameter_store_exposed_at(ordinal);
  if (index != UINT16_MAX)
    send_parameter_by_index(index);
}

static void send_named_parameter(const uint8_t name[16])
{
  const int index = parameter_store_find_exposed((const char *)name);
  if (index >= 0)
    send_parameter_by_index((unsigned)index);
}

static void send_extended_state(void)
{
  /* VTOL and landing state are undefined without target-board evidence. */
  const uint8_t payload[2] = {0, 0};
  telemetry_link_send(EXTENDED_SYS_STATE, 130, payload, sizeof(payload));
}

static bool calibration_allowed(void)
{
  static FmsStateSnapshot fms;
  static PreArmCheckResult prearm;
  return runtime_running() && runtime_get_fms(&fms, &prearm) == STATUS_OK &&
         runtime_data_header_is_current(&fms.header, runtime_now_us()) &&
         fms.arm_state == ARM_STATE_DISARMED;
}

static void handle_preflight_calibration(const uint8_t *p, uint8_t *ack)
{
  if (!calibration_allowed()) {
    gcs_status_text(4, "Flight: calibration requires DISARMED");
    ack[2] = MAV_DENIED;
    wire_u32(ack + 4, 1u); /* MAV_RESULT_DENIED */
    return;
  }
  const uint8_t kind = mavlink_calibration_kind(p);
  if (kind == SENSOR_CAL_NONE) {
    gcs_status_text(4, "Flight: no calibration selected");
    ack[2] = MAV_DENIED;
    wire_u32(ack + 4, 1u);
    return;
  }
  char error[40];
  if (!sensor_calibration_start(kind, runtime_now_us(), error, sizeof(error))) {
    gcs_status_text(4, error);
    ack[2] = MAV_DENIED;
    wire_u32(ack + 4, 1u);
    return;
  }
  ack[2] = MAV_ACCEPTED;
  gcs_status_text(6, kind == SENSOR_CAL_ACCEL ? "ACCEL CAL started: capture 6 faces" :
                    kind == SENSOR_CAL_RC ? "RC CAL started: move the sticks" :
                    kind == SENSOR_CAL_MAG ? "MAG CAL started: rotate the vehicle" :
                    kind == SENSOR_CAL_GYRO ? "GYRO CAL started: keep still" :
                    kind == SENSOR_CAL_BARO ? "BARO CAL started" : "LEVEL CAL started");
}

static void handle_calibration_capture(const uint8_t *p, uint8_t *ack)
{
  if (!calibration_allowed()) {
    gcs_status_text(4, "Flight: calibration requires DISARMED");
    ack[2] = MAV_DENIED;
    wire_u32(ack + 4, 1u);
    return;
  }
  const float face = wire_read_float(p);
  const float action = wire_read_float(p + 4);
  const uint64_t now = runtime_now_us();
  char error[48] = "Flight: invalid calibration action";
  bool accepted = false;
  if (action == 2.0f) {
    sensor_calibration_cancel(now);
    accepted = true;
  } else if (action == 3.0f) {
    accepted = sensor_calibration_finish_rc(now, error, sizeof(error));
  } else if (action == 5.0f) {
    static uint64_t last_bind_us;
    SensorCalibrationStatus status;
    sensor_calibration_get_status(&status);
    if (status.state == SENSOR_CAL_RUNNING) {
      snprintf(error, sizeof(error), "RC bind: stop calibration first");
    } else if (last_bind_us != 0u && now - last_bind_us < 3000000u) {
      snprintf(error, sizeof(error), "RC bind: retry in 3 seconds");
    } else if (telemetry_link_request_rc_bind()) {
      last_bind_us = now;
      accepted = true;
      gcs_status_text(6, "CRSF bind request sent");
    } else {
      snprintf(error, sizeof(error), "CRSF bind: UART write failed");
    }
  } else if (action == 4.0f) {
    accepted = sensor_calibration_copy_rc_trims(now, error, sizeof(error));
    if (accepted)
      gcs_status_text(6, "RC TRIM copied");
  } else if (action == 1.0f && face >= 0.0f && face < SENSOR_CAL_ACCEL_FACES &&
             face == (float)(uint8_t)face) {
    accepted = sensor_calibration_capture_accel_face((uint8_t)face, now, error, sizeof(error));
  }
  ack[2] = accepted ? MAV_ACCEPTED : MAV_DENIED;
  if (!accepted) {
    gcs_status_text(4, error);
    wire_u32(ack + 4, 1u);
  }
}

static void publish_calibration_status(void)
{
  static uint8_t last_state, last_kind, last_faces;
  static uint32_t last_started;
  static uint64_t last_publish;
  SensorCalibrationStatus status;
  const uint64_t now = runtime_now_us();
  sensor_calibration_get_status(&status);
  if (status.state == SENSOR_CAL_IDLE)
    return;
  const bool changed = status.state != last_state || status.kind != last_kind ||
                       status.face_mask != last_faces || status.started_us != last_started;
  if (!changed && (status.state != SENSOR_CAL_RUNNING || now - last_publish < 1000000u))
    return;
  char message[48];
  snprintf(message, sizeof(message), "FCAL %u %u %u %u", (unsigned)status.kind,
           (unsigned)status.state, (unsigned)status.face_mask,
           (unsigned)(status.progress_permille / 10u));
  gcs_status_text(6, message);
  if (status.state == SENSOR_CAL_DONE || status.state == SENSOR_CAL_FAILED)
    gcs_status_text(status.state == SENSOR_CAL_DONE ? 6 : 4, status.message);
  last_state = status.state;
  last_kind = status.kind;
  last_faces = status.face_mask;
  last_started = status.started_us;
  last_publish = now;
}

static void respond_to_command(uint8_t id, const uint8_t *p, uint8_t system, uint8_t component)
{
  const uint16_t command = gcs_read_u16(p + 28);
  const float message = wire_read_float(p);
  uint8_t ack[10] = {0};
  ack[2] = MAV_UNSUPPORTED;
  if (command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES ||
      (command == MAV_CMD_REQUEST_MESSAGE && message == AUTOPILOT_VERSION)) {
    uint8_t payload[60] = {0};
    /* MAVLINK2 and PARAM_ENCODE_C_CAST. All exposed parameters are floats. */
    wire_u64(payload, (UINT64_C(1) << 13) | (UINT64_C(1) << 17));
    telemetry_link_send(AUTOPILOT_VERSION, 178, payload, sizeof(payload));
    ack[2] = MAV_ACCEPTED;
  } else if (command == MAV_CMD_REQUEST_MESSAGE && message == EXTENDED_SYS_STATE) {
    send_extended_state();
    ack[2] = MAV_ACCEPTED;
  } else if (command == MAV_CMD_PREFLIGHT_CALIBRATION && id == COMMAND_LONG) {
    handle_preflight_calibration(p, ack);
  } else if (command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN && id == COMMAND_LONG) {
    if (!calibration_allowed() || wire_read_float(p) < 0.5f) {
      gcs_status_text(4, "Flight: reboot requires DISARMED");
      ack[2] = MAV_DENIED;
      wire_u32(ack + 4, 1u);
    } else {
      ack[2] = MAV_ACCEPTED;
      gcs_status_text(6, "Flight: rebooting");
      reboot_at_us = runtime_now_us() + 500000u;
    }
  } else if (command == MAV_CMD_USER_2 && id == COMMAND_LONG) {
    handle_calibration_capture(p, ack);
  } else if (command == MAV_CMD_SET_MESSAGE_INTERVAL && message == EXTENDED_SYS_STATE) {
    const float interval = wire_read_float(p + 4);
    if (interval == -1.0f || interval == 0.0f ||
        (interval >= 200000.0f && interval <= EXTENDED_STATE_DEFAULT_US)) {
      extended_state_period_us = interval < 0.0f ? 0u :
          interval == 0.0f ? EXTENDED_STATE_DEFAULT_US : (uint32_t)interval;
      ack[2] = MAV_ACCEPTED;
    } else {
      ack[2] = MAV_DENIED;
      wire_u32(ack + 4, 1u); /* INVALID_ARGUMENT */
    }
  } else {
    MavlinkAction action;
    const MavlinkActionResult decoded = mavlink_action_decode(command, id == COMMAND_LONG, p, &action);
    if (decoded != MAVLINK_ACTION_UNKNOWN) {
      /* The unsigned radio cannot assign a trusted session to a flight request. */
      ack[2] = MAV_DENIED;
      wire_u32(ack + 4, decoded == MAVLINK_ACTION_VALID ? BENCH_POLICY_REASON : 1u);
    }
  }
  wire_u16(ack, command);
  ack[8] = system;
  ack[9] = component;
  telemetry_link_send(COMMAND_ACK, 143, ack, sizeof(ack));
}

void comm_gcs_receive(uint8_t id, const uint8_t *p, uint8_t system, uint8_t component)
{
#if defined(BSP_USING_FLIGHT_DRIVERS) && !defined(FLIGHT_SIM_LINK)
  if (id == MAVLINK_GPS_RTCM_DATA) {
    static MavlinkRtcm rtcm;
    FmsStateSnapshot fms;
    PreArmCheckResult prearm;
    const uint64_t now = runtime_now_us();
    /* Corrections on this unsigned radio are only available in bench policy. */
    if (runtime_bench_inhibit_active() && runtime_get_fms(&fms, &prearm) == STATUS_OK &&
        runtime_data_header_is_current(&fms.header, now) &&
        fms.arm_state == ARM_STATE_DISARMED)
      mavlink_rtcm_receive(&rtcm, p, (uint16_t)(system << 8 | component), now,
                           sensor_gnss_queue_rtcm);
    return;
  }
#endif
  if (id == PARAM_REQUEST_LIST && targets_vehicle(p)) {
    next_parameter = 0;
  } else if (id == PARAM_REQUEST_READ && targets_vehicle(p + 2)) {
    const uint16_t index = gcs_read_u16(p);
    if (index < parameter_store_exposed_count())
      send_parameter(index);
    else if (index == UINT16_MAX)
      send_named_parameter(p + 4);
  } else if (id == PARAM_SET && p[4] == 1 && targets_vehicle(p + 4)) {
    const float value = wire_read_float(p);
    const int index = parameter_store_find_exposed((const char *)(p + 6));
    if (index < 0) {
      gcs_status_text(4, "Flight: unknown parameter");
      return;
    }
    if (p[22] != 9 || !isfinite(value)) {
      gcs_status_text(4, "Flight: parameter type must be REAL32");
    } else if (!calibration_allowed() || !parameter_store_set((unsigned)index, value)) {
      gcs_status_text(4, "Flight: parameter rejected (range or state)");
    }
    /* QGC only commits a value after seeing the echoed PARAM_VALUE. */
    send_parameter_by_index((unsigned)index);
  } else if (id == MISSION_REQUEST_LIST && targets_vehicle(p)) {
    const uint8_t reply[5] = {0, 0, system, component, p[2]};
    telemetry_link_send(MISSION_COUNT, 221, reply, sizeof(reply));
  } else if ((id == MISSION_COUNT && p[2] == 1 && targets_vehicle(p + 2)) ||
             (id == MISSION_CLEAR_ALL && p[0] == 1 && targets_vehicle(p))) {
    const uint8_t reply[4] = {system, component, 14, p[id == MISSION_COUNT ? 4 : 2]};
    telemetry_link_send(MISSION_ACK, 153, reply, sizeof(reply)); /* MAV_MISSION_DENIED */
  } else if ((id == COMMAND_LONG || id == COMMAND_INT) && p[30] == 1 &&
             targets_vehicle(p + 30)) {
    respond_to_command(id, p, system, component);
  }
}

/* Enforce the released ABI unless a simulation build explicitly opts out. */
#ifdef FLIGHT_SKIP_ABI_LOCK
#if !defined(FLIGHT_HOST_SIL) && !defined(FLIGHT_SIM_LINK)
#error "FLIGHT_SKIP_ABI_LOCK is only supported in simulation builds"
#endif
#define FLIGHT_TELEMETRY_ABI_LOCK 0
#else
#define FLIGHT_TELEMETRY_ABI_LOCK 1
#include "product_contract/abi_lock.h"

CONTRACT_STATIC_ASSERT(ABI_LOCK_SCHEMA_MAJOR == CONTRACT_SCHEMA_MAJOR, "telemetry ABI schema major");
CONTRACT_STATIC_ASSERT(ABI_LOCK_SCHEMA_MINOR == CONTRACT_SCHEMA_MINOR, "telemetry ABI schema minor");
CONTRACT_STATIC_ASSERT(ABI_LOCK_PORT_API_REVISION == PORT_API_REVISION, "telemetry ABI Port revision");
CONTRACT_STATIC_ASSERT(ABI_LOCK_PRODUCT_CONTRACT_REVISION == PRODUCT_CONTRACT_REVISION,
                       "telemetry ABI Product revision");
#endif

static void fill_telemetry_header(uint8_t *p,uint64_t now,uint64_t boot) {
  memset(p,0,128); wire_u64(p,now); wire_u64(p+8,boot); wire_u32(p+16,PRODUCT_PROFILE_ID);
}
/* One extra diagnostic per status period bounds UART load and queue burst.
 * Unknown hardware metrics keep their validity bits clear. */
static void publish_rotating_diagnostic(uint64_t now,uint64_t boot) {
  static unsigned slot;
  static ControlCapability cap;
  static ControlHealth health;
  static ActuatorDemand demand;
  uint8_t p[128]; RuntimeStats stats;
  fill_telemetry_header(p,now,boot);runtime_get_stats(&stats);
  if(slot<4) {
    RuntimeSensorHealth h;
    if(!runtime_get_sensor_health(slot,&h)) {
      wire_u32(p+FCU_SENSOR_HEALTH_SENSOR_INSTANCE_ID,h.sensor_instance_id);
      wire_u32(p+FCU_SENSOR_HEALTH_POLICY_EPOCH,h.policy_epoch);
      wire_u32(p+FCU_SENSOR_HEALTH_REASON_CODE,h.reason_code);
      wire_u32(p+FCU_SENSOR_HEALTH_GAP_COUNT,h.gap_count);
      wire_u32(p+FCU_SENSOR_HEALTH_REJECT_COUNT,h.reject_count);
      wire_u16(p+FCU_SENSOR_HEALTH_HEALTH_STATE,h.health_state);
      /* action remains NONE: this message grants no command or authority. */
      telemetry_link_send(FCU_SENSOR_HEALTH_ID,FCU_SENSOR_HEALTH_CRC,p,FCU_SENSOR_HEALTH_LEN);
    }
  } else if(slot==4) {
    uint32_t reference=0;
    (void)runtime_get_control(&cap,&health,&demand,&reference);
    wire_u32(p+FCU_CONTROL_STATUS_CONTROL_SEQUENCE,demand.control_sequence);
    wire_u32(p+FCU_CONTROL_STATUS_REFERENCE_ID,reference);
    wire_u32(p+FCU_CONTROL_STATUS_REFERENCE_EPOCH,demand.reference_epoch);
    if(now<cap.header.valid_until_us) {
      wire_u32(p+FCU_CONTROL_STATUS_CONTROLLABLE_DOF_MASK,cap.controllable_dof_mask);
      wire_u32(p+FCU_CONTROL_STATUS_DEGRADED_DOF_MASK,cap.degraded_dof_mask);
    }
    wire_u32(p+FCU_CONTROL_STATUS_REASON_CODE,cap.reason_code);
    wire_u16(p+FCU_CONTROL_STATUS_SOLVER_STATUS,(uint16_t)health.solver_status);
    telemetry_link_send(FCU_CONTROL_STATUS_ID,FCU_CONTROL_STATUS_CRC,p,FCU_CONTROL_STATUS_LEN);
  } else if(slot==5) {
    comm_rc_publish_status(p);
  } else if(slot<8) {
    wire_u32(p+FCU_LINK_STATUS_LINK_INSTANCE_ID,slot==6 ? 1 : 2);
    wire_u16(p+FCU_LINK_STATUS_LINK_KIND,slot==6 ? LINK_KIND_RC : LINK_KIND_GCS);
    wire_u16(p+FCU_LINK_STATUS_LINK_STATE,LINK_STATE_UNKNOWN);
    wire_u32(p+FCU_LINK_STATUS_AGE_US,UINT32_MAX);
    wire_u32(p+FCU_LINK_STATUS_REASON_CODE,slot==6 ? 19 : 15);
    telemetry_link_send(FCU_LINK_STATUS_ID,FCU_LINK_STATUS_CRC,p,FCU_LINK_STATUS_LEN);
  } else if(slot==8) {
    wire_u32(p+FCU_RESOURCE_STATUS_DEADLINE_MISS_COUNT,stats.deadline_misses);
    telemetry_link_send(FCU_RESOURCE_STATUS_ID,FCU_RESOURCE_STATUS_CRC,p,FCU_RESOURCE_STATUS_LEN);
  } else if(slot==9) {
    wire_u32(p+FCU_STORAGE_STATUS_CRITICAL_EVENT_DROPS,stats.critical_event_drops);
    wire_u32(p+FCU_STORAGE_STATUS_LOG_DROPS,stats.log_event_drops);
    wire_u32(p+FCU_STORAGE_STATUS_REASON_CODE,19);
    telemetry_link_send(FCU_STORAGE_STATUS_ID,FCU_STORAGE_STATUS_CRC,p,FCU_STORAGE_STATUS_LEN);
#if FLIGHT_TELEMETRY_ABI_LOCK
  } else {
    static const char hex[]=ABI_LOCK_ARM_HASH;
    unsigned i;
    wire_u32(p+FCU_BUILD_INFO_PRODUCT_REVISION,PRODUCT_CONTRACT_REVISION);
    wire_u32(p+FCU_BUILD_INFO_PORT_REVISION,PORT_API_REVISION);
    wire_u16(p+FCU_BUILD_INFO_SCHEMA_MAJOR,CONTRACT_SCHEMA_MAJOR);
    wire_u16(p+FCU_BUILD_INFO_SCHEMA_MINOR,CONTRACT_SCHEMA_MINOR);
    p[FCU_BUILD_INFO_DIGEST_KIND]=3; /* full canonical ARM ABI SHA256 */
    for(i=0;i<32;++i) {
      unsigned a=(unsigned)hex[2*i],b=(unsigned)hex[2*i+1];
      a=a>='a' ? a-'a'+10 : a-'0'; b=b>='a' ? b-'a'+10 : b-'0';
      p[FCU_BUILD_INFO_SHA256+i]=(uint8_t)((a<<4)|b);
    }
    telemetry_link_send(FCU_BUILD_INFO_ID,FCU_BUILD_INFO_CRC,p,FCU_BUILD_INFO_LEN);
#endif
  }
  slot=(slot+1)%(10u+FLIGHT_TELEMETRY_ABI_LOCK);
}
static void event_telemetry(void) {
  RuntimeEvent e; uint8_t p[128];
  /* Leave capacity for the next status burst; retain RAM evidence while
   * UART DMA is busy instead of popping and silently losing it. */
  if(!telemetry_link_tx_idle() || runtime_take_event(&e)) return;
  fill_telemetry_header(p,e.time_us,e.boot_id);
  wire_u32(p+FCU_EVENT_EVENT_ID,e.event_id);
  wire_u32(p+FCU_EVENT_REASON_CODE,e.reason_code);
  wire_u32(p+FCU_EVENT_AUTHORITY_EPOCH,e.authority_epoch);
  wire_u32(p+FCU_EVENT_DIRECTIVE_EPOCH,e.directive_epoch);
  wire_u32(p+FCU_EVENT_PERMIT_EPOCH,e.permit_epoch);
  wire_u16(p+FCU_EVENT_SOURCE_ID,e.source_id);
  wire_u16(p+FCU_EVENT_REASON_NAMESPACE,e.reason_namespace);
  p[FCU_EVENT_SEVERITY]=e.severity;
  telemetry_link_send(FCU_EVENT_ID,FCU_EVENT_CRC,p,FCU_EVENT_LEN);
}
static void publish_flight_status(uint64_t now) {
  static NavigationSnapshot nav;
  static NavigationCapability cap;
  static FmsStateSnapshot fms;
  static PreArmCheckResult prearm;
  TimeServiceStatus time; LocalTimebaseDisciplineStatus discipline; RuntimeStats stats;
  uint8_t p[128]; ClockInstant clock;
  if(clock_read(&clock)) return;
  (void)runtime_get_navigation(&nav,&cap); (void)runtime_get_fms(&fms,&prearm);
  runtime_get_stats(&stats);
  fill_telemetry_header(p,now,clock.boot_id);
  wire_u32(p+FCU_OPERATION_STATUS_AUTHORITY_EPOCH,fms.authority_epoch);
  wire_u32(p+FCU_OPERATION_STATUS_DIRECTIVE_EPOCH,fms.directive_epoch);
  wire_u32(p+FCU_OPERATION_STATUS_PERMIT_EPOCH,fms.permit_epoch);
  p[FCU_OPERATION_STATUS_ARM_STATE]=fms.arm_state; p[FCU_OPERATION_STATUS_FLIGHT_MODE]=fms.flight_mode;
  p[FCU_OPERATION_STATUS_AUTHORITY]=fms.authority; p[FCU_OPERATION_STATUS_FAILSAFE_LEVEL]=fms.failsafe_level;
  p[FCU_OPERATION_STATUS_MISSION_STATE]=fms.mission_state;
  telemetry_link_send(FCU_OPERATION_STATUS_ID,FCU_OPERATION_STATUS_CRC,p,FCU_OPERATION_STATUS_LEN);
  fill_telemetry_header(p,now,clock.boot_id);
  wire_u32(p+FCU_NAV_STATUS_STATE_EPOCH,nav.state_epoch);
  wire_u32(p+FCU_NAV_STATUS_ORIGIN_ID,nav.origin_id);
  wire_u32(p+FCU_NAV_STATUS_CAPABILITY_MASK,now<nav.header.valid_until_us ? cap.capability_mask : 0);
  wire_float(p+FCU_NAV_STATUS_POSITION_STD_M,cap.horizontal_position_std_m);
  wire_float(p+FCU_NAV_STATUS_VELOCITY_STD_MPS,cap.velocity_std_mps);
  wire_float(p+FCU_NAV_STATUS_HEADING_STD_RAD,cap.heading_std_rad);
  wire_u32(p+FCU_NAV_STATUS_AGE_US,now>=nav.header.timestamp_sample_us ?
    (uint32_t)fmin((double)UINT32_MAX,(double)(now-nav.header.timestamp_sample_us)):UINT32_MAX);
  wire_u32(p+FCU_NAV_STATUS_REASON_CODE,(uint32_t)-stats.navigation_status);
  telemetry_link_send(FCU_NAV_STATUS_ID,FCU_NAV_STATUS_CRC,p,FCU_NAV_STATUS_LEN);
  fill_telemetry_header(p,now,clock.boot_id);
  wire_u16(p+FCU_ACTUATOR_STATUS_OUTPUT_POLICY_ID,1);
  wire_u32(p+FCU_ACTUATOR_STATUS_REASON_CODE,19);
  /* available mask=0, feedback capability=NONE, no invented RPM/thrust. */
  telemetry_link_send(FCU_ACTUATOR_STATUS_ID,FCU_ACTUATOR_STATUS_CRC,p,FCU_ACTUATOR_STATUS_LEN);
  memset(p,0,sizeof(p));
  wire_u64(p+FCU_PREARM_STATUS_TIME_BOOT_US,now);
  wire_u64(p+FCU_PREARM_STATUS_BOOT_ID,clock.boot_id);
  wire_u64(p+FCU_PREARM_STATUS_EVIDENCE_TIME_US,prearm.checked_at_us);
  wire_u32(p+FCU_PREARM_STATUS_PROFILE_ID,PRODUCT_PROFILE_ID);
  wire_u32(p+FCU_PREARM_STATUS_CHECK_EPOCH,prearm.check_epoch);
  wire_u32(p+FCU_PREARM_STATUS_REASON_CODE,19); /* bench output policy */
  wire_u16(p+FCU_PREARM_STATUS_CHECK_ID,5); /* ACTUATOR_OUTPUT_READY */
  /* A per-check message: inhibited output is an explicit FAIL. */
  p[FCU_PREARM_STATUS_CHECK_STATE]=2; /* FAIL; 3 means WARNING. */
  telemetry_link_send(FCU_PREARM_STATUS_ID,FCU_PREARM_STATUS_CRC,p,FCU_PREARM_STATUS_LEN);
  if(!time_get_status(&time) && !time_get_local_discipline_status(&discipline)) {
    memset(p,0,sizeof(p)); wire_u64(p+FCU_TIME_STATUS_TIME_BOOT_US,now);
    wire_u64(p+FCU_TIME_STATUS_BOOT_ID,clock.boot_id);
    wire_u64(p+FCU_TIME_STATUS_UNIX_TIME_US,time.utc_valid ? time.utc_estimate_ns/1000 : 0);
    wire_u32(p+FCU_TIME_STATUS_PROFILE_ID,PRODUCT_PROFILE_ID);
    wire_u32(p+FCU_TIME_STATUS_TIME_EPOCH,time.time_epoch);
    wire_u32(p+FCU_TIME_STATUS_SOURCE_MAPPING_EPOCH,time.utc_mapping_epoch);
    wire_u32(p+FCU_TIME_STATUS_UTC_UNCERTAINTY_US,(uint32_t)fmin((double)UINT32_MAX,time.utc_uncertainty_ns/1000.0));
    wire_u16(p+FCU_TIME_STATUS_TIME_QUALITY,time.state);
    wire_u16(p+FCU_TIME_STATUS_DISCIPLINE_STATE,discipline.discipline_state);
    p[FCU_TIME_STATUS_UTC_VALID]=time.utc_valid;
    telemetry_link_send(FCU_TIME_STATUS_ID,FCU_TIME_STATUS_CRC,p,FCU_TIME_STATUS_LEN);
  }
  publish_rotating_diagnostic(now,clock.boot_id);
}
static void publish_standard_navigation(uint64_t now)
{
  static NavigationSnapshot nav;
  ProductHomeTargetSnapshot home = {0};
  uint8_t p[32] = {0};
  const bool snapshot = runtime_running() && runtime_data_lock(RT_WAITING_FOREVER);
  if (snapshot) {
    nav = runtime_data.nav;
    home = runtime_data.bundle.directive.payload.home;
    runtime_data_unlock();
  }
  const bool fresh = snapshot && runtime_data_header_is_current(&nav.header, now);
  uint8_t hud[20] = {0};
  wire_float(hud, NAN); /* no airspeed sensor */
  wire_float(hud + 4, fresh && (nav.core.valid_mask & NAV_VALID_HORIZONTAL_VELOCITY) ?
             hypotf(nav.core.velocity_ned_mps[0], nav.core.velocity_ned_mps[1]) : NAN);
  wire_float(hud + 8, NAN); /* no verified altitude above mean sea level from the estimator */
  wire_float(hud + 12, fresh && (nav.core.valid_mask & NAV_VALID_VERTICAL_VELOCITY) ?
             -nav.core.velocity_ned_mps[2] : NAN);
  telemetry_link_send(74, 20, hud, sizeof(hud)); /* VFR_HUD */

  const float local_altitude = fresh && (nav.core.valid_mask & NAV_VALID_VERTICAL_POSITION) ?
      -nav.core.position_ned_m[2] : NAN;
  const bool home_valid = home.validity == PRODUCT_HOME_VALID && now < home.home_expires_at_us &&
      home.origin_id == nav.origin_id && home.state_epoch == nav.state_epoch;
  uint8_t altitude[MAVLINK_ALTITUDE_LEN];
  wire_altitude(altitude, fresh ? nav.header.timestamp_sample_us : now, NAN, local_altitude,
                home_valid ? local_altitude + home.position_ned_m[2] : NAN);
  telemetry_link_send(MAVLINK_ALTITUDE_ID, MAVLINK_ALTITUDE_CRC, altitude, sizeof(altitude));
  if (!fresh)
    return;
  wire_u32(p, (uint32_t)(nav.header.timestamp_sample_us / 1000u));
  if (nav.core.valid_mask & NAV_VALID_ATTITUDE) {
    for (unsigned i = 0; i < 4; ++i) wire_float(p + 4 + i * 4, nav.core.q_nb_wxyz[i]);
    for (unsigned i = 0; i < 3; ++i)
      wire_float(p + 20 + i * 4, nav.core.valid_mask & NAV_VALID_BODY_RATE ?
                 nav.core.body_rate_frd_radps[i] : NAN);
    telemetry_link_send(31, 246, p, sizeof(p)); /* ATTITUDE_QUATERNION */
  }
  const uint32_t local_mask = NAV_VALID_HORIZONTAL_POSITION | NAV_VALID_VERTICAL_POSITION |
      NAV_VALID_HORIZONTAL_VELOCITY | NAV_VALID_VERTICAL_VELOCITY;
  if ((nav.core.valid_mask & local_mask) == local_mask) {
    for (unsigned i = 0; i < 3; ++i) {
      wire_float(p + 4 + i * 4, nav.core.position_ned_m[i]);
      wire_float(p + 16 + i * 4, nav.core.velocity_ned_mps[i]);
    }
    telemetry_link_send(32, 185, p, 28); /* LOCAL_POSITION_NED */
  }
}

#ifndef FLIGHT_SIM_LINK
static void publish_raw_gps(uint64_t now)
{
  const uint8_t mav_fix[] = {1, 1, 3, 4, 5, 6};
  GnssSample fix;
  double msl;
  uint8_t p[34] = {0};
  wire_u64(p, now);
  memset(p + 20, 0xff, 8); /* DOP, ground speed and course unavailable. */
  p[28] = 0; /* receiver unavailable */
  p[29] = UINT8_MAX;
  const bool available = sensor_gnss_latest(now, &fix, &msl);
  if (available) {
    p[28] = 1; /* receiver reporting, no valid fix yet */
    p[29] = fix.satellite_count;
  }
  if (available && (fix.quality_flags & GNSS_Q_POSITION_VALID) &&
      isfinite(fix.latitude_deg) && fabs(fix.latitude_deg) <= 90.0 &&
      isfinite(fix.longitude_deg) && fabs(fix.longitude_deg) <= 180.0 &&
      fix.fix_type >= GNSS_FIX_SINGLE && fix.fix_type < sizeof(mav_fix) &&
      isfinite(msl) && fabs(msl) < 100000.0 && fabsf(fix.altitude_m) < 100000.0f) {
    wire_u64(p, fix.header.timestamp_sample_us);
    wire_u32(p + 8, (uint32_t)(int32_t)lround(fix.latitude_deg * 1e7));
    wire_u32(p + 12, (uint32_t)(int32_t)lround(fix.longitude_deg * 1e7));
    wire_u32(p + 16, (uint32_t)(int32_t)lround(msl * 1000.0));
    p[28] = mav_fix[fix.fix_type];
    p[29] = fix.satellite_count;
    wire_u32(p + 30, (uint32_t)(int32_t)lround(fix.altitude_m * 1000.0));
    if (fix.quality_flags & GNSS_Q_VELOCITY_VALID) {
      const float speed = hypotf(fix.velocity_ned_mps[0], fix.velocity_ned_mps[1]);
      if (isfinite(speed) && speed <= 655.34f)
        wire_u16(p + 24, (uint16_t)lroundf(100.0f * speed));
      if (isfinite(speed) && speed >= 0.5f) {
        float course = atan2f(fix.velocity_ned_mps[1], fix.velocity_ned_mps[0]) * (180.0f / 3.14159265359f);
        if (course < 0.0f) course += 360.0f;
        wire_u16(p + 26, (uint16_t)(lroundf(100.0f * course) % 36000L));
      }
    }
  }
  telemetry_link_send(24, 24, p, sizeof(p)); /* GPS_RAW_INT */
}
#endif

static void write_standard_sensor_status(uint8_t *payload, int prearm_ready, int rc_state)
{
  /* SYS_STATUS uses separate bits for gyro and accelerometer, which share
   * one integrity report in Flight's IMU owner. */
  static const uint32_t sensor_bits[] = {3u, 4u, 8u, 32u};
  const uint32_t prearm_bit = UINT32_C(1) << 28;
  uint32_t present = prearm_bit, enabled = prearm_bit;
  uint32_t healthy = prearm_ready ? prearm_bit : 0u;
  for (unsigned i = 0; i < sizeof(sensor_bits) / sizeof(sensor_bits[0]); ++i) {
    RuntimeSensorHealth status;
    if (runtime_get_sensor_health(i, &status) != STATUS_OK || !status.sensor_instance_id)
      continue;
    present |= sensor_bits[i];
    if (status.use_state == SENSOR_USE_REJECT)
      continue;
    enabled |= sensor_bits[i];
    if (status.health_state == 2u)
      healthy |= sensor_bits[i];
  }
  if (rc_state != 0) {
    const uint32_t receiver_bit = UINT32_C(1) << 16;
    present |= receiver_bit;
    enabled |= receiver_bit;
    if (rc_state == 1)
      healthy |= receiver_bit;
  }
  wire_u32(payload, present);
  wire_u32(payload + 4, enabled);
  wire_u32(payload + 8, healthy);
}

static void publish_standard_status(uint64_t now)
{
  static FmsStateSnapshot fms;
  static PreArmCheckResult prearm;
  uint8_t heartbeat[9] = {0,0,0,0,2,0,1,3,3}; /* CUSTOM_MODE_ENABLED */
  uint8_t status[31] = {0};
  if (!runtime_get_fms(&fms, &prearm) && runtime_data_header_is_current(&fms.header, now)) {
    wire_u32(heartbeat, fms.flight_mode);
    if (fms.arm_state == ARM_STATE_ARMED) {
      heartbeat[6] |= 128; /* SAFETY_ARMED */
      heartbeat[7] = 4; /* ACTIVE */
    }
    if (fms.failsafe_level >= FAILSAFE_ACTION_REQUIRED) heartbeat[7] = 5; /* CRITICAL */
  }
  telemetry_link_send(0, 50, heartbeat, sizeof(heartbeat));
  /* BENCH_INHIBIT always fails the actuator readiness check. */
  write_standard_sensor_status(status, 0, comm_rc_status(now));
  wire_u16(status + 14, UINT16_MAX); /* battery voltage unknown */
  wire_u16(status + 16, UINT16_MAX); /* battery current unknown (-1) */
  status[30] = UINT8_MAX; /* battery remaining unknown (-1) */
  telemetry_link_send(1, 124, status, sizeof(status));
}

static void send_pending_parameter(void)
{
  if (next_parameter < parameter_store_exposed_count() && telemetry_link_tx_idle())
    send_parameter(next_parameter++);
}

void comm_gcs_publish(uint64_t now) {
  static uint64_t last_telemetry, last_heartbeat, next_navigation, last_extended_state;
#ifndef FLIGHT_HOST_TEST
  if (reboot_at_us && now >= reboot_at_us) {
    reboot_at_us = 0;
    rt_hw_cpu_reset();
  }
#endif
  send_pending_parameter();
  publish_calibration_status();
  if (!next_navigation) next_navigation = now + 50000u;
  if (!last_heartbeat || now - last_heartbeat >= 1000000u) {
    publish_standard_status(now);
    last_heartbeat = now;
  }
  if (now >= next_navigation && telemetry_link_tx_idle()) {
    publish_standard_navigation(now);
    next_navigation = now + 100000u;
  }
  if (extended_state_period_us && now - last_extended_state >= extended_state_period_us &&
      telemetry_link_tx_idle()) {
    send_extended_state();
    last_extended_state = now;
  }
#ifndef FLIGHT_SIM_LINK
  static uint64_t next_gps;
  if (!next_gps) next_gps = now + 250000u;
  if (now >= next_gps && telemetry_link_tx_idle()) {
    publish_raw_gps(now);
    next_gps = now + 500000u;
  }
#endif
  event_telemetry();
  if(now-last_telemetry>=100000) { publish_flight_status(now); last_telemetry=now; }
}
