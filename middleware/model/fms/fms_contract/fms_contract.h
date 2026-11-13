/*
 * Contract selection for M4/FMS.
 *
 * Production builds use the A-owned public ICD/Port headers in
 * middleware/contracts/ (T191-A-ABI) plus the selected product contract in
 * middleware/. M4 never keeps a second copy of these
 * types.
 */
#ifndef FMS_CONTRACT_H
#define FMS_CONTRACT_H

#include "ports/port_api.h"    /* common base -> product contract -> messages -> Ports */
#include "messages/message_ids.h" /* MSG_* registry values */
/* ArmState (06 §7.1). */
enum {
  ARM_STATE_DISARMED = 0,
  ARM_STATE_ARMING = 1,
  ARM_STATE_ARMED = 2,
  ARM_STATE_DISARMING = 3
};

/* Stable reasons used to bind an M4 ground-stop certificate to the exact
 * permit revocation that caused the safe ground disarm. */
#define FMS_PERMIT_REVOKE_REASON_FIRST_REFERENCE_GROUND_DISARM 11u
#define FMS_PERMIT_REVOKE_REASON_COMMAND_DISARM 20u
#define FMS_DECISION_REASON_FIRST_REFERENCE_GROUND_DISARM 6u

/* FlightMode (06 §7.1; NONE only while disarmed). */
enum {
  FLIGHT_MODE_NONE = 0,
  FLIGHT_MODE_MANUAL_RATE = 1,
  FLIGHT_MODE_MANUAL_ATTITUDE = 2,
  FLIGHT_MODE_ALT_HOLD = 3,
  FLIGHT_MODE_POS_HOLD = 4,
  FLIGHT_MODE_AUTO_MISSION = 5,
  FLIGHT_MODE_AUTO_RETURN = 6,
  FLIGHT_MODE_AUTO_LAND = 7,
  FLIGHT_MODE_EMERGENCY_LAND = 8
};

/* Authority (00 §5.1): SAFETY is a constraint, never a value here. */
enum {
  AUTHORITY_NONE = 0,
  AUTHORITY_PILOT = 1,
  AUTHORITY_MISSION = 2,
  AUTHORITY_GCS_SUPERVISED = 3
};

/* FailsafeLevel (06 §7.1). */
enum {
  FAILSAFE_NORMAL = 0,
  FAILSAFE_DEGRADED = 1,
  FAILSAFE_ACTION_REQUIRED = 2,
  FAILSAFE_TERMINAL = 3
};

/* MissionState (06 §7.1). */
enum {
  MISSION_STATE_IDLE = 0,
  MISSION_STATE_LOADED = 1,
  MISSION_STATE_RUNNING = 2,
  MISSION_STATE_PAUSED = 3,
  MISSION_STATE_ABORTED = 4,
  MISSION_STATE_COMPLETED = 5
};

/* Stable PreArm check IDs 0..14 (06 §7.6): reordering is forbidden. */
enum {
  PREARM_ID_PLATFORM_READY = 0,
  PREARM_ID_IMU_READY = 1,
  PREARM_ID_ESTIMATOR_ATTITUDE = 2,
  PREARM_ID_ESTIMATOR_POSITION = 3,
  PREARM_ID_RC_INPUT = 4,
  PREARM_ID_ACTUATOR_OUTPUT_READY = 5,
  PREARM_ID_BATTERY_ENERGY = 6,
  PREARM_ID_BARO_HEIGHT = 7,
  PREARM_ID_GNSS_HOME_ORIGIN = 8,
  PREARM_ID_COMPASS_HEADING = 9,
  PREARM_ID_CONTROL_MODEL = 10,
  PREARM_ID_CONTROL_SOLVER = 11,
  PREARM_ID_SAFETY_ENVELOPE = 12,
  PREARM_ID_GEOFENCE = 13,
  PREARM_ID_STORAGE_EVENT_LOG = 14,
  PREARM_ID_COUNT = 15
};

/* Per-item PreArm result (06 §7.6 / telemetry encoding). */
enum {
  PREARM_ITEM_UNKNOWN = 0,
  PREARM_ITEM_PASS = 1,
  PREARM_ITEM_FAIL = 2,
  PREARM_ITEM_WARNING = 3
};

/* ProductFmsInputExtension.ground_contact_state encoding. */
enum {
  GROUND_CONTACT_UNKNOWN = 0,
  GROUND_CONTACT_CONTACT = 1,
  GROUND_CONTACT_NO_CONTACT = 2
};

/* ProductFmsInputExtension.energy_state encoding. */
enum {
  ENERGY_UNKNOWN = 0,
  ENERGY_NORMAL = 1,
  ENERGY_LOW = 2,
  ENERGY_CRITICAL = 3
};

/* ProductDirectivePayload.flight_phase encoding (02 §4). */
enum {
  FLIGHT_PHASE_GROUND = 0,
  FLIGHT_PHASE_TAKEOFF = 1,
  FLIGHT_PHASE_CRUISE = 2,
  FLIGHT_PHASE_LANDING = 3,
  FLIGHT_PHASE_EMERGENCY = 4
};

#endif /* FMS_CONTRACT_H */
