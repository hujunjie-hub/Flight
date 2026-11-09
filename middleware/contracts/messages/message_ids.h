/*
 * Flight public contract - message type registry (Schema 6.0).
 *
 * Authority: spec/contracts/message_registry.json (normative, append-only) and
 *            spec/02_接口契约与公共数据规范.md section 1.4.3.
 *
 * A message_type value is assigned once and never renumbered, even when the
 * specification text is reordered.  Unknown values are rejected by consumers.
 * ProjectGeofenceSnapshot/AuthorizedMissionSnapshot are product-defined
 * messages but keep the same registry IDs.
 *
 * tests/abi/abi_report.py checks these constants against the
 * JSON registry on every ABI run; a mismatch fails the run.
 */
#ifndef MESSAGE_IDS_H
#define MESSAGE_IDS_H

#define MSG_OPERATION_REQUEST             1u
#define MSG_OPERATOR_INPUT                2u
#define MSG_EXTERNAL_ODOMETRY             3u
#define MSG_IMU_SAMPLE                    4u
#define MSG_GNSS_SAMPLE                   5u
#define MSG_BARO_SAMPLE                   6u
#define MSG_MAG_SAMPLE                    7u
#define MSG_RANGE_SAMPLE                  8u
#define MSG_FLOW_SAMPLE                   9u
#define MSG_WHEEL_ODOMETRY_SAMPLE        10u
#define MSG_AIRSPEED_SAMPLE              11u
#define MSG_WATER_SPEED_SAMPLE           12u
#define MSG_SENSOR_MEASUREMENT           13u
#define MSG_SENSOR_DIAGNOSTIC            14u
#define MSG_SENSOR_DATA_GAP_EVENT        15u
#define MSG_NAVIGATION_SNAPSHOT          16u
#define MSG_NAVIGATION_CAPABILITY        17u
#define MSG_STATE_RESET_EVENT            18u
#define MSG_ESTIMATOR_RESET_REQUEST      19u
#define MSG_SAFETY_ENVELOPE              20u
#define MSG_SENSOR_USE_POLICY            21u
#define MSG_SENSOR_CONSISTENCY_EVIDENCE  22u
#define MSG_LANDED_STATE_SNAPSHOT        23u
#define MSG_VEHICLE_DIRECTIVE            24u
#define MSG_ACTUATION_PERMIT             25u
#define MSG_MOTION_REFERENCE             26u
#define MSG_ACTUATOR_DEMAND              27u
#define MSG_ACTUATOR_COMMIT_RECEIPT      28u
#define MSG_ACTUATOR_STATUS              29u
#define MSG_ACTUATOR_CAPABILITY          30u
#define MSG_CONTROL_CAPABILITY           31u
#define MSG_FAULT_SUMMARY                32u
#define MSG_MISSION_STATUS               33u
#define MSG_GUIDANCE_FEASIBILITY         34u
#define MSG_COMMAND_ACK                  35u
#define MSG_DECISION_EVENT               36u
#define MSG_GUIDANCE_STATUS              37u
#define MSG_FMS_STATE_SNAPSHOT           38u
#define MSG_PRE_ARM_CHECK_RESULT         39u
#define MSG_PILOT_CONTROL_REFERENCE      40u
#define MSG_MODULE_HEALTH                41u
#define MSG_COMMUNICATION_LINK_STATUS    42u
#define MSG_NAVIGATION_ORIGIN_SNAPSHOT   43u
#define MSG_AUTHORIZED_OPERATOR_INPUT    44u
#define MSG_AUTHORITY_STATUS             45u
#define MSG_FAULT_EVIDENCE_EVENT         46u
#define MSG_MISSION_UPLOAD_CANDIDATE     47u
#define MSG_PRODUCT_GEOFENCE_SNAPSHOT    48u
#define MSG_AUTHORIZED_MISSION_SNAPSHOT  49u
#define MSG_PRODUCT_OBSTACLE_OBSERVATION 50u
#define MSG_CONTROL_MODEL_VIEW           51u
#define MSG_ENVIRONMENT_CONSTRAINT       52u
#define MSG_PRODUCT_FMS_INPUT_EXTENSION  53u
#define MSG_ESTIMATOR_HEALTH             54u
#define MSG_CONTROL_HEALTH               55u

/* ModuleHealth C aliases reuse message type 41; component_id distinguishes the
 * publisher.  No separate message ID is allocated. */
#define MSG_FMS_HEALTH       MSG_MODULE_HEALTH
#define MSG_GUIDANCE_HEALTH  MSG_MODULE_HEALTH
#define MSG_ACTUATOR_HEALTH  MSG_MODULE_HEALTH

#endif /* MESSAGE_IDS_H */
