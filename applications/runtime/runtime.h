#ifndef RUNTIME_H
#define RUNTIME_H
#include "ports/port_api.h"
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Starts the modules and the statically configured RT-Thread tasks. */
int runtime_start(void);

/* These entry points copy typed input into bounded mailboxes. Only a trusted
 * protocol adapter may call them; they do not grant authority or ARM. */
Status runtime_submit_request(const OperationRequest *request);
Status runtime_submit_operator(const OperatorInput *input);
Status runtime_submit_mission(const MissionUploadCandidate *candidate);
Status runtime_submit_measurement(const SensorMeasurement *measurement);
/* Session tuning entry point: applies a controller payload only while the FMS
 * reports DISARMED. Returns STATUS_NOT_PERMITTED while armed. */
Status runtime_apply_controller_parameters(const ProductControllerConfigPayload *payload);
typedef struct {
  uint32_t started, provisional_parameters, imu_batches, imu_rejected;
  uint32_t measurement_rejected, nav_updates, control_updates, control_rejected;
  uint32_t planning_updates, mpc_updates, queue_overruns, deadline_misses;
  /* M6 acceptance counters: every MPC job is classified by the status the
   * outer loop returned, and the covered time accumulates only for accepted
   * jobs so a fallback window can never be counted as coverage. */
  uint32_t mpc_solved, mpc_rejected;
  uint64_t mpc_covered_us;
  uint32_t event_drops, log_event_drops, sd_write_errors;
  uint32_t critical_event_drops, nor_io_errors;
  bool sd_recording, nor_recording;
  Status navigation_status, control_status, planning_status, last_error;
  uint64_t last_imu_us, last_navigation_us;
} RuntimeStats;
void runtime_get_stats(RuntimeStats *out);
/* Copies only; telemetry never borrows an algorithm's mutable storage. */
Status runtime_get_navigation(NavigationSnapshot *nav, NavigationCapability *cap);
Status runtime_get_fms(FmsStateSnapshot *fms, PreArmCheckResult *prearm);
bool runtime_bench_inhibit_active(void);
/* Integration diagnostics, not public Port/ABI types. Snapshots are owned by
 * the runtime; the event FIFO is volatile RAM, never a durable flight log. */
typedef struct {
  uint32_t sensor_instance_id,policy_epoch,reason_code,gap_count,reject_count;
  uint16_t health_state,use_state;
} RuntimeSensorHealth;
typedef struct {
  uint64_t time_us,boot_id;
  uint32_t event_id,reason_code,authority_epoch,directive_epoch,permit_epoch;
  uint16_t source_id,reason_namespace;
  uint8_t severity;
  uint32_t time_epoch;
} RuntimeEvent;
Status runtime_get_sensor_health(unsigned index,RuntimeSensorHealth *out);
Status runtime_get_control(ControlCapability *cap,ControlHealth *health,ActuatorDemand *demand,
    uint32_t *reference_id);
Status runtime_take_event(RuntimeEvent *event);
#ifdef __cplusplus
}
#endif
#endif
