#ifndef RUNTIME_INTERNAL_H
#define RUNTIME_INTERNAL_H

#include "runtime.h"
#include "sample_normalization/sensor_sample.h"
#include <rtthread.h>

typedef enum {
    RUNTIME_OWNER_SENSOR,
    RUNTIME_OWNER_MISSION,
    RUNTIME_OWNER_MPC,
    RUNTIME_OWNER_ACTUATOR,
    RUNTIME_OWNER_COMMUNICATION,
#ifdef FLIGHT_SIM_LINK
    RUNTIME_OWNER_SIM,
#endif
    RUNTIME_OWNER_COUNT
} RuntimeOwner;

enum {
    RUNTIME_SENSOR_COUNT = 4,
    RUNTIME_MEASUREMENT_CAPACITY = 8
};

/* Task snapshots are copied under the data mutex. Workspaces belong to the
 * tasks and algorithm calls run outside the mutex. */
typedef struct {
  NavigationSnapshot nav;
  GuidanceOutput guidance;
  VehicleDirective directive;
  SafetyEnvelope envelope;
  AuthorityStatus authority;
  ActuationPermit permit;
  GroundStopCertificate ground_stop_certificate;
  ActuatorStatus actuator;
  ActuatorCapability actuator_cap;
  ActuatorCommitReceipt receipt;
  ControlModelView model;
} ControlBundle;
typedef struct {
  struct rt_mutex mutex;
  struct rt_semaphore imu_event;
  struct rt_semaphore plan_event;
#ifdef FLIGHT_LOGGER_ENABLE
  ImuSample log_imu;
#endif
  ContractHeader origin_candidate;
  uint32_t next_origin_id;
  EstimatorResetRequest origin_reset;
  uint8_t have_origin_reset;
  NavigationSnapshot nav;
  NavigationCapability nav_cap;
  ActuatorStatus actuator;
  ActuatorCapability actuator_cap;
  ActuatorCommitReceipt receipt;
  ControlCapability control_cap;
  ControlHealth control_health;
  uint32_t control_reference_id;
  RuntimeSensorHealth sensors[RUNTIME_SENSOR_COUNT];
  ControlBundle bundle;
  uint32_t bundle_revision;
  ActuatorDemand demand;
  OperatorInput operator_input;
  MissionUploadCandidate mission;
  FmsStateSnapshot fms;
  PreArmCheckResult prearm;
  uint8_t have_operator, have_mission, have_demand, bundle_ready;
} RuntimeData;

extern RuntimeData runtime_data;
extern RuntimeStats runtime_stats;

int runtime_data_init(uint64_t start_us);
void runtime_data_deinit(void);
int runtime_data_lock(rt_int32_t wait);
void runtime_data_unlock(void);
void runtime_data_mark_heartbeat(RuntimeOwner owner, uint64_t now);
uint64_t runtime_data_get_heartbeat(RuntimeOwner owner);
void runtime_data_record_event(unsigned source, unsigned ns, unsigned reason,
                              unsigned severity, uint64_t now);
void runtime_data_set_event_epochs(const FmsStateSnapshot *snapshot);
#ifdef FLIGHT_LOGGER_ENABLE
bool runtime_data_take_log_event(RuntimeEvent *event);
#endif
#ifdef FLIGHT_NOR_LOG_ENABLE
bool runtime_data_take_critical_event(RuntimeEvent *event);
bool runtime_nor_writes_allowed(void);
#endif
int runtime_data_header_is_current(const ContractHeader *header, uint64_t now);
unsigned runtime_data_take_requests(OperationRequest *out);
bool runtime_data_take_measurement(SensorMeasurement *out);
void runtime_data_controller_input(const ControlBundle *bundle, ControllerInput *input,
                                  uint64_t now);

static inline uint64_t runtime_now_us(void)
{
    uint64_t now = 0u;
    (void)clock_now_us(&now);
    return now;
}

int runtime_initialize(void);
void runtime_shutdown(void);
int runtime_running(void);
void task_sensor_prepare(const sensor_imu_config_t *imu, const ProductEstimatorConfigPayload *estimator,
                             const ClockInstant *clock);

#endif
