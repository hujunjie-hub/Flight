/*
 * Flight public contract - Port function declarations.
 *
 * Authority: spec/02_接口契约与公共数据规范.md (V1.9.1-GLOBAL-AUDIT-R3),
 *            sections 1.4.1, 1.4.2, 4.6, 4.7 and 4.8.
 *
 * Static binding only: every Port has exactly one implementation linked into
 * the image; zero or duplicate definitions must fail at link time.  No dynamic
 * loading, reflection, runtime lookup, RTTI or heap allocation is permitted.
 * Life cycle: validate(profile) -> init(config) -> update/push/poll -> stop;
 * get_health/get_* snapshot queries are valid after a successful init.
 *
 * All functions are called from thread context for one instance at a time and
 * must not re-enter.  The only exception is actuator_update_permit, whose
 * atomic permit publication may race with the M7 thread; ISRs never call these
 * business functions.
 */
#ifndef PORT_API_H
#define PORT_API_H

#include "base_types/contract_version.h"
#include "ports/port_types.h"

EXTERN_C_BEGIN

/* ------------------------------------------------------------------------- */
/* PLATFORM time and raw acquisition services                                 */
/* ------------------------------------------------------------------------- */

Status clock_read(ClockInstant *out);
Status clock_now_us(uint64_t *mono_us); /* convenience only; authorization uses clock_read */
Status clock_ticks_to_mono_us(uint64_t tick64, uint32_t time_epoch, uint64_t *mono_us);
Status time_get_status(TimeServiceStatus *out);
Status time_get_local_discipline_status(LocalTimebaseDisciplineStatus *out);
Status time_get_mapping_status(uint32_t source_clock_domain_id,
                                  ClockDomainMappingStatus *out);
Status time_submit_pps(const PpsCaptureEvent *event);
Status time_submit_gnss_time(const GnssTimeObservation *observation);
Status time_submit_external_sync(const ExternalClockSyncObservation *observation);

/* ------------------------------------------------------------------------- */
/* M4 FMS policy                                                             */
/* ------------------------------------------------------------------------- */

Status fms_policy_validate(const ProductProfile *profile);
Status fms_policy_init(const ProductFmsConfig *config);
Status fms_policy_update(const FmsInputView *input, FmsOutputView *output);
Status fms_policy_get_health(FmsHealth *output);
Status fms_policy_stop(void);

/* ------------------------------------------------------------------------- */
/* M3 navigation state estimator                                             */
/* ------------------------------------------------------------------------- */

Status estimator_validate(const ProductProfile *profile);
Status estimator_init(const ProductEstimatorConfig *config);
Status estimator_push_imu(const ImuSample *sample);
Status estimator_push_measurement(const SensorMeasurement *sample);
Status estimator_set_sensor_policy(const SensorUsePolicy *policy, uint64_t evaluation_time_us);
Status estimator_push_gap(const SensorDataGapEvent *event);
Status estimator_get_sensor_consistency(uint32_t sensor_instance_id, SensorConsistencyEvidence *out);
Status estimator_update(uint64_t now_us);
Status estimator_get_snapshot(NavigationSnapshot *output);
Status estimator_get_origin(NavigationOriginSnapshot *output);
Status estimator_stop(void);
Status estimator_get_capability(NavigationCapability *output);
Status estimator_reset(const EstimatorResetRequest *request);
Status estimator_get_health(EstimatorHealth *output);

/* ------------------------------------------------------------------------- */
/* M5 guidance                                                               */
/* ------------------------------------------------------------------------- */

Status guidance_validate(const ProductProfile *profile);
Status guidance_stop(void);
Status guidance_init(const ProductGuidanceConfig *config);
Status guidance_update(const GuidanceInput *input, GuidanceOutput *output);
Status guidance_get_health(GuidanceHealth *output);

/* ------------------------------------------------------------------------- */
/* M6 controller                                                             */
/* ------------------------------------------------------------------------- */

Status controller_validate(const ProductProfile *profile);
Status controller_stop(void);
Status controller_init(const ProductControllerConfig *config);
Status controller_update(const ControllerInput *input, ControllerOutput *output);
Status controller_get_health(ControlHealth *output);

/* ------------------------------------------------------------------------- */
/* M7 actuator                                                               */
/* ------------------------------------------------------------------------- */

Status actuator_validate(const ProductProfile *profile);
Status actuator_stop(void);
Status actuator_update_permit(const ActuationPermit *permit, uint64_t evaluation_time_us);
Status actuator_poll(uint64_t evaluation_time_us, ActuatorStatus *status);
Status actuator_init(const ProductActuatorConfig *config);
Status actuator_apply(const ActuatorDemand *demand,
                      const ActuationPermit *permit,
                      ActuatorStatus *status);
Status actuator_get_capability(ActuatorCapability *output);
Status actuator_get_health(ActuatorHealth *output);
Status actuator_get_commit_receipt(ActuatorCommitReceipt *output);

/* ------------------------------------------------------------------------- */
/* ControlModelPort                                                          */
/* ------------------------------------------------------------------------- */

Status control_model_get_descriptor(ControlModelDescriptor *output);
Status control_model_get_view(const OperatingPoint *op, float dt_s,
                              const SafetyEnvelope *envelope, ControlModelView *output);
Status control_model_validate(const ControlModelState *x,
                              const ControlModelInput *u,
                              const OperatingPoint *op);
Status control_model_predict(const ControlModelState *x,
                             const ControlModelInput *u,
                             float dt_s,
                             ControlModelState *x_next);
Status control_model_get_linear_model(const OperatingPoint *op,
                                      float dt_s,
                                      ControlLinearModel *output);
Status control_model_get_constraints(const SafetyEnvelope *envelope,
                                     ControlModelBounds *output);
Status control_model_check_domain(const OperatingPoint *op);

/* ------------------------------------------------------------------------- */
/* PlantSimulationPort - Host/SIL/HIL targets only                           */
/*                                                                           */
/* These symbols exist only in Host/SIL/HIL builds.  The H723 firmware image */
/* must not link PlantsimulationModel, SensorSimulationPort or PlantTruth.   */
/* ------------------------------------------------------------------------- */

Status plant_sim_init(const PlantSimConfig *config);
Status plant_sim_reset(const PlantInitialCondition *initial);
Status plant_sim_step(const ActuatorDemand *applied_demand,
                      const EnvironmentInput *environment,
                      float fixed_dt_s,
                      PlantTruth *truth);
Status sensor_sim_sample(const PlantTruth *truth,
                         uint64_t sample_time_us,
                         SimulatedMeasurementBatch *output);

EXTERN_C_END

#endif /* PORT_API_H */
