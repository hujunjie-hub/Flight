#ifndef INS_MODEL_H
#define INS_MODEL_H

#include "ports/port_api.h"

/* Implemented by the model selected at build time. */
EXTERN_C_BEGIN
Status ins_model_validate(const ProductProfile *profile);
Status ins_model_init(const ProductEstimatorConfig *config);
Status ins_model_push_imu(const ImuSample *sample);
Status ins_model_push_measurement(const SensorMeasurement *sample);
Status ins_model_set_sensor_policy(const SensorUsePolicy *policy, uint64_t evaluation_time_us);
Status ins_model_push_gap(const SensorDataGapEvent *event);
Status ins_model_get_sensor_consistency(uint32_t sensor_instance_id, SensorConsistencyEvidence *out);
Status ins_model_update(uint64_t now_us);
Status ins_model_get_snapshot(NavigationSnapshot *output);
Status ins_model_get_origin(NavigationOriginSnapshot *output);
Status ins_model_stop(void);
Status ins_model_get_capability(NavigationCapability *output);
Status ins_model_reset(const EstimatorResetRequest *request);
Status ins_model_get_health(EstimatorHealth *output);
Status ins_model_take_reset_event(StateResetEvent *event);
Status ins_model_set_replay_budget(uint32_t budget_us, uint32_t period_us);
Status ins_model_get_pending_origin(ContractHeader *candidate, uint32_t *next_origin_id);
void ins_model_sim_config(ProductEstimatorConfig *config);
EXTERN_C_END

#endif
