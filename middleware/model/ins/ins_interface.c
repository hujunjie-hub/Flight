#include "ins_interface.h"
#include "ins_model.h"

Status estimator_validate(const ProductProfile *profile)
{
    return ins_model_validate(profile);
}

Status estimator_init(const ProductEstimatorConfig *config)
{
    return ins_model_init(config);
}

Status estimator_push_imu(const ImuSample *sample)
{
    return ins_model_push_imu(sample);
}

Status estimator_push_measurement(const SensorMeasurement *sample)
{
    return ins_model_push_measurement(sample);
}

Status estimator_set_sensor_policy(const SensorUsePolicy *policy, uint64_t evaluation_time_us)
{
    return ins_model_set_sensor_policy(policy, evaluation_time_us);
}

Status estimator_push_gap(const SensorDataGapEvent *event)
{
    return ins_model_push_gap(event);
}

Status estimator_get_sensor_consistency(uint32_t sensor_instance_id, SensorConsistencyEvidence *out)
{
    return ins_model_get_sensor_consistency(sensor_instance_id, out);
}

Status estimator_update(uint64_t now_us)
{
    return ins_model_update(now_us);
}

Status estimator_get_snapshot(NavigationSnapshot *output)
{
    return ins_model_get_snapshot(output);
}

Status estimator_get_origin(NavigationOriginSnapshot *output)
{
    return ins_model_get_origin(output);
}

Status estimator_stop(void)
{
    return ins_model_stop();
}

Status estimator_get_capability(NavigationCapability *output)
{
    return ins_model_get_capability(output);
}

Status estimator_reset(const EstimatorResetRequest *request)
{
    return ins_model_reset(request);
}

Status estimator_get_health(EstimatorHealth *output)
{
    return ins_model_get_health(output);
}

Status ins_take_reset_event(StateResetEvent *event)
{
    return ins_model_take_reset_event(event);
}

Status ins_set_replay_budget(uint32_t budget_us, uint32_t period_us)
{
    return ins_model_set_replay_budget(budget_us, period_us);
}

Status ins_get_pending_origin(ContractHeader *candidate, uint32_t *next_origin_id)
{
    return ins_model_get_pending_origin(candidate, next_origin_id);
}

void ins_sim_config(ProductEstimatorConfig *config)
{
    ins_model_sim_config(config);
}
