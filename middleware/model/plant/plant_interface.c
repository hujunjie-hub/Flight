#include "plant_interface.h"

static PlantModel selected;

Status plant_interface_bind(const PlantModel *model)
{
    if (!model || !model->init || !model->reset || !model->step)
        return STATUS_INVALID_ARGUMENT;
    selected = *model;
    return STATUS_OK;
}

Status plant_sim_init(const PlantSimConfig *config)
{
    return selected.init ? selected.init(config) : STATUS_NOT_READY;
}

Status plant_sim_reset(const PlantInitialCondition *initial)
{
    return selected.reset ? selected.reset(initial) : STATUS_NOT_READY;
}

Status plant_sim_step(const ActuatorDemand *demand, const EnvironmentInput *environment,
                      float fixed_dt_s, PlantTruth *truth)
{
    return selected.step ? selected.step(demand, environment, fixed_dt_s, truth)
                         : STATUS_NOT_READY;
}
