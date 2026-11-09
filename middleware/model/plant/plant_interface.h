#ifndef PLANT_INTERFACE_H
#define PLANT_INTERFACE_H

#include "ports/port_api.h"

EXTERN_C_BEGIN

typedef struct {
    Status (*init)(const PlantSimConfig *config);
    Status (*reset)(const PlantInitialCondition *initial);
    Status (*step)(const ActuatorDemand *demand, const EnvironmentInput *environment,
                   float fixed_dt_s, PlantTruth *truth);
} PlantModel;

/* A serialized Host/SIL owner selects the model before plant_sim_init.
 * The implementation owns its physical parameters and lifecycle. */
Status plant_interface_bind(const PlantModel *model);

EXTERN_C_END

#endif
