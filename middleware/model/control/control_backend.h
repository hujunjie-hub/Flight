#ifndef CONTROL_BACKEND_H
#define CONTROL_BACKEND_H

#include "ports/port_api.h"

/* Implemented by the model selected at build time. */
EXTERN_C_BEGIN
Status control_backend_validate(const ProductProfile *profile);
Status control_backend_stop(void);
Status control_backend_init(const ProductControllerConfig *config);
/* Session tuning: replaces the payload in the running controller. */
Status control_backend_apply_payload(const ProductControllerConfigPayload *payload);
Status control_backend_update(const ControllerInput *input, ControllerOutput *output);
Status control_backend_get_health(ControlHealth *output);
Status control_backend_model_get_descriptor(ControlModelDescriptor *output);
Status control_backend_model_get_view(const OperatingPoint *op, float dt_s, const SafetyEnvelope *envelope, ControlModelView *output);
Status control_backend_model_validate(const ControlModelState *x, const ControlModelInput *u, const OperatingPoint *op);
Status control_backend_model_predict(const ControlModelState *x, const ControlModelInput *u, float dt_s, ControlModelState *x_next);
Status control_backend_model_get_linear_model(const OperatingPoint *op, float dt_s, ControlLinearModel *output);
Status control_backend_model_get_constraints(const SafetyEnvelope *envelope, ControlModelBounds *output);
Status control_backend_model_check_domain(const OperatingPoint *op);
Status control_backend_outer_update(const ControllerInput *input);
void control_backend_sim_config(ProductControllerConfig *config);
EXTERN_C_END

#endif
