#include "control_interface.h"
#include "control_backend.h"

Status controller_validate(const ProductProfile *profile)
{
    return control_backend_validate(profile);
}

Status controller_stop(void)
{
    return control_backend_stop();
}

Status controller_init(const ProductControllerConfig *config)
{
    return control_backend_init(config);
}

Status controller_update(const ControllerInput *input, ControllerOutput *output)
{
    return control_backend_update(input, output);
}

/* Session tuning: the caller owns the "disarmed only" gate. */
Status controller_apply_payload(const ProductControllerConfigPayload *payload)
{
    return control_backend_apply_payload(payload);
}

Status controller_get_health(ControlHealth *output)
{
    return control_backend_get_health(output);
}

Status control_model_get_descriptor(ControlModelDescriptor *output)
{
    return control_backend_model_get_descriptor(output);
}

Status control_model_get_view(const OperatingPoint *op, float dt_s, const SafetyEnvelope *envelope, ControlModelView *output)
{
    return control_backend_model_get_view(op, dt_s, envelope, output);
}

Status control_model_validate(const ControlModelState *x, const ControlModelInput *u, const OperatingPoint *op)
{
    return control_backend_model_validate(x, u, op);
}

Status control_model_predict(const ControlModelState *x, const ControlModelInput *u, float dt_s, ControlModelState *x_next)
{
    return control_backend_model_predict(x, u, dt_s, x_next);
}

Status control_model_get_linear_model(const OperatingPoint *op, float dt_s, ControlLinearModel *output)
{
    return control_backend_model_get_linear_model(op, dt_s, output);
}

Status control_model_get_constraints(const SafetyEnvelope *envelope, ControlModelBounds *output)
{
    return control_backend_model_get_constraints(envelope, output);
}

Status control_model_check_domain(const OperatingPoint *op)
{
    return control_backend_model_check_domain(op);
}

Status control_outer_update(const ControllerInput *input)
{
    return control_backend_outer_update(input);
}

void control_sim_config(ProductControllerConfig *config)
{
    control_backend_sim_config(config);
}
