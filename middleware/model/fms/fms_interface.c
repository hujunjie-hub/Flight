#include "fms_interface.h"
#include "fms_contract/fms_model.h"

Status fms_policy_validate(const ProductProfile *profile)
{
    return fms_model_validate(profile);
}

Status fms_policy_init(const ProductFmsConfig *config)
{
    return fms_model_init(config);
}

Status fms_policy_update(const FmsInputView *input, FmsOutputView *output)
{
    return fms_model_update(input, output);
}

Status fms_policy_get_health(FmsHealth *output)
{
    return fms_model_get_health(output);
}

Status fms_policy_stop(void)
{
    return fms_model_stop();
}

uint32_t fms_selected_policy_id(void)
{
    return fms_model_policy_id();
}

void fms_default_config(ProductFmsConfig *config)
{
    fms_model_default_config(config);
}

Status fms_authorize_origin_reset(const ContractHeader *candidate, uint32_t next_origin_id, EstimatorResetRequest *request)
{
    return fms_model_authorize_origin_reset(candidate, next_origin_id, request);
}

Status fms_configure_product_geofence(const ProductGeofenceConfig *config)
{
    return fms_model_configure_product_geofence(config);
}
