#ifndef FMS_MODEL_H
#define FMS_MODEL_H

#include "ports/port_api.h"

/* Implemented by the model selected at build time. */
EXTERN_C_BEGIN
Status fms_model_validate(const ProductProfile *profile);
Status fms_model_init(const ProductFmsConfig *config);
Status fms_model_update(const FmsInputView *input, FmsOutputView *output);
Status fms_model_get_health(FmsHealth *output);
Status fms_model_stop(void);
uint32_t fms_model_policy_id(void);
void fms_model_default_config(ProductFmsConfig *config);
Status fms_model_authorize_origin_reset(const ContractHeader *candidate, uint32_t next_origin_id, EstimatorResetRequest *request);
Status fms_model_configure_product_geofence(const ProductGeofenceConfig *config);
EXTERN_C_END

#endif
