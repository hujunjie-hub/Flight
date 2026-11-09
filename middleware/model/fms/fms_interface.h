#ifndef FMS_INTERFACE_H
#define FMS_INTERFACE_H

#include "fms_contract/fms_contract.h"

EXTERN_C_BEGIN
uint32_t fms_selected_policy_id(void);
void fms_default_config(ProductFmsConfig *config);

/* FMS owner only, after the ordinary update. */
Status fms_authorize_origin_reset(const ContractHeader *candidate, uint32_t next_origin_id,
                                  EstimatorResetRequest *request);
/* Install the product fence table before init. The model owns the copy. */
Status fms_configure_product_geofence(const ProductGeofenceConfig *config);
#ifdef FLIGHT_HOST_SIL
Status fms_sil_replace_geofences(const ProductGeofenceSnapshot *fences, uint16_t count);
#endif
EXTERN_C_END

#endif
