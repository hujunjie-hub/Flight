#ifndef PARAMETER_STORE_H
#define PARAMETER_STORE_H

#include "product_contract/product_contract.h"

EXTERN_C_BEGIN

/* Initialize the session from the exact controller payload used at boot. */
void parameter_store_init(const ProductControllerConfigPayload *initial);

unsigned parameter_store_count(void);

/* MAVLink exposes only the controller tuning subset (CTL_* + VEH_MASS).
 * Calibration values and vehicle geometry stay internal. */
unsigned parameter_store_exposed_count(void);
unsigned parameter_store_exposed_at(unsigned ordinal);
int parameter_store_exposed_ordinal(unsigned index);
int parameter_store_find_exposed(const char *name);

const char *parameter_store_name(unsigned index);
float parameter_store_value(unsigned index);
float parameter_store_minimum(unsigned index);
float parameter_store_maximum(unsigned index);
int parameter_store_writable(unsigned index);
/* Returns the index or -1.  name may be the 16-byte MAVLink param_id. */
int parameter_store_find(const char *name);

/* The runtime installs this hook. It returns 0 when the payload was applied. */
void parameter_store_set_apply_hook(int (*hook)(const ProductControllerConfigPayload *payload));

/* Called after a value was accepted and applied, so services that are not part
 * of ProductControllerConfig (sensor calibration) can pick it up immediately. */
void parameter_store_set_value_hook(void (*hook)(unsigned index));

/* Returns 1 when the value is now active, 0 when it was rejected. */
int parameter_store_set(unsigned index, float value);

/* Store a validated value without running the controller/observer hooks.
 * Calibration services use this to publish a result atomically. */
int parameter_store_store(unsigned index, float value);

/* Copy the current store into a controller payload (used by diagnostics). */
void parameter_store_read_payload(ProductControllerConfigPayload *payload);

EXTERN_C_END

#endif
