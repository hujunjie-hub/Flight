#include "parameter_store.h"
#include "mavlink/parameters/ground_parameters.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

static float values[GROUND_PARAMETER_COUNT];
static uint16_t exposed_indices[GROUND_PARAMETER_COUNT];
static unsigned exposed_count;
static int (*apply_hook)(const ProductControllerConfigPayload *payload);
static void (*value_hook)(unsigned index);

/* Only the controller tuning knobs are exposed over MAVLink.  Model geometry
 * (inertia/motor positions) belongs to the compiled vehicle model, and the
 * calibration values are owned by the calibration page/commands. */
static int parameter_is_exposed(unsigned index)
{
    const char *name = ground_parameters[index].name;
    return strncmp(name, "CTL_", 4u) == 0 || strcmp(name, "VEH_MASS") == 0 ||
           strncmp(name, "RC_MODE_", 8u) == 0 || strncmp(name, "RC_AUX_", 7u) == 0;
}

static void build_exposed_indices(void)
{
    exposed_count = 0;
    for (unsigned index = 0; index < GROUND_PARAMETER_COUNT; ++index) {
        if (parameter_is_exposed(index))
            exposed_indices[exposed_count++] = (uint16_t)index;
    }
}

void parameter_store_init(const ProductControllerConfigPayload *initial)
{
    /* Start from the catalog defaults so parameters that are not part of the
     * controller payload (calibration, RC trims) keep their declared value.
     * The boot payload then overrides only the fields it actually carries. */
    for (unsigned index = 0; index < GROUND_PARAMETER_COUNT; ++index)
        values[index] = ground_parameters[index].value;
    ground_values_from_payload(values, initial);
    build_exposed_indices();
    apply_hook = 0;
    value_hook = 0;
}

unsigned parameter_store_exposed_count(void)
{
    if (exposed_count == 0u)
        build_exposed_indices();
    return exposed_count;
}

unsigned parameter_store_exposed_at(unsigned ordinal)
{
    if (exposed_count == 0u)
        build_exposed_indices();
    return ordinal < exposed_count ? exposed_indices[ordinal] : UINT16_MAX;
}

int parameter_store_find_exposed(const char *name)
{
    const int index = parameter_store_find(name);
    return index >= 0 && parameter_is_exposed((unsigned)index) ? index : -1;
}

int parameter_store_exposed_ordinal(unsigned index)
{
    if (exposed_count == 0u)
        build_exposed_indices();
    for (unsigned ordinal = 0; ordinal < exposed_count; ++ordinal) {
        if (exposed_indices[ordinal] == index)
            return (int)ordinal;
    }
    return -1;
}

unsigned parameter_store_count(void)
{
    return GROUND_PARAMETER_COUNT;
}

const char *parameter_store_name(unsigned index)
{
    return index < GROUND_PARAMETER_COUNT ? ground_parameters[index].name : "";
}

float parameter_store_value(unsigned index)
{
    return index < GROUND_PARAMETER_COUNT ? values[index] : 0.0f;
}

float parameter_store_minimum(unsigned index)
{
    return index < GROUND_PARAMETER_COUNT ? ground_parameters[index].minimum : 0.0f;
}

float parameter_store_maximum(unsigned index)
{
    return index < GROUND_PARAMETER_COUNT ? ground_parameters[index].maximum : 0.0f;
}

int parameter_store_writable(unsigned index)
{
    return index < GROUND_PARAMETER_COUNT && ground_parameters[index].writable;
}

int parameter_store_find(const char *name)
{
    for (unsigned index = 0; index < GROUND_PARAMETER_COUNT; ++index) {
        if (!strncmp(name, ground_parameters[index].name, 16))
            return (int)index;
    }
    return -1;
}

void parameter_store_set_apply_hook(int (*hook)(const ProductControllerConfigPayload *payload))
{
    apply_hook = hook;
}

void parameter_store_set_value_hook(void (*hook)(unsigned index))
{
    value_hook = hook;
}

void parameter_store_read_payload(ProductControllerConfigPayload *payload)
{
    if (payload)
        ground_payload_from_values(payload, values);
}

int parameter_store_store(unsigned index, float value)
{
    if (index >= GROUND_PARAMETER_COUNT || !parameter_store_writable(index))
        return 0;
    if (!isfinite(value) || value < ground_parameters[index].minimum ||
        value > ground_parameters[index].maximum)
        return 0;
    values[index] = value;
    return 1;
}

int parameter_store_set(unsigned index, float value)
{
    ProductControllerConfigPayload payload = {0};
    if (index >= GROUND_PARAMETER_COUNT || !parameter_store_writable(index))
        return 0;
    if (!isfinite(value) || value < ground_parameters[index].minimum ||
        value > ground_parameters[index].maximum)
        return 0;
    const char *name = ground_parameters[index].name;
    if (strncmp(name, "RC_MODE_", 8u) == 0 || strncmp(name, "RC_AUX_", 7u) == 0) {
        if (value != truncf(value) || (strcmp(name, "RC_MODE_CH") == 0 && value > 0 && value < 5))
            return 0;
        values[index] = value;
        return 1;
    }
    if (values[index] == value)
        return 1;
    const float previous = values[index];
    values[index] = value;
    ground_payload_from_values(&payload, values);
    /* A rejected apply (armed, or an invalid combination such as a
     * non-positive-definite inertia tensor) rolls the single value back.
     * A deferred apply (runtime not started yet) keeps the session value; the
     * next runtime start consumes the compiled payload, so no active control
     * path ever sees an unapplied value. */
    const int applied = apply_hook ? apply_hook(&payload) : 0;
    if (applied == 1) {
        values[index] = previous;
        return 0;
    }
    if (value_hook)
        value_hook(index);
    return 1;
}
