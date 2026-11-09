/*
 * Boundary check: the plugin-private KF-GINS_QUAD ABI must coexist with the
 * production public contract (modules, owned by PLATFORM) and
 * must agree on the frozen baseline values.
 *
 * This is a compile/link level check only.  Binding estimator_*() to the core
 * (ImuSample / SensorMeasurement / NavigationSnapshot marshalling, M2
 * calibration and installation rotation) belongs to T191-B-ADAPTER and is not
 * claimed here.
 */

#include <cstdio>
#include <cstring>

#include "base_types/contract_version.h"
#include "ports/port_api.h"

#include "backend_adapter/kf_gins_quad_port.h"

static_assert(CONTRACT_SCHEMA_MAJOR == 6u, "unexpected public schema major");
static_assert(CONTRACT_SCHEMA_MINOR == 0u, "unexpected public schema minor");
static_assert(PORT_API_REVISION == PRODUCT_CONTRACT_REVISION,
              "public Port and selected product revisions must agree");
static_assert(CONTRACT_HEADER_BYTES == 88u, "unexpected ContractHeader size");

int main() {
    std::printf("public contract : schema %u.%u, Port 0x%08x, ContractHeader %u B\n",
                static_cast<unsigned>(CONTRACT_SCHEMA_MAJOR),
                static_cast<unsigned>(CONTRACT_SCHEMA_MINOR),
                static_cast<unsigned>(PORT_API_REVISION),
                static_cast<unsigned>(CONTRACT_HEADER_BYTES));
    std::printf("plugin ABI      : 0x%08x, handle %u B, align %u B\n",
                static_cast<unsigned>(kfg_quad_abi_version()),
                static_cast<unsigned>(kfg_quad_sizeof()),
                static_cast<unsigned>(kfg_quad_alignof()));
    std::printf("sizeof(ImuSample)=%u sizeof(GnssSample)=%u\n", static_cast<unsigned>(sizeof(ImuSample)),
                static_cast<unsigned>(sizeof(GnssSample)));

    /* The public Port declares estimator_*(); the plugin must not define any
     * symbol from that namespace.  Taking the address here only proves the
     * declaration is visible to the same translation unit. */
    const void *estimator_api = reinterpret_cast<const void *>(&estimator_init);
    std::printf("estimator_init visible: %s\n", estimator_api != nullptr ? "yes" : "no");

    return 0;
}
