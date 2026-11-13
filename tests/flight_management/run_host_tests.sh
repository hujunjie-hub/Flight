#!/usr/bin/env bash
# Host build + test run for T191-C-FMS (M4/FMS).
# Host tests are not target evidence; see the delivery report for NOT_RUN items.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
while [[ ! -f "$root/SConstruct" ]]; do
  [[ "$root" != / ]] || { echo "Flight root not found" >&2; exit 1; }
  root="$(dirname "$root")"
done
cd "${root}"

out="build/m4_host"
mkdir -p "${out}"

# Regenerate nothing: the production contract comes from middleware/.
cflags=(
  -std=c11 -Wall -Wextra -Werror
  -Imiddleware/model/fms
  -Imiddleware/model/fms/quad_x4
  -Imiddleware/contracts
  -Itests/flight_management
)

gcc "${cflags[@]}" ${EXTRA_CFLAGS:-} \
  middleware/model/fms/fms_interface.c \
  middleware/model/fms/geofence_geometry/geofence_geometry.c \
  middleware/model/fms/quad_x4/flight_state_machine/fms.c \
  middleware/model/fms/quad_x4/mission_lifecycle/mission_lifecycle.c \
  middleware/model/fms/quad_x4/operator_authority/operator_authority.c \
  middleware/model/fms/quad_x4/flight_safety/actuation_permit.c \
  middleware/model/fms/quad_x4/flight_safety/ground_stop_adapter.c \
  middleware/model/fms/quad_x4/flight_safety/fms_safety.c \
  middleware/model/fms/quad_x4/flight_policy/quad_x4_fms.c \
  middleware/model/fms/quad_x4/flight_policy/quad_x4_product_config.c \
  tests/flight_management/test_m4_main.c \
  -lm -o "${out}/m4_host_tests"

"${out}/m4_host_tests"
