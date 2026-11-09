#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
while [[ ! -f "$root/SConstruct" ]]; do
  [[ "$root" != / ]] || { echo "Flight root not found" >&2; exit 1; }
  root="$(dirname "$root")"
done
cd "${root}"
mkdir -p build/m4_m5_host
gcc -std=c11 -Wall -Wextra -Werror ${EXTRA_CFLAGS:-} -DM4_M5_INTEGRATION \
  -Imiddleware/contracts \
  -Imiddleware/model/fms \
  -Imiddleware/model/fms/quad_x4 \
  -Imiddleware/guidance \
  middleware/model/fms/fms_interface.c middleware/model/fms/geofence_geometry/geofence_geometry.c \
  middleware/model/fms/quad_x4/*/*.c middleware/guidance/*/*.c \
  tests/flight_management/test_m4_main.c -lm -o build/m4_m5_host/integration_tests
build/m4_m5_host/integration_tests
