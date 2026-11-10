#!/usr/bin/env bash
# Host build + test run for T191-C-M5 (M5 Guidance, QUAD_X4 NONE-obstacle slice).
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
while [[ ! -f "$root/SConstruct" ]]; do
  [[ "$root" != / ]] || { echo "Flight root not found" >&2; exit 1; }
  root="$(dirname "$root")"
done
cd "${root}"

out="build/m5_host"
mkdir -p "${out}"

cflags=(
  -std=c11 -Wall -Wextra -Werror
  -fno-fast-math -fno-unsafe-math-optimizations
  -ffp-contract=off -frounding-math -fexcess-precision=standard
  -DFLIGHT_M5_HOST_PLANNER -DFLIGHT_M5_SAFETY_TEST_API
  -Imiddleware/model/fms
  -Imiddleware/contracts
  -Imiddleware/guidance
  -Itests/guidance
)

gcc "${cflags[@]}" ${EXTRA_CFLAGS:-} \
  middleware/model/fms/geofence_geometry/geofence_geometry.c \
  middleware/guidance/*/*.c \
  tests/guidance/test_m5_main.c \
  -lm -o "${out}/m5_host_tests"

# Build an uninstrumented shared object for the independent Python Fraction
# oracle. The primary C suite above still runs with EXTRA_CFLAGS (including
# ASan/UBSan in the sanitizer job); loading an ASan DSO into Python would
# otherwise depend on runtime preload order.
gcc "${cflags[@]}" -fPIC -shared \
  middleware/model/fms/geofence_geometry/geofence_geometry.c \
  middleware/guidance/*/*.c \
  -lm -o "${out}/libm5_safety_oracle.so"

"${out}/m5_host_tests"

python3 tests/guidance/verify_norm_bounds.py "${out}/libm5_safety_oracle.so"
