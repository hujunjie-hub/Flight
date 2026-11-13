#!/usr/bin/env bash
# Cortex-M7 compile-only check for production M4 FMS and ground-stop adapter.
# This is not a full firmware link, WCET/RTA, HIL, or flight result.
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$root"
cc="\${ARM_GCC:-arm-none-eabi-gcc}"
size="\${ARM_SIZE:-arm-none-eabi-size}"
command -v "$cc" >/dev/null 2>&1 || { echo "ARM compiler not found: $cc" >&2; exit 2; }
command -v "$size" >/dev/null 2>&1 || { echo "ARM size tool not found: $size" >&2; exit 2; }
out="build/m4_arm"
mkdir -p "$out"
flags=(-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
       -std=c11 -Wall -Wextra -Werror -fno-short-enums -fstack-usage
       -Imiddleware/contracts -Imiddleware/model/fms -Imiddleware/model/fms/quad_x4)
sources=(
  middleware/model/fms/fms_interface.c
  middleware/model/fms/geofence_geometry/geofence_geometry.c
  middleware/model/fms/quad_x4/flight_state_machine/fms.c
  middleware/model/fms/quad_x4/mission_lifecycle/mission_lifecycle.c
  middleware/model/fms/quad_x4/operator_authority/operator_authority.c
  middleware/model/fms/quad_x4/flight_safety/actuation_permit.c
  middleware/model/fms/quad_x4/flight_safety/fms_safety.c
  middleware/model/fms/quad_x4/flight_safety/ground_stop_adapter.c
  middleware/model/fms/quad_x4/flight_policy/quad_x4_fms.c
)
objects=()
for src in "\${sources[@]}"; do
  obj="$out/$(basename "$src" .c).o"
  echo "CC $src"
  "$cc" "\${flags[@]}" -c "$src" -o "$obj"
  objects+=("$obj")
done
"$size" "\${objects[@]}"
echo "M4_CORTEX_M7_COMPILE_OK"
