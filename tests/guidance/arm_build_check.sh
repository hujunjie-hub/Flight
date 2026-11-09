#!/usr/bin/env bash
# Portable Cortex-M7 compile check for production M4/M5 sources.
# Compile/stack-usage evidence only; not a target link, WCET, runtime stack
# watermark, RTA, HIL, or flight result.
set -euo pipefail

root="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$root"

cc="$(printenv ARM_GCC || printf '%s' arm-none-eabi-gcc)"
size="$(printenv ARM_SIZE || printf '%s' arm-none-eabi-size)"
command -v "$cc" >/dev/null 2>&1 || {
  echo "ARM compiler not found: $cc" >&2
  exit 2
}
command -v "$size" >/dev/null 2>&1 || {
  echo "ARM size tool not found: $size" >&2
  exit 2
}

out="build/m5_arm"
mkdir -p "$out"

flags=(
  -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
  -std=c11 -Wall -Wextra -Werror -fno-short-enums -fstack-usage
  -Imiddleware/contracts
  -Imiddleware/model/fms
  -Imiddleware/guidance
)

sources=(
  middleware/model/fms/geofence_geometry/geofence_geometry.c
  middleware/guidance/guidance_update/guidance.c
  middleware/guidance/trajectory_planning/trajectory_math.c
  middleware/guidance/guidance_update/guidance_input_checks.c
  middleware/guidance/trajectory_planning/mission_trajectory.c
  middleware/guidance/pilot_and_landing/pilot_and_landing.c
  middleware/guidance/trajectory_planning/trajectory_validation.c
  middleware/guidance/guidance_update/guidance_config.c
)

objects=()
for src in "${sources[@]}"; do
  obj="$out/$(basename "$src" .c).o"
  echo "CC $src"
  "$cc" "${flags[@]}" -c "$src" -o "$obj"
  objects+=("$obj")
done

"$size" "${objects[@]}"
echo "M5_CORTEX_M7_COMPILE_OK"
echo "Static .su files: $out/*.su"
