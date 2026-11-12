#!/usr/bin/env bash
# Cortex-M7 compile checks for the production M4/M5 sources and for the
# Host/SIL-only M5 advanced-planner translation units (local map, A*, time
# scaler, smoother, minimum-jerk kernel and the M5-06 safety validator).
# These objects are NOT linked into the product; this is architecture and
# compiler-compatibility evidence only, not a target link, WCET, runtime stack
# watermark, RTA, HIL or flight result.
#
# The object set is intentionally the full middleware/guidance tree so that
# tests/guidance/check_memory_budget.py is charged with the same translation
# units that this script compiles.
set -euo pipefail

root="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$root"

cc="$(printenv ARM_GCC || printf '%s' arm-none-eabi-gcc)"
size="$(printenv ARM_SIZE || printf '%s' arm-none-eabi-size)"
objdump="$(printenv ARM_OBJDUMP || printf '%s' arm-none-eabi-objdump)"
# Default to the product release optimisation level; measuring stack at the
# compiler default (-O0) is not representative of the shipped code.
opt="${ARM_OPT:--O2}"
command -v "$cc" >/dev/null 2>&1 || {
  echo "ARM compiler not found: $cc" >&2
  exit 2
}
command -v "$size" >/dev/null 2>&1 || {
  echo "ARM size tool not found: $size" >&2
  exit 2
}
command -v "$objdump" >/dev/null 2>&1 || {
  echo "ARM objdump not found: $objdump" >&2
  exit 2
}

out="build/m5_arm"
mkdir -p "$out"

flags=(
  -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
  -std=c11 -Wall -Wextra -Werror -fno-short-enums -fstack-usage
  -fno-fast-math -fno-unsafe-math-optimizations
  -ffp-contract=off -frounding-math -fexcess-precision=standard
  -Imiddleware/contracts
  -Imiddleware/model/fms
  -Imiddleware/guidance
)

# Sources compiled into the H723/NONE product slice.
production_sources=(
  middleware/model/fms/geofence_geometry/geofence_geometry.c
  middleware/guidance/guidance_update/guidance.c
  middleware/guidance/trajectory_planning/trajectory_math.c
  middleware/guidance/guidance_update/guidance_input_checks.c
  middleware/guidance/trajectory_planning/mission_trajectory.c
  middleware/guidance/pilot_and_landing/pilot_and_landing.c
  middleware/guidance/trajectory_planning/trajectory_validation.c
  middleware/guidance/guidance_update/guidance_config.c
)

# Host/SIL-only M5 advanced planner and M5-06 validator translation units.
# They are compiled for Cortex-M7 only to prove compiler/architecture
# compatibility; the macro keeps their production guards satisfied.
host_only_sources=(
  middleware/guidance/trajectory_planning/m5_local_map.c
  middleware/guidance/trajectory_planning/m5_kinodynamic_astar.c
  middleware/guidance/trajectory_planning/m5_time_scaler.c
  middleware/guidance/trajectory_planning/m5_path_smoother.c
  middleware/guidance/trajectory_planning/minimum_jerk_optimizer.c
  middleware/guidance/trajectory_planning/m5_trajectory_safety.c
)

objects=()
for src in "${production_sources[@]}"; do
  obj="$out/$(basename "$src" .c).o"
  echo "CC $src"
  "$cc" "${flags[@]}" "$opt" -c "$src" -o "$obj"
  objects+=("$obj")
done

for src in "${host_only_sources[@]}"; do
  obj="$out/$(basename "$src" .c).o"
  echo "CC $src (Host/SIL-only source, Cortex-M7 compile check only)"
  "$cc" "${flags[@]}" "$opt" -DFLIGHT_M5_HOST_PLANNER -c "$src" -o "$obj"
  objects+=("$obj")
done

# Disassemble in the same build step as the object so the stack analyzer can
# never consume a .dis older than the .o it describes.
"$objdump" -d -C "$out/m5_trajectory_safety.o" > "$out/m5_trajectory_safety.dis"
dis_hash="$(sha256sum "$out/m5_trajectory_safety.dis" | cut -d' ' -f1)"

{
  echo "commit=${FLIGHT_CODE_COMMIT:-$(git rev-parse HEAD)}"
  echo "source_sha256=$(sha256sum middleware/guidance/trajectory_planning/m5_trajectory_safety.c | cut -d' ' -f1)"
  echo "compiler=$("$cc" --version | head -n 1)"
  echo "size_tool=$("$size" --version | head -n 1)"
  echo "optimization=$opt"
  echo "disassembly=m5_trajectory_safety.dis"
  echo "disassembly_sha256=$dis_hash"
  echo "production_objects=${#production_sources[@]}"
  echo "host_only_objects=${#host_only_sources[@]}"
  printf 'flags='
  printf '%s ' "${flags[@]}"
  echo
  echo "M5-06 scope=Host/SIL-only validator object cross-compiled for Cortex-M7; not linked into product"
} > "$out/toolchain.txt"

"$size" "${objects[@]}"
echo "M5_CORTEX_M7_COMPILE_OK"
echo "Static .su files: $out/*.su"
echo "Toolchain record: $out/toolchain.txt"
