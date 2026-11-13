#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

CC="${CC:-gcc}"
BUILD_DIR="${BUILD_DIR:-build/mr009_focus}"
mkdir -p "$BUILD_DIR"
BIN="$BUILD_DIR/mr009_focus"
LOG="$BUILD_DIR/mr009_focus.log"

CFLAGS=(
  -std=c11 -Wall -Wextra -Werror
  -fno-fast-math -fno-unsafe-math-optimizations
  -ffp-contract=off -frounding-math -fexcess-precision=standard
  -DFLIGHT_M5_HOST_PLANNER -DFLIGHT_M5_SAFETY_TEST_API
  -Imiddleware/model/fms
  -Imiddleware/model/fms/quad_x4
  -Imiddleware/contracts
  -Imiddleware/guidance
  -Itests/guidance
  -Imiddleware/model/control
  -Imiddleware/model/control/inner_loop
  -Imiddleware/model/control/inner_loop/quad_x4
  -Imiddleware/model/control/outer_loop
)

set -o pipefail
"$CC" "${CFLAGS[@]}" ${EXTRA_CFLAGS:-} \
  middleware/model/fms/geofence_geometry/geofence_geometry.c \
  middleware/model/fms/quad_x4/flight_policy/quad_x4_product_config.c \
  middleware/guidance/*/*.c \
  middleware/model/control/outer_loop/quad_x4/position_tracking/control_reference.c \
  tests/guidance/mr009_focus_main.c \
  -lm -o "$BIN"
"$BIN" 2>&1 | tee "$LOG"
