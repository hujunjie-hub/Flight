#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
out="${OUT_DIR:-$root/build/validation/sim/sil}"
mkdir -p "$out/obj"
log="$out/sil_flight.log"
exec > >(tee "$log") 2>&1

cc="${CC:-gcc}"
cxx="${CXX:-g++}"
includes=(
  tests/runtime/stubs applications/tasks applications/runtime
  middleware/contracts middleware/communication middleware/protocol middleware/param
  middleware/sensor middleware/model/ins middleware/model/fms middleware/model/fms/quad_x4
  middleware/guidance middleware/model/control middleware/model/control/inner_loop/quad_x4
  middleware/actuator sensor_device sensor_device/time_base sensor_device/actuator
  libraries/HAL_Drivers/drivers sensor_device/imu sensor_device/mag sensor_device/barometer
  sensor_device/pmu sensor_device/gnss middleware/model/ins/kf_gins
  middleware/model/ins/kf_gins/trimmed_core/include
  middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated
  middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/inc/public
  middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/inc/private
  board
)
include_flags=()
for dir in "${includes[@]}"; do include_flags+=("-I$root/$dir"); done

sources=()
shopt -s nullglob
patterns=(
  "middleware/sensor/*/*.c"
  "middleware/model/fms/*.c"
  "middleware/model/fms/quad_x4/*/*.c"
  "middleware/guidance/*/*.c"
  "middleware/model/control/*.c"
  "middleware/model/ins/*.c"
  "middleware/model/control/inner_loop/quad_x4/*/*.c"
  "middleware/model/control/outer_loop/quad_x4/*/*.c"
  "middleware/model/ins/kf_gins/navigation_estimator/*.cpp"
  "middleware/model/ins/kf_gins/observation_replay/*.cpp"
  "middleware/model/ins/kf_gins/trimmed_core/src/*.cpp"
  "middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/src/*.c"
)
for pattern in "${patterns[@]}"; do
  for source in "$root"/$pattern; do
    [[ -f "$source" ]] && sources+=("$source")
  done
done
sources+=(
  "$root/middleware/model/fms/geofence_geometry/geofence_geometry.c"
  "$root/middleware/model/ins/kf_gins/backend_adapter/kf_gins_adapter.cpp"
  "$root/middleware/model/plant/plant_interface.c"
  "$root/middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/quad_workspace.c"
  "$root/middleware/protocol/mavlink/mavlink_frame.c"
  "$root/middleware/protocol/mavlink/mavlink_action.c"
  "$root/middleware/protocol/mavlink/mavlink_hil.c"
  "$root/middleware/param/parameter_store.c"
  "$root/middleware/communication/sim/fmt_sim_host.c"
  "$root/middleware/protocol/unicore_ascii/unicore_ascii.c"
  "$root/middleware/actuator/device_adapter/actuator_device.c"
  "$root/middleware/actuator/output_guard/actuator_guard.c"
  "$root/libraries/HAL_Drivers/drivers/drv_wdt.c"
  "$root/sensor_device/gnss/um982.c"
  "$root/sensor_device/imu/adis16505.c"
  "$root/sensor_device/barometer/bmp585.c"
  "$root/sensor_device/telemetry/elrs/elrs.c"
  "$root/sensor_device/telemetry/data_radio/data_radio.c"
  "$root/tests/simulation/sil_flight.c"
)

declare -A seen=()
objects=()
echo "== SIL compile: ${#sources[@]} translation units =="
for source in "${sources[@]}"; do
  [[ -f "$source" ]] || { echo "missing source: $source"; exit 1; }
  [[ -n "${seen[$source]:-}" ]] && continue
  seen[$source]=1
  relative="${source#"$root"/}"
  object="$out/obj/$(printf '%s' "$relative" | tr '/\\' '__').o"
  if [[ "$source" == *.cpp ]]; then
    compiler="$cxx"
    flags=(-std=c++14 -fno-exceptions -fno-rtti -fno-threadsafe-statics)
  else
    compiler="$cc"
    flags=(-std=c99)
  fi
  flags+=(-O2 -g -Wall -Wextra -Wno-unused-function -fno-short-enums
          -D_POSIX_C_SOURCE=200809L -DFLIGHT_HOST_TEST -DFLIGHT_HOST_SIL)
  if [[ "$relative" != *osqp* ]]; then flags+=(-Werror); fi
  case "$(basename "$source")" in
    adis16505.c|bmp585.c) flags+=(-DFLIGHT_DRIVER_CODEC_TEST) ;;
  esac
  "$compiler" "${flags[@]}" "${include_flags[@]}" -c "$source" -o "$object"
  objects+=("$object")
done

echo "== SIL link =="
exe="$out/sil_flight"
"$cxx" -pthread "${objects[@]}" -lm -o "$exe"

echo "== SIL live FMS DISARM-to-M7 task integration =="
"$exe" --live-fms-ground-stop-smoke
echo "SIL live FMS DISARM-to-M7 task integration: PASS"

echo "== SIL live FMS first-reference timeout-to-M7 task integration =="
"$exe" --live-fms-first-reference-ground-stop-smoke
echo "SIL live FMS first-reference timeout-to-M7 task integration: PASS"

echo "== SIL ground-stop ramp smoke =="
"$exe" --ground-stop-smoke
echo "SIL runtime ground-stop ramp integration: PASS"

echo "== SIL mission staging ownership =="
"$exe" --mission-staging-smoke
echo "SIL mission staging ownership: PASS"
