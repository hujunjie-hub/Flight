#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$root"
cc="${CC:-gcc}"
out="${OUT_DIR:-$root/build/validation/m6}"
mkdir -p "$out/obj"

extra_flags=()
if [[ -n "${EXTRA_CFLAGS:-}" ]]; then
  read -r -a extra_flags <<< "$EXTRA_CFLAGS"
fi

model="$root/middleware/model/control"
inner="$model/inner_loop/quad_x4"
outer="$model/outer_loop/quad_x4"
mpc="$outer/mpc_solver"
osqp="$mpc/osqp/generated"

includes=(
  -I"$root/middleware/contracts"
  -I"$root/middleware/actuator"
  -I"$root/middleware"
  -I"$model"
  -I"$inner"
  -I"$outer"
  -I"$mpc"
  -I"$osqp"
  -I"$osqp/inc/public"
  -I"$osqp/inc/private"
)
common=(
  "$outer/position_tracking/control_reference.c"
  "$outer/position_tracking/position_control.c"
  "$inner/attitude_rate_control/controller.c"
  "$inner/attitude_rate_control/attitude_math.c"
  "$inner/control_allocation/control_allocation.c"
)
osqp_sources=( "$osqp"/src/*.c "$osqp/quad_workspace.c" )

build_and_run() {
  local name="$1"
  local test_source="$2"
  local pattern="$3"
  shift 3
  local exe="$out/obj/$name"
  local log="$out/$name.log"

  {
    echo "# test: $name"
    echo "# compiler: $cc"
    echo "# extra flags: ${EXTRA_CFLAGS:-}"
  } > "$log"

  if ! "$cc" -std=c11 -O2 -Wall -Wextra "${extra_flags[@]}" \
      "${includes[@]}" "$test_source" "$@" -lm -o "$exe" >>"$log" 2>&1; then
    cat "$log"
    echo "M6 $name BUILD_FAILED"
    exit 1
  fi

  if ! "$exe" >>"$log" 2>&1; then
    cat "$log"
    echo "M6 $name FAIL"
    exit 1
  fi

  local summary
  summary="$(grep -E "$pattern" "$log" | tail -n 1 || true)"
  if [[ -z "$summary" ]]; then
    cat "$log"
    echo "M6 $name missing check summary"
    exit 1
  fi

  local checks failures
  read -r checks failures <<<"$(sed -E 's/.*checks=([0-9]+) failures=([0-9]+).*/\1 \2/' <<<"$summary")"
  if (( checks <= 0 || failures != 0 )); then
    cat "$log"
    echo "M6 $name failed: $summary"
    exit 1
  fi
  echo "$summary"
}

build_and_run test_m6 "$root/tests/control/test_m6_main.c" \
  'M6 checks=[0-9]+ failures=[0-9]+' \
  "${common[@]}" "$mpc/mpc.c" "${osqp_sources[@]}"

build_and_run test_m6_guard "$root/tests/control/test_m6_guard_main.c" \
  'M6 guard checks=[0-9]+ failures=[0-9]+' \
  "${common[@]}" "$mpc/mpc.c" "${osqp_sources[@]}"

echo "M6 host validation: PASS"
