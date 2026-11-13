#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$root"
cc="${CC:-gcc}"
out="${OUT_DIR:-$root/build/validation/m7}"
mkdir -p "$out/obj"

extra_flags=()
if [[ -n "${EXTRA_CFLAGS:-}" ]]; then
  read -r -a extra_flags <<< "$EXTRA_CFLAGS"
fi

exe="$out/obj/test_m7"
log="$out/test_m7.log"
{
  echo "# test: test_m7"
  echo "# compiler: $cc"
  echo "# extra flags: ${EXTRA_CFLAGS:-}"
} > "$log"

if ! "$cc" -std=c11 -O2 -Wall -Wextra -Wconversion \
    -DFLIGHT_HOST_TEST "${extra_flags[@]}" \
    -I"$root/middleware/actuator" \
    -I"$root/middleware/contracts" \
    -I"$root/middleware" \
    -I"$root/middleware/model/fms" \
    "$root/tests/actuation/test_m7_main.c" \
    "$root/middleware/actuator/output_guard/actuator_guard.c" \
    "$root/middleware/model/fms/quad_x4/flight_safety/ground_stop_adapter.c" \
    -lm -o "$exe" >>"$log" 2>&1; then
  cat "$log"
  echo "M7 host BUILD_FAILED"
  exit 1
fi

if ! "$exe" >>"$log" 2>&1; then
  cat "$log"
  echo "M7 host FAIL"
  exit 1
fi

summary="$(grep -E 'M7 checks=[0-9]+ failures=[0-9]+' "$log" | tail -n 1 || true)"
if [[ -z "$summary" ]]; then
  cat "$log"
  echo "M7 host missing check summary"
  exit 1
fi

read -r checks failures <<<"$(sed -E 's/.*checks=([0-9]+) failures=([0-9]+).*/\1 \2/' <<<"$summary")"
if (( checks <= 0 || failures != 0 )); then
  cat "$log"
  echo "M7 host failed: $summary"
  exit 1
fi

cat "$log"
echo "M7 host validation: PASS ($summary)"
