#!/usr/bin/env bash
# Cortex-M7 compile-only check for the real M7 actuator output guard.
# The RT-Thread device adapter and STM32 HAL transport are not included here.
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$root"
cc="${ARM_GCC:-arm-none-eabi-gcc}"
size="${ARM_SIZE:-arm-none-eabi-size}"
command -v "$cc" >/dev/null 2>&1 || { echo "ARM compiler not found: $cc" >&2; exit 2; }
command -v "$size" >/dev/null 2>&1 || { echo "ARM size tool not found: $size" >&2; exit 2; }
out="build/m7_arm"
mkdir -p "$out"
flags=(-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
       -std=c11 -Wall -Wextra -Werror -fno-short-enums -fstack-usage
       -Imiddleware/contracts -Imiddleware/actuator)
src="middleware/actuator/output_guard/actuator_guard.c"
obj="$out/actuator_guard.o"
echo "CC $src"
"$cc" "${flags[@]}" -c "$src" -o "$obj"
"$size" "$obj"
echo "M7_CORTEX_M7_COMPILE_OK"
