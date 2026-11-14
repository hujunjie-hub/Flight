#!/usr/bin/env bash
# Linux/CI runner for the KF-GINS_QUAD Host suite.
#
# Mirrors tests/estimator_backend/run_host_tests.ps1: same compiler flags, same
# source lists, same per-test log layout. The upstream comparison stays SKIPPED
# unless a clean checkout of the locked commit is supplied through
# KFGINS_UPSTREAM (default behaviour of the PowerShell runner as well).
#
# Usage:
#   bash tests/estimator_backend/run_host_tests.sh
#   CXX=g++ KFGINS_UPSTREAM=/path/to/KF-GINS bash tests/estimator_backend/run_host_tests.sh
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
test_dir="$root/tests/estimator_backend"
pkg_dir="$root/middleware/model/ins/kf_gins"
out="${OUT_DIR:-$root/build/validation/kf_gins_upstream}"
work="$out/obj"
mkdir -p "$work"

cxx="${CXX:-g++}"
common=(-std=c++14 -O2 -Wall -Wextra)

trim_include="$pkg_dir/trimmed_core/include"
trim_sources=("$pkg_dir/trimmed_core/src/quad_core.cpp" "$pkg_dir/trimmed_core/src/quad_models.cpp")
plugin_sources=("$pkg_dir/backend_adapter/kf_gins_adapter.cpp" "$pkg_dir/backend_adapter/kf_gins_quad_port.cpp")

pass=0
fail=0
declare -a failed_names=()

run_test() {
    local name="$1"; shift
    local includes=() sources=() extra=()
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --include) includes+=("-I$2"); shift 2 ;;
            --source)  sources+=("$2");   shift 2 ;;
            --flag)    extra+=("$2");     shift 2 ;;
            *) echo "internal error: $1" >&2; exit 2 ;;
        esac
    done

    local exe="$work/$name" log="$out/$name.log"
    local cmd=("$cxx" "${common[@]}" "${extra[@]}" "${includes[@]}" "${sources[@]}" -o "$exe")

    {
        echo "# test: $name"
        echo "# compiler: $cxx"
        echo "# command: ${cmd[*]}"
    } > "$log"

    if ! "${cmd[@]}" >> "$log" 2>&1; then
        echo "# RESULT: BUILD_FAILED" >> "$log"
        echo "$name BUILD_FAILED ($log)"
        fail=$((fail + 1)); failed_names+=("$name")
        return
    fi

    local run_exit=0
    "$exe" >> "$log" 2>&1 || run_exit=$?
    echo "# run exit: $run_exit" >> "$log"
    if [[ $run_exit -eq 0 ]]; then
        echo "# RESULT: PASS" >> "$log"
        grep -E 'checks, [0-9]+ failures|: [0-9]+ checks' "$log" | tail -1 || true
        pass=$((pass + 1))
    else
        echo "# RESULT: FAIL" >> "$log"
        echo "$name FAIL ($log)"
        fail=$((fail + 1)); failed_names+=("$name")
    fi
}

echo "KF-GINS_QUAD host tests"
echo "  repo    : $root"
echo "  compiler: $("$cxx" --version | head -1)"
echo "  logs    : $out"
echo

run_test test_jacobian \
    --include "$trim_include" --include "$test_dir" \
    --source "$test_dir/test_jacobian.cpp" \
    --source "${trim_sources[0]}" --source "${trim_sources[1]}" \
    --flag -fno-exceptions --flag -fno-rtti

run_test test_core \
    --include "$trim_include" --include "$test_dir" \
    --source "$test_dir/test_core.cpp" \
    --source "${trim_sources[0]}" --source "${trim_sources[1]}" \
    --flag -fno-exceptions --flag -fno-rtti

run_test test_port \
    --include "$trim_include" --include "$pkg_dir" --include "$test_dir" \
    --source "$test_dir/test_port.cpp" \
    --source "${plugin_sources[0]}" --source "${plugin_sources[1]}" \
    --source "${trim_sources[0]}" --source "${trim_sources[1]}" \
    --flag -fno-exceptions --flag -fno-rtti

run_test test_nis_recovery \
    --include "$trim_include" --include "$pkg_dir" --include "$test_dir" \
    --source "$test_dir/test_nis_recovery.cpp" \
    --source "${plugin_sources[0]}" --source "${plugin_sources[1]}" \
    --source "${trim_sources[0]}" --source "${trim_sources[1]}" \
    --flag -fno-exceptions --flag -fno-rtti

run_test test_contract_coexistence \
    --include "$root/middleware/contracts" --include "$pkg_dir" --include "$trim_include" \
    --source "$test_dir/test_contract_coexistence.cpp" \
    --source "${plugin_sources[0]}" --source "${plugin_sources[1]}" \
    --source "${trim_sources[0]}" --source "${trim_sources[1]}" \
    --flag -fno-exceptions --flag -fno-rtti

if [[ -n "${KFGINS_UPSTREAM:-}" ]]; then
    run_test test_upstream_compare \
        --include "$trim_include" --include "$test_dir" \
        --include "$KFGINS_UPSTREAM/ThirdParty/eigen-3.3.9" \
        --include "$KFGINS_UPSTREAM/src" \
        --source "$test_dir/test_upstream_compare.cpp" \
        --source "${trim_sources[0]}" --source "${trim_sources[1]}" \
        --flag -fno-exceptions --flag -fno-rtti
else
    echo "SKIPPED test_upstream_compare: set KFGINS_UPSTREAM to a clean checkout of the locked commit"
fi

echo
echo "summary: $pass passed, $fail failed"
if (( fail > 0 )); then
    printf 'failed: %s\n' "${failed_names[*]}"
    exit 1
fi
