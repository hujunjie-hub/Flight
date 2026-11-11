#!/usr/bin/env python3
"""Run the R28 local regression and write strictly wrapped raw logs.

Every log contains the exact RUN_ID, code commit, command argv, exit code and
duration, followed by the untouched stdout/stderr. The collector refuses any
log without this wrapper. Host/SIL and Cortex-M7 compile results are not H723
runtime, HIL or flight evidence.
"""
import argparse
import datetime
import hashlib
import json
import os
import platform
import subprocess
import sys
import time
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "SConstruct").is_file())
OUT = ROOT / "build/m4_m5_implementation/evidence"
REGEN = ROOT / "build/validation/regenerated"
SANITIZER = "-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer"


def run(command, env=None):
    started = time.monotonic()
    result = subprocess.run(command, cwd=ROOT, env=env or os.environ.copy(),
                            capture_output=True, text=True, errors="replace")
    return result, time.monotonic() - started


def status_for(name, exit_code):
    if exit_code == 0:
        return "PASS"
    if exit_code == 2 and name in ("scons_link", "stack_check"):
        return "BLOCKED"
    return "FAIL"


def toolchain_for(name):
    if name in ("m4_arm", "m5_arm"):
        return "arm-none-eabi-gcc 14.3.1 (GNU Tools for STM32 14.3.rel1.20251027-0700)"
    if name.startswith("s06_") or name in ("m5_host", "m5_sanitizer", "integration",
                                           "integration_sanitizer"):
        return "WSL2 openEuler 6.18.58.1 / gcc 16.2.0 / Python 3.14.10"
    if name in ("collector_selftest", "collector_selftest_opt",
                "evidence_checker_selftest", "product_check", "manifest_check",
                "issues_check", "stack_check", "stack_selftest"):
        return "Windows 11 10.0.26300 / Python 3.14.8 / PowerShell 7.6.9"
    return "Windows 11 10.0.26300 / Python 3.14.8"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--code-commit", required=True)
    parser.add_argument("--output", default=str(OUT / "validation.json"))
    args = parser.parse_args()
    if not args.run_id.startswith("M5-06-"):
        raise SystemExit("unexpected RUN_ID")
    if len(args.code_commit) != 40:
        raise SystemExit("code commit must be 40-hex")
    OUT.mkdir(parents=True, exist_ok=True)
    REGEN.mkdir(parents=True, exist_ok=True)

    wsl = ["wsl.exe", "-d", "Ubuntu-26.04", "--"]
    py = sys.executable
    commands = [
        ("T01", "line_endings", [py, "tools/check_line_endings.py"], {}),
        ("T02", "python_o_negative", [py, "tests/guidance/test_audit_optimized.py"], {}),
        ("T03", "m4_host", wsl + ["bash", "tests/flight_management/run_host_tests.sh"], {}),
        ("T04", "m5_host", wsl + ["bash", "tests/guidance/run_host_tests.sh"], {}),
        ("T05", "integration", wsl + ["bash", "tests/guidance/run_integration_tests.sh"], {}),
        ("T06", "m5_sanitizer", wsl + ["env", "EXTRA_CFLAGS=" + SANITIZER,
                                       "bash", "tests/guidance/run_host_tests.sh"], {}),
        ("T07", "integration_sanitizer", wsl + ["env", "EXTRA_CFLAGS=" + SANITIZER,
                                                "bash", "tests/guidance/run_integration_tests.sh"], {}),
        ("T08", "m4_arm", ["pwsh", "-NoProfile", "-File",
                           "tests/flight_management/arm_build_check.ps1"], {}),
        ("T09", "m5_arm", ["pwsh", "-NoProfile", "-File",
                           "tests/guidance/arm_build_check.ps1"], {}),
        ("T10", "consumer_fp", [py, "tests/guidance/check_consumer_fp_contraction.py"], {}),
        ("T11", "fp_model_selftest", [py, "tests/guidance/check_consumer_fp_contraction.py",
                                      "--self-test"], {}),
        ("T12", "spec_layout", [py, "spec/tools/audit_spec.py", "--round", "3",
                                "--label", "m4-m5-implementation"], {}),
        ("T13", "memory_budget", [py, "tests/guidance/check_memory_budget.py"], {}),
        ("T14", "packages_lock", [py, "tools/check_rt_packages_lock.py"], {}),
        ("T15", "packages_lock_unit", [py, "tests/guidance/test_rt_packages_lock.py"], {}),
        ("T16", "rebaseline_proposal", [py, "tools/analyze_eol_rebaseline.py"], {}),
        ("T17", "production_abi", [py, "tests/abi/abi_report.py"], {}),
        ("T18", "scons_link", [py, "tests/guidance/check_target_build.py"], {}),
        ("T19", "s06_112", wsl + ["python3", "tests/guidance/verify_jerk_continuous.py",
                                  "--library", "build/m5_host/libm5_safety_oracle.so",
                                  "--run-id", args.run_id,
                                  "--code-commit", args.code_commit,
                                  "--seed", "0x53313632",
                                  "--output", "build/validation/regenerated/s06_112_r28.json"], {}),
        ("T20", "collector_selftest", [py, "tools/collect_validation_results.py",
                                       "--self-test"], {}),
        ("T21", "collector_selftest_opt", [py, "-O", "tools/collect_validation_results.py",
                                           "--self-test"], {}),
        ("T22", "evidence_checker_selftest", [py, "tools/check_validation_evidence.py",
                                              "--self-test"], {}),
        ("T23", "product_check", [py, "tools/check_validation_evidence.py",
                                  "--check", "product"], {}),
        ("T24", "manifest_check", [py, "tools/check_validation_evidence.py",
                                   "--check", "manifest"], {}),
        ("T25", "issues_check", [py, "tools/check_validation_evidence.py",
                                 "--check", "issues"], {}),
        ("T26", "stack_check", [py, "tools/check_validation_evidence.py",
                                "--check", "stack"], {}),
        ("T27", "stack_selftest", [py, "tools/analyze_stack.py", "--self-test"], {}),
    ]
    results = []
    for test_id, name, command, additions in commands:
        env = os.environ.copy()
        env.update(additions)
        result, duration = run(command, env)
        output = result.stdout + result.stderr
        log = OUT / (name + ".log")
        header = "\n".join([
            "# FLIGHT_VALIDATION_LOG_V1",
            f"RUN_ID={args.run_id}",
            f"CODE_COMMIT={args.code_commit}",
            "COMMAND_JSON=" + json.dumps(command, separators=(",", ":")),
            f"EXIT_CODE={result.returncode}",
            f"DURATION_SECONDS={duration:.6f}",
            "# BEGIN_OUTPUT",
        ])
        log.write_text(header + "\n" + output + "\n# END_OUTPUT\n", encoding="utf-8")
        status = status_for(name, result.returncode)
        results.append({
            "test_id": test_id,
            "name": name,
            "command": command,
            "exit_code": result.returncode,
            "status": status,
            "log": str(log.relative_to(ROOT)).replace("\\", "/"),
            "duration_seconds": round(duration, 6),
            "toolchain_version": toolchain_for(name),
        })
        print(f"{test_id} {name}: {status} exit={result.returncode} "
              f"duration={duration:.3f}s", flush=True)
        if result.returncode:
            print(output[-1800:], flush=True)
    report = {
        "schema_version": "2.0",
        "run_id": args.run_id,
        "code_commit": args.code_commit,
        "git_head": subprocess.check_output(
            ["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip(),
        "date": datetime.date.today().isoformat(),
        "profile": "QUAD_X4/NONE",
        "scope": "HOST_SIL_ARM_COMPILE_FIRMWARE_LINK_AND_EVIDENCE_TOOLS",
        "commands": results,
        "hardware_execution": "NOT_RUN",
        "hil": "NOT_RUN",
        "wcet": "NOT_RUN",
        "stack_watermark": "NOT_RUN",
        "status": ("FAIL" if any(r["status"] == "FAIL" for r in results)
                   else "BLOCKED" if any(r["status"] == "BLOCKED" for r in results)
                   else "PASS"),
    }
    Path(args.output).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"VALIDATION_WROTE {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
