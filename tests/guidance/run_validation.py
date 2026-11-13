#!/usr/bin/env python3
"""Run the R28 local regression and write strictly wrapped raw logs.

Every log contains the exact RUN_ID, code commit, command argv, exit code and
duration, followed by the untouched stdout/stderr. The collector refuses any
log without this wrapper. Host/SIL and Cortex-M7 compile results are not H723
runtime, HIL or flight evidence.
"""
import argparse
import datetime
import json
import os
import platform
import re
import subprocess
import sys
import time
from pathlib import Path
from shutil import copyfile

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "SConstruct").is_file())
OUT = ROOT / "build/m4_m5_implementation/evidence"
REGEN = ROOT / "build/validation/regenerated"
SANITIZER = "-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer"
WSL = ["wsl.exe", "-d", "Ubuntu-26.04", "--"]
STATUS_EXIT = {"PASS": 0, "FAIL": 1, "BLOCKED": 2, "UNKNOWN": 3}


def run(command, env=None):
    started = time.monotonic()
    result = subprocess.run(command, cwd=ROOT, env=env or os.environ.copy(),
                            capture_output=True, text=True, errors="replace")
    return result, time.monotonic() - started


def status_for(name, exit_code):
    if exit_code == 0:
        return "PASS"
    if exit_code == 2 and name in ("scons_link", "stack_check", "stack_image"):
        return "BLOCKED"
    if exit_code == 3 and name in ("stack_check", "stack_image"):
        return "UNKNOWN"
    return "FAIL"


def toolchain_for(name):
    if name in ("m4_arm", "m5_arm", "stack_image"):
        compiler = os.environ.get("ARM_GCC") or (
            r"D:\STM32CubeCLT\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe")
        return subprocess.check_output([compiler, "--version"], text=True).splitlines()[0]
    if name.startswith("s06_") or name in ("m4_host", "m5_host", "m5_sanitizer",
                                         "integration", "integration_sanitizer"):
        versions = subprocess.check_output(WSL + ["bash", "-c",
            '. /etc/os-release && printf "%s\\n" "$PRETTY_NAME" && '
            'uname -r && gcc -dumpfullversion && python3 --version'], text=True)
        distro, kernel, gcc, python = versions.splitlines()
        return f"WSL {distro} / {kernel} / GCC {gcc} / {python}"
    return f"{platform.system()} {platform.version()} / Python {platform.python_version()}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--code-commit", required=True)
    parser.add_argument("--output", default=str(OUT / "validation.json"))
    parser.add_argument("--only",
                        help="comma-separated command names (default: all)")
    args = parser.parse_args()
    if not re.fullmatch(r"M5-06-[A-Za-z0-9._-]{1,58}", args.run_id):
        parser.error("invalid M5-06 RUN_ID")
    if not re.fullmatch(r"[0-9a-f]{40}", args.code_commit):
        parser.error("code commit must be a full lowercase Git SHA")
    head = subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"],
                                   text=True).strip()
    if args.code_commit != head:
        parser.error(f"code commit must match the checked-out HEAD ({head})")

    wsl = WSL
    py = sys.executable
    manifest = REGEN / f"acceptance_{args.run_id}.json"
    manifest_args = ["--manifest", str(manifest), "--run-id", args.run_id,
                     "--code-commit", head]
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
        ("T14", "packages_lock", [py, "tools/check_rt_packages_lock.py", "--installed"], {}),
        ("T15", "packages_lock_unit", [py, "tests/guidance/test_rt_packages_lock.py"], {}),
        ("T16", "rebaseline_proposal", [py, "tools/analyze_eol_rebaseline.py"], {}),
        ("T17", "production_abi", [py, "tests/abi/abi_report.py"], {}),
        ("T18", "scons_link", [py, "tests/guidance/check_target_build.py"], {}),
        ("T19", "s06_112", wsl + ["python3", "tests/guidance/verify_jerk_continuous.py",
                                  "--library", "build/m5_host/libm5_safety_oracle.so",
                                  "--run-id", args.run_id,
                                  "--code-commit", args.code_commit,
                                  "--seed", "0x53313632",
                                  "--output",
                                  f"build/validation/regenerated/s06_112_{args.run_id}.json"], {}),
        ("T20", "collector_selftest", [py, "tools/collect_validation_results.py",
                                       "--self-test"], {}),
        ("T21", "collector_selftest_opt", [py, "-O", "tools/collect_validation_results.py",
                                           "--self-test"], {}),
        ("T22", "evidence_checker_selftest", [py, "tools/check_validation_evidence.py",
                                              "--self-test"], {}),
        ("T23", "product_check", [py, "tools/check_validation_evidence.py",
                                  "--check", "product"], {}),
        ("T24", "manifest_check", [py, "tools/check_validation_evidence.py",
                                   "--check", "manifest", *manifest_args], {}),
        ("T25", "issues_check", [py, "tools/check_validation_evidence.py",
                                 "--check", "issues"], {}),
        ("T26", "stack_image", ["pwsh", "-NoProfile", "-File",
                                "tests/guidance/stack_image_check.ps1", "-OutJson",
                                f"build/validation/regenerated/stack_image_{args.run_id}.json"], {}),
        ("T27", "stack_check", [py, "tools/check_validation_evidence.py",
                                 "--check", "stack", "--stack",
                                 f"build/validation/regenerated/stack_image_{args.run_id}.json"], {}),
        ("T28", "stack_selftest", [py, "tools/analyze_stack.py", "--self-test"], {}),
        ("T29", "stack_image_selftest", [py, "tools/analyze_stack_image.py",
                                         "--self-test"], {}),
        ("T30", "runner_selftest", [py, "tests/guidance/test_run_validation.py"], {}),
    ]
    if args.only is not None:
        selected = {part.strip() for part in args.only.split(",")}
        unknown = selected - {name for _, name, _, _ in commands}
        if unknown:
            parser.error("unknown or empty test selection: " + ", ".join(map(repr, sorted(unknown))))
        commands = [entry for entry in commands if entry[1] in selected]

    OUT.mkdir(parents=True, exist_ok=True)
    REGEN.mkdir(parents=True, exist_ok=True)
    if any(name == "manifest_check" for _, name, _, _ in commands):
        copyfile(ROOT / "spec/validation/flight_acceptance_manifest.json", manifest)
        preparer = [py, "tools/prepare_archive_metadata.py", "--manifest", str(manifest),
                    "--run-id", args.run_id, "--validated-code", head]
    results = []
    for test_id, name, command, additions in commands:
        env = os.environ.copy()
        env.update(additions)
        if name == "manifest_check":
            result, duration = run(preparer, env)
            if result.returncode:
                command = preparer
            else:
                result, duration = run(command, env)
        else:
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
        try:
            toolchain = toolchain_for(name)
        except (OSError, ValueError, IndexError, subprocess.CalledProcessError) as error:
            toolchain = None
            status = "FAIL"
            print(f"{test_id} {name}: could not identify the toolchain: {error}", flush=True)
        results.append({
            "test_id": test_id,
            "name": name,
            "command": command,
            "exit_code": result.returncode,
            "status": status,
            "log": str(log.relative_to(ROOT)).replace("\\", "/"),
            "duration_seconds": round(duration, 6),
            "toolchain_version": toolchain,
        })
        print(f"{test_id} {name}: {status} exit={result.returncode} "
              f"duration={duration:.3f}s", flush=True)
        if result.returncode:
            print(output[-1800:], flush=True)
    statuses = {result["status"] for result in results}
    status = next((value for value in ("FAIL", "BLOCKED", "UNKNOWN") if value in statuses), "PASS")
    report = {
        "schema_version": "2.0",
        "run_id": args.run_id,
        "code_commit": args.code_commit,
        "git_head": head,
        "date": datetime.date.today().isoformat(),
        "profile": "QUAD_X4/NONE",
        "scope": "HOST_SIL_ARM_COMPILE_FIRMWARE_LINK_AND_EVIDENCE_TOOLS",
        "commands": results,
        "hardware_execution": "NOT_RUN",
        "hil": "NOT_RUN",
        "wcet": "NOT_RUN",
        "stack_watermark": "NOT_RUN",
        "status": status,
    }
    Path(args.output).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"VALIDATION_WROTE {args.output}")
    return STATUS_EXIT[status]


if __name__ == "__main__":
    sys.exit(main())
