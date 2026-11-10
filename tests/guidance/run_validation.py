"""Reproduce M4/M5 Host, sanitizer, Cortex-M7, ABI and SCons link evidence."""
from pathlib import Path
import datetime
import hashlib
import json
import os
import subprocess
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
OUT = ROOT / "build/m4_m5_implementation/evidence"


def status_for(name, exit_code):
    # Only the firmware link check has a defined blocked state (exit 2): it
    # needs the gitignored RT-Thread packages tree. Any other exit code 2 is a
    # normal failure, not an excuse to skip the check.
    if exit_code == 0:
        return "PASS"
    if exit_code == 2 and name == "scons_link":
        return "BLOCKED"
    return "FAIL"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    wsl = ["wsl.exe", "-d", "Ubuntu-26.04", "--"]
    sanitizer = "-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer"
    commands = [
        ("line_endings", [sys.executable, "tools/check_line_endings.py"], {}),
        ("python_o_negative", [sys.executable, "tests/guidance/test_audit_optimized.py"], {}),
        ("m4_host", wsl+["bash", "tests/flight_management/run_host_tests.sh"], {}),
        ("m5_host", wsl+["bash", "tests/guidance/run_host_tests.sh"], {}),
        ("integration", wsl+["bash", "tests/guidance/run_integration_tests.sh"], {}),
        ("m5_sanitizer", wsl+["env", "EXTRA_CFLAGS="+sanitizer,
            "bash", "tests/guidance/run_host_tests.sh"], {}),
        ("integration_sanitizer", wsl+["env", "EXTRA_CFLAGS="+sanitizer,
            "bash", "tests/guidance/run_integration_tests.sh"], {}),
        ("m4_arm", ["pwsh", "-NoProfile", "-File", "tests/flight_management/arm_build_check.ps1"], {}),
        ("m5_arm", ["pwsh", "-NoProfile", "-File", "tests/guidance/arm_build_check.ps1"], {}),
        ("consumer_fp", [sys.executable, "tests/guidance/check_consumer_fp_contraction.py"], {}),
        ("spec_layout", [sys.executable, "spec/tools/audit_spec.py", "--round", "3", "--label", "m4-m5-implementation"], {}),
        ("memory_budget", [sys.executable, "tests/guidance/check_memory_budget.py"], {}),
        ("packages_lock", [sys.executable, "tools/check_rt_packages_lock.py"], {}),
        ("production_abi", [sys.executable, "tests/abi/abi_report.py"], {}),
        ("scons_link", [sys.executable, "tests/guidance/check_target_build.py"], {}),
    ]
    results = []
    for name, command, additions in commands:
        env = os.environ.copy(); env.update(additions)
        result = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True)
        log = OUT / (name+".log")
        log.write_text(result.stdout+result.stderr, encoding="utf-8")
        status = status_for(name, result.returncode)
        results.append({"name": name, "command": command, "exit_code": result.returncode,
            "status": status, "log": log.relative_to(ROOT).as_posix()})
        print(f"{name}: {status} (exit {result.returncode})", flush=True)
        if result.returncode: print((result.stdout+result.stderr)[-2400:], flush=True)
    inputs = set((ROOT / "middleware/contracts").rglob("*.h"))
    for module in ("model/fms", "guidance"):
        inputs.update(p for p in (ROOT / "middleware" / module).rglob("*")
                      if p.suffix in (".c", ".h", ".sh", ".ps1", ".py")
                      or p.name in ("Kconfig", "SConscript"))
    files = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
             for p in sorted(inputs)}
    (OUT/"source_sha256.json").write_text(json.dumps(files, indent=2), encoding="utf-8")
    statuses = [item["status"] for item in results]
    overall = ("FAIL" if "FAIL" in statuses else
               "BLOCKED" if "BLOCKED" in statuses else "PASS")
    report = {"date": datetime.date.today().isoformat(), "profile": "QUAD_X4/NONE",
        "port_revision": "0x00010906",
        "scope": "HOST_SANITIZER_ARM_COMPILE_FIRMWARE_LINK", "commands": results,
        "wcet": "NOT_RUN", "hil": "NOT_RUN", "hardware_execution": "NOT_RUN",
        "independent_review": "NOT_RUN", "runtime_dispatch": "NOT_INSTALLED",
        "status": overall}
    (OUT/"validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return 0 if overall == "PASS" else (2 if overall == "BLOCKED" else 1)


if __name__ == "__main__":
    sys.exit(main())
