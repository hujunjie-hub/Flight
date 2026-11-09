"""Reproduce M4/M5 Host, sanitizer, Cortex-M7, ABI and SCons link evidence."""
from pathlib import Path
import hashlib
import json
import os
import subprocess
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
OUT = ROOT / "build/m4_m5_implementation/evidence"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    wsl = ["wsl.exe", "-d", "Ubuntu-26.04", "--"]
    sanitizer = "-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer"
    commands = [
        ("m4_host", wsl+["bash", "tests/flight_management/run_host_tests.sh"], {}),
        ("m5_host", wsl+["bash", "tests/guidance/run_host_tests.sh"], {}),
        ("integration", wsl+["bash", "tests/guidance/run_integration_tests.sh"], {}),
        ("m5_sanitizer", wsl+["env", "EXTRA_CFLAGS="+sanitizer,
            "bash", "tests/guidance/run_host_tests.sh"], {}),
        ("integration_sanitizer", wsl+["env", "EXTRA_CFLAGS="+sanitizer,
            "bash", "tests/guidance/run_integration_tests.sh"], {}),
        ("m4_arm", ["pwsh", "-NoProfile", "-File", "tests/flight_management/arm_build_check.ps1"], {}),
        ("m5_arm", ["pwsh", "-NoProfile", "-File", "tests/guidance/arm_build_check.ps1"], {}),
        ("spec_layout", [sys.executable, "spec/tools/audit_spec.py", "--round", "3", "--label", "m4-m5-implementation"], {}),
        ("memory_budget", [sys.executable, "tests/guidance/check_memory_budget.py"], {}),
        ("production_abi", [sys.executable, "tests/abi/abi_report.py"], {}),
        ("scons_link", [sys.executable, "tests/guidance/check_target_build.py"], {}),
    ]
    results = []
    for name, command, additions in commands:
        env = os.environ.copy(); env.update(additions)
        result = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True)
        log = OUT / (name+".log")
        log.write_text(result.stdout+result.stderr, encoding="utf-8")
        results.append({"name": name, "command": command, "exit_code": result.returncode,
            "log": log.relative_to(ROOT).as_posix()})
        print(name+": "+("PASS" if result.returncode == 0 else "FAIL"), flush=True)
        if result.returncode: print((result.stdout+result.stderr)[-2400:], flush=True)
    inputs = set((ROOT / "middleware/contracts").rglob("*.h"))
    for module in ("model/fms", "guidance"):
        inputs.update(p for p in (ROOT / "middleware" / module).rglob("*")
                      if p.suffix in (".c", ".h", ".sh", ".ps1", ".py")
                      or p.name in ("Kconfig", "SConscript"))
    files = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
             for p in sorted(inputs)}
    (OUT/"source_sha256.json").write_text(json.dumps(files, indent=2), encoding="utf-8")
    report = {"date": "2026-11-05", "profile": "QUAD_X4/NONE", "port_revision": "0x00010906",
        "scope": "HOST_SANITIZER_ARM_COMPILE_FIRMWARE_LINK", "commands": results,
        "wcet": "NOT_RUN", "hil": "NOT_RUN", "hardware_execution": "NOT_RUN",
        "independent_review": "NOT_RUN", "runtime_dispatch": "NOT_INSTALLED",
        "status": "PASS" if all(r["exit_code"] == 0 for r in results) else "FAIL"}
    (OUT/"validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
