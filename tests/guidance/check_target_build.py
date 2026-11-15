"""Build the existing SCons firmware with M4/M5 enabled, without changing defaults.

The temporary rtconfig.h overlay is restored in finally. Explicit function
roots ensure both production Ports are in the firmware. The check exercises
compilation, linking and the FLIGHT-14 diagnostic baseline.

Exit codes: 0 = firmware linked with verified enum diagnostics, 1 = build or
diagnostic check failed, 2 = BLOCKED by missing or unverified RT-Thread packages.
"""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
sys.path.insert(0, str(ROOT))
from tools.check_enum_link import KNOWN_WARNING, check_files

OUT = ROOT / "build/m4_m5_implementation/target"
SCOPE = "FIRMWARE_LINK_WITH_M4_M5_PORT_ROOTS"


def blocked(reason):
    OUT.mkdir(parents=True, exist_ok=True)
    report = {"status": "BLOCKED", "blocked_reason": reason,
        "scope": SCOPE,
        "hardware_execution": "NOT_RUN"}
    (OUT / "target_build.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 2


def resolve_arm_bin():
    configured = os.environ.get("FLIGHT_ARM_BIN")
    if configured:
        return configured
    candidate = Path(r"D:\STM32CubeCLT\GNU-tools-for-STM32\bin")
    return str(candidate) if candidate.is_dir() else ""


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    # packages/ is .gitignored upstream (RT-Thread package manager output), so a
    # fresh clone cannot run the root SConscript. Report that explicitly
    # instead of surfacing an opaque "missing SConscript" build error.
    if not (ROOT / "packages/SConscript").is_file():
        return blocked(
            "packages/SConscript is absent: the RT-Thread package tree is "
            "gitignored and is not produced by this repository. Run the "
            "RT-Thread package manager (pkgs --update) for the selected "
            ".config before the firmware link check.")
    lock_check = subprocess.run([sys.executable, "tools/check_rt_packages_lock.py", "--installed"],
                                cwd=ROOT, capture_output=True, text=True)
    if lock_check.returncode:
        return blocked("Installed RT-Thread packages failed lock verification:\n" +
                       lock_check.stdout + lock_check.stderr)
    python = ROOT / "build/m4_m5_tools/Scripts/python.exe"
    if not python.is_file():
        raise SystemExit("Install SCons in build/m4_m5_tools first (see implementation report)")
    header = ROOT / "rtconfig.h"
    original = header.read_bytes()
    overlay = original + b"\n#define FMS_ENABLE\n#define GUIDANCE_ENABLE\n"
    artifacts = {}
    for name in ("Flight.elf", "Flight.bin", "Flight.map"):
        path = ROOT / name
        artifacts[name] = path.read_bytes() if path.exists() else None
    env = os.environ.copy()
    env["RTT_CC"] = "gcc"
    env["RTT_BUILD"] = "release"
    env["RTT_EXEC_PATH"] = resolve_arm_bin()
    command = [str(python), "-c",
        "import rtconfig; rtconfig.LFLAGS += ' -Wl,-u,fms_policy_update,-u,fms_policy_init,-u,guidance_update,-u,guidance_init'; "
        "from SCons.Script import main; main()", "-Q", "-j4"]
    enum_link = {"status": "NOT_RUN"}
    try:
        header.write_bytes(overlay)
        # Ensure diagnostics belong to this invocation, even on a repeated run.
        (ROOT / "Flight.elf").unlink(missing_ok=True)
        result = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True)
        log = OUT / "scons.log"
        log.write_text(result.stdout + result.stderr, encoding="utf-8")
        if result.returncode == 0:
            try:
                for name in artifacts:
                    shutil.copyfile(ROOT / name, OUT / name)
                enum_link = check_files(log, OUT / "Flight.map")
            except (OSError, ValueError) as exc:
                enum_link = {"status": "FAIL", "problems": [str(exc)]}
        (OUT / "enum_link.json").write_text(json.dumps(enum_link, indent=2), encoding="utf-8")
    finally:
        if header.read_bytes() == overlay:
            header.write_bytes(original)
        else:
            raise RuntimeError("rtconfig.h changed concurrently; left it intact")
        for name, previous in artifacts.items():
            if previous is not None:
                (ROOT / name).write_bytes(previous)
            elif (ROOT / name).is_file():
                (ROOT / name).unlink()  # exact, named generated output only
    exit_code = int(result.returncode != 0 or enum_link["status"] != "PASS")
    report = {"status": "FAIL" if exit_code else "PASS", "command": command,
        "exit_code": exit_code, "scons_exit_code": result.returncode,
        "rtconfig_restored": header.read_bytes() == original,
        "rtconfig_sha256": hashlib.sha256(original).hexdigest(),
        "enum_link_status": enum_link["status"],
        "enum_warning_count": enum_link.get("enum_warning_count"),
        "enum_link_problems": enum_link.get("problems", []),
        "enum_link_report": "enum_link.json",
        "scope": SCOPE,
        "hardware_execution": "NOT_RUN"}
    (OUT / "target_build.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(result.stdout[-1600:], end="")
    diagnostics = result.stderr.splitlines()
    if enum_link["status"] == "PASS":
        diagnostics = [line for line in diagnostics if not KNOWN_WARNING.fullmatch(line)]
        print(f"FLIGHT-14: verified {enum_link['enum_warning_count']} enum diagnostics; full output in {log}")
    if diagnostics:
        print("\n".join(diagnostics))
    print(json.dumps(report, indent=2))
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
