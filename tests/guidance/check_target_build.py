"""Build the existing SCons firmware with M4/M5 enabled, without changing defaults.

The temporary rtconfig.h overlay is restored in finally. Function roots keep
both production Ports in the link despite the BSP's empty application loop.
No new thread, hardware output or automatic arming is installed by this check.

Exit codes: 0 = firmware linked, 1 = build failed, 2 = BLOCKED because the
RT-Thread packages tree (packages/SConscript) is not part of the repository.
A blocked check is not a pass; it is reported separately so that CI and local
evidence never claim a firmware link that did not happen.
"""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
OUT = ROOT / "build/m4_m5_implementation/target"


def blocked(reason):
    OUT.mkdir(parents=True, exist_ok=True)
    report = {"status": "BLOCKED", "blocked_reason": reason,
        "scope": "FIRMWARE_LINK_WITH_M4_M5_PORT_ROOTS_NO_RUNTIME_DISPATCH",
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
    env["RTT_EXEC_PATH"] = resolve_arm_bin()
    command = [str(python), "-c",
        "import rtconfig; rtconfig.LFLAGS += ' -Wl,-u,fms_policy_update,-u,fms_policy_init,-u,guidance_update,-u,guidance_init'; "
        "from SCons.Script import main; main()", "-Q", "-j4"]
    result = None
    try:
        header.write_bytes(overlay)
        result = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True)
        (OUT / "scons.log").write_text(result.stdout + result.stderr, encoding="utf-8")
        if result.returncode == 0:
            for name in artifacts:
                if (ROOT / name).is_file():
                    shutil.copyfile(ROOT / name, OUT / name)
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
    report = {"command": command, "exit_code": result.returncode,
        "rtconfig_restored": header.read_bytes() == original,
        "rtconfig_sha256": hashlib.sha256(original).hexdigest(),
        "scope": "FIRMWARE_LINK_WITH_M4_M5_PORT_ROOTS_NO_RUNTIME_DISPATCH",
        "hardware_execution": "NOT_RUN"}
    (OUT / "target_build.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(result.stdout[-1600:] + result.stderr[-2400:])
    print(json.dumps(report, indent=2))
    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
