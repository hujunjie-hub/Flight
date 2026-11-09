"""Build the existing SCons firmware with M4/M5 enabled, without changing defaults.

The temporary rtconfig.h overlay is restored in finally. Function roots keep
both production Ports in the link despite the BSP's empty application loop.
No new thread, hardware output or automatic arming is installed by this check.
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


def main():
    OUT.mkdir(parents=True, exist_ok=True)
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
    env["RTT_EXEC_PATH"] = env.get("FLIGHT_ARM_BIN", r"D:\STM32CubeCLT\GNU-tools-for-STM32\bin")
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
