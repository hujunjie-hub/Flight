"""MR-009 item 5: pin the floating-point semantics of the proved curve.

The validator replicates `control_sample_reference()`'s binary32 Horner chain.
That replication is only meaningful if neither translation unit contracts
`a*b + c` into a fused multiply-add with a different rounding sequence. The
production build takes its flags from rtconfig.py (no -ffp-contract pin), so
this check compiles:

* the M6 consumer with the production flags;
* the M5-06 validator with the Cortex-M7 check flags from
  tests/guidance/arm_build_check.sh,

then inspects both Cortex-M7 objects for fused operations and records the object
SHA256, compiler version and the complete flag list. A fused operation in either
object means MR-009 must be re-reviewed, because the coefficient chain would no
longer match the replicated one.

The error model itself is `gamma_n = n*u/(1-n*u)` with n = 10/12/10 roundings
(position/velocity/acceleration) plus the interval-coefficient term; this script
only proves that the compiler did not change the operation sequence.

Exit codes: 0 = no fused multiply-add in either object, 1 = found or build
failed, 2 = BLOCKED because the ARM toolchain is unavailable.
"""
from pathlib import Path
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
OUT = ROOT / "build/m5_arm"
CONSUMER = ROOT / "middleware/model/control/outer_loop/quad_x4/position_tracking/control_reference.c"
VALIDATOR = ROOT / "middleware/guidance/trajectory_planning/m5_trajectory_safety.c"
INCLUDES = [
    "-Imiddleware/model/control",
    "-Imiddleware/model/control/inner_loop",
    "-Imiddleware/model/control/inner_loop/quad_x4",
    "-Imiddleware/model/control/outer_loop",
    "-Imiddleware/model/fms",
    "-Imiddleware/contracts",
    "-Imiddleware/guidance",
]
FUSED = ("vfma", "vfms", "vfnma", "vfnms", "vmla", "vmls")


def find_tool(name):
    configured = os.environ.get("FLIGHT_ARM_BIN")
    candidates = []
    if configured:
        candidates.append(Path(configured) / name)
    candidates.append(Path(r"D:\STM32CubeCLT\GNU-tools-for-STM32\bin") / name)
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    found = shutil.which(name.replace(".exe", ""))
    return Path(found) if found else None


def validator_flags():
    """Read the Cortex-M7 flag list from arm_build_check.sh, not a re-typed copy."""
    script = (ROOT / "tests/guidance/arm_build_check.sh").read_text(encoding="utf-8")
    match = re.search(r"flags=\(\n(.*?)\n\)", script, re.S)
    if not match:
        raise SystemExit("cannot parse the flag list from arm_build_check.sh")
    flags = []
    for line in match.group(1).splitlines():
        flags += line.split()
    return flags


def build_and_scan(gcc, objdump, source, flags, object_name):
    obj = OUT / object_name
    command = [str(gcc)] + flags + INCLUDES + [
        "-c", str(source.relative_to(ROOT)), "-o", str(obj)]
    build = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    if build.returncode != 0:
        print(build.stdout + build.stderr)
        raise SystemExit(f"compile failed for {source.name}")
    disassembly = subprocess.run([str(objdump), "-d", str(obj)], cwd=ROOT,
                                 capture_output=True, text=True, check=True).stdout
    counts = {mnemonic: 0 for mnemonic in FUSED}
    for line in disassembly.splitlines():
        for mnemonic in FUSED:
            if ("\t" + mnemonic) in line or (" " + mnemonic + ".") in line:
                counts[mnemonic] += 1
    return {
        "source": str(source.relative_to(ROOT)).replace("\\", "/"),
        "object": str(obj.relative_to(ROOT)).replace("\\", "/"),
        "object_sha256": hashlib.sha256(obj.read_bytes()).hexdigest(),
        "flags": " ".join(flags),
        "fused_operations": sum(counts.values()),
        "mnemonics": counts,
    }


def main():
    sys.path.insert(0, str(ROOT))
    import rtconfig  # production compiler flags, not a re-typed copy

    gcc = find_tool("arm-none-eabi-gcc.exe")
    objdump = find_tool("arm-none-eabi-objdump.exe")
    if not gcc or not objdump:
        print(json.dumps({"status": "BLOCKED",
                          "reason": "GNU Arm toolchain not found (FLIGHT_ARM_BIN or PATH)"},
                         indent=2))
        return 2

    OUT.mkdir(parents=True, exist_ok=True)
    compiler = subprocess.run([str(gcc), "--version"], capture_output=True,
                              text=True, check=True).stdout.splitlines()[0]
    consumer = build_and_scan(gcc, objdump, CONSUMER, rtconfig.CFLAGS.split(),
                              "control_reference_prod.o")
    validator = build_and_scan(gcc, objdump, VALIDATOR,
                               validator_flags() + ["-DFLIGHT_M5_HOST_PLANNER"],
                               "m5_trajectory_safety_fp_check.o")
    fused_total = consumer["fused_operations"] + validator["fused_operations"]
    report = {
        "status": "PASS" if fused_total == 0 else "REVIEW_REQUIRED",
        "compiler": compiler,
        "error_model": ("gamma_n = n*u/(1-n*u), u = 2^-24, n = 10/12/10 for "
                        "position/velocity/acceleration, plus the outward "
                        "interval-coefficient term and the 2u*s_max time term"),
        "objects": [consumer, validator],
        "fused_operations": fused_total,
        "note": ("No fused multiply-add observed in either object; the replicated "
                 "binary32 coefficient chain matches the consumer operation order."
                 if fused_total == 0 else
                 "Fused multiply-add observed; MR-009 must be re-reviewed because "
                 "the consumer rounding sequence changed."),
    }
    (OUT / "consumer_fp.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if fused_total == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
