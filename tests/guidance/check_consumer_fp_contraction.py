"""MR-009 item 5: pin the floating-point semantics of the consumed curve.

M5-06 replicates control_sample_reference()'s binary32 Horner chain. That
replication is only meaningful if the production translation unit does not
contract `a*b + c` into a fused multiply-add with different rounding. The
production build takes its flags from rtconfig.py (no -ffp-contract pin), so
this check compiles the consumer with those exact flags and inspects the
Cortex-M7 disassembly for fused operations.

Exit codes: 0 = no fused multiply-add found, 1 = fused operation found (the
MR-009 envelope must be re-reviewed), 2 = BLOCKED because the ARM toolchain is
not available on this machine.
"""
from pathlib import Path
import json
import os
import shutil
import subprocess
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
OUT = ROOT / "build/m5_arm"
SOURCE = ROOT / "middleware/model/control/outer_loop/quad_x4/position_tracking/control_reference.c"
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
    obj = OUT / "control_reference_prod.o"
    command = [str(gcc)] + rtconfig.CFLAGS.split() + INCLUDES + [
        "-c", str(SOURCE.relative_to(ROOT)), "-o", str(obj)]
    build = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    if build.returncode != 0:
        print(build.stdout + build.stderr)
        return 1

    disassembly = subprocess.run([str(objdump), "-d", str(obj)], cwd=ROOT,
                                 capture_output=True, text=True, check=True).stdout
    counts = {mnemonic: 0 for mnemonic in FUSED}
    for line in disassembly.splitlines():
        for mnemonic in FUSED:
            if ("\t" + mnemonic) in line or (" " + mnemonic + ".") in line:
                counts[mnemonic] += 1
    fused_total = sum(counts.values())
    compiler = subprocess.run([str(gcc), "--version"], capture_output=True,
                              text=True, check=True).stdout.splitlines()[0]
    report = {
        "status": "PASS" if fused_total == 0 else "REVIEW_REQUIRED",
        "source": str(SOURCE.relative_to(ROOT)).replace("\\", "/"),
        "cflags": rtconfig.CFLAGS.strip(),
        "compiler": compiler,
        "fused_operations": fused_total,
        "mnemonics": counts,
        "note": ("No fused multiply-add observed; the validator's binary32 "
                 "replication and the MR-009 9u allowance cover this build."
                 if fused_total == 0 else
                 "Fused multiply-add observed; MR-009 envelope review required "
                 "because contraction changes the consumer rounding sequence."),
    }
    (OUT / "consumer_fp.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if fused_total == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
