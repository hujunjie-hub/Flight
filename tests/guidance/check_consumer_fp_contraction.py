"""MR-009 item 5: pin the floating-point semantics of the proved curve.

The validator replicates `control_sample_reference()`'s binary32 Horner chain.
That replication is only meaningful if neither translation unit contracts
`a*b + c` into a fused multiply-add with a different rounding sequence. The production build takes its flags from rtconfig.py, which pins
-ffp-contract=off, -frounding-math and -fexcess-precision=standard. This check
compiles:

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



MODEL_TOKENS = {
    "gamma_n": [
        "double nu = (double)rounding_count * unit;",
        "nu = nextafter(nu, INFINITY);",
        "gamma = nu / (1.0 - nu);",
        "gamma = nextafter(gamma, INFINITY);",
    ],
    "gamma_2_derivation": [
        "double nu2 = 2.0 * 0x1p-24;",
        "nu2 = nextafter(nu2, INFINITY);",
        "gamma2 = nu2 / (1.0 - nu2);",
        "gamma2 = nextafter(gamma2, INFINITY);",
    ],
    "time_term": [
        "delta_s = multiply_upper_nonnegative(gamma2, s_max);",
        "s_eval_max = add_upper_nonnegative(s_max, delta_s);",
        "multiply_upper_nonnegative(delta_s,",
    ],
    "interval_coefficients": [
        "low_distance = fabs(cf - exact[i].low);",
        "high_distance = fabs(cf - exact[i].high);",
        "difference = fmax(low_distance, high_distance);",
    ],
    "rounding_counts": [
        "#define M5_CONSUMER_POSITION_ROUNDINGS 10u",
        "#define M5_CONSUMER_VELOCITY_ROUNDINGS 12u",
        "#define M5_CONSUMER_ACCELERATION_ROUNDINGS 10u",
    ],
    "range_guards": [
        "if (magnitude_term > (double)FLT_MAX * 0.0625) return INFINITY;",
        "if (magnitude_term < (double)FLT_MIN * 1024.0) return INFINITY;",
    ],
    "subnormal_amplification": [
        "(double)rounding_count * 0x1p-149",
        "amplification = multiply_upper_nonnegative(amplification, scale)",
    ],
    "runtime_fp_environment": [
        "vmrs %0, fpscr",
        "_mm_getcsr()",
        "fegetround() == FE_TONEAREST",
    ],
}


def check_model_against_source(source=None):
    """The report must describe the model that is actually compiled."""
    if source is None:
        source = VALIDATOR.read_text(encoding="utf-8")
    missing = []
    for section, tokens in MODEL_TOKENS.items():
        for token in tokens:
            if token not in source:
                missing.append(f"{section}: {token}")
    import hashlib
    sources = {}
    for path in (VALIDATOR, CONSUMER):
        sources[str(path.relative_to(ROOT)).replace("\\", "/")] = hashlib.sha256(
            path.read_bytes()).hexdigest()
    return {
        "status": "MATCH" if not missing else "MISMATCH",
        "source": str(VALIDATOR.relative_to(ROOT)).replace("\\", "/"),
        "source_sha256": sources,
        "missing_tokens": missing,
        "note": ("Report fields are derived from tokens that must exist in the "
                 "compiled validator source; a mismatch fails the check."),
    }


MUTATIONS = (
    ("gamma_2", "gamma2 = nu2 / (1.0 - nu2);", "gamma2 = 0.0;"),
    ("gamma_n", "gamma = nu / (1.0 - nu);", "gamma = nu;"),
    ("interval_bound", "difference = fmax(low_distance, high_distance);",
     "difference = low_distance;"),
    ("s_eval_max", "s_eval_max = add_upper_nonnegative(s_max, delta_s);",
     "s_eval_max = s_max;"),
    ("subnormal", "amplification = multiply_upper_nonnegative(amplification, scale);",
     "amplification = 1.0;"),
    ("fp_environment", "vmrs %0, fpscr", "vmrs %0, /* removed */"),
    ("rounding_count", "#define M5_CONSUMER_POSITION_ROUNDINGS 10u",
     "#define M5_CONSUMER_POSITION_ROUNDINGS 9u"),
)


def self_test():
    """Every deliberate mutation of the model must be reported as MISMATCH."""
    source = VALIDATOR.read_text(encoding="utf-8")
    good = check_model_against_source(source)
    if good["status"] != "MATCH":
        print(f"FAIL: unmodified source reported {good['status']}")
        return 1
    for name, original, replacement in MUTATIONS:
        if original not in source:
            print(f"FAIL: mutation target for {name} not found")
            return 1
        mutated = source.replace(original, replacement, 1)
        result = check_model_against_source(mutated)
        if result["status"] != "MISMATCH":
            print(f"FAIL: mutation {name} was not detected")
            return 1
    print(f"M5_FP_MODEL_SELF_TEST_PASS ({len(MUTATIONS)} mutations detected)")
    return 0


def main():
    if "--self-test" in sys.argv[1:]:
        return self_test()
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
    model = check_model_against_source()
    report = {
        "status": ("PASS" if fused_total == 0 and model["status"] == "MATCH"
                   else "REVIEW_REQUIRED"),
        "compiler": compiler,
        "error_model": {
            "coefficient_term": "outward interval coefficients [low, high], "
                                "max(|cf - low|, |cf - high|) * s^i",
            "horner_term": "gamma_n * sum |cf_i| * s^i with gamma_n = n*u/(1-n*u)",
            "horner_roundings": {"position": 10, "velocity": 12,
                                 "acceleration": 10},
            "time_term": "gamma_2 * t_max * |d^k p/ds^k|(s_eval_max) with "
                         "gamma_2 = 2u/(1-2u), s_exact = local_us/1e6, t_max = step_us/1e6",
            "evaluation_interval": "s_eval_max = max(t_max, h) + gamma_2*t_max",
            "subnormal_term": "n * 2^-149 * max(1, s_eval_max)^n",
        },
        "model_check": model,
        "objects": [consumer, validator],
        "fused_operations": fused_total,
        "note": ("No fused multiply-add observed in either object; the replicated "
                 "binary32 coefficient chain matches the consumer operation order."
                 if fused_total == 0 else
                 "Fused multiply-add observed; MR-009 must be re-reviewed because "
                 "the consumer rounding sequence changed."),
    }
    payload = json.dumps(report, indent=2)
    (OUT / "consumer_fp.json").write_text(payload, encoding="utf-8")
    log_lines = [
        f"compiler={compiler}",
        f"fused_operations={fused_total}",
        f"model_check={model['status']}",
        f"model_missing={model['missing_tokens']}",
    ]
    for entry in report["objects"]:
        log_lines.append(
            f"object={entry['object']} sha256={entry['object_sha256']} "
            f"fused={entry['fused_operations']} flags={entry['flags']}")
    (OUT / "consumer_fp.log").write_text("\n".join(log_lines) + "\n", encoding="utf-8")
    print(payload)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
