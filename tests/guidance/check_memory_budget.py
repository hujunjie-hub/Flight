"""Charge production M4 state to the ICD/M4 budget instead of hiding its BSS.

The M5 objects are produced by tests/guidance/arm_build_check.ps1 (Windows) or
tests/guidance/arm_build_check.sh (WSL/Linux). Both scripts compile the full
middleware/guidance tree, so every source below has a matching object; a missing
object is a build-coverage defect, not a reason to skip the source.
"""
from pathlib import Path
import json
import os
import shutil
import subprocess

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
DEFAULT_ARM_BIN = Path(r"D:\STM32CubeCLT\GNU-tools-for-STM32\bin")

# H723/NONE production slice.
PRODUCTION_SOURCES = (
    "middleware/model/fms/geofence_geometry/geofence_geometry.c",
    "middleware/guidance/guidance_update/guidance.c",
    "middleware/guidance/trajectory_planning/trajectory_math.c",
    "middleware/guidance/guidance_update/guidance_input_checks.c",
    "middleware/guidance/trajectory_planning/mission_trajectory.c",
    "middleware/guidance/pilot_and_landing/pilot_and_landing.c",
    "middleware/guidance/trajectory_planning/trajectory_validation.c",
    "middleware/guidance/guidance_update/guidance_config.c",
)
# Host/SIL-only M5 advanced planner and M5-06 validator translation units.
HOST_ONLY_SOURCES = (
    "middleware/guidance/trajectory_planning/m5_local_map.c",
    "middleware/guidance/trajectory_planning/m5_kinodynamic_astar.c",
    "middleware/guidance/trajectory_planning/m5_time_scaler.c",
    "middleware/guidance/trajectory_planning/m5_path_smoother.c",
    "middleware/guidance/trajectory_planning/minimum_jerk_optimizer.c",
    "middleware/guidance/trajectory_planning/m5_trajectory_safety.c",
)


def resolve_arm_tool(name):
    configured = os.environ.get("FLIGHT_ARM_BIN")
    if configured:
        candidate = Path(configured) / name
        if candidate.is_file():
            return candidate
    candidate = DEFAULT_ARM_BIN / name
    if candidate.is_file():
        return candidate
    found = shutil.which(name.replace(".exe", ""))
    if found:
        return Path(found)
    raise SystemExit(f"ARM size tool not found: {name}")


SIZE = resolve_arm_tool("arm-none-eabi-size.exe")


def bss(path):
    output = subprocess.check_output([str(SIZE), str(path)], text=True)
    return int(output.strip().splitlines()[-1].split()[2])


def object_size(source):
    obj = ROOT / "build/m5_arm" / Path(source).with_suffix(".o").name
    if not obj.is_file():
        raise SystemExit(
            f"missing Cortex-M7 object {obj.relative_to(ROOT)}; run "
            "tests/guidance/arm_build_check.ps1 or arm_build_check.sh first")
    return (bss(obj) + 7) // 8 * 8


def main():
    report = json.loads((ROOT/"spec/validation/m4-m5-implementation_report.json").read_text(encoding="utf-8"))
    slots = next(item["arena_bytes"] for item in report["checks"] if item["name"] == "ICD_slot_and_arena_reference")
    m4_obj = ROOT/"build/m4_arm/quad_x4_fms.o"
    if not m4_obj.is_file():
        raise SystemExit(
            f"missing Cortex-M7 object {m4_obj.relative_to(ROOT)}; run "
            "tests/flight_management/arm_build_check.ps1 first")
    m4_raw = bss(m4_obj)
    m4 = (m4_raw+7)//8*8
    production = sum(object_size(source) for source in PRODUCTION_SOURCES)
    host_only = sum(object_size(source) for source in HOST_ONLY_SOURCES)
    m5 = production + host_only
    result = {"icd_slots_scratch_and_reservations_bytes": slots, "m4_object_bss_bytes": m4_raw,
        "m4_static_bytes_with_alignment": m4,
        "icd_and_m4_total_bytes": slots+m4, "icd_and_m4_limit_bytes": 48*1024,
        "m5_production_bss_bytes": production,
        "m5_host_only_bss_bytes": host_only,
        "m5_static_and_validator_bytes": m5, "m5_limit_bytes": 16*1024,
        "scope": "STATIC_ARM_OBJECTS_PLUS_SPEC_SLOT_RESERVATIONS",
        "stack_wcet_and_full_system_arena_placement": "NOT_RUN"}
    result["status"] = "PASS" if slots+m4 <= 48*1024 and m5 <= 16*1024 else "FAIL"
    out = ROOT/"build/m4_m5_implementation/evidence/memory_budget.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
