"""Charge production M4 state to the ICD/M4 budget instead of hiding its BSS."""
from pathlib import Path
import json
import os
import subprocess

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
ARM = Path(os.environ.get("FLIGHT_ARM_BIN", r"D:\STM32CubeCLT\GNU-tools-for-STM32\bin"))


def bss(path):
    output = subprocess.check_output([str(ARM/"arm-none-eabi-size.exe"), str(path)], text=True)
    return int(output.strip().splitlines()[-1].split()[2])


def main():
    report = json.loads((ROOT/"spec/validation/m4-m5-implementation_report.json").read_text(encoding="utf-8"))
    slots = next(item["arena_bytes"] for item in report["checks"] if item["name"] == "ICD_slot_and_arena_reference")
    m4_raw = bss(ROOT/"build/m4_arm/quad_x4_fms.o")
    m4 = (m4_raw+7)//8*8
    m5_sources = [*ROOT.glob("middleware/guidance/*/*.c"),
                  ROOT/"middleware/model/fms/geofence_geometry/geofence_geometry.c"]
    m5 = sum((bss(ROOT/"build/m5_arm"/source.with_suffix(".o").name)+7)//8*8
             for source in m5_sources)
    result = {"icd_slots_scratch_and_reservations_bytes": slots, "m4_object_bss_bytes": m4_raw,
        "m4_static_bytes_with_alignment": m4,
        "icd_and_m4_total_bytes": slots+m4, "icd_and_m4_limit_bytes": 48*1024,
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
