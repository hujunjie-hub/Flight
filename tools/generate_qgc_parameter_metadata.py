"""Generate the QGroundControl parameter metadata JSON for the Flight plugin.

Reads the committed parameter catalog (single source of truth) and writes the
resource file loaded by FlightParameterMetaData inside QGroundControl.

    python tools/generate_qgc_parameter_metadata.py
"""

import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "middleware/protocol/mavlink/parameters/ground_parameters.json"
DEFAULT_OUTPUT = (ROOT.parent / "FMT_GroundControl/src/FirmwarePlugin/Flight"
                  / "FlightParameterFactMetaData.json")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()

    catalog = json.loads(CATALOG.read_text(encoding="utf-8"))
    parameters = []
    for item in catalog["parameters"]:
        # Only the live controller tuning subset is editable in QGC. Model
        # geometry and calibration values remain in the firmware catalog but
        # are not part of the operator's parameter page.
        if not (item["name"].startswith("CTL_") or item["name"] == "VEH_MASS"):
            continue
        parameters.append({
            "name": item["name"],
            "type": "float",
            "group": item["group"],
            "shortDescription": item["title"],
            "longDescription": item["description"],
            "units": item["units"],
            "min": item["minimum"],
            "max": item["maximum"],
            "default": item["value"],
            "decimalPlaces": 0 if item["units"] in ("flag", "raw") else 4,
            "rebootRequired": False,
        })
    # SIL-only parameter: the H723 catalog intentionally has no SIM_TLM_HZ,
    # but the Host SIL exposes it and QGC would otherwise place it in the
    # untranslated Other/Misc bucket with no units or description.
    parameters.append({
        "name": "SIM_TLM_HZ",
        "type": "float",
        "group": "仿真",
        "shortDescription": "SIL 遥测频率",
        "longDescription": "仿真导航遥测的发布频率，仅 Host SIL 有效，范围 1–10 Hz。",
        "units": "Hz",
        "min": 1.0,
        "max": 10.0,
        "default": 5.0,
        "decimalPlaces": 1,
        "rebootRequired": False,
    })
    document = {
        "version": 1,
        "source": "middleware/protocol/mavlink/parameters/ground_parameters.json",
        "sha256": catalog["sha256"],
        "parameters": parameters,
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(
        json.dumps(document, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"wrote {len(parameters)} QGC parameter metadata entries to {arguments.output}")


if __name__ == "__main__":
    main()
