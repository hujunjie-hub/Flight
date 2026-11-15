"""Generate QGroundControl parameter metadata and the Flight model registry.

The firmware parameter catalog and model configuration remain the source of truth.
The generated files are consumed by FlightParameterMetaData and FlightFirmwarePlugin.

    python tools/generate_qgc_parameter_metadata.py
"""

import argparse
import hashlib
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "middleware/protocol/mavlink/parameters/ground_parameters.json"
MODEL_CONFIG = ROOT / "config/vehicle_model.yaml"
CONTROLLER_CONFIG = ROOT / "config/controller_tuning.yaml"
ESTIMATOR_CONFIG = ROOT / "config/estimator_tuning.yaml"
DEFAULT_OUTPUT = (ROOT.parent / "FMT_GroundControl/src/FirmwarePlugin/Flight"
                  / "FlightParameterFactMetaData.json")
DEFAULT_CATALOG_OUTPUT = DEFAULT_OUTPUT.with_name("FlightVehicleModelCatalog.json")
DEFAULT_UNREGISTERED_OUTPUT = DEFAULT_OUTPUT.with_name(
    "FlightParameterFactMetaData-Unregistered.json"
)

DOMAIN_BY_GROUP = {
    "机体模型": "VEHICLE_MODEL",
    "MPC 权重": "CONTROL",
    "位置环回退": "CONTROL",
    "姿态角速度环": "CONTROL",
    "SO3 姿态增益": "CONTROL",
    "仿真": "SIMULATION",
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _estimator_status(text: str) -> str:
    match = re.search(r"^status:\s*([A-Za-z0-9_]+)\s*$", text, re.MULTILINE)
    if not match:
        raise ValueError(f"estimator tuning status missing in {ESTIMATOR_CONFIG}")
    return match.group(1)


def build_parameter_document(catalog: dict, model_config: dict) -> dict:
    profile = model_config["profile"]
    parameters = []
    for item in catalog["parameters"]:
        # Only the live controller tuning subset and model mass are editable
        # through the Flight parameter page. Geometry/calibration remains in
        # the firmware catalog and is not exposed as ordinary tuning values.
        if not (item["name"].startswith("CTL_") or item["name"] == "VEH_MASS"):
            continue
        domain = DOMAIN_BY_GROUP.get(item["group"])
        if domain is None:
            raise ValueError(
                f"unmapped parameter group {item['group']!r} for {item['name']}"
            )
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
            "domain": domain,
            "modelProfiles": [profile],
        })

    # SIL-only parameter: the H723 catalog intentionally has no SIM_TLM_HZ,
    # but the Host SIL exposes it.
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
        "domain": "SIMULATION",
        "modelProfiles": [profile],
    })

    return {
        "version": 1,
        "source": "middleware/protocol/mavlink/parameters/ground_parameters.json",
        "sha256": catalog["sha256"],
        "modelProfiles": [profile],
        "parameters": parameters,
    }


def build_model_catalog_document(
    catalog: dict,
    model_config: dict,
    controller_config: dict,
    estimator_status: str,
    source_hashes: dict,
    parameter_document: dict,
) -> dict:
    profile_status = model_config["status"]
    controller_status = controller_config.get("status", "UNKNOWN")
    hardware_curve = str(model_config.get("hardware_thrust_curve", "TBD"))
    hardware_curve_available = bool(hardware_curve.strip()) and not hardware_curve.upper().startswith("TBD")
    hardware_eligible = (
        profile_status == "HARDWARE_VALIDATED"
        and controller_status == "HARDWARE_VALIDATED"
        and estimator_status == "HARDWARE_VALIDATED"
        and hardware_curve_available
    )
    if profile_status == "SIM_ONLY":
        write_scope = "SIL_ONLY"
        display_name = f"{model_config['profile']} · 仿真专用"
    elif hardware_eligible:
        write_scope = "HARDWARE_VALIDATED"
        display_name = model_config["profile"]
    else:
        write_scope = "LOCKED"
        display_name = f"{model_config['profile']} · 尚未通过硬件验证"

    parameters = parameter_document["parameters"]
    profile = {
        "id": model_config["profile"],
        "displayName": display_name,
        "status": profile_status,
        "hardwareEligible": hardware_eligible,
        "parameterWriteScope": write_scope,
        "sourceDescription": model_config.get("source", ""),
        "hardwareThrustCurve": hardware_curve,
        "controllerTuningStatus": controller_status,
        "estimatorTuningStatus": estimator_status,
        "parameterDomains": sorted({item["domain"] for item in parameters}),
        "parameterNames": [item["name"] for item in parameters],
    }
    return {
        "schema_version": 1,
        "source": {
            "repository": "hujunjie-hub/Flight",
            "parameterCatalog": {
                "path": "middleware/protocol/mavlink/parameters/ground_parameters.json",
                "sha256": catalog["sha256"],
            },
            "vehicleModel": {
                "path": "config/vehicle_model.yaml",
                "sha256": source_hashes["vehicleModel"],
            },
            "controllerTuning": {
                "path": "config/controller_tuning.yaml",
                "sha256": source_hashes["controllerTuning"],
            },
            "estimatorTuning": {
                "path": "config/estimator_tuning.yaml",
                "sha256": source_hashes["estimatorTuning"],
            },
        },
        "profiles": [profile],
    }


def build_unregistered_metadata_document() -> dict:
    return {
        "version": 1,
        "source": "No registered Flight model profile; intentionally empty metadata.",
        "modelProfiles": [],
        "parameters": [],
    }


def _write_json(path: Path, document: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(document, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--catalog-output", type=Path, default=DEFAULT_CATALOG_OUTPUT)
    parser.add_argument("--unregistered-output", type=Path, default=DEFAULT_UNREGISTERED_OUTPUT)
    parser.add_argument("--model-config", type=Path, default=MODEL_CONFIG)
    parser.add_argument("--controller-config", type=Path, default=CONTROLLER_CONFIG)
    parser.add_argument("--estimator-config", type=Path, default=ESTIMATOR_CONFIG)
    arguments = parser.parse_args()

    catalog = json.loads(CATALOG.read_text(encoding="utf-8"))
    model_config = json.loads(arguments.model_config.read_text(encoding="utf-8"))
    controller_config = json.loads(arguments.controller_config.read_text(encoding="utf-8"))
    estimator_status = _estimator_status(arguments.estimator_config.read_text(encoding="utf-8"))

    parameter_document = build_parameter_document(catalog, model_config)
    source_hashes = {
        "vehicleModel": _sha256(arguments.model_config),
        "controllerTuning": _sha256(arguments.controller_config),
        "estimatorTuning": _sha256(arguments.estimator_config),
    }
    model_catalog = build_model_catalog_document(
        catalog,
        model_config,
        controller_config,
        estimator_status,
        source_hashes,
        parameter_document,
    )

    _write_json(arguments.output, parameter_document)
    _write_json(arguments.catalog_output, model_catalog)
    _write_json(arguments.unregistered_output, build_unregistered_metadata_document())
    print(
        f"wrote {len(parameter_document['parameters'])} parameter entries, "
        f"{len(model_catalog['profiles'])} model profiles, and an empty "
        f"unregistered metadata catalog"
    )


if __name__ == "__main__":
    main()
