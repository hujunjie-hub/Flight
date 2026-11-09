#!/usr/bin/env python3
"""Agent A review of C's product_contract.h contract diff (CR-V191-C01/C02).

Compares the current production ABI layout with the pre-CR R3 archive
(spec/validation/T191-D-REVIEW/r3_archive/round3_abi_layout.json) and checks
that the only layout delta is the CR-V191-C01 mission_epoch addition plus the
CR-V191-C02 geofence config types; then verifies the regenerated abi_lock.h
(145 types) and the Host/ARM evidence.

Run from the Flight project root:
  python applications/tests/abi/reviews/c_contract_diff_check.py
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
PRE_CR = ROOT / "spec" / "validation" / "T191-D-REVIEW" / "r3_archive" / "round3_abi_layout.json"
ARM = ROOT / "applications" / "tests" / "abi" / "evidence" / "abi_arm.json"
HOST = ROOT / "applications" / "tests" / "abi" / "evidence" / "abi_host.json"
COMPARE = ROOT / "applications" / "tests" / "abi" / "evidence" / "abi_compare.json"
LOCK = ROOT / "applications" / "products" / "QUAD_X4_H723" / "abi_lock.h"
PRODUCT = ROOT / "applications" / "products" / "QUAD_X4_H723" / "product_contract.h"
OUT = HERE / "c_contract_diff_check.json"

PRIVATE = {"MpcOutputSnapshot"}
EXPECTED_NEW_TYPES = {"ProductGeofenceDefinition", "ProductGeofenceConfigPayload",
                      "ProductGeofenceConfig"}


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fields_of(item):
    if "fields" in item:
        return item["fields"]
    return {key.split(":", 1)[1]: value for key, value in item.items()
            if key.startswith("offset:")}


def main() -> int:
    pre = json.loads(PRE_CR.read_text(encoding="utf-8"))["layouts"]
    arm = json.loads(ARM.read_text(encoding="utf-8"))["layout"]
    host = json.loads(HOST.read_text(encoding="utf-8"))["layout"]
    cmp = json.loads(COMPARE.read_text(encoding="utf-8"))
    lock = LOCK.read_text(encoding="utf-8")
    header = PRODUCT.read_text(encoding="utf-8")

    checks = []

    def add(cid, ok, detail):
        checks.append({"id": cid, "status": "PASS" if ok else "FAIL", "detail": detail})

    old = {k: v for k, v in pre.items() if k not in PRIVATE}
    new = {k: v for k, v in arm.items() if k not in PRIVATE}

    added = sorted(set(new) - set(old))
    removed = sorted(set(old) - set(new))
    add("type_set_delta", added == sorted(EXPECTED_NEW_TYPES) and not removed,
        {"added": added, "removed": removed, "expected_added": sorted(EXPECTED_NEW_TYPES)})

    changed = []
    for name in sorted(set(old) & set(new)):
        if old[name]["size"] != new[name]["size"] or old[name]["alignment"] != new[name]["alignment"]:
            changed.append({"type": name, "old_size": old[name]["size"], "new_size": new[name]["size"],
                            "old_align": old[name]["alignment"], "new_align": new[name]["alignment"]})
    add("size_align_delta_only_pop",
        [c["type"] for c in changed] == ["ProductOperationPayload"],
        {"changed": changed})

    pop_old = old["ProductOperationPayload"]
    pop_new = new["ProductOperationPayload"]
    add("pop_size_44_to_48", pop_old["size"] == 44 and pop_new["size"] == 48,
        {"old": pop_old["size"], "new": pop_new["size"]})
    add("pop_mission_epoch_offset_44",
        fields_of(pop_new).get("mission_epoch") == 44,
        {"mission_epoch_offset": fields_of(pop_new).get("mission_epoch")})
    existing_fields_unchanged = all(
        fields_of(pop_new).get(field) == offset
        for field, offset in fields_of(pop_old).items())
    add("pop_existing_offsets_unchanged", existing_fields_unchanged,
        {"old_fields": fields_of(pop_old), "new_fields": fields_of(pop_new)})

    req_old, req_new = old["OperationRequest"], new["OperationRequest"]
    add("operation_request_unchanged",
        req_old["size"] == req_new["size"] == 160
        and req_old["alignment"] == req_new["alignment"]
        and fields_of(req_old) == fields_of(req_new),
        {"old_size": req_old["size"], "new_size": req_new["size"],
         "fields_equal": fields_of(req_old) == fields_of(req_new)})

    # Header contract checks.
    add("revision_0x00010904",
        bool(re.search(r"#define\s+PRODUCT_CONTRACT_REVISION\s+0x00010904u", header)),
        {"match": re.findall(r"#define\s+PRODUCT_CONTRACT_REVISION[^\n]*", header)})
    add("port_revision_matches",
        bool(re.search(r"FC_PORT_API_REVISION\s+0x00010904u",
                       (ROOT / "applications" / "contracts" / "fc_contract_version.h").read_text(encoding="utf-8"))),
        {})
    add("mission_epoch_valid_bit",
        bool(re.search(r"#define\s+PRODUCT_OPERATION_VALID_MISSION_EPOCH\s+\(1u\s*<<\s*8\)", header)),
        {"match": re.findall(r"#define\s+PRODUCT_OPERATION_VALID_MISSION_EPOCH[^\n]*", header)})
    in_payload = re.search(r"typedef struct \{(.*?)\} ProductOperationPayload;", header, re.S)
    add("mission_epoch_in_payload_once",
        bool(in_payload) and len(re.findall(r"\bmission_epoch\b", in_payload.group(1))) == 1
        and bool(re.search(r"uint32_t\s+mission_epoch;", in_payload.group(1))),
        {"occurrences": len(re.findall(r"\bmission_epoch\b", in_payload.group(1))) if in_payload else 0})
    add("profile_id_unchanged", "#define PRODUCT_PROFILE_ID 0x00010001u" in header, {})
    add("geofence_types_present",
        all(re.search(r"\}\s*%s;" % name, header) for name in EXPECTED_NEW_TYPES),
        {name: bool(re.search(r"\}\s*%s;" % name, header)) for name in EXPECTED_NEW_TYPES})

    # Lock checks.
    def lock_macro(name):
        match = re.search(r"#define\s+%s\s+(0x[0-9A-Fa-f]+|\d+)u?" % name, lock)
        return int(match.group(1), 0) if match else None

    add("lock_type_count_145", lock_macro("FC_ABI_LOCK_TYPE_COUNT") == 145,
        {"value": lock_macro("FC_ABI_LOCK_TYPE_COUNT")})
    add("lock_revisions",
        lock_macro("FC_ABI_LOCK_PORT_API_REVISION") == 0x00010904
        and lock_macro("FC_ABI_LOCK_PRODUCT_CONTRACT_REVISION") == 0x00010904
        and lock_macro("FC_ABI_LOCK_SCHEMA_MAJOR") == 6
        and lock_macro("FC_ABI_LOCK_SCHEMA_MINOR") == 0,
        {"port": lock_macro("FC_ABI_LOCK_PORT_API_REVISION"),
         "product": lock_macro("FC_ABI_LOCK_PRODUCT_CONTRACT_REVISION"),
         "schema": (lock_macro("FC_ABI_LOCK_SCHEMA_MAJOR"), lock_macro("FC_ABI_LOCK_SCHEMA_MINOR"))})
    add("lock_header_bytes_88", lock_macro("FC_ABI_LOCK_CONTRACT_HEADER_BYTES") == 88,
        {"value": lock_macro("FC_ABI_LOCK_CONTRACT_HEADER_BYTES")})
    arm_hash = re.search(r'FC_ABI_LOCK_ARM_HASH\s+"([0-9a-f]{64})"', lock)
    host_hash = re.search(r'FC_ABI_LOCK_HOST_HASH\s+"([0-9a-f]{64})"', lock)
    add("lock_hashes_match_evidence",
        bool(arm_hash) and bool(host_hash)
        and arm_hash.group(1) == json.loads(ARM.read_text(encoding="utf-8"))["abi_hash"]
        and host_hash.group(1) == json.loads(HOST.read_text(encoding="utf-8"))["abi_hash"],
        {"arm": arm_hash.group(1) if arm_hash else None,
         "host": host_hash.group(1) if host_hash else None})

    # Evidence checks.
    add("spec_layout_145_998",
        cmp["spec_layout_comparison"]["status"] == "PASS"
        and cmp["spec_layout_comparison"]["compared_types"] == 145
        and cmp["spec_layout_comparison"]["compared_field_offsets"] == 998
        and cmp["spec_layout_comparison"]["mismatches"] == [],
        {"status": cmp["spec_layout_comparison"]["status"],
         "types": cmp["spec_layout_comparison"]["compared_types"],
         "offsets": cmp["spec_layout_comparison"]["compared_field_offsets"]})
    add("host_arm_no_unexpected",
        cmp["host_arm_comparison"]["status"] == "PASS"
        and not cmp["host_arm_comparison"]["unexpected_differences"],
        {"identical": cmp["host_arm_comparison"]["identical_types"],
         "pointer_dependent": len(cmp["host_arm_comparison"]["differing_types"])})
    add("registry_and_flags",
        cmp["message_registry_check"]["status"] == "PASS"
        and cmp["firmware_build_flags"]["status"] == "PASS",
        {"registry": cmp["message_registry_check"]["status"],
         "firmware_flags": cmp["firmware_build_flags"]["status"]})
    add("overall_evidence_status", cmp["status"] == "PASS", {"status": cmp["status"]})

    failed = [c["id"] for c in checks if c["status"] != "PASS"]
    result = {
        "reviewer": "A",
        "subject": "C product_contract.h contract diff (CR-V191-C01/C02)",
        "baseline": "V1.9.1-GLOBAL-AUDIT-R3 / Schema 6.0 / Product-Port 0x00010904",
        "pre_cr_baseline": str(PRE_CR.relative_to(ROOT)).replace("\\", "/"),
        "files": {
            "applications/products/QUAD_X4_H723/product_contract.h": sha(PRODUCT),
            "applications/products/QUAD_X4_H723/abi_lock.h": sha(LOCK),
            "applications/tests/abi/evidence/abi_arm.json": sha(ARM),
            "applications/tests/abi/evidence/abi_host.json": sha(HOST),
            "applications/tests/abi/evidence/abi_compare.json": sha(COMPARE),
        },
        "deltas": {
            "added_types": added,
            "removed_types": removed,
            "size_or_alignment_changes": changed,
            "product_operation_payload": {"old_size": pop_old["size"], "new_size": pop_new["size"],
                                          "mission_epoch_offset": fields_of(pop_new).get("mission_epoch")},
            "operation_request": {"old_size": req_old["size"], "new_size": req_new["size"]},
        },
        "checks": checks,
        "failed_checks": failed,
        "observations": [
            "C's status request was written before D accepted T191-A-ABI and still cites A-ABI READY_FOR_ASSIGNMENT; D should update the dependency view.",
            "This review covers the contract/ABI diff only; C's M4 epoch-rejection and geofence-init implementation behaviour is outside this review (C-FMS evidence).",
        ],
        "verdict": "PASS" if not failed else "FAIL",
    }
    OUT.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"verdict": result["verdict"], "failed": failed,
                      "added": added, "changed": [c["type"] for c in changed]},
                     ensure_ascii=False))
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
