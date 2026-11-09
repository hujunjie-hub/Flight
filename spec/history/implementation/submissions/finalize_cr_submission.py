#!/usr/bin/env python3
"""Finalize C-FMS submission revision 4 after implementing CR-V191-C01/C02."""
from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone, timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
NOW = datetime.now(timezone(timedelta(hours=8))).replace(microsecond=0).isoformat()


def sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


fms_path = ROOT / "applications/core/m4/status_request_T191-C-FMS.json"
fms = json.loads(fms_path.read_text(encoding="utf-8"))
old = sha(fms_path)

fms["submission"] = {
    "revision": 4,
    "submitted_at": NOW,
    "supersedes_sha256": old,
    "reason": "Implements CR-V191-C01 (mission_epoch in ProductOperationPayload + M4 stale/absent-epoch rejection) and CR-V191-C02 (compile-time ProductGeofenceConfig loaded by M4 at init, no runtime SET_GEOFENCE). Refreshes Host/ARM evidence (17 tests/240 checks). Contract/product revision coordinated with A's FC_PORT_API_REVISION 0x00010904; A must review the contract diff and regenerate abi_lock.h.",
    "dedup_key": "T191-C-FMS:4",
}
fms["requested_transition"] = "ASSIGNED->IN_PROGRESS->REVIEW"
fms["transition_note"] = (
    "T191-C-FMS is ASSIGNED/seq4. C started and completed the CR-C01/C02 implementation in this "
    "session. This resubmission requests IN_PROGRESS->REVIEW after D merges it. A must review the "
    "C-applied product-contract diff and regenerate applications/products/QUAD_X4_H723/abi_lock.h "
    "(port revision is already 0x00010904; product revision is set to match). No ACCEPTED/CLOSED is requested.")
fms["cr_implementation"] = {
    "CR-V191-C01": {
        "status": "IMPLEMENTED_PENDING_A_REVIEW",
        "contract": "ProductOperationPayload += uint32_t mission_epoch (0 = not applicable); valid bit PRODUCT_OPERATION_VALID_MISSION_EPOCH (1u<<8)",
        "layout": {"ProductOperationPayload_before": 44, "ProductOperationPayload_after": 48,
                   "OperationRequest_before": 160, "OperationRequest_after": 160},
        "m4": "START/PAUSE/RESUME/ABORT/CLEAR require the valid bit, a non-zero epoch equal to the current M4 mission_epoch; missing/stale epoch -> REJECTED reason 12 (EPOCH). Non-mission commands must carry zero/absent epoch.",
        "revision": "FC_PORT_API_REVISION=0x00010904 (already set by A); PRODUCT_CONTRACT_REVISION set to 0x00010904 by C to satisfy the contract invariant.",
        "owner_action": "A: review the contract diff, decide Schema Major stays 6.0, regenerate abi_lock.h and ABI evidence; D: record CR disposition.",
    },
    "CR-V191-C02": {
        "status": "IMPLEMENTED_PENDING_D_MERGE",
        "contract": "ProductGeofenceDefinition/ProductGeofenceConfigPayload/ProductGeofenceConfig provided by A in product_contract.h",
        "m4": "quad_x4_fms_configure_product_geofence() installs the immutable .rodata configuration before fms_policy_init(); M4 validates profile_hash/config_version/geofence_hash/geometry and holds the fences. A required fence with no configuration fails init (PROFILE_MISMATCH); no runtime SET_GEOFENCE command exists; engine setter rejects changes while ARMED.",
        "owner_action": "D: merge and close CR-V191-C02 after reviewing the Host evidence.",
    },
}
for ref in ("applications/products/QUAD_X4_H723/product_contract.h",):
    if ref not in fms["deliverable_refs"]:
        fms["deliverable_refs"].append(ref)
fms["actual_commands"] = [
    {"command": "wsl.exe -d Ubuntu-26.04 -- bash applications/core/m4/hosttest/run_host_tests.sh",
     "exit_code": 0,
     "result": "17 tests, 240 checks, 0 failures (production headers; CR-C01/C02 paths covered)"},
    {"command": "pwsh -NoProfile -File applications/core/m4/hosttest/arm_build_check.ps1",
     "exit_code": 0,
     "result": "Cortex-M7 compile PASS; text 25768 B; .bss 7213 B"},
    {"command": "host gcc size probe",
     "exit_code": 0,
     "result": "ProductOperationPayload=48 OperationRequest=160 ProductMissionPayload=1040 AuthorizedMissionSnapshot=1184"},
]
fms["verification"]["tests"] = {"count": 17, "assertions": 240, "failures": 0,
                                "log": "applications/core/m4/hosttest/evidence/host_tests.log"}
fms["verification"]["arm_compile"]["text_bytes"] = 25768
fms["verification"]["arm_compile"]["bss_bytes"] = 7213
fms["verification"]["layout_delta"] = {
    "ProductOperationPayload": {"before": 44, "after": 48},
    "OperationRequest": {"before": 160, "after": 160},
}
fms["blocked_by"] = ["T191-A-ABI", "T191-B-ADAPTER"]
fms["next_action"] = (
    "A: review the product-contract diff (mission_epoch/geofence config) and regenerate abi_lock.h/ABI "
    "evidence at revision 0x00010904. D: merge this revision, move C-FMS to REVIEW, record CR-C01/C02 "
    "disposition, and keep target/SIL/HIL NOT_RUN. C-M5 remains waiting for C-FMS acceptance; C-M6/M7 stay NOT_STARTED.")
fms_path.write_text(json.dumps(fms, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
fms_sha = sha(fms_path)

index_path = ROOT / "applications/core/status_requests/index.json"
index = json.loads(index_path.read_text(encoding="utf-8"))
index["generated_at"] = NOW
for s in index["submissions"]:
    if s["task_id"] == "T191-C-FMS":
        s["revision"] = 4
        s["state"] = "CR_C01_C02_IMPLEMENTED_PENDING_A_D_REVIEW"
    s["sha256"] = sha(ROOT / s["path"])
index["open_c_items"] = [
    "T191-C-FMS: CR-C01/C02 implemented; A must review the contract diff and regenerate abi_lock.h; D merge/REVIEW",
    "T191-C-M5: waiting for C-FMS acceptance (NONE-source slice prepared)",
    "T191-C-M6: blocked by MPC solver decision, ControlModelPort ABI, C-M5/B-REPLAY acceptance",
    "T191-C-M7: blocked by C-M6 and A's DShot/actuator submit contract",
]
index_path.write_text(json.dumps(index, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

cmd_path = ROOT / "applications/core/m4/hosttest/evidence/commands.json"
cmd = json.loads(cmd_path.read_text(encoding="utf-8"))
cmd["commands"][0]["result"] = ("Host build gcc 16.0.1 -std=c11 -Wall -Wextra -Werror against production contracts; "
                                "17 tests, 240 checks, 0 failures (CR-C01/C02 paths covered)")
cmd["commands"][1]["result"] = ("Cortex-M7 compile PASS (arm-none-eabi-gcc 14.3.1); text 25768 B; bss 7213 B; "
                                "M5_ARM_COMPILE_OK")
cmd_path.write_text(json.dumps(cmd, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

print(json.dumps({"fms": fms_sha, "index": sha(index_path), "commands": sha(cmd_path)}, indent=2))
