#!/usr/bin/env python3
"""Record the current C state after D's C-FMS REVIEW merge (no status transition).

Does not touch AGENT_HANDOFF, 09 or any submission revision: it adds a
read-only status notice and refreshes the live index metadata.
"""
from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone, timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
NOW = datetime.now(timezone(timedelta(hours=8))).replace(microsecond=0).isoformat()


def sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


notice = {
    "owner": "C",
    "generated_at": NOW,
    "baseline": "V1.9.1-GLOBAL-AUDIT-R3 / Schema 6.0 / Product-Port 0x00010904",
    "purpose": "Read-only C state snapshot after D merged C-FMS rev4 into REVIEW; no transition is requested.",
    "handoff_observed": {
        "T191-C-FMS": "REVIEW/seq5",
        "T191-C-M5": "READY_FOR_ASSIGNMENT/seq4",
        "T191-C-M6": "READY_FOR_ASSIGNMENT/seq3",
        "T191-C-M7": "READY_FOR_ASSIGNMENT/seq3",
    },
    "tasks": [
        {
            "task_id": "T191-C-FMS",
            "state": "REVIEW_PENDING_GATE_D",
            "detail": "spec/11 §4.1 now records CR-V191-C01 = CLOSED and CR-V191-C02 = SPECIFIED (implemented; Gate D review and target/SIL/HIL evidence still NOT_RUN). C-FMS is REVIEW/seq5; C has no further code action until D completes the review/acceptance.",
            "blocked_by": ["Gate D review of T191-C-FMS", "CR-V191-C02 closure"],
            "evidence": [
                "spec/11_审核问题闭环与变更追踪.md §4.1",
                "applications/core/m4/status_request_T191-C-FMS.json (revision 4)",
                "applications/core/m4/hosttest/evidence/host_tests.log",
                "applications/core/m4/hosttest/evidence/arm_build.log",
            ],
        },
        {
            "task_id": "T191-C-M5",
            "state": "WAITING_FOR_C_FMS",
            "detail": "NONE-source M5 slice is prepared (10 tests/72 checks) but C-M5 remains READY; start after C-FMS leaves REVIEW/CR finalization.",
            "blocked_by": ["T191-C-FMS acceptance"],
        },
        {
            "task_id": "T191-C-M6",
            "state": "NOT_STARTED_BLOCKED",
            "blocked_by": ["MPC solver selection/package", "ControlModelPort ABI", "C-M5 acceptance", "B-REPLAY acceptance"],
            "evidence": [
                "packages/ has no QP/MPC solver (CMSIS-Core, KF-GINS, STM32H7 HAL/CMSIS only)",
                "applications/plugins has no control_model implementation",
            ],
        },
        {
            "task_id": "T191-C-M7",
            "state": "NOT_STARTED_BLOCKED",
            "blocked_by": ["T191-C-M6", "A DShot/actuator submit contract"],
            "evidence": [
                "applications/platform/rtthread53/evidence/platform_gap.md: OUT-DSHOT = MISSING (TIM1 UP DMA burst + DShot TC->Update)",
            ],
        },
    ],
    "needed_to_unblock": {
        "D": "Review/accept T191-C-FMS at Gate D and close CR-V191-C02; then C-M5 can start.",
        "D/A": "Choose and baseline an MPC solver package and freeze the ControlModelPort ABI for C-M6.",
        "A": "Deliver the DShot/actuator submit contract and driver evidence for C-M7.",
        "D": "Accept C-M5/B-REPLAY before C-M6 integration.",
    },
}
notice_path = ROOT / "applications/core/status_requests/status_update_20261105_C.json"
notice_path.write_text(json.dumps(notice, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

index_path = ROOT / "applications/core/status_requests/index.json"
index = json.loads(index_path.read_text(encoding="utf-8"))
index["generated_at"] = NOW
states = {
    "T191-C-FMS": {
        "state": "REVIEW_PENDING_GATE_D",
        "handoff": "REVIEW/seq5",
        "note": "CR-C01 CLOSED; CR-C02 SPECIFIED pending Gate D; waiting D review/acceptance.",
    },
    "T191-C-M5": {"state": "WAITING_FOR_C_FMS", "handoff": "READY_FOR_ASSIGNMENT/seq4"},
    "T191-C-M6": {
        "state": "NOT_STARTED_BLOCKED",
        "handoff": "READY_FOR_ASSIGNMENT/seq3",
        "blockers": ["MPC solver selection/package", "ControlModelPort ABI", "C-M5 acceptance", "B-REPLAY acceptance"],
    },
    "T191-C-M7": {
        "state": "NOT_STARTED_BLOCKED",
        "handoff": "READY_FOR_ASSIGNMENT/seq3",
        "blockers": ["T191-C-M6", "A DShot/actuator submit contract (OUT-DSHOT missing)"],
    },
}
for s in index["submissions"]:
    st = states.get(s["task_id"], {})
    s.update(st)
    s["sha256"] = sha(ROOT / s["path"])
index["status_notice"] = {
    "path": "applications/core/status_requests/status_update_20261105_C.json",
    "sha256": sha(notice_path),
}
index_path.write_text(json.dumps(index, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({"notice": sha(notice_path), "index": sha(index_path)}, indent=2))
