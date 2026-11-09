#!/usr/bin/env python3
"""Build/refresh Agent C status submissions and the D-REV-04 C-part index.

- T191-C-FMS: revision 3 (supersedes the revision-2 file)
- T191-C-M5 : revision 2
- T191-C-M6 : first submission, NOT_STARTED / blocked
- T191-C-M7 : first submission, NOT_STARTED / blocked
- index.json: what D must merge to close the C part of D-REV-04

Does not modify HANDOFF, 09 or any spec file.
"""
from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone, timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]  # .../Flight
NOW = datetime.now(timezone(timedelta(hours=8))).replace(microsecond=0).isoformat()
BASELINE = "V1.9.1-GLOBAL-AUDIT-R3 / Schema 6.0 / Product-Port 0x00010903"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path: Path, data: dict) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return sha256(path)


review_json = json.loads((ROOT / "applications/core/m4/reviews/c_review_checks.json")
                         .read_text(encoding="utf-8"))
review_summary = review_json["summary"]
review_opinion = {
    "task": "T191-D-REVIEW",
    "path": "applications/core/m4/reviews/C_REVIEW_T191-D-REVIEW.md",
    "machine_evidence": [
        "applications/core/m4/reviews/c_review_checks.json",
        "applications/core/m4/reviews/c_review_checks.py",
    ],
    "conclusion": "ACCEPT_WITH_CONDITIONS",
    "summary": review_summary,
    "status": "SUBMITTED_ON_FILE",
    "note": "C's non-author opinion is on file; D merges it when T191-D-REVIEW proceeds to closure. It is not an ACCEPTED state and does not replace B's opinion or user Gate A approval.",
}

d_rev_04 = {
    "role": "C",
    "status": "SUBMITTED_FOR_MERGE",
    "tasks": ["T191-C-FMS", "T191-C-M5", "T191-C-M6", "T191-C-M7"],
    "index": "applications/core/status_requests/index.json",
    "note": "C submits all four status requests so D can close the C part of D-REV-04. C does not edit AGENT_HANDOFF; closure remains a D action.",
}

# ---- T191-C-FMS revision 3 ------------------------------------------------
fms_path = ROOT / "applications/core/m4/status_request_T191-C-FMS.json"
fms = json.loads(fms_path.read_text(encoding="utf-8"))
old_fms = sha256(fms_path)
fms["submission"] = {
    "revision": 3,
    "submitted_at": NOW,
    "supersedes_sha256": old_fms,
    "reason": "Consolidated resubmission requested to close the C part of D-REV-04. Refreshes the T191-D-REVIEW reviewer status (A and C opinions in, B pending), adds the C review opinion reference, and keeps the implementation baseline and evidence unchanged.",
    "dedup_key": "T191-C-FMS:3",
}
fms["d_rev_04_c_part"] = d_rev_04
fms["review_opinion"] = review_opinion
fms["acknowledgement"]["prerequisite"]["reviewer_status"] = (
    "A and C independent opinions submitted; B opinion and user Gate A approval remain NOT_RUN.")
fms["next_action"] = (
    "D: merge C's four submissions (C-FMS rev3, C-M5 rev2, C-M6/C-M7 NOT_STARTED), "
    "close the C part of D-REV-04, decide ASSIGNED/BLOCKED per task, open the mission_epoch "
    "and geofence-entry CRs, and merge C's T191-D-REVIEW opinion when the review proceeds.")
fms["blocked_by"] = ["T191-A-ABI", "T191-B-ADAPTER"]
fms["next_owner"] = "D"
fms_sha = write(fms_path, fms)

# ---- T191-C-M5 revision 2 -------------------------------------------------
m5_path = ROOT / "applications/core/m5/status_request_T191-C-M5.json"
m5 = json.loads(m5_path.read_text(encoding="utf-8"))
old_m5 = sha256(m5_path)
m5["submission"] = {
    "revision": 2,
    "submitted_at": NOW,
    "supersedes_sha256": old_m5,
    "reason": "Consolidated resubmission for the C part of D-REV-04: records C's T191-D-REVIEW opinion and keeps the NONE-obstacle M5 slice unchanged.",
    "dedup_key": "T191-C-M5:2",
}
m5["d_rev_04_c_part"] = d_rev_04
m5["review_opinion"] = review_opinion
m5["next_action"] = (
    "D: merge C-M5 rev2 together with C-FMS rev3 and the M6/M7 notices; register the "
    "NONE-source Guidance evidence and keep A*/map/collision NOT_RUN; merge C's "
    "T191-D-REVIEW opinion when the review proceeds.")
m5["next_owner"] = "D"
m5["blocked_by"] = ["T191-C-FMS", "T191-B-REPLAY"]
m5_sha = write(m5_path, m5)

# ---- T191-C-M6 first submission (NOT_STARTED) -----------------------------
m6 = {
    "task_id": "T191-C-M6",
    "task_type": "IMPLEMENTATION",
    "baseline": BASELINE,
    "update_sequence": 2,
    "submission": {
        "revision": 1,
        "submitted_at": NOW,
        "supersedes_sha256": None,
        "reason": "Status notice for the C part of D-REV-04: no implementation has been started. C does not request ASSIGNED and does not claim any M6 code, test or evidence.",
        "dedup_key": "T191-C-M6:1",
    },
    "state": "NOT_STARTED",
    "requested_transition": "NO_CHANGE",
    "transition_note": "T191-C-M6 remains READY_FOR_ASSIGNMENT. If D routes it, record ASSIGNED only after the blockers below are resolved; otherwise record BLOCKED with the blocked_by list.",
    "assignee": "C",
    "d_rev_04_c_part": d_rev_04,
    "review_opinion": review_opinion,
    "blocked_by": [
        "T191-C-M5",
        "T191-B-REPLAY",
        "MPC_SOLVER_PACKAGE_DECISION",
        "CONTROL_MODEL_PORT_IMPLEMENTATION",
    ],
    "blockers": [
        {
            "id": "C-M6-B1",
            "item": "T191-C-M5",
            "fact": "C-M5 is delivered as an independent NONE-source slice but the record is READY_FOR_ASSIGNMENT/NOT_RUN and D has not reviewed it.",
        },
        {
            "id": "C-M6-B2",
            "item": "T191-B-REPLAY",
            "fact": "Record is READY_FOR_ASSIGNMENT/NOT_RUN. B artifacts exist under applications/core/m3/src/ins_replay.cpp but no accepted replay contract/evidence is available.",
        },
        {
            "id": "C-M6-B3",
            "item": "MPC solver package",
            "fact": "04 requires a 50 Hz LTI-MPC QP; packages/ contains only CMSIS-Core, KF-GINS and STM32H7 HAL/CMSIS packages. No OSQP/QP solver is baselined, and adding one changes dependencies/resources/ABI -> CR + A/D decision required before C can implement.",
            "evidence": "packages/ directory listing",
        },
        {
            "id": "C-M6-B4",
            "item": "ControlModelPort implementation",
            "fact": "M6 consumes ControlModelView/constraints; no selected control model implementation or Profile binding exists yet (C's M6 task lists packages/ and control_model plugin as its landing zone, but the model ABI is not frozen).",
        },
    ],
    "deliverable_refs": [],
    "actual_commands": [],
    "verification": {
        "scope": "IMPLEMENTATION",
        "status": "NOT_RUN",
        "note": "Nothing implemented; no test, build or evidence is claimed.",
    },
    "not_run": [
        "M6-01..M6-07 (MPC, SO(3), rate PID, allocator, fallback, anti-windup, controller Port)",
        "ControlModel view/constraints binding",
        "Host/SIL closed loop, target WCET and M7 integration",
    ],
    "reviewer": "D",
    "resume_state": "READY_FOR_ASSIGNMENT",
    "next_owner": "D",
    "next_action": "D/A: decide the MPC solver package and freeze the ControlModelPort ABI; D: accept C-M5 and B-REPLAY; then route T191-C-M6 to C.",
    "commit": None,
    "commit_note": "No code changed; workspace has no Git metadata.",
    "rollback": "No files added by this notice; nothing to roll back.",
}
m6_path = ROOT / "applications/core/m6/status_request_T191-C-M6.json"
m6_sha = write(m6_path, m6)

# ---- T191-C-M7 first submission (NOT_STARTED) -----------------------------
m7 = {
    "task_id": "T191-C-M7",
    "task_type": "IMPLEMENTATION",
    "baseline": BASELINE,
    "update_sequence": 2,
    "submission": {
        "revision": 1,
        "submitted_at": NOW,
        "supersedes_sha256": None,
        "reason": "Status notice for the C part of D-REV-04: no implementation has been started. C does not claim M7 code or the DShot waveform.",
        "dedup_key": "T191-C-M7:1",
    },
    "state": "NOT_STARTED",
    "requested_transition": "NO_CHANGE",
    "transition_note": "T191-C-M7 remains READY_FOR_ASSIGNMENT. M7 needs A's DShot/actuator submit contract before it can be implemented without inventing TIM/DMA access.",
    "assignee": "C",
    "d_rev_04_c_part": d_rev_04,
    "review_opinion": review_opinion,
    "blocked_by": ["T191-C-M6", "T191-A-PLATFORM", "DSHOT_DRIVER_CONTRACT"],
    "blockers": [
        {
            "id": "C-M7-B1",
            "item": "T191-C-M6",
            "fact": "M6 is NOT_STARTED; M7 consumes its ActuatorDemand and anti-windup context.",
        },
        {
            "id": "C-M7-B2",
            "item": "T191-A-PLATFORM / DShot driver",
            "fact": "A's platform evidence records OUT-DSHOT as MISSING: 'TIM1 UP DMA burst + DShot TC->Update' (applications/platform/rtthread53/evidence/platform_gap.md). M7 must not touch TIM1/DMA2 registers itself (10 §1).",
            "evidence": "applications/platform/rtthread53/evidence/platform_gap.md line OUT-DSHOT",
        },
        {
            "id": "C-M7-B3",
            "item": "DShot submit contract",
            "fact": "07 §3.0.1 fixes the frame buffer semantics (CNDTR=68, active zero/preload bit0, buffer bit1-15 + 2 zero-tail, absolute raw deadline) but no driver header/API or double-buffer submission contract has been delivered; M7 cannot encode/submit a frame without inventing it.",
        },
    ],
    "deliverable_refs": [],
    "actual_commands": [],
    "verification": {
        "scope": "IMPLEMENTATION",
        "status": "NOT_RUN",
        "note": "Nothing implemented; no packet, guard, receipt or waveform evidence is claimed.",
    },
    "not_run": [
        "M7 permit/demand/epoch/TTL guard",
        "thrust-curve calibration and DShot packet encoding",
        "ActuatorCommitReceipt and independent poll guard",
        "target waveform / logic analyzer / HIL",
    ],
    "reviewer": "D",
    "resume_state": "READY_FOR_ASSIGNMENT",
    "next_owner": "D",
    "next_action": "A: deliver the DShot/actuator submit contract and driver evidence; D: accept C-M6 and route M7; then C implements M7 against that contract.",
    "commit": None,
    "commit_note": "No code changed; workspace has no Git metadata.",
    "rollback": "No files added by this notice; nothing to roll back.",
}
m7_path = ROOT / "applications/core/m7/status_request_T191-C-M7.json"
m7_sha = write(m7_path, m7)

# ---- index ---------------------------------------------------------------
index = {
    "owner": "C",
    "generated_at": NOW,
    "baseline": BASELINE,
    "purpose": "C's consolidated status submissions to close the C part of D-REV-04 (HANDOFF vs worktree).",
    "d_rev_04_c_part": {
        "status": "SUBMITTED_FOR_MERGE",
        "tasks": ["T191-C-FMS", "T191-C-M5", "T191-C-M6", "T191-C-M7"],
        "closure_instruction": "D merges these four submissions (or records BLOCKED where requested) and updates AGENT_HANDOFF. C does not edit HANDOFF or 09.",
    },
    "submissions": [
        {"task_id": "T191-C-FMS", "revision": 3, "state": "INDEPENDENT_PREPARATION_DELIVERED",
         "path": "applications/core/m4/status_request_T191-C-FMS.json", "sha256": fms_sha},
        {"task_id": "T191-C-M5", "revision": 2, "state": "PARTIAL_DELIVERED_NONE_SOURCE_ONLY",
         "path": "applications/core/m5/status_request_T191-C-M5.json", "sha256": m5_sha},
        {"task_id": "T191-C-M6", "revision": 1, "state": "NOT_STARTED",
         "path": "applications/core/m6/status_request_T191-C-M6.json", "sha256": m6_sha},
        {"task_id": "T191-C-M7", "revision": 1, "state": "NOT_STARTED",
         "path": "applications/core/m7/status_request_T191-C-M7.json", "sha256": m7_sha},
    ],
    "review_opinion": review_opinion,
    "open_c_items": [
        "T191-C-M6 blocked by MPC solver decision, ControlModelPort ABI, C-M5/B-REPLAY acceptance",
        "T191-C-M7 blocked by C-M6 and A's DShot/actuator submit contract",
        "T191-C-FMS/C-M5 await D review and 09 registration",
        "C-REV-01 mission_epoch CR, C-REV-02 geofence-entry CR (from C's T191-D-REVIEW opinion)",
    ],
}
index_path = ROOT / "applications/core/status_requests/index.json"
index_sha = write(index_path, index)

print(json.dumps({
    "index": str(index_path), "sha256": index_sha,
    "FMS": fms_sha, "M5": m5_sha, "M6": m6_sha, "M7": m7_sha,
}, indent=2))
