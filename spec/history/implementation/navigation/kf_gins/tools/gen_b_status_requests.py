"""Generate the four T191-B-* status requests (resubmission revision 2).

The requests close the B part of D-REV-04 by giving D an explicit, structured
record for every B task that already has files in the workspace.  They do not
claim ACCEPTED/CLOSED and keep the dependency chain visible.

Run from the Flight root:
    python applications/plugins/estimator/kf_gins_quad/tools/gen_b_status_requests.py
"""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))

BASELINE = "V1.9.1-GLOBAL-AUDIT-R3 / Schema 6.0 / Product-Port 0x00010903"
SUBMITTED_AT = "2026-11-04T23:35+08:00"
REVIEW_EVIDENCE = [
    "applications/plugins/estimator/kf_gins_quad/reviews/B_REVIEW_T191-D-REVIEW.md",
    "applications/plugins/estimator/kf_gins_quad/reviews/b_review_checks.json",
    "applications/plugins/estimator/kf_gins_quad/reviews/b_review_checks.py",
]

COMMON_ACK = {
    "prompt_received": True,
    "baseline_confirmed": True,
    "received_at": "2026-11-04T22:10+08:00",
    "prerequisite": {
        "task_id": "T191-D-REVIEW",
        "task_status_seen": "REVIEW",
        "update_sequence_seen": 3,
        "acknowledgement_seen": "用户2026-11-04指示执行Agent D任务；D接单、执行静态复核并提交交付（见spec/validation/T191-D-REVIEW）。",
        "verification_status_seen": "IN_PROGRESS",
        "reviewer_status": "B's independent review submitted (ACCEPT_WITH_CONDITIONS); A/C opinions and user Gate A approval still NOT_RUN.",
        "review_evidence": REVIEW_EVIDENCE,
    },
}

TASKS = [
    {
        "id": "T191-B-UPSTREAM",
        "path": "applications/plugins/estimator/kf_gins_quad/status_request_T191-B-UPSTREAM.json",
        "deliverables": [
            "packages/KF-GINS/UPSTREAM_LOCK.json",
            "packages/KF-GINS/MODIFICATIONS.md",
            "packages/KF-GINS/LICENSE",
            "packages/KF-GINS/trim/include/kfgins/quad_core.hpp",
            "packages/KF-GINS/trim/include/kfgins/quad_types.hpp",
            "packages/KF-GINS/trim/include/kfgins/quad_models.hpp",
            "packages/KF-GINS/trim/include/kfgins/quad_math.hpp",
            "packages/KF-GINS/trim/include/kfgins/quad_earth.hpp",
            "packages/KF-GINS/trim/src/quad_core.cpp",
            "packages/KF-GINS/trim/src/quad_models.cpp",
            "packages/KF-GINS/SConscript",
            "packages/KF-GINS/tools/gen_locks.py",
            "packages/deps.lock",
            "applications/plugins/estimator/kf_gins_quad/kf_gins_quad_port.h",
            "applications/plugins/estimator/kf_gins_quad/kf_gins_quad_port.cpp",
            "applications/plugins/estimator/kf_gins_quad/kf_gins_adapter.hpp",
            "applications/plugins/estimator/kf_gins_quad/kf_gins_adapter.cpp",
            "applications/plugins/estimator/kf_gins_quad/SConscript",
            "applications/plugins/estimator/kf_gins_quad/hosttest/",
            "applications/plugins/estimator/kf_gins_quad/reviews/",
            "build/validation/kf_gins_upstream/",
        ],
        "commands": [
            {"command": "python packages/KF-GINS/tools/gen_locks.py", "exit_code": 0,
             "result": "UPSTREAM_LOCK.json: 10 files, 0 hash mismatches; packages/deps.lock written"},
            {"command": "pwsh -NoProfile -File applications/plugins/estimator/kf_gins_quad/hosttest/run_host_tests.ps1",
             "exit_code": 0,
             "result": "test_jacobian 220/0, test_core 2636/0, test_port 179/0, test_contract_coexistence PASS, test_upstream_compare 426/0 (dpos 3.66e-8 m, dvel 4.33e-8 m/s)"},
            {"command": "arm-none-eabi-g++ -std=c++14 -fno-exceptions -fno-rtti -mcpu=cortex-m7 ... -c quad_core.cpp quad_models.cpp kf_gins_adapter.cpp kf_gins_quad_port.cpp",
             "exit_code": 0,
             "result": "4 objects compile; text 20288/3512/600/2104 B; nm -u has no malloc/free/operator new/__cxa_throw"},
            {"command": "python -m SCons -n (RTT_CC=gcc)", "exit_code": 0,
             "result": "default build unchanged; with KF_GINS_QUAD_ENABLE defined quad_core/quad_models/kf_gins_adapter/kf_gins_quad_port compile"},
        ],
        "verification": {
            "scope": "IMPLEMENTATION_HOST_AND_ARM_COMPILE",
            "status": "PASS (executor-run evidence; NOT a 09 ledger PASS and NOT independently reviewed)",
            "upstream": {"commit": "8291a93e49de513fe9d21f819500d39082ded611",
                         "files_matching_spec_record": 10, "files_total": 10,
                         "archive_sha256": "183a86c099126412b0d20a0f72de1625d9ecc73f01b5bcab16f69b6221a30dd7"},
            "instance_memory": {"host_sizeof_handle": 14112, "arm_bss": 14104},
            "evidence_paths": ["build/validation/kf_gins_upstream/", "packages/KF-GINS/UPSTREAM_LOCK.json"],
        },
        "not_run": [
            "REQ-NAV-KFG-001 official dataset reproduction (needs the pristine desktop build with yaml-cpp/abseil)",
            "H723 link/map/WCET/RTA (T191-A-TARGET)",
            "SIL/HIL and target board",
            "independent review and Gate C sign-off",
        ],
        "blocked_by": ["T191-D-REVIEW"],
        "known_issues_for_d": [
            "B-R02: the firmware does not link Eigen although 00 §12.2 names Eigen fixed-size; a CR decision is required.",
            "B-R03: contract headers need -fno-short-enums at the top-level build, currently only per-SConscript.",
            "F-01 latent defect found by B's replay test (stale gain columns) was fixed in packages/KF-GINS/trim; the KF-GINS Host suite was re-run clean.",
        ],
        "rollback": "Delete packages/KF-GINS, packages/deps.lock, applications/plugins/estimator/kf_gins_quad and the build/validation evidence; restore the one root Kconfig line added for M2/M3/health if the whole B batch is reverted.",
    },
    {
        "id": "T191-B-ADAPTER",
        "path": "applications/core/m2/status_request_T191-B-ADAPTER.json",
        "deliverables": [
            "applications/core/m2/include/m2_sensors.h",
            "applications/core/m2/src/m2_sensors.c",
            "applications/core/m2/SConscript",
            "applications/core/m2/Kconfig",
            "applications/core/m2/hosttest/test_m2.c",
            "applications/core/m2/hosttest/run_host_tests.ps1",
            "applications/sim/sensors/include/imu_sim.h",
            "applications/sim/sensors/src/imu_sim.c",
            "applications/sim/sensors/hosttest/test_imu_sim.c",
            "applications/sim/sensors/samples/imu_static_5x200hz.json",
            "applications/sim/sensors/samples/imu_rotation_5x200hz.json",
            "applications/sim/sensors/README.md",
            "applications/config/QUAD_X4/sensor_alignment.yaml",
            "applications/config/QUAD_X4/estimator_tuning.yaml",
            "build/validation/m2_adapter/",
        ],
        "commands": [
            {"command": "pwsh -NoProfile -File applications/core/m2/hosttest/run_host_tests.ps1",
             "exit_code": 0,
             "result": "test_m2 75/0; test_imu_sim 217/0; Cortex-M7 compile PASS (text 2956 B, no heap/exception symbols)"},
            {"command": "gcc -std=c11 -O2 -Wall -Wextra -Wconversion -static ... test_m2.c m2_sensors.c",
             "exit_code": 0, "result": "Host build and run PASS"},
            {"command": "python -m SCons -n (M2_SENSORS_ENABLE)", "exit_code": 0,
             "result": "CC applications\\core\\m2\\src\\m2_sensors.o"},
        ],
        "verification": {
            "scope": "IMPLEMENTATION_HOST_AND_ARM_COMPILE",
            "status": "PASS (executor-run; datasheet frames only, no real device bursts)",
            "datasheet_sources": ["doc/ADIS16505/adis16505.pdf Rev. C p.3-4, p.19-20",
                                  "doc/BMP585/bst-bmp585-ds003.pdf p.8, p.11"],
            "evidence_paths": ["build/validation/m2_adapter/"],
        },
        "not_run": [
            "real ADIS16505/BMP585 bursts through A's SPI/I2C/DMA path",
            "calibration and installation measurement (all sensor_alignment fields TBD)",
            "TimeService/PPS mapping for GNSS epochs",
            "data-set identification for estimator_tuning",
            "WCET/SIL/HIL",
        ],
        "blocked_by": ["T191-A-PLATFORM", "T191-B-UPSTREAM"],
        "known_issues_for_d": [
            "MagSample is intentionally not produced (no LSB/uT constant and no calibration evidence); MAG_AIDING stays DISABLED.",
            "UM982 is required and UART4/PPS stay enabled; the M2 PVT semantic mapper exists, but the native UM982 parser remains NOT_READY pending implementation/validation.",
            "sensor_alignment.yaml and estimator_tuning.yaml carry explicit TBDs that must block a Release build.",
        ],
        "rollback": "Delete applications/core/m2, applications/sim/sensors and applications/config/QUAD_X4.",
    },
    {
        "id": "T191-B-REPLAY",
        "path": "applications/core/m3/status_request_T191-B-REPLAY.json",
        "deliverables": [
            "applications/core/m3/include/ins_replay.hpp",
            "applications/core/m3/src/ins_replay.cpp",
            "applications/core/m3/SConscript",
            "applications/core/m3/Kconfig",
            "applications/core/m3/hosttest/test_replay.cpp",
            "applications/core/m3/hosttest/run_host_tests.ps1",
            "packages/KF-GINS/trim/include/kfgins/quad_core.hpp",
            "packages/KF-GINS/trim/src/quad_core.cpp",
            "build/validation/b_replay_fdir/",
        ],
        "commands": [
            {"command": "pwsh -NoProfile -File applications/core/m3/hosttest/run_host_tests.ps1",
             "exit_code": 0,
             "result": "test_replay 531/0; replay equivalence 3.455e-9 (float 32-byte IMU slot); Cortex-M7 compile PASS"},
            {"command": "python -m SCons -n (M3_INS_REPLAY_ENABLE + KF_GINS_QUAD_ENABLE)", "exit_code": 0,
             "result": "CXX applications\\core\\m3\\src\\ins_replay.o"},
        ],
        "verification": {
            "scope": "IMPLEMENTATION_HOST_AND_ARM_COMPILE",
            "status": "PASS (executor-run; synthetic IMU/observations, no measured WCET)",
            "checkpoint": {"bytes": 2304, "static_assert": True},
            "history": {"imu_slots": 256, "imu_slot_bytes": 32, "checkpoints": 24,
                        "journal_slots": 64, "journal_slot_bytes": 320,
                        "sizeof_history_bytes": 71720},
            "evidence_paths": ["build/validation/b_replay_fdir/test_replay.log", "build/validation/b_replay_fdir/arm_m3_replay.log"],
        },
        "not_run": [
            "measured replay WCET and 8% server budget",
            "replay of real device bursts and real GNSS/Baro observations",
            "SIL/HIL late/out-of-order/lost-observation injection",
        ],
        "blocked_by": ["T191-B-ADAPTER"],
        "known_issues_for_d": [
            "Replay is not bit-identical to the forward run: the 32-byte IMU slot stores floats (spec 07 §6.4), measured bound 3.5e-9.",
            "The shadow core must be initialized with the same QuadConfig before restore; documented in ins_replay.hpp.",
        ],
        "rollback": "Delete applications/core/m3; revert the checkpoint API and the F-01 fix in packages/KF-GINS/trim only if the whole batch is reverted.",
    },
    {
        "id": "T191-B-FDIR",
        "path": "applications/services/health/status_request_T191-B-FDIR.json",
        "deliverables": [
            "applications/services/health/include/sensor_integrity.h",
            "applications/services/health/src/sensor_integrity.c",
            "applications/services/health/SConscript",
            "applications/services/health/Kconfig",
            "applications/services/SConscript",
            "applications/services/health/hosttest/test_health.c",
            "applications/services/health/hosttest/run_host_tests.ps1",
            "build/validation/b_replay_fdir/",
        ],
        "commands": [
            {"command": "pwsh -NoProfile -File applications/services/health/hosttest/run_host_tests.ps1",
             "exit_code": 0,
             "result": "test_health 102/0; Cortex-M7 compile PASS"},
            {"command": "python -m SCons -n (SENSOR_INTEGRITY_ENABLE)", "exit_code": 0,
             "result": "CC applications\\services\\health\\src\\sensor_integrity.o"},
        ],
        "verification": {
            "scope": "IMPLEMENTATION_HOST_AND_ARM_COMPILE",
            "status": "PASS (executor-run; no estimator handle, so it cannot modify the active P)",
            "evidence_paths": ["build/validation/b_replay_fdir/test_health.log", "build/validation/b_replay_fdir/arm_health.log"],
        },
        "not_run": [
            "integration with M2 diagnostics and the M3 capability publisher",
            "combined fault-injection matrix (GNSS loss/recovery, IMU stale, baro stuck)",
            "SIL/HIL",
        ],
        "blocked_by": ["T191-B-REPLAY"],
        "known_issues_for_d": [
            "Single-instance sources have no redundancy by construction; isolating the only IMU must degrade the navigation capability, not switch to a phantom source.",
        ],
        "rollback": "Delete applications/services/health and applications/services/SConscript; remove the applications/Kconfig line sourcing services/health/Kconfig.",
    },
]

NEXT_ACTIONS = {
    "T191-B-UPSTREAM": "D: merge the resubmission (dedup T191-B-UPSTREAM:2), record B's T191-D-REVIEW reviewer evidence, decide B-R02/B-R03, then route the task; on explicit acknowledgement B starts the official dataset reproduction for REQ-NAV-KFG-001.",
    "T191-B-ADAPTER": "D: record the submission, keep the task blocked by A-PLATFORM and B-UPSTREAM, and schedule it after both are accepted; B then binds real device bursts and TimeService.",
    "T191-B-REPLAY": "D: record the submission, keep blocked by B-ADAPTER; B then replaces synthetic fixtures with recorded bursts and measures the replay budget.",
    "T191-B-FDIR": "D: record the submission, keep blocked by B-REPLAY; B then integrates the integrity manager with M2 diagnostics and the M3 capability publisher.",
}

RESUME = {
    "T191-B-UPSTREAM": "T191-D-REVIEW is REVIEW (seq 3) with B's non-author opinion submitted; resume after D merges this resubmission and records the dependency decision.",
    "T191-B-ADAPTER": "READY_FOR_ASSIGNMENT from B's side; resume after T191-A-PLATFORM and T191-B-UPSTREAM are accepted by D.",
    "T191-B-REPLAY": "READY_FOR_ASSIGNMENT from B's side; resume after T191-B-ADAPTER is accepted.",
    "T191-B-FDIR": "READY_FOR_ASSIGNMENT from B's side; resume after T191-B-REPLAY is accepted.",
}


def build(task):
    return {
        "task_id": task["id"],
        "task_type": "IMPLEMENTATION",
        "baseline": BASELINE,
        "update_sequence": 2,
        "submission": {
            "revision": 2,
            "submitted_at": SUBMITTED_AT,
            "reason": (
                "Resubmission requested by the user after T191-D-REVIEW moved to REVIEW/update_sequence=3 "
                "with an acknowledgement. It closes the B part of D-REV-04 (handoff records still READY while "
                "the source paths already contain deliverables), adds B's independent review of T191-D-REVIEW, "
                "and does not change the delivered files or the verification results."
            ),
            "dedup_key": task["id"] + ":2",
        },
        "requested_transition": "READY_FOR_ASSIGNMENT->ASSIGNED",
        "transition_note": (
            "B confirms prompt and baseline receipt. Prerequisites are not ACCEPTED: "
            + ", ".join(task["blocked_by"])
            + ". If D enforces dependency gating, record BLOCKED with resume_state instead of ASSIGNED; "
              "no ACCEPTED/CLOSED is requested."
        ),
        "assignee": "B",
        "acknowledgement": COMMON_ACK,
        "deliverable_refs": task["deliverables"],
        "actual_commands": task["commands"],
        "verification": task["verification"],
        "not_run": task["not_run"],
        "reviewer": "D",
        "blocked_by": task["blocked_by"],
        "resume_state": RESUME[task["id"]],
        "next_owner": "D",
        "next_action": NEXT_ACTIONS[task["id"]],
        "known_issues_for_d": task["known_issues_for_d"],
        "reviewer_evidence": {
            "task_reviewed": "T191-D-REVIEW",
            "reviewer": "B",
            "verdict": "ACCEPT_WITH_CONDITIONS",
            "checks": 10,
            "failed_checks_in_snapshot": 2,
            "fail_note": "B-02 closes with this submission; B-10 is the unassigned calibration/GNSS task gap.",
            "conditions": [
                "B-1: D merges the four B status requests and marks the B part of D-REV-04 as submitted.",
                "B-2: D decides B-R02 (Eigen vs dependency-free fixed-size math) and B-R03 (-fno-short-enums top-level rule).",
                "B-3: after D confirms, B patches 05 §7 notation (B-R01) and the pipeline delay bound in sensor_alignment.",
                "B-4: D assigns or explicitly waives B-R05 (Kconfig aggregation), B-R06 (applications/calibration) and B-R07 (GNSS native backend / Mag calibration tasks).",
            ],
            "evidence": REVIEW_EVIDENCE,
        },
        "commit": None,
        "commit_note": "Workspace has no Git metadata; the source snapshot is the SHA256 lists in build/validation and the delivered file lists above. No pre-existing file was modified except one root Kconfig line sourcing applications/Kconfig.",
        "rollback": task["rollback"],
    }


def main():
    written = []
    for task in TASKS:
        path = os.path.join(ROOT, task["path"].replace("/", os.sep))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(build(task), handle, indent=2, ensure_ascii=False)
            handle.write("\n")
        written.append(task["path"])
    for path in written:
        print("wrote", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
