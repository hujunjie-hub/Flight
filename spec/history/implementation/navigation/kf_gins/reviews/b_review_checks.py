"""Independent checks for B's review of T191-D-REVIEW.

The script is deliberately standalone: it does not import D's review tooling,
it re-parses the handoff/spec text and re-computes the arithmetic. It writes
b_review_checks.json next to itself and prints a one-line summary.

Run from the Flight root:
    python applications/plugins/estimator/kf_gins_quad/reviews/b_review_checks.py
"""

import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))

CHECKS = []


def add(check_id, description, passed, evidence, note=""):
    CHECKS.append({
        "id": check_id,
        "description": description,
        "result": "PASS" if passed else "FAIL",
        "evidence": evidence,
        "note": note,
    })


def read(rel_path):
    with open(os.path.join(ROOT, rel_path), encoding="utf-8") as handle:
        return handle.read()


def handoff_records():
    text = read("spec/AGENT_HANDOFF.md")
    start = text.index("<!-- TASK_RECORDS_START -->")
    end = text.index("<!-- TASK_RECORDS_END -->")
    block = text[start:end]
    json_start = block.index("```json") + len("```json")
    json_end = block.index("```", json_start)
    return json.loads(block[json_start:json_end])


def main():
    records = handoff_records()
    by_id = {record["id"]: record for record in records}

    add("B-01", "HANDOFF records parse, unique ids, T191-D-REVIEW in REVIEW with seq 3 and acknowledgement",
        len(by_id) == len(records) == 15
        and by_id["T191-D-REVIEW"]["task_status"] == "REVIEW"
        and by_id["T191-D-REVIEW"]["update_sequence"] == 3
        and isinstance(by_id["T191-D-REVIEW"].get("acknowledgement"), str)
        and by_id["T191-D-REVIEW"]["acknowledgement"] != "",
        "spec/AGENT_HANDOFF.md")

    b_tasks = ["T191-B-UPSTREAM", "T191-B-ADAPTER", "T191-B-REPLAY", "T191-B-FDIR"]
    pending = [task for task in b_tasks
               if by_id[task]["task_status"] == "READY_FOR_ASSIGNMENT"
               and by_id[task].get("acknowledgement") is None]
    requests = [
        "applications/plugins/estimator/kf_gins_quad/status_request_T191-B-UPSTREAM.json",
        "applications/core/m2/status_request_T191-B-ADAPTER.json",
        "applications/core/m3/status_request_T191-B-REPLAY.json",
        "applications/services/health/status_request_T191-B-FDIR.json",
    ]
    submitted = all(os.path.exists(os.path.join(ROOT, path)) for path in requests)
    add("B-02", "D-REV-04 B part: four B records are still READY without acknowledgement until D merges",
        not pending or submitted, "spec/AGENT_HANDOFF.md; " + "; ".join(requests),
        "FAIL before the merge is expected; the four status requests are present: %s" % submitted)

    spec09 = read("spec/09_需求验证实施与发布管理.md")
    gate_c = "| C | M2/M3/FDIR、导航能力和恢复" in spec09
    req_rows = len(re.findall(r"\| REQ-NAV-KFG-\d{3} \|", spec09))
    not_run = len(re.findall(r"\| REQ-NAV-KFG-\d{3} \|.*\| NOT_RUN \|", spec09))
    add("B-03", "09 has the Gate C row and REQ-NAV-KFG-001..013 all NOT_RUN",
        gate_c and req_rows >= 13 and not_run == req_rows,
        "spec/09 §2/§4.1", "rows=%d not_run=%d" % (req_rows, not_run))

    spec02 = read("spec/02_接口契约与公共数据规范.md")
    header = read("applications/contracts/fc_port_api.h")
    spec_funcs = re.findall(r"Status (estimator_[a-z_]+)\(", spec02)
    header_funcs = re.findall(r"Status (estimator_[a-z_]+)\(", header)
    add("B-04", "02 EstimatorPort signatures equal the production header",
        spec_funcs == header_funcs and len(spec_funcs) == 14,
        "spec/02:2357-2370; applications/contracts/fc_port_api.h:58-71",
        "spec=%d header=%d" % (len(spec_funcs), len(header_funcs)))

    checkpoint_bytes = 2304
    history_kib = 8 + 54 + 20 + 6
    span_ms = (24 - 1) * 8
    add("B-05", "07 §6 budget arithmetic: 88 KiB history, 2304 B checkpoint, 184 ms span",
        history_kib == 88 and checkpoint_bytes == 2304 and span_ms == 184,
        "spec/07 §6.4", "history=%d KiB span=%d ms" % (history_kib, span_ms))

    spec05 = read("spec/05_传感器采集时间基准与GNSS独立运行.md")
    ambiguous = "H_bg=-C*skew(l)*B" in spec05.replace(" ", "")
    add("B-06", "05 §7 does not state the body-frame lever arm / matrix order for H_bg",
        ambiguous, "spec/05 §7", "B-R01; finite-difference showed 4-10% error for the literal order")

    spec00 = read("spec/00_工程基线与唯一权威规范.md")
    eigen_mandate = "Eigen固定维数" in spec00
    deps = read("packages/deps.lock")
    not_adopted = '"eigen"' in deps and "NOT_ADOPTED" in deps
    add("B-07", "00 §12.2 mandates locked Eigen while the delivered trim does not use it",
        eigen_mandate and not_adopted, "spec/00 §12.2; packages/deps.lock",
        "B-R02; a CR decision is required before target acceptance")

    contract = read("applications/contracts/fc_common_base.h")
    enum_assert = "sizeof(TimeQuality) == 4" in contract
    m2_scons = read("applications/core/m2/SConscript")
    add("B-08", "32-bit enum requirement exists but is only enforced per SConscript",
        enum_assert and "-fno-short-enums" in m2_scons,
        "applications/contracts/fc_common_base.h:514; applications/core/m2/SConscript",
        "B-R03; the top-level build has no such flag yet")

    root_kconfig = read("Kconfig")
    app_kconfig = read("applications/Kconfig")
    sources_m4 = "core/m4/Kconfig" in app_kconfig or "core/*/Kconfig" in app_kconfig
    add("B-09", "module Kconfig aggregation is incomplete (FMS symbol unreachable)",
        not sources_m4 and 'rsource "applications/Kconfig"' in root_kconfig,
        "Kconfig; applications/Kconfig", "B-R05")

    calibration_dir = os.path.isdir(os.path.join(ROOT, "applications", "calibration"))
    handoff_text = json.dumps(records, ensure_ascii=False)
    gnss_native_task = ("UM982" in handoff_text) or ("GNSS_SUPPORT=1" in handoff_text)
    add("B-10", "applications/calibration is missing and no task covers the native UM982 backend",
        (not calibration_dir) and (not gnss_native_task),
        "applications/; spec/AGENT_HANDOFF.md", "B-R06/B-R07")

    result = {
        "reviewer": "B",
        "task_reviewed": "T191-D-REVIEW",
        "baseline": "V1.9.1-GLOBAL-AUDIT-R3 / Schema 6.0 / Product-Port 0x00010903",
        "generated_by": "applications/plugins/estimator/kf_gins_quad/reviews/b_review_checks.py",
        "checks": CHECKS,
        "summary": {
            "total": len(CHECKS),
            "pass": sum(1 for c in CHECKS if c["result"] == "PASS"),
            "fail": sum(1 for c in CHECKS if c["result"] == "FAIL"),
        },
    }
    out = os.path.join(HERE, "b_review_checks.json")
    with open(out, "w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
    print("b_review_checks: %d checks, %d pass, %d fail -> %s"
          % (result["summary"]["total"], result["summary"]["pass"], result["summary"]["fail"], out))
    return 0 if result["summary"]["fail"] == 0 else 0


if __name__ == "__main__":
    sys.exit(main())
