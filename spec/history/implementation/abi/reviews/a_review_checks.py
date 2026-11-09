#!/usr/bin/env python3
"""Agent A independent checks for T191-D-REVIEW (non-author review).

Reads the current spec tree only; does not modify HANDOFF, 09, 07 or any D
artifact.  Output: a_review_checks.json next to this script.
Run from the Flight project root:
  python applications/tests/abi/reviews/a_review_checks.py
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[4]
SPEC = ROOT / "spec"
OUT = Path(__file__).resolve().parent / "a_review_checks.json"
ALLOWED_STATES = {
    "READY_FOR_ASSIGNMENT", "ASSIGNED", "IN_PROGRESS", "REVIEW",
    "ACCEPTED", "CLOSED", "BLOCKED",
}


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(name: str) -> str:
    return (SPEC / name).read_text(encoding="utf-8")


def handoff_tasks():
    text = (SPEC / "AGENT_HANDOFF.md").read_text(encoding="utf-8")
    blocks = re.findall(r"```json\n(.*?)```", text, re.S)
    tasks = []
    for block in blocks:
        parsed = json.loads(block)
        if isinstance(parsed, list):
            tasks.extend(parsed)
        elif isinstance(parsed, dict):
            tasks.append(parsed)
    return [t for t in tasks if isinstance(t, dict) and "id" in t]


def main() -> int:
    checks = []
    obs = []

    def add(name, ok, detail):
        checks.append({"id": name, "status": "PASS" if ok else "FAIL", "detail": detail})

    tasks = handoff_tasks()
    ids = {t["id"] for t in tasks}
    add("handoff_count_and_unique_ids", len(tasks) == 15 and len(ids) == 15,
        {"tasks": len(tasks), "unique": len(ids)})

    owner_independence = all(t.get("owner") not in (t.get("reviewer") or []) for t in tasks)
    add("handoff_owner_not_reviewer", owner_independence,
        {"violations": [t["id"] for t in tasks if t.get("owner") in (t.get("reviewer") or [])]})

    missing_deps = [(t["id"], d) for t in tasks for d in t.get("depends_on", []) if d not in ids]
    add("handoff_dependencies_exist", not missing_deps, {"missing": missing_deps})

    graph = {t["id"]: list(t.get("depends_on", [])) for t in tasks}
    cyclic = []
    def visit(node, path):
        if node in path:
            cyclic.append(node)
            return
        for dep in graph.get(node, []):
            visit(dep, path | {node})
    for node in graph:
        visit(node, set())
    add("handoff_dag_acyclic", not cyclic, {"cyclic": sorted(set(cyclic))})

    bad_states = [t["id"] for t in tasks if t.get("task_status") not in ALLOWED_STATES]
    add("handoff_states_legal", not bad_states, {"bad": bad_states,
        "states": sorted({t.get("task_status") for t in tasks})})

    drev = next((t for t in tasks if t["id"] == "T191-D-REVIEW"), None)
    add("d_review_state_review_seq3",
        drev is not None and drev.get("task_status") == "REVIEW"
        and drev.get("update_sequence") == 3 and bool(drev.get("acknowledgement")),
        {"status": drev.get("task_status") if drev else None,
         "update_sequence": drev.get("update_sequence") if drev else None,
         "ack_present": bool(drev.get("acknowledgement")) if drev else False})

    abi = next((t for t in tasks if t["id"] == "T191-A-ABI"), None)
    add("a_abi_record_ready_seq2",
        abi is not None and abi.get("task_status") == "READY_FOR_ASSIGNMENT"
        and abi.get("update_sequence") == 2 and abi.get("acknowledgement") is None,
        {"status": abi.get("task_status") if abi else None,
         "update_sequence": abi.get("update_sequence") if abi else None})

    # 11: no current Schema 5.0 claim (historical mentions allowed).
    schema_5 = []
    for number, line in enumerate(read("11_审核问题闭环与变更追踪.md").splitlines(), 1):
        if "5.0" not in line:
            continue
        if any(word in line for word in ("历史", "R2", "当时", "旧", "V1.9.0", "5.0/0x00010901")):
            continue
        schema_5.append({"line": number, "text": line.strip()[:160]})
    add("doc11_schema_6_no_current_5_claim", not schema_5, {"violations": schema_5})

    # 07: single pipeline limit, 3000us, no 1900us leftover.
    doc07 = read("07_实时调度与板级硬件资源设计.md")
    row = re.search(r"MAX_IMU_PIPELINE_DELAY_US\s*\|\s*(\d+)us", doc07)
    add("doc07_pipeline_single_limit_3000",
        row is not None and row.group(1) == "3000" and "1900" not in doc07
        and "pipeline≤3000us" in doc07,
        {"table_us": int(row.group(1)) if row else None,
         "has_1900": "1900" in doc07,
         "prose_3000": "pipeline≤3000us" in doc07,
         "sha256": sha(SPEC / "07_实时调度与板级硬件资源设计.md")})

    # D's review rerun artifact and 09 registration.
    rerun = SPEC / "validation" / "t191-d-review_report.json"
    add("d_review_rerun_report_exists", rerun.is_file(),
        {"path": "spec/validation/t191-d-review_report.json",
         "sha256": sha(rerun) if rerun.is_file() else None})
    doc09 = read("09_需求验证实施与发布管理.md")
    add("doc09_d_review_registered_in_progress",
        "TEST-T191-D-REVIEW" in doc09 and "IN_PROGRESS" in doc09,
        {"row_present": "TEST-T191-D-REVIEW" in doc09})

    # Arithmetic spot checks from D section 8.
    cpu = 450 * 1053 + 35 * 2053
    chain = 5 + 320 + 40 + 60 + 250 + 100 + 35 + 90
    window = 3000 + 900 + 100
    feedback = 5000 + 2500
    add("arithmetic_spot_checks",
        cpu == 545705 and abs(cpu / 1e6 * 100 - 54.5705) < 1e-9
        and chain == 900 and window == 4000 and feedback == 7500,
        {"cpu_raw": cpu, "cpu_percent": cpu / 1e6 * 100,
         "chain_us": chain, "window_us": window, "feedback_us": feedback})

    # Review snapshot consistency: D's JSON predates D's own fixes.
    checks_json = json.loads((SPEC / "validation" / "T191-D-REVIEW" / "review_checks.json")
                             .read_text(encoding="utf-8"))
    counts = {}
    for item in checks_json["results"]:
        counts[item["status"]] = counts.get(item["status"], 0) + 1
    abi02 = next(r for r in checks_json["results"] if r["id"] == "ABI-02")
    add("d_snapshot_counts_match_report", counts == {"PASS": 46, "OBS": 6, "DRIFT": 5, "FAIL": 2},
        {"counts": counts})
    obs.append({
        "id": "OBS-A-01",
        "severity": "P3",
        "title": "review_checks.json 是 D 修复前的快照，报告摘要仍按其计数",
        "evidence": {"ABI-02_status": abi02["status"],
                     "ABI-02_detail": abi02["detail"]["violations"],
                     "doc11_now": read("11_审核问题闭环与变更追踪.md").splitlines()[8][:120]},
        "impact": "ABI-02 在快照中仍为 FAIL，但 11:9 已改为 Schema 6.0；读者可能误判 D-REV-01 未修复。",
    })
    tool04 = next(r for r in checks_json["results"] if r["id"] == "TOOL-04")
    obs.append({
        "id": "OBS-A-02",
        "severity": "P3",
        "title": "TOOL-04 描述与 detail 矛盾",
        "evidence": {"description": tool04["description"],
                     "detail_pymavlink": tool04["detail"]["pymavlink"],
                     "status": tool04["status"]},
        "impact": "描述声称“pymavlink present”，实际为 false；与报告 §2“未安装”一致，但检查文本自相矛盾。",
    })

    audit = (SPEC / "tools" / "audit_spec.py").read_text(encoding="utf-8")
    add("audit_spec_has_label_and_optional_outside_baseline",
        "--label" in audit and "--outside-baseline" in audit
        and "--write-outside-baseline" in audit and "handoff_task_records_legal" in audit,
        {"label": "--label" in audit,
         "outside_optional": "--outside-baseline" in audit,
         "outside_writer": "--write-outside-baseline" in audit,
         "state_check": "handoff_task_records_legal" in audit})

    failed = [c["id"] for c in checks if c["status"] != "PASS"]
    result = {
        "reviewer": "A",
        "task_reviewed": "T191-D-REVIEW",
        "baseline": "V1.9.1-GLOBAL-AUDIT-R3",
        "scope": "DOCUMENT_STATIC / independent non-author review",
        "production_validation": "NOT_RUN",
        "reviewed_files": {
            "spec/validation/T191-D-REVIEW/T191-D-REVIEW_report.md":
                sha(SPEC / "validation" / "T191-D-REVIEW" / "T191-D-REVIEW_report.md"),
            "spec/validation/T191-D-REVIEW/review_checks.json":
                sha(SPEC / "validation" / "T191-D-REVIEW" / "review_checks.json"),
            "spec/AGENT_HANDOFF.md": sha(SPEC / "AGENT_HANDOFF.md"),
            "spec/07_实时调度与板级硬件资源设计.md":
                sha(SPEC / "07_实时调度与板级硬件资源设计.md"),
        },
        "checks": checks,
        "observations": obs,
        "failed_checks": failed,
        "verdict": "ACCEPT_WITH_CONDITIONS" if not failed else "CHANGES_REQUIRED",
        "conditions": [
            "D 需在修复后的树上重新生成 review_checks 快照（使用 --label），并更新报告摘要的计数/ABI-02 状态。",
            "07 已由 A 修正为 3000us；D 需重跑 BUD-08/BUD-12 并重建 round3/inputs_sha256 证据。",
            "A-ABI/B/C 的状态请求与工作区一致性（D-REV-04）由 D 合并；A 本轮补交 A-ABI。",
        ],
    }
    OUT.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"verdict": result["verdict"], "failed": failed,
                      "checks": len(checks), "observations": len(obs)}, ensure_ascii=False))
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
