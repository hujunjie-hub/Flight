#!/usr/bin/env python3
"""Agent C independent review checks for T191-D-REVIEW (revision 2).

Revision 2 uses D's post-fix rerun (`review_checks_post.json`, 23:28) instead of
the pre-fix snapshot, and records the current HANDOFF state (D-REVIEW seq4,
A-ABI ASSIGNED, B/C submissions on file).

Does not import or execute D's review_checks.py.
Output: c_review_checks.json (same directory).
"""
from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]  # .../Flight
D_DIR = ROOT / "spec" / "validation" / "T191-D-REVIEW"

results: list[dict] = []


def add(check_id: str, area: str, ok: bool | None, description: str, detail=None):
    status = "PASS" if ok is True else ("FAIL" if ok is False else "OBS")
    results.append({"id": check_id, "area": area, "status": status,
                    "description": description, "detail": detail or {}})


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8")


# 1. HANDOFF and submissions ------------------------------------------------
records = json.loads(re.search(
    r"<!-- TASK_RECORDS_START -->\s*```json\s*(.*?)\s*```",
    read("spec/AGENT_HANDOFF.md"), re.S).group(1))
by_id = {r["id"]: r for r in records}
ids = list(by_id)
graph = {r["id"]: r["depends_on"] for r in records}
seen, stack = set(), set()


def acyclic(node: str) -> bool:
    if node in stack:
        return False
    if node in seen:
        return True
    stack.add(node)
    ok = all(dep in graph and acyclic(dep) for dep in graph.get(node, []))
    stack.discard(node)
    seen.add(node)
    return ok


owner_ok = all(r["owner"] not in r["reviewer"] for r in records)
add("GOV-01", "GOVERNANCE",
    len(records) == 15 and len(set(ids)) == 15 and owner_ok and
    all(d in graph for r in records for d in r["depends_on"]) and
    all(acyclic(i) for i in ids),
    "HANDOFF 15 unique tasks, owner not reviewer, dependencies exist, DAG acyclic",
    {"tasks": len(records)})
d_rev = by_id["T191-D-REVIEW"]
add("GOV-02", "GOVERNANCE",
    d_rev["task_status"] == "REVIEW" and d_rev["update_sequence"] == 4 and
    bool(d_rev.get("acknowledgement")),
    "T191-D-REVIEW is REVIEW/seq4 with acknowledgement",
    {"status": d_rev["task_status"], "seq": d_rev["update_sequence"]})
add("GOV-03", "GOVERNANCE",
    by_id["T191-A-ABI"]["task_status"] == "ASSIGNED" and
    by_id["T191-A-ABI"]["update_sequence"] == 3,
    "T191-A-ABI merged as ASSIGNED/seq3 (D accepted A's resubmission)",
    {"status": by_id["T191-A-ABI"]["task_status"], "seq": by_id["T191-A-ABI"]["update_sequence"]})
b_sub = ["applications/plugins/estimator/kf_gins_quad/status_request_T191-B-UPSTREAM.json",
         "applications/core/m2/status_request_T191-B-ADAPTER.json",
         "applications/core/m3/status_request_T191-B-REPLAY.json",
         "applications/services/health/status_request_T191-B-FDIR.json"]
c_sub = ["applications/core/m4/status_request_T191-C-FMS.json",
         "applications/core/m5/status_request_T191-C-M5.json",
         "applications/core/m6/status_request_T191-C-M6.json",
         "applications/core/m7/status_request_T191-C-M7.json"]
add("GOV-04", "GOVERNANCE",
    all((ROOT / p).is_file() for p in b_sub + c_sub),
    "B's four and C's four status submissions exist on disk", {"B": len(b_sub), "C": len(c_sub)})

# 2. D evidence -------------------------------------------------------------
sums = {}
for line in read("spec/validation/T191-D-REVIEW/SHA256SUMS.txt").splitlines():
    parts = line.split()
    if len(parts) == 2:
        sums[parts[1]] = parts[0]
bad = [n for n, h in sums.items()
       if hashlib.sha256((D_DIR / n).read_bytes()).hexdigest() != h]
add("EVID-01", "EVIDENCE", not bad and len(sums) >= 5,
    "D SHA256SUMS match the current evidence files (post-fix manifest includes review_checks_post.json)", {"files": len(sums), "mismatch": bad})

post = json.loads((D_DIR / "review_checks_post.json").read_text(encoding="utf-8"))
pres = json.loads((D_DIR / "review_checks.json").read_text(encoding="utf-8"))


def counts(doc):
    out = {}
    for r in doc["results"]:
        out[r["status"]] = out.get(r["status"], 0) + 1
    return out


pc = counts(post)
failed_ids = [r["id"] for r in post["results"] if r["status"] == "FAIL"]
add("EVID-02", "EVIDENCE",
    len(post["results"]) == 59 and pc.get("PASS") == 46 and pc.get("OBS") == 6 and
    pc.get("DRIFT") == 5 and pc.get("FAIL") == 2 and failed_ids == ["EVID-01", "EVID-02"],
    "post-fix rerun: 59 checks, 46/2/6/5; the only FAILs are the expected historical-input drifts",
    {"counts": pc, "failed": failed_ids})
add("EVID-03", "EVIDENCE",
    "59 项自动检查" in read("spec/validation/T191-D-REVIEW/T191-D-REVIEW_report.md"),
    "the 22:58 report summary matches the pre-fix snapshot it cites")

# 3. D findings disposal ----------------------------------------------------
l11 = read("spec/11_审核问题闭环与变更追踪.md").splitlines()
add("REV-01", "DISPOSAL", "Schema升为6.0" in l11[8] and "Schema升为5.0" not in l11[8],
    "D-REV-01 fixed: 11:9 says Schema 6.0")
l07 = read("spec/07_实时调度与板级硬件资源设计.md").splitlines()
value_line = next(s for s in l07 if "MAX_IMU_PIPELINE_DELAY_US" in s)
add("REV-02", "DISPOSAL", "3000us" in value_line and "BUD-12" not in failed_ids,
    "D-REV-02: 07 table is 3000us and the post-fix rerun no longer fails BUD-12",
    {"table": value_line.strip()})
tool = read("spec/tools/audit_spec.py")
add("REV-03", "DISPOSAL",
    all(k in tool for k in ("--label", "--outside-baseline", "--write-outside-baseline")),
    "D-REV-03 tool fixes exist")

# 4. C contract findings ----------------------------------------------------
prod = read("applications/products/QUAD_X4_H723/product_contract.h")
msg = read("applications/contracts/fc_common_messages.h")
ports = read("applications/contracts/fc_port_types.h")


def block(text: str, end_marker: str) -> str:
    end = text.index(end_marker)
    return text[text.rindex("typedef struct {", 0, end):end]


op_payload = block(prod, "} ProductOperationPayload;")
op_request = block(msg, "} OperationRequest;")
d06 = read("spec/06_FMS通信与系统安全设计.md")
add("C-REV-01", "CONTRACT",
    "mission_epoch" not in op_payload and "mission_epoch" not in op_request,
    "06 §12 requires the current mission_epoch for mission commands, but OperationRequest/ProductOperationPayload carry no such field",
    {"spec_requires": "当前mission_epoch" in d06, "d_report_mentions": "mission_epoch" in
     read("spec/validation/T191-D-REVIEW/T191-D-REVIEW_report.md")})
add("C-REV-02", "CONTRACT", None,
    "M4 owns the geofence per 06:433, but 02 defines no geofence command/field and FmsInputView has no fence input",
    {"fms_input_has_geofence": "geofence" in block(ports, "} FmsInputView;").lower(),
     "command_has_geofence": "SET_GEOFENCE" in prod,
     "d_report_mentions": "geofence" in read("spec/validation/T191-D-REVIEW/T191-D-REVIEW_report.md").lower()})
add("C-REV-03", "CONTRACT", None,
    "02:2559 requires real output_policy_id>=0x100 registration but 08/09 define no registry",
    {"mentions_0x100": "0x100" in read("spec/02_接口契约与公共数据规范.md")})

# 5. Arithmetic -------------------------------------------------------------
add("BUD-01", "BUDGET", abs((450 * 1053 + 35 * 2053) / 1e6 * 100 - 54.5705) < 1e-4,
    "CPU 54.5705%")
add("BUD-02", "BUDGET", 5 + 320 + 40 + 60 + 250 + 100 + 35 + 90 == 900, "900us chain")
dshot = {"300k": (round(275e6 / 300e3), round(3 * 917 / 8), round(3 * 917 / 4), 916),
         "600k": (round(275e6 / 600e3), round(3 * 458 / 8), round(3 * 458 / 4), 457)}
add("BUD-03", "BUDGET",
    dshot["300k"] == (917, 344, 688, 916) and dshot["600k"] == (458, 172, 344, 457),
    "DShot bit timing/ARR recompute", dshot)
add("BUD-04", "BUDGET", 62016 - 61952 == 64, "M5 62,016B vs item sum 61,952B (64B capacity note)")

# 6. Records / observations -------------------------------------------------
l09 = read("spec/09_需求验证实施与发布管理.md")
add("REC-01", "RECORDS", "TEST-T191-D-REVIEW" in l09 and "IN_PROGRESS" in l09,
    "09 has TEST-T191-D-REVIEW = IN_PROGRESS")
add("REC-02", "RECORDS", None,
    "09 still has no T191-C-FMS/T191-C-M5 implementation rows; D registers C evidence on acceptance",
    {"c_fms": l09.count("T191-C-FMS"), "c_m5": l09.count("T191-C-M5")})
report = read("spec/validation/T191-D-REVIEW/T191-D-REVIEW_report.md")
add("OBS-01", "SNAPSHOT", None,
    "the 22:58 report markdown still cites the pre-fix snapshot and §17 still says 07:419=1900us/D-REV-02 open; only the post-fix JSON was rerun",
    {"stale_rev02": "1900µs" in report and "未处置" in report,
     "post_run": (D_DIR / "review_checks_post.json").is_file()})
add("OBS-02", "ABI-COUNT", None,
    "D counts 143 spec types while A's production ABI counts 142; the counting basis needs a note",
    {"d_143": "143 个规范类型" in report,
     "a_142": "142" in read("applications/tests/abi/evidence/abi_report.md")})
kconf = read("applications/core/Kconfig")
root_k = read("Kconfig")
add("OBS-03", "BUILD", None,
    "B-R05 also applies to C: FMS_QUAD_X4_ENABLE is defined but applications/core/Kconfig is not sourced by the root/applications Kconfig chain, so menuconfig cannot reach it",
    {"core_kconfig_exists": "FMS_QUAD_X4_ENABLE" in kconf,
     "root_sources_core": "applications/core/Kconfig" in root_k})

# A confirmed ICD defect is a FAIL in this review, not a passing fact.
for r in results:
    if r["id"] == "C-REV-01" and r["detail"].get("spec_requires") and not r["detail"].get("d_report_mentions"):
        r["status"] = "FAIL"

summary = {"checks": len(results),
           "pass": sum(1 for r in results if r["status"] == "PASS"),
           "fail": sum(1 for r in results if r["status"] == "FAIL"),
           "obs": sum(1 for r in results if r["status"] == "OBS")}
out = {
    "reviewer": "C",
    "revision": 2,
    "task": "T191-D-REVIEW",
    "baseline": "V1.9.1-GLOBAL-AUDIT-R3 / Schema 6.0 / Product-Port 0x00010903",
    "scope": "DOCUMENT_STATIC + C-owned contract/implementation cross-check; uses D's post-fix rerun",
    "handoff_seen": {"T191-D-REVIEW": f'{d_rev["task_status"]}/seq{d_rev["update_sequence"]}',
                     "T191-A-ABI": f'{by_id["T191-A-ABI"]["task_status"]}/seq{by_id["T191-A-ABI"]["update_sequence"]}'},
    "d_post_fix_evidence": "spec/validation/T191-D-REVIEW/review_checks_post.json",
    "summary": summary,
    "results": results,
}
(HERE / "c_review_checks.json").write_text(json.dumps(out, ensure_ascii=False, indent=2) + "\n",
                                           encoding="utf-8")
print(json.dumps(summary, ensure_ascii=False))
for r in results:
    print(f'{r["status"]:4} {r["id"]:10} {r["description"][:88]}')
