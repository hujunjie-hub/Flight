#!/usr/bin/env python3
"""Sync C's T191-D-REVIEW opinion (revision 2) into the submission index.

- writes applications/core/m4/reviews/C_REVIEW_SUBMISSION.json
- refreshes review_opinion in T191-C-FMS/C-M5 status requests (content only)
- recomputes the SHA256 list in applications/core/status_requests/index.json

No spec/HANDOFF file is touched.
"""
from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone, timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
NOW = datetime.now(timezone(timedelta(hours=8))).replace(microsecond=0).isoformat()
REV = ROOT / "applications/core/m4/reviews"


def sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


checks = json.loads((REV / "c_review_checks.json").read_text(encoding="utf-8"))
summary = checks["summary"]
md = REV / "C_REVIEW_T191-D-REVIEW.md"
py = REV / "c_review_checks.py"
js = REV / "c_review_checks.json"

submission = {
    "reviewer": "C",
    "task": "T191-D-REVIEW",
    "revision": 2,
    "submitted_at": NOW,
    "addressed_to": "D",
    "response_to": "T191-D-REVIEW update_sequence=4 next_action ('待C书面意见与用户Gate A批准')",
    "conclusion": "ACCEPT_WITH_CONDITIONS",
    "summary": summary,
    "paths": {
        "opinion": "applications/core/m4/reviews/C_REVIEW_T191-D-REVIEW.md",
        "machine_checks": "applications/core/m4/reviews/c_review_checks.json",
        "check_script": "applications/core/m4/reviews/c_review_checks.py",
    },
    "sha256": {"opinion": sha(md), "machine_checks": sha(js), "check_script": sha(py)},
    "status": "SUBMITTED_FOR_MERGE",
    "note": "C's non-author opinion. Not an ACCEPTED state, not a Gate A sign-off; B's opinion and user approval are still separate.",
}
sub_path = REV / "C_REVIEW_SUBMISSION.json"
write = json.dumps(submission, ensure_ascii=False, indent=2) + "\n"
sub_path.write_text(write, encoding="utf-8")
sub_sha = sha(sub_path)

review_opinion = {
    "task": "T191-D-REVIEW",
    "revision": 2,
    "path": "applications/core/m4/reviews/C_REVIEW_T191-D-REVIEW.md",
    "submission": "applications/core/m4/reviews/C_REVIEW_SUBMISSION.json",
    "sha256": submission["sha256"]["opinion"],
    "conclusion": "ACCEPT_WITH_CONDITIONS",
    "summary": summary,
    "status": "SUBMITTED_FOR_MERGE",
    "note": "Use revision 2 (post-fix review_checks_post.json). Not an ACCEPTED state or Gate A approval.",
}

for rel in ("applications/core/m4/status_request_T191-C-FMS.json",
            "applications/core/m5/status_request_T191-C-M5.json"):
    p = ROOT / rel
    d = json.loads(p.read_text(encoding="utf-8"))
    d["review_opinion"] = review_opinion
    p.write_text(json.dumps(d, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

index_path = ROOT / "applications/core/status_requests/index.json"
index = json.loads(index_path.read_text(encoding="utf-8"))
index["generated_at"] = NOW
index["review_opinion"] = review_opinion
for s in index["submissions"]:
    s["sha256"] = sha(ROOT / s["path"])
index["review_submission"] = {
    "path": "applications/core/m4/reviews/C_REVIEW_SUBMISSION.json",
    "sha256": sub_sha,
}
index_path.write_text(json.dumps(index, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

print(json.dumps({"review_submission": sub_sha, "summary": summary,
                  "index": sha(index_path)}, indent=2))
