#!/usr/bin/env python3
"""Add the R28 required documents to the archive staging directory, rebuild the
ZIP/SHA256SUMS and write the external archive report. Artifact-only: it does not
change Git history or source code."""
import argparse
import datetime
import hashlib
import json
import shutil
import subprocess
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-id", default="M5-06-R28-20261111")
    ap.add_argument("--archive-root", default="work/validation/archives")
    args = ap.parse_args()
    run_id = args.run_id
    root = ROOT / args.archive_root
    run_dir = root / run_id
    zip_path = root / f"{run_id}.zip"
    sidecar = root / f"{run_id}.zip.sha256"
    report_path = root / f"{run_id}.archive_report.json"
    bundle_rel = f"build/validation/regenerated/{run_id}.bundle"
    bundle = run_dir / bundle_rel
    if not run_dir.is_dir():
        raise SystemExit(f"missing staging dir {run_dir}")

    required = [
        "spec/history/implementation/guidance/M5_06_6_MR009_PROOF.md",
        "spec/quality/AGENT_SESSION_HANDOVER.md",
        "spec/quality/R28_ISSUE_DETAILS.md",
        "spec/quality/ISSUE_REGISTER.md",
        "spec/quality/BLOCKER_BOARD.md",
        "build/validation/R28_B06_STACK_ANALYSIS.md",
        "build/validation/analysis_p2.md",
        "build/validation/HANDOVER.md",
        "build/validation/regenerated/r28_test_results.json",
        "build/validation/regenerated/r28_environment.json",
        "build/validation/regenerated/r28_source_manifest.json",
        "build/validation/regenerated/r28_stack_analysis.json",
        "build/validation/handover/M5-06-R28-20261111/SESSION_HANDOVER.md",
        "build/validation/handover/M5-06-R28-20261111/handover_state.json",
        "build/validation/handover/M5-06-R28-20261111/TAKEOVER_ACK.md",
        "build/validation/handover/M5-06-R28-20261111/NEXT_SESSION_START_PROMPT.txt",
        bundle_rel,
    ]
    missing = []
    for rel in required:
        dst = run_dir / rel
        if rel.endswith("TAKEOVER_ACK.md"):
            src = ROOT / rel
        else:
            src = ROOT / rel
        if not src.is_file():
            missing.append(rel)
            continue
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
    if missing:
        raise SystemExit("missing required archive files: " + ", ".join(missing))

    head = "032d44a32c464a721dbc1ad2a4c62fe75fa3b1cd"
    branch = subprocess.check_output(["git", "-C", str(ROOT), "branch", "--show-current"], text=True).strip()
    remote = subprocess.check_output(
        ["git", "-C", str(ROOT), "rev-parse", f"origin/{branch}"], text=True).strip()
    ahead = int(subprocess.check_output(
        ["git", "-C", str(ROOT), "rev-list", "--count", f"origin/{branch}..{head}"], text=True).strip())
    code = "31463cb351d2f55e5da978c75db64e9d25fc2523"
    tooling = "5e1b8e5d6ea10d0620004f23567b2502c8aae1c4"

    evidence = []
    for path in sorted(run_dir.rglob("*")):
        if not path.is_file():
            continue
        rel = path.relative_to(run_dir).as_posix()
        if rel in ("SHA256SUMS.txt", "archive_manifest.json"):
            continue
        evidence.append({"path": rel, "size_bytes": path.stat().st_size,
                         "sha256": sha(path)})
    archive_manifest = {
        "schema_version": "2.0",
        "project": "Flight",
        "run_id": run_id,
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "repository_root": str(ROOT),
        "branch": branch,
        "head_sha": head,
        "archive_head_sha": head,
        "archive_commit_sha": head,
        "validated_code_commit": code,
        "tooling_commit": tooling,
        "collector_commit": tooling,
        "archive_script_commit": "bbf3e9205f2a764caaeb9e833c308c8069c6f7fb",
        "input_baseline_code_commit": "00baed0575e6db1212248315adf214853ecc62e3",
        "input_baseline_archive_head": "753c45994cb9cba12d60ce58acb04a9dcaa9b97a",
        "remote_head": remote,
        "ahead_commit_count": ahead,
        "tools": {
            "python": "3.14.8",
            "powershell": "7.6.9",
            "gcc_wsl": "16.2.0",
            "arm_none_eabi_gcc": "14.3.1",
        },
        "git_bundle": {
            "path": bundle_rel,
            "sha256": sha(bundle),
            "refs": [
                f"{head} HEAD",
                f"{head} refs/heads/{branch}",
            ],
        },
        "evidence_files": evidence,
        "scope_warning": "Archive contents are evidence only. PASS is not inferred from file presence. Host/SIL and Cortex-M7 compile are not H723 runtime, HIL or flight evidence.",
        "hardware_warning": "H723 stack watermark, WCET/RTA, HIL and flight are NOT_RUN.",
    }
    (run_dir / "archive_manifest.json").write_text(
        json.dumps(archive_manifest, indent=2) + "\n", encoding="utf-8")

    sums = []
    for path in sorted(run_dir.rglob("*")):
        if not path.is_file() or path.name == "SHA256SUMS.txt":
            continue
        rel = path.relative_to(run_dir).as_posix()
        sums.append(f"{sha(path)}  {rel}")
    (run_dir / "SHA256SUMS.txt").write_text("\n".join(sums) + "\n", encoding="utf-8")

    tmp_zip = root / f"{run_id}.zip.tmp"
    if tmp_zip.exists():
        tmp_zip.unlink()
    with zipfile.ZipFile(tmp_zip, "w", compression=zipfile.ZIP_DEFLATED,
                         compresslevel=9) as zf:
        for path in sorted(run_dir.rglob("*")):
            if path.is_file():
                zf.write(path, arcname=f"{run_id}/{path.relative_to(run_dir).as_posix()}")
    if zip_path.exists():
        zip_path.unlink()
    tmp_zip.replace(zip_path)
    zip_sha = sha(zip_path)
    zip_size = zip_path.stat().st_size
    sidecar.write_text(f"{zip_sha}  {zip_path.name}\n", encoding="ascii")

    report = {
        "schema_version": "2.0",
        "run_id": run_id,
        "branch": branch,
        "validated_code_commit": code,
        "tooling_commit": tooling,
        "collector_commit": tooling,
        "archive_script_commit": "bbf3e9205f2a764caaeb9e833c308c8069c6f7fb",
        "archive_head_sha": head,
        "archive_commit_sha": head,
        "input_baseline_code_commit": "00baed0575e6db1212248315adf214853ecc62e3",
        "input_baseline_archive_head": "753c45994cb9cba12d60ce58acb04a9dcaa9b97a",
        "remote_head": remote,
        "ahead_commit_count": ahead,
        "archive_zip_sha256": zip_sha,
        "archive_zip_size_bytes": zip_size,
        "archive_zip_path": str(zip_path),
        "sidecar_path": str(sidecar),
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "scope": "LOCAL_EXECUTION_ONLY; not GitHub CI, H723 runtime, HIL or flight",
        "archive_manifest": str(run_dir / "archive_manifest.json"),
        "bundle": str(bundle),
        "bundle_sha256": sha(bundle),
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"R28_ARCHIVE_FINALIZED zip={zip_path}")
    print(f"ZIP_SHA256={zip_sha}")
    print(f"ZIP_SIZE={zip_size}")
    print(f"REPORT={report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
