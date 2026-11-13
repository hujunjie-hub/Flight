#!/usr/bin/env python3
"""Prepare non-self-referential metadata before a validation ZIP is built.

The committed/source manifest records Git identity and the archive policy.
The ZIP's own SHA256 and byte size are intentionally external-only and are
written after ZIP creation to the external archive report and sidecar.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import tempfile
from pathlib import Path
from typing import Any

SHA40 = re.compile(r"^[0-9a-f]{40}$")
EXTERNAL_HASH_POLICY = "EXTERNAL_ARCHIVE_REPORT_AND_SIDECAR"
TOOLING_PATHS = (
    ("SConstruct", "firmware_build_entrypoint"),
    ("tools/check_rtt_package_tree.py", "rtt_package_tree_guard"),
    ("tools/collect_validation_results.py", "collector"),
    ("tools/check_validation_evidence.py", "evidence_checker"),
    ("tools/analyze_stack.py", "stack_analyzer"),
    ("tools/prepare_archive_metadata.py", "archive_metadata_preparer"),
    ("tools/archive_validation_run.ps1", "archive_entrypoint"),
)


def git(root: Path, *args: str) -> str:
    proc = subprocess.run(
        ["git", "-C", str(root), *args],
        check=True, capture_output=True, text=True,
    )
    return proc.stdout.strip()


def latest_commit_for_paths(root: Path, *paths: str) -> str:
    value = git(root, "log", "-1", "--format=%H", "HEAD", "--", *paths)
    if not SHA40.fullmatch(value):
        raise ValueError(f"no committed history for tooling paths: {paths}")
    return value


def source_identity(root: Path, relative: str, role: str) -> dict[str, Any]:
    path = root / relative
    if not path.is_file():
        raise ValueError(f"tooling source missing: {relative}")
    raw = path.read_bytes()
    blob = git(root, "rev-parse", f"HEAD:{relative}")
    if not SHA40.fullmatch(blob):
        raise ValueError(f"tooling source has no committed Git blob: {relative}")
    has_crlf = b"\r\n" in raw
    lf_only = b"\n" in raw.replace(b"\r\n", b"")
    if has_crlf and lf_only:
        ending = "MIXED"
    elif has_crlf:
        ending = "CRLF"
    elif b"\n" in raw:
        ending = "LF"
    else:
        ending = "NONE"
    return {
        "path": relative,
        "role": role,
        "git_blob_sha": blob,
        "working_tree_sha256": hashlib.sha256(raw).hexdigest(),
        "size_bytes": len(raw),
        "line_ending": ending,
    }


def build_latest_local_validation(
    existing: dict[str, Any], *, run_id: str, branch: str,
    validated_code_commit: str, archive_head_sha: str,
    remote_head: str, ahead_commit_count: int,
    tooling_commit: str, collector_commit: str,
    archive_script_commit: str,
    tooling_sources: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Return a complete pre-ZIP manifest record without ZIP self-hash fields."""
    for label, value in (
        ("validated_code_commit", validated_code_commit),
        ("archive_head_sha", archive_head_sha),
        ("remote_head", remote_head),
        ("tooling_commit", tooling_commit),
        ("collector_commit", collector_commit),
        ("archive_script_commit", archive_script_commit),
    ):
        if not isinstance(value, str) or not SHA40.fullmatch(value):
            raise ValueError(f"{label} must be a full 40-character Git SHA")
    if not isinstance(ahead_commit_count, int) or ahead_commit_count < 0:
        raise ValueError("ahead_commit_count must be a non-negative integer")
    if not run_id or not re.fullmatch(r"[A-Za-z0-9._-]{6,64}", run_id):
        raise ValueError("invalid run_id")
    if not branch or branch == "HEAD":
        raise ValueError("a named branch is required")

    result = dict(existing)
    result.update({
        "run_id": run_id,
        "branch": branch,
        "validated_code_commit": validated_code_commit,
        "tooling_commit": tooling_commit,
        "collector_commit": collector_commit,
        "archive_script_commit": archive_script_commit,
        "archive_head_sha": archive_head_sha,
        "archive_commit_sha": archive_head_sha,
        "remote_head": remote_head,
        "ahead_commit_count": ahead_commit_count,
        # A ZIP cannot contain its own final digest/size. Null is intentional;
        # the external report and .sha256 sidecar are authoritative for these.
        "archive_zip_sha256": None,
        "archive_zip_size_bytes": None,
        "archive_zip_hash_location": EXTERNAL_HASH_POLICY,
        "archive_external_report": f"work/validation/archives/{run_id}.archive_report.json",
        "archive_zip_sidecar": f"work/validation/archives/{run_id}.zip.sha256",
        "archive_metadata_policy": "PREZIP_GIT_METADATA_PLUS_EXTERNAL_POSTZIP_DIGEST_V1",
        "tooling_sources": tooling_sources or [],
        "tooling_source_manifest_sha256": hashlib.sha256(json.dumps(
            tooling_sources or [], sort_keys=True, separators=(",", ":")
        ).encode("utf-8")).hexdigest(),
    })
    by_path = {item["path"]: item for item in (tooling_sources or [])}
    result["collector_worktree_sha256"] = by_path.get(
        "tools/collect_validation_results.py", {}).get("working_tree_sha256")
    result["archive_script_worktree_sha256"] = by_path.get(
        "tools/archive_validation_run.ps1", {}).get("working_tree_sha256")
    return result


def prepare(repo: Path, manifest_path: Path, run_id: str,
            expected_branch: str | None = None,
            validated_code_override: str | None = None) -> dict[str, Any]:
    repo = repo.resolve()
    manifest_path = manifest_path.resolve()
    branch = git(repo, "branch", "--show-current")
    if expected_branch and branch != expected_branch:
        raise ValueError(f"branch mismatch: expected {expected_branch}, got {branch}")
    head = git(repo, "rev-parse", "HEAD")
    remote_ref = f"origin/{branch}"
    remote_head = git(repo, "rev-parse", remote_ref)
    ahead = int(git(repo, "rev-list", "--count", f"{remote_ref}..HEAD"))

    data = json.loads(manifest_path.read_text(encoding="utf-8"))
    current = data.get("latest_local_validation")
    if not isinstance(current, dict):
        raise ValueError("manifest.latest_local_validation must be an object")
    validated_code = current.get("validated_code_commit")
    if not isinstance(validated_code, str) or not SHA40.fullmatch(validated_code):
        raise ValueError("manifest has no valid validated_code_commit")
    # The record is inherited from whichever commit is checked out. When the
    # archive is built from the validated-code commit (not the metadata commit),
    # the inherited value still names the previous round's code. An explicit
    # baseline lets the caller bind the commit that was actually tested; the
    # commit must exist locally.
    if validated_code_override:
        if not SHA40.fullmatch(validated_code_override):
            raise ValueError("validated_code override must be a full 40-character Git SHA")
        git(repo, "cat-file", "-e", f"{validated_code_override}^{{commit}}")
        validated_code = validated_code_override
    git(repo, "cat-file", "-e", f"{validated_code}^{{commit}}")

    collector_commit = latest_commit_for_paths(repo, "tools/collect_validation_results.py")
    archive_script_commit = latest_commit_for_paths(repo, "tools/finalize_validation_archive.py",
                                                     "tools/archive_validation_run.ps1")
    tooling_commit = latest_commit_for_paths(
        repo, "SConstruct", "tools/check_rtt_package_tree.py",
        "tools/collect_validation_results.py", "tools/check_validation_evidence.py",
        "tools/analyze_stack.py", "tools/prepare_archive_metadata.py")
    tooling_sources = [source_identity(repo, path, role)
                       for path, role in TOOLING_PATHS]

    data["latest_local_validation"] = build_latest_local_validation(
        current, run_id=run_id, branch=branch,
        validated_code_commit=validated_code,
        archive_head_sha=head, remote_head=remote_head,
        ahead_commit_count=ahead, tooling_commit=tooling_commit,
        collector_commit=collector_commit,
        archive_script_commit=archive_script_commit,
        tooling_sources=tooling_sources)

    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=manifest_path.name + ".", suffix=".tmp",
                                    dir=str(manifest_path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(data, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp_name, manifest_path)
    except Exception:
        try:
            os.unlink(tmp_name)
        except FileNotFoundError:
            pass
        raise
    return data["latest_local_validation"]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default=".")
    ap.add_argument("--manifest", default="spec/validation/flight_acceptance_manifest.json")
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--expected-branch", default="opt/stage-02-m5-06-safety-validator")
    ap.add_argument("--validated-code", default="",
                    help="bind this 40-hex commit as validated_code_commit instead of "
                         "inheriting the previous round's value")
    args = ap.parse_args()
    repo = Path(args.repo).resolve()
    manifest = Path(args.manifest)
    if not manifest.is_absolute():
        manifest = repo / manifest
    result = prepare(repo, manifest, args.run_id, args.expected_branch,
                     args.validated_code or None)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print("ARCHIVE_METADATA_PREPARED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
