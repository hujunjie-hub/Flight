#!/usr/bin/env python3
"""Unified R28 evidence checker.

This replaces the R25/R27 split checkers. It compares the manifest, environment,
test results, external archive report, issue register, blocker board, product
config, stack analysis and handover against the actual Git tree and raw logs.
It never treats field presence alone as PASS and never turns UNKNOWN into PASS.
"""
import argparse
import hashlib
import json
import re
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = {
    "manifest": "spec/validation/flight_acceptance_manifest.json",
    "environment": "build/validation/regenerated/r28_environment.json",
    "test_results": "build/validation/regenerated/r28_test_results.json",
    "archive_report": "work/validation/archives/M5-06-R28-20261111.archive_report.json",
    "issue_register": "spec/quality/ISSUE_REGISTER.md",
    "issue_details": "spec/quality/R28_ISSUE_DETAILS.md",
    "blocker_board": "spec/quality/BLOCKER_BOARD.md",
    "handover": "build/validation/handover/M5-06-R28-20261111/SESSION_HANDOVER.md",
    "handover_dir": "build/validation/handover/M5-06-R28-20261111",
    "stack": "build/validation/regenerated/r28_stack_analysis.json",
    "source_manifest": "build/validation/regenerated/r28_source_manifest.json",
}


def sha(path):
    p = Path(path)
    return hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None


def read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except Exception as exc:
        raise RuntimeError(f"invalid JSON {path}: {exc}")


def read_text(path):
    p = Path(path)
    if not p.is_file():
        raise RuntimeError(f"missing file {path}")
    return p.read_text(encoding="utf-8", errors="replace")


def git(*args):
    proc = subprocess.run(["git", "-C", str(ROOT), *args],
                          capture_output=True, text=True)
    return proc.stdout.strip() if proc.returncode == 0 else None


def emit(out_dir, name, checks, failures, status, detail):
    out_dir.mkdir(parents=True, exist_ok=True)
    data = {
        "check": name,
        "run_id": detail.get("run_id") if isinstance(detail, dict) else None,
        "checks": checks,
        "failures": failures,
        "status": status,
        "exit_code": 0 if status == "PASS" else (2 if status == "BLOCKED" else (3 if status == "UNKNOWN" else 1)),
        "detail": detail,
    }
    text = json.dumps(data, indent=2)
    (out_dir / f"{name}.json").write_text(text + "\n", encoding="utf-8")
    (out_dir / f"{name}.log").write_text(text + "\n", encoding="utf-8")
    print(f"{name}: {status} checks={checks} failures={failures}")
    return data


def valid_version(value):
    if not isinstance(value, str) or not value.strip():
        return False
    low = value.lower()
    for token in ("see r25_environment", "see log", "unknown", "deterministic",
                  "not_recorded", "not recorded"):
        if token in low:
            return False
    return bool(re.search(r"\d+\.\d+", value))


def _valid_git_sha(value):
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{40}", value) is not None


def _git_path_commit(*paths):
    value = git("log", "-1", "--format=%H", "HEAD", "--", *paths)
    return value if _valid_git_sha(value) else None


def _resolve_external_path(raw, report_path, fallback_name=None):
    """Resolve native, relative, or Windows-authored archive paths."""
    candidates = []
    if isinstance(raw, str) and raw:
        candidates.append(Path(raw))
        # A Windows drive path is not a native POSIX path; retain only its leaf.
        leaf = re.split(r"[\\/]", raw)[-1]
        if leaf:
            candidates.append(ROOT / "work/validation/archives" / leaf)
            candidates.append(Path(report_path).parent / leaf)
    if fallback_name:
        candidates.append(ROOT / "work/validation/archives" / fallback_name)
        candidates.append(Path(report_path).parent / fallback_name)
    for candidate in candidates:
        try:
            if candidate.is_file():
                return candidate
        except OSError:
            continue
    return candidates[0] if candidates else Path("")


def check_manifest(ctx):
    m = ctx["manifest"]
    lv = m.get("latest_local_validation", {})
    checks = 0
    fails = []
    required = ("run_id", "branch", "validated_code_commit", "tooling_commit",
                "collector_commit", "archive_script_commit", "archive_head_sha",
                "archive_commit_sha", "remote_head", "ahead_commit_count",
                "archive_zip_hash_location", "archive_external_report",
                "archive_zip_sidecar", "archive_metadata_policy",
                "tooling_sources", "tooling_source_manifest_sha256",
                "collector_worktree_sha256", "archive_script_worktree_sha256")
    for key in required:
        checks += 1
        if lv.get(key) in (None, "", "EXTERNAL_ARCHIVE_REPORT"):
            fails.append(f"latest_local_validation.{key} missing")
    checks += 1
    if lv.get("run_id") != ctx["run_id"]:
        fails.append("manifest RUN_ID mismatch")
    checks += 1
    if lv.get("branch") != ctx["branch"]:
        fails.append("manifest branch mismatch")
    checks += 1
    if lv.get("validated_code_commit") != ctx["code_commit"]:
        fails.append("manifest validated_code_commit mismatch")
    for key in ("validated_code_commit", "tooling_commit", "collector_commit",
                "archive_script_commit", "archive_head_sha", "archive_commit_sha",
                "remote_head"):
        checks += 1
        if not _valid_git_sha(lv.get(key)):
            fails.append(f"manifest {key} is not a full Git SHA")
        elif subprocess.run(["git", "-C", str(ROOT), "cat-file", "-e",
                             f"{lv[key]}^{{commit}}"], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL).returncode != 0:
            fails.append(f"manifest {key} commit is not present in repository")
    checks += 1
    if lv.get("archive_head_sha") != ctx["head"]:
        fails.append("manifest archive_head_sha != current HEAD; post-archive metadata commits are forbidden")
    checks += 1
    if lv.get("archive_commit_sha") != lv.get("archive_head_sha"):
        fails.append("manifest archive_commit_sha != archive_head_sha")
    checks += 1
    if lv.get("remote_head") != ctx["remote_head"]:
        fails.append("manifest remote_head mismatch")
    checks += 1
    if lv.get("ahead_commit_count") != ctx["ahead"]:
        fails.append(f"manifest ahead {lv.get('ahead_commit_count')} != actual {ctx['ahead']}")
    checks += 1
    if lv.get("archive_zip_hash_location") != "EXTERNAL_ARCHIVE_REPORT_AND_SIDECAR":
        fails.append("manifest ZIP digest policy is not external-only")
    checks += 1
    if lv.get("archive_zip_sha256") is not None or lv.get("archive_zip_size_bytes") is not None:
        fails.append("manifest must not embed self-referential ZIP hash/size")
    checks += 1
    if "PENDING" in json.dumps(lv).upper():
        fails.append("manifest contains PENDING archive metadata")
    # Tool commits are checked against actual path history, not each other.
    expected = {
        "collector_commit": _git_path_commit("tools/collect_validation_results.py"),
        "archive_script_commit": _git_path_commit("tools/finalize_validation_archive.py",
                                                   "tools/archive_validation_run.ps1"),
        "tooling_commit": _git_path_commit("SConstruct", "tools/check_rtt_package_tree.py",
                                            "tools/collect_validation_results.py",
                                            "tools/check_validation_evidence.py",
                                            "tools/analyze_stack.py",
                                            "tools/prepare_archive_metadata.py"),
    }
    for key, value in expected.items():
        checks += 1
        if not value or lv.get(key) != value:
            fails.append(f"manifest {key} does not match Git path history")

    tooling_sources = lv.get("tooling_sources")
    expected_paths = {
        "SConstruct",
        "tools/check_rtt_package_tree.py",
        "tools/collect_validation_results.py",
        "tools/check_validation_evidence.py",
        "tools/analyze_stack.py",
        "tools/prepare_archive_metadata.py",
        "tools/archive_validation_run.ps1",
    }
    checks += 1
    if (not isinstance(tooling_sources, list) or len(tooling_sources) != len(expected_paths)
            or {x.get("path") for x in tooling_sources} != expected_paths):
        fails.append("manifest tooling_sources is incomplete, duplicated or contains unexpected paths")
    else:
        canonical = json.dumps(tooling_sources, sort_keys=True, separators=(",", ":")).encode("utf-8")
        checks += 1
        if hashlib.sha256(canonical).hexdigest() != lv.get("tooling_source_manifest_sha256"):
            fails.append("tooling_source_manifest_sha256 mismatch")
        for item in tooling_sources:
            path = item.get("path", "")
            raw_path = ROOT / path
            checks += 1
            if not raw_path.is_file():
                fails.append(f"tooling source missing: {path}")
                continue
            raw = raw_path.read_bytes()
            checks += 1
            if hashlib.sha256(raw).hexdigest() != item.get("working_tree_sha256"):
                fails.append(f"tooling working-tree SHA mismatch: {path}")
            checks += 1
            if len(raw) != item.get("size_bytes"):
                fails.append(f"tooling source size mismatch: {path}")
            has_crlf = b"\r\n" in raw
            has_lf_only = b"\n" in raw.replace(b"\r\n", b"")
            actual_eol = ("MIXED" if has_crlf and has_lf_only else
                          "CRLF" if has_crlf else "LF" if b"\n" in raw else "NONE")
            checks += 1
            if actual_eol != item.get("line_ending"):
                fails.append(f"tooling line-ending mismatch: {path}")
            blob = git("rev-parse", f"HEAD:{path}")
            checks += 1
            if blob != item.get("git_blob_sha"):
                fails.append(f"tooling Git blob SHA mismatch: {path}")
        by_path = {x.get("path"): x for x in tooling_sources}
        checks += 1
        if lv.get("collector_worktree_sha256") != by_path.get(
                "tools/collect_validation_results.py", {}).get("working_tree_sha256"):
            fails.append("collector_worktree_sha256 mismatch")
        checks += 1
        if lv.get("archive_script_worktree_sha256") != by_path.get(
                "tools/archive_validation_run.ps1", {}).get("working_tree_sha256"):
            fails.append("archive_script_worktree_sha256 mismatch")
    return emit(ctx["out"], "manifest", checks, len(fails),
                "PASS" if not fails else "FAIL",
                {"run_id": ctx["run_id"], "failures": fails,
                 "manifest": str(ctx["paths"]["manifest"])})


def check_environment(ctx):
    env = ctx["environment"]
    checks = 0
    fails = []
    for key in ("run_id", "code_commit", "toolchain_version"):
        checks += 1
        if not env.get(key):
            fails.append(f"environment missing {key}")
    checks += 1
    if env.get("run_id") != ctx["run_id"]:
        fails.append("environment RUN_ID mismatch")
    checks += 1
    if env.get("code_commit") != ctx["code_commit"]:
        fails.append("environment code_commit mismatch")
    checks += 1
    if not valid_version(env.get("toolchain_version")):
        fails.append("environment toolchain invalid")
    for key in ("windows", "wsl", "cortex_m7"):
        checks += 1
        item = env.get(key)
        if not isinstance(item, dict) or not item:
            fails.append(f"environment missing {key}")
            continue
        text = json.dumps(item)
        if not any(ch.isdigit() for ch in text) or "NOT_RECORDED" in text:
            fails.append(f"environment {key} lacks real versions")
    return emit(ctx["out"], "environment", checks, len(fails),
                "PASS" if not fails else "FAIL",
                {"run_id": ctx["run_id"], "failures": fails})


def check_archive(ctx):
    report = ctx["archive_report"]
    m = ctx["manifest"].get("latest_local_validation", {})
    checks = 0
    fails = []
    required = ("run_id", "branch", "validated_code_commit", "tooling_commit",
                "collector_commit", "archive_script_commit", "archive_head_sha",
                "archive_commit_sha", "remote_head", "ahead_commit_count",
                "archive_zip_sha256", "archive_zip_size_bytes", "archive_zip_hash_location",
                "archive_zip_path", "sidecar_path", "bundle", "bundle_sha256",
                "bundle_refs", "tooling_source_manifest_sha256",
                "collector_worktree_sha256", "archive_script_worktree_sha256", "scope")
    for key in required:
        checks += 1
        if report.get(key) in (None, ""):
            fails.append(f"archive report missing {key}")
    for key in ("run_id", "branch", "validated_code_commit", "tooling_commit",
                "collector_commit", "archive_script_commit", "archive_head_sha",
                "archive_commit_sha", "remote_head", "ahead_commit_count",
                "archive_zip_hash_location", "tooling_source_manifest_sha256",
                "collector_worktree_sha256", "archive_script_worktree_sha256"):
        checks += 1
        if report.get(key) != m.get(key):
            fails.append(f"archive report {key} != manifest")
    checks += 1
    if report.get("run_id") != ctx["run_id"]:
        fails.append("archive report RUN_ID mismatch")
    checks += 1
    if report.get("archive_head_sha") != ctx["head"]:
        fails.append("archive report archive_head_sha != current HEAD")
    checks += 1
    if report.get("remote_head") != ctx["remote_head"]:
        fails.append("archive report remote_head != actual remote ref")
    checks += 1
    if report.get("ahead_commit_count") != ctx["ahead"]:
        fails.append("archive report ahead_commit_count != actual ahead")

    expected_name = f"{ctx['run_id']}.zip"
    zip_path = _resolve_external_path(report.get("archive_zip_path"),
                                      ctx["paths"]["archive_report"], expected_name)
    checks += 1
    if not zip_path.is_file():
        fails.append(f"archive ZIP missing: {zip_path}")
        return emit(ctx["out"], "archive", checks, len(fails), "FAIL",
                    {"run_id": ctx["run_id"], "failures": fails})

    actual_zip_sha = sha(zip_path)
    actual_zip_size = zip_path.stat().st_size
    checks += 1
    if actual_zip_sha != report.get("archive_zip_sha256"):
        fails.append("archive ZIP SHA mismatch")
    checks += 1
    if actual_zip_size != report.get("archive_zip_size_bytes"):
        fails.append("archive ZIP size mismatch")

    sidecar = _resolve_external_path(report.get("sidecar_path"),
                                     ctx["paths"]["archive_report"],
                                     f"{ctx['run_id']}.zip.sha256")
    checks += 1
    if not sidecar.is_file():
        fails.append("archive ZIP sidecar missing")
    else:
        sidecar_text = sidecar.read_text(encoding="ascii", errors="replace").strip()
        expected_line = f"{actual_zip_sha}  {zip_path.name}"
        if sidecar_text != expected_line:
            fails.append("archive ZIP sidecar content mismatch")

    try:
        with zipfile.ZipFile(zip_path) as zf:
            checks += 1
            if zf.testzip() is not None:
                fails.append("archive ZIP CRC failure")
            names = zf.namelist()
            required_suffixes = ("archive_manifest.json", "SHA256SUMS.txt",
                                 f"{ctx['run_id']}.bundle")
            for required_name in required_suffixes:
                checks += 1
                if not any(name.endswith(required_name) for name in names):
                    fails.append(f"archive ZIP missing {required_name}")
            manifest_names = [name for name in names if name.endswith("archive_manifest.json")]
            if len(manifest_names) != 1:
                fails.append("archive ZIP must contain exactly one archive_manifest.json")
            else:
                archive_manifest = json.loads(zf.read(manifest_names[0]).decode("utf-8"))
                for key in ("run_id", "branch", "validated_code_commit", "tooling_commit",
                            "collector_commit", "archive_script_commit", "archive_head_sha",
                            "archive_commit_sha", "remote_head", "ahead_commit_count",
                            "tooling_source_manifest_sha256", "collector_worktree_sha256",
                            "archive_script_worktree_sha256"):
                    checks += 1
                    if archive_manifest.get(key) != report.get(key):
                        fails.append(f"ZIP archive_manifest {key} != external report")
                if archive_manifest.get("archive_zip_hash_location") != "EXTERNAL_ARCHIVE_REPORT_AND_SIDECAR":
                    fails.append("ZIP archive_manifest digest policy is not external-only")
                checks += 1
                if report.get("tooling_sources") != archive_manifest.get("tooling_sources"):
                    fails.append("external report tooling_sources != ZIP archive_manifest")
                if "archive_zip_sha256" in archive_manifest or "archive_zip_size_bytes" in archive_manifest:
                    fails.append("ZIP archive_manifest contains self-referential ZIP hash/size")
                # Verify all evidence entries in the archive manifest against ZIP bytes.
                # Verify the self-contained Git Bundle bytes, refs and structure.
                bundle_meta = archive_manifest.get("git_bundle", {})
                bundle_rel = bundle_meta.get("path")
                checks += 1
                if not isinstance(bundle_rel, str) or not bundle_rel:
                    fails.append("ZIP archive_manifest git_bundle.path missing")
                else:
                    checks += 1
                    if report.get("bundle") != bundle_rel:
                        fails.append("external report bundle path != ZIP archive_manifest")
                    bundle_member = f"{ctx['run_id']}/{bundle_rel}"
                    try:
                        bundle_bytes = zf.read(bundle_member)
                        bundle_sha = hashlib.sha256(bundle_bytes).hexdigest()
                        checks += 1
                        if bundle_sha != bundle_meta.get("sha256") or bundle_sha != report.get("bundle_sha256"):
                            fails.append("Git Bundle SHA mismatch")
                        checks += 1
                        if report.get("bundle_refs") != bundle_meta.get("refs"):
                            fails.append("Git Bundle refs differ between archive manifest and report")
                        with tempfile.TemporaryDirectory(prefix="flight_bundle_check_") as td:
                            bundle_file = Path(td) / f"{ctx['run_id']}.bundle"
                            bundle_file.write_bytes(bundle_bytes)
                            bare_repo = Path(td) / "verify.git"
                            subprocess.run(["git", "init", "--bare", str(bare_repo)],
                                           check=True, capture_output=True, text=True)
                            verify = subprocess.run(["git", "-C", str(bare_repo), "bundle",
                                                     "verify", str(bundle_file)],
                                                    capture_output=True, text=True)
                            checks += 1
                            if verify.returncode != 0:
                                fails.append("Git Bundle verify failed: " + verify.stderr.strip())
                            refs = subprocess.run(["git", "bundle", "list-heads", str(bundle_file)],
                                                  capture_output=True, text=True)
                            checks += 1
                            if refs.returncode != 0:
                                fails.append("Git Bundle list-heads failed")
                            else:
                                actual_refs = [line.strip() for line in refs.stdout.splitlines() if line.strip()]
                                if actual_refs != bundle_meta.get("refs"):
                                    fails.append("Git Bundle actual refs differ from manifest")
                    except Exception as exc:
                        fails.append(f"Git Bundle missing/unreadable: {exc}")
                # Verify all evidence entries in the archive manifest against ZIP bytes.
                evidence = archive_manifest.get("evidence_files", [])
                for item in evidence:
                    checks += 1
                    try:
                        raw = zf.read(f"{ctx['run_id']}/{item['path']}")
                        got = hashlib.sha256(raw).hexdigest()
                        if got != item.get("sha256") or len(raw) != item.get("size_bytes"):
                            fails.append(f"archive manifest evidence mismatch: {item.get('path')}")
                    except Exception:
                        fails.append(f"archive manifest evidence missing: {item.get('path')}")
    except Exception as exc:
        fails.append(f"archive ZIP unreadable: {exc}")
    return emit(ctx["out"], "archive", checks, len(fails),
                "PASS" if not fails else "FAIL",
                {"run_id": ctx["run_id"], "zip_path": str(zip_path),
                 "zip_sha256": actual_zip_sha, "zip_size_bytes": actual_zip_size,
                 "failures": fails[:80]})


def check_test_results(ctx):
    data = ctx["test_results"]
    checks = 0
    fails = []
    checks += 1
    if data.get("run_id") != ctx["run_id"]:
        fails.append("test results RUN_ID mismatch")
    checks += 1
    if data.get("code_commit") != ctx["code_commit"]:
        fails.append("test results code_commit mismatch")
    tests = data.get("tests")
    if not isinstance(tests, list) or not tests:
        return emit(ctx["out"], "test_results", checks, 1, "FAIL",
                    {"run_id": ctx["run_id"], "failures": ["test list empty"]})
    ids = set()
    names = set()
    for test in tests:
        tid = test.get("test_id")
        checks += 1
        if tid in ids:
            fails.append(f"duplicate test_id {tid}")
        ids.add(tid)
        checks += 1
        if test.get("test_name") in names:
            fails.append(f"duplicate test_name {test.get('test_name')}")
        names.add(test.get("test_name"))
        for key in ("test_id", "test_name", "run_id", "code_commit", "status",
                    "command", "exit_code", "checks", "failures", "random_seed",
                    "toolchain_version", "environment_path", "environment_sha256",
                    "source_files", "log_path", "log_sha256", "duration_seconds"):
            checks += 1
            if key not in test:
                fails.append(f"{tid}: missing {key}")
        if test.get("run_id") != ctx["run_id"] or test.get("code_commit") != ctx["code_commit"]:
            fails.append(f"{tid}: run/commit mismatch")
        if test.get("status") == "PASS":
            checks += 1
            if test.get("exit_code") != 0:
                fails.append(f"{tid}: PASS exit {test.get('exit_code')}")
            checks += 1
            if test.get("failures") != 0:
                fails.append(f"{tid}: PASS failures {test.get('failures')}")
            checks += 1
            if not isinstance(test.get("checks"), int) or test.get("checks") <= 0:
                fails.append(f"{tid}: PASS checks<=0")
            checks += 1
            if not isinstance(test.get("duration_seconds"), (int, float)) or test.get("duration_seconds") <= 0:
                fails.append(f"{tid}: PASS duration invalid")
            checks += 1
            if not valid_version(test.get("toolchain_version")):
                fails.append(f"{tid}: toolchain invalid")
            checks += 1
            seed = str(test.get("random_seed", ""))
            if seed.lower() in ("", "deterministic", "not_recorded", "unknown"):
                fails.append(f"{tid}: forbidden seed")
            if test.get("test_name") in ("map_random", "S06-108", "S06-112") and seed == "NOT_APPLICABLE":
                fails.append(f"{tid}: random test lacks seed")
            log = ROOT / test.get("log_path", "")
            checks += 1
            if not log.is_file() or sha(log) != test.get("log_sha256"):
                fails.append(f"{tid}: log/SHA mismatch")
            checks += 1
            if not test.get("source_files"):
                fails.append(f"{tid}: no source files")
            for src in test.get("source_files", []):
                checks += 1
                p = ROOT / src.get("path", "")
                if not p.is_file() or sha(p) != src.get("working_tree_sha256"):
                    fails.append(f"{tid}: source SHA mismatch {src.get('path')}")
                for key in ("git_blob_sha", "working_tree_sha256", "line_ending",
                            "size_bytes", "role"):
                    if key not in src:
                        fails.append(f"{tid}: source missing {key}")
            env = ROOT / test.get("environment_path", "")
            checks += 1
            if not env.is_file() or sha(env) != test.get("environment_sha256"):
                fails.append(f"{tid}: environment/SHA mismatch")
            checks += 1
            if test.get("collector_sha256") != sha("tools/collect_validation_results.py"):
                fails.append(f"{tid}: collector SHA mismatch")
    # S06-112 JSON binding.
    s112 = next((t for t in tests if t.get("test_name") == "s06_112"), None)
    if s112:
        for arg in s112.get("command_argv", []):
            pass
        out_arg = None
        argv = s112.get("command_argv", [])
        for index, arg in enumerate(argv):
            if arg == "--output" and index + 1 < len(argv):
                out_arg = argv[index + 1]
        checks += 1
        if not out_arg or not (ROOT / out_arg).is_file():
            fails.append("S06-112 JSON missing")
        else:
            j = read_json(ROOT / out_arg)
            checks += 1
            if j.get("run_id") != ctx["run_id"] or j.get("code_commit") != ctx["code_commit"]:
                fails.append("S06-112 JSON run/commit mismatch")
            checks += 1
            if j.get("axis_cases_checked") != s112.get("checks"):
                fails.append("S06-112 JSON checks mismatch")
            checks += 1
            if len(j.get("failures", [])) != s112.get("failures"):
                fails.append("S06-112 JSON failures mismatch")
    return emit(ctx["out"], "test_results", checks, len(fails),
                "PASS" if not fails else "FAIL",
                {"run_id": ctx["run_id"], "failures": fails[:60]})


def check_issues(ctx):
    text = ctx["issues_text"]
    details = ctx["details_text"]
    checks = 0
    fails = []
    ids = re.findall(r"^\|\s*(FLIGHT-LOCAL-\d+)\s*\|", text, re.M)
    checks += 1
    if len(ids) != len(set(ids)):
        fails.append("duplicate issue IDs")
    checks += 1
    if "GITHUB_PUBLICATION = PENDING" not in text and "GITHUB_PUBLICATION=PENDING" not in text:
        fails.append("GITHUB_PUBLICATION not PENDING")
    for n in range(35, 42):
        iid = f"FLIGHT-LOCAL-{n}"
        checks += 1
        if iid not in ids:
            fails.append(f"missing {iid} in register")
        section = re.search(rf"^##\s+{re.escape(iid)}\s*$(.*?)(?=^##\s+|\Z)",
                            details, re.M | re.S)
        checks += 1
        if not section:
            fails.append(f"missing detail section {iid}")
            continue
        body = section.group(1)
        for label in ("根因", "首次发现", "当前基线", "责任方", "安全影响",
                      "修复步骤", "复测命令", "预期结果", "实际结果", "日志路径",
                      "日志 SHA256", "解除条件", "下一检查点"):
            checks += 1
            if not re.search(rf"^-\s*{re.escape(label)}\s*[:：]", body, re.M):
                fails.append(f"{iid}: missing field {label}")
        for forbidden in ("see handover", "see log", "MISSING（无解释）"):
            checks += 1
            if forbidden.lower() in body.lower():
                fails.append(f"{iid}: forbidden placeholder {forbidden}")
    return emit(ctx["out"], "issues", checks, len(fails),
                "PASS" if not fails else "FAIL",
                {"run_id": ctx["run_id"], "failures": fails[:60]})


def check_blocker(ctx):
    text = ctx["blocker_text"]
    checks = 0
    fails = []
    for n in range(1, 8):
        checks += 1
        if f"B0{n}" not in text:
            fails.append(f"missing B0{n}")
    checks += 1
    if "ISSUE_REGISTER" not in text:
        fails.append("blocker board is not declared as issue-register derivative")
    checks += 1
    if "4,248" not in text and "4248" not in text:
        fails.append("B06 lower bound 4248 missing")
    checks += 1
    if "4,096" not in text and "4096" not in text:
        fails.append("B06 budget 4096 missing")
    checks += 1
    if "232" not in text:
        fails.append("static headroom 232 B missing")
    checks += 1
    if "NOT_RUN" not in text:
        fails.append("B07 NOT_RUN missing")
    checks += 1
    if re.search(r"MR-009[^\n]*APPROVED", text) and "WAITING" not in text:
        fails.append("MR-009 appears self-approved")
    return emit(ctx["out"], "blocker", checks, len(fails),
                "PASS" if not fails else "FAIL",
                {"run_id": ctx["run_id"], "failures": fails})


def check_product(ctx):
    src = read_text(ROOT / "middleware/model/fms/quad_x4/flight_policy/quad_x4_product_config.c")
    manifest = ctx["manifest"].get("product_config", {})
    checks = 0
    fails = []
    expected = {
        "default_max_speed_mps": 8.0,
        "default_max_accel_mps2": 4.0,
        "jerk_x_mps3": 16.0,
        "jerk_y_mps3": 16.0,
        "jerk_z_mps3": 8.0,
        "valid_mask": "0x7F",
    }
    for key, value in expected.items():
        checks += 1
        if manifest.get(key) != value:
            fails.append(f"manifest product_config.{key}={manifest.get(key)} != {value}")
    for token in ("default_max_speed_mps = 8.0f",
                  "default_max_accel_mps2 = 4.0f",
                  "default_max_accel_mps2 * 4.0f",
                  "valid_mask = 0x7Fu"):
        checks += 1
        if token not in src:
            fails.append(f"product source missing {token}")
    checks += 1
    if "obstacle_source_mode" in manifest and str(manifest.get("obstacle_source_mode")).upper() != "NONE":
        fails.append("obstacle source mode is not NONE")
    return emit(ctx["out"], "product", checks, len(fails),
                "PASS" if not fails else "FAIL",
                {"run_id": ctx["run_id"], "failures": fails})


def check_stack(ctx):
    """Validate the stack invariant, not a historical number.

    The gate is: budget is 4096; a PASS requires a COMPLETE call graph, a known
    lower bound within budget and no reachable unknown edge; a BLOCKED result
    requires an actual breach; an UNKNOWN result requires an incomplete graph.
    The historical R29 4,248 B figure stays in the acceptance record and the
    blocker board, but it is not the invariant for later runs.
    """
    data = ctx["stack"]
    checks = 0
    fails = []
    lower = data.get("known_lower_bound_bytes")
    limit = data.get("limit_bytes")
    status = data.get("status")
    callgraph = data.get("callgraph_status")
    checks += 1
    if not isinstance(lower, int) or lower <= 0:
        fails.append("known lower bound missing")
    checks += 1
    if limit != 4096:
        fails.append("budget limit is not 4096")
    checks += 1
    if status not in ("PASS", "BLOCKED", "UNKNOWN"):
        fails.append(f"stack status {status} is not fail-closed")
    checks += 1
    if callgraph not in ("UNKNOWN", "COMPLETE"):
        fails.append("callgraph status missing")
    checks += 1
    if not isinstance(data.get("unknown_edges"), list):
        fails.append("unknown edges not recorded")
    if status == "PASS":
        checks += 1
        if callgraph != "COMPLETE":
            fails.append("PASS with incomplete call graph")
        checks += 1
        if isinstance(lower, int) and isinstance(limit, int) and lower > limit:
            fails.append("PASS while the lower bound exceeds the budget")
        checks += 1
        if data.get("unknown_edges_reachable"):
            fails.append("PASS with reachable unknown edges")
    elif status == "BLOCKED":
        checks += 1
        if not (isinstance(lower, int) and isinstance(limit, int) and lower > limit):
            fails.append("BLOCKED without a budget breach")
    else:
        checks += 1
        if callgraph != "UNKNOWN":
            fails.append("UNKNOWN status with a COMPLETE call graph")
    final = ("PASS" if not fails and status == "PASS" else
             "BLOCKED" if status == "BLOCKED" else
             "UNKNOWN" if not fails else "FAIL")
    return emit(ctx["out"], "stack", checks, len(fails), final,
                {"run_id": ctx["run_id"], "failures": fails,
                 "lower_bound": lower, "limit": limit,
                 "callgraph_status": callgraph,
                 "reachable_unknown_edges": len(data.get("unknown_edges_reachable") or [])})


def check_handover(ctx):
    hdir = Path(ctx["paths"]["handover_dir"])
    checks = 0
    fails = []
    required = ("SESSION_HANDOVER.md", "handover_state.json", "git_status.txt",
                "git_head.txt", "git_branch.txt", "git_diff_cached.patch",
                "git_diff_worktree.patch", "untracked_manifest.json",
                "modified_files_manifest.json", "blocker_snapshot.json",
                "test_progress.json", "active_processes.json", "next_actions.md",
                "NEXT_SESSION_START_PROMPT.txt", "HANDOVER_SHA256SUMS.txt")
    for name in required:
        checks += 1
        if not (hdir / name).is_file():
            fails.append(f"handover missing {name}")
    text = ctx["handover_text"]
    for letter in "ABCDEFGHIJKLMNOPQRSTUVW":
        checks += 1
        if not re.search(rf"^##\s*{letter}[.、:：]", text, re.M):
            fails.append(f"handover missing section {letter}")
    sums = hdir / "HANDOVER_SHA256SUMS.txt"
    if sums.is_file():
        for line in sums.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            parts = line.split("  ", 1)
            checks += 1
            if len(parts) != 2:
                fails.append("malformed HANDOVER_SHA256SUMS line")
                continue
            expected, rel = parts
            p = hdir / rel
            if not p.is_file() or sha(p) != expected:
                fails.append(f"handover SHA mismatch {rel}")
    ack = Path(ctx["paths"]["handover_dir"]) / "TAKEOVER_ACK.md"
    checks += 1
    if not ack.is_file():
        fails.append("TAKEOVER_ACK missing")
    else:
        ack_text = ack.read_text(encoding="utf-8", errors="replace")
        checks += 1
        if "TAKEOVER_CONFIRMED" not in ack_text:
            fails.append("TAKEOVER_ACK not CONFIRMED")
    state = hdir / "handover_state.json"
    if state.is_file():
        try:
            data = json.loads(state.read_text(encoding="utf-8"))
            checks += 1
            if data.get("head") != ctx["head"]:
                fails.append("handover state head mismatch")
            checks += 1
            if data.get("run_id") != ctx["run_id"]:
                fails.append("handover state RUN_ID mismatch")
        except Exception as exc:
            fails.append(f"handover state invalid: {exc}")
    return emit(ctx["out"], "handover", checks, len(fails),
                "PASS" if not fails else "FAIL",
                {"run_id": ctx["run_id"], "failures": fails[:40]})


def build_context(args):
    head = git("rev-parse", "HEAD")
    branch = git("branch", "--show-current")
    remote_ref = f"origin/{branch}" if branch else "origin/opt/stage-02-m5-06-safety-validator"
    remote = git("rev-parse", remote_ref)
    ahead_text = git("rev-list", "--count", f"{remote_ref}..HEAD")
    ahead = int(ahead_text) if ahead_text and ahead_text.isdigit() else None
    paths = {
        "manifest": ROOT / (args.manifest or DEFAULT["manifest"]),
        "environment": ROOT / (args.environment or DEFAULT["environment"]),
        "test_results": ROOT / (args.test_results or DEFAULT["test_results"]),
        "archive_report": ROOT / (args.archive_report or DEFAULT["archive_report"]),
        "issue_register": ROOT / (args.issue_register or DEFAULT["issue_register"]),
        "issue_details": ROOT / (args.issue_details or DEFAULT["issue_details"]),
        "blocker_board": ROOT / (args.blocker_board or DEFAULT["blocker_board"]),
        "handover": ROOT / (args.handover or DEFAULT["handover"]),
        "handover_dir": ROOT / (args.handover_dir or DEFAULT["handover_dir"]),
        "stack": ROOT / (args.stack or DEFAULT["stack"]),
        "source_manifest": ROOT / (args.source_manifest or DEFAULT["source_manifest"]),
    }
    manifest = read_json(paths["manifest"])
    lv = manifest.get("latest_local_validation", {})
    run_id = args.run_id or lv.get("run_id")
    code_commit = args.code_commit or lv.get("validated_code_commit")
    return {
        "args": args,
        "out": Path(args.output_dir) if args.output_dir else ROOT / "build/validation/regenerated/checks",
        "head": head,
        "branch": branch,
        "remote_head": remote,
        "ahead": ahead,
        "run_id": run_id,
        "code_commit": code_commit,
        "manifest": manifest,
        "environment": read_json(paths["environment"]) if paths["environment"].is_file() else {},
        "test_results": read_json(paths["test_results"]) if paths["test_results"].is_file() else {},
        "archive_report": read_json(paths["archive_report"]) if paths["archive_report"].is_file() else {},
        "issues_text": paths["issue_register"].read_text(encoding="utf-8", errors="replace") if paths["issue_register"].is_file() else "",
        "details_text": paths["issue_details"].read_text(encoding="utf-8", errors="replace") if paths["issue_details"].is_file() else "",
        "blocker_text": paths["blocker_board"].read_text(encoding="utf-8", errors="replace") if paths["blocker_board"].is_file() else "",
        "handover_text": paths["handover"].read_text(encoding="utf-8", errors="replace") if paths["handover"].is_file() else "",
        "stack": read_json(paths["stack"]) if paths["stack"].is_file() else {},
        "paths": {k: str(v) for k, v in paths.items()},
    }


CHECKS = {
    "manifest": check_manifest,
    "environment": check_environment,
    "archive": check_archive,
    "test-results": check_test_results,
    "issues": check_issues,
    "blocker": check_blocker,
    "product": check_product,
    "stack": check_stack,
    "handover": check_handover,
}


def run(args):
    ctx = build_context(args)
    names = list(CHECKS) if args.check == "all" else [args.check]
    results = []
    for name in names:
        results.append(CHECKS[name](ctx))
    bad = [r for r in results if r["status"] != "PASS"]
    if not bad:
        print("EVIDENCE_CHECK_PASS")
        return 0
    if any(r["status"] == "FAIL" for r in bad):
        print("EVIDENCE_CHECK_FAIL")
        return 1
    if any(r["status"] == "UNKNOWN" for r in bad):
        print("EVIDENCE_CHECK_UNKNOWN")
        return 3
    print("EVIDENCE_CHECK_BLOCKED")
    return 2


# ---------------------------------------------------------------------------
# Negative self-test. It copies the current R28 evidence set and mutates one
# critical field at a time; every mutation must make the aggregate check fail.
# ---------------------------------------------------------------------------

def self_test():
    """Run synthetic, baseline-valid unit fixtures for archive/evidence checks.

    Do not mutate a live R28 evidence set and treat its pre-existing BLOCKED
    gates as successful negative-test rejection. Each unit fixture constructs a
    known-valid baseline, then mutates one field and expects the targeted check
    to reject it.
    """
    modules = (
        "tests.tools.test_prepare_archive_metadata",
        "tests.tools.test_validation_evidence_archive",
        "tests.tools.test_rtt_package_tree",
    )
    optimization = ["-O"] if sys.flags.optimize else []
    command = [sys.executable, *optimization, "-m", "unittest", "-v", *modules]
    proc = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    if proc.stdout:
        print(proc.stdout, end="")
    if proc.stderr:
        print(proc.stderr, end="", file=sys.stderr)
    if proc.returncode == 0:
        # The collector contract requires the count: it parses
        # "EVIDENCE_CHECKER_SELF_TEST_PASS (<n> negative cases)".
        # unittest writes "Ran N tests" to stderr.
        combined = (proc.stdout or "") + (proc.stderr or "")
        match = re.search(r"Ran (\d+) tests?", combined)
        cases = int(match.group(1)) if match else 0
        print(f"EVIDENCE_CHECKER_SELF_TEST_PASS ({cases} negative cases)")
    else:
        print("EVIDENCE_CHECKER_SELF_TEST_FAIL", file=sys.stderr)
    return proc.returncode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", default="all",
                    choices=["all", "manifest", "environment", "archive", "test-results",
                             "issues", "blocker", "product", "stack", "handover"])
    ap.add_argument("--run-id")
    ap.add_argument("--code-commit")
    ap.add_argument("--manifest")
    ap.add_argument("--archive-report")
    ap.add_argument("--test-results")
    ap.add_argument("--environment")
    ap.add_argument("--issue-register")
    ap.add_argument("--issue-details")
    ap.add_argument("--blocker-board")
    ap.add_argument("--handover")
    ap.add_argument("--handover-dir")
    ap.add_argument("--stack")
    ap.add_argument("--source-manifest")
    ap.add_argument("--output-dir")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
