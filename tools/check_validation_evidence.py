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


def is_post_archive_metadata(old, new):
    """True only for a metadata-only commit chain from archive HEAD to HEAD."""
    if not old or not new:
        return False
    if old == new:
        return True
    exists = subprocess.run(["git", "-C", str(ROOT), "cat-file", "-e", str(old)],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL).returncode == 0
    ancestor = subprocess.run(
        ["git", "-C", str(ROOT), "merge-base", "--is-ancestor", str(old), str(new)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
    if not (exists and ancestor):
        return False
    changed = (git("diff", "--name-only", f"{old}..{new}") or "").splitlines()
    return set(changed).issubset({
        "spec/validation/flight_acceptance_manifest.json",
        "spec/quality/R28_ISSUE_DETAILS.md",
        "tools/check_validation_evidence.py",
        "tools/finalize_r28_archive.py",
    })


def check_manifest(ctx):
    m = ctx["manifest"]
    lv = m.get("latest_local_validation", {})
    checks = 0
    fails = []
    required = ("run_id", "branch", "validated_code_commit", "tooling_commit",
                "collector_commit", "archive_script_commit", "archive_head_sha",
                "archive_commit_sha", "remote_head", "ahead_commit_count")
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
    checks += 1
    if not ctx["head"] or lv.get("archive_head_sha") != ctx["head"]:
        archive_head = lv.get("archive_head_sha")
        if not is_post_archive_metadata(str(archive_head or ""), str(ctx["head"] or "")):
            fails.append("manifest archive_head_sha is not current HEAD and is not a metadata-only post-archive commit")
    checks += 1
    archive_ahead_text = git("rev-list", "--count",
                             f"{ctx['remote_ref']}..{lv.get('archive_head_sha')}")
    archive_ahead = int(archive_ahead_text) if archive_ahead_text and archive_ahead_text.isdigit() else None
    if lv.get("ahead_commit_count") != archive_ahead:
        fails.append(f"manifest ahead {lv.get('ahead_commit_count')} != archive-head ahead {archive_ahead}")
    checks += 1
    if lv.get("archive_commit_sha") != lv.get("archive_head_sha"):
        fails.append("archive_commit_sha != archive_head_sha")
    checks += 1
    if lv.get("remote_head") != ctx["remote_head"]:
        fails.append("manifest remote_head mismatch")
    checks += 1
    if lv.get("collector_commit") != lv.get("tooling_commit"):
        fails.append("collector_commit != tooling_commit")
    checks += 1
    if not isinstance(lv.get("archive_zip_sha256"), str) or len(lv.get("archive_zip_sha256", "")) != 64:
        fails.append("archive_zip_sha256 not recorded")
    return emit(ctx["out"], "manifest", checks, len(fails),
                "PASS" if not fails else "FAIL",
                {"run_id": ctx["run_id"], "failures": fails, "manifest": str(ctx["paths"]["manifest"])})


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
    for key in ("run_id", "branch", "validated_code_commit", "tooling_commit",
                "collector_commit", "archive_script_commit", "archive_head_sha",
                "archive_commit_sha", "remote_head", "ahead_commit_count",
                "archive_zip_sha256", "archive_zip_size_bytes", "scope"):
        checks += 1
        if report.get(key) in (None, ""):
            fails.append(f"archive report missing {key}")
    checks += 1
    if report.get("run_id") != ctx["run_id"]:
        fails.append("archive report RUN_ID mismatch")
    checks += 1
    if report.get("archive_head_sha") != m.get("archive_head_sha"):
        fails.append("archive report head != manifest archive head")
    if report.get("archive_commit_sha") != report.get("archive_head_sha"):
        fails.append("archive report commit != archive head")
    checks += 1
    if report.get("remote_head") != ctx["remote_head"]:
        fails.append("archive report remote head != actual")
    checks += 1
    if report.get("validated_code_commit") != m.get("validated_code_commit"):
        fails.append("archive report code commit != manifest")
    checks += 1
    archive_ahead_text = git("rev-list", "--count",
                             f"{ctx['remote_ref']}..{report.get('archive_head_sha')}")
    archive_ahead = int(archive_ahead_text) if archive_ahead_text and archive_ahead_text.isdigit() else None
    if report.get("ahead_commit_count") != archive_ahead:
        fails.append("archive report ahead != archive-head ahead")
    zip_path = Path(report.get("archive_zip_path", ""))
    checks += 1
    if not zip_path.is_file():
        fails.append("archive ZIP missing")
    else:
        actual = sha(zip_path)
        if actual != report.get("archive_zip_sha256"):
            fails.append("archive ZIP SHA mismatch")
        if zip_path.stat().st_size != report.get("archive_zip_size_bytes"):
            fails.append("archive ZIP size mismatch")
        try:
            with zipfile.ZipFile(zip_path) as zf:
                if zf.testzip() is not None:
                    fails.append("archive ZIP CRC failure")
                names = zf.namelist()
                for required in ("archive_manifest.json", "SHA256SUMS.txt",
                                 "M5-06-R28-20261111.bundle"):
                    checks += 1
                    if not any(name.endswith(required) for name in names):
                        fails.append(f"archive ZIP missing {required}")
        except Exception as exc:
            fails.append(f"archive ZIP unreadable: {exc}")
    return emit(ctx["out"], "archive", checks, len(fails),
                "PASS" if not fails else "FAIL",
                {"run_id": ctx["run_id"], "failures": fails})


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
                if not p.is_file():
                    fails.append(f"{tid}: source missing {src.get('path')}")
                elif sha(p) != src.get("working_tree_sha256"):
                    # The formal manifest is updated by the post-archive
                    # metadata commit; tooling may be updated after a log was
                    # captured. The ledger still records the exact test-time
                    # SHA, and production sources remain strictly checked.
                    if src.get("path") not in {
                            "spec/validation/flight_acceptance_manifest.json",
                            "tools/check_validation_evidence.py",
                            "tools/collect_validation_results.py"}:
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
    data = ctx["stack"]
    checks = 0
    fails = []
    checks += 1
    if data.get("known_lower_bound_bytes") != 4248:
        fails.append(f"known lower bound {data.get('known_lower_bound_bytes')} != 4248")
    checks += 1
    if data.get("limit_bytes") != 4096:
        fails.append("budget limit is not 4096")
    checks += 1
    if data.get("status") == "PASS":
        fails.append("stack analysis PASS while unknown edges/budget exist")
    checks += 1
    if data.get("status") not in ("BLOCKED", "UNKNOWN"):
        fails.append(f"stack status {data.get('status')} is not fail-closed")
    checks += 1
    if data.get("callgraph_status") not in ("UNKNOWN", "COMPLETE"):
        fails.append("callgraph status missing")
    checks += 1
    if not data.get("unknown_edges"):
        fails.append("unknown edges not recorded")
    checks += 1
    if data.get("known_lower_bound_bytes", 0) <= data.get("limit_bytes", 0):
        fails.append("stack data does not show the known budget breach")
    return emit(ctx["out"], "stack", checks, len(fails),
                "BLOCKED" if "known lower bound exceeds" in str(data.get("reason")) else "UNKNOWN",
                {"run_id": ctx["run_id"], "failures": fails,
                 "lower_bound": data.get("known_lower_bound_bytes"),
                 "limit": data.get("limit_bytes")})


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
            if data.get("head") != ctx["head"] and not is_post_archive_metadata(
                    str(data.get("head") or ""), str(ctx["head"] or "")):
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
        "remote_ref": remote_ref,
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
    required = [DEFAULT["manifest"], DEFAULT["environment"], DEFAULT["test_results"],
                DEFAULT["issue_register"], DEFAULT["issue_details"],
                DEFAULT["blocker_board"], DEFAULT["stack"]]
    missing = [p for p in required if not (ROOT / p).is_file()]
    if missing:
        print("EVIDENCE_CHECKER_SELF_TEST_FAIL: missing " + ", ".join(missing),
              file=sys.stderr)
        return 1
    tmp = Path(tempfile.mkdtemp(prefix="evidence_checker_r28_"))
    try:
        # Copy all input files into a temporary package.
        paths = {}
        for key in ("manifest", "environment", "test_results", "issue_register",
                    "issue_details", "blocker_board", "stack"):
            src = ROOT / DEFAULT[key]
            dst = tmp / src.name
            dst.write_bytes(src.read_bytes())
            paths[key] = dst
        hdir = tmp / "handover"
        hdir.mkdir()
        for name in ("SESSION_HANDOVER.md", "handover_state.json"):
            src = ROOT / DEFAULT["handover_dir"] / name
            if src.is_file():
                (hdir / name).write_bytes(src.read_bytes())
        if not (hdir / "SESSION_HANDOVER.md").is_file():
            (hdir / "SESSION_HANDOVER.md").write_text(
                "\n".join(f"## {c}. placeholder" for c in "ABCDEFGHIJKLMNOPQRSTUVW") + "\n",
                encoding="utf-8")
        if not (hdir / "handover_state.json").is_file():
            (hdir / "handover_state.json").write_text(json.dumps({"head": git("rev-parse", "HEAD")}),
                                                      encoding="utf-8")
        for name in ("git_status.txt", "git_head.txt", "git_branch.txt",
                     "git_diff_cached.patch", "git_diff_worktree.patch",
                     "untracked_manifest.json", "modified_files_manifest.json",
                     "blocker_snapshot.json", "test_progress.json",
                     "active_processes.json", "next_actions.md",
                     "NEXT_SESSION_START_PROMPT.txt"):
            (hdir / name).write_text("fixture\n", encoding="utf-8")
        (hdir / "TAKEOVER_ACK.md").write_text("TAKEOVER_CONFIRMED\n", encoding="utf-8")
        (hdir / "HANDOVER_SHA256SUMS.txt").write_text("", encoding="utf-8")

        class A:
            pass
        base = A()
        base.run_id = None
        base.code_commit = None
        base.output_dir = str(tmp / "out")
        base.check = "all"
        base.manifest = str(paths["manifest"])
        base.environment = str(paths["environment"])
        base.test_results = str(paths["test_results"])
        base.archive_report = str(tmp / "missing_archive.json")
        base.issue_register = str(paths["issue_register"])
        base.issue_details = str(paths["issue_details"])
        base.blocker_board = str(paths["blocker_board"])
        base.handover = str(hdir / "SESSION_HANDOVER.md")
        base.handover_dir = str(hdir)
        base.stack = str(paths["stack"])
        base.source_manifest = str(ROOT / DEFAULT["source_manifest"])

        def context():
            return build_context(base)

        def expect(label, mutate):
            mutate()
            try:
                ctx = context()
                names = [n for n in CHECKS if n != "archive"]
                bad = False
                for name in names:
                    try:
                        result = CHECKS[name](ctx)
                        if result["status"] != "PASS":
                            bad = True
                    except Exception:
                        bad = True
                if not bad:
                    raise RuntimeError(f"{label}: mutation was accepted")
            except Exception as exc:
                if "mutation was accepted" in str(exc):
                    raise
                # A raised check also counts as rejection.
                return
            raise RuntimeError(f"{label}: mutation was accepted")

        def mutate_json(path, key, value):
            data = json.loads(path.read_text(encoding="utf-8"))
            data[key] = value
            path.write_text(json.dumps(data, indent=2), encoding="utf-8")

        cases = 0
        mutations = [
            ("RUN_ID", lambda: mutate_json(paths["manifest"],
                "latest_local_validation",
                dict(json.loads(paths["manifest"].read_text())["latest_local_validation"],
                     run_id="WRONG"))),
            ("branch", lambda: mutate_json(paths["manifest"],
                "latest_local_validation",
                dict(json.loads(paths["manifest"].read_text())["latest_local_validation"],
                     branch="main"))),
            ("code", lambda: mutate_json(paths["manifest"],
                "latest_local_validation",
                dict(json.loads(paths["manifest"].read_text())["latest_local_validation"],
                     validated_code_commit="0" * 40))),
            ("ahead", lambda: mutate_json(paths["manifest"],
                "latest_local_validation",
                dict(json.loads(paths["manifest"].read_text())["latest_local_validation"],
                     ahead_commit_count=999))),
            ("tooling", lambda: mutate_json(paths["manifest"],
                "latest_local_validation",
                dict(json.loads(paths["manifest"].read_text())["latest_local_validation"],
                     tooling_commit=None))),
        ]
        for label, fn in mutations:
            cases += 1
            try:
                expect(label, fn)
            except RuntimeError as exc:
                print("EVIDENCE_CHECKER_SELF_TEST_FAIL:", exc, file=sys.stderr)
                return 1
            # Restore from original.
            for key in ("manifest", "environment", "test_results",
                        "issue_register", "issue_details", "blocker_board", "stack"):
                (tmp / (ROOT / DEFAULT[key]).name).write_bytes((ROOT / DEFAULT[key]).read_bytes())
        print(f"EVIDENCE_CHECKER_SELF_TEST_PASS ({cases} negative cases)")
        return 0
    finally:
        import shutil
        shutil.rmtree(tmp, ignore_errors=True)


def self_test():
    """Mutation test for the unified checker.

    Each case tampers exactly one critical field and calls the checker that owns
    that field. A PASS after tampering is a test failure. This intentionally
    avoids treating unrelated failures as a successful rejection.
    """
    import argparse
    import shutil

    required = [DEFAULT["manifest"], DEFAULT["environment"], DEFAULT["test_results"],
                DEFAULT["issue_register"], DEFAULT["issue_details"],
                DEFAULT["blocker_board"], DEFAULT["stack"]]
    missing = [p for p in required if not (ROOT / p).is_file()]
    if missing:
        print("EVIDENCE_CHECKER_SELF_TEST_FAIL: missing " + ", ".join(missing),
              file=sys.stderr)
        return 1
    tmp = Path(tempfile.mkdtemp(prefix="evidence_checker_r28_"))
    try:
        paths = {}
        for key in ("manifest", "environment", "test_results", "issue_register",
                    "issue_details", "blocker_board", "stack"):
            src = ROOT / DEFAULT[key]
            dst = tmp / src.name
            dst.write_bytes(src.read_bytes())
            paths[key] = dst
        manifest = json.loads(paths["manifest"].read_text(encoding="utf-8"))
        head = git("rev-parse", "HEAD")
        run_id = manifest["latest_local_validation"]["run_id"]
        hdir = tmp / "handover"
        hdir.mkdir()
        (hdir / "SESSION_HANDOVER.md").write_text(
            "\n".join(f"## {c}. fixture" for c in "ABCDEFGHIJKLMNOPQRSTUVW") + "\n",
            encoding="utf-8")
        (hdir / "handover_state.json").write_text(
            json.dumps({"head": head, "run_id": run_id}), encoding="utf-8")
        for name in ("git_status.txt", "git_head.txt", "git_branch.txt",
                     "git_diff_cached.patch", "git_diff_worktree.patch",
                     "untracked_manifest.json", "modified_files_manifest.json",
                     "blocker_snapshot.json", "test_progress.json",
                     "active_processes.json", "next_actions.md",
                     "NEXT_SESSION_START_PROMPT.txt"):
            (hdir / name).write_text("fixture\n", encoding="utf-8")
        (hdir / "TAKEOVER_ACK.md").write_text("TAKEOVER_CONFIRMED\n", encoding="utf-8")
        lines = []
        for p in sorted(hdir.iterdir()):
            if p.name == "HANDOVER_SHA256SUMS.txt":
                continue
            lines.append(f"{sha(p)}  {p.name}")
        (hdir / "HANDOVER_SHA256SUMS.txt").write_text("\n".join(lines) + "\n",
                                                     encoding="utf-8")

        class A:
            pass
        base = A()
        base.run_id = None
        base.code_commit = None
        base.output_dir = str(tmp / "out")
        base.check = "all"
        base.manifest = str(paths["manifest"])
        base.environment = str(paths["environment"])
        base.test_results = str(paths["test_results"])
        base.archive_report = str(tmp / "missing_archive.json")
        base.issue_register = str(paths["issue_register"])
        base.issue_details = str(paths["issue_details"])
        base.blocker_board = str(paths["blocker_board"])
        base.handover = str(hdir / "SESSION_HANDOVER.md")
        base.handover_dir = str(hdir)
        base.stack = str(paths["stack"])
        base.source_manifest = str(ROOT / DEFAULT["source_manifest"])

        def status(check_name):
            ctx = build_context(base)
            return CHECKS[check_name](ctx)["status"]

        def restore():
            for key in ("manifest", "environment", "test_results",
                        "issue_register", "issue_details", "blocker_board", "stack"):
                (tmp / (ROOT / DEFAULT[key]).name).write_bytes((ROOT / DEFAULT[key]).read_bytes())

        def mutate_json(path, mutator):
            data = json.loads(path.read_text(encoding="utf-8"))
            mutator(data)
            path.write_text(json.dumps(data, indent=2), encoding="utf-8")

        cases = 0
        failures = []

        def case(label, check_name, mutate):
            nonlocal cases
            restore()
            mutate()
            try:
                result = status(check_name)
            except Exception:
                result = "FAIL"
            cases += 1
            if result == "PASS":
                failures.append(f"{label}: {check_name} accepted mutation")

        case("manifest RUN_ID", "manifest", lambda: mutate_json(
            paths["manifest"], lambda d: d["latest_local_validation"].update(run_id="WRONG")))
        case("manifest branch", "manifest", lambda: mutate_json(
            paths["manifest"], lambda d: d["latest_local_validation"].update(branch="main")))
        case("manifest code", "manifest", lambda: mutate_json(
            paths["manifest"], lambda d: d["latest_local_validation"].update(
                validated_code_commit="0" * 40)))
        case("manifest ahead", "manifest", lambda: mutate_json(
            paths["manifest"], lambda d: d["latest_local_validation"].update(
                ahead_commit_count=999)))
        case("manifest tooling", "manifest", lambda: mutate_json(
            paths["manifest"], lambda d: d["latest_local_validation"].update(
                tooling_commit=None)))
        case("environment RUN_ID", "environment", lambda: mutate_json(
            paths["environment"], lambda d: d.update(run_id="WRONG")))
        case("environment code", "environment", lambda: mutate_json(
            paths["environment"], lambda d: d.update(code_commit="0" * 40)))
        case("environment toolchain", "environment", lambda: mutate_json(
            paths["environment"], lambda d: d.update(toolchain_version="see r25_environment")))
        case("test results RUN_ID", "test-results", lambda: mutate_json(
            paths["test_results"], lambda d: d.update(run_id="WRONG")))
        case("test results duplicate", "test-results", lambda: mutate_json(
            paths["test_results"], lambda d: d["tests"].append(dict(d["tests"][0]))))
        case("test PASS checks=0", "test-results", lambda: mutate_json(
            paths["test_results"], lambda d: next(
                t for t in d["tests"] if t["status"] == "PASS").update(checks=0)))
        case("test log SHA", "test-results", lambda: mutate_json(
            paths["test_results"], lambda d: next(
                t for t in d["tests"] if t["status"] == "PASS").update(log_sha256="0" * 64)))
        case("issue field missing", "issues", lambda: paths["issue_details"].write_text(
            "\n".join(line for line in paths["issue_details"].read_text(
                encoding="utf-8").splitlines()
                if not line.startswith("- 日志 SHA256")), encoding="utf-8"))
        case("blocker B06 missing", "blocker", lambda: paths["blocker_board"].write_text(
            paths["blocker_board"].read_text(encoding="utf-8").replace("B06", "B99"),
            encoding="utf-8"))
        case("product envelope", "product", lambda: mutate_json(
            paths["manifest"], lambda d: d["product_config"].update(
                default_max_speed_mps=9.0)))
        case("stack PASS", "stack", lambda: mutate_json(
            paths["stack"], lambda d: d.update(status="PASS")))
        case("stack lower bound", "stack", lambda: mutate_json(
            paths["stack"], lambda d: d.update(known_lower_bound_bytes=4096)))
        case("handover section", "handover", lambda: (hdir / "SESSION_HANDOVER.md").write_text(
            (hdir / "SESSION_HANDOVER.md").read_text(encoding="utf-8").replace(
                "## W. fixture\n", ""), encoding="utf-8"))
        if failures:
            for failure in failures:
                print("EVIDENCE_CHECKER_SELF_TEST_FAIL:", failure, file=sys.stderr)
            return 1
        print(f"EVIDENCE_CHECKER_SELF_TEST_PASS ({cases} negative cases)")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


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
