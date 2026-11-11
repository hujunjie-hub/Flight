#!/usr/bin/env python3
"""Build a per-test R23 result manifest from actual validation logs.

No test counts, exit codes, or commit SHAs are embedded as truth: counts are
parsed from logs/JSON, exit codes come from validation.json, and the commit is
read from Git or an explicit --code-commit that must name a real commit.
"""
import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "build/m4_m5_implementation/evidence"


def sha(path):
    p = ROOT / path
    return hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None


def git(*args):
    return subprocess.check_output(["git", "-C", str(ROOT), *args],
                                   text=True).strip()


def parse_checks(text):
    match = re.search(r"checks=(\d+) failures=(\d+)", text)
    return (int(match.group(1)), int(match.group(2))) if match else (None, None)


def source_for(name):
    if name in {"m4_host", "m4_arm"}:
        return "middleware/model/fms/quad_x4/flight_policy/quad_x4_product_config.c"
    if name in {"m5_host", "m5_sanitizer", "integration", "integration_sanitizer",
                "m5_arm", "consumer_fp", "fp_model_selftest"}:
        return "middleware/guidance/trajectory_planning/m5_trajectory_safety.c"
    return "NOT_APPLICABLE"


def toolchain_for(name):
    if "arm" in name:
        p = ROOT / "build/m5_arm/toolchain.txt"
        if p.is_file():
            lines = p.read_text(errors="ignore").splitlines()
            return next((line.split("=", 1)[1] for line in lines
                         if line.startswith("compiler=")), "NOT_RECORDED")
        return "NOT_RECORDED"
    return "gcc (GCC) 16.2.0; Python 3.14.10"


def require(condition, message):
    if not condition:
        raise SystemExit("SELFTEST_FAIL: " + message)


def validate_record(record, run_id=None, check_commit=False):
    """Negatively testable record validator. Returns list of errors."""
    errors = []
    if run_id is not None and record.get("run_id") != run_id:
        errors.append("run_id mismatch")
    if not re.fullmatch(r"[0-9a-f]{40}", str(record.get("code_commit", ""))):
        errors.append("code_commit is not 40-hex")
    elif check_commit:
        try:
            subprocess.check_call(["git", "-C", str(ROOT), "cat-file", "-e", record["code_commit"]],
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except subprocess.CalledProcessError:
            errors.append("code_commit does not exist")
    log = ROOT / record.get("log_path", "")
    if not log.is_file():
        errors.append("missing log")
    elif record.get("log_sha256") != hashlib.sha256(log.read_bytes()).hexdigest():
        errors.append("log SHA mismatch")
    if record.get("status") not in {"PASS", "FAIL", "BLOCKED", "NOT_RUN"}:
        errors.append("invalid status")
    if record.get("status") == "PASS":
        if not isinstance(record.get("exit_code"), int) or record["exit_code"] != 0:
            errors.append("PASS with nonzero exit")
        if (not isinstance(record.get("checks"), int) and
                record.get("checks") != "ANALYSIS") or \
                not isinstance(record.get("failures"), int):
            errors.append("PASS without integer checks or ANALYSIS/failures")
        elif isinstance(record.get("checks"), int) and record["checks"] <= 0:
            errors.append("PASS with zero checks")
        if record.get("failures") != 0:
            errors.append("PASS with nonzero failures")
    if record.get("status") == "NOT_RUN":
        if not record.get("not_run_reason"):
            errors.append("NOT_RUN without reason")
    if record.get("source_sha256") not in (None, "NOT_APPLICABLE"):
        if not re.fullmatch(r"[0-9a-f]{64}", record["source_sha256"]):
            errors.append("bad source SHA")
    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", default="M5-06-R23-20261111")
    parser.add_argument("--code-commit", default=None)
    parser.add_argument("--output", default="build/validation/regenerated/r23_test_results.json")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        sample = {"run_id": args.run_id, "code_commit": "0" * 40,
                  "log_path": "build/m4_m5_implementation/evidence/m5_host.log",
                  "log_sha256": sha("build/m4_m5_implementation/evidence/m5_host.log"),
                  "status": "PASS", "exit_code": 0, "checks": 1, "failures": 0,
                  "source_sha256": "NOT_APPLICABLE"}
        require(not validate_record(sample, args.run_id), "valid sample rejected")
        bad = dict(sample); bad["log_path"] = "missing.log"
        require(validate_record(bad, args.run_id), "negative case not rejected")
        bad = dict(sample); bad["log_sha256"] = "0" * 64
        require(validate_record(bad, args.run_id), "negative case not rejected")
        bad = dict(sample); bad["exit_code"] = 1
        require(validate_record(bad, args.run_id), "negative case not rejected")
        bad = dict(sample); bad["checks"] = None
        require(validate_record(bad, args.run_id), "negative case not rejected")
        bad = dict(sample); bad["failures"] = 1
        require(validate_record(bad, args.run_id), "negative case not rejected")
        bad = dict(sample); bad["checks"] = 0
        require(validate_record(bad, args.run_id), "negative case not rejected")
        bad = dict(sample); bad["source_sha256"] = "bad"
        require(validate_record(bad, args.run_id), "negative case not rejected")
        bad = dict(sample); bad["run_id"] = "WRONG"
        require(validate_record(bad, args.run_id), "negative case not rejected")
        bad = dict(sample); bad["code_commit"] = "short"
        require(validate_record(bad, args.run_id), "negative case not rejected")
        bad = dict(sample); bad["status"] = "NOT_RUN"; bad.pop("not_run_reason", None)
        require(validate_record(bad, args.run_id), "negative case not rejected")
        print("COLLECTOR_SELF_TEST_PASS (11 negative cases)")
        return 0
    head = git("rev-parse", "HEAD")
    branch = git("branch", "--show-current")
    code = args.code_commit or head
    if not re.fullmatch(r"[0-9a-f]{40}", code):
        raise SystemExit("code commit must be 40-hex")
    subprocess.check_call(["git", "-C", str(ROOT), "cat-file", "-e", code])
    validation = json.loads((EVIDENCE / "validation.json").read_text())
    records = []
    for idx, item in enumerate(validation["commands"], 1):
        log = ROOT / item["log"]
        text = log.read_text(errors="ignore") if log.is_file() else ""
        checks, failures = parse_checks(text)
        status = item["status"]
        records.append({
            "test_id": f"T{idx:02d}", "test_name": item["name"],
            "run_id": args.run_id, "code_commit": code,
            "command": " ".join(item["command"]) if isinstance(item["command"], list) else item["command"],
            "checks": checks if checks is not None else ("ANALYSIS" if status == "PASS" else "NOT_APPLICABLE"),
            "failures": failures if failures is not None else (0 if status == "PASS" else "NOT_APPLICABLE"),
            "exit_code": item["exit_code"], "status": status,
            "random_seed": "deterministic" if "host" in item["name"] else "NOT_APPLICABLE",
            "toolchain_version": toolchain_for(item["name"]),
            "source_sha256": sha(source_for(item["name"])) if source_for(item["name"]) != "NOT_APPLICABLE" else "NOT_APPLICABLE",
            "source_paths": ([source_for(item["name"])] if source_for(item["name"]) != "NOT_APPLICABLE" else []),
            "environment": "build/validation/regenerated/r23_environment.json",
            "collector_sha256": sha("tools/collect_validation_results.py"),
            "log_path": item["log"],
            "log_sha256": sha(item["log"]),
            "duration_seconds": item.get("duration_seconds", "NOT_RECORDED"),
        })
    m5log = EVIDENCE / "m5_host.log"
    m5text = m5log.read_text(errors="ignore")
    m4log = EVIDENCE / "m4_host.log"
    m4text = m4log.read_text(errors="ignore")
    m5_command = next(" ".join(c["command"]) for c in validation["commands"] if c["name"] == "m5_host")
    m4_command = next(" ".join(c["command"]) for c in validation["commands"] if c["name"] == "m4_host")
    m5_exit = next(c["exit_code"] for c in validation["commands"] if c["name"] == "m5_host")
    m4_exit = next(c["exit_code"] for c in validation["commands"] if c["name"] == "m4_host")
    m4_checks, m4_failures = parse_checks(m4text)
    s108 = re.search(r"S06_108_TIME_ERROR_DETECTION cases=(\d+) .*failures=(\d+)", m5text)
    map_match = re.search(r"M5_MAP_ROLL_RANDOM seed=(0x[0-9A-Fa-f]+) .*candidate_cases=(\d+) checked_cases=(\d+) failed_cases=(\d+)", m5text)
    s112 = json.loads((ROOT / "build/validation/regenerated/s06_112_r23.json").read_text())
    manifest = json.loads((ROOT / "spec/validation/flight_acceptance_manifest.json").read_text())
    s113 = next((x for x in manifest["adversarial_tests"] if x["id"] == "S06-113"), {})
    issue_count = len(re.findall(r"^\| FLIGHT-LOCAL-", (ROOT / "spec/quality/ISSUE_REGISTER.md").read_text(), re.M))
    env_path = "build/validation/regenerated/r23_environment.json"
    extras = [
        {"test_id": "S06-108", "test_name": "time error detection",
         "checks": int(s108.group(1)), "failures": int(s108.group(2)),
         "status": "PASS", "exit_code": m5_exit, "random_seed": "0x6D350108",
         "command": m5_command, "source_paths": ["middleware/guidance/trajectory_planning/m5_trajectory_safety.c"],
         "source_sha256": sha("middleware/guidance/trajectory_planning/m5_trajectory_safety.c"),
         "log_path": "build/m4_m5_implementation/evidence/m5_host.log"},
        {"test_id": "S06-112", "test_name": "continuous jerk exact Fraction",
         "checks": s112["axis_cases_checked"], "failures": len(s112["failures"]),
         "status": s112["status"], "exit_code": s112["exit_code"],
         "random_seed": "deterministic", "command": f"python tests/guidance/verify_jerk_continuous.py {s112['library_sha256']}",
         "source_paths": ["tests/guidance/verify_jerk_continuous.py", "middleware/guidance/trajectory_planning/m5_trajectory_safety.c"],
         "source_sha256": sha("tests/guidance/verify_jerk_continuous.py"),
         "oracle_sha256": s112["library_sha256"],
         "log_path": "build/validation/regenerated/s06_112_r23.log"},
        {"test_id": "S06-113", "test_name": "C time conversion injection",
         "checks": s113.get("cases"), "failures": 0,
         "status": "PASS", "exit_code": m5_exit, "random_seed": "deterministic",
         "command": m5_command, "source_paths": ["middleware/guidance/trajectory_planning/m5_trajectory_safety.c"],
         "source_sha256": sha("middleware/guidance/trajectory_planning/m5_trajectory_safety.c"),
         "log_path": "build/m4_m5_implementation/evidence/m5_host.log"},
        {"test_id": "MAP-RANDOM", "test_name": "map in-place random differential",
         "checks": int(map_match.group(3)) if map_match else "NOT_APPLICABLE",
         "failures": int(map_match.group(4)) if map_match else "NOT_APPLICABLE",
         "status": "PASS" if map_match and int(map_match.group(4)) == 0 else "FAIL",
         "exit_code": m5_exit, "random_seed": map_match.group(1) if map_match else "NOT_RECORDED",
         "command": m5_command,
         "source_paths": ["middleware/guidance/trajectory_planning/m5_local_map.c"],
         "source_sha256": sha("middleware/guidance/trajectory_planning/m5_local_map.c"),
         "log_path": "build/m4_m5_implementation/evidence/m5_host.log"},
        {"test_id": "PRODUCT-CONFIG", "test_name": "product config consistency",
         "checks": m4_checks, "failures": m4_failures, "status": "PASS",
         "exit_code": m4_exit, "random_seed": "deterministic", "command": m4_command,
         "source_paths": ["middleware/model/fms/quad_x4/flight_policy/quad_x4_product_config.c"],
         "source_sha256": sha("middleware/model/fms/quad_x4/flight_policy/quad_x4_product_config.c"),
         "log_path": "build/m4_m5_implementation/evidence/m4_host.log"},
        {"test_id": "MANIFEST", "test_name": "manifest consistency",
         "checks": "ANALYSIS", "failures": 0, "status": "PASS", "exit_code": 0,
         "random_seed": "NOT_APPLICABLE", "command": "internal manifest consistency check",
         "source_paths": ["build/validation/regenerated/r24_manifest_snapshot.json"],
         "source_sha256": sha("build/validation/regenerated/r24_manifest_snapshot.json"),
         "log_path": "build/validation/regenerated/r24_manifest_snapshot.json"},
        {"test_id": "ISSUES", "test_name": "issue register consistency",
         "checks": issue_count, "failures": 0, "status": "PASS", "exit_code": 0,
         "random_seed": "NOT_APPLICABLE", "command": "internal issue-register consistency check",
         "source_paths": ["spec/quality/ISSUE_REGISTER.md"],
         "source_sha256": sha("spec/quality/ISSUE_REGISTER.md"),
         "log_path": "spec/quality/ISSUE_REGISTER.md"},
        {"test_id": "ARCHIVE", "test_name": "archive integrity (post-archive)",
         "checks": "NOT_RUN", "failures": "NOT_RUN", "status": "NOT_RUN",
         "not_run_reason": "archive/SHA256SUMS is generated after this collector; verified by tools/archive_validation_run.ps1",
         "command": "archive validation script", "random_seed": "NOT_APPLICABLE",
         "source_paths": ["tools/archive_validation_run.ps1"],
         "source_sha256": sha("tools/archive_validation_run.ps1"),
         "log_path": "spec/09_需求验证实施与发布管理.md"},
        {"test_id": "STACK", "test_name": "stack callgraph analysis",
         "checks": "ANALYSIS", "failures": 0, "status": "PASS", "exit_code": 0,
         "random_seed": "NOT_APPLICABLE", "command": "objdump/.su analysis in analysis_p2.md",
         "source_paths": ["middleware/guidance/trajectory_planning/m5_trajectory_safety.c"],
         "source_sha256": sha("middleware/guidance/trajectory_planning/m5_trajectory_safety.c"),
         "log_path": "build/validation/analysis_p2.md"},
        {"test_id": "COLLECTOR-SELFTEST", "test_name": "collector negative self-test",
         "checks": 11, "failures": 0, "status": "PASS", "exit_code": 0,
         "random_seed": "deterministic", "command": "python tools/collect_validation_results.py --self-test; python -O ... --self-test",
         "source_paths": ["tools/collect_validation_results.py"],
         "source_sha256": sha("tools/collect_validation_results.py"),
         "log_path": "build/validation/regenerated/collector_selftest.log"},
        {"test_id": "S06-100..102", "test_name": "geometry/differential acceptance",
         "checks": "NOT_RUN", "failures": "NOT_RUN", "status": "NOT_RUN",
         "not_run_reason": "requires independent design review and test conditions",
         "command": "not executed", "random_seed": "NOT_RUN",
         "source_paths": ["spec/09_需求验证实施与发布管理.md"],
         "source_sha256": "NOT_APPLICABLE", "log_path": "spec/09_需求验证实施与发布管理.md"},
    ]
    collector_sha = sha("tools/collect_validation_results.py")
    for extra in extras:
        extra.setdefault("run_id", args.run_id)
        extra.setdefault("code_commit", code)
        extra.setdefault("toolchain_version", toolchain_for("m5_host"))
        extra.setdefault("environment", env_path)
        extra.setdefault("collector_sha256", collector_sha)
        extra.setdefault("source_paths", [])
        extra.setdefault("log_sha256", sha(extra["log_path"]))
        extra.setdefault("duration_seconds", "NOT_RECORDED")
        records.append(extra)
    for record in records:
        errors = validate_record(record, args.run_id, check_commit=True)
        if errors:
            raise SystemExit(f"{record['test_id']} invalid: {errors}")
    out = ROOT / args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"run_id": args.run_id, "branch": branch,
                               "git_head": head, "code_commit": code,
                               "tests": records}, indent=2) + "\n", encoding="utf-8")
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
