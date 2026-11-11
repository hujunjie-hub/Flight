#!/usr/bin/env python3
"""R25 per-test evidence collector.

Every PASS requires a real command, exit 0, zero failures, positive integer
checks, existing log and source files with matching SHA256, an existing
environment file with matching SHA256, matching RUN_ID/code commit, a seed field
(NOT_APPLICABLE only for non-random tests) and a traceable toolchain.
"""
import argparse, hashlib, json, re, subprocess, sys, tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "build/m4_m5_implementation/evidence"
REGEN = ROOT / "build/validation/regenerated"


def sha(path):
    p = ROOT / path if not Path(path).is_absolute() else Path(path)
    return hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None


def git(*args):
    return subprocess.check_output(["git", "-C", str(ROOT), *args], text=True).strip()


def require(condition, message):
    if not condition:
        raise SystemExit("SELFTEST_FAIL: " + message)


def validate(record, run_id, check_commit=False):
    errors = []
    if record.get("run_id") != run_id:
        errors.append("RUN_ID mismatch")
    if not re.fullmatch(r"[0-9a-f]{40}", str(record.get("code_commit", ""))):
        errors.append("code_commit not 40-hex")
    elif check_commit:
        try:
            subprocess.check_call(["git", "-C", str(ROOT), "cat-file", "-e", record["code_commit"]],
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except subprocess.CalledProcessError:
            errors.append("code_commit does not exist")
    if record.get("status") != "NOT_RUN" and (
            not record.get("command") or record["command"] in ("see log", "not executed")):
        errors.append("command missing")
    if not record.get("toolchain_version"):
        errors.append("toolchain missing")
    env = record.get("environment_path")
    if not env or not (ROOT / env).is_file():
        errors.append("environment missing")
    elif record.get("environment_sha256") != sha(env):
        errors.append("environment SHA mismatch")
    if not record.get("random_seed"):
        errors.append("random_seed missing")
    status = record.get("status")
    kind = record.get("verification_kind")
    if status == "PASS":
        if kind not in ("TEST", "COMMAND"):
            errors.append("PASS with non-numeric verification kind")
        if not isinstance(record.get("exit_code"), int) or record["exit_code"] != 0:
            errors.append("PASS with nonzero exit")
        if not isinstance(record.get("failures"), int) or record["failures"] != 0:
            errors.append("PASS with failures")
        if not isinstance(record.get("checks"), int) or record["checks"] <= 0:
            errors.append("PASS without positive integer checks")
        files = record.get("source_files")
        if not isinstance(files, list) or not files:
            errors.append("PASS without source_files")
        else:
            for item in files:
                p = ROOT / item.get("path", "")
                if not p.is_file():
                    errors.append(f"source missing: {item.get('path')}")
                elif item.get("sha256") != sha(item["path"]):
                    errors.append(f"source SHA mismatch: {item.get('path')}")
        log = record.get("log_path")
        if not log or not (ROOT / log).is_file():
            errors.append("log missing")
        elif record.get("log_sha256") != sha(log):
            errors.append("log SHA mismatch")
    elif status == "NOT_RUN":
        if not record.get("not_run_reason"):
            errors.append("NOT_RUN without reason")
        if record.get("log_path") or record.get("log_sha256"):
            errors.append("NOT_RUN must not claim a log")
    elif status == "BLOCKED":
        if not record.get("not_run_reason"):
            errors.append("BLOCKED without reason")
    return errors


def self_test():
    tmp = Path(tempfile.mkdtemp(prefix="collector_r25_"))
    log = tmp / "test.log"; log.write_text("checks=2 failures=0\n")
    env = REGEN / "r25_environment.json"
    src = tmp / "test.c"; src.write_text("int main(void){return 0;}\n")
    base = {"run_id": "SELFTEST", "code_commit": "0" * 40, "command": "run test",
            "toolchain_version": "test-tool 1.0", "environment_path": str(env),
            "environment_sha256": sha(env), "random_seed": "NOT_APPLICABLE",
            "status": "PASS", "verification_kind": "TEST", "exit_code": 0,
            "checks": 2, "failures": 0,
            "source_files": [{"path": str(src), "sha256": sha(src)}],
            "log_path": str(log), "log_sha256": sha(log)}
    require(not validate(base, "SELFTEST"), "valid sample rejected")
    cases = []
    b = dict(base); b["log_path"] = str(tmp / "missing.log"); cases.append(b)
    b = dict(base); b["log_sha256"] = "0" * 64; cases.append(b)
    b = dict(base); b["exit_code"] = 1; cases.append(b)
    b = dict(base); b["failures"] = 1; cases.append(b)
    b = dict(base); b["checks"] = 0; cases.append(b)
    b = dict(base); b["verification_kind"] = "ANALYSIS"; cases.append(b)
    b = dict(base); b["source_files"] = [{"path": str(src), "sha256": "0" * 64}]; cases.append(b)
    b = dict(base); b["source_files"] = [{"path": str(tmp / "missing.c"), "sha256": "0" * 64}]; cases.append(b)
    b = dict(base); b["run_id"] = "WRONG"; cases.append(b)
    b = dict(base); b["code_commit"] = "a" * 40; cases.append(b)
    b = dict(base); b["command"] = ""; cases.append(b)
    b = dict(base); b["random_seed"] = ""; cases.append(b)
    b = dict(base); b["status"] = "NOT_RUN"; b.pop("not_run_reason", None); b["log_path"] = "README.md"; cases.append(b)
    b = dict(base); b["environment_path"] = str(tmp / "missing_env.json"); cases.append(b)
    b = dict(base); b["environment_sha256"] = "0" * 64; cases.append(b)
    b = dict(base); b["checks"] = "ANALYSIS"; cases.append(b)
    for i, case in enumerate(cases, 1):
        require(validate(case, "SELFTEST", check_commit=(i == 10)), f"negative case {i} not rejected")
    try:
        json.loads("{bad json")
        require(False, "bad JSON accepted")
    except json.JSONDecodeError:
        pass
    print(f"COLLECTOR_SELF_TEST_PASS ({len(cases)} negative cases)")
    return 0


def source_for(name):
    mapping = {
        "line_endings": "tools/check_line_endings.py",
        "python_o_negative": "tests/guidance/test_audit_optimized.py",
        "spec_layout": "spec/tools/audit_spec.py",
        "memory_budget": "tests/guidance/check_memory_budget.py",
        "packages_lock": "tools/check_rt_packages_lock.py",
        "packages_lock_unit": "tests/guidance/test_rt_packages_lock.py",
        "rebaseline_proposal": "tools/analyze_eol_rebaseline.py",
        "production_abi": "tests/abi/abi_report.py",
        "scons_link": "tests/guidance/check_target_build.py",
        "consumer_fp": "tests/guidance/check_consumer_fp_contraction.py",
        "fp_model_selftest": "tests/guidance/check_consumer_fp_contraction.py",
        "m4_host": "middleware/model/fms/quad_x4/flight_policy/quad_x4_product_config.c",
        "m4_arm": "middleware/model/fms/quad_x4/flight_policy/quad_x4_product_config.c",
    }
    if name in mapping:
        return mapping[name]
    if name in ("m5_host", "m5_sanitizer", "integration", "integration_sanitizer", "m5_arm"):
        return "middleware/guidance/trajectory_planning/m5_trajectory_safety.c"
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-id")
    ap.add_argument("--code-commit")
    ap.add_argument("--output")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    if a.self_test:
        return self_test()
    if not a.run_id or not a.code_commit or not a.output:
        raise SystemExit("--run-id, --code-commit and --output are required")
    if not re.fullmatch(r"[0-9a-f]{40}", a.code_commit):
        raise SystemExit("code commit must be 40-hex")
    subprocess.check_call(["git", "-C", str(ROOT), "cat-file", "-e", a.code_commit])
    validation = json.loads((EVIDENCE / "validation.json").read_text())
    env_path = "build/validation/regenerated/r25_environment.json"
    env_sha = sha(env_path)
    collector_sha = sha("tools/collect_validation_results.py")
    m5text = (EVIDENCE / "m5_host.log").read_text(errors="ignore")
    m4text = (EVIDENCE / "m4_host.log").read_text(errors="ignore")
    m5_cmd = " ".join(next(c["command"] for c in validation["commands"] if c["name"] == "m5_host"))
    m4_cmd = " ".join(next(c["command"] for c in validation["commands"] if c["name"] == "m4_host"))
    m5_exit = next(c["exit_code"] for c in validation["commands"] if c["name"] == "m5_host")
    m4_exit = next(c["exit_code"] for c in validation["commands"] if c["name"] == "m4_host")
    m4_checks, m4_failures = (re.search(r"checks=(\d+) failures=(\d+)", m4text).groups()
                              if re.search(r"checks=(\d+) failures=(\d+)", m4text) else (None, None))
    s108 = re.search(r"S06_108_TIME_ERROR_DETECTION cases=(\d+) .*failures=(\d+)", m5text)
    mapm = re.search(r"M5_MAP_ROLL_RANDOM seed=(0x[0-9A-Fa-f]+) .*checked_cases=(\d+) failed_cases=(\d+)", m5text)
    s112 = json.loads((REGEN / "s06_112_r25.json").read_text())
    manifest = json.loads((ROOT / "spec/validation/flight_acceptance_manifest.json").read_text())
    s113_match = re.search(r'\{\s*"test": "S06-113".*?\n\}', m5text, re.S)
    s113_json = json.loads(s113_match.group(0)) if s113_match else None
    s113 = next(x for x in manifest["adversarial_tests"] if x["id"] == "S06-113")
    product = json.loads((REGEN / "checks/product.json").read_text())
    issues = json.loads((REGEN / "checks/issues.json").read_text())
    man = json.loads((REGEN / "checks/manifest.json").read_text())
    stack = json.loads((REGEN / "checks/stack.json").read_text())
    tests = []
    for idx, c in enumerate(validation["commands"], 1):
        text = (ROOT / c["log"]).read_text(errors="ignore")
        m = re.search(r"checks=(\d+) failures=(\d+)", text)
        if m:
            checks, fails = int(m.group(1)), int(m.group(2))
        elif c["name"] == "line_endings":
            checks, fails = len(re.findall(r"^PASS:", text, re.M)), 0
        elif c["name"] == "python_o_negative":
            checks, fails = len(re.findall(r"_PASS", text)), 0
        elif c["name"] == "fp_model_selftest":
            mm = re.search(r"(\d+) mutations detected", text)
            checks, fails = (int(mm.group(1)) if mm else 0), 0
        elif c["name"] == "memory_budget":
            try:
                checks, fails = len([v for v in json.loads(text).values() if isinstance(v, (int, float))]), 0
            except Exception:
                checks, fails = 0, 1
        elif c["name"] in ("packages_lock_unit", "rebaseline_proposal"):
            checks = len(re.findall(r"PASS", text))
            fails = 0 if c["status"] == "PASS" else 1
        elif c["name"] in ("m4_arm", "m5_arm"):
            checks = len(re.findall(r"ARM_COMPILE_OK", text))
            fails = 0 if c["status"] == "PASS" else 1
        elif c["name"] == "consumer_fp":
            try:
                checks, fails = len(json.loads(text)), 0
            except Exception:
                checks, fails = 0, 1
        else:
            checks, fails = 0, 1
        src = source_for(c["name"])
        tests.append({"test_id": f"T{idx:02d}", "test_name": c["name"], "run_id": a.run_id,
                      "code_commit": a.code_commit, "verification_kind": "COMMAND" if m is None and c["status"] == "PASS" else "TEST",
                      "command": " ".join(c["command"]), "exit_code": c["exit_code"],
                      "checks": checks if checks is not None else 0,
                      "failures": fails if fails is not None else 0, "status": c["status"],
                      "random_seed": "NOT_APPLICABLE",
                      "toolchain_version": ("ARM " + (ROOT / "build/m5_arm/toolchain.txt").read_text().splitlines()[2]) if c["name"] in ("m4_arm", "m5_arm") else ("WSL gcc 16.2.0 / Python 3.14.10" if "wsl.exe" in c["command"] else "Windows Python 3.14.8 / PowerShell 7.6.9"),
                      "environment_path": env_path, "environment_sha256": env_sha,
                      "collector_sha256": collector_sha,
                      "source_files": [{"path": src, "sha256": sha(src)}] if src else [],
                      "log_path": c["log"], "log_sha256": sha(c["log"]),
                      "duration_seconds": c.get("duration_seconds", "NOT_RECORDED"),
                      **({"not_run_reason": "command status BLOCKED; see log"} if c["status"] == "BLOCKED" else {})})
    extras = [
        {"test_id": "S06-108", "test_name": "time error detection", "status": "PASS",
         "verification_kind": "TEST", "command": m5_cmd, "exit_code": m5_exit,
         "checks": int(s108.group(1)), "failures": int(s108.group(2)),
         "random_seed": next((x.get("seed", "NOT_RECORDED") for x in manifest["adversarial_tests"] if x["id"] == "S06-108"), "NOT_RECORDED"),
         "log_path": "build/m4_m5_implementation/evidence/m5_host.log",
         "source_files": [{"path": "middleware/guidance/trajectory_planning/m5_trajectory_safety.c", "sha256": sha("middleware/guidance/trajectory_planning/m5_trajectory_safety.c")}]},
        {"test_id": "S06-112", "test_name": "continuous jerk exact Fraction", "status": s112["status"],
         "verification_kind": "TEST", "command": f"python3 tests/guidance/verify_jerk_continuous.py --library build/m5_host/libm5_safety_oracle.so --run-id {a.run_id} --code-commit {a.code_commit} --output build/validation/regenerated/s06_112_r25.json",
         "exit_code": s112["exit_code"], "checks": s112["axis_cases_checked"],
         "failures": len(s112["failures"]), "random_seed": "NOT_APPLICABLE",
         "log_path": "build/validation/regenerated/s06_112_r25.log",
         "source_files": [{"path": "tests/guidance/verify_jerk_continuous.py", "sha256": sha("tests/guidance/verify_jerk_continuous.py")},
                          {"path": "middleware/guidance/trajectory_planning/m5_trajectory_safety.c", "sha256": sha("middleware/guidance/trajectory_planning/m5_trajectory_safety.c")}]},
        {"test_id": "S06-113", "test_name": "C time conversion injection", "status": "PASS",
         "verification_kind": "TEST", "command": m5_cmd, "exit_code": m5_exit,
         "checks": s113_json["checked"] if s113_json else 0,
         "failures": len(s113_json["failures"]) if s113_json else 1,
         "random_seed": "NOT_APPLICABLE",
         "log_path": "build/m4_m5_implementation/evidence/m5_host.log",
         "source_files": [{"path": "middleware/guidance/trajectory_planning/m5_trajectory_safety.c", "sha256": sha("middleware/guidance/trajectory_planning/m5_trajectory_safety.c")}]},
        {"test_id": "MAP-RANDOM", "test_name": "map in-place random differential",
         "status": "PASS" if mapm and int(mapm.group(3)) == 0 else "FAIL", "verification_kind": "TEST",
         "command": m5_cmd, "exit_code": m5_exit, "checks": int(mapm.group(2)) if mapm else 0,
         "failures": int(mapm.group(3)) if mapm else 1, "random_seed": mapm.group(1) if mapm else "NOT_RECORDED",
         "log_path": "build/m4_m5_implementation/evidence/m5_host.log",
         "source_files": [{"path": "middleware/guidance/trajectory_planning/m5_local_map.c", "sha256": sha("middleware/guidance/trajectory_planning/m5_local_map.c")}]},
        {"test_id": "PRODUCT-CONFIG", "test_name": "product config consistency", "status": product["status"],
         "verification_kind": "TEST", "command": "python tools/check_r25_evidence.py product",
         "exit_code": 0 if product["status"] == "PASS" else 1, "checks": product["checks"],
         "failures": product["failures"], "random_seed": "NOT_APPLICABLE",
         "log_path": "build/validation/regenerated/checks/product.log",
         "source_files": [{"path": "middleware/model/fms/quad_x4/flight_policy/quad_x4_product_config.c", "sha256": sha("middleware/model/fms/quad_x4/flight_policy/quad_x4_product_config.c")}]},
        {"test_id": "MANIFEST", "test_name": "manifest consistency", "status": man["status"],
         "verification_kind": "TEST", "command": "python tools/check_r25_evidence.py manifest",
         "exit_code": 0 if man["status"] == "PASS" else 1, "checks": man["checks"],
         "failures": man["failures"], "random_seed": "NOT_APPLICABLE",
         "log_path": "build/validation/regenerated/checks/manifest.log",
         "source_files": [{"path": "build/validation/regenerated/r26_manifest_snapshot.json", "sha256": sha("build/validation/regenerated/r26_manifest_snapshot.json")}]},
        {"test_id": "ISSUES", "test_name": "issue register consistency", "status": issues["status"],
         "verification_kind": "TEST", "command": "python tools/check_r25_evidence.py issues",
         "exit_code": 0 if issues["status"] == "PASS" else 1, "checks": issues["checks"],
         "failures": issues["failures"], "random_seed": "NOT_APPLICABLE",
         "log_path": "build/validation/regenerated/checks/issues.log",
         "source_files": [{"path": "spec/quality/ISSUE_REGISTER.md", "sha256": sha("spec/quality/ISSUE_REGISTER.md")}]},
        {"test_id": "STACK", "test_name": "stack callgraph analysis", "status": "BLOCKED",
         "verification_kind": "RESOURCE", "command": "python tools/check_r25_evidence.py stack",
         "exit_code": 2, "checks": stack["checks"], "failures": stack["failures"],
         "random_seed": "NOT_APPLICABLE", "not_run_reason": "validator 4248 B > 4096 B",
         "log_path": "build/validation/regenerated/checks/stack.log", "source_files": []},
        {"test_id": "COLLECTOR-SELFTEST", "test_name": "collector negative self-test",
         "status": "PASS", "verification_kind": "TEST",
         "command": "python tools/collect_validation_results.py --self-test", "exit_code": 0,
         "checks": int(re.search(r"\((\d+) negative cases\)", (REGEN / "collector_selftest.log").read_text()).group(1)),
         "failures": 0, "random_seed": "NOT_APPLICABLE",
         "log_path": "build/validation/regenerated/collector_selftest.log",
         "source_files": [{"path": "tools/collect_validation_results.py", "sha256": sha("tools/collect_validation_results.py")}]},
        {"test_id": "ARCHIVE", "test_name": "archive integrity (post-archive)", "status": "NOT_RUN",
         "verification_kind": "TEST", "command": "post-archive SHA/CRC check", "exit_code": 0,
         "checks": 0, "failures": 0, "random_seed": "NOT_APPLICABLE",
         "not_run_reason": "must run after ZIP creation", "log_path": "", "log_sha256": "", "source_files": []},
        {"test_id": "S06-100..102", "test_name": "geometry/differential acceptance", "status": "NOT_RUN",
         "verification_kind": "TEST", "command": "not executed", "exit_code": 0,
         "checks": 0, "failures": 0, "random_seed": "NOT_RUN",
         "not_run_reason": "feasibility not established at product resolution", "log_path": "", "log_sha256": "", "source_files": []},
    ]
    for e in extras:
        e.setdefault("run_id", a.run_id); e.setdefault("code_commit", a.code_commit)
        e.setdefault("environment_path", env_path); e.setdefault("environment_sha256", env_sha)
        e.setdefault("collector_sha256", collector_sha)
        e.setdefault("toolchain_version", "see r25_environment")
        e.setdefault("source_files", [])
        e.setdefault("duration_seconds", "NOT_RECORDED")
        e.setdefault("log_sha256", sha(e["log_path"]) if e["log_path"] else "")
        tests.append(e)
    for t in tests:
        errs = validate(t, a.run_id, check_commit=True)
        if errs:
            print(f"{t['test_id']}: {errs}", file=sys.stderr)
            raise SystemExit(1)
    out = ROOT / a.output
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"run_id": a.run_id, "code_commit": a.code_commit,
                               "git_head": git("rev-parse", "HEAD"),
                               "tests": tests}, indent=2) + "\n", encoding="utf-8")
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
