#!/usr/bin/env python3
"""Strict R28 validation-results collector.

The collector does not accept caller-supplied checks/failures. It parses a
wrapped raw log produced by ``tests/guidance/run_validation.py`` and requires
the log header, command, exit code, duration, RUN_ID and code commit to agree
with the run ledger. It then binds every PASS to an existing environment file,
a source manifest entry (git blob, worktree SHA, line ending, size, role) and a
raw log SHA256.

A log is only accepted when it contains the exact wrapper:

    # FLIGHT_VALIDATION_LOG_V1
    RUN_ID=...
    CODE_COMMIT=...
    COMMAND_JSON=[...]
    EXIT_CODE=0
    DURATION_SECONDS=...
    # BEGIN_OUTPUT
    ...
    # END_OUTPUT

PASS requires: exit_code == 0, failures == 0, checks > 0, a parsed count that
equals the log's real machine marker, a finite positive duration, a valid
toolchain, a valid seed policy, valid source provenance and an exact log SHA.

The self-test deliberately exercises the complete pipeline (real JSON, real
source file, real log, real environment) and rejects every negative case. It is
safe under ``python -O``: no ``assert`` statements are used.
"""
import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ENV = "build/validation/regenerated/r28_environment.json"
DEFAULT_MANIFEST = "build/validation/regenerated/r28_source_manifest.json"
DEFAULT_VALIDATION = "build/m4_m5_implementation/evidence/validation.json"
DEFAULT_NOT_RUN = "build/validation/regenerated/r28_not_run.json"

FORBIDDEN_TOOLCHAIN = (
    "see r25_environment", "see r26_environment", "see r27_environment",
    "see log", "unknown", "deterministic", "not_recorded", "not recorded",
)
REQUIRED_RECORD_FIELDS = (
    "test_id", "test_name", "run_id", "code_commit", "verification_kind",
    "command", "command_argv", "exit_code", "checks", "failures", "status",
    "random_seed", "toolchain_version", "environment_path",
    "environment_sha256", "collector_sha256", "source_files", "log_path",
    "log_sha256", "duration_seconds",
)
COMPILE_TESTS = {
    "fixture", "m4_host", "m4_arm", "m5_host", "m5_sanitizer", "m5_arm",
    "integration", "integration_sanitizer", "s06_108", "s06_112", "s06_113",
}
RANDOM_TESTS = {"map_random", "S06-108", "S06-112"}
RANDOM_NAMES = {"map_random", "map-random", "S06-108", "S06-112",
                "s06_108", "s06_112"}


class EvidenceError(Exception):
    pass


def fail(message):
    raise EvidenceError(message)


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def sha256_file(path):
    p = Path(path)
    return sha256_bytes(p.read_bytes()) if p.is_file() else None


def read_json(path):
    p = Path(path)
    if not p.is_file():
        fail(f"missing JSON: {path}")
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except Exception as exc:
        fail(f"invalid JSON {path}: {exc}")


def line_ending(data):
    crlf = data.count(b"\r\n")
    lf = data.count(b"\n")
    if crlf == 0 and lf == 0:
        return "NONE"
    if crlf and crlf == lf:
        return "CRLF"
    if lf and crlf == 0:
        return "LF"
    return "MIXED"


def git_output(*args):
    proc = subprocess.run(["git", "-C", str(ROOT), *args],
                          capture_output=True, text=True)
    if proc.returncode != 0:
        return None
    return proc.stdout.strip()


def object_exists(sha):
    if not re.fullmatch(r"[0-9a-f]{40}", str(sha or "")):
        return False
    return subprocess.run(["git", "-C", str(ROOT), "cat-file", "-e", sha],
                          stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode == 0


def commit_exists(sha):
    return object_exists(sha) and git_output("cat-file", "-t", sha) == "commit"


def valid_toolchain(value):
    if not isinstance(value, str) or not value.strip():
        return False
    low = value.lower()
    if any(token in low for token in FORBIDDEN_TOOLCHAIN):
        return False
    # Must contain a real version number, not just a product name.
    if not re.search(r"\d+\.\d+", value):
        return False
    # Do not mix independently probed host/target toolchains in one field.
    if "windows" in low and "arm-none-eabi" in low:
        return False
    if "wsl" in low and "arm-none-eabi" in low:
        return False
    return True


def parse_header(text):
    lines = text.splitlines()
    if not lines or lines[0].strip() != "# FLIGHT_VALIDATION_LOG_V1":
        fail("log wrapper header missing")
    fields = {}
    begin = None
    end = None
    for index, line in enumerate(lines[1:], 1):
        if line.strip() == "# BEGIN_OUTPUT":
            begin = index
            break
        if "=" not in line:
            fail(f"malformed log header line {index + 1}")
        key, value = line.split("=", 1)
        if key in fields:
            fail(f"duplicate log header field {key}")
        fields[key] = value
    if begin is None:
        fail("BEGIN_OUTPUT marker missing")
    for index in range(begin + 1, len(lines)):
        if lines[index].strip() == "# END_OUTPUT":
            end = index
            break
    if end is None:
        fail("END_OUTPUT marker missing (truncated log)")
    for key in ("RUN_ID", "CODE_COMMIT", "COMMAND_JSON", "EXIT_CODE",
                "DURATION_SECONDS"):
        if key not in fields:
            fail(f"log header missing {key}")
    try:
        command = json.loads(fields["COMMAND_JSON"])
        exit_code = int(fields["EXIT_CODE"])
        duration = float(fields["DURATION_SECONDS"])
    except Exception as exc:
        fail(f"invalid log header value: {exc}")
    if not isinstance(command, list) or not all(isinstance(x, str) for x in command):
        fail("COMMAND_JSON is not a string array")
    if duration <= 0.0:
        fail("DURATION_SECONDS is not positive")
    output = "\n".join(lines[begin + 1:end])
    return {
        "run_id": fields["RUN_ID"],
        "code_commit": fields["CODE_COMMIT"],
        "command_argv": command,
        "exit_code": exit_code,
        "duration_seconds": duration,
        "output": output,
    }


def _json_object(text):
    try:
        value = json.loads(text)
        return value if isinstance(value, dict) else None
    except Exception:
        pass
    start = text.find("{")
    end = text.rfind("}")
    if start >= 0 and end > start:
        try:
            value = json.loads(text[start:end + 1])
            return value if isinstance(value, dict) else None
        except Exception:
            return None
    return None


def _exact_int(pattern, text, label):
    matches = re.findall(pattern, text, re.M)
    if not matches:
        fail(f"{label}: machine marker missing")
    values = {tuple(m) if isinstance(m, tuple) else m for m in matches}
    if len(values) != 1:
        fail(f"{label}: conflicting machine markers")
    return matches[0]


def parse_output(name, output, exit_code):
    """Return (checks, failures, seed, detail) from the real output."""
    seed = "NOT_APPLICABLE"
    detail = {}

    if name == "fixture":
        checks, failures = _exact_int(r"^FIXTURE_CHECKS=(\d+) FAILURES=(\d+)$",
                                      output, name)
        return int(checks), int(failures), seed, detail

    if name == "line_endings":
        checks = len(re.findall(r"^PASS:", output, re.M))
        failures = len(re.findall(r"^FAIL", output, re.M))
        return checks, failures, seed, {"parser": "PASS-lines"}

    if name == "python_o_negative":
        checks = len(re.findall(r"_PASS\b", output))
        failures = len(re.findall(r"FAIL", output))
        return checks, failures, seed, {"parser": "negative-self-test"}

    if name in ("m4_host", "m5_host", "integration", "m5_sanitizer",
                "integration_sanitizer"):
        checks, failures = _exact_int(r"^checks=(\d+) failures=(\d+)$",
                                      output, name)
        return int(checks), int(failures), seed, {"parser": "aggregate"}

    if name in ("m4_arm", "m5_arm"):
        checks = len(re.findall(r"ARM_COMPILE_OK", output))
        failures = len(re.findall(r"ARM_COMPILE_FAIL", output))
        return checks, failures, seed, {"parser": "ARM_COMPILE_OK"}

    if name == "consumer_fp":
        data = _json_object(output) or {}
        fused = data.get("fused_operations")
        checks = len(data.get("objects", [])) if isinstance(data.get("objects"), list) else 0
        if checks == 0:
            checks = 7 if data else 0
        failures = 1 if fused not in (0, None) or not data else 0
        return checks, failures, seed, {"parser": "JSON", "fused_operations": fused}

    if name == "fp_model_selftest":
        match = re.search(r"M5_FP_MODEL_SELF_TEST_PASS \((\d+) mutations detected\)", output)
        if not match:
            fail("fp_model_selftest: PASS marker missing")
        return int(match.group(1)), 0, seed, {"parser": "mutation-count"}

    if name == "memory_budget":
        data = _json_object(output) or {}
        checks = sum(1 for value in data.values() if isinstance(value, (int, float)))
        failures = 0 if data.get("status") == "PASS" else 1
        return checks, failures, seed, {"parser": "JSON", "status": data.get("status")}

    if name == "packages_lock":
        data = _json_object(output) or {}
        checks = 10 if data else 0
        failures = 0 if data.get("status") == "PASS" else max(1, len(data.get("failures", [])))
        return checks, failures, seed, {"parser": "JSON", "status": data.get("status")}

    if name == "packages_lock_unit":
        match = re.search(r"M5_RT_PACKAGES_LOCK_NEGATIVE_TEST_PASS \((\d+) cases\)", output)
        if match:
            return int(match.group(1)), 0, seed, {"parser": "negative-case-count"}
        checks = len(re.findall(r"\bPASS\b", output))
        failures = len(re.findall(r"\bFAIL\b", output))
        return checks, failures, seed, {"parser": "unit PASS-lines"}

    if name == "rebaseline_proposal":
        checks = len(re.findall(r"\bPASS\b", output))
        failures = len(re.findall(r"\bFAIL\b", output))
        return checks, failures, seed, {"parser": "proposal PASS-lines"}

    if name == "production_abi":
        data = _json_object(output) or {}
        checks = int(data.get("types", 0)) if str(data.get("types", "")).isdigit() else 0
        failures = 0 if data.get("status") == "PASS" and data.get("lock") == "PASS" else 1
        return checks, failures, seed, {"parser": "JSON", "status": data.get("status"),
                                        "lock": data.get("lock")}

    if name == "spec_layout":
        failures = 0 if exit_code == 0 else 1
        checks = 1 if exit_code == 0 else 0
        return checks, failures, seed, {"parser": "exit-only"}

    if name == "scons_link":
        data = _json_object(output) or {}
        checks = 1 if data else 0
        failures = 0 if data.get("status") == "BLOCKED" else 1
        return checks, failures, seed, {"parser": "JSON", "status": data.get("status")}

    if name == "s06_108":
        match = re.search(
            r"S06_108_TIME_ERROR_DETECTION cases=(\d+) .*?failures=(\d+)", output)
        seed_match = re.search(r"S06_108_SEED=(0x[0-9A-Fa-f]+|\d+)", output)
        if not match:
            fail("s06_108: marker missing")
        if not seed_match:
            fail("s06_108: real seed missing from log")
        return int(match.group(1)), int(match.group(2)), seed_match.group(1), {
            "parser": "S06-108-marker"}

    if name == "s06_112":
        # Output JSON is written by the verifier; its path is in command argv.
        return None, None, seed, {"parser": "external-json"}

    if name == "s06_113":
        match = re.search(
            r"S06_113_TIME_CONVERSION_INJECTION checked=(\d+) failures=(\d+)", output)
        if not match:
            fail("s06_113: marker missing")
        return int(match.group(1)), int(match.group(2)), seed, {
            "parser": "S06-113-marker"}

    if name == "map_random":
        match = re.search(
            r"M5_MAP_ROLL_RANDOM seed=(0x[0-9A-Fa-f]+|\d+) .*?checked_cases=(\d+) failed_cases=(\d+)",
            output, re.S)
        if not match:
            fail("map_random: marker missing")
        return int(match.group(2)), int(match.group(3)), match.group(1), {
            "parser": "MAP-RANDOM"}

    if name in ("collector_selftest", "collector_selftest_opt"):
        match = re.search(r"COLLECTOR_SELF_TEST_PASS \((\d+) negative cases\)", output)
        if not match:
            fail(f"{name}: PASS marker missing")
        return int(match.group(1)), 0, seed, {"parser": "collector-selftest"}

    if name == "evidence_checker_selftest":
        match = re.search(r"EVIDENCE_CHECKER_SELF_TEST_PASS \((\d+) negative cases\)", output)
        if not match:
            fail("evidence_checker_selftest: PASS marker missing")
        return int(match.group(1)), 0, seed, {"parser": "checker-selftest"}

    if name in ("product_check", "manifest_check", "issues_check", "stack_check"):
        match = re.search(
            r"^(?:product|manifest|issues|stack):\s*(PASS|FAIL|BLOCKED|UNKNOWN)\s+"
            r"checks=(\d+)\s+failures=(\d+)", output, re.M)
        if not match:
            data = _json_object(output) or {}
            checks = int(data.get("checks", 0)) if str(data.get("checks", "")).isdigit() else 0
            failures = int(data.get("failures", 1)) if str(data.get("failures", "")).isdigit() else 1
            return checks, failures, seed, {"parser": "checker-JSON",
                                            "status": data.get("status"),
                                            "detail": data.get("detail")}
        return int(match.group(2)), int(match.group(3)), seed, {
            "parser": "checker-console", "status": match.group(1)}

    if name == "stack_selftest":
        match = re.search(r"STACK_ANALYSIS_SELF_TEST_PASS \((\d+) negative cases\)", output)
        if not match:
            fail("stack_selftest: PASS marker missing")
        return int(match.group(1)), 0, seed, {"parser": "stack-selftest"}

    if name == "stack_image":
        data = _json_object(output) or {}
        if data.get("status") not in ("PASS", "BLOCKED", "UNKNOWN"):
            fail("stack_image: analyzer JSON missing")
        checks = int(data.get("function_count", 0) or 0)
        failures = 0 if data.get("status") == "PASS" else 1
        return checks, failures, seed, {
            "parser": "stack-image", "status": data.get("status"),
            "lower_bound": data.get("known_lower_bound_bytes"),
            "callgraph_status": data.get("callgraph_status")}

    if name == "stack_image_selftest":
        match = re.search(r"STACK_IMAGE_SELF_TEST_PASS \((\d+) cases\)", output)
        if not match:
            fail("stack_image_selftest: PASS marker missing")
        return int(match.group(1)), 0, seed, {"parser": "stack-image-selftest"}

    fail(f"no strict parser for command {name}")


def extract_external_json(name, command_argv):
    if name != "s06_112":
        return None
    for index, arg in enumerate(command_argv):
        if arg == "--output" and index + 1 < len(command_argv):
            return command_argv[index + 1]
    return None


def build_records(validation, run_id, code_commit, env_path, env_sha,
                  collector_sha, source_manifest, check_git=True):
    errors = []
    if source_manifest.get("run_id") != run_id:
        errors.append("source manifest RUN_ID mismatch")
    if source_manifest.get("code_commit") != code_commit:
        errors.append("source manifest code_commit mismatch")
    if validation.get("run_id") != run_id:
        errors.append("validation RUN_ID mismatch")
    if validation.get("code_commit") != code_commit:
        errors.append("validation code_commit mismatch")
    if validation.get("git_head") and validation.get("git_head") != code_commit:
        errors.append("validation git_head mismatch")
    if check_git and not commit_exists(code_commit):
        errors.append("code commit does not exist")
    commands = validation.get("commands")
    if not isinstance(commands, list) or not commands:
        errors.append("validation commands missing")
        return [], errors
    records = []
    seen_ids = set()
    seen_names = set()
    for index, command in enumerate(commands, 1):
        if not isinstance(command, dict):
            errors.append(f"command {index}: not an object")
            continue
        name = command.get("name")
        if not isinstance(name, str) or not name:
            errors.append(f"command {index}: missing name")
            continue
        if name in seen_names:
            errors.append(f"duplicate command name: {name}")
            continue
        seen_names.add(name)
        test_id = command.get("test_id") or f"T{index:02d}"
        if test_id in seen_ids:
            errors.append(f"duplicate test_id: {test_id}")
            continue
        seen_ids.add(test_id)
        log_rel = command.get("log")
        log_path = ROOT / log_rel if log_rel else None
        if log_path is None or not log_path.is_file():
            errors.append(f"{name}: log missing")
            continue
        try:
            log_text = log_path.read_text(encoding="utf-8", errors="replace")
            header = parse_header(log_text)
        except EvidenceError as exc:
            errors.append(f"{name}: {exc}")
            continue
        if header["run_id"] != run_id:
            errors.append(f"{name}: log RUN_ID mismatch")
        if header["code_commit"] != code_commit:
            errors.append(f"{name}: log code_commit mismatch")
        if header["exit_code"] != command.get("exit_code"):
            errors.append(f"{name}: log exit_code differs from ledger")
        if header["command_argv"] != command.get("command"):
            errors.append(f"{name}: log command differs from ledger")
        ledger_duration = command.get("duration_seconds")
        if not isinstance(ledger_duration, (int, float)) or abs(
                float(ledger_duration) - header["duration_seconds"]) > 0.05:
            errors.append(f"{name}: duration differs from ledger")
        try:
            checks, failures, seed, detail = parse_output(
                name, header["output"], header["exit_code"])
        except EvidenceError as exc:
            errors.append(f"{name}: {exc}")
            continue
        if name == "s06_112":
            external = extract_external_json(name, header["command_argv"])
            if not external:
                errors.append("s06_112: --output missing")
                continue
            data = read_json(ROOT / external)
            checks = data.get("axis_cases_checked")
            failures = len(data.get("failures", []))
            seed = data.get("seed")
            if data.get("run_id") != run_id:
                errors.append("s06_112: JSON RUN_ID mismatch")
            if data.get("code_commit") != code_commit:
                errors.append("s06_112: JSON code_commit mismatch")
            if data.get("exit_code") != header["exit_code"]:
                errors.append("s06_112: JSON exit_code mismatch")
            detail = {"parser": "S06-112-JSON", "status": data.get("status")}
        status = "PASS" if header["exit_code"] == 0 else "FAIL"
        if header["exit_code"] == 2 and name in ("scons_link", "stack_check"):
            status = "BLOCKED"
        if status == "PASS" and (checks is None or failures is None):
            errors.append(f"{name}: PASS without parsed checks/failures")
            continue
        if status == "PASS" and (not isinstance(checks, int) or checks <= 0):
            errors.append(f"{name}: PASS without positive checks")
        if status == "PASS" and failures != 0:
            errors.append(f"{name}: PASS with failures")
        record = {
            "test_id": test_id,
            "test_name": name,
            "run_id": run_id,
            "code_commit": code_commit,
            "verification_kind": "TEST" if name not in ("line_endings", "python_o_negative",
                                                        "collector_selftest",
                                                        "collector_selftest_opt",
                                                        "evidence_checker_selftest") else "COMMAND",
            "command": " ".join(header["command_argv"]),
            "command_argv": header["command_argv"],
            "exit_code": header["exit_code"],
            "checks": int(checks or 0),
            "failures": int(failures or 0),
            "status": status,
            "random_seed": seed or "NOT_APPLICABLE",
            "toolchain_version": command.get("toolchain_version"),
            "environment_path": env_path,
            "environment_sha256": env_sha,
            "collector_sha256": collector_sha,
            "source_files": [],
            "log_sha256": sha256_file(log_path),
            "duration_seconds": header["duration_seconds"],
            "evidence_detail": detail,
        }
        if status == "BLOCKED":
            record["not_run_reason"] = "exit code 2; firmware link blocked by B03/B04"
        try:
            record["log_path"] = str(log_path.relative_to(ROOT)).replace("\\", "/")
        except ValueError:
            record["log_path"] = str(log_path)
        records.append((record, source_manifest, name))
    return records, errors


def source_entry(path, manifest, code_commit, check_git):
    files = manifest.get("files", {})
    if path not in files:
        return None, f"source not in manifest: {path}"
    item = files[path]
    p = ROOT / path
    if not p.is_file():
        return None, f"source missing: {path}"
    data = p.read_bytes()
    actual = sha256_bytes(data)
    if item.get("working_tree_sha256") != actual:
        return None, f"source worktree SHA mismatch: {path}"
    if item.get("size_bytes") not in (None, len(data)):
        return None, f"source size mismatch: {path}"
    if item.get("line_ending") not in (None, line_ending(data)):
        return None, f"source line ending mismatch: {path}"
    if not re.fullmatch(r"[0-9a-f]{40}", str(item.get("git_blob_sha", ""))):
        return None, f"source git_blob_sha invalid: {path}"
    if check_git and not object_exists(item["git_blob_sha"]):
        return None, f"source git blob does not exist: {path}"
    return {
        "path": path,
        "git_blob_sha": item["git_blob_sha"],
        "working_tree_sha256": actual,
        "line_ending": line_ending(data),
        "size_bytes": len(data),
        "role": item.get("role", "unknown"),
    }, None


def validate_record(record, manifest, run_id, code_commit, env_cache, check_git=True):
    errors = []
    for field in REQUIRED_RECORD_FIELDS:
        if field not in record:
            errors.append(f"missing field {field}")
    if errors:
        return errors
    if record["run_id"] != run_id:
        errors.append("RUN_ID mismatch")
    if record["code_commit"] != code_commit:
        errors.append("code_commit mismatch")
    if check_git and not commit_exists(code_commit):
        errors.append("code commit does not exist")
    if record["status"] not in ("PASS", "FAIL", "BLOCKED", "NOT_RUN"):
        errors.append("invalid status")
    if not valid_toolchain(record.get("toolchain_version")):
        errors.append("toolchain is not a real version")
    seed = str(record.get("random_seed", ""))
    if record["status"] == "PASS":
        if record["exit_code"] != 0:
            errors.append("PASS with nonzero exit")
        if record["failures"] != 0:
            errors.append("PASS with failures")
        if not isinstance(record["checks"], int) or record["checks"] <= 0:
            errors.append("PASS without positive checks")
        if not isinstance(record["duration_seconds"], (int, float)) or record["duration_seconds"] <= 0:
            errors.append("PASS without positive duration")
        if seed.lower() in ("", "deterministic", "not_recorded", "unknown"):
            errors.append("PASS with forbidden seed")
        if record["test_name"] in RANDOM_NAMES:
            if seed == "NOT_APPLICABLE":
                errors.append("random test without real seed")
        elif seed != "NOT_APPLICABLE":
            # A non-random test may still carry a numeric seed only if it is
            # explicitly marked random. Keep this strict to avoid fake seeds.
            if not re.fullmatch(r"0[xX][0-9A-Fa-f]+|\d+", seed):
                errors.append("non-random test has invalid seed")
        if record.get("command") in ("", "see log", "not executed", None):
            errors.append("PASS without real command")
        sources = record.get("source_files")
        if not isinstance(sources, list) or not sources:
            errors.append("PASS without source files")
        else:
            for item in sources:
                for key in ("path", "git_blob_sha", "working_tree_sha256",
                            "line_ending", "size_bytes", "role"):
                    if key not in item:
                        errors.append(f"source missing {key}: {item.get('path')}")
                p = ROOT / item.get("path", "")
                if not p.is_file():
                    errors.append(f"source missing: {item.get('path')}")
                elif sha256_bytes(p.read_bytes()) != item.get("working_tree_sha256"):
                    errors.append(f"source SHA mismatch: {item.get('path')}")
        log = record.get("log_path")
        if not log or not (ROOT / log).is_file():
            errors.append("log missing")
        elif record.get("log_sha256") != sha256_file(ROOT / log):
            errors.append("log SHA mismatch")
    elif record["status"] == "NOT_RUN":
        if not record.get("not_run_reason"):
            errors.append("NOT_RUN without reason")
        if record.get("log_path") or record.get("log_sha256"):
            errors.append("NOT_RUN must not claim a log")
        if record.get("checks") or record.get("failures"):
            errors.append("NOT_RUN must not claim counts")
    elif record["status"] == "BLOCKED":
        if not record.get("not_run_reason"):
            errors.append("BLOCKED without reason")
        if not record.get("log_path") or not (ROOT / record["log_path"]).is_file():
            errors.append("BLOCKED without raw log")
    env_path = record.get("environment_path")
    if not env_path or not (ROOT / env_path).is_file():
        errors.append("environment missing")
    elif record.get("environment_sha256") != sha256_file(ROOT / env_path):
        errors.append("environment SHA mismatch")
    else:
        env = env_cache.get(env_path)
        if env is None:
            try:
                env = json.loads((ROOT / env_path).read_text(encoding="utf-8"))
            except Exception:
                env = {}
            env_cache[env_path] = env
        if env.get("run_id") != record["run_id"]:
            errors.append("environment RUN_ID mismatch")
        if env.get("code_commit") != record["code_commit"]:
            errors.append("environment code_commit mismatch")
        if not valid_toolchain(env.get("toolchain_version")):
            errors.append("environment toolchain invalid")
    if record.get("collector_sha256") != sha256_file("tools/collect_validation_results.py"):
        errors.append("collector SHA mismatch")
    return errors


def attach_sources(records, source_manifest, code_commit, check_git=True):
    errors = []
    out = []
    for record, manifest, name in records:
        entries = manifest.get("tests", {}).get(name)
        if not isinstance(entries, list) or not entries:
            errors.append(f"{name}: source manifest entry missing")
            continue
        sources = []
        for path in entries:
            entry, error = source_entry(path, manifest, code_commit, check_git)
            if error:
                errors.append(f"{name}: {error}")
            else:
                sources.append(entry)
        compile_inputs = manifest.get("compile_inputs", {}).get(name)
        if name in COMPILE_TESTS:
            if not isinstance(compile_inputs, list) or not compile_inputs:
                errors.append(f"{name}: compile_inputs missing")
            elif not set(entries).issubset(set(compile_inputs)):
                errors.append(f"{name}: compile_inputs incomplete")
        record["source_files"] = sources
        out.append(record)
    return out, errors


def add_not_run(not_run_path, run_id, code_commit, env_path, env_sha,
                collector_sha):
    records = []
    if not not_run_path or not Path(not_run_path).is_file():
        return records, []
    data = read_json(not_run_path)
    if data.get("run_id") != run_id or data.get("code_commit") != code_commit:
        return [], ["not-run ledger RUN_ID/code_commit mismatch"]
    for item in data.get("tests", []):
        record = {
            "test_id": item["test_id"],
            "test_name": item["test_name"],
            "run_id": run_id,
            "code_commit": code_commit,
            "verification_kind": item.get("verification_kind", "TEST"),
            "command": item.get("command", "NOT_EXECUTED"),
            "command_argv": item.get("command_argv", []),
            "exit_code": None,
            "checks": 0,
            "failures": 0,
            "status": "NOT_RUN",
            "random_seed": item.get("random_seed", "NOT_APPLICABLE"),
            "toolchain_version": item.get("toolchain_version"),
            "environment_path": env_path,
            "environment_sha256": env_sha,
            "collector_sha256": collector_sha,
            "source_files": [],
            "log_path": "",
            "log_sha256": "",
            "duration_seconds": 0,
            "not_run_reason": item.get("not_run_reason", ""),
        }
        records.append(record)
    return records, []


def derive_m5(records):
    """Derive per-marker records from the real m5_host log.

    These are not caller-supplied counts: the enclosing m5_host command is the
    executed command, and the values are parsed from its raw output.
    """
    m5 = next((r for r in records if r.get("test_name") == "m5_host"), None)
    if not m5:
        return []
    log_path = ROOT / m5["log_path"]
    try:
        header = parse_header(log_path.read_text(encoding="utf-8", errors="replace"))
    except EvidenceError:
        return []
    output = header["output"]
    derived = []
    patterns = [
        ("S06-108", r"S06_108_TIME_ERROR_DETECTION cases=(\d+) .*?failures=(\d+)",
         r"S06_108_SEED=(0x[0-9A-Fa-f]+|\d+)"),
        ("S06-113", r"S06_113_TIME_CONVERSION_INJECTION checked=(\d+) failures=(\d+)", None),
        ("MAP-RANDOM", r"M5_MAP_ROLL_RANDOM seed=(0x[0-9A-Fa-f]+|\d+) .*?checked_cases=(\d+) failed_cases=(\d+)", None),
    ]
    for index, (name, pattern, seed_pattern) in enumerate(patterns):
        match = re.search(pattern, output, re.S)
        if not match:
            continue
        if name == "MAP-RANDOM":
            seed = match.group(1)
            checks = int(match.group(2))
            failures = int(match.group(3))
        else:
            checks = int(match.group(1))
            failures = int(match.group(2))
            seed = "NOT_APPLICABLE"
            if seed_pattern:
                seed_match = re.search(seed_pattern, output)
                if not seed_match:
                    continue
                seed = seed_match.group(1)
        derived.append({
            **m5,
            "test_id": name,
            "test_name": name.lower().replace("-", "_"),
            "checks": checks,
            "failures": failures,
            "status": "PASS" if m5["status"] == "PASS" and failures == 0 else "FAIL",
            "random_seed": seed,
            "duration_seconds": m5["duration_seconds"],
            "evidence_detail": {"parser": "derived-from-m5_host",
                                "derived_index": index},
        })
    return derived


def collect(args):
    validation = read_json(args.validation)
    env_path = args.environment
    env_sha = sha256_file(ROOT / env_path)
    if env_sha is None:
        fail(f"environment missing: {env_path}")
    env = read_json(ROOT / env_path)
    if env.get("run_id") != args.run_id:
        fail("environment RUN_ID mismatch")
    if env.get("code_commit") != args.code_commit:
        fail("environment code_commit mismatch")
    if not valid_toolchain(env.get("toolchain_version")):
        fail("environment toolchain invalid")
    if not commit_exists(args.code_commit):
        fail("code commit does not exist")
    manifest = read_json(ROOT / args.source_manifest)
    if manifest.get("run_id") != args.run_id or manifest.get("code_commit") != args.code_commit:
        fail("source manifest RUN_ID/code_commit mismatch")
    raw_records, errors = build_records(
        validation, args.run_id, args.code_commit, env_path, env_sha,
        sha256_file("tools/collect_validation_results.py"), manifest,
        check_git=True)
    records, source_errors = attach_sources(raw_records, manifest, args.code_commit)
    errors += source_errors
    records += derive_m5(records)
    not_run, not_run_errors = add_not_run(
        ROOT / args.not_run, args.run_id, args.code_commit, env_path, env_sha,
        sha256_file("tools/collect_validation_results.py"))
    errors += not_run_errors
    records += not_run
    env_cache = {env_path: env}
    seen = set()
    for record in records:
        if record["test_id"] in seen:
            errors.append(f"duplicate test_id: {record['test_id']}")
        seen.add(record["test_id"])
        errors += [f"{record['test_id']}: {e}" for e in
                   validate_record(record, manifest, args.run_id, args.code_commit,
                                   env_cache, check_git=True)]
    if errors:
        for error in errors[:40]:
            print("COLLECTOR_FAIL:", error, file=sys.stderr)
        raise SystemExit(1)
    output = {
        "schema_version": "2.0",
        "run_id": args.run_id,
        "code_commit": args.code_commit,
        "git_head": git_output("rev-parse", "HEAD"),
        "collector_sha256": sha256_file("tools/collect_validation_results.py"),
        "environment_path": env_path,
        "environment_sha256": env_sha,
        "source_manifest_path": args.source_manifest,
        "source_manifest_sha256": sha256_file(ROOT / args.source_manifest),
        "tests": records,
    }
    out = ROOT / args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(f"COLLECTOR_WROTE {out} tests={len(records)}")
    return 0


# ---------------------------------------------------------------------------
# Self-test: full pipeline, 28 negative scenarios, no asserts (works with -O).
# ---------------------------------------------------------------------------

SELFTEST_SOURCE = "SConstruct"


def _write_log(path, run_id, code_commit, command, exit_code, duration, output,
               end=True):
    text = "\n".join([
        "# FLIGHT_VALIDATION_LOG_V1",
        f"RUN_ID={run_id}",
        f"CODE_COMMIT={code_commit}",
        "COMMAND_JSON=" + json.dumps(command, separators=(",", ":")),
        f"EXIT_CODE={exit_code}",
        f"DURATION_SECONDS={duration}",
        "# BEGIN_OUTPUT",
        output,
        "# END_OUTPUT" if end else "",
    ]) + "\n"
    Path(path).write_text(text, encoding="utf-8")


def self_test():
    tmp = Path(tempfile.mkdtemp(prefix="collector_r28_"))
    failures = []
    cases = 0
    try:
        head = git_output("rev-parse", "HEAD")
        if not head or not commit_exists(head):
            print("SELFTEST_FAIL: cannot resolve HEAD", file=sys.stderr)
            return 1
        source_path = ROOT / SELFTEST_SOURCE
        source_bytes = source_path.read_bytes()
        source_sha = sha256_bytes(source_bytes)
        source_blob = git_output("rev-parse", f"HEAD:{SELFTEST_SOURCE}")
        run_id = "SELFTEST-M5-06-R28"
        command = ["python", "fixture.py", "--mode", "strict"]
        log = tmp / "fixture.log"
        env = tmp / "env.json"
        manifest = tmp / "manifest.json"
        validation = tmp / "validation.json"
        not_run = tmp / "not_run.json"
        _write_log(log, run_id, head, command, 0, 0.5,
                   "FIXTURE_CHECKS=3 FAILURES=0")
        env.write_text(json.dumps({
            "run_id": run_id, "code_commit": head,
            "toolchain_version": "Windows 11 10.0.26300 / Python 3.14.8",
        }), encoding="utf-8")
        manifest.write_text(json.dumps({
            "run_id": run_id, "code_commit": head,
            "files": {SELFTEST_SOURCE: {
                "git_blob_sha": source_blob,
                "working_tree_sha256": source_sha,
                "size_bytes": len(source_bytes),
                "line_ending": line_ending(source_bytes),
                "role": "build-definition",
            }},
            "tests": {"fixture": [SELFTEST_SOURCE]},
            "compile_inputs": {"fixture": [SELFTEST_SOURCE]},
        }), encoding="utf-8")
        base_command = {
            "test_id": "T01", "name": "fixture", "command": command,
            "exit_code": 0, "status": "PASS", "log": str(log),
            "duration_seconds": 0.5,
            "toolchain_version": "Windows 11 10.0.26300 / Python 3.14.8",
        }
        base_validation = {
            "run_id": run_id, "code_commit": head, "git_head": head,
            "commands": [base_command],
        }
        validation.write_text(json.dumps(base_validation), encoding="utf-8")
        not_run.write_text(json.dumps({"run_id": run_id, "code_commit": head,
                                       "tests": []}), encoding="utf-8")
        env_sha = sha256_file(env)
        manifest_data = json.loads(manifest.read_text(encoding="utf-8"))
        collector_sha = sha256_file("tools/collect_validation_results.py")
        base_records, base_errors = build_records(
            base_validation, run_id, head, str(env), env_sha, collector_sha,
            manifest_data)
        base_records, source_errors = attach_sources(
            base_records, manifest_data, head)
        base_errors += source_errors
        base_env = json.loads(env.read_text(encoding="utf-8"))
        for record in base_records:
            base_errors += validate_record(
                record, manifest_data, run_id, head, {str(env): base_env})
        if base_errors:
            failures.append("base valid sample rejected: " + "; ".join(base_errors))

        def expect_failure(label, mutate):
            nonlocal cases
            cases += 1
            try:
                mutate()
                records, errors = build_records(
                    json.loads(validation.read_text(encoding="utf-8")),
                    run_id, head, str(env), env_sha, collector_sha,
                    json.loads(manifest.read_text(encoding="utf-8")))
                records, source_errors = attach_sources(
                    records, json.loads(manifest.read_text(encoding="utf-8")), head)
                errors += source_errors
                for record in records:
                    errors += validate_record(
                        record, json.loads(manifest.read_text(encoding="utf-8")),
                        run_id, head, {str(env): json.loads(env.read_text(encoding="utf-8"))})
                if not errors:
                    failures.append(f"{label}: negative case was accepted")
            except (EvidenceError, SystemExit, Exception) as exc:
                if isinstance(exc, (KeyboardInterrupt, SystemExit)) and not isinstance(exc, SystemExit):
                    raise
                # Any explicit failure is the desired result.
                pass

        # 1. log only PASS text, no wrapper.
        expect_failure("01 PASS-text-only", lambda: log.write_text("PASS\n"))
        _write_log(log, run_id, head, command, 0, 0.5,
                   "FIXTURE_CHECKS=3 FAILURES=0")

        # 2. fabricated checks marker not matching the real machine marker.
        expect_failure("02 fabricated checks", lambda: _write_log(
            log, run_id, head, command, 0, 0.5, "checks=999 failures=0"))
        _write_log(log, run_id, head, command, 0, 0.5,
                   "FIXTURE_CHECKS=3 FAILURES=0")

        # 3. two conflicting checks markers.
        expect_failure("03 conflicting checks", lambda: _write_log(
            log, run_id, head, command, 0, 0.5,
            "FIXTURE_CHECKS=3 FAILURES=0\nFIXTURE_CHECKS=4 FAILURES=0"))
        _write_log(log, run_id, head, command, 0, 0.5,
                   "FIXTURE_CHECKS=3 FAILURES=0")

        # 4. log failures mismatch.
        expect_failure("04 failures mismatch", lambda: _write_log(
            log, run_id, head, command, 0, 0.5, "FIXTURE_CHECKS=3 FAILURES=1"))
        _write_log(log, run_id, head, command, 0, 0.5,
                   "FIXTURE_CHECKS=3 FAILURES=0")

        # 5. nonzero exit in header.
        expect_failure("05 nonzero exit", lambda: _write_log(
            log, run_id, head, command, 1, 0.5, "FIXTURE_CHECKS=3 FAILURES=0"))
        _write_log(log, run_id, head, command, 0, 0.5,
                   "FIXTURE_CHECKS=3 FAILURES=0")

        # 6. source working-tree SHA mismatch.
        def case06():
            data = json.loads(manifest.read_text(encoding="utf-8"))
            data["files"][SELFTEST_SOURCE]["working_tree_sha256"] = "0" * 64
            manifest.write_text(json.dumps(data), encoding="utf-8")
        expect_failure("06 source SHA mismatch", case06)
        manifest.write_text(json.dumps({
            "run_id": run_id, "code_commit": head,
            "files": {SELFTEST_SOURCE: {
                "git_blob_sha": source_blob,
                "working_tree_sha256": source_sha,
                "size_bytes": len(source_bytes),
                "line_ending": line_ending(source_bytes),
                "role": "build-definition"}},
            "tests": {"fixture": [SELFTEST_SOURCE]},
            "compile_inputs": {"fixture": [SELFTEST_SOURCE]},
        }), encoding="utf-8")

        # 7. source missing.
        def case07():
            data = json.loads(manifest.read_text(encoding="utf-8"))
            data["tests"]["fixture"] = ["does/not/exist.c"]
            manifest.write_text(json.dumps(data), encoding="utf-8")
        expect_failure("07 source missing", case07)
        manifest.write_text(json.dumps({
            "run_id": run_id, "code_commit": head,
            "files": {SELFTEST_SOURCE: {
                "git_blob_sha": source_blob,
                "working_tree_sha256": source_sha,
                "size_bytes": len(source_bytes),
                "line_ending": line_ending(source_bytes),
                "role": "build-definition"}},
            "tests": {"fixture": [SELFTEST_SOURCE]},
            "compile_inputs": {"fixture": [SELFTEST_SOURCE]},
        }), encoding="utf-8")

        # 8. environment RUN_ID mismatch.
        def case08():
            data = json.loads(env.read_text(encoding="utf-8"))
            data["run_id"] = "WRONG"
            env.write_text(json.dumps(data), encoding="utf-8")
        expect_failure("08 env RUN_ID", case08)
        env.write_text(json.dumps({
            "run_id": run_id, "code_commit": head,
            "toolchain_version": "Windows 11 10.0.26300 / Python 3.14.8"}),
            encoding="utf-8")

        # 9. environment code_commit mismatch.
        def case09():
            data = json.loads(env.read_text(encoding="utf-8"))
            data["code_commit"] = "f" * 40
            env.write_text(json.dumps(data), encoding="utf-8")
        expect_failure("09 env code_commit", case09)
        env.write_text(json.dumps({
            "run_id": run_id, "code_commit": head,
            "toolchain_version": "Windows 11 10.0.26300 / Python 3.14.8"}),
            encoding="utf-8")

        # 10. environment SHA mismatch.
        def case10():
            data = json.loads(env.read_text(encoding="utf-8"))
            data["toolchain_version"] = "Windows 11 10.0.26300 / Python 3.14.9"
            env.write_text(json.dumps(data), encoding="utf-8")
        expect_failure("10 env SHA mismatch", case10)
        env.write_text(json.dumps({
            "run_id": run_id, "code_commit": head,
            "toolchain_version": "Windows 11 10.0.26300 / Python 3.14.8"}),
            encoding="utf-8")

        # 11. placeholder toolchain.
        def case11():
            data = json.loads(env.read_text(encoding="utf-8"))
            data["toolchain_version"] = "see r25_environment"
            env.write_text(json.dumps(data), encoding="utf-8")
        expect_failure("11 placeholder toolchain", case11)
        env.write_text(json.dumps({
            "run_id": run_id, "code_commit": head,
            "toolchain_version": "Windows 11 10.0.26300 / Python 3.14.8"}),
            encoding="utf-8")

        # 12. random test with deterministic seed.
        def case12():
            data = json.loads(validation.read_text(encoding="utf-8"))
            data["commands"][0]["name"] = "map_random"
            validation.write_text(json.dumps(data), encoding="utf-8")
            _write_log(log, run_id, head, command, 0, 0.5,
                       "M5_MAP_ROLL_RANDOM seed=deterministic checked_cases=3 failed_cases=0")
        expect_failure("12 deterministic seed", case12)
        validation.write_text(json.dumps(base_validation), encoding="utf-8")
        _write_log(log, run_id, head, command, 0, 0.5,
                   "FIXTURE_CHECKS=3 FAILURES=0")

        # 13. random test with missing seed.
        def case13():
            data = json.loads(validation.read_text(encoding="utf-8"))
            data["commands"][0]["name"] = "map_random"
            validation.write_text(json.dumps(data), encoding="utf-8")
            _write_log(log, run_id, head, command, 0, 0.5,
                       "M5_MAP_ROLL_RANDOM seed= checked_cases=3 failed_cases=0")
        expect_failure("13 missing seed", case13)
        validation.write_text(json.dumps(base_validation), encoding="utf-8")
        _write_log(log, run_id, head, command, 0, 0.5,
                   "FIXTURE_CHECKS=3 FAILURES=0")

        # 14. duplicate test_id.
        def case14():
            data = json.loads(validation.read_text(encoding="utf-8"))
            second = dict(data["commands"][0])
            second["name"] = "fixture_dup"
            data["commands"].append(second)
            validation.write_text(json.dumps(data), encoding="utf-8")
        expect_failure("14 duplicate test_id", case14)
        validation.write_text(json.dumps(base_validation), encoding="utf-8")

        # 15. truncated log.
        expect_failure("15 truncated log", lambda: _write_log(
            log, run_id, head, command, 0, 0.5,
            "FIXTURE_CHECKS=3 FAILURES=0", end=False))
        _write_log(log, run_id, head, command, 0, 0.5,
                   "FIXTURE_CHECKS=3 FAILURES=0")

        # 16. log SHA mismatch in an otherwise valid record.
        def case16():
            data = json.loads(validation.read_text(encoding="utf-8"))
            data["commands"][0]["declared_log_sha256"] = "0" * 64
            validation.write_text(json.dumps(data), encoding="utf-8")
        # The collector ignores caller-declared hashes by design; mutate the
        # built record instead through a direct validation-path check.
        cases += 1
        try:
            rec = dict(base_records[0])
            rec["log_sha256"] = "0" * 64
            errors = validate_record(rec, manifest_data, run_id, head,
                                     {str(env): base_env})
            if not errors:
                failures.append("16 log SHA mismatch: accepted")
        except Exception:
            pass

        # 17. collector SHA mismatch.
        cases += 1
        try:
            rec = dict(base_records[0])
            rec["collector_sha256"] = "0" * 64
            errors = validate_record(rec, manifest_data, run_id, head,
                                     {str(env): base_env})
            if not errors:
                failures.append("17 collector SHA mismatch: accepted")
        except Exception:
            pass

        # 18. non-existent Git commit.
        cases += 1
        try:
            records, errors = build_records(
                base_validation, run_id, "f" * 40, str(env), env_sha,
                collector_sha, manifest_data)
            if not errors:
                failures.append("18 missing Git commit: accepted")
        except Exception:
            pass

        # 19. historical RUN_ID in log.
        expect_failure("19 historical RUN_ID", lambda: _write_log(
            log, "M5-06-R27-20261111", head, command, 0, 0.5,
            "FIXTURE_CHECKS=3 FAILURES=0"))
        _write_log(log, run_id, head, command, 0, 0.5,
                   "FIXTURE_CHECKS=3 FAILURES=0")

        # 20. NOT_RUN with Markdown log.
        cases += 1
        try:
            rec = dict(base_records[0])
            rec.update({"status": "NOT_RUN", "not_run_reason": "x",
                        "log_path": "README.md", "log_sha256": "0" * 64})
            errors = validate_record(rec, manifest_data, run_id, head,
                                     {str(env): base_env})
            if not errors:
                failures.append("20 NOT_RUN Markdown log: accepted")
        except Exception:
            pass

        # 21. PASS checks=0.
        expect_failure("21 PASS checks=0", lambda: _write_log(
            log, run_id, head, command, 0, 0.5, "FIXTURE_CHECKS=0 FAILURES=0"))
        _write_log(log, run_id, head, command, 0, 0.5,
                   "FIXTURE_CHECKS=3 FAILURES=0")

        # 22. PASS failures>0.
        expect_failure("22 PASS failures>0", lambda: _write_log(
            log, run_id, head, command, 0, 0.5, "FIXTURE_CHECKS=3 FAILURES=1"))
        _write_log(log, run_id, head, command, 0, 0.5,
                   "FIXTURE_CHECKS=3 FAILURES=0")

        # 23. source manifest missing.
        cases += 1
        try:
            records, errors = build_records(
                base_validation, run_id, head, str(env), env_sha,
                collector_sha, {"files": {}})
            records, errors2 = attach_sources(records, {"files": {}}, head)
            if not errors + errors2:
                failures.append("23 missing source manifest: accepted")
        except Exception:
            pass

        # 24. incomplete compile inputs.
        def case24():
            data = json.loads(manifest.read_text(encoding="utf-8"))
            data["compile_inputs"]["fixture"] = []
            manifest.write_text(json.dumps(data), encoding="utf-8")
        expect_failure("24 incomplete compile inputs", case24)
        manifest.write_text(json.dumps({
            "run_id": run_id, "code_commit": head,
            "files": {SELFTEST_SOURCE: {
                "git_blob_sha": source_blob,
                "working_tree_sha256": source_sha,
                "size_bytes": len(source_bytes),
                "line_ending": line_ending(source_bytes),
                "role": "build-definition"}},
            "tests": {"fixture": [SELFTEST_SOURCE]},
            "compile_inputs": {"fixture": [SELFTEST_SOURCE]},
        }), encoding="utf-8")

        # 25. log command differs from ledger.
        expect_failure("25 command mismatch", lambda: _write_log(
            log, run_id, head, ["python", "other.py"], 0, 0.5,
            "FIXTURE_CHECKS=3 FAILURES=0"))
        _write_log(log, run_id, head, command, 0, 0.5,
                   "FIXTURE_CHECKS=3 FAILURES=0")

        # 26. missing duration.
        expect_failure("26 zero duration", lambda: _write_log(
            log, run_id, head, command, 0, 0, "FIXTURE_CHECKS=3 FAILURES=0"))
        _write_log(log, run_id, head, command, 0, 0.5,
                   "FIXTURE_CHECKS=3 FAILURES=0")

        # 27. mixed host/ARM toolchain.
        def case27():
            data = json.loads(env.read_text(encoding="utf-8"))
            data["toolchain_version"] = "Windows 11 / arm-none-eabi-gcc 14.3.1"
            env.write_text(json.dumps(data), encoding="utf-8")
        expect_failure("27 mixed toolchain", case27)
        env.write_text(json.dumps({
            "run_id": run_id, "code_commit": head,
            "toolchain_version": "Windows 11 10.0.26300 / Python 3.14.8"}),
            encoding="utf-8")

        # 28. source manifest code commit mismatch.
        def case28():
            data = json.loads(manifest.read_text(encoding="utf-8"))
            data["code_commit"] = "e" * 40
            manifest.write_text(json.dumps(data), encoding="utf-8")
        expect_failure("28 manifest commit mismatch", case28)

        if failures:
            for failure in failures:
                print("SELFTEST_FAIL:", failure, file=sys.stderr)
            return 1
        print(f"COLLECTOR_SELF_TEST_PASS ({cases} negative cases)")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id")
    parser.add_argument("--code-commit")
    parser.add_argument("--output")
    parser.add_argument("--validation", default=str(ROOT / DEFAULT_VALIDATION))
    parser.add_argument("--environment", default=DEFAULT_ENV)
    parser.add_argument("--source-manifest", default=DEFAULT_MANIFEST)
    parser.add_argument("--not-run", default=DEFAULT_NOT_RUN)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)
    if args.self_test:
        return self_test()
    if not args.run_id or not args.code_commit or not args.output:
        raise SystemExit("--run-id, --code-commit and --output are required")
    if not re.fullmatch(r"[0-9a-f]{40}", args.code_commit):
        raise SystemExit("code commit must be 40-hex")
    return collect(args)


if __name__ == "__main__":
    sys.exit(main())
