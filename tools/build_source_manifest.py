#!/usr/bin/env python3
"""Build the R28 source manifest used by the strict collector.

Every entry records the Git blob at the validated code commit, the current
working-tree SHA256, line ending, size and role. Compile tests also list their
actual translation-unit inputs; the collector rejects an incomplete list.
"""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def line_ending(data):
    crlf = data.count(b"\r\n")
    lf = data.count(b"\n")
    if crlf == 0 and lf == 0:
        return "NONE"
    if crlf and crlf == lf:
        return "CRLF"
    return "LF" if crlf == 0 else "MIXED"


def ls_tree(commit):
    proc = subprocess.run(["git", "-C", str(ROOT), "ls-tree", "-r", commit],
                          capture_output=True, text=True, check=True)
    result = {}
    for line in proc.stdout.splitlines():
        meta, path = line.split("\t", 1)
        mode, kind, blob = meta.split()
        result[path] = {"blob": blob, "kind": kind, "mode": mode}
    return result


def files_under(*roots):
    paths = []
    for root in roots:
        base = ROOT / root
        if base.is_file():
            paths.append(base)
        elif base.is_dir():
            paths.extend(p for p in base.rglob("*") if p.is_file())
    return paths


def rel(paths):
    out = []
    for p in paths:
        try:
            out.append(p.resolve().relative_to(ROOT.resolve()).as_posix())
        except ValueError:
            continue
    return sorted(set(out))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--code-commit", required=True)
    ap.add_argument("--output", default="build/validation/regenerated/r28_source_manifest.json")
    args = ap.parse_args()
    tree = ls_tree(args.code_commit)

    all_c = files_under("middleware/contracts", "middleware/model/fms",
                        "middleware/guidance")
    m5_c = files_under("middleware/guidance")
    m5_source = rel(m5_c) + rel(files_under(
        "middleware/model/control/outer_loop/quad_x4/position_tracking/control_reference.c",
        "middleware/model/fms/quad_x4/flight_policy/quad_x4_product_config.c",
        "middleware/model/fms/geofence_geometry/geofence_geometry.c",
        "tests/guidance/test_m5_main.c",
        "tests/guidance/m5_trajectory_safety_test_api.h",
        "middleware/model/control",
    ))
    m4_source = rel(files_under(
        "middleware/model/fms",
        "middleware/contracts",
        "tests/flight_management/test_m4_main.c",
    ))
    integration_source = sorted(set(m4_source + m5_source))
    common_tools = rel(files_under(
        "tools/collect_validation_results.py", "tools/check_validation_evidence.py",
        "tools/analyze_stack.py", "SConstruct", ".gitattributes",
        "spec/validation/flight_acceptance_manifest.json",
        "spec/quality/ISSUE_REGISTER.md", "spec/quality/R28_ISSUE_DETAILS.md",
        "spec/quality/BLOCKER_BOARD.md",
        "spec/history/implementation/guidance/M5_06_6_MR009_PROOF.md",
        "build/validation/regenerated/r28_environment.json",
        "build/validation/regenerated/r28_test_results.json",
        "build/validation/regenerated/r28_stack_analysis.json",
    ))
    tests = {
        "line_endings": rel(files_under("tools/check_line_endings.py", ".gitattributes")),
        "python_o_negative": rel(files_under(
            "tests/guidance/test_audit_optimized.py", "spec/tools/audit_spec.py")),
        "m4_host": m4_source,
        "m5_host": m5_source,
        "integration": integration_source,
        "m5_sanitizer": m5_source,
        "integration_sanitizer": integration_source,
        "m4_arm": m4_source,
        "m5_arm": m5_source,
        "consumer_fp": rel(files_under(
            "tests/guidance/check_consumer_fp_contraction.py",
            "middleware/model/control/outer_loop/quad_x4/position_tracking/control_reference.c")),
        "fp_model_selftest": rel(files_under(
            "tests/guidance/check_consumer_fp_contraction.py")),
        "spec_layout": rel(files_under(
            "spec/tools/audit_spec.py", "spec/FCU.xml", "spec/message_registry.json")),
        "memory_budget": rel(files_under(
            "tests/guidance/check_memory_budget.py",
            "spec/validation/flight_acceptance_manifest.json",
            "spec/validation/m4-m5-implementation_report.json")),
        "packages_lock": rel(files_under(
            "tools/check_rt_packages_lock.py", "tools/rt_packages.lock.json", ".config")),
        "packages_lock_unit": rel(files_under(
            "tests/guidance/test_rt_packages_lock.py", "tools/rt_packages.lock.json")),
        "rebaseline_proposal": rel(files_under(
            "tools/analyze_eol_rebaseline.py", "spec/FCU.xml", "spec/message_registry.json")),
        "production_abi": rel(files_under(
            "tests/abi/abi_report.py", "middleware/contracts/abi_lock.h",
            "spec/message_registry.json")),
        "scons_link": rel(files_under(
            "tests/guidance/check_target_build.py", "SConstruct", "SConscript",
            "tools/check_rt_packages_lock.py", "tools/rt_packages.lock.json", ".config")),
        "s06_112": rel(files_under(
            "tests/guidance/verify_jerk_continuous.py",
            "tests/guidance/m5_product_envelope.py",
            "middleware/guidance/trajectory_planning/m5_trajectory_safety.c",
            "tests/guidance/test_m5_main.c",
            "tests/guidance/m5_trajectory_safety_test_api.h")),
        "collector_selftest": rel(files_under(
            "tools/collect_validation_results.py", "SConstruct")),
        "collector_selftest_opt": rel(files_under(
            "tools/collect_validation_results.py", "SConstruct")),
        "evidence_checker_selftest": common_tools,
        "product_check": rel(files_under(
            "tools/check_validation_evidence.py",
            "middleware/model/fms/quad_x4/flight_policy/quad_x4_product_config.c",
            "spec/validation/flight_acceptance_manifest.json")),
        "manifest_check": rel(files_under(
            "tools/check_validation_evidence.py",
            "tools/prepare_archive_metadata.py", "tests/guidance/run_validation.py",
            "spec/validation/flight_acceptance_manifest.json")),
        "issues_check": rel(files_under(
            "tools/check_validation_evidence.py", "spec/quality/ISSUE_REGISTER.md")),
        "stack_check": rel(files_under(
            "tools/check_validation_evidence.py", "tools/analyze_stack.py")),
        "stack_selftest": rel(files_under("tools/analyze_stack.py")),
        "stack_image": rel(files_under(
            "tests/guidance/stack_image_check.ps1",
            "tests/guidance/stack_image_stubs.c",
            "tests/guidance/arm_build_check.ps1",
            "tests/guidance/arm_build_check.sh",
            "tools/analyze_stack_image.py",
            "tools/analyze_stack.py")),
        "stack_image_selftest": rel(files_under("tools/analyze_stack_image.py")),
        "runner_selftest": rel(files_under("tests/guidance/test_run_validation.py",
                                           "tests/guidance/run_validation.py")),
    }
    files = {}
    for test_name, paths in tests.items():
        for path in paths:
            if path in files or path not in tree:
                continue
            p = ROOT / path
            if not p.is_file():
                continue
            data = p.read_bytes()
            files[path] = {
                "git_blob_sha": tree[path]["blob"],
                "working_tree_sha256": sha(data),
                "size_bytes": len(data),
                "line_ending": line_ending(data),
                "role": "test-input" if path.startswith("tests/") else "source",
            }
    tests = {name: [p for p in paths if p in files] for name, paths in tests.items()}
    compile_inputs = {}
    for name in ("m4_host", "m5_host", "integration", "m5_sanitizer",
                 "integration_sanitizer", "m4_arm", "m5_arm", "s06_112"):
        compile_inputs[name] = tests[name]
    manifest = {
        "schema_version": "2.0",
        "run_id": args.run_id,
        "code_commit": args.code_commit,
        "generated_from": "git ls-tree + working-tree bytes",
        "files": files,
        "tests": tests,
        "compile_inputs": compile_inputs,
    }
    out = ROOT / args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"SOURCE_MANIFEST_WROTE {out} files={len(files)} tests={len(tests)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
