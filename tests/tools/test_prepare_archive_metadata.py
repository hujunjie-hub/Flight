#!/usr/bin/env python3
"""Negative/positive tests for the two-phase archive metadata contract."""
import json
import subprocess
import tempfile
import unittest
from pathlib import Path

from tools.prepare_archive_metadata import (
    EXTERNAL_HASH_POLICY,
    build_latest_local_validation,
    prepare,
)

SHA_A = "a" * 40
SHA_B = "b" * 40
SHA_C = "c" * 40
SHA_D = "d" * 40


class ArchiveMetadataTests(unittest.TestCase):
    def base(self):
        return {
            "status": "FAIL",
            "test_failures": 0,
            "gate_failures": 3,
            "blocked_gates": 1,
            "archive_zip_sha256": "PENDING_ARCHIVE_ZIP",
            "archive_zip_size_bytes": None,
            "archive_head_sha": "PENDING_ARCHIVE_HEAD",
        }

    def build(self, **overrides):
        values = dict(
            run_id="M5-06-R29-20261111",
            branch="opt/stage-02-m5-06-safety-validator",
            validated_code_commit=SHA_A,
            archive_head_sha=SHA_B,
            remote_head=SHA_C,
            ahead_commit_count=92,
            tooling_commit=SHA_D,
            collector_commit=SHA_A,
            archive_script_commit=SHA_C,
        )
        values.update(overrides)
        return build_latest_local_validation(self.base(), **values)

    def test_valid_metadata_is_complete_and_external_hash_only(self):
        result = self.build()
        self.assertEqual(result["run_id"], "M5-06-R29-20261111")
        self.assertEqual(result["archive_head_sha"], SHA_B)
        self.assertEqual(result["archive_commit_sha"], SHA_B)
        self.assertEqual(result["ahead_commit_count"], 92)
        self.assertIsNone(result["archive_zip_sha256"])
        self.assertIsNone(result["archive_zip_size_bytes"])
        self.assertEqual(result["archive_zip_hash_location"], EXTERNAL_HASH_POLICY)
        self.assertNotIn("PENDING", json.dumps(result))

    def test_rejects_short_git_sha(self):
        with self.assertRaises(ValueError):
            self.build(archive_head_sha="abc123")

    def test_rejects_negative_ahead(self):
        with self.assertRaises(ValueError):
            self.build(ahead_commit_count=-1)

    def test_rejects_invalid_run_id(self):
        with self.assertRaises(ValueError):
            self.build(run_id="R29")

    def test_rejects_detached_head(self):
        with self.assertRaises(ValueError):
            self.build(branch="HEAD")

    def test_does_not_overwrite_test_gate_state(self):
        result = self.build()
        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["gate_failures"], 3)
        self.assertEqual(result["blocked_gates"], 1)

    def test_external_report_and_sidecar_paths_are_run_scoped(self):
        result = self.build()
        self.assertTrue(result["archive_external_report"].endswith(
            "M5-06-R29-20261111.archive_report.json"))
        self.assertTrue(result["archive_zip_sidecar"].endswith(
            "M5-06-R29-20261111.zip.sha256"))

    def test_prepare_reads_real_git_head_remote_and_path_commits(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            def run(*args):
                return subprocess.check_output(["git", "-C", str(root), *args], text=True).strip()
            run("init", "-b", "opt/stage-02-m5-06-safety-validator")
            run("config", "user.email", "agent@example.invalid")
            run("config", "user.name", "Archive Metadata Test")
            (root / "tools").mkdir()
            for rel in ("SConstruct", "tools/check_rtt_package_tree.py",
                        "tools/collect_validation_results.py",
                        "tools/check_validation_evidence.py",
                        "tools/analyze_stack.py",
                        "tools/prepare_archive_metadata.py",
                        "tools/archive_validation_run.ps1"):
                (root / rel).write_text("# fixture\n", encoding="utf-8")
            manifest = root / "spec/validation/flight_acceptance_manifest.json"
            manifest.parent.mkdir(parents=True)
            manifest.write_text(json.dumps({"latest_local_validation": {
                "run_id": "OLD-RUN",
                "validated_code_commit": "0" * 40,
                "status": "FAIL",
                "gate_failures": 2,
            }}), encoding="utf-8")
            run("add", ".")
            run("commit", "-m", "baseline")
            base = run("rev-parse", "HEAD")
            run("update-ref", "refs/remotes/origin/opt/stage-02-m5-06-safety-validator", base)
            (root / "README.md").write_text("ahead\n", encoding="utf-8")
            run("add", "README.md")
            run("commit", "-m", "ahead")
            head = run("rev-parse", "HEAD")

            # The manifest's validated code commit must refer to an existing commit.
            data = json.loads(manifest.read_text(encoding="utf-8"))
            data["latest_local_validation"]["validated_code_commit"] = base
            manifest.write_text(json.dumps(data), encoding="utf-8")
            result = prepare(root, manifest, "M5-06-R29-20261111",
                             "opt/stage-02-m5-06-safety-validator")
            self.assertEqual(result["archive_head_sha"], head)
            self.assertEqual(result["remote_head"], base)
            self.assertEqual(result["ahead_commit_count"], 1)
            self.assertEqual(result["validated_code_commit"], base)
            self.assertEqual(result["archive_zip_hash_location"], EXTERNAL_HASH_POLICY)
            self.assertIsNone(result["archive_zip_sha256"])
            self.assertEqual(result["status"], "FAIL")
            self.assertEqual(result["gate_failures"], 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
