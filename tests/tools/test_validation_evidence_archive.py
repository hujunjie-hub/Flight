#!/usr/bin/env python3
"""Tests for the non-self-referential archive metadata/checker contract."""
import hashlib
import json
import subprocess
import tempfile
import unittest
import zipfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import tools.check_validation_evidence as checker

SHA_A = "a" * 40
SHA_B = "b" * 40
SHA_C = "c" * 40
SHA_D = "d" * 40
SHA_E = "e" * 40
BRANCH = "opt/stage-02-m5-06-safety-validator"
RUN_ID = "M5-06-R29-20261111"


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


class ArchiveManifestCheckerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.out = self.root / "checks"
        self.manifest_path = self.root / "flight_acceptance_manifest.json"
        self.lv = {
            "run_id": RUN_ID,
            "branch": BRANCH,
            "validated_code_commit": SHA_A,
            "tooling_commit": SHA_B,
            "collector_commit": SHA_C,
            "archive_script_commit": SHA_D,
            "archive_head_sha": SHA_E,
            "archive_commit_sha": SHA_E,
            "remote_head": SHA_D,
            "ahead_commit_count": 92,
            "archive_zip_sha256": None,
            "archive_zip_size_bytes": None,
            "archive_zip_hash_location": "EXTERNAL_ARCHIVE_REPORT_AND_SIDECAR",
            "archive_external_report": f"work/validation/archives/{RUN_ID}.archive_report.json",
            "archive_zip_sidecar": f"work/validation/archives/{RUN_ID}.zip.sha256",
            "archive_metadata_policy": "PREZIP_GIT_METADATA_PLUS_EXTERNAL_POSTZIP_DIGEST_V1",
        }
        tooling_paths = (
            ("SConstruct", "firmware_build_entrypoint"),
            ("tools/check_rtt_package_tree.py", "rtt_package_tree_guard"),
            ("tools/collect_validation_results.py", "collector"),
            ("tools/check_validation_evidence.py", "evidence_checker"),
            ("tools/analyze_stack.py", "stack_analyzer"),
            ("tools/prepare_archive_metadata.py", "archive_metadata_preparer"),
            ("tools/archive_validation_run.ps1", "archive_entrypoint"),
        )
        self.tooling_sources = []
        self.tooling_blob_by_path = {}
        for rel, role in tooling_paths:
            raw = (checker.ROOT / rel).read_bytes()
            blob = subprocess.check_output(
                ["git", "-C", str(checker.ROOT), "rev-parse", f"HEAD:{rel}"], text=True).strip()
            ending = "CRLF" if b"\r\n" in raw and b"\n" not in raw.replace(b"\r\n", b"") else ("MIXED" if b"\r\n" in raw else ("LF" if b"\n" in raw else "NONE"))
            self.tooling_blob_by_path[rel] = blob
            self.tooling_sources.append({"path": rel, "role": role, "git_blob_sha": blob,
                                         "working_tree_sha256": digest(raw),
                                         "size_bytes": len(raw), "line_ending": ending})
        self.lv["tooling_sources"] = self.tooling_sources
        self.lv["tooling_source_manifest_sha256"] = digest(json.dumps(
            self.tooling_sources, sort_keys=True, separators=(",", ":")).encode("utf-8"))
        self.lv["collector_worktree_sha256"] = next(
            x["working_tree_sha256"] for x in self.tooling_sources
            if x["path"] == "tools/collect_validation_results.py")
        self.lv["archive_script_worktree_sha256"] = next(
            x["working_tree_sha256"] for x in self.tooling_sources
            if x["path"] == "tools/archive_validation_run.ps1")
        self.ctx = {
            "manifest": {"latest_local_validation": dict(self.lv)},
            "run_id": RUN_ID,
            "branch": BRANCH,
            "code_commit": SHA_A,
            "head": SHA_E,
            "remote_head": SHA_D,
            "ahead": 92,
            "out": self.out,
            "paths": {"manifest": str(self.manifest_path)},
        }
        self.manifest_path.write_text(json.dumps(self.ctx["manifest"]), encoding="utf-8")

    def tearDown(self):
        self.tmp.cleanup()

    def run_manifest(self):
        def fake_git(*args):
            joined = " ".join(args)
            if args and args[0] == "rev-parse" and len(args) > 1 and args[1].startswith("HEAD:"):
                return self.tooling_blob_by_path.get(args[1][5:], "")
            if "collect_validation_results.py" in joined and "check_validation_evidence.py" not in joined:
                return SHA_C
            if "archive_validation_run.ps1" in joined or "finalize_validation_archive.py" in joined:
                return SHA_D
            if "collect_validation_results.py" in joined and "check_validation_evidence.py" in joined:
                return SHA_B
            return ""
        completed = SimpleNamespace(returncode=0)
        with patch.object(checker, "git", side_effect=fake_git), \
             patch.object(checker.subprocess, "run", return_value=completed):
            return checker.check_manifest(self.ctx)

    def test_accepts_external_only_zip_digest(self):
        result = self.run_manifest()
        self.assertEqual(result["status"], "PASS")

    def test_rejects_pending_archive_head(self):
        self.ctx["manifest"]["latest_local_validation"]["archive_head_sha"] = "PENDING_ARCHIVE_HEAD"
        result = self.run_manifest()
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_embedded_zip_hash(self):
        self.ctx["manifest"]["latest_local_validation"]["archive_zip_sha256"] = "f" * 64
        result = self.run_manifest()
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_stale_ahead_count(self):
        self.ctx["manifest"]["latest_local_validation"]["ahead_commit_count"] = 87
        result = self.run_manifest()
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_wrong_archive_head(self):
        self.ctx["manifest"]["latest_local_validation"]["archive_head_sha"] = SHA_A
        result = self.run_manifest()
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_wrong_tooling_commit(self):
        self.ctx["manifest"]["latest_local_validation"]["tooling_commit"] = SHA_D
        result = self.run_manifest()
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_missing_tooling_sources(self):
        self.ctx["manifest"]["latest_local_validation"]["tooling_sources"] = []
        result = self.run_manifest()
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_wrong_external_hash_policy(self):
        self.ctx["manifest"]["latest_local_validation"]["archive_zip_hash_location"] = "EMBEDDED"
        result = self.run_manifest()
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_wrong_remote_head(self):
        self.ctx["manifest"]["latest_local_validation"]["remote_head"] = SHA_A
        result = self.run_manifest()
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_wrong_collector_worktree_hash(self):
        self.ctx["manifest"]["latest_local_validation"]["collector_worktree_sha256"] = "0" * 64
        result = self.run_manifest()
        self.assertEqual(result["status"], "FAIL")


class ArchiveZipCheckerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.run_id = RUN_ID
        self.zip_path = self.root / f"{RUN_ID}.zip"
        self.sidecar = self.root / f"{RUN_ID}.zip.sha256"
        self.report_path = self.root / f"{RUN_ID}.archive_report.json"
        self.out = self.root / "checks"
        self.file_data = b"evidence bytes\n"
        source_repo = self.root / "source_repo"
        source_repo.mkdir()
        def git(*args):
            return subprocess.check_output(["git", "-C", str(source_repo), *args],
                                           text=True, stderr=subprocess.STDOUT).strip()
        git("init", "-b", BRANCH)
        git("config", "user.email", "archive-test@example.invalid")
        git("config", "user.name", "Archive Test")
        (source_repo / "evidence.txt").write_text("bundle fixture\n", encoding="utf-8")
        git("add", "evidence.txt")
        git("commit", "-m", "bundle fixture")
        self.bundle_path = self.root / f"{RUN_ID}.bundle"
        git("bundle", "create", str(self.bundle_path), "HEAD", f"refs/heads/{BRANCH}")
        self.bundle_data = self.bundle_path.read_bytes()
        self.bundle_sha = digest(self.bundle_data)
        self.bundle_refs = subprocess.check_output(
            ["git", "bundle", "list-heads", str(self.bundle_path)], text=True).splitlines()
        self.bundle_rel = f"build/validation/regenerated/{RUN_ID}.bundle"
        archive_manifest = {
            "run_id": RUN_ID,
            "branch": BRANCH,
            "validated_code_commit": SHA_A,
            "tooling_commit": SHA_B,
            "collector_commit": SHA_C,
            "archive_script_commit": SHA_D,
            "archive_head_sha": SHA_E,
            "archive_commit_sha": SHA_E,
            "remote_head": SHA_D,
            "ahead_commit_count": 92,
            "archive_zip_hash_location": "EXTERNAL_ARCHIVE_REPORT_AND_SIDECAR",
            "tooling_source_manifest_sha256": "1" * 64,
            "collector_worktree_sha256": "2" * 64,
            "archive_script_worktree_sha256": "3" * 64,
            "tooling_sources": [{"path": "tools/fake.py", "working_tree_sha256": "4" * 64}],
            "git_bundle": {"path": self.bundle_rel, "sha256": self.bundle_sha,
                           "refs": self.bundle_refs},
            "evidence_files": [{"path": "evidence.log", "size_bytes": len(self.file_data),
                                "sha256": digest(self.file_data)}],
        }
        with zipfile.ZipFile(self.zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
            zf.writestr(f"{RUN_ID}/archive_manifest.json", json.dumps(archive_manifest))
            zf.writestr(f"{RUN_ID}/SHA256SUMS.txt", "fixture\n")
            zf.writestr(f"{RUN_ID}/{self.bundle_rel}", self.bundle_data)
            zf.writestr(f"{RUN_ID}/evidence.log", self.file_data)
        zip_sha = digest(self.zip_path.read_bytes())
        self.sidecar.write_text(f"{zip_sha}  {self.zip_path.name}\n", encoding="ascii")
        self.report = {
            "run_id": RUN_ID,
            "branch": BRANCH,
            "validated_code_commit": SHA_A,
            "tooling_commit": SHA_B,
            "collector_commit": SHA_C,
            "archive_script_commit": SHA_D,
            "archive_head_sha": SHA_E,
            "archive_commit_sha": SHA_E,
            "remote_head": SHA_D,
            "ahead_commit_count": 92,
            "archive_zip_hash_location": "EXTERNAL_ARCHIVE_REPORT_AND_SIDECAR",
            "tooling_source_manifest_sha256": "1" * 64,
            "collector_worktree_sha256": "2" * 64,
            "archive_script_worktree_sha256": "3" * 64,
            "tooling_sources": [{"path": "tools/fake.py", "working_tree_sha256": "4" * 64}],
            "bundle": self.bundle_rel,
            "bundle_sha256": self.bundle_sha,
            "bundle_refs": self.bundle_refs,
            "archive_zip_sha256": zip_sha,
            "archive_zip_size_bytes": self.zip_path.stat().st_size,
            "archive_zip_path": str(self.zip_path),
            "sidecar_path": str(self.sidecar),
            "scope": "LOCAL_EXECUTION_ONLY",
        }
        self.report_path.write_text(json.dumps(self.report), encoding="utf-8")
        self.ctx = {
            "archive_report": self.report,
            "manifest": {"latest_local_validation": {
                k: self.report.get(k) for k in (
                    "run_id", "branch", "validated_code_commit", "tooling_commit",
                    "collector_commit", "archive_script_commit", "archive_head_sha",
                    "archive_commit_sha", "remote_head", "ahead_commit_count",
                    "archive_zip_hash_location", "tooling_source_manifest_sha256",
                    "collector_worktree_sha256", "archive_script_worktree_sha256")
            }},
            "run_id": RUN_ID,
            "head": SHA_E,
            "remote_head": SHA_D,
            "ahead": 92,
            "out": self.out,
            "paths": {"archive_report": str(self.report_path)},
        }

    def rewrite_zip(self, mutate=None, mutate_member=None):
        with zipfile.ZipFile(self.zip_path, "r") as source:
            members = {name: source.read(name) for name in source.namelist()}
        manifest_name = f"{RUN_ID}/archive_manifest.json"
        if mutate is not None:
            data = json.loads(members[manifest_name].decode("utf-8"))
            mutate(data)
            members[manifest_name] = json.dumps(data).encode("utf-8")
        if mutate_member is not None:
            name, content = mutate_member
            members[name] = content
        with zipfile.ZipFile(self.zip_path, "w", compression=zipfile.ZIP_DEFLATED) as target:
            for name, content in members.items():
                target.writestr(name, content)
        zip_sha = digest(self.zip_path.read_bytes())
        self.report["archive_zip_sha256"] = zip_sha
        self.report["archive_zip_size_bytes"] = self.zip_path.stat().st_size
        self.sidecar.write_text(f"{zip_sha}  {self.zip_path.name}\n", encoding="ascii")
        self.report_path.write_text(json.dumps(self.report), encoding="utf-8")

    def tearDown(self):
        self.tmp.cleanup()

    def test_archive_report_zip_sidecar_and_embedded_manifest_match(self):
        result = checker.check_archive(self.ctx)
        self.assertEqual(result["status"], "PASS")

    def test_rejects_wrong_external_ahead(self):
        self.ctx["archive_report"]["ahead_commit_count"] = 91
        result = checker.check_archive(self.ctx)
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_sidecar_mismatch(self):
        self.sidecar.write_text("0" * 64 + "  " + self.zip_path.name + "\n", encoding="ascii")
        result = checker.check_archive(self.ctx)
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_report_run_id_mismatch(self):
        self.ctx["archive_report"]["run_id"] = "M5-06-OTHER"
        result = checker.check_archive(self.ctx)
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_report_branch_mismatch(self):
        self.ctx["archive_report"]["branch"] = "main"
        result = checker.check_archive(self.ctx)
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_report_bundle_sha_mismatch(self):
        self.ctx["archive_report"]["bundle_sha256"] = "0" * 64
        result = checker.check_archive(self.ctx)
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_report_bundle_refs_mismatch(self):
        self.ctx["archive_report"]["bundle_refs"] = ["wrong HEAD"]
        result = checker.check_archive(self.ctx)
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_report_bundle_path_mismatch(self):
        self.ctx["archive_report"]["bundle"] = "wrong.bundle"
        result = checker.check_archive(self.ctx)
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_report_tooling_sources_mismatch(self):
        self.ctx["archive_report"]["tooling_sources"] = [{"path": "other.py"}]
        result = checker.check_archive(self.ctx)
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_embedded_archive_manifest_mismatch(self):
        self.rewrite_zip(mutate=lambda data: data.update({"ahead_commit_count": 91}))
        result = checker.check_archive(self.ctx)
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_evidence_file_hash_mismatch(self):
        self.rewrite_zip(mutate_member=(f"{RUN_ID}/evidence.log", b"tampered\n"))
        result = checker.check_archive(self.ctx)
        self.assertEqual(result["status"], "FAIL")

    def test_rejects_missing_sidecar(self):
        self.sidecar.unlink()
        result = checker.check_archive(self.ctx)
        self.assertEqual(result["status"], "FAIL")


if __name__ == "__main__":
    unittest.main(verbosity=2)
