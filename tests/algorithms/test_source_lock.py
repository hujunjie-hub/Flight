"""Regression tests for the embedded OSQP provenance lock generator."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(
    0,
    str(ROOT / "middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp"),
)
from source_lock import git_blob_sha1, refresh_lock


class SourceLockTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.generated = self.root / "generated"
        self.generated.mkdir()
        self.lock_path = self.root / "UPSTREAM_LOCK.json"

        upstream = b"pinned upstream source\n"
        patched = b"local patched source\n"
        generated = b"generated workspace\n"
        (self.generated / "upstream.c").write_bytes(upstream)
        (self.generated / "patched.c").write_bytes(patched)
        (self.generated / "workspace.c").write_bytes(generated)
        (self.generated / "Makefile").write_text("host only\n", encoding="utf-8")

        self.lock = {
            "schema_version": 2,
            "hash_algorithm": "git-blob-sha1",
            "host_only_files": ["Makefile"],
            "files": {
                "upstream.c": {
                    "git_blob_sha1": git_blob_sha1(upstream),
                    "provenance": "upstream",
                    "upstream": {"blob_sha1": git_blob_sha1(upstream)},
                },
                "patched.c": {
                    "git_blob_sha1": git_blob_sha1(patched),
                    "provenance": "local_patch",
                    "upstream": {"blob_sha1": git_blob_sha1(b"original upstream\n")},
                    "patch_summary": "Intentional local patch.",
                },
                "workspace.c": {
                    "git_blob_sha1": git_blob_sha1(generated),
                    "provenance": "generated",
                    "generator": "test generator",
                },
            },
        }
        self.write_lock()

    def tearDown(self):
        self.temp.cleanup()

    def write_lock(self):
        self.lock_path.write_text(
            json.dumps(self.lock, indent=2) + "\n", encoding="utf-8"
        )

    def test_git_blob_known_vector(self):
        self.assertEqual(
            git_blob_sha1(b"abc"),
            "f2ba8f84ab5c1bce84a7b441cb1959cfc7093b7f",
        )

    def test_refresh_updates_only_local_and_generated_hashes(self):
        (self.generated / "patched.c").write_bytes(b"patched v2\n")
        (self.generated / "workspace.c").write_bytes(b"workspace v2\n")
        refreshed = refresh_lock(self.lock_path, self.generated)
        self.assertEqual(
            refreshed["files"]["upstream.c"]["git_blob_sha1"],
            refreshed["files"]["upstream.c"]["upstream"]["blob_sha1"],
        )
        self.assertEqual(
            refreshed["files"]["patched.c"]["git_blob_sha1"],
            git_blob_sha1(b"patched v2\n"),
        )
        self.assertEqual(
            refreshed["files"]["workspace.c"]["git_blob_sha1"],
            git_blob_sha1(b"workspace v2\n"),
        )
        self.assertTrue((self.generated / "Makefile").is_file())

    def test_upstream_change_is_rejected(self):
        (self.generated / "upstream.c").write_bytes(b"unreviewed upstream change\n")
        with self.assertRaisesRegex(ValueError, "differs from pinned source"):
            refresh_lock(self.lock_path, self.generated)

    def test_untracked_embedded_file_is_rejected(self):
        (self.generated / "unexpected.c").write_text("unexpected\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "file set mismatch"):
            refresh_lock(self.lock_path, self.generated)

    def test_missing_embedded_file_is_rejected(self):
        (self.generated / "workspace.c").unlink()
        with self.assertRaisesRegex(ValueError, "file set mismatch"):
            refresh_lock(self.lock_path, self.generated)


if __name__ == "__main__":
    unittest.main()
