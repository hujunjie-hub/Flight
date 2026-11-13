#!/usr/bin/env python3
"""The BSP package guard must accept the entrypoints Env 4.x actually writes.

Env 4.x emits packages/SConscript + packages/pkgs.json + package directories, and
no packages/Kconfig (that tree lives in the package index and is sourced
optionally by the BSP Kconfig). The guard must reject an empty tree but accept a
real `pkgs --update` result.
"""
import tempfile
import unittest
from subprocess import CompletedProcess
from pathlib import Path
from unittest.mock import patch

from tools.check_rtt_package_tree import (
    installed_package_dirs,
    missing_generated_metadata,
    package_tree_ready,
)
from tools.check_rt_packages_lock import verify_installed_packages


class RtThreadPackageTreeTests(unittest.TestCase):
    def test_installed_package_matches_lock_and_has_no_tracked_edits(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            package = root / "packages/hal"
            package.mkdir(parents=True)
            pin = "a" * 40
            lock = {"packages": [{"symbol": "HAL", "installed_dir": "packages/hal",
                                  "pinned_commit": pin}]}
            for head, dirty, expected in ((pin, "", None), ("b" * 40, "", "HEAD"),
                                          (pin, " M Inc/hal.h", "tracked changes")):
                with self.subTest(head=head, dirty=dirty), patch(
                        "tools.check_rt_packages_lock.subprocess.run", side_effect=[
                            CompletedProcess([], 0, str(package.resolve()) + "\n"),
                            CompletedProcess([], 0, head + "\n"),
                            CompletedProcess([], 0, dirty + "\n")]):
                    _, errors = verify_installed_packages(root, lock)
                    if expected:
                        self.assertTrue(any(expected in error for error in errors), errors)
                    else:
                        self.assertEqual(errors, [])

    def test_missing_entrypoints_block(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            self.assertEqual(missing_generated_metadata(root),
                             ["packages/SConscript", "packages/pkgs.json"])
            self.assertFalse(package_tree_ready(root))

    def test_versioned_package_tree_is_accepted(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            packages = root / "packages"
            (packages / "CMSIS-Core-5.9.0").mkdir(parents=True)
            (packages / "stm32h7_cmsis_driver-v1.10.7").mkdir()
            (packages / "stm32h7_hal_driver-v1.11.6").mkdir()
            for name in ("CMSIS-Core-5.9.0", "stm32h7_cmsis_driver-v1.10.7",
                         "stm32h7_hal_driver-v1.11.6"):
                (packages / name / "SConscript").write_text("# generated\n",
                                                            encoding="utf-8")
            (packages / "SConscript").write_text("# generated\n", encoding="utf-8")
            (packages / "pkgs.json").write_text("[]\n", encoding="utf-8")
            self.assertEqual(missing_generated_metadata(root), [])
            self.assertTrue(package_tree_ready(root))
            self.assertEqual(len(installed_package_dirs(root)), 3)

    def test_missing_manifest_blocks(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            packages = root / "packages"
            packages.mkdir()
            (packages / "SConscript").write_text("# generated\n", encoding="utf-8")
            self.assertEqual(missing_generated_metadata(root), ["packages/pkgs.json"])

    def test_bridge_without_packages_blocks(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            packages = root / "packages"
            packages.mkdir()
            (packages / "SConscript").write_text("# generated\n", encoding="utf-8")
            (packages / "pkgs.json").write_text("[]\n", encoding="utf-8")
            self.assertEqual(missing_generated_metadata(root),
                             ["packages/<package>/SConscript"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
