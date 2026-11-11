#!/usr/bin/env python3
"""The BSP package guard must accept versioned packages, not only `latest`."""
import tempfile
import unittest
from pathlib import Path

from tools.check_rtt_package_tree import (
    missing_generated_metadata,
    package_tree_ready,
)


class RtThreadPackageTreeTests(unittest.TestCase):
    def test_missing_entrypoints_block(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            self.assertEqual(missing_generated_metadata(root),
                             ["packages/SConscript", "packages/Kconfig"])
            self.assertFalse(package_tree_ready(root))

    def test_versioned_package_tree_is_accepted(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            packages = root / "packages"
            (packages / "CMSIS-Core-5.9.0").mkdir(parents=True)
            (packages / "stm32h7_cmsis_driver-v1.10.7").mkdir()
            (packages / "stm32h7_hal_driver-v1.11.6").mkdir()
            (packages / "SConscript").write_text("# generated\n", encoding="utf-8")
            (packages / "Kconfig").write_text("# generated\n", encoding="utf-8")
            self.assertEqual(missing_generated_metadata(root), [])
            self.assertTrue(package_tree_ready(root))

    def test_missing_one_entrypoint_blocks(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            packages = root / "packages"
            packages.mkdir()
            (packages / "SConscript").write_text("# generated\n", encoding="utf-8")
            self.assertEqual(missing_generated_metadata(root), ["packages/Kconfig"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
