#!/usr/bin/env python3
"""Regression tests for deterministic, read-only Flight parameter catalog generation."""
import tempfile
import unittest
from pathlib import Path

from tools.generate_ground_config import canonical_source_sha256, generate


class GroundConfigGeneratorTests(unittest.TestCase):
    def test_source_hash_is_independent_of_line_endings(self):
        lf = b"alpha\nbeta\ngamma\n"
        crlf = lf.replace(b"\n", b"\r\n")
        mixed = b"alpha\r\nbeta\ngamma\r\n"
        self.assertEqual(canonical_source_sha256(lf), canonical_source_sha256(crlf))
        self.assertEqual(canonical_source_sha256(lf), canonical_source_sha256(mixed))

    def test_generated_catalog_passes_read_only_check(self):
        with tempfile.TemporaryDirectory(prefix="flight-ground-config-test-") as temporary:
            output = Path(temporary)
            generate(output)
            generate(output, check=True)

    def test_stale_check_does_not_rewrite_outputs(self):
        with tempfile.TemporaryDirectory(prefix="flight-ground-config-stale-") as temporary:
            output = Path(temporary)
            generate(output)
            target = output / "ground_parameters.json"
            stale_bytes = target.read_bytes() + b"stale-marker\n"
            target.write_bytes(stale_bytes)
            with self.assertRaises(SystemExit):
                generate(output, check=True)
            self.assertEqual(target.read_bytes(), stale_bytes)


if __name__ == "__main__":
    unittest.main(verbosity=2)
