"""FLIGHT-14 deterministic checks for the Release enum diagnostic gate."""
from hashlib import sha256
from pathlib import Path
import tempfile
import unittest

from tools.check_enum_link import check_enum_link


class EnumLinkTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.runtime = Path(temp.name, "GNU tools", "libc.a")
        self.runtime.parent.mkdir()
        self.runtime.write_bytes(b"reviewed runtime")
        self.member = f"{self.runtime.as_posix()}(strchr.o)"
        self.map_text = (
            "Archive member included to satisfy reference by file (symbol)\n\n"
            f"{self.member}\n    build/board.o (strchr)\n\n"
            f"LOAD {self.runtime.as_posix()}\nLOAD build/board.o\n"
            "\nCross Reference Table\n\nSymbol    File\n"
            f"strchr    {self.member}\n          build/board.o\n"
        )
        self.warning = (
            r"toolchain/bin/ld.exe: warning: build\board.o uses 32-bit enums yet "
            "the output is to use variable-size enums; use of enum values across objects may fail"
        )
        self.log = f"LINK Flight.elf\n{self.warning}\n"
        self.baseline = {
            "enum_warning_objects": ["build/board.o"],
            "runtime_loads": [{"file": "libc.a", "sha256": sha256(b"reviewed runtime").hexdigest()}],
            "runtime_members": ["libc.a(strchr.o)"],
            "runtime_boundary": ["strchr: libc.a(strchr.o)"],
        }

    def check(self):
        return check_enum_link(self.log, self.map_text, self.baseline)

    def test_reviewed_warning_and_runtime(self):
        self.log += "ld.exe: warning: unrelated linker diagnostic\n"
        report = self.check()
        self.assertEqual(report["status"], "PASS", report)
        self.assertEqual(report["enum_warning_count"], 1)

    def test_same_count_different_warning_object(self):
        self.log = self.log.replace(r"build\board.o", r"build\new.o")
        self.assertEqual(self.check()["status"], "FAIL")

    def test_additional_enum_warning(self):
        self.log += self.warning + "\n"
        self.assertEqual(self.check()["status"], "FAIL")

    def test_changed_runtime_bytes(self):
        self.runtime.write_bytes(b"different runtime")
        self.assertEqual(self.check()["status"], "FAIL")

    def test_changed_runtime_member(self):
        self.map_text = self.map_text.replace(self.member + "\n    build/board.o (strchr)",
                                              f"{self.runtime.as_posix()}(other.o)\n    build/board.o (strchr)")
        self.assertEqual(self.check()["status"], "FAIL")

    def test_changed_runtime_boundary(self):
        self.map_text = self.map_text.replace("strchr    ", "new_api   ")
        self.assertEqual(self.check()["status"], "FAIL")

    def test_missing_link_or_suppressed_diagnostic(self):
        for log in (self.warning, "LINK Flight.elf\n"):
            self.log = log
            self.assertEqual(self.check()["status"], "FAIL")

    def test_unrecognized_enum_diagnostic(self):
        self.log += "ld.exe: warning: link rejected an incompatible enum size\n"
        self.assertEqual(self.check()["status"], "FAIL")

    def test_missing_cross_reference_table(self):
        self.map_text = self.map_text.split("Cross Reference Table")[0]
        self.assertEqual(self.check()["status"], "FAIL")


if __name__ == "__main__":
    unittest.main()
