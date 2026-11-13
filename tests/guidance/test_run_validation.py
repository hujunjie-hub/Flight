"""Exercise the local validation entrypoint's exit status and input checks."""

import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from subprocess import CompletedProcess
from unittest.mock import patch

import run_validation as validation


HEAD = "a" * 40


class ValidationRunnerTests(unittest.TestCase):
    def invoke(self, *arguments, test_exit=0):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "validation.json"
            argv = ["run_validation.py", "--run-id", "M5-06-RUNNER-TEST",
                    "--code-commit", HEAD, "--output", str(output), *arguments]
            with (patch.object(sys, "argv", argv),
                  patch.object(validation, "OUT", Path(directory)),
                  patch.object(validation, "REGEN", Path(directory)),
                  patch.object(validation, "ROOT", Path(directory)),
                  patch.object(validation.subprocess, "check_output", return_value=HEAD),
                  patch.object(validation, "toolchain_for", return_value="Python 3.14"),
                  patch.object(validation, "run", return_value=(
                      CompletedProcess([], test_exit, stdout="test output", stderr=""),
                      0.1)) as run,
                  contextlib.redirect_stdout(io.StringIO()),
                  contextlib.redirect_stderr(io.StringIO())):
                try:
                    result = validation.main()
                except SystemExit as error:
                    result = error.code
            report = json.loads(output.read_text(encoding="utf-8")) if output.exists() else None
            return result, report, run.call_count

    def test_rejects_unknown_selection_even_with_known_test(self):
        self.assertEqual(self.invoke("--only", "line_endings,does_not_exist"), (2, None, 0))

    def test_rejects_empty_selection(self):
        self.assertEqual(self.invoke("--only", ",,"), (2, None, 0))

    def test_rejects_nonhex_commit(self):
        self.assertEqual(self.invoke("--code-commit", "z" * 40), (2, None, 0))

    def test_rejects_commit_different_from_head(self):
        self.assertEqual(self.invoke("--code-commit", "b" * 40), (2, None, 0))

    def test_pass_returns_zero(self):
        self.check_result("line_endings", 0, 0, "PASS")

    def test_failed_command_returns_failure_and_writes_report(self):
        self.check_result("line_endings", 1, 1, "FAIL")

    def test_blocked_command_returns_blocked(self):
        self.check_result("scons_link", 2, 2, "BLOCKED")

    def test_unknown_stack_returns_unknown(self):
        self.check_result("stack_image", 3, 3, "UNKNOWN")

    def check_result(self, name, test_exit, expected_exit, status):
        exit_code, report, calls = self.invoke("--only", name, test_exit=test_exit)
        self.assertEqual(calls, 1)
        self.assertEqual(exit_code, expected_exit)
        self.assertEqual(report["status"], status)
        self.assertEqual(report["commands"][0]["status"], status)
        self.assertEqual(report["git_head"], HEAD)


if __name__ == "__main__":
    unittest.main(verbosity=2)
