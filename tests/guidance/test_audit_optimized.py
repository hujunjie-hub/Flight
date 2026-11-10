"""P1.4: `python -O` must not turn the spec audit into a silent pass.

audit_spec.py used `assert` for the spec dialect evidence check. Python removes
assertions under -O, so a stale or forged evidence file would silently pass the
acceptance audit. The check now calls audit_spec.require(); this test proves the
guard survives optimisation.
"""
from pathlib import Path
import subprocess
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
PROBE = (
    "import sys; sys.path.insert(0, 'spec/tools'); import audit_spec; "
    "audit_spec.require(False, 'negative control')"
)


def main():
    result = subprocess.run([sys.executable, '-O', '-c', PROBE], cwd=ROOT,
                            capture_output=True, text=True)
    output = result.stdout + result.stderr
    if result.returncode == 0:
        print('FAIL: audit_spec.require() passed under python -O')
        return 1
    if 'SPEC_AUDIT_FAILED' not in output or 'negative control' not in output:
        print('FAIL: unexpected output from the -O negative control:')
        print(output)
        return 1

    passing = subprocess.run(
        [sys.executable, '-O', '-c',
         "import sys; sys.path.insert(0, 'spec/tools'); import audit_spec; "
         "audit_spec.require(True, 'positive control')"],
        cwd=ROOT, capture_output=True, text=True)
    if passing.returncode != 0:
        print('FAIL: audit_spec.require(True) rejected a valid condition')
        print(passing.stdout + passing.stderr)
        return 1
    print('M5_PYTHON_O_NEGATIVE_TEST_PASS (audit_spec.require is not stripped by -O)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
