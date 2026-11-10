"""P1.5: `python -O` must not turn the spec audit into a silent pass.

The audit used `assert` for its acceptance checks; Python removes assertions
under -O, so a corrupted spec input could produce a PASS report and exit 0.
The checks now call audit_spec.require(), which always raises and writes
spec/validation/audit_failure.json.

This test runs the *real tool* with a temporarily corrupted input:

  1. positive control: the unmodified tree passes `--round 1`;
  2. the message registry is corrupted in place (original bytes restored in a
     finally block) and the audit must fail in normal mode;
  3. the same corrupted input must fail identically under `python -O`;
  4. the structured failure artifact must record status FAIL with a location.

The generated `tmpnegctl*` evidence files are removed afterwards so the working
tree is left exactly as it was (verified by the SHA256 of the registry).
"""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
REGISTRY = ROOT / 'spec/contracts/message_registry.json'
VALIDATION = ROOT / 'spec/validation'
# The audit regenerates these evidence files; snapshot and restore them so the
# working tree is unchanged even when the run fails.
TOUCHED = (REGISTRY, VALIDATION / 'abi_layout.json',
           VALIDATION / 'mathematical_reference_report.json')
LABEL = 'tmpnegctl'


def run_audit(optimized):
    command = [sys.executable]
    if optimized:
        command.append('-O')
    command += ['spec/tools/audit_spec.py', '--round', '1', '--label', LABEL]
    return subprocess.run(command, cwd=ROOT, capture_output=True, text=True)


def cleanup():
    for pattern in (f'{LABEL}_abi_layout.json', f'{LABEL}_report.json',
                    'audit_failure.json'):
        target = VALIDATION / pattern
        if target.is_file():
            target.unlink()


def main():
    originals = {path: path.read_bytes() for path in TOUCHED if path.is_file()}
    original = originals[REGISTRY]
    original_sha = hashlib.sha256(original).hexdigest()
    failures = []
    try:
        cleanup()
        positive = run_audit(optimized=False)
        if positive.returncode != 0 or '"status": "PASS"' not in positive.stdout:
            print('FAIL: positive control did not report PASS')
            print(positive.stdout + positive.stderr)
            return 1

        registry = json.loads(original)
        first = sorted(registry['messages'])[0]
        registry['messages'][first + '_CORRUPTED'] = registry['messages'].pop(first)
        REGISTRY.write_text(json.dumps(registry, indent=2) + '\n', encoding='utf-8')

        for label, optimized in (('normal', False), ('-O', True)):
            result = run_audit(optimized=optimized)
            output = result.stdout + result.stderr
            if result.returncode == 0:
                failures.append(f'corrupted audit passed under {label}')
                continue
            if 'SPEC_AUDIT_FAILED' not in output:
                failures.append(f'{label} failure was not a structured acceptance failure')
                continue
            artifact = VALIDATION / 'audit_failure.json'
            if not artifact.is_file():
                failures.append(f'{label} did not write the structured failure artifact')
                continue
            failure = json.loads(artifact.read_text(encoding='utf-8'))
            if (failure.get('status') != 'FAIL' or
                    'audit_spec.py' not in failure.get('file', '') or
                    not failure.get('line')):
                failures.append(f'{label} failure artifact is incomplete: {failure}')
            artifact.unlink()
    finally:
        for path, data in originals.items():
            path.write_bytes(data)
        cleanup()
        if hashlib.sha256(REGISTRY.read_bytes()).hexdigest() != original_sha:
            failures.append('message_registry.json was not restored')

    if failures:
        for failure in failures:
            print('FAIL:', failure)
        return 1
    print('M5_PYTHON_O_NEGATIVE_TEST_PASS '
          '(audit_spec.py fails closed in normal and -O modes, artifact written, '
          'input restored)')
    return 0


if __name__ == '__main__':
    sys.exit(main())

