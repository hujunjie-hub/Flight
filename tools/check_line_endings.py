"""Reject (or repair) CRLF line endings in repository shell scripts.

Windows Git installations commonly set core.autocrlf=true. That rewrites every
*.sh file to CRLF on checkout, and bash then parses the CR as part of the next
word: `set -euo pipefail` becomes the invalid option `pipefail\\r`. All WSL and
Linux test entry points in this repository start with that line, so the failure
is immediate and looks like a broken test harness rather than an EOL problem.

The repository keeps a LF policy in .gitattributes. This check makes a
regression (or an old checkout that ignored the attribute) fail in CI instead
of failing later inside a test runner.

`--fix` rewrites only the affected *.sh files in place (CRLF -> LF). It never
runs git reset, checkout, clean or stash, so unrelated local edits are kept.
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
# Vendored third-party trees carry their own upstream .gitattributes. `build/`
# is generated/scratch and may hold an external toolchain virtualenv (for
# example the RT-Thread Env used by the firmware link), so it is not source.
EXCLUDED_PREFIXES = ('doc/ADIS16505/no-OS/', 'build/')
REPAIR = (
    'Repair without touching other files:\n'
    '  python tools/check_line_endings.py --fix\n'
)


def scan():
    offenders = []
    for path in sorted(ROOT.rglob('*.sh')):
        relative = path.relative_to(ROOT).as_posix()
        if any(relative.startswith(prefix) for prefix in EXCLUDED_PREFIXES):
            continue
        if b'\r\n' in path.read_bytes():
            offenders.append(path)
    return offenders


def main():
    fix = '--fix' in sys.argv[1:]
    offenders = scan()
    attributes = ROOT / '.gitattributes'
    policy_missing = (not attributes.is_file() or
                      'eol=lf' not in attributes.read_text(encoding='utf-8'))

    if fix and offenders:
        for path in offenders:
            data = path.read_bytes().replace(b'\r\n', b'\n')
            path.write_bytes(data)
        print(f'CONVERTED {len(offenders)} shell script(s) to LF')
        for path in offenders:
            print(f'  {path.relative_to(ROOT).as_posix()}')
        offenders = scan()

    if offenders or policy_missing:
        print('CRLF line endings are not allowed in shell scripts:')
        for path in offenders:
            print(f'  {path.relative_to(ROOT).as_posix()}')
        if policy_missing:
            print('  .gitattributes (missing eol=lf policy)')
        print(REPAIR, end='')
        return 1
    print('PASS: shell scripts use LF and .gitattributes pins eol=lf')
    return 0


if __name__ == '__main__':
    sys.exit(main())
