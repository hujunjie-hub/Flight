"""Reject CRLF line endings in repository shell scripts.

Windows Git installations commonly set core.autocrlf=true. That rewrites every
*.sh file to CRLF on checkout, and bash then parses the CR as part of the next
word: `set -euo pipefail` becomes the invalid option `pipefail\\r`. All WSL and
Linux test entry points in this repository start with that line, so the failure
is immediate and looks like a broken test harness rather than an EOL problem.

The repository keeps a LF policy in .gitattributes. This check makes a
regression (or a fresh checkout that ignored the attribute) fail in CI instead
of failing later inside a test runner.
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
# Vendored third-party trees carry their own upstream .gitattributes.
EXCLUDED_PREFIXES = ('doc/ADIS16505/no-OS/',)
REPAIR = (
    'Repair a Windows checkout with:\n'
    '  git config core.autocrlf false\n'
    '  git update-index --refresh\n'
    '  git checkout -- .\n'
)


def main():
    offenders = []
    for path in sorted(ROOT.rglob('*.sh')):
        relative = path.relative_to(ROOT).as_posix()
        if any(relative.startswith(prefix) for prefix in EXCLUDED_PREFIXES):
            continue
        if b'\r\n' in path.read_bytes():
            offenders.append(relative)

    attributes = ROOT / '.gitattributes'
    if not attributes.is_file() or 'eol=lf' not in attributes.read_text(
            encoding='utf-8'):
        offenders.append('.gitattributes')

    if offenders:
        print('CRLF line endings are not allowed in shell scripts:')
        for item in offenders:
            print(f'  {item}')
        print(REPAIR, end='')
        return 1
    print('PASS: shell scripts use LF and .gitattributes pins eol=lf')
    return 0


if __name__ == '__main__':
    sys.exit(main())
