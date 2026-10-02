#!/usr/bin/env python3
"""Strict-warning rescan of project-owned code using compile_commands.json.

Re-runs each compile with -fsyntax-only plus a stricter warning set and
prints a deduplicated warning report. Host-side tool only; not part of
firmware build.
"""
import json
import os
import re
import subprocess
import sys

DB = os.path.join(os.path.dirname(__file__), '..', 'cmake-build-debug', 'compile_commands.json')

# Project-owned code (rt-thread/, packages/, KF-GINS upstream are excluded)
OWN_PATTERNS = ('/middleware/', '/applications/', 'libraries/HAL_Drivers', 'board/board.c',
                'board/CubeMX_Config')

# KF-GINS/Eigen are upstream C++ (minimal embedded port); C-only strict pass
SKIP_SUFFIX = ('.cpp', '.cc', '.cxx')

EXTRA_FLAGS = [
    '-Wshadow', '-Wdouble-promotion', '-Wformat=2', '-Wundef',
    '-Wlogical-op', '-Wwrite-strings', '-Wcast-align',
    '-Wmissing-prototypes', '-Wredundant-decls', '-Wswitch-default',
]


def main():
    db = json.load(open(DB))
    own = []
    for e in db:
        f = e['file'].replace('\\', '/')
        if not any(p in f for p in OWN_PATTERNS):
            continue
        if f.endswith(SKIP_SUFFIX):
            continue
        own.append(e)

    print(f'scanning {len(own)} own-code C units', file=sys.stderr)

    # Normalize command: argv list already available in compile_commands
    report = {}
    for e in own:
        cmd = e.get('arguments')
        if cmd is None:
            cmd = e['command'].split()
        # strip -o output and any -c/-O*/-g flags; add syntax-only + strict flags
        new = [cmd[0]]
        skip_next = False
        for a in cmd[1:]:
            if skip_next:
                skip_next = False
                continue
            if a == '-o':
                skip_next = True
                continue
            if a == '-c' or a.startswith('-O') or a.startswith('-g') or a == '-MD':
                continue
            new.append(a)
        new += ['-fsyntax-only'] + EXTRA_FLAGS
        r = subprocess.run(new, capture_output=True, text=True, cwd=e['directory'],
                           errors='replace')
        out = r.stderr
        for line in out.splitlines():
            m = re.match(r'(.+?):(\d+):(\d+)?\s*warning:\s*(.+)', line)
            if m:
                key = (m.group(1), m.group(4))
                report.setdefault(key, []).append((m.group(1), m.group(2), m.group(4)))
            elif 'error:' in line:
                report.setdefault(('ERR', line), []).append(('ERR', '0', line))

    # Print sorted by file
    def normfile(p):
        return p.replace('\\', '/').split('Flight/')[-1]

    seen = set()
    rows = []
    for key, occs in report.items():
        f, w = key
        f = normfile(f)
        if (f, w) in seen:
            continue
        seen.add((f, w))
        rows.append((f, occs[0][1], w, len(occs)))
    rows.sort()
    for f, ln, w, n in rows:
        print(f'{f}:{ln}: {w}  [x{n}]')
    print(f'\ntotal distinct warnings: {len(rows)}')


if __name__ == '__main__':
    main()
