#!/usr/bin/env python3
"""Audit rtconfig.h against the RT-Thread 5.3.0 tree actually in this repo.

Every functional #define in rtconfig.h must be referenced somewhere in
rt-thread/, libraries/, board/, middleware/ or applications/ (source code,
Kconfig or SConscript). Unreferenced macros are upgrade leftovers from an
older RT-Thread and are reported.

Known-noise prefixes (version meta, toolchain guards) are skipped.
Host-side tool only; not part of firmware build.
"""
import os
import re
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

# Directories whose text is scanned for references
SCAN_DIRS = ['rt-thread', 'libraries', 'board', 'middleware', 'applications', 'packages']
SCAN_EXT = ('.c', '.h', '.cpp', '.S', '.s', '.py', '.txt', '.cmake', '')

SKIP_PREFIX = ('RT_VER_NUM', 'RTTHREAD_VERSION', 'DEMO')
# function-like feature macros that only Kconfig menus know about
KNOWN_KCONFIG_ONLY = ()


def main():
    rtconfig = os.path.join(ROOT, 'rtconfig.h')
    macros = []
    for line in open(rtconfig, encoding='utf-8', errors='replace'):
        m = re.match(r'#define\s+([A-Za-z0-9_]+)(?:\s+(.*))?', line)
        if not m:
            continue
        name, val = m.group(1), (m.group(2) or '').strip()
        if name.startswith(SKIP_PREFIX):
            continue
        if val.startswith('RT_VER_NUM'):
            continue
        macros.append((name, val))

    # Build one big haystack of all scannable text
    haystack = []
    for d in SCAN_DIRS:
        base = os.path.join(ROOT, d)
        for dirpath, _dirs, files in os.walk(base):
            if 'cmake-build' in dirpath or os.sep + 'backup' in dirpath:
                continue
            for fn in files:
                p = os.path.join(dirpath, fn)
                try:
                    haystack.append(open(p, encoding='utf-8', errors='replace').read())
                except OSError:
                    pass
    # root-level build files too
    for fn in ('CMakeLists.txt', 'SConstruct', 'SConscript', 'Kconfig', 'rtconfig.py'):
        p = os.path.join(ROOT, fn)
        if os.path.exists(p):
            haystack.append(open(p, encoding='utf-8', errors='replace').read())
    big = '\n'.join(haystack)

    orphans = []
    for name, val in macros:
        # a macro is "referenced" if its bare name appears outside rtconfig.h
        n = big.count(name)
        if n == 0:
            orphans.append((name, val))

    print(f'rtconfig.h functional macros: {len(macros)}')
    print(f'unreferenced anywhere in tree: {len(orphans)}')
    for name, val in orphans:
        print(f'  {name} = {val[:50]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
