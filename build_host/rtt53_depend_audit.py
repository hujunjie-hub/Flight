#!/usr/bin/env python3
"""Verify every GetDepend() symbol used by SConscripts is a real Kconfig
option in the RT-Thread 5.3.0 tree (+ board Kconfig + rtconfig.h).

A GetDepend on an undefined symbol silently evaluates false and silently
drops source files from the build. Host-side tool only.
"""
import os
import re

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

# collect all Kconfig text
kconfig_text = []
for base in ('rt-thread', 'board', 'libraries'):
    for dirpath, _dirs, files in os.walk(os.path.join(ROOT, base)):
        if 'backup' in dirpath:
            continue
        for fn in files:
            if fn == 'Kconfig':
                kconfig_text.append(open(os.path.join(dirpath, fn),
                                         encoding='utf-8', errors='replace').read())
kc = '\n'.join(kconfig_text)
defined = set(re.findall(r'^\s*(?:menu)?config\s+([A-Za-z0-9_]+)', kc, re.M))
# also symbols selectable via 'select'
selected = set(re.findall(r'^\s*select\s+([A-Za-z0-9_]+)', kc, re.M))

# rtconfig.h macro set (manually-set values without Kconfig entry)
rtcfg = open(os.path.join(ROOT, 'rtconfig.h'), encoding='utf-8', errors='replace').read()
rtmacros = set(re.findall(r'^#define\s+([A-Za-z0-9_]+)', rtcfg, re.M))

# collect GetDepend symbols from all SConscript + CMakeLists RTT_IS_ENABLED
deps = {}
for dirpath, _dirs, files in os.walk(ROOT):
    rel = os.path.relpath(dirpath, ROOT)
    if rel.split(os.sep)[0] in ('backup', 'cmake-build-debug', 'build', 'doc',
                                'dist', 'rt-thread', 'packages'):
        continue
    for fn in files:
        if fn in ('SConscript', 'SConstruct', 'CMakeLists.txt'):
            p = os.path.join(dirpath, fn)
            t = open(p, encoding='utf-8', errors='replace').read()
            for m in re.finditer(r'GetDepend\(\s*[\[\'\"]+([A-Za-z0-9_]+)', t):
                deps.setdefault(m.group(1), set()).add(os.path.relpath(p, ROOT))
            for m in re.finditer(r'RTT_IS_ENABLED\(\s*([A-Za-z0-9_]+)', t):
                deps.setdefault(m.group(1), set()).add(os.path.relpath(p, ROOT))

print(f'Kconfig-defined options: {len(defined)}')
print(f'GetDepend/RTT_IS_ENABLED symbols used: {len(deps)}')
missing = {s: f for s, f in sorted(deps.items())
           if s not in defined and s not in selected and s not in rtmacros}
if missing:
    print('\nUNRESOLVED symbols (never true — silent file exclusion risk):')
    for s, f in missing.items():
        print(f'  {s}: used by {sorted(f)}')
else:
    print('all symbols resolve to Kconfig config/select or rtconfig.h macros')
