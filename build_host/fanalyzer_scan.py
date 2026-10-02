#!/usr/bin/env python3
"""GCC -fanalyzer static-analysis pass over project-owned C code.

Uses cmake-build-debug/compile_commands.json for exact per-unit flags.
Host-side tool only; not part of firmware build.
"""
import json
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
DB = os.path.join(HERE, '..', 'cmake-build-debug', 'compile_commands.json')

OWN = ('/middleware/', '/applications/', 'board/board.c', 'board/CubeMX_Config')


def main():
    db = json.load(open(DB))
    n = 0
    hits = 0
    for e in db:
        f = e['file'].replace('\\', '/')
        if not any(p in f for p in OWN):
            continue
        if f.endswith(('.cpp', '.cc')):
            continue
        cmd = e.get('arguments') or e['command'].split()
        new = [cmd[0]]
        skip = False
        for a in cmd[1:]:
            if skip:
                skip = False
                continue
            if a == '-o':
                skip = True
                continue
            if a == '-c' or a.startswith(('-O', '-g')):
                continue
            new.append(a)
        new += ['-fsyntax-only', '-fanalyzer']
        r = subprocess.run(new, capture_output=True, text=True,
                           cwd=e['directory'], errors='replace')
        for line in r.stderr.splitlines():
            if 'warning:' in line or 'error:' in line:
                hits += 1
                print(f.replace('D:/STM32Project/Flight/', '')[:60], '|', line[:220])
        n += 1
    print(f'units analyzed: {n}, findings: {hits}')


if __name__ == '__main__':
    main()
