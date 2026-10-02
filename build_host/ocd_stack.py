#!/usr/bin/env python3
"""One-shot: halt, read PSP stack, scan for flash return addresses, resolve."""
import re
import subprocess
import time

from ocd_diag_rst import Ocd

ADDR2LINE = r"C:/env-windows/tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-addr2line.exe"
ELF = "../cmake-build-debug/Flight.elf"

RE_PC = re.compile(r'^pc\s+\([^)]*\):\s+(0x[0-9a-f]+)')
RE_PSP = re.compile(r'^psp\s+\([^)]*\):\s+(0x[0-9a-f]+)')


def resolve(addr):
    r = subprocess.run([ADDR2LINE, "-e", ELF, "-f", "-i", hex(addr)],
                       capture_output=True, text=True)
    lines = [l for l in r.stdout.splitlines() if l.strip()]
    return " <- ".join(lines[:4]) if lines else "?"


o = Ocd()
try:
    time.sleep(6)
    h = o.cmd("halt", 0.5)
    print(h)
    psp = pc = 0
    m = re.search(r'pc:\s+(0x[0-9a-f]+)', h)
    if m:
        pc = int(m.group(1), 16)
    m = re.search(r'psp:\s+(0x[0-9a-f]+)', h)
    if m:
        psp = int(m.group(1), 16)
    if pc == 0:
        regs = o.cmd("reg pc", 0.3)
        m = re.search(r'(0x[0-9a-f]+)', regs)
        pc = int(m.group(1), 16) if m else 0
    if psp == 0:
        regs = o.cmd("reg psp", 0.3)
        m = re.search(r'(0x[0-9a-f]+)', regs)
        psp = int(m.group(1), 16) if m else 0
    print(f"pc=0x{pc:08x}  psp=0x{psp:08x}")
    print("pc ->", resolve(pc))
    words = o.mdw(psp, 160)
    seen = []
    for w in words:
        if 0x08000000 <= w < 0x08100000:
            if not seen or w != seen[-1]:
                seen.append(w)
    print("\nstack return-address chain:")
    for a in seen[:14]:
        print(f"  0x{a:08x}  {resolve(a)}")
finally:
    o.close()
