#!/usr/bin/env python3
"""One-shot: diagnose mag link (full st) + gins thread state after fix."""
import re
import subprocess
import time

from ocd_diag_rst import Ocd

ADDR2LINE = r"C:/env-windows/tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-addr2line.exe"
ELF = "../cmake-build-debug/Flight.elf"
MAGCTX_ST = 0x24006188 + 0x50      # running,pushed,popped,lost,errors
MAGCTX_LAST = 0x24006188 + 0x68


def resolve(addr):
    r = subprocess.run([ADDR2LINE, "-e", ELF, "-f", "-i", hex(addr)],
                       capture_output=True, text=True)
    return " <- ".join([l for l in r.stdout.splitlines() if l.strip()][:3])


o = Ocd()
try:
    time.sleep(8)
    h = o.cmd("halt", 0.5)
    m = re.search(r'pc:\s+(0x[0-9a-f]+)', h)
    pc = int(m.group(1), 16) if m else 0
    m = re.search(r'psp:\s+(0x[0-9a-f]+)', h)
    psp = int(m.group(1), 16) if m else 0
    print(f"pc=0x{pc:08x} -> {resolve(pc)}")
    st = o.mdw(MAGCTX_ST, 5)
    print(f"mag st: run={st[0]} pushed={st[1]} popped={st[2]} "
          f"lost={st[3]} errors={st[4]}")
    last = o.mdw(MAGCTX_LAST, 10)
    print("mag last (raw+cal floats):")
    import struct
    for off in (0, 4):
        pass
    words = last
    # T_event(2w) + mag[3]f + cal[3]f + quality
    f = lambda i: struct.unpack("<f", struct.pack("<I", words[i]))[0]
    print(f"  T=0x{words[0]:08x}{words[1]:08x}  mag=({f(2):.2f},{f(3):.2f},{f(4):.2f})"
          f"  cal=({f(5):.2f},{f(6):.2f},{f(7):.2f})  q=0x{words[8]:x}")
    # stack chain
    sw = o.mdw(psp, 120)
    seen = []
    for w in sw:
        if 0x08000000 <= w < 0x08100000 and (not seen or w != seen[-1]):
            seen.append(w)
    print("stack chain:")
    for a in seen[:10]:
        print(f"  0x{a:08x}  {resolve(a)}")
finally:
    o.close()
