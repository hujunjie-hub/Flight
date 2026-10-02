#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HardFault 取证: 全寄存器 + MSP 栈帧 + 崩溃线程栈回溯线索"""
import re
import sys
import time

sys.path.insert(0, r"D:\STM32Project\Flight\build_host")
from check_gins_ocd import Ocd, mdw_words

MAP = r"D:\STM32Project\Flight\cmake-build-debug\rtthread.map"

_symtab = None
def sym_of(addr):
    global _symtab
    if _symtab is None:
        _symtab = []
        txt = open(MAP, "r", errors="replace").read()
        for m in re.finditer(r"^\s*(0x[0-9a-fA-F]{8})\s+0x[0-9a-f]+\s+(\S+)$", txt, re.M):
            _symtab.append((int(m.group(1), 16), m.group(2)))
        _symtab.sort()
    lo, hi = 0, len(_symtab)
    while lo < hi:
        mid = (lo + hi) // 2
        if _symtab[mid][0] <= addr:
            lo = mid + 1
        else:
            hi = mid
    if lo == 0:
        return "?"
    a, s = _symtab[lo - 1]
    return f"{s}+{addr - a:#x}" if addr > a else s


def main():
    ocd = Ocd()

    print("== 全寄存器 ==")
    regs = ocd.cmd("reg", wait=1.2)
    keep = {}
    for line in regs.splitlines():
        m = re.match(r"\s*(\w+): (0x[0-9a-fA-F]+)", line)
        if m:
            keep[m.group(1)] = int(m.group(2), 16)
            print("   " + line.rstrip())
    for rn in ("pc", "lr", "sp", "msp", "psp", "xpsr"):
        if rn in keep:
            print(f"   {rn} = {keep[rn]:#010x} -> {sym_of(keep[rn])}")

    msp = keep.get("msp", 0)
    psp = keep.get("psp", 0)

    for name, sp in (("MSP", msp), ("PSP", psp)):
        if not sp:
            continue
        print(f"\n== {name} 栈顶 64 字 ({sp:#x}) ==")
        words = mdw_words(ocd, sp, 64)
        if not words:
            print("   读取失败")
            continue
        for i in range(0, len(words), 4):
            row = words[i:i + 4]
            cells = []
            for w in row:
                note = ""
                if 0x08000000 <= w < 0x08200000:
                    note = " " + sym_of(w)
                cells.append(f"{w:08x}{note}")
            print(f"   {sp + i*4:#010x}: " + "  ".join(cells))

    # Cortex-M 异常栈帧 (basic frame): R0,R1,R2,R3,R12,LR,PC,xPSR
    # handler 先用 MSP; RT-Thread fault handler 保存上下文后再打印,
    # 原 fault 的栈帧通常在 MSP/PSP 当前值之上 (handler 已推栈)。
    for name, sp in (("MSP", msp), ("PSP", psp)):
        if not sp:
            continue
        print(f"\n== {name} 栈帧搜索: 在 {name} 附近找看似异常帧的 (R0-R3,R12,LR,PC,xPSR) ==")
        words = mdw_words(ocd, sp, 128)
        if not words:
            continue
        for off in range(0, 100):
            w = words[off:off + 8]
            if len(w) < 8:
                break
            pc, xpsr = w[6], w[7]
            lr = w[5]
            if (0x08000000 <= pc < 0x08200000 and (xpsr & 0xFF) in (0x03, 0x0B, 0x12)  # Handler/Thread+Thumb
                    and 0x08000000 <= lr < 0x08200000):
                print(f"   @{sp + off*4:#x} (sp+{off*4:#x}): LR={lr:#x}({sym_of(lr)}) "
                      f"PC={pc:#x}({sym_of(pc)}) xPSR={xpsr:#x}")

    print("\n== g_sol 当前值 (崩溃前状态) ==")
    from check_gins_ocd import G_SOL, parse_sol
    w = mdw_words(ocd, G_SOL, 40)
    if w:
        s = parse_sol(w)
        for k, v in s.items():
            print(f"   {k:12s}: {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
