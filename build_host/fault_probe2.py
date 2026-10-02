#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HardFault 补充取证: reg 原始输出 + 异常栈帧定位"""
import re
import sys

sys.path.insert(0, r"D:\STM32Project\Flight\build_host")
from check_gins_ocd import Ocd, mdw_words
from fault_probe import sym_of


def main():
    ocd = Ocd()

    raw = ocd.cmd("reg", wait=1.5)
    print("== reg RAW ==")
    print(raw)
    print()

    # 兼容不同格式解析
    vals = {}
    for m in re.finditer(r"(\w+)\s*=?\s*(0x[0-9a-fA-F]{1,8})", raw):
        vals.setdefault(m.group(1), int(m.group(2), 16))

    for rn in ("pc", "lr", "sp", "msp", "psp"):
        if rn in vals:
            print(f"{rn} = {vals[rn]:#010x} -> {sym_of(vals[rn])}")

    msp = vals.get("msp", 0)
    psp = vals.get("psp", 0)

    for name, sp in (("MSP", msp), ("PSP", psp)):
        if not sp:
            continue
        print(f"\n== {name} @ {sp:#x} ==")
        words = mdw_words(ocd, sp, 96)
        if not words:
            print("  read fail")
            continue
        for i in range(0, len(words), 4):
            row = words[i:i + 4]
            cells = []
            for w in row:
                if 0x08000000 <= w < 0x08200000:
                    cells.append(f"{w:08x}[{sym_of(w)}]")
                elif 0x24000000 <= w < 0x24080000:
                    cells.append(f"{w:08x}(ram)")
                else:
                    cells.append(f"{w:08x}")
            print(f"  {sp + i*4:#010x}: " + " ".join(cells))
    return 0


if __name__ == "__main__":
    sys.exit(main())
