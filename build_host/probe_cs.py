#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓 busy 瞬间的 GPIOC ODRAW: CS=PC4 应为 0 (拉低)"""
import sys
import time

sys.path.insert(0, r"D:\STM32Project\Flight\build_host")
from check_gins_ocd import Ocd, mdw_words

ADIS_DEV = 0x24000ED8
GPIOC = 0x58024400
MODER = GPIOC + 0x00
IDR = GPIOC + 0x10
ODR = GPIOC + 0x14


def main():
    ocd = Ocd()

    mod = mdw_words(ocd, MODER, 1)
    print(f"GPIOC MODER = {mod[0]:#010x}" if mod else "mod read fail")
    # PC4 = bits[9:8]; 01=output, 00=input, 10=alt
    if mod:
        m = (mod[0] >> 8) & 0x3
        print(f"PC4 mode = {m:02b} ({'output' if m == 1 else 'input' if m == 0 else 'alt' if m == 2 else 'analog'})")

    busy_cs = []
    idle_cs = []
    for i in range(80):
        w = mdw_words(ocd, ADIS_DEV + 0x14, 1)
        if w and w[0] == 1:
            od = mdw_words(ocd, ODR, 1)
            idr = mdw_words(ocd, IDR, 1)
            if od and idr:
                busy_cs.append(((od[0] >> 4) & 1, (idr[0] >> 4) & 1))
            if len(busy_cs) >= 6:
                break
        else:
            od = mdw_words(ocd, ODR, 1)
            if od and len(idle_cs) < 3:
                idle_cs.append((od[0] >> 4) & 1)
        time.sleep(0.005)

    print(f"busy 瞬间 (ODR_bit4, IDR_bit4): {busy_cs}")
    print(f"idle ODR_bit4: {idle_cs}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
