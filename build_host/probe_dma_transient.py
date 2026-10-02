#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓 DMA 传输瞬态: 等 dma_busy=1 时读 DMA1 Stream0/1 寄存器"""
import sys
import time

sys.path.insert(0, r"D:\STM32Project\Flight\build_host")
from check_gins_ocd import Ocd, mdw_words

ADIS_DEV = 0x24000ED8
DMA1 = 0x40020000          # H7: DMA1 @ 0x40020000
S0 = DMA1 + 0x010          # Stream0 (SPI1_RX): CR NDTR PAR M0AR
S1 = DMA1 + 0x028          # Stream1 (SPI1_TX)


def main():
    ocd = Ocd()
    caught = 0
    for attempt in range(60):
        w = mdw_words(ocd, ADIS_DEV + 0x14, 1)
        if w and w[0] == 1:  # dma_busy
            cr0 = mdw_words(ocd, S0, 4)
            cr1 = mdw_words(ocd, S1, 4)
            print(f"[{attempt}] caught busy=1")
            if cr0:
                print(f"  RX: CR={cr0[0]:#010x} EN={cr0[0]&1} NDTR={cr0[1]} "
                      f"PAR={cr0[2]:#010x} M0AR={cr0[3]:#010x}")
            if cr1:
                print(f"  TX: CR={cr1[0]:#010x} EN={cr1[0]&1} NDTR={cr1[1]} "
                      f"PAR={cr1[2]:#010x} M0AR={cr1[3]:#010x}")
            caught += 1
            if caught >= 5:
                break
        time.sleep(0.01)
    if not caught:
        print("never caught busy=1 in 60 tries (openocd too slow?)")
    # 空闲时的寄存器 (帧间隙)
    cr0 = mdw_words(ocd, S0, 4)
    cr1 = mdw_words(ocd, S1, 4)
    print("idle:")
    if cr0:
        print(f"  RX: CR={cr0[0]:#010x} EN={cr0[0]&1} NDTR={cr0[1]} "
              f"PAR={cr0[2]:#010x} M0AR={cr0[3]:#010x}")
    if cr1:
        print(f"  TX: CR={cr1[0]:#010x} EN={cr1[0]&1} NDTR={cr1[1]} "
              f"PAR={cr1[2]:#010x} M0AR={cr1[3]:#010x}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
