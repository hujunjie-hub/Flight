#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""烧录 DR->DMA 新固件并验证 ADIS16505 丢拍改善

对比指标:
  - dr_dma.missed / busy_skip / crc_err : 新 DMA 链路自身丢拍/异常
  - g_sol.drop_cnt : gins 桥接侧检测的 DR 丢拍 (旧轮询架构 ~15/s = 1.5%)
内存布局 (rtthread.map):
  dr_dma @0x24000F48 (0x28): prev_cntr u16, cntr_valid, isr_cnt, dma_done,
                             missed, busy_skip, crc_err, start_err, recover,
                             stale_warn
  g_sol  @0x2400A258: drop_cnt @ +0x6C, imu_cnt @ +0x58, gnss_cnt @ +0x5C
"""
import re
import sys
import time

sys.path.insert(0, r"D:\STM32Project\Flight\build_host")
from check_gins_ocd import Ocd, mdw_words, parse_sol, G_SOL, DWT_CYCCNT

DR_DMA = 0x24000F48
ELF = "D:/STM32Project/Flight/cmake-build-debug/Flight.elf"


def parse_dr(words):
    u32 = lambda off: words[off // 4]
    return {
        "isr_cnt": u32(0x08), "dma_done": u32(0x0C), "missed": u32(0x10),
        "busy_skip": u32(0x14), "crc_err": u32(0x18), "start_err": u32(0x1C),
        "recover": u32(0x20), "stale_warn": u32(0x24),
    }


def snapshot(ocd):
    dr = parse_dr(mdw_words(ocd, DR_DMA, 10))
    sol = parse_sol(mdw_words(ocd, G_SOL, 40))
    return dr, sol


def main():
    ocd = Ocd()

    print("[1] flash new firmware ...")
    print("    " + ocd.cmd("reset halt", wait=1.5).strip().replace("\n", " | "))
    print("    " + ocd.cmd(f"program {ELF} verify", wait=25.0).strip().splitlines()[-1])
    print("    " + ocd.cmd("reset run", wait=1.5).strip().replace("\n", " | "))

    print("[2] wait 15 s for init (reset delay + config + DR start) ...")
    time.sleep(15.0)

    cys = []
    for _ in range(3):
        c = mdw_words(ocd, DWT_CYCCNT, 1)
        if c:
            cys.append(c[0])
        time.sleep(0.25)
    if len(set(cys)) < 2:
        print("FAIL: core not running after reset")
        return 1
    print("    core running.")

    dr1, s1 = snapshot(ocd)
    print(f"[3] t0  : dr={dr1}")
    print(f"       sol: imu={s1['imu_cnt']} gnss={s1['gnss_cnt']} drop={s1['dr_drop']} ready={s1['ready']}")

    print("[4] measuring 60 s ...")
    time.sleep(60.0)

    dr2, s2 = snapshot(ocd)
    dt = s2["time"] - s1["time"]
    if dt <= 0:
        dt = 60.0
    rate = lambda a, b: (b - a) / dt
    print(f"    t+{dt:.0f}s: dr={dr2}")

    print()
    print(f"    imu_cnt   : {s1['imu_cnt']} -> {s2['imu_cnt']} ({rate(s1['imu_cnt'], s2['imu_cnt']):.1f} /s, expect ~1000)")
    print(f"    gnss_cnt  : {s1['gnss_cnt']} -> {s2['gnss_cnt']} ({rate(s1['gnss_cnt'], s2['gnss_cnt']):.1f} /s)")
    print(f"    gins drop : {s1['dr_drop']} -> {s2['dr_drop']} ({rate(s1['dr_drop'], s2['dr_drop']):.2f} /s)  <- 旧架构 ~15/s")
    print(f"    dr isr    : {dr2['isr_cnt']} ({rate(dr1['isr_cnt'], dr2['isr_cnt']):.1f} /s, expect ~1000)")
    print(f"    dma done  : {dr2['dma_done']} ({rate(dr1['dma_done'], dr2['dma_done']):.1f} /s)")
    print(f"    missed    : +{dr2['missed'] - dr1['missed']}")
    print(f"    busy_skip : +{dr2['busy_skip'] - dr1['busy_skip']}")
    print(f"    crc_err   : +{dr2['crc_err'] - dr1['crc_err']}")
    print(f"    start_err : +{dr2['start_err'] - dr1['start_err']}")
    print(f"    recover   : +{dr2['recover'] - dr1['recover']}")

    imu_rate = rate(s1["imu_cnt"], s2["imu_cnt"])
    dr_rate = rate(dr1["isr_cnt"], dr2["isr_cnt"])
    drops = rate(s1["dr_drop"], s2["dr_drop"])
    ok = (imu_rate > 900 and dr_rate > 900 and drops < 1.0
          and dr2["missed"] == dr1["missed"] and dr2["busy_skip"] == dr1["busy_skip"])
    print()
    print(">> " + ("PASS: DR->DMA 链路 1kHz 稳定, 无丢拍" if ok
                else f"WARN/FAIL: imu={imu_rate:.0f}/s dr_isr={dr_rate:.0f}/s drops={drops:.2f}/s"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
