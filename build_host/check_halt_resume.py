#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查 halt 位置 (PC), resume 内核, 再读 g_sol 验证 UM982 -> KF-GINS"""
import re
import sys
import time

sys.path.insert(0, r"D:\STM32Project\Flight\build_host")
from check_gins_ocd import Ocd, mdw_words, parse_sol, G_SOL, DWT_CYCCNT


def pc_symbol(pc, mapfile=r"D:\STM32Project\Flight\cmake-build-debug\rtthread.map"):
    """在 .map 里找 <= pc 的最近符号 (粗定位)"""
    best = ("?", 0)
    try:
        txt = open(mapfile, "r", errors="replace").read()
        for m in re.finditer(r"^\s*(0x[0-9a-fA-F]{8})\s+\S+\s+(\S+)$", txt, re.M):
            a = int(m.group(1), 16)
            if a <= pc and a > best[1]:
                best = (m.group(2), a)
    except OSError:
        pass
    return best


def main():
    ocd = Ocd()

    st = ocd.cmd("targets")
    halted = "halted" in st.split("stm32h7x.cpu0")[-1]
    print(f"[0] cpu0 state: {'halted' if halted else 'running'}")

    if halted:
        regs = ocd.cmd("reg pc")
        m = re.search(r"pc: (0x[0-9a-fA-F]+)", regs)
        pc = int(m.group(1), 16) if m else 0
        sym, base = pc_symbol(pc)
        print(f"[1] halted at PC={pc:#x}  (nearest symbol: {sym} @ {base:#x})")
        # fault 状态寄存器: 判断是否 HardFault
        cfsr = mdw_words(ocd, 0xE000ED28, 1)
        hfsr = mdw_words(ocd, 0xE000ED2C, 1)
        if cfsr and hfsr:
            print(f"    CFSR={cfsr[0]:#010x} HFSR={hfsr[0]:#010x} "
                  f"({'FORCED/HARDFAULT!' if hfsr[0] & 1 else 'no hardfault flag'})")

        print("[2] resume ...")
        print(ocd.cmd("resume", wait=1.0).strip())
        time.sleep(3.0)

    st = ocd.cmd("targets")
    state = st.split("stm32h7x.cpu0")[-1]
    print(f"[3] cpu0 state now: {'halted' if 'halted' in state else 'running'}")

    # CYCCNT 复查
    cys = []
    for _ in range(3):
        c = mdw_words(ocd, DWT_CYCCNT, 1)
        if c:
            cys.append(c[0])
        time.sleep(0.3)
    running = len(set(cys)) > 1
    print(f"[4] CYCCNT: {[hex(c) for c in cys]} -> {'RUNNING' if running else 'STILL FROZEN'}")
    if not running:
        return 1

    # g_sol 两次采样
    w1 = mdw_words(ocd, G_SOL, 40)
    if not w1:
        print("FAIL: g_sol 读取失败")
        return 1
    s1 = parse_sol(w1)
    print("[5] waiting 6 s ...")
    time.sleep(6.0)
    w2 = mdw_words(ocd, G_SOL, 40)
    if not w2:
        print("FAIL: g_sol 二次读取失败")
        return 1
    s2 = parse_sol(w2)

    dt = s2["time"] - s1["time"]
    rate = lambda a, b: (b - a) / dt if dt > 0 else 0.0
    print(f"\n[6] 引擎时间: {s1['time']:.3f} -> {s2['time']:.3f} (dt={dt:.3f} s)")
    print(f"    ready      : {s2['ready']} (1=RUNNING, 0=ALIGN/WAIT)")
    print(f"    imu_cnt    : {s1['imu_cnt']} -> {s2['imu_cnt']} ({rate(s1['imu_cnt'], s2['imu_cnt']):.0f} /s, expect ~1000)")
    print(f"    gnss_cnt   : {s1['gnss_cnt']} -> {s2['gnss_cnt']} ({rate(s1['gnss_cnt'], s2['gnss_cnt']):.1f} /s, expect ~10)")
    print(f"    gnss_stale : {s2['gnss_stale']}  gnss_age_ms: {s2['gnss_age_ms']}")
    print(f"    cov_warn   : {s2['cov_warn']}  dr_drop: {s2['dr_drop']}")
    print(f"    mag_cnt    : {s2['mag_cnt']}  baro_cnt: {s2['baro_cnt']}")
    print(f"    position   : lat={s2['lat']:.7f} lon={s2['lon']:.7f} alt={s2['alt']:.2f}")
    print(f"    attitude   : r={s2['roll']:.2f} p={s2['pitch']:.2f} y={s2['yaw']:.2f} deg")

    print()
    if s2["gnss_cnt"] > 0 and rate(s1["gnss_cnt"], s2["gnss_cnt"]) > 5:
        print(">> PASS: UM982 观测正以 ~10Hz 持续喂入 KF-GINS (gnss_cnt 增长中)")
    elif s2["gnss_cnt"] > 0:
        print(f">> WARN: 有历史 GNSS 观测但当前速率异常 ({rate(s1['gnss_cnt'], s2['gnss_cnt']):.1f}/s)")
    elif rate(s1["imu_cnt"], s2["imu_cnt"]) > 500:
        print(">> FAIL: IMU 在喂但 gnss_cnt 无增长 —— GNSS 观测未进入引擎")
    else:
        print(">> FAIL: IMU 与 GNSS 均无喂入, 引擎未初始化或传感器无数据")
    return 0


if __name__ == "__main__":
    sys.exit(main())
