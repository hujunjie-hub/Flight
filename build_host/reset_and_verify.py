#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""复位固件 -> 轮询 g_sol 直到 ready=1 -> 验证 gnss_cnt ~10Hz 增长 -> 观察稳定性"""
import re
import sys
import time

sys.path.insert(0, r"D:\STM32Project\Flight\build_host")
from check_gins_ocd import Ocd, mdw_words, parse_sol, G_SOL, DWT_CYCCNT


def core_running(ocd):
    cys = []
    for _ in range(3):
        c = mdw_words(ocd, DWT_CYCCNT, 1)
        if c:
            cys.append(c[0])
        time.sleep(0.25)
    return len(set(cys)) > 1


def main():
    ocd = Ocd()

    print("[1] reset run ...")
    ocd.cmd("reset run", wait=2.0)
    time.sleep(8.0)  # 等初始化 (LINE PROBE + 线程创建)
    print("    state:", "running" if core_running(ocd) else "NOT RUNNING")

    print("[2] polling g_sol for engine ready (max 300 s) ...")
    t0 = time.time()
    s_prev = None
    last = None
    while time.time() - t0 < 300:
        w = mdw_words(ocd, G_SOL, 40)
        if w:
            s = parse_sol(w)
            last = s
            if s["ready"] and s_prev and s["gnss_cnt"] > s_prev[1]:
                break
            if not s["ready"] and s_prev is None:
                print(f"    t+{time.time()-t0:5.1f}s ready=0 (对准/等定位中)")
                s_prev = (s["imu_cnt"], s["gnss_cnt"])
            elif s["ready"] and not s_prev:
                print(f"    t+{time.time()-t0:5.1f}s ready=1! imu={s['imu_cnt']} gnss={s['gnss_cnt']}")
                s_prev = (s["imu_cnt"], s["gnss_cnt"])
        time.sleep(5.0)
    else:
        print("    TIMEOUT waiting for ready. last:", last)
        return 1

    print("[3] engine ready, measuring feed rates over 12 s ...")
    w1 = mdw_words(ocd, G_SOL, 40)
    s1 = parse_sol(w1)
    time.sleep(12.0)
    w2 = mdw_words(ocd, G_SOL, 40)
    s2 = parse_sol(w2)
    dt = s2["time"] - s1["time"]
    r = lambda a, b: (b - a) / dt if dt > 0 else 0
    print(f"    ready={s2['ready']} dt={dt:.1f}s")
    print(f"    imu_cnt : {s1['imu_cnt']} -> {s2['imu_cnt']} ({r(s1['imu_cnt'], s2['imu_cnt']):.0f} /s)")
    print(f"    gnss_cnt: {s1['gnss_cnt']} -> {s2['gnss_cnt']} ({r(s1['gnss_cnt'], s2['gnss_cnt']):.1f} /s)   <- UM982 -> KF-GINS")
    print(f"    mag_cnt : {s1['mag_cnt']} -> {s2['mag_cnt']} ({r(s1['mag_cnt'], s2['mag_cnt']):.0f} /s)")
    print(f"    pos     : lat={s2['lat']:.7f} lon={s2['lon']:.7f} alt={s2['alt']:.2f}")
    print(f"    att     : r={s2['roll']:.2f} p={s2['pitch']:.2f} y={s2['yaw']:.2f}")

    print("[4] stability watch 120 s ...")
    time.sleep(120)
    w3 = mdw_words(ocd, G_SOL, 40)
    running = core_running(ocd)
    if not w3 or not running:
        print("    !! core NOT running after 120 s — crash reproduced")
        return 1
    s3 = parse_sol(w3)
    dt2 = s3["time"] - s2["time"]
    print(f"    alive. gnss_cnt {s2['gnss_cnt']} -> {s3['gnss_cnt']} "
          f"({(s3['gnss_cnt']-s2['gnss_cnt'])/dt2:.1f} /s), "
          f"gnss_age={s3['gnss_age_ms']}ms, imu_rate="
          f"{(s3['imu_cnt']-s2['imu_cnt'])/dt2:.0f} /s")
    ok = (s3["gnss_cnt"] - s2["gnss_cnt"]) / dt2 > 5
    print("\n>> " + ("PASS: UM982 -> KF-GINS 链路正常, gnss_cnt 以 ~10Hz 持续增长"
                    if ok else "FAIL/WARN: 链路异常, 见上方数据"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
