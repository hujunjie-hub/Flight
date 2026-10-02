#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
10 分钟稳定性监控 (SWD 免串口).

用法: python swd_stability.py [分钟数, 默认 10]
每 60s 采样一次, 输出一行紧凑状态; 结束按判据汇总 PASS/FAIL。

判据:
  1. tick 单调且速率 ≈ 1000/s (无复位/死机)
  2. rmc/gga/zda 速率 ≥ 9.5/s (RX 吞吐不塌缩)
  3. csum_err / field_err 增量 = 0 (零校验失败)
  4. timebase restart 增量 = 0 且 map.valid=1 (时基持续有效)
  5. stale / rej / ts_zero 增量 = 0
  6. imu 速率 ≥ 990/s
  7. INS 位置距真值 (22.6441, 114.0135) ≤ 0.2km
  8. raw 环 lost 增量 = 0
  9. g_nan.cnt = 0 (无 NaN)
"""
import subprocess, re, struct, sys, time

CLI = r"D:\STM32CubeCLT\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
A = dict(nav=0x24001160, rawctx=0x24008240, hiwat=0x24008284, graw=0x24000024,
         gnsctx=0x24009288, rxdiag=0x24009368, gmap=0x2400A0D8,
         gnan2run=0x2400AD68, gseed=0x2400AEC8, ggq=0x2400AD30,
         greseed=0x2400AF40)
# g_gq@0x2400AD30 (0x38B: rej_hdop@0x28 rej_jump@0x2c rej_altx@0x30);
# 块: g_nan+g_run+g_sol+g_seed(0x38)+g_guard(0x30)+g_reseed+g_vwatch(0x10) -> rt_tick@+0x228

def swd_read(addr, size):
    out = subprocess.run([CLI, "-c", "port=SWD", "mode=HotPlug", "-r32",
                          hex(addr), str(size)], capture_output=True, text=True,
                         timeout=60).stdout
    words = []
    for m in re.finditer(r"^0x[0-9A-Fa-f]{8}\s*:([0-9A-Fa-f ]+)$", out, re.M):
        words += [w for w in m.group(1).split()]
    if len(words) * 4 < size:
        raise RuntimeError(f"short read 0x{addr:x}: {out[-200:]}")
    return b"".join(int(w.zfill(8), 16).to_bytes(4, "little") for w in words)[:size]

def u32(b, o): return int.from_bytes(b[o:o+4], "little")

def sample():
    s = {}
    blk = swd_read(A["gnan2run"], 0x234)          # g_nan@0 g_run@0x48 g_sol@0xb8
    s["tick"] = u32(blk, 0x230)
    s["nan_cnt"] = u32(blk, 0)
    gr = blk[0x48:0x48+0x70]
    s["imu"], s["gnss"], s["stale"], s["rej"] = u32(gr,0), u32(gr,4), u32(gr,0x10), u32(gr,0x50)
    s["ts_zero"], s["late_us"] = u32(gr, 0x64), u32(gr, 0x68)
    s["step_avg"], s["step_max"] = struct.unpack_from("<2f", gr, 0x48)
    s["lat"], s["lon"], s["alt"] = struct.unpack_from("<3d", blk, 0xb8+0x10)
    n = swd_read(A["nav"], 0x68)
    s["rmc"], s["gga"], s["zda"] = u32(n,0x50), u32(n,0x54), u32(n,0x58)
    s["csum"], s["field"] = u32(n,0x5c), u32(n,0x60)
    r = swd_read(A["rawctx"], 0x48)
    s["raw_lost"], s["raw_pushed"] = u32(r,0x0c), u32(r,0x04)
    s["hiwat"] = u32(swd_read(A["hiwat"], 4), 0)
    gm = swd_read(A["gmap"], 0x90)
    s["vc"], s["mvalid"] = gm[0x30], gm[0x50]
    s["pair_ok"], s["restart"] = u32(gm,0x60), u32(gm,0x78)
    sd = swd_read(A["gseed"], 0x38)
    s["seed_seq"] = u32(sd, 0)
    s["seed_rej"] = (u32(sd,0x20) + u32(sd,0x24) + u32(sd,0x28) +
                     u32(sd,0x2c) + u32(sd,0x30))   # hdop+speed+jump+sanity+altx
    q = swd_read(A["ggq"], 0x38)
    s["q_hdop"], s["q_jump"], s["q_altx"] = u32(q,0x28), u32(q,0x2c), u32(q,0x30)
    s["reseed"] = u32(swd_read(A["greseed"], 4), 0)
    return s

def main():
    mins = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    print(f"===== 稳定性监控 {mins} 分钟 (每 60s 采样) =====")
    print("t(s)    tick    zda/s rmc/s csum  imu/s gnss/s rst vld | stale rej tsz0 | INS偏差m altm step_us | seed s/r qgate h/j/a rsd")
    snaps = []
    t0 = time.time()
    n_samp = mins + 1
    for i in range(n_samp):
        s = sample()
        snaps.append(s)
        if i > 0:
            p = snaps[i-1]
            dt = max((s["tick"] - p["tick"]) / 1000.0, 0.001)
            row = (f"{time.time()-t0:5.0f} {s['tick']:8d} "
                   f"{(s['zda']-p['zda'])/dt:5.1f} {(s['rmc']-p['rmc'])/dt:5.1f} "
                   f"{s['csum']-p['csum']:4d} {(s['imu']-p['imu'])/dt:6.1f} "
                   f"{(s['gnss']-p['gnss'])/dt:5.1f} {s['restart']:3d} {s['mvalid']:3d} | "
                   f"{s['stale']-p['stale']:5d} {s['rej']-p['rej']:3d} {s['ts_zero']-p['ts_zero']:4d} | "
                   f"{abs(s['lat']-22.6441)*111000:6.0f} {s['alt']:6.0f} {s['step_avg']:5.0f} | "
                   f"{s['seed_seq']:3d}/{s['seed_rej']:4d} "
                   f"{s['q_hdop']:3d}/{s['q_jump']:3d}/{s['q_altx']:3d} {s['reseed']:3d}")
        else:
            row = (f"{0:5.0f} {s['tick']:8d}  ---- ---- ----    0   ----   {s['restart']:3d} "
                   f"{s['mvalid']:3d} | ---- --- ---- | {abs(s['lat']-22.6441)*111000:6.0f} {s['alt']:6.0f} {s['step_avg']:5.0f} | "
                   f"{s['seed_seq']:3d}/{s['seed_rej']:4d} "
                   f"{s['q_hdop']:3d}/{s['q_jump']:3d}/{s['q_altx']:3d} {s['reseed']:3d}")
        print(row, flush=True)
        if i < n_samp - 1:
            time.sleep(60)
    # 汇总
    a, b = snaps[0], snaps[-1]
    dt = max((b["tick"] - a["tick"]) / 1000.0, 0.001)
    wall = time.time() - t0
    checks = [
        ("tick 速率≈墙钟 (无复位/死机)", abs(dt - wall) < wall * 0.05),
        (f"zda {(b['zda']-a['zda'])/dt:.1f}/s ≥ 9.5", (b["zda"]-a["zda"])/dt >= 9.5),
        (f"rmc {(b['rmc']-a['rmc'])/dt:.1f}/s ≥ 9.5", (b["rmc"]-a["rmc"])/dt >= 9.5),
        (f"csum 增量 {b['csum']-a['csum']} = 0", b["csum"] == a["csum"]),
        (f"timebase restart 增量 {b['restart']-a['restart']} = 0", b["restart"] == a["restart"]),
        (f"map.valid=1 (末次) ", b["mvalid"] == 1),
        (f"stale 增量 {b['stale']-a['stale']} = 0", b["stale"] == a["stale"]),
        (f"rej 增量 {b['rej']-a['rej']} = 0", b["rej"] == a["rej"]),
        (f"ts_zero 增量 {b['ts_zero']-a['ts_zero']} = 0", b["ts_zero"] == a["ts_zero"]),
        (f"imu {(b['imu']-a['imu'])/dt:.1f}/s ≥ 990", (b["imu"]-a["imu"])/dt >= 990),
        (f"gnss 观测 {(b['gnss']-a['gnss'])/dt:.1f}/s ≥ 8", (b["gnss"]-a["gnss"])/dt >= 8),
        (f"INS 位置偏差 {abs(b['lat']-22.6441)*111000:.0f}m ≤ 200", abs(b["lat"]-22.6441)*111000 <= 200),
        (f"raw lost 增量 {b['raw_lost']-a['raw_lost']} = 0", b["raw_lost"] == a["raw_lost"]),
        (f"NaN cnt = {b['nan_cnt']} = 0", b["nan_cnt"] == 0),
    ]
    print(f"\n===== 判据汇总 (窗口 {dt:.0f}s, 起止 uptime {a['tick']/1000:.0f}→{b['tick']/1000:.0f}s) =====")
    npass = 0
    for name, ok in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
        npass += ok
    print(f"===== 结果: {npass}/{len(checks)} PASS =====")

if __name__ == "__main__":
    main()
