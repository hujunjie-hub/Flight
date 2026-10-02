#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
稳定性监控 + 高度通道专项 (SWD 免串口).

在 swd_stability.py 的 14 判据基础上增加垂直通道字段:
  gnss_alt  UM982 GGA 椭球高原始值 (nav.data.position.altitude, m)
  ins_alt   融合解椭球高 (g_sol.altitude, m)
  d_ins_gn  INS-GNSS 原始高度差 (融合对 GNSS 的跟踪偏差, m)
  baro_h   引擎气压高度模型 (hraw+bias, m)
  d_ins_ba  INS-气压高度差 (新息, m)
  vd       融合解垂速 (NED, 正=下沉, m/s)
  b_cnt/r/s 气压观测计数/门限拒绝/过旧丢弃
"""
import subprocess, re, struct, sys, time, os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# 2026-10-01: 当前台架只挂 DAPLink (CubeProgrammer 需要 ST-Link), 读通道
# 改走常驻 OpenOCD telnet 4444 (ocd_swd_read); 换回 ST-Link 时把下面
# _USE_OCD 置 0 即恢复 CLI 路径。
_USE_OCD = 1
CLI = r"D:\STM32CubeCLT\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
# 2026-09-30 A3/A4/C9/D13/B6 固件符号 (每次重编须 nm 复核)
# 2026-09-30 C7/C8/B5 固件符号 (每次重编须 nm 复核)
# 2026-10-01 ADIS 自愈固件符号 (nm+gdb DWARF 复核; g_nan/g_run/g_sol 相对
# 布局不变 run@+0x48 sol@+0xb8; 引擎成员 barohgt_@0x3c8 updstat_@0x408
# baroupd@0x414 pvacur_@0x440 与上一版一致; rt_tick 独立符号)
A = dict(nav=0x24000EC8, rawctx=0x24007EA0, hiwat=0x24007EE4,
         gmap=0x24009D40, gnan2run=0x2400A818, gseed=0x2400A998,
         ggq=0x2400A7E0, greseed=0x2400AA28, sengine=0x2400AA40)
RT_TICK_ADDR = 0x2400AA9C

if _USE_OCD:
    from ocd_swd_read import ocd_read as _ocd_read

def swd_read(addr, size):
    if _USE_OCD:
        return _ocd_read(addr, size)
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
    blk = swd_read(A["gnan2run"], 0x180)          # g_nan@0 g_run@0x48 g_sol@0xb8
    s["tick"] = u32(swd_read(RT_TICK_ADDR, 4), 0)
    s["nan_cnt"] = u32(blk, 0)
    s["nan_streak"] = u32(blk, 0x44)
    gr = blk[0x48:0x48+0x70]
    s["imu"], s["gnss"], s["stale"], s["rej"] = u32(gr,0), u32(gr,4), u32(gr,0x10), u32(gr,0x50)
    s["ts_zero"], s["late_us"] = u32(gr, 0x64), u32(gr, 0x68)
    s["skip_mid"] = u32(gr, 0x6c)
    s["step_avg"], s["step_max"] = struct.unpack_from("<2f", gr, 0x48)
    s["lat"], s["lon"], s["alt"] = struct.unpack_from("<3d", blk, 0xb8+0x10)
    s["vd"] = struct.unpack_from("<d", blk, 0xb8+0x38)[0]
    s["b_cnt"], s["b_rej"], s["b_stale"] = (u32(blk,0xb8+0x84), u32(blk,0xb8+0x88),
                                            u32(blk,0xb8+0x8c))
    s["baro_h"] = struct.unpack_from("<d", blk, 0xb8+0x90)[0]
    s["b_skip"] = u32(blk, 0xb8+0xa0)
    s["vreset"] = u32(blk, 0xb8+0xa4)
    s["nan_sol"] = u32(blk, 0xb8+0xa8)
    s["degraded"] = u32(blk, 0xb8+0xac)
    s["updfail"] = u32(blk, 0xb8+0xb0)
    n = swd_read(A["nav"], 0x68)
    s["rmc"], s["gga"], s["zda"] = u32(n,0x50), u32(n,0x54), u32(n,0x58)
    s["csum"], s["field"] = u32(n,0x5c), u32(n,0x60)
    s["gnss_alt"], s["geoid"] = struct.unpack_from("<2f", n, 0x18)
    s["hdop"] = struct.unpack_from("<f", n, 0x30)[0]
    s["fix"] = n[0x2c]
    r = swd_read(A["rawctx"], 0x48)
    s["raw_lost"], s["raw_pushed"] = u32(r,0x0c), u32(r,0x04)
    gm = swd_read(A["gmap"], 0x90)
    s["vc"], s["mvalid"] = gm[0x30], gm[0x50]
    s["pair_ok"], s["restart"] = u32(gm,0x60), u32(gm,0x78)
    sd = swd_read(A["gseed"], 0x38)
    s["seed_seq"] = u32(sd, 0)
    s["seed_rej"] = (u32(sd,0x20) + u32(sd,0x24) + u32(sd,0x28) +
                     u32(sd,0x2c) + u32(sd,0x30))
    q = swd_read(A["ggq"], 0x38)
    s["q_hdop"], s["q_jump"], s["q_altx"] = u32(q,0x28), u32(q,0x2c), u32(q,0x30)
    s["reseed"] = u32(swd_read(A["greseed"], 4), 0)
    # 引擎内部 (解引用 s_engine, 偏移随构建漂移须 nm+DWARF 复核):
    # barohgt_@0x3c8 updstat_@0x3f8 (baroupd@0xc..vreset@0x1c) pvacur_.pos@0x420
    # 指针域校验: SWD 毛刺/并发会话会读出垃圾指针, 域外按无引擎处理
    # (重试同一垃圾值会死循环; 引擎在 RT-Thread 堆, AXI SRAM 0x24000000+320K)
    eng = u32(swd_read(A["sengine"], 4), 0)
    if not (0x24000000 <= eng < 0x24050000):
        eng = 0
    if eng:
        eb = swd_read(eng+0x3c8, 0x90)
        s["e_baroupd"]  = u32(eb, 0x408-0x3c8+0xc)
        s["e_barorej"]  = u32(eb, 0x408-0x3c8+0x10)
        s["e_baroskip"] = u32(eb, 0x408-0x3c8+0x18)
        s["e_vreset"]   = u32(eb, 0x408-0x3c8+0x1c)
        s["e_gnssdrop"] = u32(eb, 0x408-0x3c8+0x28)
        s["e_covheal"]  = u32(eb, 0x408-0x3c8+0x2c)
        s["e_alt"]      = struct.unpack_from("<d", eb, 0x450-0x3c8)[0]
    else:
        s["e_baroupd"] = s["e_barorej"] = s["e_baroskip"] = s["e_vreset"] = 0
        s["e_gnssdrop"] = s["e_covheal"] = 0
        s["e_alt"] = float("nan")
    return s

def main():
    mins = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    print(f"===== 稳定性+高度通道监控 {mins} 分钟 (每 60s 采样) =====", flush=True)
    print("t(s)    tick    zda/s rmc/s csum  imu/s gnss/s rst vld | stale rej tsz0 | "
          "gnss_alt ins_alt d(I-G) baro_h d(I-B)   vd | b c/r/s fix hdop", flush=True)
    snaps = []
    walls = []                              # 每次采样"开始"的墙钟 (2026-10-01:
    t0 = time.time()                        # OpenOCD 采样耗时 ~5-8s, tick 判据
    n_samp = mins + 1                       # 须按采样起点间隔对比, 否则误杀)
    bad_streak = 0
    for i in range(n_samp):
        tw = time.time()
        while True:
            if bad_streak >= 60:      # 连续 ~5min 无有效采样: DAP 卡死/链路断, 放弃
                print("[abort] 采样连续失败 60 次, 提前结束本轮", flush=True)
                raise SystemExit(2)
            try:
                s = sample()
                if 1000 < s["tick"] < 1000000000 or i == 0:   # SWD 毛刺/DAP 卡死: tick 域外重采
                    bad_streak = 0
                    break
                bad_streak += 1
                print(f"  [warn] tick={s['tick']} 毛刺, 重采", flush=True)
                time.sleep(2)
            except RuntimeError as e:
                bad_streak += 1
                print(f"  [warn] {e}, 重试", flush=True)
                time.sleep(5)
        snaps.append(s)
        walls.append(tw)
        if i > 0:
            p = snaps[i-1]
            dt = max((s["tick"] - p["tick"]) / 1000.0, 0.001)
            print(f"{time.time()-t0:5.0f} {s['tick']:8d} "
                  f"{(s['zda']-p['zda'])/dt:5.1f} {(s['rmc']-p['rmc'])/dt:5.1f} "
                  f"{s['csum']-p['csum']:4d} {(s['imu']-p['imu'])/dt:6.1f} "
                  f"{(s['gnss']-p['gnss'])/dt:5.1f} {s['restart']:3d} {s['mvalid']:3d} | "
                  f"{s['stale']-p['stale']:5d} {s['rej']-p['rej']:3d} {s['ts_zero']-p['ts_zero']:4d} | "
                  f"{s['gnss_alt']:8.1f} {s['alt']:8.1f} {s['alt']-s['gnss_alt']:6.1f} "
                  f"{s['baro_h']:8.1f} {s['alt']-s['baro_h']:6.1f} {s['vd']:6.2f} | "
                  f"{s['b_cnt']:5d} {s['b_rej']:3d}/{s['b_stale']:3d} {s['fix']:2d} {s['hdop']:4.1f} | "
                  f"eng up {(s['e_baroupd']-p['e_baroupd'])/dt:4.1f}/s rj {(s['e_barorej']-p['e_barorej'])/dt:4.1f}/s "
                  f"sk {(s['e_baroskip']-p['e_baroskip'])/dt:5.1f}/s vrs {s['e_vreset']:3d} ealt {s['e_alt']:7.1f}",
                  flush=True)
        else:
            print(f"{0:5.0f} {s['tick']:8d}  ---- ---- ----    0   ----   {s['restart']:3d} "
                  f"{s['mvalid']:3d} | ---- --- ---- | "
                  f"{s['gnss_alt']:8.1f} {s['alt']:8.1f} {s['alt']-s['gnss_alt']:6.1f} "
                  f"{s['baro_h']:8.1f} {s['alt']-s['baro_h']:6.1f} {s['vd']:6.2f} | "
                  f"{s['b_cnt']:5d} {s['b_rej']:3d}/{s['b_stale']:3d} {s['fix']:2d} {s['hdop']:4.1f}",
                  flush=True)
        if i < n_samp - 1:
            time.sleep(60)
    # ---- 汇总: 标准 14 判据 ----
    a, b = snaps[0], snaps[-1]
    dt = max((b["tick"] - a["tick"]) / 1000.0, 0.001)
    wall = walls[-1] - walls[0]
    checks = [
        ("tick 速率≈墙钟 (无复位/死机)", abs(dt - wall) < max(2.0, wall * 0.03)),
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
        (f"发布降级 degraded 末次 = {b['degraded']} = 0", b["degraded"] == 0),
        (f"Cholesky 失败 updfail 增量 {b['updfail']-a['updfail']} = 0", b["updfail"] == a["updfail"]),
        (f"重对准 reseed 增量 {b['reseed']-a['reseed']} = 0", b["reseed"] == a["reseed"]),
    ]
    print(f"\n===== 判据汇总 (窗口 {dt:.0f}s, 起止 uptime {a['tick']/1000:.0f}→{b['tick']/1000:.0f}s) =====")
    npass = 0
    for name, ok in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
        npass += ok
    print(f"===== 结果: {npass}/{len(checks)} PASS =====")
    # ---- 高度通道专项分析 ----
    n = len(snaps) - 1
    dur = max((snaps[-1]["tick"] - snaps[0]["tick"]) / 1000.0, 0.001)
    drift_ins  = (snaps[-1]["alt"] - snaps[0]["alt"]) / dur
    drift_gnss = (snaps[-1]["gnss_alt"] - snaps[0]["gnss_alt"]) / dur
    drift_baro = (snaps[-1]["baro_h"] - snaps[0]["baro_h"]) / dur
    print("\n----- 高度通道专项 -----")
    print(f"  INS 融合高度漂移率  : {drift_ins:+.3f} m/s "
          f"({snaps[0]['alt']:.1f} -> {snaps[-1]['alt']:.1f} m)")
    print(f"  GNSS 原始高度漂移率 : {drift_gnss:+.3f} m/s "
          f"({snaps[0]['gnss_alt']:.1f} -> {snaps[-1]['gnss_alt']:.1f} m)")
    print(f"  气压模型高度漂移率  : {drift_baro:+.3f} m/s "
          f"({snaps[0]['baro_h']:.1f} -> {snaps[-1]['baro_h']:.1f} m)")
    print(f"  气压观测: 入滤 {snaps[-1]['b_cnt']-snaps[0]['b_cnt']} 次, "
          f"门限拒绝 +{snaps[-1]['b_rej']-snaps[0]['b_rej']}, "
          f"过旧丢弃 +{snaps[-1]['b_stale']-snaps[0]['b_stale']}")
    vd_mean = sum(x["vd"] for x in snaps[1:]) / n
    print(f"  融合垂速 vd 均值    : {vd_mean:+.3f} m/s (正=下沉)")
    di = [x["alt"] - x["gnss_alt"] for x in snaps]
    db = [x["alt"] - x["baro_h"] for x in snaps]
    print(f"  INS-GNSS 高度差     : min {min(di):.1f} / max {max(di):.1f} / "
          f"末值 {di[-1]:.1f} m")
    print(f"  INS-baro 高度差(新息): min {min(db):.1f} / max {max(db):.1f} / "
          f"末值 {db[-1]:.1f} m")

if __name__ == "__main__":
    main()
