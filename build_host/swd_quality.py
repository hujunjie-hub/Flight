# -*- coding: utf-8 -*-
"""KF-GINS 融合输出质量评估: 固定采样 g_sol + nav 对照, 统计报告."""
import subprocess, re, struct, time, sys, io, math

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
CLI = r"D:\STM32CubeCLT\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"

A = dict(
    rt_tick=0x2400ae88, nav_cnt=0x240011b0, nav_data=0x24001160,
    pushed=0x24008244, g_map=0x2400a0c0,
    g_sol=0x2400ad98, g_run=0x2400ad30, g_nan=0x2400ace8,
)

def rd(addr, nbytes, tries=3):
    for _ in range(tries):
        p = subprocess.run([CLI, "-c", "port=SWD mode=HotPlug", "-r32", hex(addr), str(nbytes)],
                           capture_output=True, text=True, timeout=30)
        words = []
        for line in p.stdout.splitlines():
            m = re.match(r"^0x[0-9A-F]{8}\s*:\s*(.*)$", line)
            if m: words += [int(w, 16) for w in m.group(1).split()]
        if len(words) >= nbytes // 4: return words
    return None

def mean(xs): return sum(xs) / len(xs) if xs else float("nan")
def std(xs):
    if len(xs) < 2: return float("nan")
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
samples = []
prev = None
t0 = time.time()
while time.time() - t0 < DUR:
    sol = rd(A["g_sol"], 160)
    nav = rd(A["nav_cnt"], 20)
    navd = rd(A["nav_data"], 32)
    nan = rd(A["g_nan"], 72)
    if not all([sol, nav, navd, nan]):
        time.sleep(1); continue
    b = struct.pack("<40I", *sol[:40])
    nb = struct.pack("<8I", *navd[:8])
    f64 = lambda o: struct.unpack_from("<d", b, o)[0]
    s = dict(t=time.time(), tick=rd(A["rt_tick"], 4)[0] if rd(A["rt_tick"], 4) else 0,
             ready=sol[0], imu=sol[22], gnss=sol[23], stale=sol[24], age=sol[25],
             mag=sol[30], magrej=sol[31], baro=sol[33],
             roll=f64(64), pitch=f64(72), yaw=f64(80),
             vn=f64(40), ve=f64(48), vd=f64(56),
             lat=f64(16), lon=f64(24), alt=f64(32),
             nancnt=nan[0],
             nav_lat=struct.unpack_from("<d", nb, 8)[0],
             nav_lon=struct.unpack_from("<d", nb, 16)[0],
             nav_alt=struct.unpack_from("<f", nb, 24)[0],
             nav_fix=nb[44] if len(nb) > 44 else 0,
             zda=nav[2], csum=nav[3])
    samples.append(s)
    if prev is None:
        print(f"t={time.time()-t0:5.0f}s ready={s['ready']} gnss_obs={s['gnss']} "
              f"| r/p/y={s['roll']:+.2f}/{s['pitch']:+.2f}/{s['yaw']:+.2f} "
              f"v={s['vn']:+.2f},{s['ve']:+.2f},{s['vd']:+.2f} alt={s['alt']:.1f}")
    else:
        dt = s["t"] - prev["t"]
        dlat = (s["lat"] - prev["lat"]) * 111320.0 * math.cos(math.radians(s["lat"]))
        dlon = (s["lon"] - prev["lon"]) * 111320.0
        print(f"t={time.time()-t0:5.0f}s imu{(s['imu']-prev['imu'])/dt:4.0f}/s "
              f"gnss{(s['gnss']-prev['gnss'])/dt:+4.1f}/s mag{(s['mag']-prev['mag'])/dt:4.0f}/s "
              f"age={s['age']/1000:4.1f}s nan={s['nancnt']} "
              f"| r/p/y={s['roll']:+7.2f}/{s['pitch']:+7.2f}/{s['yaw']:+8.2f} "
              f"v={s['vn']:+6.2f},{s['ve']:+6.2f},{s['vd']:+6.2f} dpos={math.hypot(dlat,dlon):+.2f}m")
    sys.stdout.flush()
    prev = s
    time.sleep(2.0)

print("\n" + "=" * 60)
print("KF-GINS 融合输出质量统计 (静置台架)")
print("=" * 60)
ok = [s for s in samples if s["ready"] and s["roll"] == s["roll"]]
if not ok:
    print("无有效样本 (未 ready 或 NaN)")
else:
    def report(name, key, unit, wrap=None):
        vals = [s[key] for s in ok]
        if wrap:  # 圆量 (yaw)
            vals = [(v + 180) % 360 - 180 for v in vals]
        print(f"{name:16s}: mean={mean(vals):+9.3f}  std={std(vals):7.3f}  "
              f"min={min(vals):+9.3f}  max={max(vals):+9.3f}  {unit}")
    print(f"样本数: {len(ok)}  采样时长: {DUR:.0f}s  NaN计数: {ok[-1]['nancnt']}")
    print(f"GNSS 观测: 总数={ok[-1]['gnss']} (平均 {ok[-1]['gnss']/DUR:.2f} obs/s)  最近age={ok[-1]['age']/1000:.1f}s")
    print(f"IMU 喂入: 总数={ok[-1]['imu']}  磁观测: {ok[-1]['mag']}  磁拒绝: {ok[-1]['magrej']}")
    print()
    report("roll", "roll", "deg")
    report("pitch", "pitch", "deg")
    report("yaw", "yaw", "deg", wrap=True)
    report("vn", "vn", "m/s")
    report("ve", "ve", "m/s")
    report("vd", "vd", "m/s")
    report("altitude", "alt", "m")
    # 与 GNSS 原始定位对照
    if ok[-1]["nav_fix"] and ok[-1]["gnss"] > 0:
        dlat = (ok[-1]["lat"] - ok[-1]["nav_lat"]) * 111320.0
        dlon = (ok[-1]["lon"] - ok[-1]["nav_lon"]) * 111320.0 * math.cos(math.radians(ok[-1]["lat"]))
        print(f"\n融合 vs GNSS 原始: dN={dlat:+.2f}m dE={dlon:+.2f}m |dh={ok[-1]['alt']-ok[-1]['nav_alt']:+.2f}m")
