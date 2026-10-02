#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
收敛性能测试: 硬复位后密集采样, 测 KF-GINS 输出各通道收敛时间。

用法: python swd_converge.py [分钟数, 默认 6]
每 ~3.5s 采样一次 (g_sol + tick + GNSS 原始高度 + 播种/重播种状态)。

收敛判据 (静止台架):
  ready     g_sol.ready 置位 (引擎开始解算)
  seed      播种稳定窗凑满 (seed_seq==10)
  水平      |lat-真值|·111km ≤ 阈值且此后不再超
  垂直      |alt-gnss_alt| ≤ 3m 且此后不再超
  速度      |v| ≤ 0.15 m/s 且此后不再超
  姿态      roll/pitch 30s 滑窗极差 < 0.3°
"""
import subprocess, re, struct, sys, time

CLI = r"D:\STM32CubeCLT\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
GSOL   = 0x2400ADB0          # g_nan 块: g_sol@+0xb8, rt_tick@+0x254
NAV    = 0x240011A8
GSEED  = 0x2400AF20
GRESD  = 0x2400AFA0
LAT_TRUE = 22.6441

def swd_read(addr, size):
    out = subprocess.run([CLI, "-c", "port=SWD", "mode=HotPlug", "-r32",
                          hex(addr), str(size)], capture_output=True, text=True,
                         timeout=60).stdout
    words = []
    for m in re.finditer(r"^0x[0-9A-Fa-f]{8}\s*:([0-9A-Fa-f ]+)$", out, re.M):
        words += [w for w in m.group(1).split()]
    if len(words) * 4 < size:
        raise RuntimeError(f"short read 0x{addr:x}")
    return b"".join(int(w.zfill(8), 16).to_bytes(4, "little") for w in words)[:size]

def u32(b, o): return int.from_bytes(b[o:o+4], "little")

def sample():
    s = {}
    blk = swd_read(GSOL, 0x258)
    s["tick"] = u32(blk, 0x254)
    s["nan"] = u32(blk, 0)
    g = blk[0xb8:0xb8+0xa8]
    s["ready"] = u32(g, 0)
    s["lat"], s["lon"], s["alt"] = struct.unpack_from("<3d", g, 0x10)
    s["vn"], s["ve"], s["vd"] = struct.unpack_from("<3d", g, 0x28)
    s["roll"], s["pitch"], s["yaw"] = struct.unpack_from("<3d", g, 0x40)
    s["imu"] = u32(g, 0x58); s["gnss"] = u32(g, 0x5c)
    n = swd_read(NAV, 0x38)
    s["gnss_alt"], s["geoid"] = struct.unpack_from("<2f", n, 0x18)
    s["fix"] = n[0x2c]
    sd = swd_read(GSEED, 0x38)
    s["seed_seq"] = u32(sd, 0)
    s["reseed"] = u32(swd_read(GRESD, 4), 0)
    return s

def hard_rst():
    subprocess.run([CLI, "-c", "port=SWD", "mode=HotPlug", "-hardRst"],
                   capture_output=True, text=True, timeout=60)

def main():
    mins = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
    print("===== 收敛性能测试: 硬复位后密集采样 =====")
    hard_rst()
    t0 = time.time()
    rows = []
    while time.time() - t0 < mins * 60:
        try:
            s = sample()
        except Exception as e:
            time.sleep(1.5)
            continue
        if s["tick"] == 0:                      # CLI 偶发软复位/毛刺
            time.sleep(1.5)
            continue
        t = time.time() - t0
        herr = abs(s["lat"] - LAT_TRUE) * 111000 if s["lat"] == s["lat"] else float("nan")
        verr = (s["alt"] - s["gnss_alt"]) if s["alt"] == s["alt"] else float("nan")
        vmod = (s["vn"]**2 + s["ve"]**2 + s["vd"]**2) ** 0.5 if s["vn"] == s["vn"] else float("nan")
        rows.append((t, s, herr, verr, vmod))
        print(f"t={t:5.1f}s tick={s['tick']:7d} rdy={s['ready']} sd={s['seed_seq']:2d} "
              f"rsd={s['reseed']} fix={s['fix']} | "
              f"Herr={herr:7.1f}m Verr={verr:6.1f}m |v|={vmod:5.2f} "
              f"r={s['roll']:6.2f} p={s['pitch']:6.2f} y={s['yaw']:7.2f} "
              f"alt={s['alt']:7.1f} galt={s['gnss_alt']:6.1f}", flush=True)
        time.sleep(3.0)

    # ---- 收敛时间分析 (首次进入且此后不再超阈值) ----
    def first_stable(pred, hold=30.0):
        for i, (t, s, herr, verr, vmod) in enumerate(rows):
            if not pred(s, herr, verr, vmod):
                continue
            ok = True
            for (t2, s2, h2, v2, vm2) in rows:
                if t2 >= t and t2 <= t + hold and not pred(s2, h2, v2, vm2):
                    ok = False
                    break
            if ok:
                return t
        return None

    def ready_pred(s, h, v, vm): return s["ready"] == 1
    def seed_pred(s, h, v, vm):  return s["seed_seq"] >= 10
    def h10(s, h, v, vm): return h == h and h <= 10
    def h5(s, h, v, vm):  return h == h and h <= 5
    def v3(s, h, v, vm):  return v == v and abs(v) <= 3
    def vel(s, h, v, vm): return vm == vm and vm <= 0.15

    print("\n===== 收敛时间 (自硬复位, 保持 30s 不超阈) =====")
    for name, pred in [("ready 引擎解算", ready_pred), ("播种窗凑满", seed_pred),
                       ("水平 ≤10m", h10), ("水平 ≤5m", h5),
                       ("垂直(vs GNSS) ≤3m", v3), ("速度 |v|≤0.15m/s", vel)]:
        t = first_stable(pred)
        print(f"  {name:18s}: {'%6.1fs' % t if t is not None else '  未收敛'}")
    if rows:
        last = rows[-1]
        print(f"\n末态: Herr={last[2]:.1f}m Verr={last[3]:.1f}m |v|={last[4]:.2f}m/s "
              f"roll={last[1]['roll']:.2f}° pitch={last[1]['pitch']:.2f}° "
              f"yaw={last[1]['yaw']:.2f}° reseed={last[1]['reseed']} nan={last[1]['nan']}")

if __name__ == "__main__":
    main()
