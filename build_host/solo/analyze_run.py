#!/usr/bin/env python3
"""无GNSS融合数据质量 + 收敛速度分析 (独立会话, 2026-10-02)
输入: run_capture.ps1 采集的原始 console 流 (含 gins_fused_data 行 + gins 快照 + ZUPT 日志)
用法: python analyze_run.py run1.bin
"""
import re
import sys
import math

ANSI = re.compile(r'\x1b\[[0-9;]*m')

FUSED = re.compile(
    r'ready:(\d) time:(\d+\.\d+) '
    r'roll:(-?[\d.]+) pitch:(-?[\d.]+) yaw:(-?[\d.]+) '
    r'vn:(-?[\d.]+) ve:(-?[\d.]+) vd:(-?[\d.]+) '
    r'lat:(-?[\d.]+) lon:(-?[\d.]+) alt:(-?[\d.]+) '
    r'imu_data:(\d+) gnss_data:(\d+) mag_calib_data:(\d+) baro_calib_data:(\d+) fused_data')

ZUPT_EV = re.compile(r'\[(\d+)\] I/gins: ZUPT: ([^ ].*?) \(w²=([\d.]+) n\(rad/s\)², \|f\|-g=([\d.]+)')

def load(path):
    rows = []
    with open(path, 'r', errors='replace') as f:
        for line in f:
            line = ANSI.sub('', line)
            m = FUSED.search(line)
            if m and m.group(1) == '1':
                rows.append(dict(
                    t=float(m.group(2)),
                    r=float(m.group(3)), p=float(m.group(4)), y=float(m.group(5)),
                    vn=float(m.group(6)), ve=float(m.group(7)), vd=float(m.group(8)),
                    lat=float(m.group(9)), lon=float(m.group(10)), alt=float(m.group(11)),
                    imu=int(m.group(12)), mag=int(m.group(14)), baro=int(m.group(15))))
    return rows

def stats(v):
    n = len(v)
    mu = sum(v) / n
    var = sum((x - mu) ** 2 for x in v) / max(1, n - 1)
    return mu, math.sqrt(var), min(v), max(v)

def circ_mean_deg(v):
    sx = sum(math.cos(math.radians(x)) for x in v)
    sy = sum(math.sin(math.radians(x)) for x in v)
    return math.degrees(math.atan2(sy, sx)) % 360.0

def circ_std_deg(v):
    mu = circ_mean_deg(v)
    d2 = sum(((x - mu + 180) % 360 - 180) ** 2 for x in v)
    return math.sqrt(d2 / max(1, len(v) - 1))

def main(path):
    rows = load(path)
    if not rows:
        print('NO fused rows'); return
    t0 = rows[0]['t']
    print(f'rows={len(rows)}  t=[{t0:.1f},{rows[-1]["t"]:.1f}]s  dur={rows[-1]["t"]-t0:.1f}s')

    # ---- 传感器喂入速率 ----
    dt = rows[-1]['t'] - rows[0]['t']
    imu_r = (rows[-1]['imu'] - rows[0]['imu']) / dt
    mag_r = (rows[-1]['mag'] - rows[0]['mag']) / dt
    baro_r = (rows[-1]['baro'] - rows[0]['baro']) / dt
    print(f'feed rate: imu={imu_r:.0f}Hz mag={mag_r:.1f}Hz baro={baro_r:.1f}Hz')

    # ---- 速度收敛 ----
    vabs = [math.sqrt(x['vn']**2 + x['ve']**2 + x['vd']**2) for x in rows]
    def settle(pred, hold=50):
        """从播种起持续 hold 个样本满足 pred 的首时刻 (相对首行)"""
        c = 0
        for i, x in enumerate(rows):
            c = c + 1 if pred(x, i) else 0
            if c >= hold:
                return rows[i - hold + 1]['t'] - t0
        return None
    v_settle = settle(lambda x, i: vabs[i] < 0.05)

    # ---- 姿态收敛: 与末段圆均值差 < 0.1deg 持续 5s ----
    tail = rows[-300:] if len(rows) > 300 else rows[-len(rows)//2:]
    r_ref = stats([x['r'] for x in tail])[0]
    p_ref = stats([x['p'] for x in tail])[0]
    y_ref = circ_mean_deg([x['y'] for x in tail])
    def att_close(x, i):
        return (abs(x['r'] - r_ref) < 0.1 and abs(x['p'] - p_ref) < 0.1
                and abs((x['y'] - y_ref + 180) % 360 - 180) < 0.5)
    a_settle = settle(att_close)

    # 高度稳定: 与末段均值差 < 0.2m
    alt_ref = stats([x['alt'] for x in tail])[0]
    h_settle = settle(lambda x, i: abs(x['alt'] - alt_ref) < 0.2)

    print(f'settle (from first ready=1): |v|<0.05 -> {v_settle}s' if v_settle is not None else '|v| never settled <0.05')
    print(f'settle: att within 0.1deg(0.5 yaw) of final -> {a_settle}s' if a_settle is not None else 'att never settled')
    print(f'settle: alt within 0.2m of final -> {h_settle}s' if h_settle is not None else 'alt never settled')

    # ---- 稳态质量 (末段) ----
    print(f'\n== steady state (last {len(tail)} rows, {tail[-1]["t"]-tail[0]["t"]:.0f}s) ==')
    print(f'ref att: r={r_ref:.3f} p={p_ref:.3f} y={y_ref:.3f} alt={alt_ref:.3f}')
    for k, f in (('roll', lambda x: x['r']), ('pitch', lambda x: x['p']),
                 ('vn', lambda x: x['vn']), ('ve', lambda x: x['ve']),
                 ('vd', lambda x: x['vd']), ('alt', lambda x: x['alt'])):
        mu, sd, mn, mx = stats([f(x) for x in tail])
        print(f'  {k:5s}: mean={mu:9.4f} std={sd:8.5f} min={mn:9.4f} max={mx:9.4f}')
    ys = [x['y'] for x in tail]
    print(f'  yaw  : mean={circ_mean_deg(ys):9.4f} circstd={circ_std_deg(ys):8.5f}')
    mu, sd, mn, mx = stats(vabs[-len(tail):])
    print(f'  |v|  : mean={mu:9.5f} std={sd:8.5f} max={mx:9.5f} m/s')
    dr = (tail[-1]['lat'] - tail[0]['lat']) * 111320.0
    dlon = (tail[-1]['lon'] - tail[0]['lon']) * 111320.0 * math.cos(math.radians(22.64))
    ddt = tail[-1]['t'] - tail[0]['t']
    print(f'  pos drift over {ddt:.0f}s: dN={dr:+.3f}m dE={dlon:+.3f}m ({math.hypot(dr,dlon)/ddt*1000:.3f} mm/s)')
    dalt = tail[-1]['alt'] - tail[0]['alt']
    print(f'  alt drift: {dalt:+.4f}m over {ddt:.0f}s')

    # ---- 全程姿态/速度轨迹粗览 (每30s一行) ----
    print('\n== trajectory (every ~30s) ==')
    nxt = t0
    for x in rows:
        if x['t'] >= nxt:
            print(f"  t+{x['t']-t0:6.1f}s r={x['r']:9.3f} p={x['p']:8.3f} y={x['y']:9.3f} "
                  f"|v|={math.hypot(x['vn'],x['ve'],x['vd']):7.4f} alt={x['alt']:8.3f}")
            nxt += 30

if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'run1.bin')
