#!/usr/bin/env python3
"""多轮回归分析: 每轮播种/爆炸/恢复/收敛/稳态"""
import re, sys, math, glob

ANSI = re.compile(r'\x1b\[[0-9;]*m')
FUSED = re.compile(r'ready:(\d) time:(\d+\.\d+) roll:(-?[\d.]+) pitch:(-?[\d.]+) yaw:(-?[\d.]+) '
                   r'vn:(-?[\d.]+) ve:(-?[\d.]+) vd:(-?[\d.]+) lat:(-?[\d.]+) lon:(-?[\d.]+) '
                   r'alt:(-?[\d.]+) imu_data:(\d+) gnss_data:(\d+) mag_calib_data:(\d+) '
                   r'baro_calib_data:(\d+) fused_data')
SEED = re.compile(r'\[(\d+)\] I/gins: KF-GINS')
GUARD = re.compile(r'\[(\d+)\] W/gins: .*?([\d.]+) km')
RESEED = re.compile(r'\[(\d+)\] W/gins: reseed')
MAGBARO = re.compile(r'\[(\d+)\] I/gins: mag/baro: mag=(\d+) rej=(\d+).*?baro=(\d+) rej=(\d+).*?baro_h=(-?[\d.]+) m \(alt=(-?[\d.]+)\)')

def analyze(path):
    txt = [ANSI.sub('', l) for l in open(path, errors='replace')]
    rows = []
    for line in txt:
        m = FUSED.search(line)
        if m and m.group(1) == '1':
            rows.append(dict(t=float(m.group(2)), r=float(m.group(3)), p=float(m.group(4)),
                             y=float(m.group(5)), v=math.hypot(float(m.group(6)), float(m.group(7)), float(m.group(8))),
                             lat=float(m.group(9)), alt=float(m.group(12))))
    seeds = [int(m.group(1)) for l in txt for m in [SEED.search(l)] if m]
    guards = [(int(m.group(1)), float(m.group(2))) for l in txt for m in [GUARD.search(l)] if m]
    reseeds = [int(m.group(1)) for l in txt for m in [RESEED.search(l)] if m]
    mb = [(int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4)), int(m.group(5)), float(m.group(6)), float(m.group(7)))
          for l in txt for m in [MAGBARO.search(l)] if m]
    # 组: 1=ts 2=mag 3=magrej 4=baro 5=barorej 6=baro_h 7=alt
    print(f'--- {path} ---')
    print(f'  seeds@{seeds}  guards@{[(t, round(k)) for t, k in guards]}  reseeds@{reseeds}')
    if mb:
        t, mag, mrej, baro, brej, bh, alt = mb[-1][0], mb[-1][1], mb[-1][2], mb[-1][3], mb[-1][4], mb[-1][5], mb[-1][6]
        rate = mrej / max(1, mag + mrej) * 100
        print(f'  final mag rej={rate:.1f}%  baro rej={brej} baro_h={bh} alt={alt}')
    if not rows:
        print('  NO SOLUTION ROWS'); return
    t0 = rows[0]['t']
    # 收敛: |v|<0.05 持续 5s
    cnt = 0; v_set = None
    for x in rows:
        cnt = cnt + 1 if x['v'] < 0.05 else 0
        if cnt >= 50: v_set = rows[rows.index(x) - 49]['t'] - t0; break
    # 稳态: 末 40% 时段且要求 |v| 全程<0.1 (排除被扰动)
    tail = [x for x in rows if x['t'] - t0 > (rows[-1]['t'] - t0) * 0.6]
    if tail and max(x['v'] for x in tail) < 0.1 and len(tail) > 50:
        mus = {k: sum(x[k] for x in tail) / len(tail) for k in ('r', 'p', 'y', 'v', 'alt')}
        sds = {k: (sum((x[k] - mus[k]) ** 2 for x in tail) / (len(tail) - 1)) ** 0.5 for k in ('r', 'p', 'y', 'v', 'alt')}
        dl = (tail[-1]['lat'] - tail[0]['lat']) * 111320
        dt = tail[-1]['t'] - tail[0]['t']
        print(f"  v<0.05 settle: {v_set:.1f}s" if v_set is not None else "  v never settled")
        print(f"  steady: r={mus['r']:.3f}±{sds['r']:.4f} p={mus['p']:.3f}±{sds['p']:.4f} "
              f"y={mus['y']:.2f}±{sds['y']:.4f} |v|={mus['v']:.5f} alt={mus['alt']:.3f}±{sds['alt']:.4f} "
              f"drift={abs(dl)/max(dt,0.1)*1000:.1f}mm/s")
    else:
        print('  tail disturbed (no steady window)')

for p in sys.argv[1:]:
    analyze(p)
