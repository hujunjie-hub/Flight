#!/usr/bin/env python3
"""10-round no-GNSS regression: per-round metrics + aggregate report.

Usage: python eval_10rounds.py <round_dir> [outdir]
Round files: round_XX.bin (cap_10rounds.ps1 output)
Outputs: report.md, report.png, summary.json
"""
import sys, re, os, json, glob
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

LINE_RE = re.compile(rb'((?:[A-Za-z_]+:-?\d+(?:\.\d+)?\s+)+fused_data)')
KV_RE = re.compile(r'([A-Za-z_]+):(-?\d+(?:\.\d+)?)')

def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0

def m_per_deg(lat_deg):
    p = np.radians(lat_deg)
    mlat = 111132.954 - 559.822*np.cos(2*p) + 1.175*np.cos(4*p)
    mlon = 111412.84*np.cos(p) - 93.5*np.cos(3*p) + 0.118*np.cos(5*p)
    return mlat, mlon

def conv_time(t, vh, th):
    idx = np.where(vh > th)[0]
    if len(idx) == 0:
        return t[0]
    return t[min(idx.max()+1, len(t)-1)]

def analyze_round(path):
    raw = open(path, 'rb').read()
    text = raw.decode('utf-8', 'replace')
    clean = re.sub(r'\x1b\[[0-9;]*m', '', text)

    rows = []
    for m in LINE_RE.finditer(raw):
        d = dict(KV_RE.findall(m.group(1).decode('ascii')))
        # 完整行校验: 半行 (串口截断) 缺 vn/lat, 丢弃
        if not all(k in d for k in ('time', 'ready', 'vn', 'lat')):
            continue
        try:
            r = {k: float(v) for k, v in d.items()}
        except ValueError:
            continue
        if r['ready'] > 0.5:
            rows.append(r)
    if len(rows) < 50:
        return {'file': os.path.basename(path), 'ok': False, 'rows': len(rows)}

    t  = np.array([r['time'] for r in rows])
    vn = np.array([r['vn'] for r in rows]); ve = np.array([r['ve'] for r in rows])
    vd = np.array([r['vd'] for r in rows])
    lat = np.array([r['lat'] for r in rows]); lon = np.array([r['lon'] for r in rows])
    alt = np.array([r['alt'] for r in rows])
    roll = np.array([r['roll'] for r in rows]); pitch = np.array([r['pitch'] for r in rows])
    yaw  = np.array([r['yaw'] for r in rows])
    imu  = np.array([r['imu_data'] for r in rows])
    vh = np.hypot(vn, ve)

    j5 = np.searchsorted(t, 5.0)
    mlat, mlon = m_per_deg(lat[j5])
    dH = np.hypot((lat-lat[j5])*mlat, (lon-lon[j5])*mlon)

    def at(sec):
        j = min(np.searchsorted(t, sec), len(t)-1)
        return dH[j], vh[j]

    i30 = np.searchsorted(t, 30.0)
    span = t[-1] - t[i30]
    res = {
        'file': os.path.basename(path), 'ok': True, 'rows': len(rows),
        'seed_t': t[0], 'span_end': t[-1],
        'ready_span_s': float(t[-1]-t[0]),
        'imu_rate': float((imu[-1]-imu[0])/(t[-1]-t[0])),
        'dH_30': at(30.0)[0], 'dH_60': at(60.0)[0],
        'dH_120': at(120.0)[0], 'dH_180': at(180.0)[0],
        'vh_30': at(30.0)[1], 'vh_60': at(60.0)[1],
        'end_dH': float(dH[-1]), 'end_vh': float(vh[-1]), 'end_vd': float(vd[-1]),
        'max_vh_30s': float(vh[i30:].max()),
        'conv_0p5': float(conv_time(t, vh, 0.5)),
        'conv_0p15': float(conv_time(t, vh, 0.15)),
        'att_drift': [float(roll[-1]-roll[i30]), float(pitch[-1]-pitch[i30]),
                      float(wrap180(yaw[-1]-yaw[i30]))],
        'att_std': [float(np.std(roll[i30:])), float(np.std(pitch[i30:])),
                    float(np.std(yaw[i30:]))],
        'alt_drift': float(alt[-1]-alt[i30]),
        'events': [],
    }
    for pat, name in [('reseed:', 'reseed'), ('播种位置守卫', 'posguard'),
                      ('协方差对角出现负值', 'cov_warn'), ('检测到运动', 'motion_pause'),
                      ('快照已保存', 'accbias_save'), ('拆引擎重对准', 'engine_rebuild'),
                      ('非有限', 'nan')]:
        n = len(re.findall(pat, clean))
        if n:
            res['events'].append(f'{name}x{n}')
    m = re.search(r'零偏先验 \(([-\d.]+), ([-\d.]+), ([-\d.]+)\) deg/h \(std (\d+)', clean)
    if m:
        res['gbias_seed'] = [float(m.group(1)), float(m.group(2)),
                             float(m.group(3)), float(m.group(4))]
    return res

def main():
    rdir = sys.argv[1]
    outdir = sys.argv[2] if len(sys.argv) > 2 else rdir
    files = sorted(glob.glob(os.path.join(rdir, 'round_*.bin')))
    if not files:
        print('no round files in', rdir); return 1

    results = [analyze_round(f) for f in files]
    ok = [r for r in results if r['ok']]

    # ---------- report.md ----------
    L = []
    L.append('# KF-GINS 无 GNSS 模式 10 轮回归测试报告\n')
    L.append('- 时间: 2026-10-01  固件: 静止粗对准最终版 (双构建)')
    L.append('- 条件: 每轮 reboot → 对准/播种 (~34s) → 采集至播种后 ~3min, 静止台架, 无 GNSS 天线')
    L.append(f'- 轮次: {len(ok)}/{len(results)} 有效\n')

    hdr = ('| 轮 | dH@60s | dH@120s | dH@180s | END dH | END \\|vh\\| | max \\|vh\\| 30s+ | '
           'conv<0.5 | conv<0.15 | att漂移 r/p/y | alt漂移 | 事件 |')
    sep = '|---|---|---|---|---|---|---|---|---|---|---|---|'
    L += [hdr, sep]
    for i, r in enumerate(results, 1):
        if not r['ok']:
            L.append(f'| {i} | PARSE FAIL (rows={r["rows"]}) |' + '---|'*10)
            continue
        a = r['att_drift']
        L.append(
            f'| {i} | {r["dH_60"]:.2f} | {r["dH_120"]:.2f} | {r["dH_180"]:.2f} | '
            f'{r["end_dH"]:.2f} | {r["end_vh"]:.4f} | {r["max_vh_30s"]:.3f} | '
            f'{r["conv_0p5"]:.0f}s | {r["conv_0p15"]:.0f}s | '
            f'{a[0]:+.2f}/{a[1]:+.2f}/{a[2]:+.2f} | {r["alt_drift"]:+.2f} | '
            f'{", ".join(r["events"]) or "-"} |')

    if ok:
        keys = ['dH_60', 'dH_120', 'dH_180', 'end_dH', 'end_vh', 'max_vh_30s']
        L.append('\n## 汇总统计 (n=%d)\n' % len(ok))
        L.append('| 指标 | mean | max | min |')
        L.append('|---|---|---|---|')
        for k in keys:
            v = np.array([r[k] for r in ok])
            unit = ' m/s' if 'vh' in k else ' m'
            L.append(f'| {k}{unit} | {v.mean():.3f} | {v.max():.3f} | {v.min():.3f} |')
        ad = np.array([r['att_drift'] for r in ok])
        L.append(f'| att漂移 r/p/y (deg) | {np.abs(ad).mean(0)[0]:.2f}/{np.abs(ad).mean(0)[1]:.2f}/{np.abs(ad).mean(0)[2]:.2f} (mean abs) | '
                 f'{np.abs(ad).max(0)[0]:.2f}/{np.abs(ad).max(0)[1]:.2f}/{np.abs(ad).max(0)[2]:.2f} | - |')

        # verdict
        max_dh = max(r['end_dH'] for r in ok)
        max_vh = max(r['end_vh'] for r in ok)
        max_att = max(max(abs(x) for x in r['att_drift']) for r in ok)
        bad = [r for r in ok if any('reseed' in e or 'posguard' in e or 'cov_warn' in e
                                    or 'nan' in e or 'engine_rebuild' in e
                                    for e in r['events'])]
        L.append('\n## 判定\n')
        L.append(f'- END dH max = {max_dh:.2f} m (判据 < 2 m): {"PASS" if max_dh < 2 else "FAIL"}')
        L.append(f'- END |vh| max = {max_vh:.4f} m/s (判据 < 0.05): {"PASS" if max_vh < 0.05 else "FAIL"}')
        L.append(f'- 姿态漂移 max = {max_att:.2f} deg (判据 < 1): {"PASS" if max_att < 1 else "FAIL"}')
        L.append(f'- 异常事件 (reseed/守卫/cov_warn/NaN): {len(bad)} 轮: {"PASS" if not bad else "FAIL"}')
        if all('gbias_seed' in r for r in ok):
            gb = np.array([r['gbias_seed'] for r in ok])
            L.append(f'- 粗对准零偏播种: 10/10 生效, 测量均值 x/y/z = '
                     f'{gb[:,0].mean():.0f}/{gb[:,1].mean():.0f}/{gb[:,2].mean():.0f} deg/h '
                     f'(std {gb[:,3].mean():.0f})')

    # per-round gbias seed stability
    L.append('\n## 粗对准零偏先验逐轮测量 (deg/h)\n')
    L.append('| 轮 | gx | gy | gz | std |')
    L.append('|---|---|---|---|---|')
    for i, r in enumerate(results, 1):
        if r['ok'] and 'gbias_seed' in r:
            g = r['gbias_seed']
            L.append(f'| {i} | {g[0]:.0f} | {g[1]:.0f} | {g[2]:.0f} | {g[3]:.0f} |')
    L.append('')

    rp = os.path.join(outdir, 'report.md')
    open(rp, 'w', encoding='utf-8').write('\n'.join(L))
    print('report ->', rp)
    print('\n'.join(L))

    # ---------- figure ----------
    fig, ax = plt.subplots(3, 2, figsize=(14, 11))
    colors = plt.cm.viridis(np.linspace(0, 1, len(files)))
    for i, f in enumerate(files):
        r = analyze_round(f)
        if not r['ok']:
            continue
        raw = open(f, 'rb').read()
        rows = []
        for m in LINE_RE.finditer(raw):
            d = dict(KV_RE.findall(m.group(1).decode('ascii')))
            if not all(k in d for k in ('time', 'ready', 'vn', 'lat')):
                continue
            if float(d['ready']) > 0.5:
                try:
                    rows.append({k: float(v) for k, v in d.items()})
                except ValueError:
                    pass
        t = np.array([x['time'] for x in rows])
        vh = np.hypot(np.array([x['vn'] for x in rows]),
                      np.array([x['ve'] for x in rows]))
        lat = np.array([x['lat'] for x in rows]); lon = np.array([x['lon'] for x in rows])
        alt = np.array([x['alt'] for x in rows])
        j5 = np.searchsorted(t, 5.0)
        mlat, mlon = m_per_deg(lat[j5])
        dH = np.hypot((lat-lat[j5])*mlat, (lon-lon[j5])*mlon)
        ts = t - t[j5]

        a = ax[0][0]; a.plot(ts, dH, lw=0.9, color=colors[i], label=f'r{i+1}')
        a = ax[0][1]; a.plot(ts, vh, lw=0.9, color=colors[i])
        a = ax[1][0]; a.plot(ts, np.array([x['roll'] for x in rows]), lw=0.7, color=colors[i])
        a = ax[1][1]; a.plot(ts, np.array([x['pitch'] for x in rows]), lw=0.7, color=colors[i])
        a = ax[2][0]; a.plot(ts, np.array([x['yaw'] for x in rows]), lw=0.7, color=colors[i])
        a = ax[2][1]; a.plot(ts, alt, lw=0.9, color=colors[i])

    ax[0][0].set_title('horizontal drift from seed [m]'); ax[0][0].set_ylabel('m')
    ax[0][1].set_title('|vh| [m/s]'); ax[0][1].set_ylabel('m/s')
    ax[1][0].set_title('roll [deg]'); ax[1][1].set_title('pitch [deg]')
    ax[2][0].set_title('yaw [deg]'); ax[2][1].set_title('altitude [m]')
    for a in ax.flat:
        a.grid(alpha=0.3); a.set_xlabel('t since seed [s]')
    ax[0][0].legend(fontsize=6, ncol=2, loc='upper left')
    fig.suptitle('KF-GINS no-GNSS 10-round regression (each ~3 min)')
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    png = os.path.join(outdir, 'report.png')
    fig.savefig(png, dpi=110)
    print('figure ->', png)

    json.dump(results, open(os.path.join(outdir, 'summary.json'), 'w'), indent=1)
    return 0

if __name__ == '__main__':
    sys.exit(main())
