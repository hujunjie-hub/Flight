#!/usr/bin/env python3
"""Evaluate KF-GINS no-GNSS output quality from a captured COM log.

Input : raw capture file (bytes) containing `ready:... fused_data` lines
         plus board boot/GINS log lines.
Output: console report + parsed CSV + PNG figure.

Usage: python eval_nognss.py <capture.bin> [outdir]
"""
import sys, re, json, os
import numpy as np

LINE_RE = re.compile(rb'((?:[A-Za-z_]+:-?\d+(?:\.\d+)?\s+)+fused_data)')
KV_RE = re.compile(r'([A-Za-z_]+):(-?\d+(?:\.\d+)?)')
LOG_RE = re.compile(r'\[(\d+)\]\s+([IWE])/(gins|gnss|timebase|adis|imudata|magdata|barodata|main)(.*)')

# WGS84 meters per degree (classic series approximation)
def m_per_deg(lat_deg):
    p = np.radians(lat_deg)
    mlat = 111132.954 - 559.822*np.cos(2*p) + 1.175*np.cos(4*p)
    mlon = 111412.84*np.cos(p) - 93.5*np.cos(3*p) + 0.118*np.cos(5*p)
    return mlat, mlon

def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0

def stats(name, arr, unit='', skip_first=0):
    a = np.asarray(arr[skip_first:], dtype=float)
    if a.size == 0:
        return f'{name:16s}: NO DATA'
    return (f'{name:16s}: mean={np.mean(a):+10.4f} std={np.std(a):8.4f} '
            f'min={np.min(a):+10.4f} max={np.max(a):+10.4f} '
            f'last={a[-1]:+10.4f} {unit}'.rstrip())

def main():
    path = sys.argv[1]
    outdir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(os.path.abspath(path))
    os.makedirs(outdir, exist_ok=True)
    raw = open(path, 'rb').read()
    text = raw.decode('utf-8', errors='replace')

    # ---------- parse fused_data lines ----------
    rows = []
    for m in LINE_RE.finditer(raw):
        d = dict(KV_RE.findall(m.group(1).decode('ascii')))
        if 'time' not in d or 'ready' not in d:   # 半行 (采集开始/复位截断)
            continue
        try:
            rows.append({k: float(v) for k, v in d.items()})
        except ValueError:
            pass
    # ---------- extract interesting log lines ----------
    logs = []
    for line in text.splitlines():
        line = re.sub(r'\x1b\[[0-9;]*m', '', line)  # strip ANSI colors
        lm = LOG_RE.search(line)
        if lm:
            t_ms, lvl, tag, msg = lm.groups()
            if any(k in msg for k in ('播种', 'seed', 'Seed', '对准', 'align', 'Align',
                                      'NOGNSS', 'nognss', 'WARN', '复位', 'reset', 'reset',
                                      'reseed', 'cov', 'NaN', 'nan', 'drop', 'DR', 'init',
                                      'ready', '初始化')):
                logs.append((int(t_ms), lvl, tag, msg.strip()))

    if not rows:
        print('NO fused_data lines found! (%d bytes, %d log lines)' % (len(raw), len(logs)))
        for t, l, tg, m in logs[:40]:
            print(f'  [{t/1000:9.1f}s {l}/{tg}] {m[:120]}')
        return 1

    keys = list(rows[0].keys())
    A = {k: np.array([r.get(k, np.nan) for r in rows]) for k in keys}
    n = len(rows)
    t = A['time']
    t0 = t[0]

    print('='*100)
    print(f'KF-GINS NO-GNSS QUALITY EVAL  —  {os.path.basename(path)}')
    print('='*100)
    print(f'fused lines: {n}   span: {t0:.1f} -> {t[-1]:.1f} s  ({t[-1]-t0:.1f} s, '
          f'avg {(n-1)/(t[-1]-t0):.2f} Hz)')

    # ---------- ready timeline ----------
    ready = A['ready']
    rising = np.where((ready[1:] > 0.5) & (ready[:-1] <= 0.5) & np.isfinite(ready[:-1]))[0]
    falling = np.where((ready[1:] <= 0.5) & (ready[:-1] > 0.5))[0]
    ready_ratio = np.mean(ready > 0.5)
    print(f'\nready=1 ratio: {ready_ratio*100:.1f}%   0->1 at: '
          f'{[f"{t[i]-t0:.1f}s" for i in rising]}   1->0 at: {[f"{t[i]-t0:.1f}s" for i in falling]}')

    # ---------- NaN / sanity ----------
    nan_cnt = {k: int(np.sum(~np.isfinite(A[k]))) for k in ('roll','pitch','yaw','vn','ve','vd','lat','lon','alt') if k in A}
    bad_geo = int(np.sum((np.abs(A['lat']) > 90) | (np.abs(A['lon']) > 180)))
    print(f'non-finite values: { {k:v for k,v in nan_cnt.items() if v} or "none" }   lat/lon out-of-range rows: {bad_geo}')

    # ---------- link health: counter rates ----------
    print('\n--- link counter rates (per second) ---')
    for cnt, expect in (('imu_data', 1000), ('gnss_data', 10), ('mag_calib_data', 100), ('baro_calib_data', 100)):
        if cnt not in A: continue
        c = A[cnt]
        dt = np.diff(t); dc = np.diff(c)
        rate = np.where(dt > 0, dc/dt, 0)
        ok = rate > 0.5*expect if expect > 0 else rate == 0
        print(f'{cnt:16s}: total={c[-1]-c[0]:9.0f}  avg={np.mean(rate[rate>0]) if np.any(rate>0) else 0:8.1f}/s '
              f'(expect ~{expect})  zero-rate spans: {int(np.sum(rate<=0.5*expect))} samples')
        if cnt == 'imu_data':
            # outage detection: consecutive samples with rate below 500 Hz
            bad = rate < 500
            outages = []
            i = 0
            while i < len(bad):
                if bad[i]:
                    j = i
                    while j < len(bad) and bad[j]: j += 1
                    dur = t[j] - t[i] if j < len(t) else t[-1]-t[i]
                    if dur > 0.3:
                        outages.append((t[i]-t0, t[min(j, len(t)-1)]-t0, dur, c[min(j, len(c)-1)]-c[i]))
                    i = j
                else:
                    i += 1
            A['_outages'] = outages
            print(f'    IMU outages (>0.3 s): {len(outages)}')
            for (a, b, d, lost) in outages[:10]:
                print(f'      t={a:8.1f}s -> {b:8.1f}s  dur={d:6.2f}s  lost~{lost:.0f} samples')

    # ---------- logs of interest ----------
    print('\n--- board log highlights ---')
    for ts, lvl, tag, msg in logs[:60]:
        print(f'  [{ts/1000:9.1f}s {lvl}/{tag}] {msg[:140]}')

    # ---------- post-seed quality ----------
    print('\n--- post-seed (ready=1) navigation quality ---')
    idx = np.where(ready > 0.5)[0]
    if len(idx) < 10:
        print('INSUFFICIENT ready=1 samples for quality stats')
        return 0
    s = idx[0]
    ts = t[s:] - t[s]
    lat, lon, alt = A['lat'][s:], A['lon'][s:], A['alt'][s:]
    mlat, mlon = m_per_deg(lat[0])
    dN = (lat - lat[0]) * mlat
    dE = (lon - lon[0]) * mlon
    dH = np.hypot(dN, dE)
    vh = np.hypot(A['vn'][s:], A['ve'][s:])
    roll, pitch, yaw = A['roll'][s:], A['pitch'][s:], A['yaw'][s:]
    yaw_u = np.degrees(np.unwrap(np.radians(yaw)))
    span_min = (ts[-1]-ts[0])/60.0

    print(f'seed position (first ready=1): lat={lat[0]:.7f} lon={lon[0]:.7f} alt={alt[0]:.2f} m')
    print(f'  config seed expected: lat=22.64 lon=114.01 alt=50.0 -> '
          f'delta=({(lat[0]-22.64)*mlat:+.1f} m N, {(lon[0]-114.01)*mlon:+.1f} m E, {alt[0]-50:+.1f} m)')
    print(f'eval span: {ts[0]:.1f} -> {ts[-1]:.1f} s ({span_min:.2f} min)\n')
    print(stats('roll', roll, 'deg', skip_first=30))
    print(stats('pitch', pitch, 'deg', skip_first=30))
    print(stats('yaw', yaw_u, 'deg', skip_first=30))
    print(f'{"att drift(30s->end)":16s}: droll={roll[-1]-roll[30]:+.3f} dpitch={pitch[-1]-pitch[30]:+.3f} '
          f'dyaw={wrap180(yaw_u[-1]-yaw_u[30]):+.3f} deg')
    print()
    print(stats('vn', A['vn'][s:], 'm/s', skip_first=30))
    print(stats('ve', A['ve'][s:], 'm/s', skip_first=30))
    print(stats('vd', A['vd'][s:], 'm/s', skip_first=30))
    print(stats('|vh|', vh, 'm/s', skip_first=30))
    print()
    print(f'{"horiz drift":16s}: end={dH[-1]:.2f} m over {span_min:.2f} min -> {dH[-1]/max(span_min,1e-9):.2f} m/min')
    seg = 60.0
    k = 1
    print('    per-minute horiz drift rate:')
    while (k-1)*seg < ts[-1]:
        m_ = (ts >= (k-1)*seg) & (ts < k*seg)
        if np.any(m_) and dH[m_].size > 2:
            dd = dH[m_][-1]
            print(f'      min {k}: cumulative {dd:8.2f} m')
        k += 1
    print(stats('alt', alt, 'm', skip_first=30))
    print(f'{"alt drift":16s}: {alt[-1]-alt[30]:+.3f} m (30s->end)')

    # ---------- CSV dump ----------
    csvp = os.path.join(outdir, 'nognss_quality_parsed.csv')
    with open(csvp, 'w') as f:
        f.write(','.join(keys + ['dN_m','dE_m','dH_m','vh_ms']) + '\n')
        for i in range(n):
            j = i - s
            f.write(','.join(f'{A[k][i]:.7f}' for k in keys) +
                    f',{dN[j] if j>=0 else 0:.3f},{dE[j] if j>=0 else 0:.3f},'
                    f'{dH[j] if j>=0 else 0:.3f},{vh[j] if j>=0 else 0:.4f}\n')
    print(f'\nCSV -> {csvp}')

    # ---------- figure ----------
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(5, 1, figsize=(13, 16), sharex=True)
        fig.suptitle('KF-GINS no-GNSS output quality', fontsize=14)
        a = ax[0]
        a.plot(ts, roll, label='roll', lw=0.8); a.plot(ts, pitch, label='pitch', lw=0.8)
        a.plot(ts, yaw_u, label='yaw(unwrapped)', lw=0.8)
        a.set_ylabel('attitude [deg]'); a.legend(loc='best', fontsize=8); a.grid(alpha=0.3)
        a.set_title('attitude')
        a = ax[1]
        a.plot(ts, A['vn'][s:], label='vn', lw=0.8); a.plot(ts, A['ve'][s:], label='ve', lw=0.8)
        a.plot(ts, A['vd'][s:], label='vd', lw=0.8); a.plot(ts, vh, label='|vh|', lw=1.2, color='k')
        a.set_ylabel('velocity [m/s]'); a.legend(loc='best', fontsize=8); a.grid(alpha=0.3)
        a.set_title('velocity')
        a = ax[2]
        a.plot(ts, dN, label='dN', lw=0.8); a.plot(ts, dE, label='dE', lw=0.8)
        a.plot(ts, dH, label='|horiz|', lw=1.2, color='k')
        a.set_ylabel('drift from seed [m]'); a.legend(loc='best', fontsize=8); a.grid(alpha=0.3)
        a.set_title('position drift (relative to first ready=1)')
        a = ax[3]
        a.plot(ts, alt, color='tab:red', lw=0.8)
        a.set_ylabel('altitude [m]'); a.grid(alpha=0.3); a.set_title('altitude (baro-anchored)')
        a = ax[4]
        cnt = A['imu_data']; dt2 = np.diff(t); r2 = np.where(dt2 > 0, np.diff(cnt)/dt2, 0)
        a.plot((t[1:]-t[s])/1.0, np.clip(r2, 0, 1200), lw=0.6)
        a.axhline(1000, color='g', ls='--', lw=0.8, label='1000 Hz nominal')
        a.set_ylabel('imu rate [Hz]'); a.set_xlabel('t since seed [s]'); a.grid(alpha=0.3); a.legend(fontsize=8)
        a.set_title('IMU sample rate at solver (ADIS16505 health)')
        for (oa, ob, d, l) in A.get('_outages', []):
            for a_ in ax:
                a_.axvspan(oa-t[s], ob-t[s], color='r', alpha=0.15)
        pngp = os.path.join(outdir, 'nognss_quality_report.png')
        fig.tight_layout(rect=[0, 0, 1, 0.97])
        fig.savefig(pngp, dpi=110)
        print(f'PNG -> {pngp}')

        fig2, a = plt.subplots(figsize=(7, 7))
        sc = a.scatter(dE, dN, c=ts, s=4, cmap='viridis')
        a.plot(0, 0, 'r+', ms=12, mew=2)
        a.set_aspect('equal'); a.grid(alpha=0.3)
        a.set_xlabel('East [m]'); a.set_ylabel('North [m]')
        a.set_title(f'horizontal drift trajectory (end: {dH[-1]:.1f} m, {dH[-1]/max(span_min,1e-9):.2f} m/min)')
        fig2.colorbar(sc, label='t since seed [s]')
        fig2.tight_layout()
        png2 = os.path.join(outdir, 'nognss_quality_traj.png')
        fig2.savefig(png2, dpi=110)
        print(f'PNG -> {png2}')
    except Exception as e:
        print('plot skipped:', e)

    # ---------- machine-readable summary ----------
    summary = dict(
        file=os.path.basename(path), lines=n, ready_ratio=float(ready_ratio),
        seed_pos=[float(lat[0]), float(lon[0]), float(alt[0])],
        span_s=float(ts[-1]-ts[0]),
        att_std_deg=[float(np.std(roll[30:])), float(np.std(pitch[30:])), float(np.std(yaw_u[30:]))],
        att_drift_deg=[float(roll[-1]-roll[30]), float(pitch[-1]-pitch[30]), float(wrap180(yaw_u[-1]-yaw_u[30]))],
        v_end=[float(A['vn'][s:][-1]), float(A['ve'][s:][-1]), float(A['vd'][s:][-1])],
        vh_max=float(np.max(vh)), drift_end_m=float(dH[-1]),
        drift_rate_m_per_min=float(dH[-1]/max(span_min, 1e-9)),
        alt_drift_m=float(alt[-1]-alt[30]),
        imu_outages=[(float(a), float(b), float(d)) for a, b, d, _ in A.get('_outages', [])],
    )
    jp = os.path.join(outdir, 'nognss_quality_summary.json')
    json.dump(summary, open(jp, 'w'), indent=2)
    print(f'JSON -> {jp}')
    return 0

if __name__ == '__main__':
    sys.exit(main())
