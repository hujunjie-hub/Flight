#!/usr/bin/env python3
"""磁/气压原始流质量分析: magout/barout 标签文本 -> 噪声/稳定性/速率"""
import re, sys, math

ANSI = re.compile(r'\x1b\[[0-9;]*m')
MAG = re.compile(r'time:([\d.]+) act:(\d) q:(\d+) '
                 r'rx:(-?[\d.]+) ry:(-?[\d.]+) rz:(-?[\d.]+) '
                 r'cx:(-?[\d.]+) cy:(-?[\d.]+) cz:(-?[\d.]+) '
                 r'rmag:([\d.]+) cmag:([\d.]+)')
BARO = re.compile(r'time:([\d.]+) pa:([\d.]+) pac:([\d.]+) temp:(-?[\d.]+)')

def st(v):
    n=len(v); mu=sum(v)/n
    var=sum((x-mu)**2 for x in v)/max(1,n-1)
    return mu, math.sqrt(var), min(v), max(v)

def main(path):
    mag, baro = [], []
    for line in open(path, errors='replace'):
        line = ANSI.sub('', line)
        m = MAG.search(line)
        if m:
            mag.append((float(m.group(1)), int(m.group(2)), int(m.group(3)),
                        float(m.group(4)), float(m.group(5)), float(m.group(6)),
                        float(m.group(7)), float(m.group(8)), float(m.group(9)),
                        float(m.group(10)), float(m.group(11))))
        b = BARO.search(line)
        if b:
            baro.append((float(b.group(1)), float(b.group(2)), float(b.group(3)), float(b.group(4))))

    print(f'mag lines={len(mag)}  baro lines={len(baro)}')
    if mag:
        dt = (mag[-1][0]-mag[0][0])
        rate = (len(mag)-1)/dt if dt>0 else 0
        gaps = [mag[i+1][0]-mag[i][0] for i in range(len(mag)-1)]
        _,gmin,_,gmax = st(gaps)
        print(f'\n== MAG (BMM350) ==  rate={rate:.1f}Hz  dt: mean={sum(gaps)/len(gaps)*1000:.1f}ms min={gmin*1000:.1f} max={gmax*1000:.1f}ms')
        for i, name in ((3,'rx'),(4,'ry'),(5,'rz'),(6,'cx'),(7,'cy'),(8,'cz')):
            mu,sd,mn,mx = st([x[i] for x in mag])
            print(f'  {name}: mean={mu:8.3f} std={sd:7.4f} min={mn:8.3f} max={mx:8.3f} uT')
        mu,sd,mn,mx = st([x[9] for x in mag])
        print(f'  rmag: mean={mu:8.3f} std={sd:7.4f} min={mn:8.3f} max={mx:8.3f} uT (地磁场约46uT@深圳)')
        mu2,sd2,_,_ = st([x[10] for x in mag])
        print(f'  cmag: mean={mu2:8.3f} std={sd2:7.4f} uT')
        q1 = sum(1 for x in mag if x[2]&1)
        act = sum(1 for x in mag if x[1])
        print(f'  quality干扰位率={q1/len(mag)*100:.1f}%  act(驱动异常)率={act/len(mag)*100:.2f}%')
        # 趋势: 首15s vs 末15s 均值
        n15 = min(len(mag), int(15*rate))
        if len(mag) > 2*n15:
            for i,name in ((3,'rx'),(4,'ry'),(5,'rz')):
                h1,_ = st([x[i] for x in mag[:n15]]); h2,_ = st([x[i] for x in mag[-n15:]])
                print(f'  drift {name}: {h1:.3f} -> {h2:.3f} uT ({h2-h1:+.3f})')
    if baro:
        dt = (baro[-1][0]-baro[0][0])
        rate = (len(baro)-1)/dt if dt>0 else 0
        print(f'\n== BARO (BMP585) ==  rate={rate:.1f}Hz')
        mu,sd,mn,mx = st([x[1] for x in baro])
        print(f'  pa   : mean={mu:.2f} std={sd:.4f} min={mn:.2f} max={mx:.2f} Pa')
        mu,sd,mn,mx = st([x[3] for x in baro])
        print(f'  temp : mean={mu:.2f} std={sd:.4f} C')
        # 气压->高度噪声
        p = [x[1] for x in baro]
        mu,sd,_,_ = st(p)
        h_noise = sd/ (1.225*9.80665) * 1000  # mm, 近似 ρgh
        print(f'  气压噪声折算高度噪声 ≈ {h_noise:.2f} mm (std)')
        n10 = max(2,len(p)//9)
        seg1,_ = st(p[:n10]); seg2,_ = st(p[-n10:])
        drift_pa = (seg2-seg1)/ (dt* (n10/len(p)) ) * 60  # rough per min — 用首末10%段
        print(f'  pa 首末段差 {seg2-seg1:+.2f} Pa over {dt:.0f}s ≈ {(seg2-seg1)*8.3:.2f} m 高度趋势')

if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'raw90.bin')
