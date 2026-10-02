import re, sys

# rtconfig.h: #define SYM [value]
rh = {}
for ln in open('rtconfig.h', encoding='utf-8', errors='ignore'):
    m = re.match(r'#define\s+([A-Za-z0-9_]+)(?:\s+(.*?))?\s*(?://.*)?$', ln.strip())
    if m and not m.group(2):
        rh[m.group(1)] = True
    elif m:
        rh[m.group(1)] = m.group(2).strip()

# .config: CONFIG_X=y / =val / "# CONFIG_X is not set"
cfg_on, cfg_off = {}, set()
for ln in open('.config', encoding='utf-8', errors='ignore'):
    ln = ln.strip()
    m = re.match(r'CONFIG_([A-Za-z0-9_]+)=(.*)', ln)
    if m:
        v = m.group(2)
        cfg_on[m.group(1)] = True if v == 'y' else v
        continue
    m = re.match(r'#\s*CONFIG_([A-Za-z0-9_]+) is not set', ln)
    if m:
        cfg_off.add(m.group(1))

# RT-Thread kconfig symbols map 1:1 to rtconfig defines (no CONFIG_ prefix)
mismatch = []
for sym, v in sorted(cfg_on.items()):
    if sym.startswith('BSP_') or sym.startswith('RT_') or sym.startswith('ULOG') or sym.startswith('UTEST') or sym.startswith('FINSH') or sym.startswith('RT_'):
        if sym not in rh:
            mismatch.append(f".config 有而 rtconfig.h 缺: {sym}={v}")
        elif v is not True and rh[sym] is True:
            mismatch.append(f"值类型不一致: {sym} .config={v} rtconfig.h=无值")
for sym in cfg_off:
    if sym in rh and rh[sym] is True:
        mismatch.append(f".config 关闭但 rtconfig.h 开启: {sym}")

print(f"rtconfig.h defines: {len(rh)}, .config on: {len(cfg_on)}, off: {len(cfg_off)}")
print(f"mismatches: {len(mismatch)}")
for m_ in mismatch[:30]:
    print("  ", m_)
