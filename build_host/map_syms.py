#!/usr/bin/env python3
"""One-shot: resolve static-variable addresses from Flight.map by object
ownership (several files define same-name statics like ctx/s_cal).
Prints assignments for swd_magcal_session.py constants."""
import re

MAP = '../cmake-build-debug/Flight.map'
WANT = {
    ('ctx', 'imu_data.c'): 'IMU_CTX',
    ('ctx', 'mag_data.c'): 'MAGCTX',
    ('s_cal', 'mag_calib.c'): 'S_CAL',
    ('s_store', 'calib_store.c'): 'CALIB',
    ('g_run', 'gins_bridge.cpp'): 'G_RUN',
    ('_ZL5g_sol', 'gins_bridge.cpp'): 'GSOL',
}

cur_obj = ''
result = {}
for line in open(MAP, encoding='utf-8', errors='replace'):
    m = re.search(r'(\S+\.[co]\.obj)\s*$', line)
    if m:
        cur_obj = m.group(1).replace(chr(92), '/')
    m2 = re.match(r'\s*\.bss\.([A-Za-z0-9_]+|_ZL[0-9]+[A-Za-z0-9_]*)\s+(0x[0-9a-f]+)', line)
    if m2:
        sym, addr = m2.group(1), m2.group(2)
        for (name, src), tag in WANT.items():
            if sym == name and src in cur_obj and tag not in result:
                result[tag] = (addr, cur_obj.split('/')[-1])

for tag in sorted(result):
    print(tag, result[tag][0], result[tag][1])
missing = [t for t in WANT.values() if t not in result]
if missing:
    print('MISSING:', missing)
