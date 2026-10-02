# -*- coding: utf-8 -*-
"""带姿态遥测的综合监测: 观察解算是否 NaN / 冻结 / 正常."""
import subprocess, re, struct, time, sys, io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
CLI = r"D:\STM32CubeCLT\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"

A = dict(
    rt_tick=0x2400ae88, nav_cnt=0x240011b0,
    pushed=0x24008244, g_map=0x2400a0c0,
    g_sol=0x2400ad98, g_run=0x2400ad30, g_nan=0x2400ace8,
)

def rd(addr, nbytes, tries=4):
    for _ in range(tries):
        p = subprocess.run([CLI, "-c", "port=SWD mode=HotPlug", "-r32", hex(addr), str(nbytes)],
                           capture_output=True, text=True, timeout=30)
        words = []
        for line in p.stdout.splitlines():
            m = re.match(r"^0x[0-9A-F]{8}\s*:\s*(.*)$", line)
            if m: words += [int(w, 16) for w in m.group(1).split()]
        if len(words) >= nbytes // 4: return words
    return None

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 420.0
t0 = time.time()
prev = None
while time.time() - t0 < DUR:
    tick = rd(A["rt_tick"], 4)
    nav = rd(A["nav_cnt"], 20)
    push = rd(A["pushed"], 4)
    gm = rd(A["g_map"], 128)
    sol = rd(A["g_sol"], 160)
    nan = rd(A["g_nan"], 72)
    if not all([tick, nav, push, gm, sol, nan]):
        time.sleep(2); continue
    b = struct.pack("<40I", *sol[:40])
    f64 = lambda o: struct.unpack_from("<d", b, o)[0]
    is_nan = (f64(64) != f64(64)) or (f64(80) != f64(80))
    cur = dict(t=time.time(), tick=tick[0], rmc=nav[0], gga=nav[1], zda=nav[2],
               csum=nav[3], push=push[0], pair_ok=gm[24], ready=sol[0],
               imu=sol[22], gnss=sol[23],
               rpy=(f64(64), f64(72), f64(80)), nan=nan[0], nanmask=nan[1])
    if prev:
        dt = cur["t"] - prev["t"]
        tot = (cur["rmc"]-prev["rmc"])+(cur["gga"]-prev["gga"])+(cur["zda"]-prev["zda"])+(cur["csum"]-prev["csum"])
        okr = ((cur["rmc"]-prev["rmc"])+(cur["gga"]-prev["gga"])+(cur["zda"]-prev["zda"]))/max(tot,1)*100
        r, p, y = cur["rpy"]
        rpy_s = "NaN!" if is_nan else "r%+7.2f p%+7.2f y%+8.2f" % (r, p, y)
        print("[%5.0fs] %s %5.0fB/s ok%3.0f%% pair%+3d imu%5.0f/s gnss%+4.1f/s | %s nancnt=%d"
              % (time.time()-t0, "Y" if cur['tick']!=prev['tick'] else "N",
                 (cur['push']-prev['push'])/dt, okr, cur['pair_ok']-prev['pair_ok'],
                 (cur['imu']-prev['imu'])/dt, (cur['gnss']-prev['gnss'])/dt,
                 rpy_s, cur['nan']))
        sys.stdout.flush()
    prev = cur
    time.sleep(8)
print("done")
