# -*- coding: utf-8 -*-
"""BMP585 修复验证: 无界重试链路四级监测 (注册->打开->入环->入引擎)."""
import subprocess, re, struct, time, sys, io, math

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8")
CLI = r"D:\STM32CubeCLT\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"

LOCK, CTX, G_RUN, G_SOL, RTTICK = 0x240010B0, 0x240081D0, 0x2400AD48, 0x2400ADB8, 0x2400AEAC

def rd(addr, nbytes):
    for _ in range(3):
        p = subprocess.run([CLI, "-c", "port=SWD mode=HotPlug", "-r32", hex(addr), str(nbytes)],
                           capture_output=True, text=True, timeout=30)
        words = []
        for line in p.stdout.splitlines():
            m = re.match(r"^0x[0-9A-F]{8}\s*:\s*(.*)$", line.strip())
            if m:
                words += [int(w, 16) for w in m.group(1).split()]
        if len(words) >= nbytes // 4:
            return words
    return None

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 1080.0
t0 = time.time()
prev_baro = None
while time.time() - t0 < DUR:
    tick = rd(RTTICK, 4)
    lock = rd(LOCK, 4)
    ctx  = rd(CTX, 8)
    gr   = rd(G_RUN, 28)
    sol  = rd(G_SOL, 40)
    if not all([lock, ctx, gr, sol]):
        time.sleep(5); continue
    b = struct.pack("<10I", *sol[:10])
    f64 = lambda o: struct.unpack_from("<d", b, o)[0]
    uptime = gr[0] / 1000.0
    baro_cnt = gr[6]
    line = (f"t={time.time()-t0:6.0f}s tick={tick[0] if tick else -1} up={uptime:5.0f}s reg={lock[0]} "
            f"dev={ctx[0]:#x}/{ctx[1]:#x} baro_cnt={baro_cnt}")
    if sol[0]:  # ready
        lat, lon, alt = f64(16), f64(24), f64(32)
        line += f" | ready lat={lat:.5f} lon={lon:.5f} alt={alt:.1f}"
    print(line); sys.stdout.flush()
    if lock[0] and ctx[0] and baro_cnt and prev_baro is not None and baro_cnt > prev_baro:
        rate = (baro_cnt - prev_baro) / (time.time() - t_prev)
        print(f"*** BARO CHAIN ONLINE: obs rate {rate:.1f}/s ***")
        break
    prev_baro = baro_cnt if (lock[0] and ctx[0]) else None
    t_prev = time.time()
    time.sleep(60)
