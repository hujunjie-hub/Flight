# -*- coding: utf-8 -*-
"""决定性填充实验: 全 buffer 写 0xAA, 等 pushed 增量 >= 600B, 全量回读比对.
同时输出 nav 当前定位状态."""
import subprocess, socket, time, sys, io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
OPENOCD = r"D:\STM32CubeCLT\OpenOCD\bin\openocd.exe"
M0AR = 0x2401181C

ocd = subprocess.Popen(
    [OPENOCD, "-f", "interface/stlink.cfg", "-f", "target/stm32h7x.cfg",
     "-c", "gdb_port disabled"],
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

s = None
for _ in range(40):
    time.sleep(0.25)
    try:
        s = socket.create_connection(("127.0.0.1", 4444), timeout=2); break
    except ConnectionRefusedError:
        pass
if s is None:
    ocd.terminate(); raise RuntimeError("OpenOCD telnet not reachable")

def drain():
    out = b""
    s.settimeout(1.0)
    try:
        while True:
            c = s.recv(65536)
            if not c: break
            out += c
    except socket.timeout: pass
    return out.decode(errors="replace").replace("\x00", "\n")

def cmds(clist, wait=0.4):
    s.sendall(("".join(c + "\n" for c in clist)).encode())
    time.sleep(wait)
    return drain()

def words(txt):
    vals = []
    for line in txt.split("\n"):
        line = line.strip()
        if line.startswith("0x") and ":" in line:
            vals += [int(t, 16) for t in line.split(":", 1)[1].split()]
    return vals

def pushed():
    w = words(cmds(["mdw 0x24004398 1"]))   # ctx.st.pushed @ +4
    return w[0] if w else None

def mdb(addr, n):
    w = words(cmds([f"mdb {addr:#x} {n}"], 0.5))
    return w

try:
    # nav 状态 (定位/卫星/utc)
    w = words(cmds(["mdw 0x2400a9c0 18"], 0.5))
    if len(w) >= 18:
        import struct
        b = struct.pack("<18I", *w)
        lat, lon = struct.unpack_from("<dd", b, 8)
        alt, = struct.unpack_from("<f", b, 24)
        fix, sats = b[44], b[45]
        print(f"GNSS: fix={fix} sats={sats} lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}")

    p0 = pushed()
    # 填充 0xAA x 128 words
    cmds(["halt", f"mww {M0AR:#x} 0xAAAAAAAA 128", "resume"], 0.6)
    p_fill = pushed()
    print(f"pushed: {p0} -> (fill) {p_fill}")

    # 等待 >= 700 字节新流量 (覆盖全圈+余量)
    t0 = time.time()
    while time.time() - t0 < 120:
        p = pushed()
        if p is None: break
        if p - p_fill >= 700:
            break
        time.sleep(1.0)
    p1 = pushed()
    print(f"pushed after wait: {p1} (delta {p1-p_fill}) in {time.time()-t0:.1f}s")

    # 全量回读
    buf = mdb(M0AR, 512)
    ndtr = words(cmds(["mdw 0x40020044 1"]))[0] & 0xFFFF
    wpos = 512 - ndtr
    print(f"NDTR={ndtr} wpos={wpos}")
    holes = []
    for i in range(512):
        if buf[i] == 0xAA:
            holes.append(i)
    # 0xAA 存在于 [wpos, 512) 尾部 = 尚未写到的区域, 属正常
    in_written = [i for i in holes if (i - wpos) % 512 >= 100]  # 距写指针 100B 之前仍是 AA = 空洞
    print(f"0xAA 剩余: {len(holes)}/512, 其中写指针后方(正常未写区): {len(holes)-len(in_written)}")
    print(f"疑点空洞(写指针已越过 100B 以上仍为 AA): {len(in_written)} 个 -> {in_written[:20]}")
    # 文本视图
    print("\n--- buffer 文本视图 (wpos 用 >>> 标) ---")
    for off in range(0, 512, 64):
        chunk = buf[off:off+64]
        mark = " <<< wpos" if off <= wpos < off+64 else ""
        t = "".join(chr(x) if 0x20 <= x <= 0x7e else "." for x in chunk)
        print(f"{off:3d} {M0AR+off:#x}: {t}{mark}")
finally:
    try:
        s.sendall(b"shutdown\n"); time.sleep(0.3)
    except: pass
    try: s.close()
    except: pass
    try: ocd.wait(timeout=5)
    except: ocd.terminate()
