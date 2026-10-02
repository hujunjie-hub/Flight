# -*- coding: utf-8 -*-
"""干净启动后活体监测 GNSS 链路 (不 halt): 按地址解析, 防响应错位."""
import subprocess, socket, time, sys, io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
OPENOCD = r"D:\STM32CubeCLT\OpenOCD\bin\openocd.exe"
M0AR = 0x24011F2C
FIFO = M0AR - 12
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0

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
    s.settimeout(0.8)
    try:
        while True:
            c = s.recv(65536)
            if not c: break
            out += c
    except socket.timeout: pass
    return out.decode(errors="replace").replace("\x00", "\n")

def mdw(addr, n=1):
    """单命令读, 带重试, 按返回地址行匹配."""
    for _ in range(3):
        s.sendall(f"mdw {addr:#x} {n}\n".encode())
        txt = drain()
        for line in txt.split("\n"):
            line = line.strip()
            if line.startswith(f"0x{addr:x}:") or line.startswith(f"0x{addr:X}:"):
                try:
                    return [int(x, 16) for x in line.split(":", 1)[1].split()]
                except ValueError:
                    pass
    return None

def snap():
    nd = mdw(0x40020044)                 # NDTR
    fi = mdw(FIFO, 3)                    # fifo
    st = mdw(0x24004398, 3)              # pushed popped lost
    nv = mdw(0x2400a9d0, 5)              # rmc gga zda csum field
    if not (nd and fi and st and nv):
        return None
    return dict(t=time.time(), ndtr=nd[0] & 0xFFFF, put=fi[1] & 0xFFFF,
                get=fi[1] >> 16, pushed=st[0], popped=st[1], lost=st[2],
                rmc=nv[0], gga=nv[1], zda=nv[2], csum=nv[3], fld=nv[4])

try:
    t0 = time.time()
    prev = None
    while time.time() - t0 < DUR:
        cur = snap()
        if cur:
            if prev:
                dt = cur["t"] - prev["t"]
                print(f"[{time.time()-t0:5.1f}s] {((cur['pushed']-prev['pushed'])/dt):6.0f} B/s "
                      f"rmc{cur['rmc']-prev['rmc']:+3d} gga{cur['gga']-prev['gga']:+3d} "
                      f"zda{cur['zda']-prev['zda']:+3d} csum{cur['csum']-prev['csum']:+5d} "
                      f"lost={cur['lost']} pend={(cur['put']-cur['get'])%512}")
            else:
                print(f"[{time.time()-t0:5.1f}s] 初始: pushed={cur['pushed']} "
                      f"rmc={cur['rmc']} gga={cur['gga']} zda={cur['zda']} "
                      f"csum={cur['csum']} lost={cur['lost']}")
            prev = cur
        time.sleep(3.0)
finally:
    try:
        s.sendall(b"shutdown\n"); time.sleep(0.3)
    except: pass
    try: s.close()
    except: pass
    try: ocd.wait(timeout=5)
    except: ocd.terminate()
