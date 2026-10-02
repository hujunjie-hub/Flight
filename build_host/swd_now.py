# -*- coding: utf-8 -*-
"""当前链路全面体检: M0AR/CR3/ISR/NDTR 增速 + 环计数器增速 + 解析计数增速."""
import subprocess, socket, time, sys, io, re

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
OPENOCD = r"D:\STM32CubeCLT\OpenOCD\bin\openocd.exe"

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
    s.settimeout(1.2)
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

def snap():
    w = words(cmds([
        "mdw 0x40020044",          # DMA1_S2 NDTR
        "mdw 0x4002004c",          # M0AR
        "mdw 0x40004408",          # USART2 CR3
        "mdw 0x4000441c",          # USART2 ISR
        "mdw 0x24004394",          # gnss_raw ctx: st{inited,pushed,popped,lost}
        "mdw 0x2400a9c0+80 5",     # nav: rmc,gga,zda,csum,field (nav+80)
    ]))
    return w

try:
    a = snap()
    time.sleep(5.0)
    b = snap()
    names = ["NDTR", "M0AR", "CR3", "ISR", "raw.inited", "raw.pushed",
             "raw.popped", "raw.lost", "nav.rmc", "nav.gga"]
    for i, n in enumerate(names):
        if i < len(a) and i < len(b):
            print(f"{n:12s}: {a[i]:#010x} -> {b[i]:#010x}  "
                  f"delta={b[i]-a[i]:+d}")
    print(f"nav.zda/csum/field a: {[hex(x) for x in a[8:11]]}")
    print(f"nav.zda/csum/field b: {[hex(x) for x in b[8:11]]}")
finally:
    try:
        s.sendall(b"shutdown\n"); time.sleep(0.3)
    except: pass
    try: s.close()
    except: pass
    try: ocd.wait(timeout=5)
    except: ocd.terminate()
