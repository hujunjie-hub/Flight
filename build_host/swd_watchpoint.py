# -*- coding: utf-8 -*-
"""DWT 写观察点抓零值写入者: 在 fifo 区域内的零运行地址设 watchpoint,
命中后读 PC 并映射到固件符号."""
import subprocess, socket, time, sys, io, re

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
OPENOCD = r"D:\STM32CubeCLT\OpenOCD\bin\openocd.exe"
M0AR = 0x2401181C
WATCH = int(sys.argv[1], 0) if len(sys.argv) > 1 else 0x240118E0
TIMEOUT_S = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0

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
    s.settimeout(1.5)
    try:
        while True:
            c = s.recv(65536)
            if not c: break
            out += c
    except socket.timeout: pass
    return out.decode(errors="replace").replace("\x00", "\n")

def cmd(c, wait=0.25):
    s.sendall((c + "\n").encode())
    time.sleep(wait)
    return drain()

try:
    out = cmd(f"wp {WATCH:#x} 4 write")
    print("wp set:", out.strip().split("\n")[-2:])
    cmd("resume", 0.1)
    hit = None
    t0 = time.time()
    while time.time() - t0 < TIMEOUT_S:
        out = cmd("poll", 0.05)
        if "halted" in out and "watchpoint" in out.lower():
            hit = out
            break
        if "halted" in out and "running" not in out:
            hit = out  # 任何 halt 都看看
            break
        time.sleep(0.05)
    if hit:
        print("HIT:", hit.strip()[-300:])
        pc = cmd("reg pc").strip()
        print(pc)
        lr = cmd("reg lr").strip()
        print(lr)
        cmd("resume", 0.05)
    else:
        print(f"{TIMEOUT_S}s 内未命中 (CPU 未写该地址; 零可能来自 DMA 或一次性写入)")
        cmd("halt", 0.3)
        fw = cmd(f"mdw {M0AR-12:#x} 3")
        print("fifo:", fw.strip().split("\n")[-2])
        cmd("resume", 0.05)
finally:
    try:
        s.sendall(b"shutdown\n"); time.sleep(0.3)
    except: pass
    try: s.close()
    except: pass
    try: ocd.wait(timeout=5)
    except: ocd.terminate()
