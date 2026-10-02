# -*- coding: utf-8 -*-
"""双快照对比: 零运行是固定绝对地址 (本地写入者) 还是跟随 DMA 写指针 (线缆收到)."""
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

def parse_bytes(txt, want=512):
    buf = bytearray()
    for line in txt.split("\n"):
        line = line.strip()
        if line.startswith("0x") and ":" in line:
            for tok in line.split(":", 1)[1].split():
                try: buf.append(int(tok, 16) & 0xFF)
                except ValueError: pass
    return bytes(buf)

def snapshot():
    s.sendall(("halt\n"
               "mdw 0x40020044\n"
               f"mdb {M0AR:#x} 512\n"
               "resume\n").encode())
    time.sleep(0.5)
    txt = drain()
    ndtr = None
    for line in txt.split("\n"):
        line = line.strip()
        if line.startswith("0x40020044:"):
            ndtr = int(line.split(":", 1)[1].strip(), 16) & 0xFFFF
    return ndtr, parse_bytes(txt)

def zero_runs(buf):
    runs, i = [], 0
    while i < len(buf):
        if buf[i] == 0:
            j = i
            while j < len(buf) and buf[j] == 0: j += 1
            if j - i >= 4: runs.append((i, j - i))
            i = j
        else: i += 1
    return runs

def nonascii_runs(buf):
    """非 NMEA 可打印字节 (排除 \r \n) 的游程"""
    runs, i = [], 0
    while i < len(buf):
        b = buf[i]
        ok = (0x20 <= b <= 0x7e) or b in (0x0d, 0x0a)
        if not ok:
            j = i
            while j < len(buf):
                b = buf[j]
                if (0x20 <= b <= 0x7e) or b in (0x0d, 0x0a): break
                j += 1
            if j - i >= 2: runs.append((i, j - i))
            i = j
        else: i += 1
    return runs

snaps = []
try:
    for k in range(4):
        ndtr, buf = snapshot()
        wpos = 512 - ndtr if ndtr is not None else None
        snaps.append((wpos, buf))
        zr = zero_runs(buf)
        na = nonascii_runs(buf)
        print(f"[{k}] wpos={wpos} 零运行(>=4B): {[(hex(M0AR+a), n) for a,n in zr][:8]}")
        print(f"    非ASCII运行(>=2B): {[(hex(M0AR+a), n) for a,n in na][:8]}")
        print(f"    零字节总数: {buf.count(0)}/512, 非打印总数: "
              f"{sum(1 for b in buf if not(0x20<=b<=0x7e or b in (13,10)))}/512")
        time.sleep(0.25)
finally:
    try:
        s.sendall(b"shutdown\n"); time.sleep(0.3)
    except: pass
    try: s.close()
    except: pass
    try: ocd.wait(timeout=5)
    except: ocd.terminate()

# 打印一份完整 buffer 文本视图
wpos, buf = snaps[-1]
print("\n--- 最后一次快照 buffer 全文 (按绝对地址线性, | 标记写指针) ---")
for off in range(0, 512, 64):
    chunk = buf[off:off+64]
    mark = " <<< wpos" if off <= wpos < off+64 else ""
    txtc = "".join(chr(b) if 0x20 <= b <= 0x7e else ("." if b not in (0,) else "0") for b in chunk)
    print(f"{M0AR+off:#x}: {txtc}{mark}")
