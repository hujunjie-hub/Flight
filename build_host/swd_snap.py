# -*- coding: utf-8 -*-
"""OpenOCD 同步快照 (telnet 4444, 流水线命令最小化 halt 窗口):
halt -> 同读 NDTR / fifo put,get / 512B DMA buffer -> resume."""
import subprocess, socket, time, sys, io, re

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
    s.settimeout(0.5)
    try:
        while True:
            c = s.recv(65536)
            if not c: break
            out += c
    except socket.timeout: pass
    return out.decode(errors="replace")

drain()  # banner

def batch(cmds):
    s.sendall(("".join(c + "\n" for c in cmds)).encode())
    txt, prompts = drain(), 0
    return txt

def parse_mdb(txt):
    buf = bytearray()
    for line in txt.split("\n"):
        if line.startswith("0x") and ":" in line:
            for tok in line.split(":", 1)[1].split():
                try: buf.append(int(tok, 16) & 0xFF)
                except ValueError: pass
    return bytes(buf)

def snapshot():
    txt = batch([
        "halt",
        "mdw 0x40020044",          # NDTR
        f"mdw {M0AR-12:#x} 3",     # fifo: bufptr, put|get, full
        f"mdb {M0AR:#x} 512",      # DMA buffer
        "resume",
    ])
    # 每条命令结果在 \x00 之后; 提取
    results = [seg for seg in txt.split("\x00") if seg.strip()]
    ndtr = put = get = None
    buf = b""
    for seg in results:
        for line in seg.split("\n"):
            line = line.strip()
            if line.startswith("0x40020044:"):
                ndtr = int(line.split(":")[1].strip(), 16) & 0xFFFF
            elif line.lower().startswith(hex(M0AR-12).lstrip("0x") if False else ""):
                pass
            if line.startswith(hex(M0AR-12)) or line.startswith(f"0x{M0AR-12:x}") or \
               line.startswith(f"0x{M0AR-12:X}"):
                w = [int(t, 16) for t in line.split(":", 1)[1].split()]
                put, get = w[1] & 0xFFFF, w[1] >> 16
    buf = parse_mdb(txt)
    return ndtr, put, get, buf

snaps = []
try:
    for k in range(14):
        ndtr, put, get, buf = snapshot()
        snaps.append((ndtr, put, get, buf))
        hw = 512 - ndtr if ndtr is not None else -1
        print(f"[{k:2d}] NDTR={ndtr} hw={hw:3} put={put} get={get} pend={(put-get)%512 if put is not None else -1}")
        time.sleep(0.15)
finally:
    try:
        s.sendall(b"shutdown\n"); time.sleep(0.3)
    except: pass
    try: s.close()
    except: pass
    try: ocd.wait(timeout=5)
    except: ocd.terminate()

print("\n--- 各快照 DMA buffer 有效区 (get..put) ---")
for k, (ndtr, put, get, buf) in enumerate(snaps):
    if put is None or get is None: continue
    n = (put - get) % 512
    if n == 0:
        print(f"[{k}] pend=0")
        continue
    valid = bytes((buf[(get + i) % 512]) for i in range(n))
    txtv = valid.decode("ascii", errors="replace")
    print(f"[{k}] pend={n}: {txtv}")
