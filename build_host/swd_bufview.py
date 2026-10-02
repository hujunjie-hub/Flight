# -*- coding: utf-8 -*-
"""连续快照 DMA buffer, 从 put 位置回溯展开最近 448B, 看原始流的句子完整性."""
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
    s.settimeout(0.6)
    try:
        while True:
            c = s.recv(65536)
            if not c: break
            out += c
    except socket.timeout: pass
    return out.decode(errors="replace")

drain()

def parse_mdb(txt):
    buf = bytearray()
    for line in txt.split("\n"):
        line = line.strip()
        if line.startswith("0x") and ":" in line:
            for tok in line.split(":", 1)[1].split():
                try: buf.append(int(tok, 16) & 0xFF)
                except ValueError: pass
    return bytes(buf)

def snapshot():
    txt = ""
    s.sendall(("halt\n"
               "mdw 0x40020044\n"
               f"mdw {M0AR-12:#x} 3\n"
               f"mdb {M0AR:#x} 512\n"
               "resume\n").encode())
    txt = drain()
    ndtr = put = get = None
    for line in txt.split("\n"):
        line = line.strip()
        if line.startswith("0x40020044:"):
            ndtr = int(line.split(":", 1)[1].strip(), 16) & 0xFFFF
        elif line.startswith(f"0x{M0AR-12:x}:") or line.startswith(f"0x{M0AR-12:X}:"):
            w = [int(t, 16) for t in line.split(":", 1)[1].split()]
            put, get = w[1] & 0xFFFF, w[1] >> 16
    return ndtr, put, get, parse_mdb(txt)

N_SNAP = int(sys.argv[1]) if len(sys.argv) > 1 else 10
snaps = []
try:
    for k in range(N_SNAP):
        ndtr, put, get, buf = snapshot()
        snaps.append((ndtr, put, get, buf))
        time.sleep(0.12)
finally:
    try:
        s.sendall(b"shutdown\n"); time.sleep(0.3)
    except: pass
    try: s.close()
    except: pass
    try: ocd.wait(timeout=5)
    except: ocd.terminate()

# 展开分析: 每个快照取 [put-448, put) 区间 (应用最近读走的字节流)
for k, (ndtr, put, get, buf) in enumerate(snaps):
    if put is None: continue
    start = (put - 448) % 512
    seq = bytes(buf[(start + i) % 512] for i in range(448))
    hw = 512 - ndtr if ndtr is not None else -1
    txtv = seq.decode("ascii", errors="replace")
    print(f"=== [{k}] NDTR={ndtr} hw={hw} put={put} get={get} (put-hw={((put-hw)%512)}) ===")
    print(txtv)
    print()
