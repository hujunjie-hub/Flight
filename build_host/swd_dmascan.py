# -*- coding: utf-8 -*-
"""扫描 DMA1/DMA2 全部 16 个 stream 的寄存器 + DMAMUX 请求线,
寻找指向 UART2 fifo buffer 区域 (0x2401181c +512B) 的其他 DMA 写入者."""
import subprocess, socket, time, sys, io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
OPENOCD = r"D:\STM32CubeCLT\OpenOCD\bin\openocd.exe"
TARGET_LO, TARGET_HI = 0x2401181C, 0x2401181C + 512

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
    return out.decode(errors="replace")

drain()

def getregs(addr, n):
    for attempt in range(3):
        s.sendall(f"mdw {addr:#x} {n}\n".encode())
        txt = drain().replace("\x00", "\n")
        vals = []
        for line in txt.split("\n"):
            line = line.strip()
            if line.startswith("0x") and ":" in line:
                vals += [int(t, 16) for t in line.split(":", 1)[1].split()]
        if len(vals) >= n:
            return vals
        print(f"  DBG addr={addr:#x} attempt={attempt} got {len(vals)}/{n}: {txt[:120]!r}")
    return vals

DMA_BASES = {"DMA1": 0x40020000, "DMA2": 0x40020400}
DMAMUX = 0x40020800
REQ_NAMES = {43: "USART2_RX", 44: "USART2_TX", 9: "SPI1_RX", 10: "SPI1_TX",
             11: "SPI2_RX", 12: "SPI2_TX", 13: "SPI3_RX", 14: "SPI3_TX",
             19: "SPI4_RX", 51: "USART1_RX", 52: "USART1_TX",
             61: "I2C1_RX", 62: "I2C1_TX", 63: "I2C2_RX", 64: "I2C2_TX",
             65: "I2C3_RX", 66: "I2C3_TX"}

print(f"关注区域: [{TARGET_LO:#x}, {TARGET_HI:#x})  (uart2 RX fifo DMA buffer)")
for dname, dbase in DMA_BASES.items():
    for stream in range(8):
        base = dbase + 0x10 + stream * 0x18
        regs = getregs(base, 6)
        cr, ndtr, par, m0ar, m1ar = regs[0], regs[1], regs[2], regs[3], regs[4]
        # DMAMUX channel: DMA1 -> ch 0-7, DMA2 -> ch 8-15
        muxch = stream if dname == "DMA1" else stream + 8
        mux = getregs(DMAMUX + 4 * muxch, 1)[0]
        req = mux & 0x7F
        reqname = REQ_NAMES.get(req, f"req{req}")
        en = cr & 1
        dirbits = (cr >> 6) & 3
        dirname = {0: "P2M", 1: "M2P", 2: "M2M"}.get(dirbits, "?")
        hit = ""
        for m, tag in ((m0ar, "M0AR"), (m1ar, "M1AR")):
            if TARGET_LO <= m < TARGET_HI:
                hit = f"  <<<=== {tag} 落在 UART2 FIFO 区域内!!!"
        print(f"{dname}_S{stream}: EN={en} DIR={dirname} NDTR={ndtr:5d} "
              f"PAR={par:#010x} M0AR={m0ar:#010x} M1AR={m1ar:#010x} "
              f"MUX={reqname}{hit}")

s.sendall(b"shutdown\n"); time.sleep(0.3)
try: s.close()
except: pass
try: ocd.wait(timeout=5)
except: ocd.terminate()
