# -*- coding: utf-8 -*-
"""1) 扫描 BDMA (0x58025800) + MDMA (0x52000000) 通道;
2) 0xAA 填充实验: 在零区域写图案, 运行后回读判定写入者."""
import subprocess, socket, time, sys, io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
OPENOCD = r"D:\STM32CubeCLT\OpenOCD\bin\openocd.exe"
M0AR = 0x2401181C
FILL = 0x240119E0   # 零运行起始 (对齐 4)

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

def parse_words(txt):
    vals = []
    for line in txt.split("\n"):
        line = line.strip()
        if line.startswith("0x") and ":" in line:
            vals += [int(t, 16) for t in line.split(":", 1)[1].split()]
    return vals

try:
    # ---- 1. BDMA 扫描 (H7: ISR@0, IFCR@4, 通道寄存器 0x08 起步进 0x14) ----
    print("=== BDMA (0x58025800) 通道 ===")
    for ch in range(8):
        base = 0x58025800 + 0x08 + ch * 0x14
        w = parse_words(cmds([f"mdw {base:#x} 5"]))
        if len(w) >= 5:
            cr, ndtr, par, m0ar = w[0], w[1], w[2], w[3]
            en = cr & 1
            hit = " <<<=== 指向 FIFO 区域!" if 0x2401181C <= m0ar < 0x24011A1C else ""
            if en or m0ar:
                print(f"BDMA_CH{ch}: EN={en} CR={cr:#010x} NDTR={ndtr} "
                      f"PAR={par:#010x} M0AR={m0ar:#010x}{hit}")
    # DMAMUX2 (BDMA): 0x58025600
    w = parse_words(cmds([f"mdw 0x58025800 8"]))
    print("BDMA ISR/IFCR 区:", [f"{x:#010x}" for x in w[:8]])

    # ---- 2. MDMA 粗查: CISR@0xC, 通道 0x40 间距 ----
    print("\n=== MDMA (0x52000000) 通道 (仅活跃的) ===")
    for ch in range(16):
        base = 0x52000000 + 0x40 + ch * 0x40
        w = parse_words(cmds([f"mdw {base:#x} 8"]))
        if len(w) >= 8:
            cr = w[0]
            if cr & 1:
                # MDMA CxTBR/CxMAR 等在偏移 0x10/0x14... 打印关键
                w2 = parse_words(cmds([f"mdw {base+0x10:#x} 4"]))
                mar = w2[0] if w2 else 0
                dar = w2[1] if len(w2) > 1 else 0
                hit = " <<<=== 涉及 FIFO 区域!" if (0x2401181C <= dar < 0x24011A1C or 0x2401181C <= mar < 0x24011A1C) else ""
                print(f"MDMA_CH{ch}: EN=1 CR={cr:#010x} MAR={mar:#010x} DAR={dar:#010x}{hit}")

    # ---- 3. 0xAA 填充实验 ----
    print(f"\n=== 0xAA 填充实验 @ {FILL:#x} (40 字节) ===")
    txt = cmds(["halt",
                f"mww {FILL:#x} 0xAAAAAAAA 10",
                f"mdw {FILL:#x} 10",
                "resume"])
    before = parse_words(txt)
    print("填充后立即读:", [f"{x:08x}" for x in before[:10]])
    time.sleep(1.2)   # ~2-3 圈数据流
    txt = cmds(["halt", f"mdw {FILL:#x} 10", f"mdw 0x40020044", "resume"])
    after = parse_words(txt)
    ndtr = after[10] if len(after) > 10 else None
    print("运行 1.2s 后读:", [f"{x:08x}" for x in after[:10]],
          f"NDTR={ndtr & 0xFFFF if ndtr is not None else '?'}")
    a = after[:10]
    if all(x == 0xAAAAAAAA for x in a):
        print(">>> 保持 0xAA: DMA 从不写这块区域 (缓冲区空洞!)")
    elif all(x == 0 for x in a):
        print(">>> 回到全零: 有周期性零写入者 (非 CPU, 应为某 DMA)")
    else:
        print(">>> 变成数据: 该区域正常被流覆盖 (零是一次性/陈旧写入)")
finally:
    try:
        s.sendall(b"shutdown\n"); time.sleep(0.3)
    except: pass
    try: s.close()
    except: pass
    try: ocd.wait(timeout=5)
    except: ocd.terminate()
