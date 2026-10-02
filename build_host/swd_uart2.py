# -*- coding: utf-8 -*-
"""直读 USART2 + DMA1_Stream2 + DMAMUX1 寄存器, 判定 RX DMA 实际运行状态."""
import subprocess, re, struct, sys, io, time

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
CLI = r"D:\STM32CubeCLT\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"

def read_words(addr, nbytes):
    n = (nbytes + 3) // 4
    p = subprocess.run([CLI, "-c", "port=SWD mode=HotPlug",
                        "-r32", hex(addr), str(nbytes)],
                       capture_output=True, text=True, timeout=30)
    words = []
    for line in p.stdout.splitlines():
        m = re.match(r"^0x[0-9A-Fa-f]{8}\s*:\s*(.*)$", line)
        if m:
            words += [int(w, 16) for w in m.group(1).split()]
    if len(words) < n:
        raise RuntimeError(f"read {addr:#x}: got {len(words)}/{n}")
    return words

USART2 = 0x40004400
DMA1_S2 = 0x40020040        # DMA1 base + 0x10 + 2*0x18
DMAMUX1_C2 = 0x40020808     # DMAMUX1 channel 2 (DMA1 Stream2)

for k in range(4):
    cr1   = read_words(USART2 + 0x00, 4)[0]
    isr   = read_words(USART2 + 0x18, 4)[0]
    cr3   = read_words(USART2 + 0x20, 4)[0]
    brr   = read_words(USART2 + 0x0C, 4)[0]
    dcr   = read_words(DMA1_S2 + 0x00, 4)[0]
    ndtr  = read_words(DMA1_S2 + 0x04, 4)[0]
    par   = read_words(DMA1_S2 + 0x10, 4)[0]
    m0ar  = read_words(DMA1_S2 + 0x14, 4)[0]
    mux   = read_words(DMAMUX1_C2, 4)[0]

    print(f"[{k}] USART2: CR1={cr1:08X} CR3={cr3:08X} (DMAR={(cr3>>6)&1}) "
          f"BRR={brr:04X} ISR={isr:08X} (ORE={(isr>>3)&1} IDLE={(isr>>4)&1})")
    print(f"    DMA1_S2: CR={dcr:08X} (EN={dcr&1} CIRC={(dcr>>5)&1} "
          f"TCIE={(dcr>>1)&1} HTIE={(dcr>>3)&1} TEIE={(dcr>>7)&1}) "
          f"NDTR={ndtr} PAR={par:08X} M0AR={m0ar:08X}")
    print(f"    DMAMUX1_C2: {mux:08X} (REQ_ID={mux&0x7F} DMAREQ_ID={mux&0x3F})")
    time.sleep(0.8)
