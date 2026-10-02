# -*- coding: utf-8 -*-
"""观测 USART2 RX 链路三级指针滞后: DMA NDTR vs fifo.put_index vs fifo.get_index."""
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
DMA1_S2 = 0x40020040

# M0AR @ +0x0C (H7 stream: CR+0,NDTR+4,PAR+8,M0AR+0xC)
m0ar = read_words(DMA1_S2 + 0x0C, 4)[0]
print(f"DMA M0AR (fifo buffer) = {m0ar:#010x}")

FIFO = m0ar - 12   # struct rt_serial_rx_fifo { u32 buf; u16 put, get; i32 full }
for k in range(10):
    cr3  = read_words(USART2 + 0x08, 4)[0]
    isr  = read_words(USART2 + 0x1C, 4)[0]
    ndtr = read_words(DMA1_S2 + 0x04, 4)[0]
    fw = read_words(FIFO, 12)
    bufptr = fw[0]
    put, get = fw[1] & 0xFFFF, fw[1] >> 16
    full = struct.unpack("<i", struct.pack("<I", fw[2]))[0]
    hwpos = 512 - ndtr
    pend_isr = (hwpos - put) % 512
    pend_cons = (put - get) % 512
    print(f"[{k}] CR3={cr3:08X}(DMAR={(cr3>>6)&1},EIE={cr3&1},OVRDIS={(cr3>>12)&1}) "
          f"ISR={isr:08X}(ORE={(isr>>3)&1},IDLE={(isr>>4)&1},RXNE={(isr>>5)&1}) "
          f"NDTR={ndtr:3d} hw={hwpos:3d} put={put:3d} get={get:3d} full={full} "
          f"| ISR_lag={pend_isr} CONS_lag={pend_cons}")
    time.sleep(1.0)
