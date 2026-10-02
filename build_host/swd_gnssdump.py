# -*- coding: utf-8 -*-
"""转储 USART2 原始 NMEA 环形缓冲区, 分析丢字节形态."""
import subprocess, re, struct, sys, io

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
    return struct.pack("<%dI" % n, *words)

def u32(b, o): return struct.unpack_from("<I", b, o)[0]

# g_raw @ 0x240001e4: head tail size bufptr
g = read_words(0x240001e4, 16)
head, tail, size, bufptr = u32(g,0), u32(g,4), u32(g,8), u32(g,12)
print(f"ring: head={head} tail={tail} size={size} buf={bufptr:#x}")
print(f"pending bytes: {(head + size - tail) % size}")

# ctx @ 0x24004394: inited pushed popped lost
c = read_words(0x24004394, 16)
print(f"stats: inited={struct.unpack_from('<i',c,0)[0]} pushed={u32(c,4)} "
      f"popped={u32(c,8)} lost={u32(c,12)}")

# 全环转储, 从 tail 到 head 展开
pool = read_words(0x24003394, 4096)
n = (head + size - tail) % size
data = bytes((pool[(tail + i) % size]) for i in range(n))
print(f"\n--- {n} bytes pending in ring (tail->head) ---")
txt = data.decode('ascii', errors='replace')
print(txt[-3000:])
