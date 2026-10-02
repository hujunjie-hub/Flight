# -*- coding: utf-8 -*-
"""读 nav.last (最近喂入解析器的句子) + gnss_data 解析统计, 连续采 8 次."""
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
    return struct.pack("<%dI" % n, *words)

def u32(b, o): return struct.unpack_from("<I", b, o)[0]

# nav @ 0x2400a9c0: last[] @+100, 160 bytes (UM982_NMEA_LINE_MAX)
# gnss_data ctx @ 0x24002910: 定位 st 字段需要算偏移 — 结构里有指针/线程句柄,
# 先只看 nav 的句子内容与计数变化.
for k in range(8):
    nav = read_words(0x2400a9c0, 0x108)
    last = nav[100:100+160]
    last = last.split(b'\0', 1)[0].decode('ascii', errors='replace')
    counts = (u32(nav,80), u32(nav,84), u32(nav,88), u32(nav,92), u32(nav,96))
    print(f"[{k}] rmc={counts[0]} gga={counts[1]} zda={counts[2]} "
          f"csum_err={counts[3]} field_err={counts[4]}")
    print(f"    last: {last[:150]}")
    time.sleep(1.2)
