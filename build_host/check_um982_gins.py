#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查 UM982 数据是否成功送入 KF-GINS (COM9 @460800)

1. 被动抓取 gins_fused_data 文本行, 统计 imu_data/gnss_data 计数增长率
   (gnss_data ≈10/s 即 UM982 观测正持续喂入 KF-GINS 引擎)
2. FinSH `gins`: 引擎状态 (ready/计数/stale)
3. FinSH `gnss`: UM982 USART2 接收统计 (字节/语句/配对)
"""
import re
import sys
import time

import serial

PORT = "COM9"
BAUD = 460800

FUSED_RE = re.compile(
    rb"ready:(\d) time:(\d+)\.(\d{3}) .*?"
    rb"imu_data:(\d+) gnss_data:(\d+) mag_calib_data:(\d+) baro_calib_data:(\d+) fused_data",
    re.S,
)


def clean(b):
    return bytes(c if 0x20 <= c < 0x7F or c in (0x0D, 0x0A) else 0x2E for c in b)


def read_for(ser, seconds, sink=None):
    end = time.time() + seconds
    out = b""
    while time.time() < end:
        n = ser.read(8192)
        if n:
            out += n
            if sink is not None:
                sink.extend(n)
    return out


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    print(f"[1] opened {PORT} @ {BAUD}, passive capture 12 s ...")
    raw = bytearray()
    data = read_for(ser, 12, sink=raw)
    open(r"D:\STM32Project\Flight\build_host\capture_um982_gins.bin", "wb").write(raw)

    rows = [
        {
            "ready": int(m.group(1)),
            "t": int(m.group(2)) + int(m.group(3)) / 1000.0,
            "imu": int(m.group(4)),
            "gnss": int(m.group(5)),
            "mag": int(m.group(6)),
            "baro": int(m.group(7)),
        }
        for m in FUSED_RE.finditer(data)
    ]
    # 去重 (同一条行可能被二进制干扰后重复匹配)
    uniq = []
    for r in rows:
        if not uniq or (r["t"], r["gnss"], r["imu"]) != (uniq[-1]["t"], uniq[-1]["gnss"], uniq[-1]["imu"]):
            uniq.append(r)
    rows = uniq

    if not rows:
        print("  NO fused lines found. Head of stream (cleaned):")
        print(clean(data[:400]).decode("ascii", "replace"))
    else:
        span = rows[-1]["t"] - rows[0]["t"]
        dur = span if span > 0 else 1e-6
        imu_r = (rows[-1]["imu"] - rows[0]["imu"]) / dur
        gnss_r = (rows[-1]["gnss"] - rows[0]["gnss"]) / dur
        mag_r = (rows[-1]["mag"] - rows[0]["mag"]) / dur
        print(f"  fused lines: {len(rows)}, engine time span {span:.1f} s")
        print(f"  ready      : {rows[-1]['ready']} (1=RUNNING, 0=ALIGN/WAIT)")
        print(f"  imu_data   : {rows[-1]['imu']} total, rate {imu_r:.1f} /s (expect ~1000)")
        print(f"  gnss_data  : {rows[-1]['gnss']} total, rate {gnss_r:.1f} /s (expect ~10  <- UM982 -> KF-GINS)")
        print(f"  mag_data   : {rows[-1]['mag']} total, rate {mag_r:.1f} /s")
        print(f"  first row  : t={rows[0]['t']:.3f} ready={rows[0]['ready']} imu={rows[0]['imu']} gnss={rows[0]['gnss']}")
        print(f"  last  row  : t={rows[-1]['t']:.3f} ready={rows[-1]['ready']} imu={rows[-1]['imu']} gnss={rows[-1]['gnss']}")
        verdict = "PASS: GNSS observations ARE being fed into KF-GINS" if gnss_r > 5 else \
                  "WARN: engine running but GNSS feed rate abnormal"
        print(f"  >> {verdict}")

    for cmd, sec in ((b"gins\r\n", 2.0), (b"gnss\r\n", 2.0)):
        print(f"\n[{cmd.decode().strip()}] FinSH response:")
        ser.reset_input_buffer()
        ser.write(cmd)
        resp = read_for(ser, sec)
        for line in clean(resp).decode("ascii", "replace").splitlines():
            s = line.strip()
            if s and not s.startswith("."):
                print("  " + s)

    ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
