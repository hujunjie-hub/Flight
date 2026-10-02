#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通过 openocd telnet (4444) 直接读目标板内存, 验证 UM982 -> KF-GINS 数据链

g_sol  @0x2400A258 (struct gins_solution, 160B)
g_run  @0x2400A208
DWT->CYCCNT @0xE0001004 (内核运行心跳)
"""
import re
import socket
import struct
import sys
import time

OCD_TELNET = ("127.0.0.1", 4444)
G_SOL = 0x2400A258
DWT_CYCCNT = 0xE0001004


class Ocd:
    def __init__(self):
        self.s = socket.create_connection(OCD_TELNET, timeout=3)
        self.buf = b""
        time.sleep(0.3)
        self._drain()  # 吃掉 banner + telnet 协商字节

    def _drain(self):
        self.s.settimeout(0.4)
        while True:
            try:
                d = self.s.recv(65536)
            except socket.timeout:
                break
            if not d:
                break
            self.buf += d

    def cmd(self, c, wait=0.6):
        self.buf = b""
        self.s.settimeout(None)
        self.s.sendall(c.encode() + b"\n")
        time.sleep(wait)
        self._drain()
        out = self.buf
        # 去 telnet IAC 序列
        out = re.sub(rb"\xff[\xf0-\xfe].", b"", out)
        out = re.sub(rb"\xff.", b"", out)
        txt = out.decode("ascii", "replace")
        return txt[: txt.rfind(">")] if ">" in txt else txt


def mdw_words(ocd, addr, n, verbose=False):
    resp = ocd.cmd(f"mdw 0x{addr:X} {n}")
    if verbose:
        print("RAW:", repr(resp[:400]))
    # openocd mdw 多字输出: "0x2400a258: w0 w1 w2 w3\r\n" 每行 4 值,
    # 只有行首地址带冒号。先剥掉命令回显行与行首 "0x........:" 地址,
    # 再收所有 8hex 值。
    body = re.sub(r"^.*mdw[^\n]*\n", "", resp, count=1)
    body = re.sub(r"0x[0-9A-Fa-f]{8}:\s*", "", body)
    words = [int(w, 16) for w in re.findall(r"\b([0-9A-Fa-f]{8})\b", body)]
    return words[:n] if len(words) >= n else None


def parse_sol(words):
    b = b"".join(struct.pack("<I", w) for w in words)
    u32 = lambda off: struct.unpack_from("<I", b, off)[0]
    f64 = lambda off: struct.unpack_from("<d", b, off)[0]
    return {
        "ready": u32(0x00),
        "time": f64(0x08),
        "lat": f64(0x10),
        "lon": f64(0x18),
        "alt": f64(0x20),
        "vn": f64(0x28), "ve": f64(0x30), "vd": f64(0x38),
        "roll": f64(0x40), "pitch": f64(0x48), "yaw": f64(0x50),
        "imu_cnt": u32(0x58), "gnss_cnt": u32(0x5C),
        "gnss_stale": u32(0x60), "gnss_age_ms": u32(0x64),
        "cov_warn": u32(0x68), "dr_drop": u32(0x6C),
        "mag_cnt": u32(0x78), "mag_rej": u32(0x7C), "mag_stale": u32(0x80),
        "baro_cnt": u32(0x84), "baro_rej": u32(0x88), "baro_stale": u32(0x8C),
    }


def main():
    ocd = Ocd()

    # 1. 内核运行检查: DWT CYCCNT 必须增长 (多次采样)
    print(ocd.cmd("targets").strip())
    cys = []
    for _ in range(4):
        c = mdw_words(ocd, DWT_CYCCNT, 1, verbose=(_ == 0))
        if c:
            cys.append(c[0])
        time.sleep(0.3)
    if not cys:
        print("FAIL: 无法读取 DWT->CYCCNT, openocd 未连接目标或 target 已断开")
        return 1
    running = len(set(cys)) > 1
    print(f"[1] core running: CYCCNT samples {[hex(c) for c in cys]} : "
          f"{'RUNNING' if running else 'HALTED/FROZEN'}")
    if not running:
        print("FAIL: 内核未运行 (被调试器暂停), 计数器不会增长")
        return 1

    # 2. 两次读 g_sol, 间隔 ~6 s, 对比计数差
    w1 = mdw_words(ocd, G_SOL, 40)
    s1 = parse_sol(w1)
    print(f"[2] g_sol snapshot #1: {s1}")
    print("    waiting 6 s ...")
    time.sleep(6.0)
    w2 = mdw_words(ocd, G_SOL, 40)
    s2 = parse_sol(w2)
    dt = s2["time"] - s1["time"]
    d_imu = (s2["imu_cnt"] - s1["imu_cnt"]) / dt if dt > 0 else 0
    d_gnss = (s2["gnss_cnt"] - s1["gnss_cnt"]) / dt if dt > 0 else 0
    d_mag = (s2["mag_cnt"] - s1["mag_cnt"]) / dt if dt > 0 else 0

    print(f"\n[3] 引擎时间: {s1['time']:.3f} -> {s2['time']:.3f} (dt={dt:.3f} s)")
    print(f"    ready        : {s2['ready']} (1=RUNNING, 0=ALIGN/WAIT)")
    print(f"    imu_cnt      : {s1['imu_cnt']} -> {s2['imu_cnt']}  ({d_imu:.0f} /s, expect ~1000)")
    print(f"    gnss_cnt     : {s1['gnss_cnt']} -> {s2['gnss_cnt']}  ({d_gnss:.1f} /s, expect ~10)")
    print(f"    gnss_stale   : {s2['gnss_stale']}   gnss_age_ms: {s2['gnss_age_ms']}")
    print(f"    mag_cnt      : {s2['mag_cnt']} ({d_mag:.0f} /s)  cov_warn: {s2['cov_warn']}  dr_drop: {s2['dr_drop']}")
    print(f"    position     : lat={s2['lat']:.7f} lon={s2['lon']:.7f} alt={s2['alt']:.2f}")
    print(f"    attitude(deg): r={s2['roll']:.2f} p={s2['pitch']:.2f} y={s2['yaw']:.2f}")

    print()
    if s2["gnss_cnt"] > 0 and d_gnss > 5:
        print(">> PASS: UM982 观测正以 ~10Hz 持续喂入 KF-GINS 引擎 (gnss_cnt 增长中)")
    elif s2["gnss_cnt"] > 0:
        print(f">> WARN: 引擎有历史 GNSS 观测但当前速率 {d_gnss:.1f}/s 异常 (检查 UM982/PPS)")
    elif d_imu > 500:
        print(">> FAIL: IMU 在喂但 gnss_cnt=0 —— 引擎未收到任何 GNSS 观测")
        print("   (可能: UM982 无定位 fix / utc_sec=0 / PPS 未同步, 看 `gnss` `um982` `pps`)")
    else:
        print(">> FAIL: IMU 与 GNSS 均无数据喂入, 引擎未初始化")
    return 0


if __name__ == "__main__":
    sys.exit(main())
