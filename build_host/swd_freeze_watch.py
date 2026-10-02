#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
冻结看护 + 自动取证 (2026-09-30 round-2 型故障专用).

背景: 10 轮测试中 round 2 出现 ginsaux 线程 + ADIS 链路冻结 (baro_cnt
38384 冻结 -> 46s 后 imu_cnt 冻结, gins 主线程存活), 复位即愈, 间歇性
(~1/15min)。本脚本不复位, 只读 tick/imu/baro/gnss 四计数器, 检测到
冻结后立即转储: ADIS DR 统计 / 全线程链表 / g_run 关键字段, 供根因
定位 (取证后退出, 保持现场直到人工复位)。

用法: python swd_freeze_watch.py [分钟, 缺省 20]
"""
import struct, subprocess, sys, time, os

CLI = r"D:\STM32CubeCLT\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
HERE = os.path.dirname(os.path.abspath(__file__))

# 符号 (00:44 固件, 每次重编须 nm 复核)
A = dict(gnan2run=0x2400ADB0, gmap=0x2400A120,
         adis_dev=0x24000EB8)        # adis_dev.stats 偏移待 DWARF 核实

def swd_read(addr, size):
    out = subprocess.run([CLI, "-c", "port=SWD", "mode=HotPlug", "-r32",
                          hex(addr), str(size)], capture_output=True, text=True,
                         timeout=60).stdout
    words = []
    for m in __import__("re").finditer(r"^0x[0-9A-Fa-f]{8}\s*:([0-9A-Fa-f ]+)$", out, __import__("re").M):
        words += [w for w in m.group(1).split()]
    if len(words) * 4 < size:
        raise RuntimeError(f"short read 0x{addr:x}")
    return b"".join(int(w.zfill(8), 16).to_bytes(4, "little") for w in words)[:size]

def u32(b, o): return int.from_bytes(b[o:o+4], "little")

def sample():
    blk = swd_read(A["gnan2run"], 0x258)
    gr = blk[0x48:0x48+0x70]
    return dict(tick=u32(blk, 0x254), imu=u32(gr, 0), gnss=u32(gr, 4),
                baro=u32(gr, 0x18), mag=u32(gr, 0x14))

def forensics():
    print("\n===== 冻结现场取证 =====", flush=True)
    # 1. g_run 全字段
    try:
        gr = swd_read(A["gnan2run"] + 0x48, 0x70)
        names = ["imu","gnss","cov_warn","drop","gnss_stale","mag","baro","mag_seq",
                 "baro_seq","mag_stale","baro_stale"]
        print("g_run:", {n: u32(gr, 4*i) for i, n in enumerate(names)},
              "skip_mid=", u32(gr, 0x6c), flush=True)
    except Exception as e:
        print("g_run dump fail:", e, flush=True)
    # 2. 全线程链表 (复用 rxdiag)
    try:
        subprocess.run([sys.executable, os.path.join(HERE, "swd_rxdiag.py")],
                       timeout=180)
    except Exception as e:
        print("rxdiag fail:", e, flush=True)

def main():
    mins = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    print(f"冻结看护 {mins} 分钟 (每 10s 轻量采样, 检测 30s 无 IMU 增量即取证)",
          flush=True)
    last = None
    frozen_t0 = None
    t_end = time.time() + mins * 60
    while time.time() < t_end:
        try:
            s = sample()
        except Exception as e:
            print(f"[warn] {e}", flush=True)
            time.sleep(5)
            continue
        if not (1000 < s["tick"] < 1000000000):
            print(f"[warn] tick={s['tick']} 毛刺/复位", flush=True)
            time.sleep(5)
            continue
        if last is not None and s["tick"] > last["tick"]:
            dimu = s["imu"] - last["imu"]
            dbaro = s["baro"] - last["baro"]
            dt = (s["tick"] - last["tick"]) / 1000.0
            tag = ""
            if dimu == 0:
                tag += " <== IMU 冻结"
            if dbaro == 0 and dimu == 0:
                tag += " <== AUX 冻结"
            print(f"t={time.strftime('%H:%M:%S')} tick={s['tick']} "
                  f"imu {dimu/max(dt,0.001):6.1f}/s baro {dbaro/max(dt,0.001):6.1f}/s "
                  f"gnss={s['gnss']} mag={s['mag']}{tag}", flush=True)
            if dimu == 0:
                if frozen_t0 is None:
                    frozen_t0 = time.time()
                if time.time() - frozen_t0 > 30:
                    forensics()
                    print("取证完成, 退出 (保持现场不复位)", flush=True)
                    return 2
            else:
                frozen_t0 = None
        last = s
        time.sleep(10)
    print("看护期结束, 未检测到冻结", flush=True)
    return 0

if __name__ == "__main__":
    sys.exit(main())
