#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DR->DMA 新固件运行时取证: 快照刷新/真实样本率/PPS/UM982 状态

地址来源: arm-none-eabi-nm Flight.elf
"""
import struct
import sys
import time

sys.path.insert(0, r"D:\STM32Project\Flight\build_host")
from check_gins_ocd import Ocd, mdw_words

ADIS_DEV = 0x24000ED8
DR_DMA = 0x24000F48
IMU_CTX = 0x24001590
NAV = 0x24001478

PPS_LOCKED = 0x24001214
PPS_MISS = 0x24001218
PPS_LOCK_CNT = 0x24001210
PPS_PPS_COUNT = 0x24001224
PPS_QUALITY = 0x24001228
PPS_UTC_VALID = 0x24001280


def words2bytes(words):
    return b"".join(struct.pack("<I", w) for w in words)


def main():
    ocd = Ocd()

    # ---- adis_dev: dec_rate/dr_mode/dma_busy + sample 关键字段 ----
    dev = words2bytes(mdw_words(ocd, ADIS_DEV, 0x70 // 4))
    prod_id, dec_rate = struct.unpack_from("<HH", dev, 0x08)
    dr_mode, dma_busy = struct.unpack_from("<II", dev, 0x10)
    ping = dev[0x18]
    isr_us = struct.unpack_from("<Q", dev, 0x20)[0]
    (snap_local_us, snap_utc_us, sec, us_frac, time_ms, local_ms,
     quality) = struct.unpack_from("<QQIIIIB", dev, 0x28)
    synced = dev[0x4C]
    gyro = struct.unpack_from("<3h", dev, 0x50)
    acce = struct.unpack_from("<3h", dev, 0x56)
    temp, data_cntr, diag = struct.unpack_from("<3H", dev, 0x5C)
    sample_valid = struct.unpack_from("<I", dev, 0x68)[0]

    print("== adis_dev ==")
    print(f"  prod_id={prod_id} dec_rate={dec_rate} (0 -> DEC_RATE 写失败=2000Hz 默认)")
    print(f"  dr_mode={dr_mode} dma_busy={dma_busy} ping={ping} sample_valid={sample_valid}")
    print(f"  snap: local_us={snap_local_us} utc_us={snap_utc_us} sec={sec} synced={synced} quality={quality}")
    print(f"  snap: gyro={gyro} acce={acce} temp={temp} cntr={data_cntr} diag={diag:#06x}")

    # ---- 两次读快照 DATA_CNTR 差分 = 真实样本率 ----
    c1 = data_cntr
    t1 = time.time()
    l1 = struct.unpack_from("<Q", words2bytes(mdw_words(ocd, ADIS_DEV + 0x28, 2)), 0)[0]
    time.sleep(10.0)
    b = words2bytes(mdw_words(ocd, ADIS_DEV + 0x28, 0x40 // 4))
    c2 = struct.unpack_from("<H", b, 54)[0]
    l2 = struct.unpack_from("<Q", b, 0)[0]
    g2 = struct.unpack_from("<3h", b, 40)
    a2 = struct.unpack_from("<3h", b, 46)
    dt = time.time() - t1
    dcntr = (c2 - c1) & 0xFFFF
    print(f"\n== 10s 窗口 ==")
    print(f"  DATA_CNTR: {c1} -> {c2} (+{dcntr}, {dcntr/dt:.1f} /s 真实芯片样本率)")
    print(f"  local_us : {l1} -> {l2} (快照{'刷新中' if l2 != l1 else '未刷新!'})")
    print(f"  gyro={g2} acce={a2}")

    # ---- dr_dma 统计 ----
    dr = words2bytes(mdw_words(ocd, DR_DMA, 10))
    isr_cnt, dma_done, missed, busy_skip, crc_err, start_err, recover, stale = \
        struct.unpack_from("<8I", dr, 8)
    print(f"\n== dr_dma ==")
    print(f"  isr={isr_cnt} done={dma_done} missed={missed} busy_skip={busy_skip} "
          f"crc={crc_err} start_err={start_err} recover={recover} stale_warn={stale}")

    # ---- imu_data ctx ----
    ic = words2bytes(mdw_words(ocd, IMU_CTX, 0x90 // 4))
    # 搜索模式: 快速增长的大数(pushed) + 三个小数 (lost/errors)
    p1 = struct.unpack_from("<I", ic, 0)[0]
    print("\n== imu_data ctx raw ==")
    for off in range(0, 0x90, 16):
        row = struct.unpack_from("<4I", ic, off)
        print(f"  +{off:#04x}: " + " ".join(f"{v:08x}" for v in row))
    time.sleep(2.0)
    ic2 = words2bytes(mdw_words(ocd, IMU_CTX, 0x90 // 4))
    diffs = [(off, (struct.unpack_from("<I", ic2, off)[0] -
                    struct.unpack_from("<I", ic, off)[0]) & 0xFFFFFFFF)
             for off in range(0, 0x90, 4)]
    print("  2s 增量字段: " + ", ".join(f"+{off:#04x}:+{d}" for off, d in diffs if d != 0))

    # ---- PPS / GNSS ----
    print("\n== PPS ==")
    w = mdw_words(ocd, PPS_LOCK_CNT, 1)
    lock_cnt = w[0] if w else 0
    w = mdw_words(ocd, PPS_PPS_COUNT, 1)
    pps_cnt = w[0] if w else 0
    w = mdw_words(ocd, PPS_MISS, 1)
    miss = w[0] if w else 0
    w = mdw_words(ocd, PPS_QUALITY, 1)
    qual = w[0] & 0xFF if w else 0
    w = mdw_words(ocd, PPS_LOCKED, 1)
    locked = w[0] if w else 0
    w = mdw_words(ocd, PPS_UTC_VALID, 1)
    utc_valid = w[0] if w else 0
    print(f"  locked={locked} quality={qual} utc_valid={utc_valid}")
    print(f"  pps_count={pps_cnt} miss_count={miss} lock_cnt={lock_cnt}")

    print("\n== um982 nav (struct gnss_data) ==")
    nv = words2bytes(mdw_words(ocd, NAV, 0x118 // 4))
    for off in range(0, 0x80, 16):
        row = struct.unpack_from("<4I", nv, off)
        print(f"  +{off:#04x}: " + " ".join(f"{v:08x}" for v in row))
    return 0


if __name__ == "__main__":
    sys.exit(main())
