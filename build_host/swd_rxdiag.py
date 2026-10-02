#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RX 吞吐退化取证 (USART2/GNSS 链路, 免串口 SWD 直读).

用法:
    python swd_rxdiag.py            # 单次快照
    python swd_rxdiag.py -w 120     # 两次快照, 间隔 120s, 输出速率与增量

判据 (三级水位):
    1) DMA:      NDTR 是否推进 / Stream CR.EN / LISR.TEIF2 (传输错误)
    2) UART:     ISR.IDLE/ORE/RXFNE 活动位
    3) gnssrx:   rx_sem.value>0 表示 ISR 已唤醒但线程没消费 -> 线程饿死
    4) gnssdata: raw data_sem.value>0 表示 gnssrx 已入环但解析没消费 -> 解析饿死
    5) 环积压:   g_raw head-tail 字节数; serial 侧由 NDTR 间接看
地址表匹配 历史构建 (原 cmake-build-vscode, 见 rtthread.map + nm 提取记录).
"""
import argparse
import re
import subprocess
import sys
import time

CLI = r"D:\STM32CubeCLT\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"

# ---- 地址表 (历史实验构建 2026-09-29 (原 cmake-build-vscode, DCache-off)) ----
A = {
    "rt_tick":      0x2400AEDC,
    "g_run":        0x2400AD78,   # 0x70: +0x64 gnss_ts_zero +0x68 gnss_late_us_max
    "g_sol":        0x2400ADE8,
    "nav":          0x24001160,   # 0x168: counters @+0x50..
    "gns_ctx":      0x24009288,   # gnss_data.c ctx 0xE0
    "raw_ctx":      0x24008240,   # gnss_raw_data.c ctx 0x44
    "g_raw":        0x24000024,   # head/tail/size/buffer
    "g_rxdiag":     0x24009368,   # isr_evt,wake,wake_gap_max_ms,drain_us_max,moved,chunk_max
    "g_raw_hiwat":  0x24008284,
    "irq_nest":     0x2400B0C0,
    "obj_container":0x2400027C,   # [16] rt_object_information{type,list,size}=0x10
    "USART2":       0x40004400,
    "DMA1":         0x40020000,
}

# rt_thread 偏移
T_NAME, T_SP, T_STACK_ADDR, T_STACK_SZ, T_STAT = 0x00, 0x1C, 0x28, 0x2C, 0x3C
T_REMAIN, T_PRIO, T_TIMER = 0x44, 0x48, 0x50       # timer.timeout_tick @+0x30
T_STAT_NAMES = {0: "INIT", 1: "READY", 2: "SUSPND", 3: "RUNNING", 4: "CLOSE"}

def swd_read(addr, size):
    """STM32_Programmer_CLI -r32: size 参数是字节数 (8 对齐最优)."""
    out = subprocess.run(
        [CLI, "-c", "port=SWD", "mode=HotPlug", "-r32", hex(addr), str(size)],
        capture_output=True, text=True, timeout=60).stdout
    words = []
    for m in re.finditer(r"^0x[0-9A-Fa-f]{8}\s*:([0-9A-Fa-f ]+)$", out, re.M):
        words += [w for w in m.group(1).split()]
    if len(words) * 4 < size // 4 * 4:
        raise RuntimeError(f"read 0x{addr:x}+{size} short: {len(words)} words\n{out[-500:]}")
    b = b"".join(int(w.zfill(8), 16).to_bytes(4, "little") for w in words)
    return b[:size]

def u32(b, o):  return int.from_bytes(b[o:o+4], "little")
def u16(b, o):  return int.from_bytes(b[o:o+2], "little")
def u8(b, o):   return b[o]

def fmt_thread(addr, b):
    name = b[T_NAME:T_NAME+16].split(b"\x00")[0].decode("ascii", "replace")
    stat = u8(b, T_STAT) & 0x1F
    return (f"{name:<10.10} stat={T_STAT_NAMES.get(stat, hex(stat)):>7} "
            f"prio={u8(b, T_PRIO):>2} remain={u32(b, T_REMAIN):>6} "
            f"tmr_exp_in={u32(b, T_TIMER+0x30) - u32(b, T_TIMER+0x2C) if False else 0}")

def snapshot():
    s = {}
    s["tick"]    = u32(swd_read(A["rt_tick"], 4), 0)
    # USART2: CR1 CR2 CR3 BRR GTPR RTOR RQR ISR ICR RDR
    s["usart2"]  = swd_read(A["USART2"], 0x28)
    # DMA1 全局标志 + Stream2 寄存器组
    s["dma1"]    = swd_read(A["DMA1"], 0x10)
    s["dma_s2"]  = swd_read(A["DMA1"] + 0x40, 0x18)
    # gnss_data ctx
    g            = swd_read(A["gns_ctx"], 0xE0)
    s["rx_sem"]      = u16(g, 0x04 + 0x24)       # ctx.rx_sem.value
    s["data_sem"]    = u16(g, 0x48 + 0x24)       # ctx.data_sem.value
    s["rx_thread_p"] = u32(g, 0x34)              # ctx.rx_thread
    s["th_thread_p"] = u32(g, 0x78)              # ctx.thread
    s["st_pushed"]   = u32(g, 0x88)
    s["st_lost"]     = u32(g, 0x90)
    s["st_ts_zero"]  = u32(g, 0x94)
    s["last_T_event"]  = u32(g, 0x98) | (u32(g, 0x9C) << 32)     # u64
    s["last_T_arriv"]  = u32(g, 0xA8) | (u32(g, 0xAC) << 32)
    s["last_upd"]    = u32(g, 0x7C)
    # gnss_raw ctx + g_raw
    r            = swd_read(A["raw_ctx"], 0x44)
    s["raw_pushed"]  = u32(r, 0x04)
    s["raw_popped"]  = u32(r, 0x08)
    s["raw_lost"]    = u32(r, 0x0C)
    s["raw_sem"]     = u16(r, 0x10 + 0x24)
    raw          = swd_read(A["g_raw"], 0x10)
    head, tail, sz = u32(raw, 0), u32(raw, 4), u32(raw, 8)
    s["raw_head"], s["raw_tail"], s["raw_size"] = head, tail, sz
    s["raw_fill"]    = (head + sz - tail) % sz
    # nav 计数器
    n            = swd_read(A["nav"], 0x68)
    s["rmc"], s["gga"], s["zda"], s["csum_err"], s["field_err"] = (
        u32(n, 0x50), u32(n, 0x54), u32(n, 0x58), u32(n, 0x5C), u32(n, 0x60))
    s["upd_cnt"]     = u32(n, 0x44)
    # g_run
    gr           = swd_read(A["g_run"], 0x70)
    s["imu_cnt"], s["gnss_cnt"], s["gnss_stale"], s["gnss_rej"] = (
        u32(gr, 0), u32(gr, 4), u32(gr, 0x10), u32(gr, 0x50))
    import struct as _st
    s["step_us_avg"] = _st.unpack("<f", gr[0x48:0x4C])[0]
    s["step_us_max"] = _st.unpack("<f", gr[0x4C:0x50])[0]
    s["now_gpst"]    = _st.unpack("<d", gr[0x38:0x40])[0]
    s["gnss_time"]   = _st.unpack("<d", gr[0x40:0x48])[0]
    s["gnss_ts_zero"]     = u32(gr, 0x64)
    s["gnss_late_us_max"] = u32(gr, 0x68)
    # RX 链路诊断计数
    rd = swd_read(A["g_rxdiag"], 24)
    (s["isr_evt"], s["wake"], s["wake_gap_ms"], s["drain_us"], s["moved"],
     s["chunk_max"]) = _st.unpack("<6I", rd)
    s["raw_hiwat"] = u32(swd_read(A["g_raw_hiwat"], 4), 0)
    # 线程: gnssrx/gnssdata 直接解引用; 其余走对象容器线程链
    thr = {}
    for key, ptr in (("gnssrx", s["rx_thread_p"]), ("gnssdata", s["th_thread_p"])):
        if 0x24000000 <= ptr < 0x24080000:
            thr[key] = swd_read(ptr, 0xA8)
    # 容器线程链表: RT_Object_Info_Thread = 0 (object.c 枚举), 信息体 0x10, list@+4
    cont = swd_read(A["obj_container"] + 4, 8)   # Thread 是 index 0
    head = u32(cont, 0)
    node = u32(cont, 4)   # next
    walked = 0
    while node != head and walked < 40:
        tobj = node - 0x14
        if not (0x24000000 <= tobj < 0x24080000):
            break
        tb = swd_read(tobj, 0x50)
        nm = tb[T_NAME:T_NAME+16].split(b"\x00")[0].decode("ascii", "replace")
        thr[nm or hex(tobj)] = tb + swd_read(tobj + 0x50, 0x58)
        node = u32(swd_read(node, 8), 0)
        walked += 1
    s["threads"] = thr
    s["irq_nest"] = u32(swd_read(A["irq_nest"], 4), 0)
    return s

def print_snap(s, tag):
    tick = s["tick"]
    print(f"\n===== {tag}  rt_tick={tick} ({tick/1000:.1f}s) =====")
    cr1, cr3, isr = u32(s["usart2"], 0), u32(s["usart2"], 8), u32(s["usart2"], 0x1C)
    print(f"USART2 CR1={cr1:08x} CR3={cr3:08x} ISR={isr:08x}")
    print(f"  flags: {'RXFNE ' if isr&(1<<5) else ''}{'IDLE ' if isr&(1<<4) else ''}"
          f"{'ORE! ' if isr&(1<<3) else ''}{'NE! ' if isr&(1<<2) else ''}"
          f"{'FE! ' if isr&(1<<1) else ''}{'PE! ' if isr&1 else ''}"
          f"{'UE! ' if isr&(1<<0) else ''}"
          f"TEACK={1 if isr&(1<<21) else 0} (0=无标志)")
    lsr = u32(s["dma1"], 0)
    cr, ndtr = u32(s["dma_s2"], 0), u32(s["dma_s2"], 4)
    m0 = u32(s["dma_s2"], 0xC)
    print(f"DMA1 LISR={lsr:08x} Stream2 CR={cr:08x} (EN={cr&1}) NDTR={ndtr} M0AR=0x{m0:08x}")
    print(f"  LISR: TEIF2={lsr>>19&1} HTIF2={lsr>>20&1} TCIF2={lsr>>21&1}  <- 1=挂起未清")
    print(f"信号量水位: rx_sem={s['rx_sem']} data_sem={s['data_sem']} raw_sem={s['raw_sem']}")
    print(f"raw环: fill={s['raw_fill']}/{s['raw_size']}B pushed={s['raw_pushed']} "
          f"popped={s['raw_popped']} lost={s['raw_lost']}")
    print(f"nav: rmc={s['rmc']} gga={s['gga']} zda={s['zda']} csum_err={s['csum_err']} "
          f"field_err={s['field_err']} upd_cnt={s['upd_cnt']}")
    print(f"g_run: imu={s['imu_cnt']} gnss={s['gnss_cnt']} stale={s['gnss_stale']} "
          f"rej={s['gnss_rej']} step_us avg/max={s['step_us_avg']:.0f}/{s['step_us_max']:.0f}")
    print(f"       样本环: pushed={s['st_pushed']} lost={s['st_lost']} ts_zero={s['st_ts_zero']}"
          f"  引擎gnss年龄={s['now_gpst']-s['gnss_time']:.1f}s")
    print(f"       g_run尾: gnss_ts_zero={s['gnss_ts_zero']} gnss_late_us_max={s['gnss_late_us_max']}")
    print(f"rxdiag: isr_evt={s['isr_evt']} wake={s['wake']} wake_gap_max={s['wake_gap_ms']}ms "
          f"drain_max={s['drain_us']}us moved={s['moved']} chunk_max={s['chunk_max']} "
          f"raw_hiwat={s['raw_hiwat']}B")
    ev, ar = s["last_T_event"], s["last_T_arriv"]
    if ev:
        print(f"last样本: T_event={ev} T_arrival={ar} 解析延迟={(ar-ev)/1e6:.3f}s")
    print(f"irq_nest={s['irq_nest']}")
    print("线程 (对象容器遍历):")
    for nm, tb in s["threads"].items():
        stat = u8(tb, T_STAT) & 0x1F
        sa, ss = u32(tb, T_STACK_ADDR), u32(tb, T_STACK_SZ)
        sp = u32(tb, T_SP)
        used = (sa + ss - sp) if sa and sp >= sa else 0
        print(f"  {nm:<10.10} {T_STAT_NAMES.get(stat, hex(stat)):>7} prio={u8(tb, T_PRIO):>2} "
              f"remain={u32(tb, T_REMAIN):>6} 栈用≈{used}/{ss}B")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-w", "--wait", type=int, default=0, help="两次快照间隔秒数")
    args = ap.parse_args()
    s1 = snapshot()
    print_snap(s1, "SNAP-1")
    if args.wait > 0:
        print(f"\n... 等待 {args.wait}s ...")
        time.sleep(args.wait)
        s2 = snapshot()
        print_snap(s2, "SNAP-2")
        dt = max((s2["tick"] - s1["tick"]) / 1000.0, 1e-3)
        print(f"\n===== 增量/速率 (dt={dt:.1f}s) =====")
        for k in ("rmc", "gga", "zda", "csum_err", "field_err", "upd_cnt",
                  "raw_pushed", "raw_popped", "raw_lost", "st_pushed",
                  "imu_cnt", "gnss_cnt", "gnss_stale", "gnss_rej"):
            d = s2[k] - s1[k]
            print(f"  {k:<12} +{d:<8} {d/dt:8.1f}/s")
        print(f"  T_event推进 = {(s2['last_T_event']-s1['last_T_event'])/1e6:.3f}s / wall {dt:.1f}s")

if __name__ == "__main__":
    main()
