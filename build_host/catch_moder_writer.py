#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓 GPIOC MODER 的写入者: halt -> 设写监视点 -> resume -> 触发 -> halt -> PC"""
import re
import sys
import time

sys.path.insert(0, r"D:\STM32Project\Flight\build_host")
from check_gins_ocd import Ocd
from fault_probe import sym_of


def get_reg(ocd, name):
    raw = ocd.cmd(f"reg {name}", wait=1.0)
    m = re.search(r"0x([0-9a-fA-F]+)", raw.replace("=", " "))
    return int(m.group(1), 16) if m else 0


def main():
    ocd = Ocd()

    print("[1] halt + set write watchpoint on GPIOC MODER (0x58024400)")
    print("    " + ocd.cmd("halt", wait=1.5).strip().replace("\r\n", " | "))
    print("    " + ocd.cmd("wp 0x58024400 4 w", wait=1.0).strip().replace("\r\n", " | "))
    print("[2] resume, waiting for MODER write (max 30 s) ...")
    print("    " + ocd.cmd("resume", wait=1.0).strip().replace("\r\n", " | "))

    t0 = time.time()
    caught = False
    while time.time() - t0 < 30:
        time.sleep(1.0)
        st = ocd.cmd("targets", wait=0.8)
        seg = st.split("stm32h7x.cpu0")[-1]
        if "halted" in seg:
            caught = True
            print(f"[3] caught at t+{time.time()-t0:.0f}s (watchpoint hit)")
            break

    if caught:
        pc = get_reg(ocd, "pc")
        lr = get_reg(ocd, "lr")
        print(f"    PC = {pc:#010x} -> {sym_of(pc)}")
        print(f"    LR = {lr:#010x} -> {sym_of(lr)}")
        # 返回地址: 栈上的更多线索
        msp = get_reg(ocd, "msp")
        psp = get_reg(ocd, "psp")
        sp = psp if psp else msp
        ws = __import__("check_gins_ocd").mdw_words(ocd, sp, 24)
        if ws:
            print("    stack (callers):")
            for i, w in enumerate(ws):
                if 0x08000000 <= w < 0x08200000:
                    print(f"      sp+{i*4:#04x}: {w:#010x} {sym_of(w)}")
    else:
        print("[3] no MODER write in 30 s (nobody writes it -> 静态状态, 非周期覆盖)")

    print("[4] cleanup")
    print("    " + ocd.cmd("rwp", wait=0.8).strip().splitlines()[0])
    print("    " + ocd.cmd("resume", wait=1.0).strip().replace("\r\n", " | "))
    return 0


if __name__ == "__main__":
    sys.exit(main())
