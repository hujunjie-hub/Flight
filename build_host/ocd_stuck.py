#!/usr/bin/env python3
"""One-shot: halt target, dump registers + PC region symbols, and walk the
RT-Thread thread list from _object_container to find where init is stuck."""
import struct
import time

from ocd_diag_rst import Ocd

OBJ_CONTAINER = 0x24000218     # _object_container (array of rt_information)
# rt_information: object_list(rt_list_t:2 ptrs=8B) size(2B) ... Thread class idx = 1
TICK = 0x2400aa94


def rd_words(o, addr, n):
    return o.mdw(addr, n)


def sym_at(addr, syms):
    best = None
    for a, n in syms:
        if a <= addr and (best is None or a > best[0]):
            best = (a, n)
    return best


o = Ocd()
try:
    # wait past the reset so init is mid-flight, then halt
    time.sleep(6)
    print(o.cmd("halt", 0.3))
    regs = o.cmd("reg", 0.5)
    for line in regs.splitlines():
        ls = line.strip()
        if ls.startswith(("pc ", "sp ", "lr ", "xPSR")):
            print(ls)

    # walk thread objects
    info = rd_words(o, OBJ_CONTAINER + 1 * 16, 4)   # Thread class
    head = OBJ_CONTAINER + 1 * 16
    node = info[0]
    print(f"\nthreads (list head 0x{head:08x}, first node 0x{node:08x}):")
    n = 0
    while node != head and n < 24:
        # rt_object: type(1) id(1) flag... name ptr @ +4? layout: type,id (2B pad) name@+4 list@+8
        obj_words = rd_words(o, node - 8, 6)
        name_ptr = obj_words[1]
        name = o.cmd("mdb 0x%08x 12" % name_ptr, 0.25)
        nm = name.split(":", 1)[-1].strip() if ":" in name else "?"
        try:
            nm = bytes.fromhex(nm.replace(" ", "")).decode(errors="replace").split("\x00")[0]
        except ValueError:
            pass
        # rt_thread: sp@+0x10(from list start +8: list is at +8, sp at +0x10 rel object)
        sp = rd_words(o, node - 8 + 0x10, 1)[0]
        # thread state @ +0x3E-ish; try reading stat char at object+0x3C area
        misc = rd_words(o, node - 8 + 0x38, 4)
        stat = (misc[1] >> 24) & 0xFF
        pcs = rd_words(o, sp + 0x14, 6) if 0x20000000 <= sp < 0x24040000 else []
        ret_pc = pcs[5] if len(pcs) > 5 else 0
        print(f"  {nm:12s} stat={stat:3d} sp=0x{sp:08x} ret~0x{ret_pc:08x}")
        node = rd_words(o, node, 1)[0]     # next
        n += 1

    tick = rd_words(o, TICK)[0]
    print("\ntick at halt:", tick)
    print(o.cmd("soft reset_halt", 0.1))
finally:
    o.close()
