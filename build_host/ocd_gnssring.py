#!/usr/bin/env python3
"""One-shot: halt, dump gnss_data ctx.ring (record_ring + rt_ringbuffer
state) to see why rt_ringbuffer_get loops forever."""
import time

from ocd_diag_rst import Ocd

GCTX = 0x24008ee8
# gnss ctx layout (DWARF): rx_dev@0 rx_sem@4 rx_thread@52 ring@56
RING = GCTX + 56
# struct record_ring: rb{ buffer(4) read_index(4) read_mirror(1)+pad
#   write_index(4) write_mirror(1)+pad buffer_size(4) } data_sem(rt_semaphore 24?)
# rt_ringbuffer: buffer@0 read_index@4 (mirror in top bit? 5.3.0 uses
#   read_index u32 + read_mirror u16; check both) — dump raw 16 words.

o = Ocd()
try:
    time.sleep(6)
    print(o.cmd("halt", 0.5))
    w = o.mdw(RING, 16)
    names = ["buffer_ptr", "read_index", "word2(mirror/wi)", "word3",
             "write_index", "word5(wmirror)", "buffer_size", "word7",
             "sem*", "sem", "sem", "sem", "sem", "rec_size?", "lost?", "pad"]
    for i, x in enumerate(w):
        print(f"  +0x{i*4:02x} {names[i]:16s} 0x{x:08x}")
    # also read first words of the pool for sanity
    pool = w[0]
    if pool:
        pw = o.mdw(pool, 4)
        print("pool head:", " ".join(f"0x{x:08x}" for x in pw))
finally:
    o.close()
