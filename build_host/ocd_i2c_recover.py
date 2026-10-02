#!/usr/bin/env python3
"""One-shot: recover stuck I2C peripherals (BUSY flag with idle bus) by
PE toggle (software reset), then watch BMM350 retry thread for recovery."""
import time

from ocd_diag_rst import Ocd

I2C1 = 0x40005400
I2C2 = 0x40005800
MAGCTX_ST = 0x24006188 + 0x50
BMM = 0x24000d58


def busy(o, base):
    return (o.mdw(base + 0x18 >> 2, 1)[0] >> 15) & 1


o = Ocd()
try:
    print("before: I2C1 BUSY=%d I2C2 BUSY=%d" % (busy(o, I2C1), busy(o, I2C2)))
    for base in (I2C1, I2C2):
        cr1 = o.mdw(base, 1)[0]
        # PE=0 (soft reset of peripheral state machine)
        o.cmd("mww 0x%08x %d" % (base, cr1 & ~1), 0.2)
        time.sleep(0.05)
        o.cmd("mww 0x%08x %d" % (base, cr1 | 1), 0.2)      # PE=1 re-enable
        time.sleep(0.05)
    print("after : I2C1 BUSY=%d I2C2 BUSY=%d" % (busy(o, I2C1), busy(o, I2C2)))

    # watch BMM350 retry thread for up to ~8s
    for i in range(4):
        time.sleep(2)
        st = o.mdw(MAGCTX_ST, 5)
        w1 = o.mdw(BMM + 4, 1)[0]
        chip_id = (w1 >> 8) & 0xFF
        print(f"t+{(i + 1) * 2}s: mag pushed={st[1]} errors={st[4]} "
              f"chip_id=0x{chip_id:02x} powered={(w1 >> 24) & 0xFF}")
finally:
    o.close()
