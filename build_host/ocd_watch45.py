#!/usr/bin/env python3
"""One-shot: single-connection 45s watch — mag link bring-up, IWDG reset
check, NOGNSS seeding (ready flag at ~30s static)."""
import time
from ocd_diag_rst import Ocd, TICK  # reuse helper

MAGCTX_ST = 0x24006188 + 0x50       # mag_data ctx.st (running@0 pushed@4)
GSOL = 0x2400a8c8                    # gins_solution (ready@0 time@8 yaw@0x50)
G_SOL_FIELDS = GSOL

o = Ocd()
try:
    t0 = time.time()
    last_tick = 0
    while time.time() - t0 < 45:
        tick = o.mdw(TICK)[0]
        st = o.mdw(MAGCTX_ST, 2)      # running, pushed
        sol = o.mdw(GSOL, 24)
        ready = sol[0]
        import struct
        yaw = struct.unpack_from("<d", struct.pack("<2I", sol[20], sol[21]))[0]
        elapsed = time.time() - t0
        reset = " <<< MCU RESET" if tick < last_tick else ""
        print(f"t={elapsed:5.1f}s  tick={tick:8d}  mag(run={st[0]},pushed={st[1]})"
              f"  g_sol(ready={ready},yaw={yaw:7.1f}){reset}")
        last_tick = tick
        time.sleep(3)
finally:
    o.close()
