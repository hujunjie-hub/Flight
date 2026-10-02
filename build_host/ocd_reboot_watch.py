#!/usr/bin/env python3
"""One-shot: MCU reset + 90s watch — verify all sensor links + NOGNSS after
a clean reboot (I2C BUSY stuck likely a power-on race)."""
import time

from ocd_diag_rst import Ocd

TICK = 0x2400aa94
MAGCTX_ST = 0x24006188 + 0x50
IMU_ST = 0x240010b8 + 136
BARO_ST = 0x24007e28 + 84
GSOL = 0x2400a8c8

o = Ocd()
try:
    print("resetting MCU ...")
    print(o.cmd("reset run", 0.5)[:120])
    t0 = time.time()
    last = 0
    while time.time() - t0 < 95:
        tick = o.mdw(TICK)[0]
        mag = o.mdw(MAGCTX_ST, 5)
        imu = o.mdw(IMU_ST, 5)
        baro = o.mdw(BARO_ST, 5)
        sol = o.mdw(GSOL, 26)
        import struct
        yaw = struct.unpack_from("<d", struct.pack("<2I", sol[20], sol[21]))[0]
        reset = " <<<RESET" if tick < last else ""
        print(f"t={time.time()-t0:5.1f}s tick={tick:8d} imu(r{imu[0]},p{imu[1]},e{imu[4]})"
              f" mag(r{mag[0]},p{mag[1]},e{mag[4]}) baro(r{baro[0]},p{baro[1]},e{baro[4]})"
              f" ready={sol[0]} yaw={yaw:6.1f}{reset}")
        last = tick
        time.sleep(8)
finally:
    o.close()
