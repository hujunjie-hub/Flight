#!/usr/bin/env python3
"""One-shot: decisive I2C probe — bit-bang PB6(SCL)/PB7(SDA) as open-drain
GPIO, send START + address byte, check ACK from BMM350 (0x15) / BMP585
(0x46/0x47 on PB10/PB11). Bypasses firmware entirely."""
import time

from ocd_diag_rst import Ocd

GPIOB = 0x58020400
# MODER@0 OTYPER@4 OSPEEDR@8 PUPDR@0x0C IDR@0x10 ODR@0x14
# PB6=SCL1 PB7=SDA1 (I2C1)  PB10=SCL2 PB11=SDA2 (I2C2)


class Pins:
    def __init__(self, ocd, scl, sda):
        self.o = ocd
        self.scl, self.sda = scl, sda

    def wr(self, off, val):
        self.o.cmd("mww 0x%08x 0x%08x" % (GPIOB + off, val), 0.02)

    def rd(self, off):
        return self.o.mdw(GPIOB + off, 1)[0]

    def setup(self):
        m = self.rd(0)
        m &= ~((3 << (self.scl * 2)) | (3 << (self.sda * 2)))   # both output
        m |= (1 << (self.scl * 2)) | (1 << (self.sda * 2))
        self.wr(0, m)
        t = self.rd(4) | (1 << self.scl) | (1 << self.sda)       # open-drain
        self.wr(4, t)
        d = self.rd(0x14) | (1 << self.scl) | (1 << self.sda)    # release both
        self.wr(0x14, d)
        time.sleep(0.01)

    def restore(self):
        # back to AF mode is firmware's job on reboot; just set AF (10) safe:
        m = self.rd(0)
        m |= (2 << (self.scl * 2)) | (2 << (self.sda * 2))
        self.wr(0, m)

    def scl_h(self):
        d = self.rd(0x14) | (1 << self.scl)
        self.wr(0x14, d)

    def scl_l(self):
        d = self.rd(0x14) & ~(1 << self.scl)
        self.wr(0x14, d)

    def sda_h(self):
        d = self.rd(0x14) | (1 << self.sda)
        self.wr(0x14, d)

    def sda_l(self):
        d = self.rd(0x14) & ~(1 << self.sda)
        self.wr(0x14, d)

    def sda_read(self):
        return (self.rd(0x10) >> self.sda) & 1

    def half(self):
        time.sleep(0.003)

    def byte(self, b):
        for i in range(7, -1, -1):
            if (b >> i) & 1:
                self.sda_h()
            else:
                self.sda_l()
            self.half()
            self.scl_h()
            self.half()
            self.scl_l()
        # ACK slot
        self.sda_h()
        self.half()
        self.scl_h()
        self.half()
        ack = self.sda_read()          # 0 = ACK (slave pulled low)
        self.scl_l()
        return ack == 0


def probe(ocd, name, scl, sda, addr7):
    p = Pins(ocd, scl, sda)
    p.setup()
    # idle
    p.scl_h(); p.sda_h(); p.half()
    # START
    p.sda_l(); p.half(); p.scl_l(); p.half()
    ok = p.byte(addr7 << 1)            # address + write
    # STOP
    p.sda_l(); p.half(); p.scl_h(); p.half(); p.sda_h(); p.half()
    p.restore()
    print(f"{name}: addr 0x{addr7:02x} -> {'ACK (chip on bus!)' if ok else 'NACK (no response)'}")


o = Ocd()
try:
    print(o.cmd("halt", 0.3)[:80])
    probe(o, "I2C1/BMM350", 6, 7, 0x15)
    probe(o, "I2C2/BMP585", 10, 11, 0x46)
    probe(o, "I2C2/BMP585-alt", 10, 11, 0x47)
    print(o.cmd("resume", 0.2)[:60])
finally:
    o.close()
