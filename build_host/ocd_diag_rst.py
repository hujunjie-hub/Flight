#!/usr/bin/env python3
"""One-shot: read reset-source (RCC_RSR) + boot tick + IWDG-related status."""
import socket
import subprocess
import struct
import time

OPENOCD = r"D:\STM32CubeCLT\OpenOCD\bin\openocd.exe"


class Ocd:
    def __init__(self):
        # DAPLink 探针 (CMSIS-DAP, cfg/daplink.cfg: 10MHz)
        self.p = subprocess.Popen(
            [OPENOCD, "-f", "interface/cmsis-dap.cfg", "-f", "target/stm32h7x.cfg",
             "-c", "adapter speed 10000"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(40):
            time.sleep(0.25)
            try:
                self.s = socket.create_connection(("127.0.0.1", 4444), timeout=2)
                self.s.recv(4096)
                return
            except OSError:
                continue
        raise RuntimeError("openocd telnet not ready")

    def cmd(self, c, wait=0.2):
        self.s.sendall((c + "\n").encode())
        time.sleep(wait)
        out = b""
        while True:
            try:
                ch = self.s.recv(65536)
                if not ch:
                    break
                out += ch
            except socket.timeout:
                break
        return out.decode(errors="replace")

    def mdw(self, addr, n=1):
        r = self.cmd("mdw 0x%08x %d" % (addr, n))
        words = []
        for line in r.splitlines():
            if ":" in line:
                for tok in line.split(":", 1)[1].split():
                    try:
                        words.append(int(tok, 16))
                    except ValueError:
                        pass
        return words

    def close(self):
        try:
            self.s.close()
        finally:
            self.p.terminate()
            try:
                self.p.wait(timeout=5)
            except Exception:
                pass


TICK = 0x2400aa94
RSR = 0x58024D28          # RCC_RSR (STM32H7)
IWDG_SR = 0x58004C0C      # IWDG_SR? (IWDG base 0x58004C00, SR @+0xC)

if __name__ == "__main__":
    o = Ocd()
    try:
        t1 = o.mdw(TICK)[0]
        time.sleep(1.5)
        t2 = o.mdw(TICK)[0]
        rsr = o.mdw(RSR)[0]
        print(f"tick : {t1} -> {t2} ({'alive' if t2 != t1 else 'STATIC!'})")
        print(f"RSR  : 0x{rsr:08x}")
        flags = []
        if rsr & (1 << 16):
            flags.append("IWDG1_RESET")
        if rsr & (1 << 17):
            flags.append("WWDG1_RESET")
        if rsr & (1 << 24):
            flags.append("PIN_RESET")
        if rsr & (1 << 26):
            flags.append("BORRSTF")
        if rsr & (1 << 28):
            flags.append("SFTRSTF")
        if rsr & (1 << 29):
            flags.append("PORRSTF")
        print("flags:", " | ".join(flags) if flags else "(none known)")
    finally:
        o.close()
