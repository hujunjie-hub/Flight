#!/usr/bin/env python3
"""One-shot: quick OpenOCD telnet mdw probe (reads words, prints hex)."""
import socket
import subprocess
import sys
import time

OPENOCD = r"D:\STM32CubeCLT\OpenOCD\bin\openocd.exe"
CFG = "-f ../cfg/stlink.cfg"


class Ocd:
    def __init__(self):
        self.p = subprocess.Popen([OPENOCD, CFG], stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL)
        time.sleep(2.0)
        self.s = socket.create_connection(("127.0.0.1", 4444), timeout=3)
        self.s.recv(4096)

    def cmd(self, c):
        self.s.sendall((c + "\n").encode())
        time.sleep(0.15)
        out = b""
        while True:
            try:
                chunk = self.s.recv(65536)
                if not chunk:
                    break
                out += chunk
            except socket.timeout:
                break
        return out.decode(errors="replace")

    def mdw(self, addr, n=1):
        r = self.cmd("mdw 0x%08x %d" % (addr, n))
        words = []
        for line in r.splitlines():
            line = line.strip()
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
            self.p.wait(timeout=5)


if __name__ == "__main__":
    ocd = Ocd()
    try:
        for name, addr in [("old TICK", 0x2400b00c), ("new TICK", 0x2400aa94)]:
            w = ocd.mdw(addr)
            time.sleep(1.0)
            w2 = ocd.mdw(addr)
            print(f"{name} @0x{addr:08x}: 0x{w[0]:08x} -> 0x{w2[0]:08x} "
                  f"({'moving' if w != w2 else 'static'})")
    finally:
        ocd.close()
