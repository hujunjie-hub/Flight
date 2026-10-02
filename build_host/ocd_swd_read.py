#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OpenOCD telnet (4444) 替代 CubeProgrammer CLI 的 SWD 读写 (DAPLink 场景).
持久连接, 多次读写复用同一 telnet 会话。
用法 (模块导入): from ocd_swd_read import ocd_read, ocd_reset_run, ocd_close
独立运行: python ocd_swd_read.py 0x2400AA9C 4   (读并打印)
         python ocd_swd_read.py reset            (复位并运行)"""
import re, socket, sys, time

_conn = None


def _get_conn(timeout=8.0):
    global _conn
    if _conn is None:
        _conn = socket.create_connection(("127.0.0.1", 4444), timeout=timeout)
        _conn.settimeout(timeout)
        time.sleep(0.2)
        try:
            _conn.recv(4096)                 # banner
        except socket.timeout:
            pass
    return _conn


def _drain():
    s = _get_conn()
    s.settimeout(0.15)
    try:
        while True:
            if not s.recv(4096):
                raise RuntimeError("openocd telnet closed")
    except socket.timeout:
        pass
    finally:
        s.settimeout(8.0)


def _strip_iac(b):
    """剥离 telnet IAC 协商序列 (OpenOCD telnet server 会在数据流中插入
    0xFF FB/FC/FD/FE <opt> 三字节组与子协商, 不剥离会破坏行格式)"""
    out = bytearray()
    i = 0
    n = len(b)
    while i < n:
        c = b[i]
        if c == 0xFF:
            if i + 1 >= n:
                break                          # 尾部悬挂, 丢弃
            nxt = b[i + 1]
            if nxt == 0xFF:
                out.append(0xFF)
                i += 2
            elif nxt in (0xFB, 0xFC, 0xFD, 0xFE):
                i += 3
            elif nxt == 0xFA:                  # 子协商 ... 0xF0
                j = b.find(b"\xF0", i + 2)
                i = (j + 1) if j >= 0 else n
            else:
                i += 2
        else:
            out.append(c)
            i += 1
    return bytes(out)


def _telnet_cmd(cmd, timeout=8.0):
    s = _get_conn(timeout)
    _drain()
    s.sendall((cmd + "\n").encode())
    out = b""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            chunk = s.recv(4096)
            if not chunk:
                raise RuntimeError("openocd telnet closed")
            out += chunk
            # 静默 0.25s 且已出现提示符 -> 完成 (mdw 响应毫秒级完成)
            if out and b">" in out[-16:]:
                time.sleep(0.05)
                s.settimeout(0.25)
                got = False
                try:
                    extra = s.recv(4096)
                    if extra:
                        out += extra
                        got = True
                except socket.timeout:
                    pass
                finally:
                    s.settimeout(timeout)
                if not got:
                    break
        except socket.timeout:
            break
    txt = _strip_iac(out).decode("ascii", "replace").replace("\r", "")
    txt = txt.replace("\x00", "\n")      # 回显后的 NUL 会粘连首行数据
    if "Error" in txt or "error" in txt.split(">", 1)[0]:
        raise RuntimeError("ocd cmd %r failed: %s" % (cmd, txt[-160:]))
    return txt


def ocd_read(addr, size):
    """返回 bytes; 失败抛 RuntimeError"""
    out = _telnet_cmd("mdw 0x%08X %d" % (addr, (size + 3) // 4))
    words = []
    for m in re.finditer(r"^\s*0x[0-9A-Fa-f]{8}\s*:([0-9A-Fa-f ]+)$", out, re.M):
        words += [w for w in m.group(1).split()]
    if len(words) * 4 < size:
        raise RuntimeError("short read 0x%x: %s" % (addr, out[-160:]))
    return b"".join(int(w.zfill(8), 16).to_bytes(4, "little") for w in words)[:size]


def ocd_reset_run():
    _telnet_cmd("reset run", timeout=20.0)


def ocd_close():
    global _conn
    if _conn is not None:
        try:
            _conn.close()
        finally:
            _conn = None


if __name__ == "__main__":
    try:
        if len(sys.argv) >= 2 and sys.argv[1] == "reset":
            ocd_reset_run()
            print("reset run sent")
        else:
            a, n = int(sys.argv[1], 0), int(sys.argv[2])
            data = ocd_read(a, n)
            print(" ".join("%02X" % b for b in data))
    finally:
        ocd_close()
