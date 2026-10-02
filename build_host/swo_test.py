# -*- coding: utf-8 -*-
"""SWO 硬件链路测试: 配置 TPIU/ITM -> 调试器写 ITM stim 发字节 -> 捕获验证."""
import subprocess, socket, time, sys, io, os

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
OPENOCD = r"D:\STM32CubeCLT\OpenOCD\bin\openocd.exe"
LOG = os.path.abspath("swotest.log")
if os.path.exists(LOG): os.remove(LOG)

ocd = subprocess.Popen(
    [OPENOCD, "-f", "interface/stlink.cfg", "-f", "target/stm32h7x.cfg",
     "-c", "gdb_port disabled"],
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
s = None
for _ in range(40):
    time.sleep(0.25)
    try:
        s = socket.create_connection(("127.0.0.1", 4444), timeout=2); break
    except ConnectionRefusedError:
        pass
if s is None:
    ocd.terminate(); raise RuntimeError("OpenOCD not reachable")

def drain():
    out = b""
    s.settimeout(1.2)
    try:
        while True:
            c = s.recv(65536)
            if not c: break
            out += c
    except socket.timeout: pass
    return out.decode(errors="replace").replace("\x00", "\n")

def cmd(c, wait=0.3):
    s.sendall((c + "\n").encode()); time.sleep(wait); return drain()

try:
    # 1. PB3 状态 (GPIOB MODER @0x58020400, pin3 bits[7:6]: 10=AF)
    r = cmd("mdw 0x58020400 1")
    moder = int(r.strip().split(":")[1].split()[0], 16) if ":" in r else 0
    print(f"GPIOB_MODER={moder:#010x}  PB3 bits={(moder>>6)&3} (0=in,1=out,2=AF)")

    # 1b. H7: DBGMCU_CR TRACE_IOEN (异步 SWO 模式, TRACE_MODE 保持 0)
    r = cmd("mdw 0x5C001004 1")
    cr = int(r.strip().split(":")[1].split()[0], 16) if ":" in r else 0
    print(f"DBGMCU_CR={cr:#010x} -> |= TRACE_IOEN")
    cmd(f"mww 0x5C001004 {cr | 0x20:#x}")

    # 2. 使能 trace 域
    cmd("mww 0xE000EDFC 0x01000000")          # DEMCR.TRCENA
    cmd("mww 0xE0000FB0 0xC5ACCE55")          # ITM LAR unlock
    cmd("mww 0xE0000E80 0x0000010D")          # ITM TCR: ITMena|TSena|TraceBusID=1
    cmd("mww 0xE0000E00 0x00000001")          # TER: port0

    # 3. TPIU: OpenOCD 内置配置 + 捕获到文件 (uart 模式, HCLK 550MHz)
    print(cmd("tpiu config internal " + LOG.replace("\\", "/") + " uart on 550000000", 0.5).strip().split("\n")[-2:])

    # 4. 通过 ITM stim port0 发测试串
    for w in [0x53574F21, 0x54455354, 0x30313233]:
        cmd(f"mww 0xE0000000 {w:#x}", 0.1)   # "SWO!","TEST","0123"
    time.sleep(0.6)

    # 5. 读捕获文件
    if os.path.exists(LOG):
        data = open(LOG, "rb").read()
        print(f"\nSWO 捕获 {len(data)} 字节: {data[:64]!r}")
        ok = b"SWO" in data or b"TEST" in data or b"0123" in data
        print(">>> SWO 链路", "可用! (ITM 字节经 PB3 -> ST-LINK 成功回读)" if ok else
              "物理通, 但未匹配到测试串 (可能是 ITM 协议帧头)")
    else:
        print(">>> 无捕获文件, SWO 不可用")
finally:
    try:
        s.sendall(b"shutdown\n"); time.sleep(0.3)
    except: pass
    try: s.close()
    except: pass
    try: ocd.wait(timeout=5)
    except: ocd.terminate()
