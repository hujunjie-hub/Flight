#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
10 轮 x 3 分钟数据质量测试 (2026-10-01, DAPLink/OpenOCD 版).

与 swd_10rounds.py (10x10min, CubeProgrammer/ST-Link) 的差异:
  - 复位与 tick 核验走常驻 OpenOCD telnet (ocd_swd_read), DAPLink 场景
    CubeProgrammer CLI 连不上;
  - 每轮 3 分钟 (swd_stab_alt.py 3), 硬超时 8min;
  - 输出 test10_3min/ (不覆盖 09-30 的 test10/ 证据)。

前置: OpenOCD 已用 `openocd -f cfg/daplink.cfg` 常驻 (telnet 4444)。
"""
import re, subprocess, sys, time, os

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
OUTDIR = os.path.join(HERE, "test10_3min")
os.makedirs(OUTDIR, exist_ok=True)

from ocd_swd_read import ocd_read, ocd_reset_run, ocd_close

import atexit
_LOCK = os.path.join(OUTDIR, ".runner.lock")
try:
    _lf = open(_LOCK, "x")
    _lf.write(str(os.getpid()))
    _lf.close()
except FileExistsError:
    print("another runner instance is active (.runner.lock), abort", flush=True)
    sys.exit(1)
atexit.register(lambda: os.path.exists(_LOCK) and os.remove(_LOCK))


def read_tick():
    import struct
    return struct.unpack("<I", ocd_read(0x2400AA9C, 4))[0]


def reset_verified():
    """OpenOCD reset run + tick 归零核验 (复位偶发不生效且无报错)"""
    for attempt in range(4):
        ocd_reset_run()
        time.sleep(15)
        t = read_tick()
        if t is not None and t < 120000:
            return t
        print(f"[reset-verify] attempt {attempt+1}: tick={t}, retry", flush=True)
    return None


def run_round(n):
    log = os.path.join(OUTDIR, f"round_{n:02d}.log")
    print(f"\n########## ROUND {n}/10 (3 min) ##########", flush=True)
    t = reset_verified()
    if t is None:
        print(f"[round {n}] 复位核验失败, 跳过该轮", flush=True)
        with open(log, "w", encoding="utf-8", newline="\n") as f:
            f.write("reset verification failed\n")
        return 0, 0, ["reset-verify failed"]
    with open(log, "w", encoding="utf-8", newline="\n") as f:
        proc = subprocess.Popen([sys.executable, os.path.join(HERE, "swd_stab_alt.py"), "3"],
                                stdout=f, stderr=subprocess.STDOUT, text=True)
        deadline = time.time() + 8 * 60
        while time.time() < deadline and proc.poll() is None:
            time.sleep(5)
        if proc.poll() is None:
            proc.kill()
            proc.wait(timeout=30)
            f.write("\n[runner] 超时, 已强杀\n")
            f.flush()
    txt = open(log, encoding="utf-8").read()
    m = re.search(r"===== 结果: (\d+)/(\d+) PASS =====", txt)
    npass, ntot = (int(m.group(1)), int(m.group(2))) if m else (0, 0)
    fails = [ln.strip() for ln in txt.splitlines() if "[FAIL]" in ln]
    drift = re.search(r"INS 融合高度漂移率\s*: ([+-][\d.]+) m/s", txt)
    vd = re.search(r"融合垂速 vd 均值\s*: ([+-][\d.]+) m/s", txt)
    print(f"[round {n}] {npass}/{ntot} PASS" +
          (f", drift={drift.group(1)} m/s, vd_mean={vd.group(1)} m/s"
           if drift and vd else ""), flush=True)
    for ln in fails:
        print(f"    FAIL: {ln}", flush=True)
    return npass, ntot, fails


def main():
    t0 = time.time()
    results = []
    try:
        for n in range(1, 11):
            try:
                results.append((n,) + run_round(n))
            except Exception as e:
                print(f"[round {n}] ERROR: {e}", flush=True)
                results.append((n, 0, 0, [f"runner-error: {e}"]))
    finally:
        ocd_close()
    print("\n================ 10 轮汇总 (3 min/轮) ================", flush=True)
    full = 0
    for n, npass, ntot, fails in results:
        tag = "FULL-PASS" if npass == ntot and ntot > 0 else "INVESTIGATE"
        full += (npass == ntot and ntot > 0)
        print(f"  round {n:2d}: {npass:2d}/{ntot:2d}  {tag}", flush=True)
        for ln in fails:
            print(f"      FAIL: {ln}", flush=True)
    print(f"  总计: {full}/10 轮全过, 总耗时 {(time.time()-t0)/60:.0f} min", flush=True)


if __name__ == "__main__":
    main()
