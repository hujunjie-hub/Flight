#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
10 轮 x 10min 数据质量测试运行器 (2026-09-30 修复固件验证).

每轮: 硬复位 (10 次冷启动, 顺带锻炼新初始化路径: NaN 看门狗/对准滑动
均值/calib 追加日志加载/BMM350-BMP585 重试) -> 15s 收敛等待 ->
swd_stab_alt.py 10min (17 判据 + 高度专项) -> 汇总解析.
"""
import re, subprocess, sys, time, os

CLI = r"D:\STM32CubeCLT\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
HERE = os.path.dirname(os.path.abspath(__file__))
OUTDIR = os.path.join(HERE, "test10")
os.makedirs(OUTDIR, exist_ok=True)


# 单实例锁: 两个监控实例并发连 SWD 会互相破坏读数 (曾致 s_engine 读出
# 垃圾指针无限重试), O_EXCL 锁文件兜底
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

def hard_reset(under=False):
    mode = "UnderReset" if under else "HotPlug"
    subprocess.run([CLI, "-c", "port=SWD", "mode=" + mode, "-hardRst"],
                   capture_output=True, text=True, timeout=60)

def reset_verified():
    """硬复位并核验 tick 确实归零 (CLI 复位偶发不生效且无报错);
    HotPlug 两次不行换 UnderReset 恢复 DAP 后再试"""
    for attempt in range(4):
        hard_reset(under=(attempt >= 2))
        time.sleep(15)
        t = read_tick()
        if t is not None and t < 120000:
            return t
        print(f"[reset-verify] attempt {attempt+1}: tick={t}, retry", flush=True)
    return None

def read_tick():
    out = subprocess.run([CLI, "-c", "port=SWD", "mode=HotPlug", "-r32",
                          "0x2400B004", "4"], capture_output=True, text=True,
                         timeout=60).stdout
    m = re.search(r"0x2400B004\s*:\s*([0-9A-Fa-f]{8})", out)
    return int(m.group(1), 16) if m else None

def run_round(n):
    log = os.path.join(OUTDIR, f"round_{n:02d}.log")
    print(f"\n########## ROUND {n}/10 ##########", flush=True)
    t = reset_verified()
    if t is None:
        print(f"[round {n}] 复位核验失败, 跳过该轮", flush=True)
        with open(log, "w", encoding="utf-8", newline="\n") as f:
            f.write("reset verification failed\n")
        return 0, 0, ["reset-verify failed"]
    with open(log, "w", encoding="utf-8", newline="\n") as f:
        # 显式 Popen + 轮询硬超时: subprocess.run(timeout=) 在本机曾不触发,
        # 子进程卡死拖住整轮 (实测 4.7h), 必须 kill 兜底
        proc = subprocess.Popen([sys.executable, os.path.join(HERE, "swd_stab_alt.py"), "10"],
                                stdout=f, stderr=subprocess.STDOUT, text=True)
        # 每拍 = 60s 间隔 + 8 次 CLI 采样 (~10-25s), 11 拍实测 12-15min:
        # 14min 硬超时会误杀慢速 CLI 轮次 (round 6 教训), 放宽到 19min
        deadline = time.time() + 19 * 60
        while time.time() < deadline and proc.poll() is None:
            time.sleep(5)
        if proc.poll() is None:
            proc.kill()
            proc.wait(timeout=30)
            f.write("\n[runner] 超时 14min, 已强杀\n")
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
    for n in range(1, 11):
        try:
            results.append((n,) + run_round(n))
        except Exception as e:
            print(f"[round {n}] ERROR: {e}", flush=True)
            results.append((n, 0, 0, [f"runner-error: {e}"]))
    print("\n================ 10 轮汇总 ================", flush=True)
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
