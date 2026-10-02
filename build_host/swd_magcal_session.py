#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""磁力计台架标定会话 (SWD, 免串口).

用法:
  python swd_magcal_session.py check           预检: tick/磁链路速率/现有标定/解状态
  python swd_magcal_session.py run [sec]       触发椭球采集并全程记录 raw+姿态,
                                              结束读回拟合结果 (默认 75s)
  python swd_magcal_session.py analyze <csv>   离线轴映射分析 (旋转期间记录的数据,
                                              用拟合出的 S/b 重建校准向量, 对 48 个
                                              候选 AXIS_SRC/SIGN 暴力搜索使导航系
                                              磁场最恒定的映射)

符号/偏移来源: 历史构建 rtthread.elf (原 cmake-build-vscode) (12:20 固件, mag_calib.c 加
g_magcal_swd_req 之后), 见 magcal_syms.py 与 map 文件交叉核对:
  rt_tick         0x2400b00c
  g_magcal_swd_req 0x2400bf40
  s_cal (mag_calib) 0x2400bf48  thread@0 cancel@4 acc@8 (mn@+0x2D0 mx@+0x2E8 cnt@+0x300)
  s_store         0x2400c2d0  data@0 (mag_valid@0 bias@4 softiron@0x10 radius@0x34
                              resid@0x38 ratio@0x3c samples@0x40)
  ctx (mag_data)  0x24006560  st@0x48 (pushed@+4) last@0x60 (T@0 mag@8 cal@0x14 q@0x20)
  g_sol           0x2400ae58  ready@0 time@8 roll@0x40 pitch@0x48 yaw@0x50
"""
import csv
import math
import re
import socket
import subprocess
import sys
import time

OPENOCD = r"D:\STM32CubeCLT\OpenOCD\bin\openocd.exe"
TELNET = ("127.0.0.1", 4444)

# 2026-09-30 重编后地址 (Flight.elf, record_ring 迁移后 ctx 布局:
# mag_data ctx st@0x50 last@0x68; gins_solution 前段布局未变)
TICK = 0x2400aa94
MAG_REQ = 0x2400b1dc
S_CAL = 0x2400b1e0
S_CAL_CNT = S_CAL + 8 + 0x300
S_CAL_MNMX = S_CAL + 8 + 0x2D0
CALIB = 0x2400b568
MAGCTX_ST_PUSHED = 0x24006188 + 0x50 + 4
MAGCTX_LAST = 0x24006188 + 0x68
GSOL = 0x2400a8c8
IMU_LAST = 0x240010b8 + 0xA0           # imu ctx last@0xA0 (DWARF: thread@124 st@136 last@160)


def u32(b, o):
    return int.from_bytes(b[o:o + 4], "little")


def f32(b, o):
    return struct.unpack_from("<f", b, o)[0]


def f64(b, o):
    return struct.unpack_from("<d", b, o)[0]


import struct  # noqa: E402


class Ocd:
    """openocd telnet 客户端 (复用 check_gins_ocd.py 的模式)."""

    def __init__(self):
        self.s = socket.create_connection(TELNET, timeout=5)
        self.buf = b""
        time.sleep(0.3)
        self._drain()

    def _drain(self):
        self.s.settimeout(0.25)
        while True:
            try:
                d = self.s.recv(65536)
            except socket.timeout:
                break
            if not d:
                break
            self.buf += d

    def cmd(self, c, wait=0.03):
        self.buf = b""
        self.s.settimeout(2.0)
        self.s.sendall(c.encode() + b"\n")
        time.sleep(wait)
        self._drain()
        out = re.sub(rb"\xff[\xf0-\xfe].", b"", self.buf)
        out = re.sub(rb"\xff.", b"", out)
        return out.decode("ascii", "replace")

    def mdw(self, addr, n):
        """读 n 个 32bit 字, 返回 bytes (小端拼接)."""
        resp = self.cmd(f"mdw 0x{addr:X} {n}", wait=0.02 + 0.004 * n)
        body = re.sub(r"^.*mdw[^\n]*\n", "", resp, count=1)
        body = re.sub(r"0x[0-9A-Fa-f]{8}:\s*", "", body)
        words = [w for w in re.findall(r"\b([0-9A-Fa-f]{8})\b", body)]
        if len(words) < n:
            raise RuntimeError(f"short mdw 0x{addr:x}: {resp[:200]!r}")
        return b"".join(int(w, 16).to_bytes(4, "little") for w in words[:n])

    def mww(self, addr, val):
        resp = self.cmd(f"mww 0x{addr:X} {val}")
        if "Error" in resp:
            raise RuntimeError(f"mww failed: {resp[:200]}")


def start_openocd():
    p = subprocess.Popen(
        [OPENOCD, "-f", "interface/cmsis-dap.cfg", "-f", "target/stm32h7x.cfg",
         "-c", "adapter speed 10000"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(40):
        time.sleep(0.25)
        try:
            return p, Ocd()
        except OSError:
            continue
    p.kill()
    raise RuntimeError("openocd telnet 4444 未就绪")


# ------------------------- 采样 -------------------------

def read_calib(ocd):
    b = ocd.mdw(CALIB, 0x44 // 4 + 1)
    d = {"mag_valid": u32(b, 0)}
    d["bias"] = [f32(b, 4 + 4 * i) for i in range(3)]
    d["softiron"] = [[f32(b, 0x10 + 4 * (3 * i + j)) for j in range(3)]
                     for i in range(3)]
    d["radius"] = f32(b, 0x34)
    d["resid"] = f32(b, 0x38)
    d["ratio"] = f32(b, 0x3C)
    d["samples"] = int.from_bytes(b[0x40:0x42], "little")
    return d


def read_link(ocd):
    pushed = u32(ocd.mdw(MAGCTX_ST_PUSHED, 1), 0)
    tick = u32(ocd.mdw(TICK, 1), 0)
    last = ocd.mdw(MAGCTX_LAST, 10)
    return pushed, tick, last


def precheck(ocd):
    print("== 预检 ==")
    t1, p1 = u32(ocd.mdw(TICK, 1), 0), u32(ocd.mdw(MAGCTX_ST_PUSHED, 1), 0)
    time.sleep(1.5)
    t2, p2 = u32(ocd.mdw(TICK, 1), 0), u32(ocd.mdw(MAGCTX_ST_PUSHED, 1), 0)
    dr = (p2 - p1) / max((t2 - t1) / 1000.0, 0.1)
    print(f"tick      : {t1} -> {t2} ({t2 - t1} ms/1.5s) "
          f"{'OK' if 1000 < t2 - t1 < 2500 else '异常!'}")
    print(f"mag pushed: {p2 - p1}/1.5s = {dr:.1f}/s "
          f"{'OK' if 80 < dr < 130 else '异常! (应 ~100/s)'}")
    last = ocd.mdw(MAGCTX_LAST, 10)
    raw = [f32(last, 8 + 4 * i) for i in range(3)]
    cal = [f32(last, 0x14 + 4 * i) for i in range(3)]
    print(f"last raw  : {raw[0]:8.2f} {raw[1]:8.2f} {raw[2]:8.2f} uT (传感器系)")
    print(f"last cal  : {cal[0]:8.2f} {cal[1]:8.2f} {cal[2]:8.2f} uT (体系)")
    sol = ocd.mdw(GSOL, 0x58 // 4)
    print(f"g_sol     : ready={u32(sol, 0)} "
          f"roll={f64(sol, 0x40):.2f} pitch={f64(sol, 0x48):.2f} "
          f"yaw={f64(sol, 0x50):.2f} deg (已为角度制)")
    print(f"swd_req   : {u32(ocd.mdw(MAG_REQ, 1), 0)} (应为 0)")
    print("calib     :", read_calib(ocd))
    return 80 < dr < 130


def run_session(ocd, seconds):
    base_pre = read_calib(ocd)
    print(f"触发前标定镜像: valid={base_pre['mag_valid']} "
          f"radius={base_pre['radius']:.1f} samples={base_pre['samples']}")

    print(f"\n>>> 触发采集 {seconds}s —— 现在开始缓慢旋转板子! <<<")
    ocd.mww(MAG_REQ, seconds)
    t0 = time.time()
    csv_path = time.strftime("magcal_%H%M%S.csv")
    f = open(csv_path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "tick", "pushed", "acc_cnt", "raw0", "raw1", "raw2",
                "cal0", "cal1", "cal2", "q", "roll", "pitch", "yaw"])
    n = 0
    busy_prev = None
    try:
        while True:
            el = time.time() - t0
            hdr = ocd.mdw(S_CAL, 1)                     # thread 指针
            busy = u32(hdr, 0) != 0
            cnt_b = ocd.mdw(S_CAL_CNT, 1)
            cnt = u32(cnt_b, 0)
            pushed, tick, last = read_link(ocd)
            sol = ocd.mdw(GSOL, 0x58 // 4)
            w.writerow([f"{el:.2f}", tick, pushed, cnt,
                        f32(last, 8), f32(last, 12), f32(last, 16),
                        f32(last, 0x14), f32(last, 0x18), f32(last, 0x1C),
                        last[0x20],
                        f64(sol, 0x40), f64(sol, 0x48), f64(sol, 0x50)])
            f.flush()
            n += 1
            if busy_prev and not busy and el > 3:
                break                                   # 采集线程退出
            if el > seconds + 25:
                print("超时退出 (采集线程未在预期时间内结束)")
                break
            busy_prev = busy
            time.sleep(0.2)
    finally:
        f.close()
    print(f"\n记录 {n} 点 -> {csv_path}, 采集线程状态: "
          f"{'已结束' if not busy else '仍在运行?!'}")
    time.sleep(1.0)                                     # 拟合+flash 写完成
    post = read_calib(ocd)
    print(f"\n== 拟合结果 ==\n触发前: valid={base_pre['mag_valid']} "
          f"radius={base_pre['radius']:.2f} samples={base_pre['samples']}")
    print(f"触发后: valid={post['mag_valid']} radius={post['radius']:.2f} "
          f"resid={post['resid']:.4f} ratio={post['ratio']:.3f} "
          f"samples={post['samples']}")
    print(f"bias  uT : [{post['bias'][0]:8.2f} {post['bias'][1]:8.2f} {post['bias'][2]:8.2f}]")
    print("softiron :")
    for row in post["softiron"]:
        print("   [" + " ".join(f"{v:8.4f}" for v in row) + "]")
    if post["mag_valid"] and (post["samples"] != base_pre["samples"]
                              or post["radius"] != base_pre["radius"]):
        print("\n判定: 新参数已保存并生效 ✓")
    elif not post["mag_valid"]:
        print("\n判定: 无有效参数 (拟合被门限拒绝, 见日志)")
    else:
        print("\n判定: 参数未变化 (拟合可能失败, 旧参数保留)")
    return csv_path, post


# ------------------------- 离线轴映射分析 -------------------------

def euler2cbn(roll, pitch, yaw):
    """KF-GINS ZYX: Cbn = Rz(yaw)Ry(pitch)Rx(roll), 输入 rad."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Rz(yaw) @ Ry(pitch) @ Rx(roll)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def analyze(csv_path, calib):
    rows = list(csv.DictReader(open(csv_path)))
    S = calib["softiron"]
    b = calib["bias"]
    pts = []
    for r in rows:
        try:
            raw = [float(r["raw0"]), float(r["raw1"]), float(r["raw2"])]
            rpy = [math.radians(float(r["roll"])),
                   math.radians(float(r["pitch"])),
                   math.radians(float(r["yaw"]))]
        except ValueError:
            continue
        if abs(math.degrees(rpy[1])) > 80:      # pitch 接近万向节锁, 跳过
            continue
        v = [sum(S[i][j] * (raw[j] - b[j]) for j in range(3)) for i in range(3)]
        pts.append((v, rpy))
    if len(pts) < 40:
        print(f"样本太少 ({len(pts)}), 无法分析")
        return
    print(f"分析 {len(pts)} 点 (已剔除 |pitch|>80°)")

    import itertools
    results = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product([1.0, -1.0], repeat=3):
            det = (1 if perm in ((0, 1, 2), (1, 2, 0), (2, 0, 1)) else -1) \
                * signs[0] * signs[1] * signs[2]
            B = []
            for v, rpy in pts:
                m = [signs[i] * v[perm[i]] for i in range(3)]
                C = euler2cbn(*rpy)
                B.append([sum(C[i][j] * m[j] for j in range(3)) for i in range(3)])
            mean = [sum(x[k] for x in B) / len(B) for k in range(3)]
            var = sum(sum((x[k] - mean[k]) ** 2 for k in range(3)) for x in B) / len(B)
            results.append((var, perm, signs, det, mean))
    results.sort()
    var, perm, signs, det, mean = results[0]
    nrm = math.sqrt(sum(x * x for x in mean))
    print(f"\n最优映射 (det={det:+.0f}): SRC={{{perm[0]}, {perm[1]}, {perm[2]}}}"
          f"  SIGN={{{signs[0]:.0f}, {signs[1]:.0f}, {signs[2]:.0f}}}")
    print(f"  导航系磁场均值 = [{mean[0]:7.2f} {mean[1]:7.2f} {mean[2]:7.2f}] uT,"
          f" |B|={nrm:.1f} uT (北 东 地)")
    print(f"  残差 rms = {math.sqrt(var):.2f} uT")
    print(f"  次优 (应显著更差, 否则数据姿态覆盖不足):")
    for var2, perm2, signs2, det2, mean2 in results[1:4]:
        print(f"    rms={math.sqrt(var2):6.2f} uT  "
              f"SRC={{{perm2[0]},{perm2[1]},{perm2[2]}}} "
              f"SIGN={{{signs2[0]:.0f},{signs2[1]:.0f},{signs2[2]:.0f}}} det={det2:+.0f}")
    # 水平投影恒定性 (排除绕 D 慢漂的 yaw 漂移影响)
    Hh = []
    for v, rpy in pts:
        m = [signs[i] * v[perm[i]] for i in range(3)]
        C = euler2cbn(*rpy)
        Bv = [sum(C[i][j] * m[j] for j in range(3)) for i in range(3)]
        Hh.append(math.hypot(Bv[0], Bv[1]))
    hm = sum(Hh) / len(Hh)
    print(f"  水平分量 |H|: mean={hm:.2f} uT, std={math.sqrt(sum((x - hm) ** 2 for x in Hh) / len(Hh)):.2f} uT")
    return perm, signs, mean, nrm


def log_session(ocd, seconds, csv_path=None):
    """慢旋转记录: mag raw + accel (轴映射判别用, 不依赖 INS/GNSS)."""
    if not csv_path:
        csv_path = time.strftime("maglog_%H%M%S.csv")
    f = open(csv_path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "tick", "raw0", "raw1", "raw2", "cal0", "cal1", "cal2",
                "q", "ax", "ay", "az", "gx", "gy", "gz",
                "roll", "pitch", "yaw"])
    t0 = time.time()
    n = 0
    try:
        while time.time() - t0 < seconds:
            last = ocd.mdw(MAGCTX_LAST, 10)
            imu = ocd.mdw(IMU_LAST, 10)
            sol = ocd.mdw(GSOL, 0x58 // 4)
            tick = u32(ocd.mdw(TICK, 1), 0)
            w.writerow([f"{time.time()-t0:.2f}", tick,
                        f32(last, 8), f32(last, 12), f32(last, 16),
                        f32(last, 0x14), f32(last, 0x18), f32(last, 0x1C),
                        last[0x20],
                        f32(imu, 0x14), f32(imu, 0x18), f32(imu, 0x1C),
                        f32(imu, 8), f32(imu, 12), f32(imu, 16),
                        f64(sol, 0x40), f64(sol, 0x48), f64(sol, 0x50)])
            n += 1
            time.sleep(0.12)
    finally:
        f.close()
    print(f"记录 {n} 点 -> {csv_path}")
    return csv_path


def analyze2(csv_path, calib):
    """accel 基准轴映射判别: 水平化后的磁场 D 分量/水平模长与未知偏航无关,
    正确映射下应跨姿态恒定。不依赖 INS 姿态/GNSS。"""
    import itertools
    rows = list(csv.DictReader(open(csv_path)))
    S = calib["softiron"]
    b = calib["bias"]
    pts = []
    for r in rows:
        try:
            raw = [float(r["raw0"]), float(r["raw1"]), float(r["raw2"])]
            acc = [float(r["ax"]), float(r["ay"]), float(r["az"])]
            gyr = [float(r["gx"]), float(r["gy"]), float(r["gz"])]
        except ValueError:
            continue
        anorm = math.sqrt(sum(x * x for x in acc))
        gn = math.sqrt(sum(x * x for x in gyr))
        if abs(anorm - 9.81) > 0.6 or gn > 0.6:      # 仅保留准静态样本
            continue
        v = [sum(S[i][j] * (raw[j] - b[j]) for j in range(3)) for i in range(3)]
        pts.append((v, [-x / anorm for x in acc]))   # d_b = -a/|a| (重力向下方向)
    if len(pts) < 30:
        print(f"准静态样本太少 ({len(pts)}), 旋转需更慢更平稳")
        return
    print(f"准静态样本 {len(pts)}/{len(rows)}")

    def rot_z_to_d(d):
        """返回 R 使 R@(0,0,1)=d (用 xy 平面两轴正交化构造, 数值稳)"""
        x = [-d[1], d[0], 0.0]
        nx = math.sqrt(x[0] * x[0] + x[1] * x[1])
        if nx < 1e-9:
            x = [1.0, 0.0, 0.0] if d[2] > 0 else [1.0, 0.0, 0.0]
            nx = 1.0
        x = [v / nx for v in x]
        y = [d[1] * x[2] - d[2] * x[1], d[2] * x[0] - d[0] * x[2],
             d[0] * x[1] - d[1] * x[0]]      # y = d×x
        return [[x[0], y[0], d[0]], [x[1], y[1], d[1]], [x[2], y[2], d[2]]]

    results = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product([1.0, -1.0], repeat=3):
            det = (1 if perm in ((0, 1, 2), (1, 2, 0), (2, 0, 1)) else -1)                 * signs[0] * signs[1] * signs[2]
            cds, hns = [], []
            for v, d in pts:
                m = [signs[i] * v[perm[i]] for i in range(3)]
                R = rot_z_to_d(d)
                # R^T m
                lm = [R[0][0] * m[0] + R[1][0] * m[1] + R[2][0] * m[2],
                      R[0][1] * m[0] + R[1][1] * m[1] + R[2][1] * m[2],
                      R[0][2] * m[0] + R[1][2] * m[1] + R[2][2] * m[2]]
                cds.append(lm[2])
                hns.append(math.hypot(lm[0], lm[1]))
            mc = sum(cds) / len(cds)
            mh = sum(hns) / len(hns)
            score = (sum((x - mc) ** 2 for x in cds) +
                     sum((x - mh) ** 2 for x in hns)) / len(cds)
            # 全局符号简并: 方差绕均值无法区分 w->-w 的孪生映射;
            # 北半球磁场向下 (D 分量为正, 深圳 WMM ~+27uT), 据此硬过滤
            if mc < 15.0:
                score += 1e6
            results.append((score, perm, signs, det, mc, mh))
    results.sort()
    score, perm, signs, det, mc, mh = results[0]
    print("")
    print(f"最优映射 (det={det:+.0f}): SRC={{{perm[0]}, {perm[1]}, {perm[2]}}}"
          f"  SIGN={{{signs[0]:.0f}, {signs[1]:.0f}, {signs[2]:.0f}}}")
    print(f"  水平化后 D 分量均值 B_D={mc:.2f} uT (深圳 WMM 预期 ~+27), "
          f"|B_H|={mh:.2f} uT (预期 ~40), 总场={math.sqrt(mc*mc+mh*mh):.1f} uT")
    print(f"  恒定性残差 rms={math.sqrt(score):.3f} uT")
    print(f"  次优:")
    for s2, p2, g2, d2, c2, h2 in results[1:5]:
        print(f"    rms={math.sqrt(s2):7.3f} uT  SRC={{{p2[0]},{p2[1]},{p2[2]}}} "
              f"SIGN={{{g2[0]:.0f},{g2[1]:.0f},{g2[2]:.0f}}} det={d2:+.0f}")
    return perm, signs


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "check"
    proc, ocd = start_openocd()
    try:
        if mode == "check":
            precheck(ocd)
        elif mode == "run":
            sec = int(sys.argv[2]) if len(sys.argv) > 2 else 75
            if not precheck(ocd):
                print("预检未过, 中止")
                return
            input("\n按回车触发采集 (期间缓慢旋转板子, 覆盖所有姿态)... ")
            csv_path, calib = run_session(ocd, sec)
            if calib["mag_valid"]:
                analyze(csv_path, calib)
        elif mode == "analyze":
            # 用当前板上标定参数分析已有 csv
            analyze(sys.argv[2], read_calib(ocd))
        elif mode == "log":
            sec = int(sys.argv[2]) if len(sys.argv) > 2 else 100
            print(">>> 慢旋转开始: 缓慢翻滚板子覆盖各姿态 (准静态, 每姿态停 1-2s) <<<")
            input("回车开始记录...")
            log_session(ocd, sec)
        elif mode == "analyze2":
            analyze2(sys.argv[2], read_calib(ocd))
        else:
            print(__doc__)
    finally:
        ocd.s.close()
        proc.terminate()


if __name__ == "__main__":
    main()
