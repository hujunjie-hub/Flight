#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""KF-GINS 结果精度评估 + 数据质量检查 (vs truth.nav)。

用法: python eval_kfgins.py [navresult] [truth] [stdfile] [imuerr] [imu数据] [gnss数据]
全部可省略, 缺省取 dataset/ 下标准文件。
"""
import sys, io, math
import numpy as np

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8")

D2R = math.pi / 180.0
R2D = 180.0 / math.pi
WGS84_RA = 6378137.0
WGS84_E1 = 0.00669437999013

def radiusmn(lat):
    tmp = 1 - WGS84_E1 * np.sin(lat) ** 2
    sqrttmp = np.sqrt(tmp)
    return WGS84_RA * (1 - WGS84_E1) / (sqrttmp * tmp), WGS84_RA / sqrttmp

def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0

def stats_line(name, e, unit):
    return (f"{name:10s} mean={np.mean(e):+9.4f} std={np.std(e):8.4f} "
            f"rms={math.sqrt(np.mean(e**2)):8.4f} max|{np.max(np.abs(e)):.4f}| p95={np.percentile(np.abs(e),95):.4f} {unit}")

def smooth_yaw(y):
    y = y.copy()
    for i in range(1, len(y)):
        d = y[i] - y[i-1]
        if d < -180: y[i:] += 360
        elif d > 180: y[i:] -= 360
    return y

def main():
    base = __file__.rsplit("\\", 2)[0] + "\\dataset\\"
    navpath  = sys.argv[1] if len(sys.argv) > 1 else base + "KF_GINS_Navresult.nav"
    refpath  = sys.argv[2] if len(sys.argv) > 2 else base + "truth.nav"
    stdpath  = sys.argv[3] if len(sys.argv) > 3 else base + "KF_GINS_STD.txt"
    errpath  = sys.argv[4] if len(sys.argv) > 4 else base + "KF_GINS_IMU_ERR.txt"
    imupath  = sys.argv[5] if len(sys.argv) > 5 else base + "Leador-A15.txt"
    gnsspath = sys.argv[6] if len(sys.argv) > 6 else base + "GNSS-RTK.txt"

    print("=" * 78)
    print("一、输入数据质量检查")
    print("=" * 78)
    imu = np.loadtxt(imupath, usecols=range(7))
    dt_imu = np.diff(imu[:, 0])
    print(f"IMU: {len(imu)} 行, {imu[0,0]:.3f} ~ {imu[-1,0]:.3f} s ({imu[-1,0]-imu[0,0]:.1f} s)")
    print(f"     实际速率 {len(imu)/(imu[-1,0]-imu[0,0]):.3f} Hz (名义 200), "
          f"dt min/mean/max = {dt_imu.min():.4f}/{dt_imu.mean():.4f}/{dt_imu.max():.4f} s")
    print(f"     dt 超差(|dt-5ms|>0.1ms) 比例 {(np.abs(dt_imu-0.005)>1e-4).mean()*100:.3f}%, "
          f"时间倒流 {int((dt_imu<=0).sum())} 个, NaN 行 {int(np.isnan(imu).any(axis=1).sum())} 个")
    f_abs = np.linalg.norm(imu[:, 4:7], axis=1)
    w_abs = np.linalg.norm(imu[:, 1:4], axis=1)
    print(f"     |f| 均值 {f_abs.mean():.4f} m/s^2 (min {f_abs.min():.3f} max {f_abs.max():.3f}), "
          f"|w| 均值 {w_abs.mean():.5f} rad/s")

    gnss = np.loadtxt(gnsspath, usecols=range(7))
    dt_g = np.diff(gnss[:, 0])
    gaps = np.where(dt_g > 1.5)[0]
    print(f"GNSS: {len(gnss)} 行, {gnss[0,0]:.3f} ~ {gnss[-1,0]:.3f} s, 平均间隔 {dt_g.mean():.3f} s ({1/dt_g.mean():.2f} Hz)")
    print(f"     间隔 min/max = {dt_g.min():.3f}/{dt_g.max():.3f} s, >1.5s 间隙 {len(gaps)} 处"
          + (f" (首处 @{gnss[gaps[0],0]:.1f}s)" if len(gaps) else ""))
    print(f"     位置 std 列 (N/E/D): 均值 {gnss[:,4].mean():.4f}/{gnss[:,5].mean():.4f}/{gnss[:,6].mean():.4f} m, "
          f"最大 {gnss[:,4].max():.3f}/{gnss[:,5].max():.3f}/{gnss[:,6].max():.3f} m")
    lat0 = gnss[:,1].mean(); lon0 = gnss[:,2].mean()
    rm, rn = radiusmn(lat0 * D2R)
    dn = np.diff(gnss[:,1]) * D2R * (rm + 20)
    de = np.diff(gnss[:,2]) * D2R * (rn + 20) * math.cos(lat0 * D2R)
    step = np.hypot(dn, de)
    print(f"     相邻点位移: mean {step.mean():.3f} m, p99 {np.percentile(step,99):.3f} m, max {step.max():.3f} m")
    print(f"     静止度: 总位移路程 {step.sum():.1f} m, 包络 N {dn.sum():.1f} / E {de.sum():.1f} m")

    # ---------------- 精度评估 ----------------
    print()
    print("=" * 78)
    print("二、导航精度 (vs truth.nav)")
    print("=" * 78)
    nav = np.loadtxt(navpath)
    ref = np.loadtxt(refpath)
    nav[:, 10] = smooth_yaw(nav[:, 10])
    ref[:, 10] = smooth_yaw(ref[:, 10])
    refinter = np.column_stack([
        np.interp(nav[:, 1], ref[:, 1], ref[:, c]) for c in range(2, 11)])
    t = nav[:, 1]

    err = {}
    lat = nav[:, 2] * D2R
    rm, rn = radiusmn(nav[0, 2] * D2R)
    err["dN"] = (nav[:, 2] - refinter[:, 0]) * D2R * (rm + nav[:, 4])
    err["dE"] = (nav[:, 3] - refinter[:, 1]) * D2R * (rn + nav[:, 4]) * np.cos(lat)
    err["dD"] = -(nav[:, 4] - refinter[:, 2])
    for i, k in enumerate(["dvN", "dvE", "dvD"]):
        err[k] = nav[:, 5 + i] - refinter[:, 3 + i]
    for i, k in enumerate(["droll", "dpitch", "dyaw"]):
        err[k] = wrap180(nav[:, 8 + i] - refinter[:, 6 + i])

    conv = t >= t[0] + 60      # 收敛段 (跳过前 60s)
    dur = t[-1] - t[0]
    print(f"结果: {len(nav)} 历元, {t[0]:.3f} ~ {t[-1]:.3f} s ({dur:.1f} s), 收敛段 {conv.sum()} 历元 (>60s)")
    print(f"输出连续性: NaN {int(np.isnan(nav).any(axis=1).sum())} 行, 输出间隔 max {np.diff(t).max()*1000:.2f} ms")

    print("\n-- 全程 --")
    for k in ["dN", "dE", "dD"]:
        print("  " + stats_line(k, err[k], "m"))
    print(f"  {'水平RMS':8s} {math.sqrt(np.mean(err['dN']**2+err['dE']**2)):.4f} m | "
          f"3D RMS {math.sqrt(np.mean(err['dN']**2+err['dE']**2+err['dD']**2)):.4f} m")
    for k in ["dvN", "dvE", "dvD"]:
        print("  " + stats_line(k, err[k], "m/s"))
    for k in ["droll", "dpitch", "dyaw"]:
        print("  " + stats_line(k, err[k], "deg"))

    print("\n-- 收敛段 (>60s) --")
    for k in ["dN", "dE", "dD"]:
        print("  " + stats_line(k, err[k][conv], "m"))
    print(f"  {'水平RMS':8s} {math.sqrt(np.mean(err['dN'][conv]**2+err['dE'][conv]**2)):.4f} m")
    for k in ["dvN", "dvE", "dvD"]:
        print("  " + stats_line(k, err[k][conv], "m/s"))
    for k in ["droll", "dpitch", "dyaw"]:
        print("  " + stats_line(k, err[k][conv], "deg"))

    # ---------------- 一致性: 估计 std vs 实际误差 ----------------
    print()
    print("=" * 78)
    print("三、滤波一致性 (估计 std vs 实际误差, 收敛段)")
    print("=" * 78)
    try:
        std = np.loadtxt(stdpath)
        stdinter = np.column_stack([np.interp(t, std[:, 0], std[:, c]) for c in range(1, 22)])
        pairs = [("dN", 0, "m"), ("dE", 1, "m"), ("dD", 2, "m"),
                 ("dvN", 3, "m/s"), ("dvE", 4, "m/s"), ("dvD", 5, "m/s"),
                 ("droll", 6, "deg"), ("dpitch", 7, "deg"), ("dyaw", 8, "deg")]
        for ek, sc, unit in pairs:
            s = stdinter[conv, sc]
            e = err[ek][conv]
            n1 = (np.abs(e) <= 1 * s).mean() * 100
            n2 = (np.abs(e) <= 2 * s).mean() * 100
            n3 = (np.abs(e) <= 3 * s).mean() * 100
            ratio = np.std(e) / np.mean(s)
            print(f"  {ek:7s} std(估计)={np.mean(s):8.4f} {unit:4s} 实际std={np.std(e):8.4f} "
                  f"实际/估计={ratio:5.2f}  覆盖率 1σ={n1:5.1f}% (期望68.3) 2σ={n2:5.1f}% (95.4) 3σ={n3:5.1f}% (99.7)")
    except Exception as ex:
        print(f"  std 一致性跳过: {ex}")

    # ---------------- IMU 误差估计合理性 ----------------
    print()
    print("=" * 78)
    print("四、IMU 误差估计 (KF 估计值)")
    print("=" * 78)
    try:
        ie = np.loadtxt(errpath)
        n = len(ie)
        names = ["陀螺零偏X deg/h", "陀螺零偏Y deg/h", "陀螺零偏Z deg/h",
                 "加计零偏X mGal", "加计零偏Y mGal", "加计零偏Z mGal"]
        for i in range(6):
            col = ie[:, 1 + i]
            print(f"  {names[i]:16s} 首值 {col[n//20]:+9.4f} -> 末值 {col[-1]:+9.4f} "
                  f"(range {col.min():+.4f}~{col.max():+.4f})")
        nan_ie = int(np.isnan(ie).any(axis=1).sum())
        print(f"  NaN 行: {nan_ie}")
    except Exception as ex:
        print(f"  跳过: {ex}")

    print()
    print("评估完成")

if __name__ == "__main__":
    main()
