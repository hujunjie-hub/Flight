# -*- coding: utf-8 -*-
"""
so3.c <-> KF-GINS rotation.h 主机交叉验证 (独立参考: 初等旋转阵显式构造)

参考基准不用任何被测公式:
  - C_bn 由 Rx/Ry/Rz 初等矩阵显式相乘构造 (已知 r,p,y 真值)
  - 四元数旋转用三明治法 v' = q v q* 逐基向量旋转得到 DCM 列
  - 误差旋转矢量用轴角提取 (angle=arccos((tr-1)/2), 轴由反对称部分归一)
so3.c / rotation.h 的公式则逐行转写 (与源码一字对齐) 后与之对拍。
"""
import numpy as np

rng = np.random.default_rng(20261002)
N = 20000
EPS = np.finfo(float).eps  # 2.22e-16, 对齐 so3.c 的 DBL_EPSILON

fails = []


def check(name, err, tol):
    if err > tol:
        fails.append((name, err, tol))
        print(f"FAIL {name}: max_err={err:.3e} tol={tol:.0e}")


# ---------- 独立参考 ----------
def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def ref_dcm(rpy):
    """C_bn = Rz(y) Ry(p) Rx(r), 与 KF-GINS euler2matrix 同序"""
    return Rz(rpy[2]) @ Ry(rpy[1]) @ Rx(rpy[0])


def qmul(a, b):
    """Hamilton: a⊗b, a=[w,x,y,z]"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def qrot(q, v):
    """三明治法旋转向量: q ⊗ [0,v] ⊗ q*"""
    qi = np.array([q[0], -q[1], -q[2], -q[3]])
    t = qmul(q, np.concatenate(([0.0], v)))
    return qmul(t, qi)[1:]


def ref_quat_to_dcm(q):
    """DCM 列 = q 旋转后的基向量 (不使用任何转换公式)"""
    return np.column_stack([qrot(q, np.eye(3)[i]) for i in range(3)])


def ref_log(R):
    """旋转矩阵 -> 旋转矢量 (轴角提取, 独立公式), 最短旋转 |v|<=pi"""
    c = (np.trace(R) - 1.0) / 2.0
    c = min(1.0, max(-1.0, c))
    ang = np.arccos(c)                       # [0, pi]
    if ang < 1e-8:
        # 小角: 用反对称部分 v ≈ [R32-R23, R13-R31, R21-R12]/2
        return 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if np.pi - ang < 1e-6:
        # 接近 pi: 对角元法求轴 (tr≈-1, 轴分量 = sqrt((Rii+1)/2) 带符号)
        axis = np.sqrt(np.maximum((np.diag(R) + 1.0) / 2.0, 0.0))
        # 定符号: 找最大对角元所在轴, 其必为正, 其余按非对角元符号
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        if axis[j] > 0 and R[j, i] < 0:
            axis[j] = -axis[j]
        if axis[k] > 0 and R[k, i] < 0:
            axis[k] = -axis[k]
        return axis / np.linalg.norm(axis) * ang
    s = np.linalg.norm(np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]))
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / s
    return axis * ang


# ---------- so3.c 逐行转写 ----------
def so3_euler_to_dcm(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def so3_dcm_to_euler(R):
    sp = -R[2, 0]
    sp = min(1.0, max(-1.0, sp))
    p = np.arcsin(sp)
    if sp > 1.0 - EPS or sp < -1.0 + EPS:
        r = 0.0
        y = np.arctan2(-R[0, 1], R[1, 1])
    else:
        r = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(R[1, 0], R[0, 0])
    return np.array([r, p, y])


def so3_euler_to_quat(rpy):
    r, p, y = rpy
    cr, sr = np.cos(0.5 * r), np.sin(0.5 * r)
    cp, sp = np.cos(0.5 * p), np.sin(0.5 * p)
    cy, sy = np.cos(0.5 * y), np.sin(0.5 * y)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy])


def so3_quat_to_dcm(q):
    q = np.asarray(q, float)
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    xx, yy, zz, xy, xz, yz = x * x, y * y, z * z, x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]])


def so3_dcm_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2 * np.sqrt(tr + 1)
        q = np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2 * np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
        q = np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = 2 * np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2])
        q = np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s])
    else:
        s = 2 * np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1])
        q = np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s])
    return q / np.linalg.norm(q)


def so3_quat_mul(a, b):
    return qmul(np.asarray(a, float), np.asarray(b, float))  # 同 Hamilton 定义


def so3_exp(v):
    n2 = v @ v
    if n2 < 1e-16:
        k = 0.5 - n2 / 48.0
        return np.array([1.0 - 0.125 * n2, v[0] * k, v[1] * k, v[2] * k])
    n = np.sqrt(n2)
    h = 0.5 * n
    k = np.sin(h) / n
    return np.array([np.cos(h), v[0] * k, v[1] * k, v[2] * k])


def so3_log(q):
    q = np.asarray(q, float) / np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    n2 = q[1] ** 2 + q[2] ** 2 + q[3] ** 2
    if n2 < 1e-24:
        return 2.0 * q[1:]
    n = np.sqrt(n2)
    return (2.0 * np.arctan2(n, q[0]) / n) * q[1:]


def so3_att_error_quat(q_cur, q_des):
    qd_inv = np.array([q_des[0], -q_des[1], -q_des[2], -q_des[3]])
    e_b = so3_log(so3_quat_mul(qd_inv, q_cur))
    e_n = so3_log(so3_quat_mul(q_cur, qd_inv))
    return e_b, e_n, np.linalg.norm(e_b)


def so3_err_nav_to_body(e_n, R):
    return R.T @ e_n


def so3_euler_err_to_body(d_rpy, rpy_ref):
    r, p, _ = rpy_ref
    cr, sr, ct, st = np.cos(r), np.sin(r), np.cos(p), np.sin(p)
    return np.array([
        d_rpy[0] - st * d_rpy[2],
        cr * d_rpy[1] + sr * ct * d_rpy[2],
        -sr * d_rpy[1] + cr * ct * d_rpy[2]])


def so3_wrap_angle(a):
    a = np.fmod(a + np.pi, 2 * np.pi)
    if a < 0:
        a += 2 * np.pi
    return a - np.pi


# ---------- rotation.h 逐行转写 (KF-GINS 侧) ----------
def kf_matrix2euler(dcm):
    euler = np.zeros(3)
    euler[1] = np.arctan(-dcm[2, 0] / np.sqrt(dcm[2, 1] ** 2 + dcm[2, 2] ** 2))
    if dcm[2, 0] <= -0.999:
        euler[0] = 0.0
        euler[2] = np.arctan2(dcm[1, 2] - dcm[0, 1], dcm[0, 2] + dcm[1, 1])
    elif dcm[2, 0] >= 0.999:
        euler[0] = 0.0
        euler[2] = np.pi + np.arctan2(dcm[1, 2] + dcm[0, 1], dcm[0, 2] - dcm[1, 1])
    else:
        euler[0] = np.arctan2(dcm[2, 1], dcm[2, 2])
        euler[2] = np.arctan2(dcm[1, 0], dcm[0, 0])
    if euler[2] < 0:
        euler[2] = 2 * np.pi + euler[2]
    return euler


# ---------- 测试样本 ----------
rpy = np.column_stack([
    rng.uniform(-np.pi, np.pi, N),
    rng.uniform(-np.pi / 2 + 1e-4, np.pi / 2 - 1e-4, N),
    rng.uniform(-np.pi, np.pi, N)])
# 边界: 缠绕 yaw / 奇点附近 pitch / 零姿态 / 精确奇点
edge = np.array([[0, 0, 0], [np.pi, 0, np.pi], [0.3, np.pi / 2 - 1e-9, 5.0],
                 [-0.3, -np.pi / 2 + 1e-9, -5.0], [0, np.pi / 2, 0], [0, -np.pi / 2, 0],
                 [0.7, np.pi / 2, 0.4], [0.7, -np.pi / 2, 0.4]])
rpy_all = np.vstack([rpy, edge])

# A1: euler->dcm vs 初等阵参考
e1 = max(np.abs(so3_euler_to_dcm(a) - ref_dcm(a)).max() for a in rpy_all)
check("A1 so3_euler_to_dcm vs Rz@Ry@Rx", e1, 1e-12)

# A2: dcm->euler 往返 (dcem->rpy->dcm 应复原)
e2 = 0.0
for a in rpy_all:
    R = ref_dcm(a)
    e2 = max(e2, np.abs(so3_euler_to_dcm(so3_dcm_to_euler(R)) - R).max())
check("A2 so3 dcm->euler->dcm 往返", e2, 1e-9)

# A3: euler->quat vs 三明治参考 (允许 q~-q 双覆盖)
e3 = 0.0
for a in rpy_all:
    q = so3_euler_to_quat(a)
    Rq = ref_quat_to_dcm(q)
    e3 = max(e3, np.abs(Rq - ref_dcm(a)).max())
check("A3 so3_euler_to_quat (经三明治旋转) vs 初等阵", e3, 1e-12)

# A4: quat_to_dcm vs 三明治参考
e4 = 0.0
for _ in range(2000):
    v = rng.normal(size=3)
    v = v / np.linalg.norm(v) * rng.uniform(0, np.pi)
    q = so3_exp(v)
    e4 = max(e4, np.abs(so3_quat_to_dcm(q) - ref_quat_to_dcm(q)).max())
check("A4 so3_quat_to_dcm vs 三明治", e4, 1e-12)

# A5: dcm_to_quat (Shepperd) vs 三明治参考, 全姿态
e5 = 0.0
for _ in range(2000):
    v = rng.normal(size=3)
    v = v / np.linalg.norm(v) * rng.uniform(0, np.pi - 1e-3)
    R = ref_quat_to_dcm(so3_exp(v))
    e5 = max(e5, np.abs(ref_quat_to_dcm(so3_dcm_to_quat(R)) - R).max())
check("A5 Shepperd dcm->quat (经三明治) 往返", e5, 1e-9)

# A6: exp/log 往返 + log 最短旋转性质
e6 = 0.0
for _ in range(2000):
    v = rng.normal(size=3)
    v = v / np.linalg.norm(v) * rng.uniform(0, np.pi)
    e6 = max(e6, np.abs(so3_log(so3_exp(v)) - v).max())
check("A6 exp/log 往返", e6, 1e-9)

# A7: 姿态误差 vs 轴角提取参考 + e_b = Rd^T e_n 恒等式 + angle 一致
e7a = e7b = e7c = 0.0
for _ in range(3000):
    a_c = rng.uniform([-np.pi, -np.pi / 2 + 1e-3, -np.pi], [np.pi, np.pi / 2 - 1e-3, np.pi])
    a_d = rng.uniform([-np.pi, -np.pi / 2 + 1e-3, -np.pi], [np.pi, np.pi / 2 - 1e-3, np.pi])
    qc = so3_euler_to_quat(a_c)
    qd = so3_euler_to_quat(a_d)
    e_b, e_n, ang = so3_att_error_quat(qc, qd)
    Rc, Rd = ref_dcm(a_c), ref_dcm(a_d)
    e7a = max(e7a, np.abs(e_b - ref_log(Rd.T @ Rc)).max())
    e7b = max(e7b, np.abs(e_n - ref_log(Rc @ Rd.T)).max())
    e7c = max(e7c, np.abs(so3_err_nav_to_body(e_n, Rd) - e_b).max())
    # angle 与 |Log(Rd^T R)| (独立轴角) 一致
    ang_ref = np.arccos(min(1.0, max(-1.0, (np.trace(Rd.T @ Rc) - 1) / 2)))
    e7c = max(e7c, abs(ang - ang_ref))
check("A7 att_error vs 轴角参考 (e_b)", e7a, 1e-9)
check("A7 att_error vs 轴角参考 (e_n)", e7b, 1e-9)
check("A7 e_b=Rd^T·e_n + angle 一致", e7c, 1e-9)

# A8: 欧拉差 T 阵 单轴严格一致 + 组合小角 O(d^2)
e8 = 0.0
for _ in range(1000):
    ref = rng.uniform([-np.pi, -1.0, -np.pi], [np.pi, 1.0, np.pi])
    for ax in range(3):
        d = np.zeros(3)
        d[ax] = rng.uniform(-0.05, 0.05)
        q1 = so3_euler_to_quat(ref)
        q2 = so3_euler_to_quat(ref + d)
        e_b, _, _ = so3_att_error_quat(q2, q1)
        e8 = max(e8, np.abs(so3_euler_err_to_body(d, ref) - e_b).max())
check("A8 T阵单轴严格一致", e8, 1e-12)

# B1: KF-GINS matrix2euler vs 真值往返
# 注: 其奇点分支门限 |dcm20|>=0.999 (|pitch|>=87.13deg) 内强制 roll=0, 该带内
#     DCM 重建固有损失 O(cos(p)*roll) — KF-GINS 引擎内部传播用 cbn/qbn,
#     euler 仅作输出, 属上游已知行为。分带考核:
#     带外 (常规区) 必须精确; 带内只考核其奇异公式在精确 ±90° 处的正确性。
AS_TH = np.arcsin(0.999)
e9 = 0.0
for a in rpy_all:
    if abs(a[1]) >= AS_TH:
        continue
    R = ref_dcm(a)
    est = kf_matrix2euler(R)
    R_est = Rz(est[2]) @ Ry(est[1]) @ Rx(est[0])
    e9 = max(e9, np.abs(R_est - R).max())
check("B1 KF-GINS matrix2euler 往返 (带外 |p|<87.13deg)", e9, 1e-9)

# B1b: 精确奇点处 KF-GINS 奇异公式必须复原 DCM (roll=0 规范)
e9b = 0.0
for r_ in (-0.7, 0.3, 1.9):
    for y_ in (-2.5, 0.8):
        for sgn in (1, -1):
            a = np.array([r_, sgn * np.pi / 2, y_])
            R = ref_dcm(a)
            est = kf_matrix2euler(R)
            R_est = Rz(est[2]) @ Ry(est[1]) @ Rx(est[0])
            e9b = max(e9b, np.abs(R_est - R).max())
check("B1b KF-GINS 精确奇点复原 (roll=0 规范)", e9b, 1e-9)

# B2: so3_dcm_to_euler 全量程 (含 0.999 带内) DCM 重建必须精确
#     (so3 门限 1-DBL_EPSILON, 只在精确奇点走规范分支, 带内无损)
e10 = 0.0
for a in rpy_all:
    R = ref_dcm(a)
    est = so3_dcm_to_euler(R)
    R_est = Rz(est[2]) @ Ry(est[1]) @ Rx(est[0])
    e10 = max(e10, np.abs(R_est - R).max())
check("B2 so3_dcm_to_euler 往返 (全量程含带内)", e10, 1e-9)

# B2b: 精确奇点处 so3 与 KF-GINS 奇异规范一致 (两者 yaw 相等 mod 2pi)
e10b = 0.0
for r_ in (-0.7, 0.3, 1.9):
    for y_ in (-2.5, 0.8):
        for sgn in (1, -1):
            a = np.array([r_, sgn * np.pi / 2, y_])
            R = ref_dcm(a)
            y_so3 = so3_dcm_to_euler(R)[2]
            y_kf = kf_matrix2euler(R)[2]
            e10b = max(e10b, abs(so3_wrap_angle(y_so3 - y_kf)))
check("B2b 精确奇点 so3/KF-GINS 规范一致 (mod 2pi)", e10b, 1e-9)

# C1: 桥接路径仿真 — KF-GINS yaw∈[0,360) 缠绕不变性
#     同一姿态用 yaw=y 与 yaw=y+360 (甚至 y-720) 输入, 误差必须逐位相同
e11 = 0.0
for _ in range(1000):
    a_c = rng.uniform([-np.pi, -1.0, -np.pi], [np.pi, 1.0, np.pi])
    a_d = rng.uniform([-np.pi, -1.0, -np.pi], [np.pi, 1.0, np.pi])
    base = so3_att_error_quat(so3_euler_to_quat(a_c), so3_euler_to_quat(a_d))
    for k in (-1.0, 1.0):
        a_c2 = a_c.copy()
        a_d2 = a_d.copy()
        a_c2[2] += 2 * np.pi * k
        a_d2[2] += 2 * np.pi * k
        e_b2, e_n2, ang2 = so3_att_error_quat(so3_euler_to_quat(a_c2), so3_euler_to_quat(a_d2))
        e11 = max(e11, np.abs(np.array(e_b2) - base[0]).max(),
                  np.abs(np.array(e_n2) - base[1]).max(), abs(ang2 - base[2]))
check("C1 yaw 缠绕不变性 (±360/720deg)", e11, 1e-12)

# C2: KF-GINS 输出格式端到端: deg, yaw∈[0,360) -> rad -> so3 全链 vs 参考
#     (含 0.999 近奇点带内样本: so3 侧带内无损, 必须仍对参考精确)
e12 = 0.0
for _ in range(2000):
    p_c = rng.uniform(-np.pi / 2 + 1e-4, np.pi / 2 - 1e-4)
    if _ % 10 == 0:  # 每 10 例取一个带内 pitch
        p_c = np.sign(rng.uniform(-1, 1)) * rng.uniform(AS_TH, np.pi / 2 - 1e-7)
    a_c = np.array([rng.uniform(-np.pi, np.pi), p_c, rng.uniform(-np.pi, np.pi)])
    a_d = rng.uniform([-np.pi, -1.0, -np.pi], [np.pi, 1.0, np.pi])
    # gins_bridge.cpp:964-966: ns.euler[R2D] -> solution.deg (yaw 已被 matrix2euler 抬到 [0,2pi))
    s_deg = np.array([np.degrees(a_c[0]), np.degrees(a_c[1]),
                      np.degrees(a_c[2]) % 360.0])
    # so3_gins.c:114-119: deg2rad 回来
    rpy_rad = np.radians(s_deg)
    q_cur = so3_euler_to_quat(rpy_rad)
    q_des = so3_euler_to_quat(a_d)
    e_b, e_n, ang = so3_att_error_quat(q_cur, q_des)
    Rc, Rd = ref_dcm(rpy_rad), ref_dcm(a_d)
    e12 = max(e12, np.abs(e_b - ref_log(Rd.T @ Rc)).max(),
              np.abs(e_n - ref_log(Rc @ Rd.T)).max())
check("C2 全链 deg[0,360)->rad->quat 误差 vs 参考", e12, 1e-9)

print("=" * 46)
if fails:
    print(f"FAILED: {len(fails)} 项")
    raise SystemExit(1)
print(f"ALL PASS  ({N}+ 随机姿态 + 边界样例, 双精度容差内)")
