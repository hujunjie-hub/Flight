# -*- coding: utf-8 -*-
"""
middleware/control 纯 C 核心 (mpc_pos.c / att_pid.c, 含 so3.c) 主机交叉验证

与 so3_xcheck.py 不同, 本脚本用主机 gcc (CLion 自带 MinGW) 直接编译**固件
源文件本体**, 跑确定性数值用例并打印结果; Python 侧用独立构造的参考对拍:

  1. MPC 预计算 H/M: 参考用稠密 Phi/Gamma 递推构造 + 矩阵型 Riccati 终端权
     (与 mpc_pos.c 的闭式和式/标量 Riccati 完全不同的推导路径), H/M 逐元素对拍
  2. MPC QP 解最优性: 对 C 解出的 U 直接验 KKT (框约束最优性充要条件,
     与求解器算法无关)
  3. MPC 闭环仿真: 双积分器 + MPC, 收敛到设定点且全程不越加速度界
  4. a_des -> 期望姿态: 旋转阵正交性 / 第三列==推力方向 / 倾斜限幅几何
     (限幅时锥角精确等于 tilt_max) / 航向角 / 推力大小 / 欧拉角回构
  5. SO3 误差与串级 PID: e_b 与独立四元数 Log 对拍, omega_des/alpha 限幅与
     符号, 零误差零输出, 饱和抗积分 (积分器冻结)

用法:  python build_host/mpc_xcheck.py
"""
import os
import subprocess
import sys
import tempfile

import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
GCC_CANDIDATES = [
    r"D:\CLion\bin\mingw\bin\gcc.exe",
    "gcc",
]

fails = []


def check(name, err, tol):
    status = "ok  " if err <= tol else "FAIL"
    print(f"{status} {name}: max_err={err:.3e} tol={tol:.0e}")
    if err > tol:
        fails.append((name, err, tol))


def find_gcc():
    for g in GCC_CANDIDATES:
        try:
            r = subprocess.run([g, "--version"], capture_output=True, timeout=20)
            if r.returncode == 0:
                return g
        except (OSError, subprocess.TimeoutExpired):
            continue
    raise SystemExit("no host gcc found (tried: %s)" % GCC_CANDIDATES)


# ---------------------------------------------------------------- driver (固件源码本体)
DRIVER = r"""
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "mpc_pos.h"
#include "att_pid.h"
#include "mixer.h"
#include "dshot_enc.h"

static unsigned long long ls = 20261003ULL;
static double frand(void)
{
    ls = ls * 6364136223846793005ULL + 1442695040888963407ULL;
    return (double)((ls >> 11) & ((1ULL << 53) - 1)) / (double)(1ULL << 53);
}
static double fsym(void) { return 2.0 * frand() - 1.0; }

static void print_cfg_ctx(const struct mpc_pos_ctx *ctx, const char *tag)
{
    const struct mpc_pos_cfg *c = &ctx->cfg;
    printf("%s cfg %d %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n",
           tag, c->N, c->dt,
           c->q_pos[0], c->q_pos[1], c->q_pos[2],
           c->q_vel[0], c->q_vel[1], c->q_vel[2],
           c->r_acc[0], c->r_acc[1], c->r_acc[2],
           c->u_min[0], c->u_min[1], c->u_min[2],
           c->u_max[0], c->u_max[1], c->u_max[2]);
    for (int ax = 0; ax < 3; ax++)
    {
        printf("%s H %d", tag, ax);
        for (int j = 0; j < c->N * c->N; j++)
            printf(" %.17g", ctx->H[ax][j]);
        printf("\n");
        printf("%s M %d", tag, ax);
        for (int j = 0; j < c->N * 2; j++)
            printf(" %.17g", ctx->M[ax][j]);
        printf("\n");
    }
}

int main(void)
{
    /* ---------- 1. 预计算 H/M (非整值参数, 防偶然对拍通过) ---------- */
    {
        struct mpc_pos_cfg cfg = mpc_pos_cfg_default;
        struct mpc_pos_ctx ctx;
        cfg.N = 7;
        cfg.dt = 0.043;
        double qp[3] = {1.7, 2.3, 4.1}, qv[3] = {1.1, 0.7, 1.9}, ra[3] = {0.21, 0.37, 0.52};
        for (int i = 0; i < 3; i++)
        {
            cfg.q_pos[i] = qp[i];
            cfg.q_vel[i] = qv[i];
            cfg.r_acc[i] = ra[i];
        }
        int rc = mpc_pos_setup(&ctx, &cfg);
        printf("setup1 %d\n", rc);
        print_cfg_ctx(&ctx, "C1");
    }

    /* ---------- 2. QP 随机状态 KKT (紧界, 激活约束) ---------- */
    {
        struct mpc_pos_cfg cfg = mpc_pos_cfg_default;
        struct mpc_pos_ctx ctx;
        struct mpc_pos_out out;
        cfg.N = 10;
        cfg.dt = 0.05;
        cfg.u_min[0] = cfg.u_min[1] = -1.3;
        cfg.u_max[0] = cfg.u_max[1] = 1.1;      /* 不对称界 */
        cfg.u_min[2] = -2.0;
        cfg.u_max[2] = 0.9;
        int rc = mpc_pos_setup(&ctx, &cfg);
        printf("setup2 %d\n", rc);
        print_cfg_ctx(&ctx, "C2");
        for (int t = 0; t < 60; t++)
        {
            double p[3], v[3], pr[3], vr[3];
            for (int i = 0; i < 3; i++)
            {
                p[i] = 40.0 * fsym();
                v[i] = 8.0 * fsym();
                pr[i] = 30.0 * fsym();
                vr[i] = 2.0 * fsym();
            }
            rc = mpc_pos_step(&ctx, p, v, pr, vr, NULL, &out);
            printf("QP %d rc %d x %.17g %.17g %.17g %.17g %.17g %.17g r %.17g %.17g %.17g %.17g %.17g %.17g acmd %.17g %.17g %.17g sweeps %d res %.3e\n",
                   t, rc, p[0], p[1], p[2], v[0], v[1], v[2],
                   pr[0], pr[1], pr[2], vr[0], vr[1], vr[2],
                   out.a_cmd[0], out.a_cmd[1], out.a_cmd[2],
                   out.st.sweeps, out.st.res_grad);
            for (int ax = 0; ax < 3; ax++)
            {
                printf("QP %d U %d", t, ax);
                for (int j = 0; j < cfg.N; j++)
                    printf(" %.17g", out.u_pred[j * 3 + ax]);
                printf("\n");
            }
        }
        /* NaN 输入拒绝 */
        double p[3] = {0, 0, NAN};
        double v[3] = {0, 0, 0}, pr[3] = {0, 0, 0}, vr[3] = {0, 0, 0};
        rc = mpc_pos_step(&ctx, p, v, pr, vr, NULL, &out);
        printf("QP nan rc %d acmd %.17g %.17g %.17g\n",
               rc, out.a_cmd[0], out.a_cmd[1], out.a_cmd[2]);
    }

    /* ---------- 3. 闭环仿真 (默认配置, 远设定点) ---------- */
    {
        struct mpc_pos_ctx ctx;
        struct mpc_pos_out out;
        double p[3] = {30.0, -20.0, 8.0}, v[3] = {0, 0, 0};
        double pr[3] = {0, 0, 0}, vr[3] = {0, 0, 0};
        double umaxabs = 0.0;
        mpc_pos_setup(&ctx, &mpc_pos_cfg_default);
        for (int k = 0; k < 600; k++)
        {
            mpc_pos_step(&ctx, p, v, pr, vr, NULL, &out);
            for (int i = 0; i < 3; i++)
            {
                if (fabs(out.a_cmd[i]) > umaxabs)
                    umaxabs = fabs(out.a_cmd[i]);
                p[i] += v[i] * ctx.cfg.dt + 0.5 * out.a_cmd[i] * ctx.cfg.dt * ctx.cfg.dt;
                v[i] += out.a_cmd[i] * ctx.cfg.dt;
            }
        }
        printf("CL final %.17g %.17g %.17g %.17g %.17g %.17g umax %.17g\n",
               p[0], p[1], p[2], v[0], v[1], v[2], umaxabs);
    }

    /* ---------- 4. a_des -> 期望姿态 / 推力 ---------- */
    {
        for (int t = 0; t < 205; t++)
        {
            double a[3], yaw = M_PI * fsym();
            if (t < 5)
            {
                a[0] = a[1] = a[2] = 0.0;   /* 确定性水平样本: 航向须精确 */
            }
            else
            {
                for (int i = 0; i < 3; i++)
                    a[i] = 14.0 * fsym();
            }
            struct so3_quat q;
            double rpy[3], dir[3], f;
            int tl;
            int rc = att_thrust_to_attitude(a, yaw, 30.0 * M_PI / 180.0, 9.80665,
                                            &q, rpy, dir, &tl);
            att_thrust_newton(1.4, 9.80665, a, &f);
            printf("AT %d rc %d a %.17g %.17g %.17g yaw %.17g q %.17g %.17g %.17g %.17g rpy %.17g %.17g %.17g dir %.17g %.17g %.17g tl %d f %.17g\n",
                   t, rc, a[0], a[1], a[2], yaw,
                   q.w, q.x, q.y, q.z, rpy[0], rpy[1], rpy[2],
                   dir[0], dir[1], dir[2], tl, f);
        }
    }

        /* ---------- 5. 串级 PID: 零误差 / 抗饱和 / 随机误差 ---------- */
    {
        struct att_pid_ctx ctx;
        struct att_pid_in in;
        struct att_pid_out out;

        /* ctx 必须先清零 (API 约定: 静态定义或 memset, 再赋 cfg) */
        memset(&ctx, 0, sizeof(ctx));
        ctx.cfg = att_pid_cfg_default;

        /* 5a. 零误差 -> 零输出, 悬停推力 */
        double a0[3] = {0, 0, 0};
        att_thrust_to_attitude(a0, 0.7, ctx.cfg.tilt_max_rad, ctx.cfg.g0,
                               &in.q_cur, NULL, NULL, NULL);
        in.a_des[0] = in.a_des[1] = in.a_des[2] = 0.0;
        in.yaw_des = 0.7;
        in.omega_b[0] = in.omega_b[1] = in.omega_b[2] = 0.0;
        in.dt = 0.002;
        att_pid_step(&ctx, &in, &out);
        printf("PD zero valid %d alpha %.17g %.17g %.17g thrust %.17g norm %.17g err %.17g\n",
               out.valid, out.alpha_b[0], out.alpha_b[1], out.alpha_b[2],
               out.thrust_n, out.thrust_norm, out.att_err_deg);

        /* 5b. 恒定姿态误差 + 饱和 -> 积分冻结 (alpha 顶在限幅) */
        att_pid_reset(&ctx);
        in.q_cur.w = 1.0; in.q_cur.x = in.q_cur.y = in.q_cur.z = 0.0;
        double ah[3] = {6.0, 0.0, 0.0};   /* 倾斜 ~31.5 deg, 超 30 deg 锥 */
        in.a_des[0] = ah[0]; in.a_des[1] = ah[1]; in.a_des[2] = ah[2];
        in.yaw_des = 0.0;
        for (int k = 0; k < 800; k++)
            att_pid_step(&ctx, &in, &out);
        printf("PD aw alpha %.17g %.17g %.17g integ %.17g %.17g %.17g amax %.17g\n",
               out.alpha_b[0], out.alpha_b[1], out.alpha_b[2],
               ctx.st.integ[0], ctx.st.integ[1], ctx.st.integ[2],
               ctx.cfg.alpha_max[0]);

        /* 5c. 随机姿态/角速度: e_b 对拍 + 限幅检查 */
        for (int t = 0; t < 100; t++)
        {
            double rpy[3];
            for (int i = 0; i < 3; i++)
                rpy[i] = 0.9 * fsym();
            so3_euler_to_quat(rpy, &in.q_cur);
            for (int i = 0; i < 3; i++)
            {
                in.a_des[i] = 10.0 * fsym();
                in.omega_b[i] = 3.0 * fsym();
            }
            in.yaw_des = M_PI * fsym();
            in.dt = 0.002;
            att_pid_reset(&ctx);
            att_pid_step(&ctx, &in, &out);
            printf("PD %d valid %d q %.17g %.17g %.17g %.17g qdes %.17g %.17g %.17g %.17g om %.17g %.17g %.17g eb %.17g %.17g %.17g omdes %.17g %.17g %.17g alpha %.17g %.17g %.17g tl %d\n",
                   t, out.valid, in.q_cur.w, in.q_cur.x, in.q_cur.y, in.q_cur.z,
                   out.q_des.w, out.q_des.x, out.q_des.y, out.q_des.z,
                   in.omega_b[0], in.omega_b[1], in.omega_b[2],
                   out.e_b[0], out.e_b[1], out.e_b[2],
                   out.omega_des[0], out.omega_des[1], out.omega_des[2],
                   out.alpha_b[0], out.alpha_b[1], out.alpha_b[2], out.tilt_limited);
        }
    }
    /* ---------- 6. mixer: 往返 / 饱和 / 非有限 ---------- */
    {
        struct mixer_ctx mx;
        struct mixer_out out;
        mixer_setup(&mx, &mixer_cfg_default);
        /* 悬停: T = m*g (缺省 mass 1.0, g0 9.80665) */
        double T0 = 1.0 * 9.80665;
        double a0[3] = {0, 0, 0};
        mixer_step(&mx, T0, a0, &out);
        printf("MX hover %.17g %.17g %.17g %.17g %d\n",
               out.f_n[0], out.f_n[1], out.f_n[2], out.f_n[3], out.n_sat);
        /* 随机: T in [1,8], alpha in ±8 rad/s^2 (该缺省参数下多数不触界) */
        for (int t = 0; t < 80; t++)
        {
            double T = 1.0 + 7.0 * frand();
            double a[3];
            for (int i = 0; i < 3; i++)
                a[i] = 16.0 * fsym();
            mixer_step(&mx, T, a, &out);
            printf("MX %d T %.17g a %.17g %.17g %.17g f %.17g %.17g %.17g %.17g sat %d\n",
                   t, T, a[0], a[1], a[2],
                   out.f_n[0], out.f_n[1], out.f_n[2], out.f_n[3], out.n_sat);
        }
        /* 饱和: T 超 4*fmax -> 全触上界; 大 alpha + T=0 -> 触下界 */
        mixer_step(&mx, 20.0, a0, &out);
        printf("MX satT %.17g %.17g %.17g %.17g %d\n",
               out.f_n[0], out.f_n[1], out.f_n[2], out.f_n[3], out.n_sat);
        double abig[3] = {100, -100, 100};
        mixer_step(&mx, 0.0, abig, &out);
        printf("MX satA %.17g %.17g %.17g %.17g %d\n",
               out.f_n[0], out.f_n[1], out.f_n[2], out.f_n[3], out.n_sat);
        double anan[3] = {0, NAN, 0};
        int rc = mixer_step(&mx, 1.0, anan, &out);
        printf("MX nan rc %d u %.17g %.17g %.17g %.17g valid %d\n",
               rc, out.u_norm[0], out.u_norm[1], out.u_norm[2], out.u_norm[3],
               out.valid);
    }

    /* ---------- 7. dshot 编码黄金值 / 节流映射 ---------- */
    {
        /* 黄金值: value=2047 telem=0 -> 0xFFEE; value=48 -> 0x0606 */
        printf("DS gold %u %u %u\n",
               (unsigned)dshot_enc_frame(2047, 0),
               (unsigned)dshot_enc_frame(48, 0),
               (unsigned)dshot_enc_frame(0, 0));
        /* 节流映射端点/中点/单调性 */
        printf("DS thr %.17g %.17g %.17g %.17g %.17g\n",
               (double)dshot_enc_throttle(-0.1),
               (double)dshot_enc_throttle(0.0),
               (double)dshot_enc_throttle(0.5),
               (double)dshot_enc_throttle(1.0),
               (double)dshot_enc_throttle(1.5));
        /* 全值域 CRC: python 独立公式对拍 */
        for (int t = 0; t < 500; t++)
        {
            unsigned v = (unsigned)(frand() * 2048.0);
            int tel = frand() > 0.5;
            printf("DS %d v %u tel %d frame %u\n",
                   t, v, tel, (unsigned)dshot_enc_frame(v, tel));
        }
    }
    return 0;
}
"""


def build_driver(workdir):
    src = os.path.join(workdir, "_xcheck_driver.c")
    with open(src, "w", encoding="utf-8") as fh:
        fh.write(DRIVER)
    gcc = find_gcc()
    inc = [
        os.path.join(ROOT, "middleware", "control", "position_mpc"),
        os.path.join(ROOT, "middleware", "control", "attitude_so3"),
        os.path.join(ROOT, "middleware", "control", "so3"),
        os.path.join(ROOT, "middleware", "control", "control_allocation"),
        os.path.join(ROOT, "middleware", "control", "dshot_output"),
    ]
    exe = os.path.join(workdir, "_xcheck_driver.exe")
    cmd = [gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Wno-unused-parameter"]
    cmd += ["-I" + p for p in inc]
    cmd += [src,
            os.path.join(ROOT, "middleware", "control", "position_mpc", "mpc_pos.c"),
            os.path.join(ROOT, "middleware", "control", "attitude_so3", "att_pid.c"),
            os.path.join(ROOT, "middleware", "control", "so3", "so3.c"),
            os.path.join(ROOT, "middleware", "control", "control_allocation", "mixer.c"),
            os.path.join(ROOT, "middleware", "control", "dshot_output", "dshot_enc.c"),
            "-lm", "-o", exe]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    if r.returncode != 0:
        print(r.stdout)
        print(r.stderr)
        raise SystemExit("driver compile failed")
    if r.stderr.strip():
        print("[gcc warnings]\n" + r.stderr)
    return exe


# ---------------------------------------------------------------- 独立参考 (numpy)

def dense_pred_mats(N, dt, qp, qv, r):
    """稠密递推构造 Phi/Gamma/Qbar/Rbar + 矩阵型 Riccati 终端权"""
    A = np.array([[1.0, dt], [0.0, 1.0]])
    B = np.array([[0.5 * dt * dt], [dt]])
    Q = np.diag([qp, qv])

    P = Q.copy()
    for _ in range(4000):
        S = r + B.T @ P @ B
        K = np.linalg.solve(S, B.T @ P @ A)
        Pn = A.T @ P @ A - A.T @ P @ B @ K + Q
        if np.max(np.abs(Pn - P)) <= 1e-14 * max(1.0, np.max(np.abs(Pn))):
            P = Pn
            break
        P = Pn

    Apow = [np.eye(2)]
    for _ in range(N):
        Apow.append(Apow[-1] @ A)
    Phi_rows, Gam_rows = [], []
    for k in range(1, N + 1):
        Phi_rows.append(Apow[k])
        row = [Apow[k - 1 - j] @ B if j <= k - 1 else np.zeros((2, 1))
               for j in range(N)]
        Gam_rows.append(np.hstack(row))
    Phi = np.vstack(Phi_rows)
    Gam = np.vstack(Gam_rows)
    Qb = np.zeros((2 * N, 2 * N))
    for k in range(N):
        w = Q if k < N - 1 else P
        Qb[2 * k:2 * k + 2, 2 * k:2 * k + 2] = w
    Rb = np.eye(N) * r
    H = Gam.T @ Qb @ Gam + Rb
    M = Gam.T @ Qb @ Phi
    return H, M


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def qrot_dcm(q):
    """四元数 -> DCM (体->导航), 独立实现"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def qlog(q):
    """四元数 -> 旋转矢量 (最短旋转), 独立实现"""
    q = q / np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    v = q[1:]
    nv = np.linalg.norm(v)
    if nv < 1e-300:
        return np.zeros(3)
    ang = 2.0 * np.arctan2(nv, q[0])
    return v * (ang / nv)


def zyx_dcm(rpy):
    c, s = np.cos, np.sin
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, c(r), -s(r)], [0, s(r), c(r)]])
    Ry = np.array([[c(p), 0, s(p)], [0, 1, 0], [-s(p), 0, c(p)]])
    Rz = np.array([[c(y), -s(y), 0], [s(y), c(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def is_num(s):
    if s.lower() in ("nan", "inf", "-inf", "+inf", "infinity", "-infinity"):
        return False        # "nan" 是行内字面 tag, 不是数值
    try:
        float(s)
        return True
    except ValueError:
        return False


def parse(lines):
    """分桶: key = 首token (+非数值次token); 每行转浮点数组 (丢弃字面 tag)"""
    buckets = {}
    for ln in lines:
        parts = ln.split()
        if not parts:
            continue
        key = parts[0]
        if len(parts) > 1 and not is_num(parts[1]):
            key += " " + parts[1]
        if parts[0] == "QP" and len(parts) > 2 and parts[2] == "U":
            key = "QP U"
        nums = [float(p) for p in parts if is_num(p)]
        buckets.setdefault(key, []).append(nums)
    return buckets


def main():
    tmp = tempfile.mkdtemp(prefix="mpc_xcheck_")
    exe = build_driver(tmp)
    r = subprocess.run([exe], capture_output=True, text=True, timeout=120, cwd=tmp)
    if r.returncode != 0:
        print(r.stdout[-3000:])
        raise SystemExit("driver run failed")
    B = parse(r.stdout.splitlines())

    # ---- 1. H/M 对拍 ----
    for tag in ("C1", "C2"):
        c = B[f"{tag} cfg"][0]
        N = int(c[0])
        dt = c[1]
        qp = np.array(c[2:5])
        qv = np.array(c[5:8])
        ra = np.array(c[8:11])
        for ax in range(3):
            Href, Mref = dense_pred_mats(N, dt, qp[ax], qv[ax], ra[ax])
            Hc = np.array(B[f"{tag} H"][ax][1:]).reshape(N, N)
            Mc = np.array(B[f"{tag} M"][ax][1:]).reshape(N, 2)
            scale_h = max(1.0, np.max(np.abs(Href)))
            scale_m = max(1.0, np.max(np.abs(Mref)))
            check(f"{tag} H[{ax}] vs dense", np.max(np.abs(Hc - Href)) / scale_h, 1e-12)
            check(f"{tag} M[{ax}] vs dense", np.max(np.abs(Mc - Mref)) / scale_m, 5e-12)
            assert np.min(np.linalg.eigvalsh(Hc)) > 0, "H not PD"

    # ---- 2. QP KKT ----
    c = B["C2 cfg"][0]
    N = int(c[0])
    qp = np.array(c[2:5])
    qv = np.array(c[5:8])
    ra = np.array(c[8:11])
    umin = np.array(c[11:14])
    umax = np.array(c[14:17])
    Hm, Mm = {}, {}
    for ax in range(3):
        Hm[ax] = np.array(B["C2 H"][ax][1:]).reshape(N, N)
        Mm[ax] = np.array(B["C2 M"][ax][1:]).reshape(N, 2)

    # U 行按 (t, ax) 索引: nums = [t, ax, U(N)]
    Urows = {(int(r[0]), int(r[1])): np.array(r[2:]) for r in B["QP U"]}
    for r in B["QP nan"]:
        assert r[0] == -2, "NaN input must return -2"

    worst_free = worst_lb = worst_ub = worst_bound = 0.0
    n_constrained = 0
    for row in B["QP"]:
        # nums = [t, rc, x(6), xref(6), acmd(3), sweeps, res]
        t = int(row[0])
        x = row[2:8]
        xref = row[8:14]
        for ax in range(3):
            U = Urows[(t, ax)]
            g = Mm[ax] @ (np.array([x[ax], x[ax + 3]]) - np.array([xref[ax], xref[ax + 3]]))
            grad = Hm[ax] @ U + g
            scale = max(1.0, np.max(np.abs(g)))
            for j in range(N):
                if U[j] < umin[ax] - 1e-12 or U[j] > umax[ax] + 1e-12:
                    worst_bound = max(worst_bound, abs(U[j]))
                at_lb = abs(U[j] - umin[ax]) <= 1e-12
                at_ub = abs(U[j] - umax[ax]) <= 1e-12
                if at_lb and not at_ub:
                    worst_lb = max(worst_lb, -grad[j] / scale)
                    n_constrained += 1
                elif at_ub and not at_lb:
                    worst_ub = max(worst_ub, grad[j] / scale)
                    n_constrained += 1
                else:
                    worst_free = max(worst_free, abs(grad[j]) / scale)
    check("QP KKT free |grad|", worst_free, 2e-4)
    check("QP KKT lb grad>=-tol", worst_lb, 2e-4)
    check("QP KKT ub grad<=tol", worst_ub, 2e-4)
    check("QP bounds violation", worst_bound, 0.0)
    print(f"     (active-constraint coords: {n_constrained})")
    assert n_constrained > 50, "bounds too loose to exercise constraints"

    # ---- 3. 闭环 ----
    cl = B["CL final"][0]          # [p(3), v(3), umax]
    check("CL final |p|", np.max(np.abs(cl[0:3])), 0.05)
    check("CL final |v|", np.max(np.abs(cl[3:6])), 0.02)
    check("CL |u|<=umax_default(4)", max(0.0, cl[6] - 4.0 + 1e-12), 1e-9)

    # ---- 4. 姿态构造 ----
    g0 = 9.80665
    tilt_max = np.deg2rad(30.0)
    err_orth = err_z = err_tilt = err_yaw = err_rpy = 0.0
    err_ybx = err_yaw_level = 0.0
    n_level = 0
    err_f = 0.0
    n_lim = 0
    for row in B["AT"]:
        # nums = [t, rc, a(3), yaw, q(4), rpy(3), dir(3), tl, f]
        a = row[2:5]
        yaw = row[5]
        q = row[6:10]
        rpy = row[10:13]
        dirn = row[13:16]
        tl = int(row[16])
        f = row[17]

        R = qrot_dcm(q)
        err_orth = max(err_orth, np.max(np.abs(R.T @ R - np.eye(3))))
        z_exp = np.array([-a[0], -a[1], g0 - a[2]])
        f_ref = 1.4 * np.linalg.norm(z_exp)
        z_exp = z_exp / np.linalg.norm(z_exp)
        err_f = max(err_f, abs(f - f_ref))
        zhat = np.array([0.0, 0.0, 1.0])
        tilt_exp = np.arccos(np.clip(z_exp @ zhat, -1, 1))
        if tilt_exp > tilt_max:
            n_lim += 1
            got = np.arccos(np.clip(dirn @ zhat, -1, 1))
            err_tilt = max(err_tilt, abs(got - tilt_max))
            assert tl == 1, "tilt_limited flag missing"
            # 限幅方向应落在 (z_exp, ẑ) 平面内: 三矢量共面
            assert abs(np.linalg.det(np.stack([z_exp, zhat, dirn]))) < 1e-9
        else:
            assert tl == 0, "spurious tilt_limited"
            err_z = max(err_z, np.max(np.abs(dirn - z_exp)))
            err_tilt = max(err_tilt, abs(np.arccos(np.clip(dirn @ zhat, -1, 1)) - tilt_exp))
            if tilt_exp < 1e-9:
                n_level += 1
        # R 第三列 == dirn
        err_z = max(err_z, np.max(np.abs(R[:, 2] - dirn)))
        # 航向不变量 (Lee/PX4 构造的精确性质): 机体 y 轴与期望航向方向 x_c
        # 正交 —— y_b = unit(z_b x x_c) 恒成立; 欧拉 yaw 在倾斜时天然偏离
        # ψ (一阶小量, 非缺陷), 只做松异数量级校验。
        err_ybx = max(err_ybx, abs(R[0, 1] * np.cos(yaw) + R[1, 1] * np.sin(yaw)))
        assert R[0, 0] * np.cos(yaw) + R[1, 0] * np.sin(yaw) > 0, "heading flipped"
        dy = np.arctan2(R[1, 0], R[0, 0]) - yaw
        dy = (dy + np.pi) % (2 * np.pi) - np.pi
        if tl == 0 and tilt_exp < 1e-9:
            err_yaw_level = max(err_yaw_level, abs(dy))   # 无倾斜时精确等于 ψ
        err_yaw = max(err_yaw, abs(dy))
        # rpy 回构
        err_rpy = max(err_rpy, np.max(np.abs(zyx_dcm(rpy) - R)))
    check("AT orthonormal", err_orth, 1e-14)
    check("AT z-col == thrust dir (未限幅)", err_z, 1e-14)
    check("AT tilt clamp geometry", err_tilt, 1e-12)
    check("AT y_b ⟂ x_c (heading 不变量)", err_ybx, 1e-14)
    check("AT heading exact when level", err_yaw_level, 1e-12)
    check("AT heading within tilt scale", err_yaw, 0.5)
    check("AT rpy round-trip", err_rpy, 1e-14)
    check("AT thrust |f| = m|g-a|", err_f, 1e-12)
    print(f"     (tilt-limited: {n_lim}/{len(B['AT'])}, level: {n_level})")
    assert n_level >= 5, "level cases missing"

    # ---- 5. PID ----
    zd = B["PD zero"][0]           # [valid, alpha(3), thrust, norm, err_deg]
    assert zd[0] == 1
    check("PD zero-error alpha", np.max(np.abs(zd[1:4])), 1e-15)
    check("PD hover thrust norm", abs(zd[5] - 1.0), 1e-15)
    check("PD zero-error angle", zd[6], 1e-12)

    aw = B["PD aw"][0]             # [alpha(3), integ(3), amax]
    amax = aw[6]
    # 误差主要在俯仰通道: alpha 应顶在该轴限幅
    check("PD anti-windup alpha@clamp", np.max(np.abs(aw[0:3])) - amax, 1e-9)
    check("PD anti-windup integ bounded", np.max(np.abs(aw[3:6])), amax / 60.0 + 1e-6)

    err_eb = err_om = err_alpha_bound = 0.0
    omax = 4.0
    amax_v = np.array([20.0, 20.0, 12.0])
    for row in B["PD"]:
        # nums = [t, valid, q(4), qdes(4), om(3), eb(3), omdes(3), alpha(3), tl]
        q_cur = row[2:6]
        q_des = row[6:10]
        e_b = row[13:16]
        omdes = row[16:19]
        alpha = row[19:22]
        # e_b 独立参考: e_b = Log(q_des^{-1} ⊗ q_cur) (期望体系误差, Hamilton)
        q_des = q_des / np.linalg.norm(q_des)
        q_cur = q_cur / np.linalg.norm(q_cur)
        qinv = np.array([q_des[0], -q_des[1], -q_des[2], -q_des[3]])
        e_b_ref = qlog(qmul(qinv, q_cur))
        err_eb = max(err_eb, np.max(np.abs(np.array(e_b) - e_b_ref)))
        kp_att = np.array([5.0, 5.0, 3.0])
        omdes_ref = np.clip(kp_att * np.array(e_b), -omax, omax)
        err_om = max(err_om, np.max(np.abs(np.array(omdes) - omdes_ref)))
        err_alpha_bound = max(err_alpha_bound,
                              np.max(np.maximum(np.abs(np.array(alpha)) - amax_v, 0.0)))
    check("PD e_b = Log(q_des^-1 ⊗ q_cur)", err_eb, 1e-12)
    check("PD omega_des = clip(katt*e_b)", err_om, 1e-15)
    check("PD alpha within amax", err_alpha_bound, 1e-15)

    # ---- 6. mixer: 正映射重建往返 / 饱和 / 非有限 ----
    cfgd = {"arm_l_m": 0.11, "tau_coeff": 0.015,     # 与 mixer_cfg_default 手抄对拍
            "inertia": [2.0e-3, 2.0e-3, 3.5e-3], "f_min_n": 0.2, "f_max_n": 3.0}
    l, cq = cfgd["arm_l_m"], cfgd["tau_coeff"]
    J = np.array(cfgd["inertia"])
    fmin, fmax = cfgd["f_min_n"], cfgd["f_max_n"]

    def fwd(f):                        # 正映射 (mixer.h 公式独立转写)
        T = f.sum()
        tx = l / np.sqrt(2.0) * (-f[0] + f[1] - f[2] + f[3])
        ty = l / np.sqrt(2.0) * (f[0] + f[1] - f[2] - f[3])
        tz = cq * (-f[0] + f[1] + f[2] - f[3])
        return T, np.array([tx, ty, tz])

    h = B["MX hover"][0]
    check("MX hover 各电机 = T/4", np.max(np.abs(np.array(h[0:4]) - 1.0 * 9.80665 / 4)), 1e-12)
    assert h[4] == 0

    err_rt = 0.0
    n_free = 0
    for row in B["MX"]:
        if len(row) != 10:
            continue                   # hover/sat/nan 行
        T, a = row[1], np.array(row[2:5])
        f = np.array(row[5:9])
        sat = int(row[9])
        if sat == 0:                   # 未触界才有精确往返
            Tc, tau = fwd(f)
            err_rt = max(err_rt, abs(Tc - T) / max(1.0, T),
                         np.max(np.abs(tau - J * a)) / max(1.0, np.max(np.abs(J * a))))
            n_free += 1
    assert n_free > 40, "随机样本未触界过少"
    check("MX 往返 Σf=T, A·f=J·α", err_rt, 1e-12)

    st = B["MX satT"][0]
    assert st[4] == 4 and np.allclose(st[0:4], fmax), "T 超界应全触 fmax"
    sa = B["MX satA"][0]
    assert sa[4] == 4, "大 alpha 应全部触界"
    mn = B["MX nan"][0]
    assert mn[0] == -1 and mn[5] == 0, "NaN alpha 拒绝且 valid=0"

    # ---- 7. dshot 编码 ----
    g = B["DS gold"][0]
    assert g[0] == 0xFFEE and g[1] == 0x0606 and g[2] == 0x0000, \
        f"dshot 黄金值不符: {[hex(x) for x in g]}"
    th = B["DS thr"][0]
    assert th[0] == 0 and th[1] == 0, "u<=0 应为停转命令 0"
    assert th[3] == 2047 and th[4] == 2047, "u>=1 应为 2047"
    assert 1047 <= th[2] <= 1049, "u=0.5 应约 1048"

    def ref_frame(v, tel):             # 独立公式 (DShot 规范)
        packet = ((v & 0x7FF) << 1) | (1 if tel else 0)
        csum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0xF
        return (packet << 4) | csum

    err_ds = sum(abs(row[3] - ref_frame(int(row[1]), int(row[2]))) for row in B["DS"])
    check("DS 帧编码 = 规范公式", err_ds, 0)

    print()
    if fails:
        print(f"FAILED: {len(fails)} checks")
        for n, e, t in fails:
            print(f"  {n}: err={e:.3e} tol={t:.0e}")
        sys.exit(1)
    print("ALL CHECKS PASSED")


if __name__ == "__main__":
    main()
