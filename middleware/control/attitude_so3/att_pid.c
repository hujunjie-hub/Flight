/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 内环姿态控制器实现 (SO(3) 误差串级 PID; 公式与约定见 att_pid.h 头注)
 *
 * 数值交叉验证: build_host/mpc_xcheck.py (推力方向/姿态构造闭合,
 * R_des 第三列 == z_b_des, 倾斜限幅几何, PID 符号与抗饱和性质)。
 */
#include <math.h>
#include <string.h>

#include "att_pid.h"

const struct att_pid_cfg att_pid_cfg_default =
{
    1.0,                                /* mass kg (占位, 需实测) */
    9.80665,
    { 5.0,  5.0,  3.0 },                /* kp_att 1/s */
    { 30.0, 30.0, 12.0 },               /* kp_rate 1/s */
    { 60.0, 60.0, 20.0 },               /* ki_rate 1/s^2 */
    { 0.05, 0.05, 0.03 },               /* kd_rate */
    30.0,                               /* d_cutoff_hz */
    30.0 * (M_PI / 180.0),              /* tilt_max 30 deg */
    4.0,                                /* omega_max rad/s */
    { 20.0, 20.0, 12.0 },               /* alpha_max rad/s^2 */
    0.0, 25.0,                          /* thrust [0, 25] N */
    2.0,                                /* int_hold_e rad/s */
};

static int finite3(const double v[3])
{
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static void cross(const double a[3], const double b[3], double out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static double norm3(const double v[3])
{
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

int att_thrust_newton(double mass, double g0, const double a_des[3],
                      double *f_out)
{
    double t[3] = { -a_des[0], -a_des[1], g0 - a_des[2] };
    double n = norm3(t);
    if (!finite3(a_des) || !isfinite(n) || !(n > 0.0))
        return -1;
    *f_out = mass * n;
    return 0;
}

int att_thrust_to_attitude(const double a_des[3], double yaw_des,
                           double tilt_max, double g0,
                           struct so3_quat *q_des, double rpy_des[3],
                           double thrust_dir_n[3], int *tilt_limited)
{
    if (!finite3(a_des) || !isfinite(yaw_des) || !isfinite(g0) ||
        !(tilt_max > 0.0) || tilt_max >= M_PI / 2.0)
        return -1;

    /* 推力方向 (未限幅): z_b = unit(g_vec - a_des), 悬停 = [0,0,1] */
    double z[3] = { -a_des[0], -a_des[1], g0 - a_des[2] };
    double nz = norm3(z);
    if (!(nz > 0.0))
        return -1;
    z[0] /= nz; z[1] /= nz; z[2] /= nz;

    /* 倾斜限幅: 与 [0,0,1] 夹角 > tilt_max 时投影回锥边界
     * (tan 尺度缩放水平分量, 再归一化, 锥角精确等于 tilt_max) */
    if (tilt_limited)
        *tilt_limited = 0;
    if (z[2] <= 0.0 ||
        z[0] * z[0] + z[1] * z[1] > z[2] * z[2] * tan(tilt_max) * tan(tilt_max))
    {
        if (tilt_limited)
            *tilt_limited = 1;
        double hx = z[0], hy = z[1];
        double hn = sqrt(hx * hx + hy * hy);
        if (hn < 1.0e-9)
        {
            hx = 1.0; hy = 0.0; hn = 1.0;
        }
        double t = tan(tilt_max);
        double inv = 1.0 / sqrt(t * t + 1.0);
        z[0] = hx / hn * t * inv;
        z[1] = hy / hn * t * inv;
        z[2] = inv;
    }

    /* 航向正交化: y_b = unit(z_b x x_c); x_b = y_b x z_b (右手系) */
    double xc[3] = { cos(yaw_des), sin(yaw_des), 0.0 };
    double yb[3];
    cross(z, xc, yb);
    double ny = norm3(yb);
    if (ny < 1.0e-6)
    {
        /* 退化 (z 平行 x_c): 倾斜限幅后不会发生, 兜底换参考轴 */
        double alt[3] = { 0.0, 1.0, 0.0 };
        cross(z, alt, yb);
        ny = norm3(yb);
        if (ny < 1.0e-9)
            return -1;
    }
    yb[0] /= ny; yb[1] /= ny; yb[2] /= ny;

    double xb[3];
    cross(yb, z, xb);                    /* 单位: yb,z 正交单位矢 */

    so3_dcm R;
    R[0][0] = xb[0]; R[0][1] = yb[0]; R[0][2] = z[0];
    R[1][0] = xb[1]; R[1][1] = yb[1]; R[1][2] = z[1];
    R[2][0] = xb[2]; R[2][1] = yb[2]; R[2][2] = z[2];

    so3_dcm_to_quat(R, q_des);
    if (rpy_des)
        so3_dcm_to_euler(R, rpy_des);
    if (thrust_dir_n)
    {
        thrust_dir_n[0] = z[0];
        thrust_dir_n[1] = z[1];
        thrust_dir_n[2] = z[2];
    }
    return 0;
}

void att_pid_reset(struct att_pid_ctx *ctx)
{
    memset(&ctx->st, 0, sizeof(ctx->st));
}

int att_pid_step(struct att_pid_ctx *ctx, const struct att_pid_in *in,
                 struct att_pid_out *out)
{
    const struct att_pid_cfg *c = &ctx->cfg;

    memset(out, 0, sizeof(*out));

    if (!finite3(in->a_des) || !isfinite(in->yaw_des) ||
        !isfinite(in->q_cur.w) || !isfinite(in->q_cur.x) ||
        !isfinite(in->q_cur.y) || !isfinite(in->q_cur.z) ||
        !finite3(in->omega_b) ||
        !(in->dt > 0.0) || in->dt > 1.0)
    {
        att_pid_reset(ctx);
        return -1;
    }

    /* 1) a_des -> 期望姿态 + 总推力 */
    struct so3_quat q_des;
    double rpy_des[3];
    int tilt_limited;
    if (att_thrust_to_attitude(in->a_des, in->yaw_des, c->tilt_max_rad,
                               c->g0, &q_des, rpy_des, NULL, &tilt_limited) != 0)
    {
        att_pid_reset(ctx);
        return -1;
    }

    double f;
    if (att_thrust_newton(c->mass, c->g0, in->a_des, &f) != 0)
    {
        att_pid_reset(ctx);
        return -1;
    }
    if (f < c->thrust_min) f = c->thrust_min;
    else if (f > c->thrust_max) f = c->thrust_max;

    /* 2) SO(3) 姿态误差 (期望体轴系 e_b) */
    struct so3_att_err err;
    struct so3_quat q_cur = in->q_cur;
    so3_quat_normalize(&q_cur);
    so3_att_error_quat(&q_cur, &q_des, &err);

    /* 3) 角度环 P: omega_des = Kp_att * e_b (逐轴限幅) */
    double omega_des[3];
    for (int i = 0; i < 3; i++)
    {
        double w = c->kp_att[i] * err.e_b[i];
        if (w > c->omega_max) w = c->omega_max;
        else if (w < -c->omega_max) w = -c->omega_max;
        omega_des[i] = w;
    }

    /* 4) 角速度环 PID (D 作用于测量微分 + 一阶低通; 条件积分抗饱和) */
    double alpha[3];
    double dalpha = exp(-2.0 * M_PI * c->d_cutoff_hz * in->dt);
    for (int i = 0; i < 3; i++)
    {
        double e = omega_des[i] - in->omega_b[i];
        double dw = 0.0;
        if (ctx->st.has_prev)
        {
            double dw_raw = (in->omega_b[i] - ctx->st.wprev[i]) / in->dt;
            ctx->st.dwf[i] = dalpha * ctx->st.dwf[i] + (1.0 - dalpha) * dw_raw;
            dw = ctx->st.dwf[i];
        }

        double integ = ctx->st.integ[i];
        if (fabs(e) <= c->int_hold_e)
            integ += e * in->dt;
        /* 饱和方向上不再积分 (条件积分抗饱和) */
        double cand = c->kp_rate[i] * e + c->ki_rate[i] * integ - c->kd_rate[i] * dw;
        if (cand > c->alpha_max[i] && e > 0.0)
            integ = ctx->st.integ[i];
        else if (cand < -c->alpha_max[i] && e < 0.0)
            integ = ctx->st.integ[i];
        ctx->st.integ[i] = integ;

        double a = c->kp_rate[i] * e + c->ki_rate[i] * integ - c->kd_rate[i] * dw;
        if (a > c->alpha_max[i]) a = c->alpha_max[i];
        else if (a < -c->alpha_max[i]) a = -c->alpha_max[i];
        alpha[i] = a;

        ctx->st.wprev[i] = in->omega_b[i];
    }
    ctx->st.has_prev = 1;

    out->valid = 1;
    out->thrust_n = f;
    out->thrust_norm = f / (c->mass * c->g0);
    out->q_des = q_des;
    for (int i = 0; i < 3; i++)
    {
        out->alpha_b[i] = alpha[i];
        out->omega_des[i] = omega_des[i];
        out->rpy_des[i] = rpy_des[i];
        out->e_b[i] = err.e_b[i];
    }
    out->att_err_deg = SO3_RAD2DEG(err.angle);
    out->tilt_limited = tilt_limited;
    return 0;
}
