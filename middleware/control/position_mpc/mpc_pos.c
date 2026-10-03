/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 位置 MPC 核心实现 (纯 C; 公式与推导见 mpc_pos.h 头注)
 *
 * 预计算闭式推导 (每轴独立, 状态 [p, v], 输入 u):
 *   A^k = [1  k*dt; 0 1],  A^kB = [dt^2*(k-j-1/2); dt]  (j 为输入时刻, k 为步序)
 *   凝结阵 Gamma 行 k 列 j 的 2x2 块 = [c_kj 0; dt 0] (c_kj = dt^2 (k-j-1/2)),
 *   代入 H = Gamma'QbarGamma + Rbar / M = Gamma'QbarPhi 展开即下方闭式和式;
 *   正确性由 build_host/mpc_xcheck.py 用稠密 Phi/Gamma 独立构造对拍。
 */
#include <math.h>
#include <string.h>

#include "mpc_pos.h"

const struct mpc_pos_cfg mpc_pos_cfg_default =
{
    10,                                     /* N: 0.5s 视界 */
    0.05,                                   /* dt: 20Hz 外环 */
    { 2.0,  2.0,  4.0 },                    /* q_pos */
    { 1.2,  1.2,  1.5 },                    /* q_vel */
    { 0.30, 0.30, 0.50 },                   /* r_acc */
    { -4.0, -4.0, -3.0 },                   /* u_min (z: 上升加速上限 3 m/s^2) */
    {  4.0,  4.0,  2.0 },                   /* u_max (z: 下降加速上限 2 m/s^2) */
    60,                                     /* max_sweeps */
    1.0e-4,                                 /* tol_grad */
};

static double finite3(const double v[3])
{
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]) ? 1.0 : 0.0;
}

/* 每轴 2 状态 LQR Riccati 迭代 -> 终端权 P = [p11 p12; p12 p22] */
static void axis_riccati(double dt, double qp, double qv, double r,
                         double *p11, double *p12, double *p22)
{
    double b1 = 0.5 * dt * dt, b2 = dt;
    double a11 = qp, a12 = 0.0, a22 = qv;      /* P0 = Q */

    for (int it = 0; it < 1000; it++)
    {
        /* S = r + B'PB */
        double s = r + b1 * (a11 * b1 + a12 * b2) + b2 * (a12 * b1 + a22 * b2);
        /* B'PA = [k1, k1*dt + k2] */
        double k1 = b1 * a11 + b2 * a12;
        double k2 = b1 * a12 + b2 * a22;
        double l1 = k1, l2 = k1 * dt + k2;
        /* A'PA (A = [1 dt; 0 1]) */
        double pa11 = a11, pa12 = a11 * dt + a12;
        double pa22 = a12 * dt + a22;
        double apa11 = pa11, apa12 = pa12;
        double apa22 = dt * pa12 + pa22;
        /* P <- A'PA - (B'PA)'(B'PA)/S + Q */
        double n11 = apa11 - l1 * l1 / s + qp;
        double n12 = apa12 - l1 * l2 / s;
        double n22 = apa22 - l2 * l2 / s + qv;
        double d = fabs(n11 - a11);
        double sc;
        if (fabs(n12 - a12) > d) d = fabs(n12 - a12);
        if (fabs(n22 - a22) > d) d = fabs(n22 - a22);
        sc = fabs(n11) > fabs(n12) ? fabs(n11) : fabs(n12);
        if (fabs(n22) > sc) sc = fabs(n22);
        a11 = n11; a12 = n12; a22 = n22;
        if (d < 1.0e-13 * sc)
            break;
    }
    *p11 = a11; *p12 = a12; *p22 = a22;
}

static int cfg_valid(const struct mpc_pos_cfg *c)
{
    if (c->N < 2 || c->N > MPC_POS_N_MAX)
        return 0;
    if (!(c->dt > 0.0) || !isfinite(c->dt))
        return 0;
    if (!(c->max_sweeps >= 1) || !(c->tol_grad > 0.0))
        return 0;
    for (int i = 0; i < 3; i++)
    {
        if (!(c->q_pos[i] > 0.0) || c->q_vel[i] < 0.0 || !(c->r_acc[i] > 0.0))
            return 0;
        if (!isfinite(c->q_pos[i]) || !isfinite(c->q_vel[i]) || !isfinite(c->r_acc[i]))
            return 0;
        if (!(c->u_min[i] < 0.0) || !(c->u_max[i] > 0.0))
            return 0;
        if (!finite3(c->u_min) || !finite3(c->u_max))
            return 0;
    }
    return 1;
}

int mpc_pos_setup(struct mpc_pos_ctx *ctx, const struct mpc_pos_cfg *cfg)
{
    if (!cfg_valid(cfg))
        return MPC_POS_ECFG;

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;

    int N = cfg->N;
    double dt = cfg->dt;

    for (int ax = 0; ax < MPC_POS_AXES; ax++)
    {
        double qp = cfg->q_pos[ax], qv = cfg->q_vel[ax], r = cfg->r_acc[ax];
        double p11, p12, p22;
        axis_riccati(dt, qp, qv, r, &p11, &p12, &p22);

        /* M[j] = sum_{k=j+1..N-1} stage + terminal (k=N) */
        double *M = ctx->M[ax];
        double *H = ctx->H[ax];
        for (int j = 0; j < N; j++)
        {
            double m0 = 0.0, m1 = 0.0;
            for (int k = j + 1; k <= N - 1; k++)
            {
                double c = dt * dt * (k - j - 0.5);
                m0 += c * qp;
                m1 += c * (k * dt) * qp + dt * qv;
            }
            double t = dt * dt * (N - j - 0.5);        /* c_{N,j} */
            m0 += t * p11 + dt * p12;
            m1 += t * (N * dt * p11 + p12) + dt * (N * dt * p12 + p22);
            M[j * 2 + 0] = m0;
            M[j * 2 + 1] = m1;
        }

        /* H[j][l] 对称; stage 求和下界 k > max(j,l) */
        for (int j = 0; j < N; j++)
        {
            for (int l = j; l < N; l++)
            {
                double s = 0.0;
                for (int k = (j > l ? j : l) + 1; k <= N - 1; k++)
                {
                    double cj = dt * dt * (k - j - 0.5);
                    double cl = dt * dt * (k - l - 0.5);
                    s += cj * cl * qp + dt * dt * qv;
                }
                double tj = dt * dt * (N - j - 0.5);
                double tl = dt * dt * (N - l - 0.5);
                s += tj * tl * p11 + dt * (tj + tl) * p12 + dt * dt * p22;
                if (j == l)
                    s += r;
                H[j * N + l] = s;
                H[l * N + j] = s;
            }
        }

        for (int k = 0; k < N; k++)
        {
            ctx->lb[ax][k] = cfg->u_min[ax];
            ctx->ub[ax][k] = cfg->u_max[ax];
        }
    }
    return MPC_POS_OK;
}

void mpc_pos_reset(struct mpc_pos_ctx *ctx)
{
    memset(ctx->U, 0, sizeof(ctx->U));
    ctx->has_sol = 0;
}

/*
 * 框约束 QP: min 1/2 u'Hu + g'u, lb<=u<=ub (H 正定)。
 * 序贯坐标法 (投影 Gauss-Seidel), r 增量维护 H*u。返回实际 sweep 数,
 * *res_out 为终止时投影梯度 inf 范数 (含未收敛路径)。
 */
static int sca_solve(int n, const double *H, const double *g,
                     const double *lb, const double *ub,
                     double *U, int max_sweeps, double tol, double *res_out)
{
    double r[MPC_POS_N_MAX];
    double res = 0.0;
    int sweeps;

    for (int i = 0; i < n; i++)
    {
        double s = 0.0;
        for (int j = 0; j < n; j++)
            s += H[i * n + j] * U[j];
        r[i] = s;
    }

    for (sweeps = 1; sweeps <= max_sweeps; sweeps++)
    {
        for (int i = 0; i < n; i++)
        {
            double grad = g[i] + r[i];
            double un = U[i] - grad / H[i * n + i];
            if (un < lb[i]) un = lb[i];
            else if (un > ub[i]) un = ub[i];
            double d = un - U[i];
            if (d != 0.0)
            {
                U[i] = un;
                for (int m = 0; m < n; m++)
                    r[m] += H[m * n + i] * d;
            }
        }
        res = 0.0;
        for (int i = 0; i < n; i++)
        {
            double pg = g[i] + r[i];
            if (U[i] <= lb[i] && pg > 0.0) pg = 0.0;
            else if (U[i] >= ub[i] && pg < 0.0) pg = 0.0;
            if (fabs(pg) > res) res = fabs(pg);
        }
        if (res < tol)
            break;
    }
    *res_out = res;
    return sweeps > max_sweeps ? max_sweeps : sweeps;
}

int mpc_pos_step(struct mpc_pos_ctx *ctx,
                 const double p[3], const double v[3],
                 const double p_ref[3], const double v_ref[3],
                 const double a_ff[3],
                 struct mpc_pos_out *out)
{
    if (ctx->cfg.N == 0)
        return MPC_POS_ENOTSETUP;

    if (!finite3(p) || !finite3(v) || !finite3(p_ref) || !finite3(v_ref) ||
        (a_ff && !finite3(a_ff)))
    {
        ctx->st.bad_cnt++;
        memset(out, 0, sizeof(*out));
        out->st = ctx->st;
        return MPC_POS_EINPUT;
    }

    int N = ctx->cfg.N;
    double g[MPC_POS_N_MAX];
    double cost = 0.0, res_max = 0.0;
    int sweeps_max = 0, stalled = 0;

    for (int ax = 0; ax < MPC_POS_AXES; ax++)
    {
        double ep = p[ax] - p_ref[ax];
        double ev = v[ax] - v_ref[ax];
        const double *H = ctx->H[ax];
        const double *M = ctx->M[ax];
        double *U = ctx->U[ax];

        for (int j = 0; j < N; j++)
            g[j] = M[j * 2 + 0] * ep + M[j * 2 + 1] * ev;

        if (ctx->has_sol)
        {
            /* 热启动: 左移一步, 末位重复; 界可能已变, 先 clamp 保证可行 */
            for (int j = 0; j < N - 1; j++)
            {
                U[j] = U[j + 1];
                if (U[j] < ctx->lb[ax][j]) U[j] = ctx->lb[ax][j];
                else if (U[j] > ctx->ub[ax][j]) U[j] = ctx->ub[ax][j];
            }
        }

        double res = 0.0;
        int sweeps = sca_solve(N, H, g, ctx->lb[ax], ctx->ub[ax], U,
                               ctx->cfg.max_sweeps, ctx->cfg.tol_grad, &res);
        if (sweeps > sweeps_max) sweeps_max = sweeps;
        if (res > res_max) res_max = res;
        if (res >= ctx->cfg.tol_grad) stalled = 1;

        /* J = 1/2 U'HU + g'U */
        double hu = 0.0, gu = 0.0;
        for (int j = 0; j < N; j++)
        {
            double s = 0.0;
            for (int k = 0; k < N; k++)
                s += H[j * N + k] * U[k];
            hu += U[j] * s;
            gu += g[j] * U[j];
        }
        cost += 0.5 * hu + gu;

        double u0 = U[0] + (a_ff ? a_ff[ax] : 0.0);
        if (u0 < ctx->cfg.u_min[ax]) u0 = ctx->cfg.u_min[ax];
        else if (u0 > ctx->cfg.u_max[ax]) u0 = ctx->cfg.u_max[ax];
        out->a_cmd[ax] = u0;
        for (int j = 0; j < N; j++)
            out->u_pred[j * 3 + ax] = U[j];
    }

    ctx->has_sol = 1;
    ctx->st.step_cnt++;
    ctx->st.sweeps = sweeps_max;
    ctx->st.res_grad = res_max;
    ctx->st.cost = cost;
    if (stalled)
        ctx->st.stall_cnt++;

    out->st = ctx->st;
    return MPC_POS_OK;
}
