/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 控制分配核心实现 (公式推导见 mixer.h 头注)
 *
 * 数值交叉验证: build_host/mpc_xcheck.py (正逆映射往返一致 / 饱和界 /
 * 非有限输入拒绝)。 参考: ref/FMT-Firmware 控制器模型把混控输出归一化
 * 到 [0,1] 通道值的约定 (src/hal/actuator)。
 */
#include <math.h>
#include <string.h>

#include "mixer.h"

const struct mixer_cfg mixer_cfg_default =
{
    0.11,                               /* arm_l_m: 220mm 轴距量级 (占位) */
    0.015,                              /* tau_coeff c_q (占位) */
    { 2.0e-3, 2.0e-3, 3.5e-3 },         /* inertia kg·m² (占位) */
    0.2,                                /* f_min_n (占位: 起转推力量级) */
    3.0,                                /* f_max_n (占位: 500g 级单桨静推力) */
};

static int finite3(const double v[3])
{
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

int mixer_setup(struct mixer_ctx *ctx, const struct mixer_cfg *cfg)
{
    if (!(cfg->arm_l_m > 0.0) || !isfinite(cfg->arm_l_m) ||
        !(cfg->tau_coeff > 0.0) || !isfinite(cfg->tau_coeff) ||
        !(cfg->inertia[0] > 0.0) || !(cfg->inertia[1] > 0.0) ||
        !(cfg->inertia[2] > 0.0) || !finite3(cfg->inertia) ||
        !(cfg->f_min_n >= 0.0) || !(cfg->f_max_n > cfg->f_min_n) ||
        !isfinite(cfg->f_min_n) || !isfinite(cfg->f_max_n))
        return -1;

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    return 0;
}

int mixer_step(struct mixer_ctx *ctx, double thrust_n, const double alpha_b[3],
               struct mixer_out *out)
{
    const struct mixer_cfg *c = &ctx->cfg;

    memset(out, 0, sizeof(*out));

    if (!isfinite(thrust_n) || !finite3(alpha_b))
    {
        ctx->st.bad_cnt++;
        return -1;
    }

    double T = thrust_n > 0.0 ? thrust_n : 0.0;
    double kx = sqrt(2.0) / c->arm_l_m;      /* τ -> Mx */
    double my_kx = (c->inertia[0] * alpha_b[0]) * kx;
    double my_ky = (c->inertia[1] * alpha_b[1]) * kx;
    double mz = (c->inertia[2] * alpha_b[2]) / c->tau_coeff;

    double f[MIXER_MOTORS];
    f[0] = (T - my_kx + my_ky - mz) * 0.25;  /* FR */
    f[1] = (T + my_kx + my_ky + mz) * 0.25;  /* FL */
    f[2] = (T - my_kx - my_ky + mz) * 0.25;  /* RR */
    f[3] = (T + my_kx - my_ky - mz) * 0.25;  /* RL */

    double span = c->f_max_n - c->f_min_n;
    for (int i = 0; i < MIXER_MOTORS; i++)
    {
        if (f[i] < c->f_min_n)
        {
            f[i] = c->f_min_n;
            out->sat[i] = 1;
        }
        else if (f[i] > c->f_max_n)
        {
            f[i] = c->f_max_n;
            out->sat[i] = 1;
        }
        out->n_sat += out->sat[i];
        out->f_n[i] = f[i];
        out->u_norm[i] = (f[i] - c->f_min_n) / span;
    }

    ctx->st.step_cnt++;
    if (out->n_sat)
        ctx->st.sat_cnt++;

    out->valid = 1;
    return 0;
}
