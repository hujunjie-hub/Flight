/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 控制分配桥接层: 取 attitude_so3 最新输出 -> 混控 + FinSH `mix` 命令
 *
 * 本模块不建线程: 未来 ctl 任务按输出周期调 mixer_cmd_step_att()
 * (内环一拍一调用), 结果 u[4] 喂 dshot_output。当前台架验证走
 * FinSH `mix step` 单步。参数 (力臂/惯量/推力界) 为占位值, 硬件方案
 * 落地后经 `mix set` 整定 (见本目录 README)。
 */
#include <string.h>

#include "mixer.h"
#include "att_pid_gins.h"
#include "param_calib.h"

static struct mixer_ctx g_mix;

static void ensure_setup(void)
{
    static rt_bool_t done = RT_FALSE;
    if (!done)
    {
        mixer_setup(&g_mix, &mixer_cfg_default);
        done = RT_TRUE;
    }
}

/* 一拍分配: 内环最新 (thrust_n, alpha_b) -> 电机推力/归一化量。
 * 内环无有效输出时 out 清零 valid=0 (电机层应停转)。 */
rt_bool_t mixer_cmd_step_att(struct mixer_out *out)
{
    struct att_pid_out att;

    ensure_setup();
    att_pid_gins_get_last(&att);

    if (!att.valid)
    {
        memset(out, 0, sizeof(*out));
        return RT_FALSE;
    }

    if (mixer_step(&g_mix, att.thrust_n, att.alpha_b, out) != 0)
        return RT_FALSE;
    return RT_TRUE;
}

struct mixer_ctx *mixer_cmd_ctx(void)
{
    ensure_setup();
    return &g_mix;
}

/* ------------------------- FinSH 调试命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

#define LOG_TAG "mix"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

static void mix_show(void)
{
    const struct mixer_cfg *c = &g_mix.cfg;

    LOG_I("=== control allocation (quad-X) ===");
    LOG_I("param  : arm_l=%.3fm cq=%.4f J=(%.2e %.2e %.2e) kg·m² (占位)",
          c->arm_l_m, c->tau_coeff, c->inertia[0], c->inertia[1],
          c->inertia[2]);
    LOG_I("bounds : f=[%.2f %.2f] N/motor (占位)", c->f_min_n, c->f_max_n);
    LOG_I("layout : m0 FR(CW) m1 FL(CCW) m2 RR(CCW) m3 RL(CW)");
    LOG_I("stat   : steps=%u sat=%u bad=%u", g_mix.st.step_cnt,
          g_mix.st.sat_cnt, g_mix.st.bad_cnt);
    LOG_W("usage: mix set l|cq|j|fmin|fmax <v...> | step | default");
}

static void mix(int argc, char **argv)
{
    ensure_setup();

    if (argc < 2)
    {
        mix_show();
        return;
    }

    if (!rt_strcmp(argv[1], "set") && argc >= 4)
    {
        struct mixer_cfg cfg = g_mix.cfg;
        if (!rt_strcmp(argv[2], "l"))
            cfg.arm_l_m = calib_parse_num(argv[3]);
        else if (!rt_strcmp(argv[2], "cq"))
            cfg.tau_coeff = calib_parse_num(argv[3]);
        else if (!rt_strcmp(argv[2], "fmin"))
            cfg.f_min_n = calib_parse_num(argv[3]);
        else if (!rt_strcmp(argv[2], "fmax"))
            cfg.f_max_n = calib_parse_num(argv[3]);
        else if (!rt_strcmp(argv[2], "j") && argc >= 6)
        {
            cfg.inertia[0] = calib_parse_num(argv[3]);
            cfg.inertia[1] = calib_parse_num(argv[4]);
            cfg.inertia[2] = calib_parse_num(argv[5]);
        }
        else
        {
            LOG_W("usage: mix set l|cq|fmin|fmax <v> | j <x y z>");
            return;
        }
        if (mixer_setup(&g_mix, &cfg) != 0)
            LOG_W("invalid cfg, rejected");
        else
            LOG_I("%s updated (占位整定: 持久化未接)", argv[2]);
        return;
    }

    if (!rt_strcmp(argv[1], "step"))
    {
        struct mixer_out out;
        if (mixer_cmd_step_att(&out))
            LOG_I("f=(%.2f %.2f %.2f %.2f)N u=(%.3f %.3f %.3f %.3f)%s",
                  out.f_n[0], out.f_n[1], out.f_n[2], out.f_n[3],
                  out.u_norm[0], out.u_norm[1], out.u_norm[2],
                  out.u_norm[3], out.n_sat ? " [饱和]" : "");
        else
            LOG_W("rejected (内环无有效输出, att step 先跑)");
        return;
    }

    if (!rt_strcmp(argv[1], "default"))
    {
        mixer_setup(&g_mix, &mixer_cfg_default);
        LOG_I("cfg restored to defaults (占位)");
        return;
    }

    LOG_W("usage: mix [set ...|step|default]");
}
MSH_CMD_EXPORT(mix, control allocation: mix [set l|cq|j|fmin|fmax v|step|default]);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */
