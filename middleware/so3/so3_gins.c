/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SO(3) 姿态误差 <-> KF-GINS 桥接实现 (含 FinSH 调试命令 `so3`)
 */
#include "so3_gins.h"
#include "gins_bridge.h"

/* ------------------------- 目标姿态 (关中断快照) ------------------------- */

static struct
{
    struct so3_quat q;
    rt_bool_t valid;
} g_target;

static void target_publish(const struct so3_quat *q)
{
    rt_base_t level = rt_hw_interrupt_disable();

    g_target.q = *q;
    g_target.valid = RT_TRUE;
    rt_hw_interrupt_enable(level);
}

rt_bool_t so3_target_set_rpy_deg(double roll, double pitch, double yaw)
{
    return so3_target_set_rpy_rad(SO3_DEG2RAD(roll), SO3_DEG2RAD(pitch), SO3_DEG2RAD(yaw));
}

rt_bool_t so3_target_set_rpy_rad(double roll, double pitch, double yaw)
{
    double rpy[3];
    struct so3_quat q;

    rpy[0] = roll;
    rpy[1] = pitch;
    rpy[2] = yaw;
    so3_euler_to_quat(rpy, &q);
    target_publish(&q);

    return RT_TRUE;
}

rt_bool_t so3_target_set_quat(const struct so3_quat *q)
{
    struct so3_quat qn;

    if (q == RT_NULL)                   /* 判空须先于解引用, 反序是 UB */
        return RT_FALSE;

    qn = *q;
    so3_quat_normalize(&qn);
    target_publish(&qn);

    return RT_TRUE;
}

rt_bool_t so3_target_get(struct so3_quat *q)
{
    rt_base_t level;
    struct so3_quat local;
    rt_bool_t valid;

    level = rt_hw_interrupt_disable();
    local = g_target.q;
    valid = g_target.valid;
    rt_hw_interrupt_enable(level);

    if (q != RT_NULL && valid)
        *q = local;

    return valid;
}

void so3_target_clear(void)
{
    rt_base_t level = rt_hw_interrupt_disable();

    g_target.valid = RT_FALSE;
    rt_hw_interrupt_enable(level);
}

/* ------------------------- 当前姿态 (KF-GINS) ------------------------- */

/*
 * 测试注入口: 非 NULL 时 current_rpy_from_gins 用该快照代替真实
 * gins 桥接; 传 NULL 恢复真实桥接 (主机交叉验证见
 * build_host/so3_xcheck.py, 不经此口)。固件正常运行不调用。
 */
static const struct gins_solution *s_test_sol;

void so3_test_inject(const struct gins_solution *sol)
{
    s_test_sol = sol;
}

static rt_bool_t current_rpy_from_gins(struct gins_solution *s_out, double rpy[3])
{
    if (s_test_sol != RT_NULL)
        *s_out = *s_test_sol;
    else
        gins_bridge_get_solution(s_out);

    if (!s_out->ready)
    {
        if (rpy != RT_NULL)
            rpy[0] = rpy[1] = rpy[2] = 0.0;
        return RT_FALSE;
    }

    if (rpy != RT_NULL)
    {
        rpy[0] = SO3_DEG2RAD(s_out->roll);
        rpy[1] = SO3_DEG2RAD(s_out->pitch);
        rpy[2] = SO3_DEG2RAD(s_out->yaw);
    }
    return RT_TRUE;
}

rt_bool_t so3_current_quat(struct so3_quat *q)
{
    struct gins_solution s;
    double rpy[3];

    if (q == RT_NULL)
        return RT_FALSE;

    if (!current_rpy_from_gins(&s, rpy))
    {
        q->w = 1.0;
        q->x = q->y = q->z = 0.0;
        return RT_FALSE;
    }

    so3_euler_to_quat(rpy, q);
    return RT_TRUE;
}

rt_bool_t so3_current_rpy_rad(double rpy[3])
{
    struct gins_solution s;
    double rpy_local[3];

    if (!current_rpy_from_gins(&s, rpy_local))
    {
        if (rpy != RT_NULL)
            rpy[0] = rpy[1] = rpy[2] = 0.0;
        return RT_FALSE;
    }

    if (rpy != RT_NULL)
    {
        rpy[0] = so3_wrap_angle(rpy_local[0]);
        rpy[1] = so3_wrap_angle(rpy_local[1]);
        rpy[2] = so3_wrap_angle(rpy_local[2]);
    }
    return RT_TRUE;
}

/* ------------------------- 姿态误差 ------------------------- */

rt_bool_t so3_att_error(struct so3_att_err *out)
{
    struct so3_quat q_cur, q_des;
    rt_bool_t cur_ok, tgt_ok;

    if (out == RT_NULL)
        return RT_FALSE;

    cur_ok = so3_current_quat(&q_cur);
    tgt_ok = so3_target_get(&q_des);

    if (!cur_ok || !tgt_ok)
    {
        out->e_b[0] = out->e_b[1] = out->e_b[2] = 0.0;
        out->e_n[0] = out->e_n[1] = out->e_n[2] = 0.0;
        out->angle = 0.0;
        return RT_FALSE;
    }

    so3_att_error_quat(&q_cur, &q_des, out);
    return RT_TRUE;
}

/* ------------------------- FinSH 调试命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

#define LOG_TAG "so3"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

/* rad -> deg (ulog 已支持 %f, 不再需要整数+小数两段打印) */
static double so3_rad2deg(double rad)
{
    return SO3_RAD2DEG(rad);
}

/* 简易十进制浮点解析 (项目风格: 不引入 libc strtod/atof, 见 um982_nmea.c 注) */
static double so3_parse_num(const char *s)
{
    double v = 0.0, frac = 0.1;
    int neg = 0;

    if (s == RT_NULL)
        return 0.0;

    while (*s == ' ')
        s++;
    if (*s == '-')
    {
        neg = 1;
        s++;
    }
    else if (*s == '+')
    {
        s++;
    }

    while (*s >= '0' && *s <= '9')
        v = v * 10.0 + (*s++ - '0');

    if (*s == '.')
    {
        s++;
        while (*s >= '0' && *s <= '9')
        {
            v += (*s++ - '0') * frac;
            frac *= 0.1;
        }
    }

    return neg ? -v : v;
}

static void so3(int argc, char **argv)
{
    double rpy_cur[3], rpy_tgt[3];
    struct so3_quat q_tgt;
    struct so3_att_err err;
    rt_bool_t cur_ok, tgt_ok, err_ok;

    if (argc >= 2 && !rt_strcmp(argv[1], "target"))
    {
        if (argc >= 5)
        {
            so3_target_set_rpy_deg(so3_parse_num(argv[2]), so3_parse_num(argv[3]),
                                   so3_parse_num(argv[4]));
            LOG_I("so3 target set: roll=%s pitch=%s yaw=%s deg",
                  argv[2], argv[3], argv[4]);
        }
        else
        {
            LOG_W("usage: so3 target <roll> <pitch> <yaw> (deg)");
        }
        return;
    }

    if (argc >= 2 && !rt_strcmp(argv[1], "clear"))
    {
        so3_target_clear();
        LOG_I("so3 target cleared");
        return;
    }

    cur_ok = so3_current_rpy_rad(rpy_cur);
    tgt_ok = so3_target_get(&q_tgt);
    so3_quat_to_euler(&q_tgt, rpy_tgt);
    err_ok = so3_att_error(&err);

    LOG_I("=== SO3 attitude error (KF-GINS gins_fused_data) ===");
    if (cur_ok)
        /* 当前姿态 = gins_fused_data 链路数据 */
        LOG_I("cur    : r=%.3f p=%.3f y=%.3f deg fused_data",
              so3_rad2deg(rpy_cur[0]), so3_rad2deg(rpy_cur[1]),
              so3_rad2deg(rpy_cur[2]));
    else
        LOG_W("cur    : invalid (GINS not ready)");
    if (tgt_ok)
        LOG_I("target : r=%.3f p=%.3f y=%.3f deg",
              so3_rad2deg(rpy_tgt[0]), so3_rad2deg(rpy_tgt[1]),
              so3_rad2deg(rpy_tgt[2]));
    else
        LOG_W("target : invalid (use `so3 target r p y`)");

    if (err_ok)
    {
        LOG_I("angle  : %.3f deg", so3_rad2deg(err.angle));
        /* e_b 体轴系, 喂控制律; e_n 导航系失准角 */
        LOG_I("e_b    : (%.3f, %.3f, %.3f) (FRD)",
              so3_rad2deg(err.e_b[0]), so3_rad2deg(err.e_b[1]),
              so3_rad2deg(err.e_b[2]));
        LOG_I("e_n    : (%.3f, %.3f, %.3f) (NED)",
              so3_rad2deg(err.e_n[0]), so3_rad2deg(err.e_n[1]),
              so3_rad2deg(err.e_n[2]));
    }
    else
    {
        LOG_W("error  : invalid (cur_ok=%d tgt_ok=%d)", (int)cur_ok, (int)tgt_ok);
    }
}
MSH_CMD_EXPORT(so3, SO3 attitude error vs KF-GINS: so3 [target r p y|clear]);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */
