/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 内环姿态控制器 <-> KF-GINS/IMU 桥接层 (含 FinSH 调试命令 `att`)
 *
 * 数据流 (一拍):
 *   a_des      <- mpc_pos_gins_last_accel()  外环输出快照 (无效 -> 0, 悬停)
 *   q_cur      <- so3_current_quat()         KF-GINS 融合姿态
 *   omega_b    <- imu_data_peek_latest()     ADIS 陀螺 (FRD rad/s)
 *   dt         <- 相邻样本 T_event 差 (us); 跳变/超窗 (>10ms 或 <=0) 退回 2ms
 *   -> att_pid_step()
 *   -> q_des 发布到 so3_target (`so3` 命令可直接观察 e_b/e_n)
 *
 * 本文件不建线程: 未来控制任务按 内环周期(建议 500Hz) 调 att_pid_gins_step,
 * 外环按 1/cfg.dt 调 mpc_pos_gins_step (见根 README "飞行控制" 章)。
 * step/set 同一时间只允许一个执行者; get_last 任意线程可用。
 */
#include <math.h>
#include <string.h>

#include "att_pid.h"
#include "att_pid_gins.h"
#include "so3_gins.h"
#include "mpc_pos_gins.h"
#include "imu_data.h"
#include "param_calib.h"

#define ATT_DT_NOMINAL_S    0.002      /* 标称内环周期 500Hz */
#define ATT_DT_MAX_S        0.010      /* T_event 差超过此值视为不可信 */

static struct att_pid_ctx g_att;

static struct
{
    double yaw_sp_deg;
    /* 最近一拍输出快照 (关中断拷贝) */
    struct att_pid_out last;
    rt_bool_t last_valid;
    rt_uint64_t t_prev_us;
    rt_bool_t has_prev;
} g_run;

static void ensure_setup(void)
{
    static rt_bool_t done = RT_FALSE;
    if (!done)
    {
        g_att.cfg = att_pid_cfg_default;
        done = RT_TRUE;
    }
}

static void out_snapshot(const struct att_pid_out *out, rt_bool_t valid)
{
    rt_base_t level = rt_hw_interrupt_disable();
    g_run.last = *out;
    g_run.last_valid = valid;
    rt_hw_interrupt_enable(level);
}

void att_pid_gins_set_yaw_deg(double yaw_deg)
{
    g_run.yaw_sp_deg = yaw_deg;
}

double att_pid_gins_get_yaw_deg(void)
{
    return g_run.yaw_sp_deg;
}

rt_bool_t att_pid_gins_hold_yaw(void)
{
    double rpy[3];

    if (!so3_current_rpy_rad(rpy))
        return RT_FALSE;
    g_run.yaw_sp_deg = SO3_RAD2DEG(rpy[2]);
    return RT_TRUE;
}

void att_pid_gins_reset(void)
{
    att_pid_reset(&g_att);
}

rt_bool_t att_pid_gins_step(struct att_pid_out *out)
{
    struct att_pid_in in;
    struct imu_sample imu;
    rt_uint32_t seq;
    double a_des[3];

    ensure_setup();

    memset(out, 0, sizeof(*out));

    if (!so3_current_quat(&in.q_cur))
        goto reject;

    if (imu_data_peek_latest(&imu, &seq) != RT_EOK)
        goto reject;

    /* dt: 相邻 IMU 样本 T_event 差 (DR 硬件时戳, 与线程抖动无关) */
    double dt = ATT_DT_NOMINAL_S;
    if (g_run.has_prev && imu.T_event > g_run.t_prev_us)
    {
        double d = (double)(imu.T_event - g_run.t_prev_us) * 1.0e-6;
        if (d > 0.0 && d <= ATT_DT_MAX_S)
            dt = d;
    }
    g_run.t_prev_us = imu.T_event;
    g_run.has_prev = RT_TRUE;

    if (!mpc_pos_gins_last_accel(a_des))
    {
        /* 外环未就绪: 悬停姿态目标 (a=0), 推力为重力悬停 */
        a_des[0] = a_des[1] = a_des[2] = 0.0;
    }

    for (int i = 0; i < 3; i++)
    {
        in.a_des[i] = a_des[i];
        in.omega_b[i] = imu.gyro[i];
    }
    in.yaw_des = SO3_DEG2RAD(g_run.yaw_sp_deg);
    in.dt = dt;

    if (att_pid_step(&g_att, &in, out) != 0)
        goto reject;

    /* 期望姿态发布到 so3_target, `so3` 命令可观察误差, 链路同源 */
    so3_target_set_quat(&out->q_des);

    out_snapshot(out, RT_TRUE);
    return RT_TRUE;

reject:
    att_pid_reset(&g_att);
    memset(out, 0, sizeof(*out));
    out->valid = 0;
    out_snapshot(out, RT_FALSE);
    so3_target_clear();
    return RT_FALSE;
}

void att_pid_gins_get_last(struct att_pid_out *out)
{
    rt_base_t level = rt_hw_interrupt_disable();
    *out = g_run.last;
    rt_hw_interrupt_enable(level);
}

struct att_pid_ctx *att_pid_gins_ctx(void)
{
    ensure_setup();
    return &g_att;
}

/* ------------------------- FinSH 调试命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

#define LOG_TAG "att"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

static void att_set_vec(const char *key, double v[3], int argc, char **argv)
{
    if (argc < 5)
    {
        LOG_W("usage: att set %s <x> <y> <z>", key);
        return;
    }
    for (int i = 0; i < 3; i++)
        v[i] = calib_parse_num(argv[2 + i]);
    LOG_I("%s = (%.3f %.3f %.3f)", key, v[0], v[1], v[2]);
}

static void att_show(void)
{
    const struct att_pid_cfg *c = &g_att.cfg;
    struct att_pid_out last;

    att_pid_gins_get_last(&last);

    LOG_I("=== SO3 cascade attitude PID (inner loop) ===");
    LOG_I("vehicle: mass=%.3fkg g0=%.4f yaw_sp=%.1f deg",
          c->mass, c->g0, g_run.yaw_sp_deg);
    LOG_I("angle P : katt=(%.2f %.2f %.2f) 1/s, tilt_max=%.1f deg, omega_max=%.2f rad/s",
          c->kp_att[0], c->kp_att[1], c->kp_att[2],
          SO3_RAD2DEG(c->tilt_max_rad), c->omega_max);
    LOG_I("rate PID: kp=(%.2f %.2f %.2f) ki=(%.2f %.2f %.2f) kd=(%.4f %.4f %.4f)",
          c->kp_rate[0], c->kp_rate[1], c->kp_rate[2],
          c->ki_rate[0], c->ki_rate[1], c->ki_rate[2],
          c->kd_rate[0], c->kd_rate[1], c->kd_rate[2]);
    LOG_I("limits : dcut=%.0fHz amax=(%.1f %.1f %.1f) rad/s^2 "
          "thrust=[%.1f %.1f]N ihold=%.2f rad/s",
          c->d_cutoff_hz, c->alpha_max[0], c->alpha_max[1], c->alpha_max[2],
          c->thrust_min, c->thrust_max, c->int_hold_e);
    if (g_run.last_valid)
    {
        LOG_I("des    : rpy=(%.2f %.2f %.2f) deg q=(%.4f %.4f %.4f %.4f)",
              SO3_RAD2DEG(last.rpy_des[0]), SO3_RAD2DEG(last.rpy_des[1]),
              SO3_RAD2DEG(last.rpy_des[2]),
              last.q_des.w, last.q_des.x, last.q_des.y, last.q_des.z);
        LOG_I("err    : %.3f deg e_b=(%.3f %.3f %.3f) deg",
              last.att_err_deg,
              SO3_RAD2DEG(last.e_b[0]), SO3_RAD2DEG(last.e_b[1]),
              SO3_RAD2DEG(last.e_b[2]));
        LOG_I("out    : thrust=%.2fN (%.2f norm) alpha=(%.2f %.2f %.2f) rad/s^2 "
          "wdes=(%.2f %.2f %.2f)%s",
              last.thrust_n, last.thrust_norm,
              last.alpha_b[0], last.alpha_b[1], last.alpha_b[2],
              last.omega_des[0], last.omega_des[1], last.omega_des[2],
              last.tilt_limited ? " [tilt-limited]" : "");
    }
    else
        LOG_W("out    : none (gins/imu ready? `att step`)");
}

static void att(int argc, char **argv)
{
    struct att_pid_cfg cfg;

    ensure_setup();

    if (argc < 2)
    {
        att_show();
        return;
    }

    if (!rt_strcmp(argv[1], "set") && argc >= 4)
    {
        cfg = g_att.cfg;
        if (!rt_strcmp(argv[2], "mass"))
            cfg.mass = calib_parse_num(argv[3]);
        else if (!rt_strcmp(argv[2], "g0"))
            cfg.g0 = calib_parse_num(argv[3]);
        else if (!rt_strcmp(argv[2], "dcut"))
            cfg.d_cutoff_hz = calib_parse_num(argv[3]);
        else if (!rt_strcmp(argv[2], "tilt"))
            cfg.tilt_max_rad = SO3_DEG2RAD(calib_parse_num(argv[3]));
        else if (!rt_strcmp(argv[2], "omax"))
            cfg.omega_max = calib_parse_num(argv[3]);
        else if (!rt_strcmp(argv[2], "thrmin"))
            cfg.thrust_min = calib_parse_num(argv[3]);
        else if (!rt_strcmp(argv[2], "thrmax"))
            cfg.thrust_max = calib_parse_num(argv[3]);
        else if (!rt_strcmp(argv[2], "ihold"))
            cfg.int_hold_e = calib_parse_num(argv[3]);
        else if (!rt_strcmp(argv[2], "katt"))
        {
            att_set_vec("katt", cfg.kp_att, argc, argv);
            g_att.cfg = cfg;
            return;
        }
        else if (!rt_strcmp(argv[2], "kpr"))
        {
            att_set_vec("kpr", cfg.kp_rate, argc, argv);
            g_att.cfg = cfg;
            return;
        }
        else if (!rt_strcmp(argv[2], "kir"))
        {
            att_set_vec("kir", cfg.ki_rate, argc, argv);
            g_att.cfg = cfg;
            return;
        }
        else if (!rt_strcmp(argv[2], "kdr"))
        {
            att_set_vec("kdr", cfg.kd_rate, argc, argv);
            g_att.cfg = cfg;
            return;
        }
        else if (!rt_strcmp(argv[2], "amax"))
        {
            att_set_vec("amax", cfg.alpha_max, argc, argv);
            g_att.cfg = cfg;
            return;
        }
        else
        {
            LOG_W("unknown key: %s", argv[2]);
            return;
        }
        g_att.cfg = cfg;        /* att 层无预计算, 校验在 att_pid_step 输入侧 */
        LOG_I("%s = %s", argv[2], argv[3]);
        return;
    }

    if (!rt_strcmp(argv[1], "yaw") && argc >= 3)
    {
        att_pid_gins_set_yaw_deg(calib_parse_num(argv[2]));
        LOG_I("yaw_sp = %.1f deg", g_run.yaw_sp_deg);
        return;
    }

    if (!rt_strcmp(argv[1], "hold"))
    {
        if (att_pid_gins_hold_yaw())
            LOG_I("yaw_sp = current heading %.1f deg", g_run.yaw_sp_deg);
        else
            LOG_W("gins not ready");
        return;
    }

    if (!rt_strcmp(argv[1], "step"))
    {
        struct att_pid_out out;
        if (att_pid_gins_step(&out))
            LOG_I("thrust=%.2fN alpha=(%.2f %.2f %.2f) err=%.3fdeg%s",
                  out.thrust_n, out.alpha_b[0], out.alpha_b[1], out.alpha_b[2],
                  out.att_err_deg, out.tilt_limited ? " [tilt-limited]" : "");
        else
            LOG_W("step rejected (gins/imu not ready)");
        return;
    }

    if (!rt_strcmp(argv[1], "reset"))
    {
        att_pid_gins_reset();
        LOG_I("integrators / D history cleared");
        return;
    }

    if (!rt_strcmp(argv[1], "default"))
    {
        g_att.cfg = att_pid_cfg_default;
        LOG_I("cfg restored to defaults");
        return;
    }

    LOG_W("usage: att [set key v|set katt/kpr/kir/kdr/amax x y z|yaw deg|hold|step|reset|default]");
}
MSH_CMD_EXPORT(att, SO3 cascade attitude PID (inner loop): att [set ...|yaw d|hold|step]);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */
