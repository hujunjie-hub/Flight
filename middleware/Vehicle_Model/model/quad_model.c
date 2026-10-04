/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 四旋翼控制模型 — 实现见 quad_model.h 头注 (FMT 模式移植/安全约定)
 *
 * 数据流: gins 快照/IMU (mpc_pos_gins/att_pid_gins 桥接层内取数)
 *   -> 20Hz MPC 外环 -> 500Hz SO(3)/PID 内环 -> 500Hz 混控 -> 输出门控
 *   (dry-run 快照 / dshot_out_write), 与 FMT task_vehicle 的单线程分频
 *   调度同型 (PERIOD_EXECUTE3 门控的 ms 时间门控版)。
 */
#include <string.h>
#include <math.h>

#include <rtthread.h>

#include "quad_model.h"
#include "mpc_pos.h"               /* mpc_pos_reset() */
#include "mpc_pos_gins.h"          /* mpc_pos_gins_*: 外环桥接 */
#include "att_pid_gins.h"          /* att_pid_gins_*: 内环桥接 */
#include "mixer_cmd.h"             /* mixer_cmd_step_att(): 混控桥接 */
#include "dshot.h"                 /* dshot_out_*: 输出引擎 (TIM1 四路) */
#include "gins_bridge.h"           /* gins_bridge_get_solution(): 预检 */
#include "imu_data.h"              /* imu_data_wait(): 1kHz 事件源 */
#include "rc_data.h"               /* rc_data_get_latest(): 摇杆叠加/失效保护 */

/* ulog 日志: LOG_E/LOG_W/LOG_I/LOG_D, 行尾自动补 \r\n */
#define LOG_TAG "quad"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

/* 输出协议 (硬件占位期按 DShot300; dshot_hw.h 落地后随实机调) */
#define QUAD_OUT_PROTO          DSHOT_OUT_DSHOT300

/* IMU 事件唤醒的兜底超时 (ms): 1kHz DR 丢失时模型仍低速空转 */
#define QUAD_WAKE_TIMEOUT_MS    20

/* ARM 期连续无效拍上限 (500Hz x 25 = 50ms) -> 自动 disarm */
#define QUAD_BAD_STREAK_DISARM  25

/* 摇杆叠加 (POSCTL 语义, FMT_README §13.5 任务 #4): 与 mpc_pos_gins 的
 * 钳位界同源的杆速/死区参数 */
#define QUAD_STICK_DEADBAND     0.08f  /* 死区, 越界后线性重标度 */
#define QUAD_STICK_VXY_MPS      5.0   /* 满杆水平速度 m/s */
#define QUAD_STICK_VZ_MPS       2.0   /* 满杆爬升速度 m/s (上为 +) */
#define QUAD_STICK_YAW_DPS      90.0  /* 满杆航向速率 deg/s */
#define QUAD_STICK_LOOKAHEAD_S  0.5   /* 滑设定点视界 (= MPC 时域) */
#define QUAD_RC_LOST_DISARM_MS  3000  /* RC 曾在线后失联自动 disarm */

const struct quad_model_info quad_model_info = {
    .period_mpc_ms = 50,           /* 20 Hz 外环 (mpc cfg.dt 与此对齐) */
    .period_att_ms = 2,            /* 500 Hz 内环+混控+输出 */
    .info = "quad: MPC(20Hz)+SO3/PID(500Hz)+mixer",
};

/* ---------------------------- 运行状态 ---------------------------- */

static struct
{
    rt_thread_t thread;

    volatile rt_uint8_t status;    /* enum quad_model_status */
    rt_bool_t   power;             /* 输出硬件已 dshot_out_init */
    rt_bool_t   hold_done;         /* hold_here 已成功 */
    volatile rt_uint8_t reset_pending;    /* 待复位 (静默分支执行) */

    /* 分频时间戳 (ms, 无符号回绕安全: 差值比较) */
    rt_uint32_t t_mpc, t_att;

    /* 最近一拍结果与快照 (ctl 线程单写) */
    struct mpc_pos_out mpc;
    struct att_pid_out att;
    struct mixer_out   mix;
    struct quad_model_out snap;

    /* 摇杆叠加状态 (ctl 线程内, 20Hz) */
    rt_bool_t   ovl_active;      /* 任一杆越过死区 */
    rt_bool_t   yaw_int_active;  /* yaw 杆积分中 */
    double      yaw_des_deg;     /* 积分中的期望航向 */
    rt_bool_t   rc_seen_once;    /* RC/虚拟摇杆曾在线 (失联保护启用条件) */
    rt_uint32_t last_rc_ok_ms;
} s;

static void status_set(rt_uint8_t st)
{
    rt_base_t lv = rt_hw_interrupt_disable();
    s.status = st;
    s.snap.status = st;
    rt_hw_interrupt_enable(lv);
}

/* 快照发布 (ctl 线程, 关中断防读者撕裂) */
static void snap_publish(void)
{
    rt_base_t lv = rt_hw_interrupt_disable();

    s.snap.dry_run = !s.power;
    s.snap.a_cmd[0] = s.mpc.a_cmd[0];
    s.snap.a_cmd[1] = s.mpc.a_cmd[1];
    s.snap.a_cmd[2] = s.mpc.a_cmd[2];
    s.snap.thrust_n = s.att.thrust_n;
    s.snap.alpha_b[0] = s.att.alpha_b[0];
    s.snap.alpha_b[1] = s.att.alpha_b[1];
    s.snap.alpha_b[2] = s.att.alpha_b[2];
    s.snap.att_err_deg = s.att.att_err_deg;
    for (int i = 0; i < MIXER_MOTORS; i++)
        s.snap.u_norm[i] = s.mix.u_norm[i];
    s.snap.n_sat = (rt_int8_t)s.mix.n_sat;

    rt_hw_interrupt_enable(lv);
}

/* ---------------------------- 摇杆叠加 (ctl 线程 20Hz) ----------------------------
 * POSCTL 语义: 死区外杆量 = 速度指令 (滑设定点 = 当前位姿 + v*视界),
 * 全中位 = 复捕当前位置保持; ch5 低 = kill 立即 disarm; RC 曾在线后
 * 失联 >3s 自动 disarm。本函数与 mpc step 同线程 (单执行者约定天然满足)。 */

static float stick_db(float x)
{
    float a = x < 0 ? -x : x;

    if (a < QUAD_STICK_DEADBAND)
        return 0.0f;
    return x * (a - QUAD_STICK_DEADBAND) / ((1.0f - QUAD_STICK_DEADBAND) * a);
}

static void pilot_overlay(rt_uint32_t now)
{
    struct rc_data rc;
    float dp, dr, dy, du;

    rc_data_get_latest(&rc);

    if (!rc.valid)
    {
        if (s.rc_seen_once && now - s.last_rc_ok_ms > QUAD_RC_LOST_DISARM_MS)
        {
            LOG_E("rc lost %u ms while ARM, auto disarm",
                  (unsigned)(now - s.last_rc_ok_ms));
            quad_model_disarm();
        }
        return;
    }
    s.rc_seen_once = RT_TRUE;
    s.last_rc_ok_ms = now;

    if (rc.aux[0] < -0.5f)              /* kill: 真实 RC ch5 低 */
    {
        LOG_W("kill switch (ch5), disarm");
        quad_model_disarm();
        return;
    }

    dp = stick_db(rc.axis[0]);          /* pitch 前飞 + */
    dr = stick_db(rc.axis[1]);          /* roll 右飞 + */
    dy = stick_db(rc.axis[2]);          /* yaw 右转 + */
    du = stick_db(rc.axis[3]);          /* throttle 上升 + */

    if (dp == 0.0f && dr == 0.0f && dy == 0.0f && du == 0.0f)
    {
        if (s.ovl_active)
        {
            /* 杆全回中: 复捕当前位置+航向为保持目标 */
            (void)quad_model_hold_here();
            s.ovl_active = RT_FALSE;
            s.yaw_int_active = RT_FALSE;
        }
        return;
    }
    s.ovl_active = RT_TRUE;

    /* 滑设定点: 水平速度指令 + 前视; NED D 轴向下 -> 上升 = 负。
     * 杆量是机体系意图 (dp 前 / dr 右), 按当前航向 psi 旋到 NED 再交外环,
     * 否则航向非 0 时前推杆飞向正北而非机头方向。 */
    {
        struct gins_solution sol;
        double v[3];
        double p[3];
        double p_sp[3];
        double ps, cp;

        gins_bridge_get_solution(&sol);
        ps = sin(sol.yaw * (M_PI / 180.0));
        cp = cos(sol.yaw * (M_PI / 180.0));
        v[0] = ((double)dp * cp - (double)dr * ps) * QUAD_STICK_VXY_MPS;
        v[1] = ((double)dp * ps + (double)dr * cp) * QUAD_STICK_VXY_MPS;
        v[2] = -(double)du * QUAD_STICK_VZ_MPS;

        mpc_pos_gins_get_p_ned(p);
        for (int i = 0; i < 3; i++)
            p_sp[i] = p[i] + v[i] * QUAD_STICK_LOOKAHEAD_S;
        mpc_pos_gins_set_sp_soft(p_sp, v);      /* 连续重定向, 不清热启动 */
    }

    /* 航向速率积分; 回中后由下一次 hold/当前积分值保持 */
    if (dy != 0.0f)
    {
        struct gins_solution sol;

        if (!s.yaw_int_active)
        {
            gins_bridge_get_solution(&sol);
            s.yaw_des_deg = sol.yaw;
            s.yaw_int_active = RT_TRUE;
        }
        s.yaw_des_deg += (double)dy * QUAD_STICK_YAW_DPS *
                         ((double)quad_model_info.period_mpc_ms / 1000.0);
        att_pid_gins_set_yaw_deg(s.yaw_des_deg);
    }
    else
    {
        s.yaw_int_active = RT_FALSE;
    }
}

/* ---------------------------- ctl 线程 ---------------------------- */

static void quad_ctl_entry(void *parameter)
{
    RT_UNUSED(parameter);

    while (1)
    {
        rt_uint32_t now;

        imu_data_wait(QUAD_WAKE_TIMEOUT_MS);   /* 1kHz 事件, 超时兜底 */
        now = rt_tick_get_millisecond();

        if (s.status == QUAD_MODEL_ARM)
        {
            /* 外环 20Hz: a_cmd 快照供内环取用; 失败时内环自动回落
             * 悬停姿态目标 (att_pid_gins 对 a_des 无效取 0), 不打断链 */
            if (now - s.t_mpc >= quad_model_info.period_mpc_ms)
            {
                s.t_mpc = now;
                pilot_overlay(now);
                s.snap.mpc_ok = (mpc_pos_gins_step(&s.mpc) == RT_TRUE) ? 1 : 0;
                s.snap.mpc_cnt++;
            }

            /* 内环+混控+输出 500Hz */
            if (now - s.t_att >= quad_model_info.period_att_ms)
            {
                rt_bool_t ok = (att_pid_gins_step(&s.att) == RT_TRUE);

                s.snap.att_cnt++;
                if (ok)
                    ok = (mixer_cmd_step_att(&s.mix) == RT_TRUE) && s.mix.valid;

                if (ok)
                {
                    s.snap.bad_streak = 0;
                    if (s.power)
                        dshot_out_write(s.mix.u_norm);
                }
                else
                {
                    /* 无有效输出: 压停转值, 连续超限自动 disarm */
                    static const double u_stop[MIXER_MOTORS] = {0};

                    s.snap.bad_streak++;
                    s.snap.out_drop++;
                    if (s.power)
                        dshot_out_write(u_stop);
                    if (s.snap.bad_streak >= QUAD_BAD_STREAK_DISARM)
                    {
                        LOG_E("att/mix invalid x%u, auto disarm",
                              (unsigned)s.snap.bad_streak);
                        quad_model_disarm();
                    }
                }
                s.snap.att_ok = ok ? 1 : 0;
                snap_publish();
            }
        }
        else
        {
            /* 静默 (DISARM/STANDBY): 时间基压到上一周期前, arm 后立即
             * 首拍; 复位在静默上下文执行, 与 step 天然互斥 */
            if (s.reset_pending)
            {
                att_pid_gins_reset();
                mpc_pos_reset(mpc_pos_gins_ctx());
                s.reset_pending = 0;
            }
            s.t_mpc = now - quad_model_info.period_mpc_ms - 1u;
            s.t_att = now - quad_model_info.period_att_ms - 1u;
        }
    }
}

/* ---------------------------- 接口 ---------------------------- */

int quad_model_hold_here(void)
{
    struct gins_solution sol;

    gins_bridge_get_solution(&sol);
    if (!sol.ready || sol.degraded)
    {
        LOG_W("hold: gins not ready, cannot capture");
        return -RT_ERROR;
    }

    mpc_pos_gins_set_ref_here();
    if (!mpc_pos_gins_ref_valid())
    {
        LOG_W("hold: ref capture failed (non-finite pos?)");
        return -RT_ERROR;
    }

    {
        double p0[3] = {0.0, 0.0, 0.0};

        mpc_pos_gins_set_sp(p0, RT_NULL);          /* 原点零偏差 = 悬停 */
    }
    if (att_pid_gins_hold_yaw() != RT_TRUE)
    {
        LOG_W("hold: yaw capture failed");
        return -RT_ERROR;
    }

    s.hold_done = RT_TRUE;
    LOG_I("hold: origin captured, yaw=%d deg, sp=(0,0,0)", (int)sol.yaw);
    return RT_EOK;
}

int quad_model_arm(void)
{
    struct gins_solution sol;

    if (s.status == QUAD_MODEL_ARM)
        return RT_EOK;

    gins_bridge_get_solution(&sol);
    if (!sol.ready || sol.degraded)
    {
        LOG_W("arm denied: gins not ready/degraded");
        return -RT_ERROR;
    }
    if (!s.hold_done || !mpc_pos_gins_ref_valid())
    {
        LOG_W("arm denied: run `quad hold` first");
        return -RT_ERROR;
    }

    if (s.power)
    {
        if (dshot_out_arm() != 0)
        {
            LOG_E("arm denied: dshot arm failed");
            return -RT_ERROR;
        }
    }

    /* 复位控制器 (静默态, 与 ctl 线程 step 互斥) 后进入闭环 */
    att_pid_gins_reset();
    mpc_pos_reset(mpc_pos_gins_ctx());
    s.ovl_active = RT_FALSE;
    s.yaw_int_active = RT_FALSE;

    s.snap.bad_streak = 0;
    status_set(QUAD_MODEL_ARM);
    LOG_I("armed (%s)", s.power ? "output enabled" : "DRY-RUN, no output");
    return RT_EOK;
}

void quad_model_disarm(void)
{
    if (s.status == QUAD_MODEL_DISARM && !s.reset_pending)
        return;

    status_set(QUAD_MODEL_DISARM);
    if (s.power)
        dshot_out_disarm();
    s.ovl_active = RT_FALSE;
    s.yaw_int_active = RT_FALSE;
    s.reset_pending = 1;
    LOG_I("disarm: motors stopped, controllers reset pending");
}

int quad_model_power_enable(rt_bool_t on)
{
    if (on)
    {
        if (s.power)
            return RT_EOK;
        if (s.status == QUAD_MODEL_ARM)
            return -RT_EBUSY;

        if (dshot_out_init(QUAD_OUT_PROTO) != 0)
        {
            LOG_E("power on: dshot_out_init failed (TIM1/PE9-14, 见 dshot_hw.h)");
            return -RT_ERROR;
        }
        s.power = RT_TRUE;
        LOG_I("power on: output engine ready (proto DShot300, still disarmed)");
    }
    else
    {
        if (!s.power)
            return RT_EOK;

        quad_model_disarm();
        dshot_out_deinit();
        s.power = RT_FALSE;
        LOG_I("power off: output engine deinit (back to dry-run)");
    }
    return RT_EOK;
}

enum quad_model_status quad_model_get_status(void)
{
    return (enum quad_model_status)s.status;
}

void quad_model_get_out(struct quad_model_out *out)
{
    rt_base_t lv = rt_hw_interrupt_disable();

    *out = s.snap;
    rt_hw_interrupt_enable(lv);
}

int quad_model_init(void)
{
    if (s.thread != RT_NULL)
        return RT_EOK;

    memset(&s, 0, sizeof(s));

    /* CRSF 接收线程 (uart3 不存在时告警跳过, 虚拟摇杆源不受影响) */
    (void)rc_data_crsf_start();

    s.thread = rt_thread_create("ctl", quad_ctl_entry, RT_NULL,
                                QUAD_THREAD_STACK, QUAD_THREAD_PRIO,
                                QUAD_THREAD_TICK);
    if (s.thread == RT_NULL)
        return -RT_ENOMEM;
    rt_thread_startup(s.thread);

    LOG_I("quad model up: %s | att %ums (%uHz) mpc %ums (%uHz) | "
          "default DISARM + dry-run",
          quad_model_info.info,
          (unsigned)quad_model_info.period_att_ms,
          (unsigned)(1000u / quad_model_info.period_att_ms),
          (unsigned)quad_model_info.period_mpc_ms,
          (unsigned)(1000u / quad_model_info.period_mpc_ms));
    return RT_EOK;
}

/* ---------------------------- FinSH 命令 ---------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static const char *quad_status_name(rt_uint8_t st)
{
    switch (st)
    {
    case QUAD_MODEL_STANDBY:   return "STANDBY";
    case QUAD_MODEL_ARM:       return "ARM";
    default:                   return "DISARM";
    }
}

static void quad(int argc, char **argv)
{
    struct quad_model_out o;
    struct gins_solution sol;

    if (argc >= 2)
    {
        if (!rt_strcmp(argv[1], "hold"))
        {
            (void)quad_model_hold_here();
            return;
        }
        if (!rt_strcmp(argv[1], "arm"))
        {
            (void)quad_model_arm();
            return;
        }
        if (!rt_strcmp(argv[1], "disarm"))
        {
            quad_model_disarm();
            return;
        }
        if (!rt_strcmp(argv[1], "power") && argc >= 3)
        {
            if (!rt_strcmp(argv[2], "on"))
                (void)quad_model_power_enable(RT_TRUE);
            else if (!rt_strcmp(argv[2], "off"))
                (void)quad_model_power_enable(RT_FALSE);
            else
                LOG_W("usage: quad power on|off");
            return;
        }
        LOG_W("usage: quad [hold|arm|disarm|power on|off]");
        return;
    }

    quad_model_get_out(&o);
    gins_bridge_get_solution(&sol);

    LOG_I("=== quad model (%s) ===", quad_model_info.info);
    LOG_I("status  : %s%s, hold=%d", quad_status_name(o.status),
          o.dry_run ? " (dry-run)" : "", (int)s.hold_done);
    {
        struct rc_data rc;

        rc_data_get_latest(&rc);
        LOG_I("rc      : src=%d (%s), %s, ovl=%d", rc.src,
              rc.src == RC_SRC_CRSF ? "CRSF" :
              (rc.src == RC_SRC_VIRTUAL ? "virtual" : "none"),
              rc.valid ? "VALID" : "lost/none", (int)s.ovl_active);
    }
    LOG_I("gins    : %s, degraded=%d", sol.ready ? "RUNNING" : "ALIGN/WAIT",
          (int)sol.degraded);
    LOG_I("chain   : mpc_ok=%d att_ok=%d, mpc_cnt=%u att_cnt=%u drop=%u streak=%u",
          (int)o.mpc_ok, (int)o.att_ok, o.mpc_cnt, o.att_cnt,
          o.out_drop, o.bad_streak);
    LOG_I("last    : att_err=%.2fdeg thrust=%.2fN u=(%.3f %.3f %.3f %.3f)%s",
          o.att_err_deg, o.thrust_n,
          o.u_norm[0], o.u_norm[1], o.u_norm[2], o.u_norm[3],
          o.n_sat ? " [sat]" : "");
    LOG_I("hint    : `quad hold` -> `quad arm` (dry-run 观察 u_norm); "
          "`quad power on` 后输出硬件才初始化");
}
MSH_CMD_EXPORT(quad, quad control model: quad [hold|arm|disarm|power on|off]);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */
