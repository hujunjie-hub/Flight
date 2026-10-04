/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 位置 MPC <-> KF-GINS 桥接层 (含 FinSH 调试命令 `mpc`)
 *
 * 职责:
 *   - 经纬高 -> 本地 NED 切平面坐标 (等距圆柱近似, <1km 范围比例误差 <0.3%,
 *     控制回路足够; 高精度换算属导航域职责)
 *   - 设定点管理 (p_ref/v_ref/a_ff, 相对参考点; `mpc pos` 首次调用自动把
 *     当前位姿捕为参考点)
 *   - 对外单步入口 mpc_pos_gins_step(): 供未来控制任务按 cfg.dt 周期调用;
 *     本文件不建线程 (执行链路: IMU 1kHz -> gins 融合 -> 外环 @1/cfg.dt
 *     -> attitude_so3 内环 -> 混控, 见根 README)
 *   - 内环取用接口 mpc_pos_gins_last_accel()
 *
 * 线程约定: step/set 同一时间只允许一个执行者 (控制任务或 FinSH `mpc step`,
 * 不能并发); get_last 任意线程可用 (关中断快照)。
 */
#include <math.h>
#include <string.h>

#include "mpc_pos.h"
#include "mpc_pos_gins.h"
#include "so3.h"
#include "gins_bridge.h"
#include "param_calib.h"

#define MPC_REF_EARTH_A     6378137.0     /* WGS84 长半轴, m */

static struct mpc_pos_ctx g_mpc;

static struct
{
    struct
    {
        rt_bool_t valid;
        double lat0, lon0, alt0;      /* 参考点 (deg, deg, m) */
    } ref;
    double  p_ref[3];                 /* NED m (D 轴向下为正, 高度增 = D 减) */
    double  v_ref[3];
    double  a_ff[3];
    rt_bool_t sp_valid;
    /* 最近一拍输出快照 (关中断拷贝) */
    struct mpc_pos_out last;
    rt_bool_t last_valid;
    double  p_ned[3], v_ned[3];       /* 最近一拍本地坐标 (日志) */
} g_run;

static void out_snapshot(const struct mpc_pos_out *out, rt_bool_t valid)
{
    rt_base_t level = rt_hw_interrupt_disable();
    g_run.last = *out;
    g_run.last_valid = valid;
    rt_hw_interrupt_enable(level);
}

static void ensure_setup(void)
{
    static rt_bool_t done = RT_FALSE;
    if (!done)
    {
        mpc_pos_setup(&g_mpc, &mpc_pos_cfg_default);
        done = RT_TRUE;
    }
}

/* LLA (deg, deg, m) -> 本地 NED (m): 等距圆柱近似, 公共入口 (QGC 装配层
 * LOCAL_POSITION_NED 同用本函数, 原两处内联公式已归一) */
void mpc_pos_gins_lla_to_ned(const double ref[3], const double lla[3],
                             double ned[3])
{
    ned[0] = SO3_DEG2RAD(lla[0] - ref[0]) * MPC_REF_EARTH_A;
    ned[1] = SO3_DEG2RAD(lla[1] - ref[1]) * MPC_REF_EARTH_A
             * cos(SO3_DEG2RAD(ref[0]));
    ned[2] = -(lla[2] - ref[2]);
}

/* 经纬高 (gins_solution) -> 本地 NED (m), 相对 g_run.ref */
static void latlon_to_ned(const struct gins_solution *s, double p[3])
{
    const double ref[3] = { g_run.ref.lat0, g_run.ref.lon0, g_run.ref.alt0 };
    const double lla[3] = { s->latitude, s->longitude, s->altitude };

    mpc_pos_gins_lla_to_ned(ref, lla, p);
}

void mpc_pos_gins_set_ref_here(void)
{
    struct gins_solution s;

    ensure_setup();
    gins_bridge_get_solution(&s);
    g_run.ref.lat0 = s.latitude;
    g_run.ref.lon0 = s.longitude;
    g_run.ref.alt0 = s.altitude;
    g_run.ref.valid = (s.ready && isfinite(s.latitude) && isfinite(s.longitude)
                       && isfinite(s.altitude)) ? RT_TRUE : RT_FALSE;
}

rt_bool_t mpc_pos_gins_ref_valid(void)
{
    return g_run.ref.valid;
}

void mpc_pos_gins_set_sp(const double p_ned[3], const double v_ned[3])
{
    ensure_setup();
    if (!g_run.ref.valid)
        mpc_pos_gins_set_ref_here();
    for (int i = 0; i < 3; i++)
    {
        g_run.p_ref[i] = p_ned[i];
        g_run.v_ref[i] = v_ned ? v_ned[i] : 0.0;
    }
    g_run.sp_valid = RT_TRUE;
    mpc_pos_reset(&g_mpc);
}

void mpc_pos_gins_get_sp(double p_ned[3], double v_ned[3])
{
    for (int i = 0; i < 3; i++)
    {
        p_ned[i] = g_run.p_ref[i];
        v_ned[i] = g_run.v_ref[i];
    }
}

/*
 * 经纬高定点设定 (QGC 地面站 DO_REPOSITION 通路, FMT_README §13.5 任务#3):
 * LLA -> 相对参考点的 NED 后走 set_sp。高度基准约定: 调用方传入的 alt
 * 与本模块 GLOBAL_POSITION_INT 上报口径一致 (gins 椭球高, QGC 地图往返
 * 自洽; MSL 与椭球高之差 = 大地水准面起伏, 未修正, 台架量级 m 级)。
 * 安全钳位: 相对当前位置水平 >50m / 垂直 >10m 的设定点按界截断
 * (MPC 只限加速度不限速度, 远距设定会积累大速度, 台架阶段拒收跳点)。
 * gins 未就绪 (无法取当前位姿钳位) 返回 -RT_ERROR。
 */
rt_err_t mpc_pos_gins_set_sp_lla(double lat_deg, double lon_deg, double alt_m)
{
    struct gins_solution s;
    double p_sp[3], d[3];
    double dh, dv;

    gins_bridge_get_solution(&s);
    if (!s.ready || !isfinite(lat_deg) || !isfinite(lon_deg) || !isfinite(alt_m))
        return -RT_ERROR;

    if (!g_run.ref.valid)
        mpc_pos_gins_set_ref_here();
    if (!g_run.ref.valid)
        return -RT_ERROR;

    {
        const double ref[3] = { g_run.ref.lat0, g_run.ref.lon0,
                                g_run.ref.alt0 };
        const double sp_lla[3] = { lat_deg, lon_deg, alt_m };

        mpc_pos_gins_lla_to_ned(ref, sp_lla, p_sp);
    }

    latlon_to_ned(&s, d);                       /* 当前位置 NED */
    dh = sqrt((p_sp[0] - d[0]) * (p_sp[0] - d[0]) +
              (p_sp[1] - d[1]) * (p_sp[1] - d[1]));
    dv = p_sp[2] - d[2];
    if (dh > 50.0 || fabs(dv) > 10.0)
    {
        double h_scale = (dh > 50.0) ? 50.0 / dh : 1.0;
        double v_scale = (fabs(dv) > 10.0) ? 10.0 / fabs(dv) : 1.0;
        double k = h_scale < v_scale ? h_scale : v_scale;

        p_sp[0] = d[0] + (p_sp[0] - d[0]) * k;
        p_sp[1] = d[1] + (p_sp[1] - d[1]) * k;
        p_sp[2] = d[2] + (p_sp[2] - d[2]) * k;
    }

    mpc_pos_gins_set_sp(p_sp, RT_NULL);
    return RT_EOK;
}

/*
 * 一拍外环: 读 KF-GINS 快照 -> 本地 NED -> MPC step。
 * 未就绪 (gins 未对准 / 参考点未设 / 设定点未设) 返回 RT_FALSE。
 */
rt_bool_t mpc_pos_gins_step(struct mpc_pos_out *out)
{
    struct gins_solution s;
    double p[3], v[3];

    ensure_setup();
    gins_bridge_get_solution(&s);

    if (!s.ready || !g_run.ref.valid || !g_run.sp_valid || s.degraded)
    {
        memset(out, 0, sizeof(*out));
        out_snapshot(out, RT_FALSE);
        return RT_FALSE;
    }

    latlon_to_ned(&s, p);
    v[0] = s.vn; v[1] = s.ve; v[2] = s.vd;

    int rc = mpc_pos_step(&g_mpc, p, v, g_run.p_ref, g_run.v_ref,
                          g_run.a_ff, out);
    if (rc != MPC_POS_OK)
    {
        memset(out, 0, sizeof(*out));
        out_snapshot(out, RT_FALSE);
        return RT_FALSE;
    }

    for (int i = 0; i < 3; i++)
    {
        g_run.p_ned[i] = p[i];
        g_run.v_ned[i] = v[i];
    }
    out_snapshot(out, RT_TRUE);
    return RT_TRUE;
}

void mpc_pos_gins_get_last(struct mpc_pos_out *out)
{
    rt_base_t level = rt_hw_interrupt_disable();
    *out = g_run.last;
    rt_hw_interrupt_enable(level);
}

/* 最近一拍 step 时的当前位置 NED m (摇杆叠加层滑设定点用; 未 step 过为 0) */
void mpc_pos_gins_get_p_ned(double p_ned[3])
{
    p_ned[0] = g_run.p_ned[0];
    p_ned[1] = g_run.p_ned[1];
    p_ned[2] = g_run.p_ned[2];
}

/* 内环 (attitude_so3) 取最近一拍期望加速度; 无有效输出时返回 RT_FALSE */
rt_bool_t mpc_pos_gins_last_accel(double a_des[3])
{
    rt_base_t level;
    struct mpc_pos_out local;
    rt_bool_t valid;

    level = rt_hw_interrupt_disable();
    local = g_run.last;
    valid = g_run.last_valid;
    rt_hw_interrupt_enable(level);

    if (!valid)
        return RT_FALSE;
    for (int i = 0; i < 3; i++)
        a_des[i] = local.a_cmd[i];
    return RT_TRUE;
}

struct mpc_pos_ctx *mpc_pos_gins_ctx(void)
{
    ensure_setup();
    return &g_mpc;
}

/* ------------------------- FinSH 调试命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

#define LOG_TAG "mpc"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

static void mpc_show(void)
{
    const struct mpc_pos_cfg *c = &g_mpc.cfg;
    struct mpc_pos_out last;
    double psp[3], vsp[3];

    mpc_pos_gins_get_last(&last);
    mpc_pos_gins_get_sp(psp, vsp);

    LOG_I("=== position MPC (outer loop) ===");
    LOG_I("cfg   : N=%d dt=%.3fs (%.1fHz) sweeps<=%d tol=%.1e",
          c->N, c->dt, 1.0 / c->dt, c->max_sweeps, c->tol_grad);
    LOG_I("weight: qpos=(%.2f %.2f %.2f) qvel=(%.2f %.2f %.2f) racc=(%.2f %.2f %.2f)",
          c->q_pos[0], c->q_pos[1], c->q_pos[2],
          c->q_vel[0], c->q_vel[1], c->q_vel[2],
          c->r_acc[0], c->r_acc[1], c->r_acc[2]);
    LOG_I("bounds: umin=(%.2f %.2f %.2f) umax=(%.2f %.2f %.2f) m/s^2 (NED)",
          c->u_min[0], c->u_min[1], c->u_min[2],
          c->u_max[0], c->u_max[1], c->u_max[2]);
    if (!g_run.ref.valid)
        LOG_W("ref   : not set (`mpc ref` captures current position)");
    else
        LOG_I("ref   : %.7f %.7f alt=%.2f", g_run.ref.lat0, g_run.ref.lon0,
              g_run.ref.alt0);
    LOG_I("sp    : p=(%.2f %.2f %.2f) v=(%.2f %.2f %.2f) %s",
          psp[0], psp[1], psp[2], vsp[0], vsp[1], vsp[2],
          g_run.sp_valid ? "" : "(unset)");
    LOG_I("aff   : (%.2f %.2f %.2f) m/s^2",
          g_run.a_ff[0], g_run.a_ff[1], g_run.a_ff[2]);
    if (g_run.last_valid)
    {
        LOG_I("meas  : p=(%.2f %.2f %.2f) v=(%.3f %.3f %.3f) NED",
              g_run.p_ned[0], g_run.p_ned[1], g_run.p_ned[2],
              g_run.v_ned[0], g_run.v_ned[1], g_run.v_ned[2]);
        LOG_I("a_cmd : (%.3f %.3f %.3f) m/s^2  sweeps=%d res=%.2e cost=%.3f",
              last.a_cmd[0], last.a_cmd[1], last.a_cmd[2],
              last.st.sweeps, last.st.res_grad, last.st.cost);
        LOG_I("stat  : steps=%u stall=%u bad=%u",
              last.st.step_cnt, last.st.stall_cnt, last.st.bad_cnt);
    }
    else
        LOG_W("out   : none (gins ready / ref / sp / `mpc step`)");
}

static void mpc_set_vec(const char *key, struct mpc_pos_cfg *cfg,
                         double v[3], int argc, char **argv)
{
    if (argc < 5)
    {
        LOG_W("usage: mpc set %s <x> <y> <z>", key);
        return;
    }
    for (int i = 0; i < 3; i++)
        v[i] = calib_parse_num(argv[2 + i]);
    if (mpc_pos_setup(&g_mpc, cfg) != MPC_POS_OK)
        LOG_W("invalid cfg, setup rejected (values not applied)");
    else
        LOG_I("%s = (%.3f %.3f %.3f)", key, v[0], v[1], v[2]);
}

static void mpc(int argc, char **argv)
{
    struct mpc_pos_cfg cfg;

    ensure_setup();

    if (argc < 2)
    {
        mpc_show();
        return;
    }

    if (!rt_strcmp(argv[1], "set") && argc >= 4)
    {
        cfg = g_mpc.cfg;
        if (!rt_strcmp(argv[2], "N"))
            cfg.N = (int)calib_parse_num(argv[3]);
        else if (!rt_strcmp(argv[2], "dt"))
            cfg.dt = calib_parse_num(argv[3]);
        else if (!rt_strcmp(argv[2], "sweeps"))
            cfg.max_sweeps = (int)calib_parse_num(argv[3]);
        else if (!rt_strcmp(argv[2], "tol"))
            cfg.tol_grad = calib_parse_num(argv[3]);
        else
        {
            /* 向量型: qpos/qvel/racc/umin/umax */
            if (!rt_strcmp(argv[2], "qpos"))
            {
                mpc_set_vec("qpos", &cfg, cfg.q_pos, argc, argv);
                return;
            }
            if (!rt_strcmp(argv[2], "qvel"))
            {
                mpc_set_vec("qvel", &cfg, cfg.q_vel, argc, argv);
                return;
            }
            if (!rt_strcmp(argv[2], "racc"))
            {
                mpc_set_vec("racc", &cfg, cfg.r_acc, argc, argv);
                return;
            }
            if (!rt_strcmp(argv[2], "umin"))
            {
                mpc_set_vec("umin", &cfg, cfg.u_min, argc, argv);
                return;
            }
            if (!rt_strcmp(argv[2], "umax"))
            {
                mpc_set_vec("umax", &cfg, cfg.u_max, argc, argv);
                return;
            }
            LOG_W("unknown key: %s", argv[2]);
            return;
        }
        if (mpc_pos_setup(&g_mpc, &cfg) != MPC_POS_OK)
            LOG_W("invalid cfg, setup rejected (values not applied)");
        else
            LOG_I("%s = %s", argv[2], argv[3]);
        return;
    }

    if (!rt_strcmp(argv[1], "qpos") || !rt_strcmp(argv[1], "qvel") ||
        !rt_strcmp(argv[1], "racc") || !rt_strcmp(argv[1], "umin") ||
        !rt_strcmp(argv[1], "umax"))
    {
        /* 兼容 `mpc qpos x y z` 直写形式 */
        char key[8];
        rt_strncpy(key, argv[1], sizeof(key) - 1);
        key[sizeof(key) - 1] = '\0';
        cfg = g_mpc.cfg;
        double *dst = !rt_strcmp(key, "qpos") ? cfg.q_pos :
                      !rt_strcmp(key, "qvel") ? cfg.q_vel :
                      !rt_strcmp(key, "racc") ? cfg.r_acc :
                      !rt_strcmp(key, "umin") ? cfg.u_min : cfg.u_max;
        if (argc < 4)
        {
            LOG_W("usage: mpc %s <x> <y> <z>", key);
            return;
        }
        for (int i = 0; i < 3; i++)
            dst[i] = calib_parse_num(argv[1 + i]);
        if (mpc_pos_setup(&g_mpc, &cfg) != MPC_POS_OK)
            LOG_W("invalid cfg, setup rejected");
        else
            LOG_I("%s = (%.3f %.3f %.3f)", key, dst[0], dst[1], dst[2]);
        return;
    }

    if (!rt_strcmp(argv[1], "ref"))
    {
        mpc_pos_gins_set_ref_here();
        if (g_run.ref.valid)
        {
            LOG_I("ref captured: %.7f %.7f alt=%.2f",
                  g_run.ref.lat0, g_run.ref.lon0, g_run.ref.alt0);
            g_run.sp_valid = RT_FALSE;
        }
        else
            LOG_W("ref capture failed (gins not ready)");
        return;
    }

    if (!rt_strcmp(argv[1], "pos") || !rt_strcmp(argv[1], "vel"))
    {
        double v[3] = {0, 0, 0};
        double sp[3];
        if (argc < 4)
        {
            LOG_W("usage: mpc %s <x> <y> <z> (NED m / m/s)", argv[1]);
            return;
        }
        for (int i = 0; i < 3; i++)
            v[i] = calib_parse_num(argv[1 + i]);
        if (!rt_strcmp(argv[1], "pos"))
        {
            mpc_pos_gins_get_sp(sp, RT_NULL);
            mpc_pos_gins_set_sp(v, sp);
            LOG_I("p_sp = (%.2f %.2f %.2f) NED m", v[0], v[1], v[2]);
        }
        else
        {
            for (int i = 0; i < 3; i++)
                g_run.v_ref[i] = v[i];
            LOG_I("v_sp = (%.2f %.2f %.2f) NED m/s", v[0], v[1], v[2]);
        }
        return;
    }

    if (!rt_strcmp(argv[1], "aff"))
    {
        if (argc < 4)
        {
            LOG_W("usage: mpc aff <x> <y> <z> (m/s^2)");
            return;
        }
        for (int i = 0; i < 3; i++)
            g_run.a_ff[i] = calib_parse_num(argv[1 + i]);
        LOG_I("a_ff = (%.2f %.2f %.2f)",
              g_run.a_ff[0], g_run.a_ff[1], g_run.a_ff[2]);
        return;
    }

    if (!rt_strcmp(argv[1], "step"))
    {
        struct mpc_pos_out out;
        if (mpc_pos_gins_step(&out))
            LOG_I("a_cmd = (%.3f %.3f %.3f) m/s^2 sweeps=%d res=%.2e",
                  out.a_cmd[0], out.a_cmd[1], out.a_cmd[2],
                  out.st.sweeps, out.st.res_grad);
        else
            LOG_W("step rejected (gins ready / ref / sp, see `mpc`)");
        return;
    }

    if (!rt_strcmp(argv[1], "reset"))
    {
        mpc_pos_reset(&g_mpc);
        LOG_I("solver warm-start cleared");
        return;
    }

    if (!rt_strcmp(argv[1], "default"))
    {
        if (mpc_pos_setup(&g_mpc, &mpc_pos_cfg_default) == MPC_POS_OK)
            LOG_I("cfg restored to defaults");
        return;
    }

    LOG_W("usage: mpc [set key v|qpos/qvel/racc/umin/umax x y z|pos/vel/aff x y z|ref|step|reset|default]");
}
MSH_CMD_EXPORT(mpc, position MPC (outer loop): mpc [set ...|pos x y z|ref|step]);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */
