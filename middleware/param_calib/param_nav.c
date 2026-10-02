/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 导航参数域实现: 镜像缺省 = gins_config.h 编译期值 (静态初始化, 上电
 * 即可被热路径安全读取), W25Q64 nav 分区记录覆盖, 见 param_nav.h。
 */

#include <rtthread.h>
#include <string.h>

#include "gins_config.h"
#include "param_part.h"
#include "param_nav.h"

#define LOG_TAG "nav"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

/* ------------------------- payload 布局 (小端) ------------------------- */

#define NAV_PAYLOAD_SIZE        64u

#define NOF_DECL                0       /* f32 磁偏角 deg */
#define NOF_LAT                 4       /* f64 纬度 deg */
#define NOF_LON                 12      /* f64 经度 deg */
#define NOF_ALT                 20      /* f64 椭球高 m */
#define NOF_IMU_SRC             28      /* u8 x3 */
#define NOF_IMU_SIGN            32      /* f32 x3 */
#define NOF_MAG_SRC             44      /* u8 x3 */
#define NOF_MAG_SIGN            48      /* f32 x3 */
#define NOF_MAG_ENABLE          60      /* u8 */

/* ------------------------- 镜像 ------------------------- */

/* 缺省表 = 编译期参数 (宏驱动, nav reset 的拷贝源) */
static const struct nav_params s_nav_defaults =
{
    .mag_decl_deg  = GINS_MAG_DECL_DEG,
    .nognss_lat    = GINS_NOGNSS_LAT_DEG,
    .nognss_lon    = GINS_NOGNSS_LON_DEG,
    .nognss_alt    = GINS_NOGNSS_ALT_M,

    .imu_axis_src  = GINS_AXIS_SRC,
    .imu_axis_sign = GINS_AXIS_SIGN,

    .mag_axis_src  = GINS_MAG_AXIS_SRC,
    .mag_axis_sign = GINS_MAG_AXIS_SIGN,

    .mag_enable    = GINS_MAG_ENABLE,
};

/* 工作镜像以同一组宏静态初始化: 无记录时行为与原宏完全一致; 消费方
 * (data 层轴映射/gins 播种) 从上电第一拍起即可读, 无初始化顺序依赖 */
static struct nav_params s_nav =
{
    .mag_decl_deg  = GINS_MAG_DECL_DEG,
    .nognss_lat    = GINS_NOGNSS_LAT_DEG,
    .nognss_lon    = GINS_NOGNSS_LON_DEG,
    .nognss_alt    = GINS_NOGNSS_ALT_M,

    .imu_axis_src  = GINS_AXIS_SRC,
    .imu_axis_sign = GINS_AXIS_SIGN,

    .mag_axis_src  = GINS_MAG_AXIS_SRC,
    .mag_axis_sign = GINS_MAG_AXIS_SIGN,

    .mag_enable    = GINS_MAG_ENABLE,
};

static struct
{
    rt_bool_t inited;
    rt_bool_t dirty;                /* nav set 修改未 save */
    rt_bool_t stored;               /* nav 分区存在有效记录 */
} s_navrun;

/* ------------------------- 小工具 ------------------------- */

static void put_u32(rt_uint8_t *p, rt_uint32_t v)
{
    p[0] = (rt_uint8_t)v;
    p[1] = (rt_uint8_t)(v >> 8);
    p[2] = (rt_uint8_t)(v >> 16);
    p[3] = (rt_uint8_t)(v >> 24);
}

static rt_uint32_t get_u32(const rt_uint8_t *p)
{
    return (rt_uint32_t)p[0] | ((rt_uint32_t)p[1] << 8) |
           ((rt_uint32_t)p[2] << 16) | ((rt_uint32_t)p[3] << 24);
}

static rt_uint8_t get_u8(const rt_uint8_t *p)
{
    return p[0];
}

static void put_f32(rt_uint8_t *p, float f)
{
    rt_uint32_t u;

    memcpy(&u, &f, 4);
    put_u32(p, u);
}

static float get_f32(const rt_uint8_t *p)
{
    rt_uint32_t u = get_u32(p);
    float f;

    memcpy(&f, &u, 4);
    return f;
}

static void put_f64(rt_uint8_t *p, double d)
{
    rt_uint64_t u;

    memcpy(&u, &d, 8);
    put_u32(p, (rt_uint32_t)u);
    put_u32(p + 4, (rt_uint32_t)(u >> 32));
}

static double get_f64(const rt_uint8_t *p)
{
    rt_uint64_t u = get_u32(p) | ((rt_uint64_t)get_u32(p + 4) << 32);
    double d;

    memcpy(&d, &u, 8);
    return d;
}

/* 简易十进制浮点解析 (同 calib_parse_num, 不引入 libc strtod) */
static double nav_parse_num(const char *s)
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

static long nav_parse_int(const char *s)
{
    return (long)nav_parse_num(s);
}

/* 轴重排合法性: 三个下标互异且均在 0..2 (一一重排, 不允许重复取轴) */
static rt_bool_t axis_src_valid(const rt_int8_t src[3])
{
    return (rt_bool_t)(src[0] >= 0 && src[0] <= 2 &&
                        src[1] >= 0 && src[1] <= 2 &&
                        src[2] >= 0 && src[2] <= 2 &&
                        src[0] != src[1] && src[1] != src[2] &&
                        src[0] != src[2]);
}

/* ------------------------- 序列化 ------------------------- */

static void pack_payload(rt_uint8_t *pl)
{
    memset(pl, 0, NAV_PAYLOAD_SIZE);

    put_f32(pl + NOF_DECL, s_nav.mag_decl_deg);
    put_f64(pl + NOF_LAT, s_nav.nognss_lat);
    put_f64(pl + NOF_LON, s_nav.nognss_lon);
    put_f64(pl + NOF_ALT, s_nav.nognss_alt);

    for (int i = 0; i < 3; i++)
        pl[NOF_IMU_SRC + i] = (rt_uint8_t)s_nav.imu_axis_src[i];
    for (int i = 0; i < 3; i++)
        put_f32(pl + NOF_IMU_SIGN + 4u * i, s_nav.imu_axis_sign[i]);

    for (int i = 0; i < 3; i++)
        pl[NOF_MAG_SRC + i] = (rt_uint8_t)s_nav.mag_axis_src[i];
    for (int i = 0; i < 3; i++)
        put_f32(pl + NOF_MAG_SIGN + 4u * i, s_nav.mag_axis_sign[i]);

    pl[NOF_MAG_ENABLE] = s_nav.mag_enable;
}

static void unpack_payload(const rt_uint8_t *pl)
{
    s_nav.mag_decl_deg = get_f32(pl + NOF_DECL);
    s_nav.nognss_lat   = get_f64(pl + NOF_LAT);
    s_nav.nognss_lon   = get_f64(pl + NOF_LON);
    s_nav.nognss_alt   = get_f64(pl + NOF_ALT);

    for (int i = 0; i < 3; i++)
        s_nav.imu_axis_src[i] = (rt_int8_t)get_u8(pl + NOF_IMU_SRC + i);
    for (int i = 0; i < 3; i++)
        s_nav.imu_axis_sign[i] = get_f32(pl + NOF_IMU_SIGN + 4u * i);

    for (int i = 0; i < 3; i++)
        s_nav.mag_axis_src[i] = (rt_int8_t)get_u8(pl + NOF_MAG_SRC + i);
    for (int i = 0; i < 3; i++)
        s_nav.mag_axis_sign[i] = get_f32(pl + NOF_MAG_SIGN + 4u * i);

    s_nav.mag_enable = get_u8(pl + NOF_MAG_ENABLE);
}

/* 记录内容合法性: 轴重排非法的记录拒用 (保持缺省), 防误写坏参数上线 */
static rt_bool_t payload_sane(void)
{
    if (!axis_src_valid(s_nav.imu_axis_src) ||
        !axis_src_valid(s_nav.mag_axis_src))
        return RT_FALSE;
    if (s_nav.mag_decl_deg < -90.0f || s_nav.mag_decl_deg > 90.0f)
        return RT_FALSE;
    if (s_nav.nognss_lat < -90.0 || s_nav.nognss_lat > 90.0 ||
        s_nav.nognss_lon < -180.0 || s_nav.nognss_lon > 180.0)
        return RT_FALSE;
    if (s_nav.mag_enable > 1u)
        return RT_FALSE;
    return RT_TRUE;
}

/* ------------------------- 对外接口 ------------------------- */

rt_err_t param_nav_init(void)
{
    rt_uint8_t payload[NAV_PAYLOAD_SIZE];

    if (s_navrun.inited)
        return RT_EOK;
    s_navrun.inited = RT_TRUE;

    if (param_part_load(PARAM_PART_NAV, payload, sizeof(payload), RT_NULL)
        == RT_EOK)
    {
        unpack_payload(payload);
        if (!payload_sane())
        {
            LOG_W("nav record insane, keep compile-time defaults");
            param_nav_defaults();
            return -RT_ERROR;
        }
        s_navrun.stored = RT_TRUE;
        LOG_I("nav: loaded from W25Q64 (decl=%.1f° pos=%.5f,%.5f,%0.1fm)",
              (double)s_nav.mag_decl_deg, s_nav.nognss_lat,
              s_nav.nognss_lon, s_nav.nognss_alt);
    }
    else
    {
        LOG_I("nav: no record, compile-time defaults active");
    }
    return RT_EOK;
}

const struct nav_params *param_nav(void)
{
    return &s_nav;
}

rt_err_t param_nav_save(void)
{
    rt_uint8_t payload[NAV_PAYLOAD_SIZE];

    if (!payload_sane())
    {
        LOG_W("nav save refused: insane values");
        return -RT_EINVAL;
    }
    pack_payload(payload);
    {
        rt_err_t err = param_part_save(PARAM_PART_NAV, payload,
                                       sizeof(payload));

        if (err == RT_EOK)
            s_navrun.dirty = RT_FALSE;
        return err;
    }
}

rt_err_t param_nav_load(void)
{
    rt_uint8_t payload[NAV_PAYLOAD_SIZE];

    if (param_part_load(PARAM_PART_NAV, payload, sizeof(payload), RT_NULL)
        == RT_EOK)
    {
        unpack_payload(payload);
        if (payload_sane())
        {
            s_navrun.stored = RT_TRUE;
            s_navrun.dirty = RT_FALSE;
            return RT_EOK;
        }
    }
    param_nav_defaults();
    s_navrun.stored = RT_FALSE;          /* 分区无有效记录, 展示如实 */
    s_navrun.dirty = RT_FALSE;
    return -RT_ERROR;
}

void param_nav_defaults(void)
{
    memcpy(&s_nav, &s_nav_defaults, sizeof(s_nav));
}

rt_bool_t param_nav_dirty(void)
{
    return s_navrun.dirty;
}

/* ------------------------- 上电自启 + MSH 命令 ------------------------- */

static int param_nav_boot(void)
{
    (void)param_nav_init();
    return 0;
}
INIT_COMPONENT_EXPORT(param_nav_boot);

static void nav_show(void)
{
    const struct nav_params *n = &s_nav;

    LOG_I("=== 导航参数 (nav 分区, %s%s) ===",
          s_navrun.stored ? "stored" : "defaults",
          s_navrun.dirty ? ", DIRTY" : "");
    LOG_I("decl=%.2f°  pos=%.5f, %.5f, %.1fm  mag_enable=%d",
          (double)n->mag_decl_deg, n->nognss_lat, n->nognss_lon,
          n->nognss_alt, (int)n->mag_enable);
    LOG_I("imu axis src=[%d %d %d] sign=[%.0f %.0f %.0f]",
          (int)n->imu_axis_src[0], (int)n->imu_axis_src[1],
          (int)n->imu_axis_src[2],
          (double)n->imu_axis_sign[0], (double)n->imu_axis_sign[1],
          (double)n->imu_axis_sign[2]);
    LOG_I("mag axis src=[%d %d %d] sign=[%.0f %.0f %.0f]",
          (int)n->mag_axis_src[0], (int)n->mag_axis_src[1],
          (int)n->mag_axis_src[2],
          (double)n->mag_axis_sign[0], (double)n->mag_axis_sign[1],
          (double)n->mag_axis_sign[2]);
    LOG_I("usage: nav set decl|lat|lon|alt|mag <v> | "
          "nav set iaxis|maxis <sx sy sz kx ky kz> | nav save|load|reset");
}

/* 解析 "src x3 + sign x3" 六参数并校验 */
static rt_bool_t nav_set_axis(rt_int8_t src[3], float sign[3],
                              int argc, char **argv)
{
    if (argc < 8)
        return RT_FALSE;

    for (int i = 0; i < 3; i++)
    {
        long v = nav_parse_int(argv[2 + i]);

        if (v < 0 || v > 2)
            return RT_FALSE;
        src[i] = (rt_int8_t)v;
    }
    if (!axis_src_valid(src))
        return RT_FALSE;

    for (int i = 0; i < 3; i++)
    {
        double k = nav_parse_num(argv[5 + i]);

        if (!(k > 0.999 && k < 1.001) && !(k < -0.999 && k > -1.001))
            return RT_FALSE;
        sign[i] = (float)k;
    }
    return RT_TRUE;
}

static void nav(int argc, char **argv)
{
    if (argc < 2)
    {
        nav_show();
        return;
    }

    if (!rt_strcmp(argv[1], "save"))
    {
        rt_err_t err = param_nav_save();

        LOG_I("nav save: %s (%d)", err == RT_EOK ? "OK" : "failed", (int)err);
        return;
    }
    if (!rt_strcmp(argv[1], "load"))
    {
        rt_err_t err = param_nav_load();

        LOG_I("nav load: %s (%d)", err == RT_EOK ? "OK" : "no record",
              (int)err);
        return;
    }
    if (!rt_strcmp(argv[1], "reset"))
    {
        param_nav_defaults();
        s_navrun.dirty = RT_TRUE;
        LOG_I("nav reset: compile-time defaults (save to persist)");
        return;
    }

    if (!rt_strcmp(argv[1], "set") && argc >= 3 &&
        !rt_strcmp(argv[2], "iaxis"))
    {
        rt_int8_t src[3];
        float sign[3];

        if (nav_set_axis(src, sign, argc, argv))
        {
            memcpy(s_nav.imu_axis_src, src, 3);
            memcpy(s_nav.imu_axis_sign, sign, sizeof(sign));
            s_navrun.dirty = RT_TRUE;
            LOG_I("nav set iaxis OK (effective immediately)");
        }
        else
            LOG_W("bad iaxis args (src 0..2 permutation + sign ±1)");
        return;
    }
    if (!rt_strcmp(argv[1], "set") && argc >= 3 &&
        !rt_strcmp(argv[2], "maxis"))
    {
        rt_int8_t src[3];
        float sign[3];

        if (nav_set_axis(src, sign, argc, argv))
        {
            memcpy(s_nav.mag_axis_src, src, 3);
            memcpy(s_nav.mag_axis_sign, sign, sizeof(sign));
            s_navrun.dirty = RT_TRUE;
            LOG_I("nav set maxis OK (effective immediately)");
        }
        else
            LOG_W("bad maxis args (src 0..2 permutation + sign ±1)");
        return;
    }

    if (!rt_strcmp(argv[1], "set") && argc >= 4)
    {
        if (!rt_strcmp(argv[2], "decl"))
        {
            double v = nav_parse_num(argv[3]);

            if (!(v >= -90.0 && v <= 90.0))
            {
                LOG_W("decl out of range [-90,90]");
                return;
            }
            s_nav.mag_decl_deg = (float)v;
        }
        else if (!rt_strcmp(argv[2], "lat") || !rt_strcmp(argv[2], "lon") ||
                 !rt_strcmp(argv[2], "alt"))
        {
            double v = nav_parse_num(argv[3]);

            if (!rt_strcmp(argv[2], "lat") && !(v >= -90.0 && v <= 90.0))
            {
                LOG_W("lat out of range [-90,90]");
                return;
            }
            if (!rt_strcmp(argv[2], "lon") && !(v >= -180.0 && v <= 180.0))
            {
                LOG_W("lon out of range [-180,180]");
                return;
            }
            if (!rt_strcmp(argv[2], "alt") && !(v >= -1e4 && v <= 1e5))
            {
                LOG_W("alt out of range [-1e4,1e5]");
                return;
            }
            if (!rt_strcmp(argv[2], "lat"))
                s_nav.nognss_lat = v;
            else if (!rt_strcmp(argv[2], "lon"))
                s_nav.nognss_lon = v;
            else
                s_nav.nognss_alt = v;
        }
        else if (!rt_strcmp(argv[2], "mag"))
        {
            long v = nav_parse_int(argv[3]);

            if (v != 0 && v != 1)
            {
                LOG_W("mag expects 0|1");
                return;
            }
            s_nav.mag_enable = (rt_uint8_t)v;
        }
        else
        {
            nav_show();
            return;
        }

        s_navrun.dirty = RT_TRUE;
        LOG_I("nav set %s = %s (nav save to persist)", argv[2], argv[3]);
        return;
    }

    nav_show();
}
MSH_CMD_EXPORT(nav, navigation params: show / set / save / load / reset);
