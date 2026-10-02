/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * UM982 NMEA PVT 解析实现 (见 um982_nmea.h)
 *
 * 纯解析, 无串口/线程: gnss_data.c 解析线程组句后从这里进,
 * 结果汇入 static nav.data, 消费方用 um982_nmea_get_data() 取快照。
 * 喂入侧只有一个 (gnss 线程), 读取侧用短关中断拷贝保证 double 字段一致。
 *
 * 解析不使用 libc (不引入 strtod/strtok)。
 */

#include "um982_nmea.h"

#include <rtthread.h>
#include <math.h>               /* cos/sin */

#if UM982_NMEA_ENABLE

/* ------------------------- 常量 ------------------------- */

#define UM982_KNOT_TO_MS    (1852.0 / 3600.0)
#define UM982_D2R           (3.14159265358979323846 / 180.0)
#define UM982_GPS_EPOCH_SEC 315964800u    /* 1980-01-06 00:00:00 UTC 的 Unix 秒 */
#define UM982_UTC_LEAP_SEC  18u           /* 2017-01-01 起的 GPS-UTC 闰秒 */

/* ------------------------- 内部状态 ------------------------- */

static struct
{
    struct gnss_data data;

    rt_uint32_t last_date_sec;      /* 最近已知 UTC 日 0 点 (GGA 补日期用) */
    rt_bool_t   have_date;

    /* 统计 */
    rt_uint32_t rmc_cnt;
    rt_uint32_t gga_cnt;
    rt_uint32_t zda_cnt;
    rt_uint32_t csum_err;           /* 校验和错误 */
    rt_uint32_t field_err;          /* 字段缺失/非法 */
    char        last[UM982_NMEA_LINE_MAX];  /* 最近一条语句 (调试) */
} nav;

/* ------------------------- 基础解析 ------------------------- */

static char *nav_find(const char *s, char c)
{
    for (; *s != '\0'; s++)
    {
        if (*s == c)
            return (char *)s;
    }
    return RT_NULL;
}

static rt_int32_t nav_hex(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

/* 十进制定点数 "123.456" / "-13.7" -> 123.456 / -13.7,
 * 空字段/非数字返回失败 (海面分离常为负, 必须带符号) */
static rt_bool_t nav_f64(const char *s, double *v)
{
    rt_uint64_t iv = 0, fv = 0;
    rt_uint32_t nd = 0, fd = 0, k;
    double scale = 1.0, r;
    rt_bool_t neg = RT_FALSE;

    if (s == RT_NULL)
        return RT_FALSE;

    if (*s == '-' || *s == '+')
    {
        neg = (*s == '-');
        s++;
    }
    while (*s >= '0' && *s <= '9' && nd < 12u)
    {
        iv = iv * 10u + (rt_uint64_t)(*s - '0');
        s++;
        nd++;
    }
    if (*s == '.')
    {
        s++;
        while (*s >= '0' && *s <= '9' && fd < 9u)
        {
            fv = fv * 10u + (rt_uint64_t)(*s - '0');
            s++;
            fd++;
        }
    }
    if (nd == 0u && fd == 0u)
        return RT_FALSE;

    for (k = 0; k < fd; k++)
        scale *= 10.0;
    r = (double)iv + (double)fv / scale;
    *v = neg ? -r : r;
    return RT_TRUE;
}

/* 十进制无符号整数 */
static rt_bool_t nav_u32(const char *s, rt_uint32_t *v)
{
    rt_uint32_t x = 0, nd = 0;

    if (s == RT_NULL)
        return RT_FALSE;

    while (*s >= '0' && *s <= '9' && nd < 9u)
    {
        x = x * 10u + (rt_uint32_t)(*s - '0');
        s++;
        nd++;
    }
    if (nd == 0u)
        return RT_FALSE;

    *v = x;
    return RT_TRUE;
}

/* HHMMSS.sss -> 时/分/秒 + 秒内微秒 */
static rt_bool_t nav_time(const char *s, rt_uint32_t *h, rt_uint32_t *m,
                          rt_uint32_t *sec, rt_uint32_t *frac_us)
{
    rt_uint32_t v = 0, nd = 0, fr = 0, fd = 0;

    if (s == RT_NULL)
        return RT_FALSE;

    while (*s >= '0' && *s <= '9' && nd < 6u)
    {
        v = v * 10u + (rt_uint32_t)(*s - '0');
        s++;
        nd++;
    }
    if (nd < 6u)
        return RT_FALSE;

    *h   = v / 10000u;
    *m   = (v / 100u) % 100u;
    *sec = v % 100u;

    /* 日历合法性: 畸形句 (如 999999) 会产出 +356h 偏移的 utc_sec,
     * 下游 PPS 配对门槛虽会拒掉, 但 time 字段已被污染成 time_valid */
    if (*h > 23u || *m > 59u || *sec > 60u)
        return RT_FALSE;

    if (*s == '.')
    {
        s++;
        while (*s >= '0' && *s <= '9' && fd < 3u)
        {
            fr = fr * 10u + (rt_uint32_t)(*s - '0');
            s++;
            fd++;
        }
    }
    while (fd < 3u)
    {
        fr *= 10u;
        fd++;
    }
    *frac_us = fr * 1000u;
    return RT_TRUE;
}

/* 天数基准: Howard Hinnant civil_from_days 的逆运算 */
static rt_int32_t nav_days_from_civil(rt_int32_t y, rt_uint32_t m, rt_uint32_t d)
{
    rt_int32_t era;
    rt_uint32_t yoe, doy, doe;

    y -= (m <= 2u);
    era = (y >= 0 ? y : y - 399) / 400;
    yoe = (rt_uint32_t)(y - era * 400);
    doy = (153u * (m + (m > 2u ? (rt_uint32_t)-3 : 9u)) + 2u) / 5u + d - 1u;
    doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    return era * 146097 + (rt_int32_t)doe - 719468;
}

/* DDMMYY -> 年(20xx)/月/日 */
static rt_bool_t nav_date_ddmmyy(const char *s, rt_int32_t *y,
                                 rt_uint32_t *mo, rt_uint32_t *d)
{
    rt_uint32_t v = 0, nd = 0;

    if (s == RT_NULL)
        return RT_FALSE;

    while (*s >= '0' && *s <= '9' && nd < 6u)
    {
        v = v * 10u + (rt_uint32_t)(*s - '0');
        s++;
        nd++;
    }
    if (nd != 6u)
        return RT_FALSE;

    *d  = v / 10000u;
    *mo = (v / 100u) % 100u;
    *y  = 2000 + (rt_int32_t)(v % 100u);

    if (*mo < 1u || *mo > 12u || *d < 1u || *d > 31u)
        return RT_FALSE;
    return RT_TRUE;
}

static rt_uint32_t nav_day_epoch(rt_int32_t y, rt_uint32_t mo, rt_uint32_t d)
{
    return (rt_uint32_t)((rt_int64_t)nav_days_from_civil(y, mo, d) * 86400);
}

/*
 * ddmm.mmmmm[N/S] / dddmm.mmmmm[E/W] -> 度 (N/E 为正)。
 * 分钟部分固定 2 位整数, 度部分 = 整数部分 / 100。
 */
static rt_bool_t nav_latlon(const char *num, const char *hemi, double *deg)
{
    rt_uint64_t iv = 0, fv = 0;
    rt_uint32_t nd = 0, fd = 0, k;
    double scale = 1.0, minutes, d;
    char h;

    if (num == RT_NULL || hemi == RT_NULL)
        return RT_FALSE;

    while (*num >= '0' && *num <= '9' && nd < 6u)
    {
        iv = iv * 10u + (rt_uint64_t)(*num - '0');
        num++;
        nd++;
    }
    if (*num == '.')
    {
        num++;
        while (*num >= '0' && *num <= '9' && fd < 9u)
        {
            fv = fv * 10u + (rt_uint64_t)(*num - '0');
            num++;
            fd++;
        }
    }
    if (nd < 3u)                            /* 至少 dmm. */
        return RT_FALSE;

    h = *hemi;
    if (h != 'N' && h != 'S' && h != 'E' && h != 'W')
        return RT_FALSE;

    for (k = 0; k < fd; k++)
        scale *= 10.0;
    minutes = (double)(iv % 100u) + (double)fv / scale;
    d = (double)(iv / 100u) + minutes / 60.0;

    if (h == 'S' || h == 'W')
        d = -d;
    *deg = d;
    return RT_TRUE;
}

/* 日期入库: GGA 只有时间, 靠最近一条 RMC/ZDA 的日期补全 */
static void nav_set_date(rt_int32_t y, rt_uint32_t mo, rt_uint32_t d)
{
    nav.last_date_sec = nav_day_epoch(y, mo, d);
    nav.have_date = RT_TRUE;
}

/* 秒 + 秒内微秒入库, GGA 场景做跨 UTC 零点修正 */
static void nav_set_utc(rt_uint32_t h, rt_uint32_t mi, rt_uint32_t sec,
                        rt_uint32_t frac_us)
{
    rt_uint32_t cand;

    if (!nav.have_date)
        return;

    if (sec >= 60u)
        sec = 59u;                          /* 闰秒: 钳到 59s */
    cand = nav.last_date_sec + h * 3600u + mi * 60u + sec;

    /* 与上一次的秒比较, 跨 UTC 零点时把日期挪一天 */
    if (nav.data.time.utc_sec != 0u)
    {
        rt_int32_t diff = (rt_int32_t)(cand - nav.data.time.utc_sec);

        if (diff > 43200)
            cand -= 86400u;
        else if (diff < -43200)
            cand += 86400u;
    }

    nav.data.time.utc_sec  = cand;
    nav.data.time.utc_usec = frac_us;
    nav.data.time_valid = RT_TRUE;
}

/* ------------------------- 语句处理 ------------------------- */

static void nav_rmc(char **f, rt_uint8_t nf)
{
    rt_uint32_t h, mi, sec, frac;
    rt_int32_t yy;
    rt_uint32_t mo, dd;
    double lat, lon, spd, cog;

    /* $--RMC,time,status,lat,N,lon,E,spd(kn),cog(deg true),date,... */
    if (nf < 10u || !nav_time(f[1], &h, &mi, &sec, &frac))
    {
        nav.field_err++;
        return;
    }

    if (f[2][0] != 'A')
    {
        /* 'V' (警告): 定位无效, 时间/日期也不可信 */
        nav.data.status.fix_type  = GNSS_FIX_INVALID;
        nav.data.status.rtk_status = GNSS_RTK_NONE;
        nav.data.pos_valid = RT_FALSE;
        nav.data.vel_valid = RT_FALSE;
        return;
    }

    if (nav_date_ddmmyy(f[9], &yy, &mo, &dd))
        nav_set_date(yy, mo, dd);
    nav_set_utc(h, mi, sec, frac);

    /* RMC 也带经纬度: 仅在可解析时刷新数值, pos_valid 由 GGA 定位质量决定 */
    if (nav_latlon(f[3], f[4], &lat) && nav_latlon(f[5], f[6], &lon))
    {
        nav.data.position.latitude  = lat;
        nav.data.position.longitude = lon;
    }

    /* 地面速度(kn) + 航迹角(deg true) -> N/E 速度; NMEA 无天向速度 */
    if (nav_f64(f[7], &spd) && nav_f64(f[8], &cog))
    {
        spd *= UM982_KNOT_TO_MS;
        cog *= UM982_D2R;
        nav.data.velocity.vn = (float)(spd * cos(cog));
        nav.data.velocity.ve = (float)(spd * sin(cog));
        nav.data.velocity.vu = 0.0f;
        nav.data.vel_valid = RT_TRUE;
    }
    else
    {
        /* A 状态但速度/航迹角本句不可解析 (低速/静止时接收机常置空 COG):
         * vel_valid 语义是"本句可解析", 只置位不清零会把上一历元的速度
         * 配本句新时标送下游 (gins velne 先验被冻结的旧速度污染,
         * 运动->静止过程尤其危险), 故逐句重算并清零速度分量 */
        nav.data.velocity.vn = 0.0f;
        nav.data.velocity.ve = 0.0f;
        nav.data.velocity.vu = 0.0f;
        nav.data.vel_valid = RT_FALSE;
    }

    nav.rmc_cnt++;
}

static void nav_gga(char **f, rt_uint8_t nf)
{
    rt_uint32_t h, mi, sec, frac, q = 0, sats;
    double lat, lon, x, sep_d;

    /* $--GGA,time,lat,N,lon,E,quality,numsat,hdop,alt,M,geoid,M,diffage,stn */
    if (nf < 10u || !nav_time(f[1], &h, &mi, &sec, &frac))
    {
        nav.field_err++;
        return;
    }

    /* 定位质量: 单个数字 0..8 */
    if (f[6][0] >= '0' && f[6][0] <= '8' && f[6][1] == '\0')
        q = (rt_uint32_t)(f[6][0] - '0');

    nav.data.status.fix_type = (rt_uint8_t)q;
    switch (q)
    {
    case 4:
        nav.data.status.rtk_status = GNSS_RTK_FIX;
        break;
    case 5:
        nav.data.status.rtk_status = GNSS_RTK_FLOAT;
        break;
    case 2:
    case 3:
        nav.data.status.rtk_status = GNSS_RTK_DGPS;
        break;
    default:
        nav.data.status.rtk_status = GNSS_RTK_NONE;
        break;
    }

    if (nav_u32(f[7], &sats))
        nav.data.status.satellites = (rt_uint8_t)sats;
    if (nav_f64(f[8], &x))
        nav.data.status.hdop = (float)x;

    /* pos_valid 与本次经纬度解析结果耦合: quality>0 但 lat/lon 字段缺失/
     * 畸形时撤销定位标志 —— 否则上一条的旧坐标配本句新时标流入下游
     * (环内样本会被当作新鲜观测); quality=0 同样撤销 */
    if (nav_latlon(f[2], f[3], &lat) && nav_latlon(f[4], f[5], &lon))
    {
        nav.data.position.latitude  = lat;
        nav.data.position.longitude = lon;
        nav.data.pos_valid = (q != 0u);
    }
    else
    {
        nav.data.pos_valid = RT_FALSE;
        if (q != 0u)
            nav.field_err++;
    }

    /* GGA 天线高是海拔高(MSL), 加海面分离得组合导航用的 WGS84 椭球高 */
    if (nav_f64(f[9], &x))
    {
        sep_d = 0.0;
        if (nf >= 12u && nav_f64(f[11], &sep_d))
            nav.data.position.geoid_sep = (float)sep_d;
        else
            nav.data.position.geoid_sep = 0.0f;
        nav.data.position.altitude = (float)x + (float)sep_d;
    }

    nav_set_utc(h, mi, sec, frac);
    if (q != 0u)
        nav.data.update_cnt++;
    nav.gga_cnt++;
}

static void nav_zda(char **f, rt_uint8_t nf)
{
    rt_uint32_t h, mi, sec, frac, d2, m2, y2;

    /* $--ZDA,time,day,month,year,tz_h,tz_m (时间与日期同句) */
    if (nf < 5u || !nav_time(f[1], &h, &mi, &sec, &frac) ||
        !nav_u32(f[2], &d2) || !nav_u32(f[3], &m2) || !nav_u32(f[4], &y2))
    {
        nav.field_err++;
        return;
    }
    if (m2 < 1u || m2 > 12u || d2 < 1u || d2 > 31u || y2 < 2000u || y2 > 2099u)
    {
        nav.field_err++;
        return;
    }

    nav_set_date((rt_int32_t)y2, m2, d2);
    nav_set_utc(h, mi, sec, frac);
    nav.zda_cnt++;
}

/* ------------------------- 对外接口 ------------------------- */

void um982_nmea_feed_line(const char *s)
{
    char buf[UM982_NMEA_LINE_MAX];
    char *star, *f[UM982_NMEA_FIELD_MAX];
    char *walk;
    const char *p;
    rt_uint16_t n = 0;
    rt_uint8_t nf = 0, sum = 0;
    const char *type, *body;

    if (s == RT_NULL || s[0] != '$')
        return;

    /* 校验和: '$' 与 '*' 之间逐字节异或 */
    star = nav_find(s, '*');
    if (star == RT_NULL)
        return;                             /* 不完整语句 */
    if (star[1] == '\0' || star[2] == '\0')
    {
        nav.csum_err++;
        return;
    }
    {
        rt_int32_t hi = nav_hex(star[1]);
        rt_int32_t lo = nav_hex(star[2]);

        if (hi < 0 || lo < 0)
        {
            nav.csum_err++;
            return;
        }
        for (p = s + 1; p < star; p++)
            sum ^= (rt_uint8_t)*p;
        if ((rt_uint8_t)(hi * 16 + lo) != sum)
        {
            nav.csum_err++;
            return;
        }
    }

    /* 工作副本: 不改写调用方缓冲 (调用方之后还要解析原句) */
    while (s[n] != '\0' && n < (UM982_NMEA_LINE_MAX - 1u))
    {
        buf[n] = s[n];
        n++;
    }
    buf[n] = '\0';
    rt_memcpy(nav.last, buf, (rt_size_t)n + 1u);

    /* 逗号分段 (替换为 '\0', 截掉校验和)。工作副本只拷贝了前
     * LINE_MAX-1 字符, 原句 '*' 在截断点之后时副本中无 '*' —— 判空,
     * 否则 NULL 解引用写 HardFault (对外接口允许直接喂长句) */
    star = nav_find(buf, '*');
    if (star == RT_NULL)
    {
        nav.field_err++;
        return;
    }
    *star = '\0';

    f[nf++] = buf;
    for (walk = buf; *walk != '\0' && nf < UM982_NMEA_FIELD_MAX; walk++)
    {
        if (*walk == ',')
        {
            *walk = '\0';
            f[nf++] = walk + 1;
        }
    }

    type = f[0];
    if (rt_strlen(type) < 6u)               /* "$xxTTT" 至少 6 字符 */
        return;
    body = type + 3;                        /* 跳过 "$xx" 台标 (GN/GP/BD/...) */

    if (body[0] == 'R' && body[1] == 'M' && body[2] == 'C')
        nav_rmc(f, nf);
    else if (body[0] == 'G' && body[1] == 'G' && body[2] == 'A')
        nav_gga(f, nf);
    else if (body[0] == 'Z' && body[1] == 'D' && body[2] == 'A')
        nav_zda(f, nf);
}

void um982_nmea_get_data(struct gnss_data *out)
{
    rt_base_t level;

    if (out == RT_NULL)
        return;

    level = rt_hw_interrupt_disable();
    *out = nav.data;
    rt_hw_interrupt_enable(level);
}

rt_err_t um982_nmea_to_gpst(const struct gnss_data *d,
                            rt_uint16_t *week, double *sow)
{
    rt_uint64_t gpst;

    if (d == RT_NULL || !d->time_valid)
        return -RT_ERROR;

    gpst = (rt_uint64_t)d->time.utc_sec - UM982_GPS_EPOCH_SEC + UM982_UTC_LEAP_SEC;
    if (week != RT_NULL)
        *week = (rt_uint16_t)(gpst / 604800u);
    if (sow != RT_NULL)
        *sow = (double)(gpst % 604800u) + (double)d->time.utc_usec * 1e-6;
    return RT_EOK;
}

void um982_nmea_pos_std(const struct gnss_data *d, double std[3])
{
    double hdop;

    if (d == RT_NULL || std == RT_NULL)
        return;

    hdop = (d->status.hdop > 0.01f) ? (double)d->status.hdop
                                    : (double)UM982_NMEA_HDOP_FALLBACK;
    std[0] = hdop * UM982_NMEA_UERE_M;
    std[1] = hdop * UM982_NMEA_UERE_M;
    std[2] = 2.0 * hdop * UM982_NMEA_UERE_M;
}

void um982_nmea_reset(void)
{
    rt_memset(&nav, 0, sizeof(nav));
}

#endif /* UM982_NMEA_ENABLE */

/* ------------------------- FinSH 调试命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH) && UM982_NMEA_ENABLE
#include <finsh.h>

#define LOG_TAG "um982"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

static void um982(int argc, char **argv)
{
    struct gnss_data d;

    RT_UNUSED(argc);
    RT_UNUSED(argv);

    um982_nmea_get_data(&d);

    LOG_I("=== UM982 NMEA PVT ===");
    LOG_I("time    : %u.%06u valid=%d (UTC epoch)",
          d.time.utc_sec, d.time.utc_usec, d.time_valid);
    LOG_I("position: lat=%.7f deg lon=%.7f deg",
          (double)d.position.latitude, (double)d.position.longitude);
    LOG_I("          alt=%.1f m (ellipsoid, geoid=%.1f) %s",
          (double)d.position.altitude, (double)d.position.geoid_sep,
          d.pos_valid ? "" : "INVALID");
    LOG_I("velocity: vn=%.3f ve=%.3f vu=%.3f m/s valid=%d",
          (double)d.velocity.vn, (double)d.velocity.ve, (double)d.velocity.vu, d.vel_valid);
    LOG_I("status  : fix=%d rtk=%d sats=%u hdop=%.2f",
          d.status.fix_type, d.status.rtk_status, d.status.satellites,
          (double)d.status.hdop);
    LOG_I("stats   : rmc=%u gga=%u zda=%u csum_err=%u field_err=%u "
          "updates=%u",
          nav.rmc_cnt, nav.gga_cnt, nav.zda_cnt,
          nav.csum_err, nav.field_err, d.update_cnt);
}
MSH_CMD_EXPORT(um982, UM982 PVT 解析结果与统计);
#endif
