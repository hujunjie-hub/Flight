/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 传感器标定参数域实现: 字段序列化 + W25Q64 calib 分区读写 + 片内
 * Flash 扇区 7 旧格式记录一次性导入, 布局与背景见 param_calib.h。
 */

#include <rtthread.h>
#include <string.h>

#include "param_part.h"
#include "param_calib.h"
#include "param_sys.h"

#define LOG_TAG "calib"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

/* ------------------------- 记录体布局 (payload 内偏移, 小端 f32/u16) ------------------------- */

#define CALIB_PAYLOAD_SIZE      92u

#define COF_MAG_BIAS            0       /* f32 x3  硬磁偏置 µT */
#define COF_MAG_SOFTIRON        12      /* f32 x9  软磁矩阵, 行主序 */
#define COF_MAG_RADIUS          48      /* f32    等效半径 µT */
#define COF_MAG_RESID           52      /* f32    拟合残差 */
#define COF_MAG_MAXRATIO        56      /* f32    主轴半径比 */
#define COF_MAG_SAMPLES         60      /* u16    样本数 */
#define COF_FLAGS               62      /* u16    有效位 (同旧格式定义) */
#define COF_BARO_OFFSET         64      /* f32    压强偏移 Pa */
#define COF_BARO_CALTEMP        68      /* f32    标定温度 °C */
#define COF_BARO_MEAN           72      /* f32    标定均值 Pa */
#define COF_ACC_BIAS            76      /* f32 x3  加计零偏 mGal (FRD) */
#define COF_ACC_TEMP            88      /* f32    采集时 IMU 温度 °C */

/* ------------------------- RAM 镜像 ------------------------- */

static struct
{
    struct calib_data data;
    rt_bool_t         inited;
} s_calib;

/* ------------------------- 小工具 ------------------------- */

static void put_u16(rt_uint8_t *p, rt_uint16_t v)
{
    p[0] = (rt_uint8_t)v;
    p[1] = (rt_uint8_t)(v >> 8);
}

static void put_u32(rt_uint8_t *p, rt_uint32_t v)
{
    p[0] = (rt_uint8_t)v;
    p[1] = (rt_uint8_t)(v >> 8);
    p[2] = (rt_uint8_t)(v >> 16);
    p[3] = (rt_uint8_t)(v >> 24);
}

static rt_uint16_t get_u16(const rt_uint8_t *p)
{
    return (rt_uint16_t)(p[0] | (p[1] << 8));
}

static rt_uint32_t get_u32(const rt_uint8_t *p)
{
    return (rt_uint32_t)p[0] | ((rt_uint32_t)p[1] << 8) |
           ((rt_uint32_t)p[2] << 16) | ((rt_uint32_t)p[3] << 24);
}

/* float 与 u32 位模式互转 (避免别名违规) */
static rt_uint32_t f2u(float f)
{
    rt_uint32_t u;

    memcpy(&u, &f, 4);
    return u;
}

static float u2f(rt_uint32_t u)
{
    float f;

    memcpy(&f, &u, 4);
    return f;
}

double calib_parse_num(const char *s)
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

/* ------------------------- 序列化 ------------------------- */

/* 有效位定义与旧片内格式一致: 显式存 flags, 不靠数值反推 (清除磁标定
 * 后字段仍保留旧值, 数值反推会把已失效参数复活) */
#define CALIB_FLAG_MAG          0x0001u
#define CALIB_FLAG_BARO         0x0002u
#define CALIB_FLAG_ACC          0x0004u

static void pack_payload(rt_uint8_t *pl)
{
    struct calib_data *d = &s_calib.data;
    rt_uint16_t flags = 0;

    memset(pl, 0, CALIB_PAYLOAD_SIZE);

    if (d->mag_valid)
        flags |= CALIB_FLAG_MAG;
    if (d->baro_valid)
        flags |= CALIB_FLAG_BARO;
    if (d->acc_valid)
        flags |= CALIB_FLAG_ACC;

    for (int i = 0; i < 3; i++)
        put_u32(pl + COF_MAG_BIAS + 4u * i, f2u(d->mag_bias_ut[i]));
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            put_u32(pl + COF_MAG_SOFTIRON + 12u * i + 4u * j,
                    f2u(d->mag_softiron[i][j]));
    put_u32(pl + COF_MAG_RADIUS,   f2u(d->mag_radius_ut));
    put_u32(pl + COF_MAG_RESID,    f2u(d->mag_resid));
    put_u32(pl + COF_MAG_MAXRATIO, f2u(d->mag_maxratio));
    put_u16(pl + COF_MAG_SAMPLES,  d->mag_samples);
    put_u16(pl + COF_FLAGS,        flags);

    put_u32(pl + COF_BARO_OFFSET,  f2u(d->baro_offset_pa));
    put_u32(pl + COF_BARO_CALTEMP, f2u(d->baro_cal_temp));
    put_u32(pl + COF_BARO_MEAN,    f2u(d->baro_mean_pa));
    for (int i = 0; i < 3; i++)
        put_u32(pl + COF_ACC_BIAS + 4u * i, f2u(d->acc_bias_mgal[i]));
    put_u32(pl + COF_ACC_TEMP,     f2u(d->acc_cal_temp));
}

static void unpack_payload(const rt_uint8_t *pl)
{
    struct calib_data *d = &s_calib.data;
    rt_uint16_t flags = get_u16(pl + COF_FLAGS);

    for (int i = 0; i < 3; i++)
        d->mag_bias_ut[i] = u2f(get_u32(pl + COF_MAG_BIAS + 4u * i));
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            d->mag_softiron[i][j] = u2f(get_u32(pl + COF_MAG_SOFTIRON +
                                                12u * i + 4u * j));
    d->mag_radius_ut = u2f(get_u32(pl + COF_MAG_RADIUS));
    d->mag_resid     = u2f(get_u32(pl + COF_MAG_RESID));
    d->mag_maxratio  = u2f(get_u32(pl + COF_MAG_MAXRATIO));
    d->mag_samples   = get_u16(pl + COF_MAG_SAMPLES);

    d->baro_offset_pa = u2f(get_u32(pl + COF_BARO_OFFSET));
    d->baro_cal_temp  = u2f(get_u32(pl + COF_BARO_CALTEMP));
    d->baro_mean_pa   = u2f(get_u32(pl + COF_BARO_MEAN));

    for (int i = 0; i < 3; i++)
        d->acc_bias_mgal[i] = u2f(get_u32(pl + COF_ACC_BIAS + 4u * i));
    d->acc_cal_temp = u2f(get_u32(pl + COF_ACC_TEMP));

    d->mag_valid  = (flags & CALIB_FLAG_MAG) ? RT_TRUE : RT_FALSE;
    d->baro_valid = (flags & CALIB_FLAG_BARO) ? RT_TRUE : RT_FALSE;
    d->acc_valid  = (flags & CALIB_FLAG_ACC) ? RT_TRUE : RT_FALSE;
}

/* ------------------------- 片内 Flash 旧格式一次性导入 ------------------------- */

/* 旧 calib_store (片内扇区 7, 0x080E0000) 的记录格式: 128B 槽 x 1024,
 * 头 magic u32 | ver u16 | flags u16 | seq u32 | crc u32, 参数体在 +16。
 * 导入仅读该区 (ROM 已截到 896KB, 链接器不会把代码放进该扇区)。 */
#define LEG_FLASH_ADDR          0x080E0000UL
#define LEG_FLASH_SIZE          (128u * 1024u)
#define LEG_RECORD_SIZE         128u
#define LEG_SLOT_NUM            (LEG_FLASH_SIZE / LEG_RECORD_SIZE)
#define LEG_PAYLOAD_OFF         16u
#define LEG_MAGIC               0x4C41434DUL               /* 'M','C','A','L' */
#define LEG_VERSION             1u
#define LEG_SEQ_OFF             8u
#define LEG_CRC_OFF             12u

#define LEG_FLAG_MAG            0x0001u
#define LEG_FLAG_BARO           0x0002u
#define LEG_FLAG_ACC            0x0004u

/* 旧参数体偏移 (记录内) */
#define LEG_COF_MAG_BIAS        16
#define LEG_COF_MAG_SOFTIRON    28
#define LEG_COF_MAG_RADIUS      64
#define LEG_COF_MAG_RESID       68
#define LEG_COF_MAG_MAXRATIO    72
#define LEG_COF_MAG_SAMPLES     76
#define LEG_COF_BARO_OFFSET     80
#define LEG_COF_BARO_CALTEMP    84
#define LEG_COF_BARO_MEAN       88
#define LEG_COF_ACC_BIAS        96
#define LEG_COF_ACC_TEMP        108

static rt_uint32_t leg_crc32_update(rt_uint32_t crc, const rt_uint8_t *p,
                                    rt_size_t len)
{
    while (len--)
    {
        crc ^= *p++;

        for (int i = 0; i < 8; i++)
            crc = (crc >> 1) ^ (0xEDB88320UL & (0u - (crc & 1u)));
    }
    return crc;
}

/* 旧格式记录 CRC: 覆盖 ver/flags/seq [4,12) + 参数体 [16,128) */
static rt_uint32_t leg_record_crc(const rt_uint8_t *rec)
{
    rt_uint32_t crc = leg_crc32_update(0xFFFFFFFFUL, rec + 4u,
                                       LEG_SEQ_OFF - 4u);

    crc = leg_crc32_update(crc, rec + LEG_PAYLOAD_OFF,
                           LEG_RECORD_SIZE - LEG_PAYLOAD_OFF);
    return crc ^ 0xFFFFFFFFUL;
}

/* 旧格式有效性: 新格式 (seq + CRC 盖 ver/flags/seq/体) 或升级前旧格式
 * (crc 在偏移 8 只盖参数体); 通过时 *seq 出记录序号 (旧格式恒 0) */
static rt_bool_t leg_record_valid(const rt_uint8_t *rec, rt_uint32_t *seq)
{
    if (get_u32(rec + 0) != LEG_MAGIC || get_u16(rec + 4) != LEG_VERSION)
        return RT_FALSE;

    if (get_u32(rec + LEG_CRC_OFF) == leg_record_crc(rec))
    {
        *seq = get_u32(rec + LEG_SEQ_OFF);
        return RT_TRUE;
    }

    if (get_u32(rec + 12u) == 0u)
    {
        rt_uint32_t crc = leg_crc32_update(0xFFFFFFFFUL, rec + LEG_PAYLOAD_OFF,
                                           LEG_RECORD_SIZE - LEG_PAYLOAD_OFF);

        if ((crc ^ 0xFFFFFFFFUL) == get_u32(rec + 8u))
        {
            *seq = 0u;
            return RT_TRUE;
        }
    }
    return RT_FALSE;
}

static void leg_unpack(const rt_uint8_t *rec)
{
    struct calib_data *d = &s_calib.data;
    rt_uint16_t flags = get_u16(rec + 6);

    d->mag_valid = (flags & LEG_FLAG_MAG) ? RT_TRUE : RT_FALSE;
    for (int i = 0; i < 3; i++)
        d->mag_bias_ut[i] = u2f(get_u32(rec + LEG_COF_MAG_BIAS + 4u * i));
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            d->mag_softiron[i][j] = u2f(get_u32(rec + LEG_COF_MAG_SOFTIRON +
                                                12u * i + 4u * j));
    d->mag_radius_ut = u2f(get_u32(rec + LEG_COF_MAG_RADIUS));
    d->mag_resid     = u2f(get_u32(rec + LEG_COF_MAG_RESID));
    d->mag_maxratio  = u2f(get_u32(rec + LEG_COF_MAG_MAXRATIO));
    d->mag_samples   = get_u16(rec + LEG_COF_MAG_SAMPLES);

    d->baro_valid    = (flags & LEG_FLAG_BARO) ? RT_TRUE : RT_FALSE;
    d->baro_offset_pa = u2f(get_u32(rec + LEG_COF_BARO_OFFSET));
    d->baro_cal_temp  = u2f(get_u32(rec + LEG_COF_BARO_CALTEMP));
    d->baro_mean_pa   = u2f(get_u32(rec + LEG_COF_BARO_MEAN));

    d->acc_valid = (flags & LEG_FLAG_ACC) ? RT_TRUE : RT_FALSE;
    for (int i = 0; i < 3; i++)
        d->acc_bias_mgal[i] = u2f(get_u32(rec + LEG_COF_ACC_BIAS + 4u * i));
    d->acc_cal_temp = u2f(get_u32(rec + LEG_COF_ACC_TEMP));
}

/*
 * 扫描片内扇区 7 旧记录, 取 "有效且 seq 最大" 一条解包进 RAM 镜像。
 * 返回 RT_TRUE = 找到。直接 volatile 内存读, 不做任何擦写。
 */
static rt_bool_t legacy_scan_internal(void)
{
    rt_uint8_t rec[LEG_RECORD_SIZE];
    rt_uint32_t best_seq = 0;
    rt_bool_t have_best = RT_FALSE;

    for (rt_uint32_t i = 0; i < LEG_SLOT_NUM; i++)
    {
        const volatile rt_uint8_t *p = (const volatile rt_uint8_t *)
            (LEG_FLASH_ADDR + i * LEG_RECORD_SIZE);
        rt_uint32_t seq = 0;

        if (p[0] == 0xFFu && p[1] == 0xFFu && p[2] == 0xFFu &&
            p[3] == 0xFFu && p[4] == 0xFFu && p[5] == 0xFFu &&
            p[6] == 0xFFu && p[7] == 0xFFu)
            break;                               /* 首个未编程槽: 后面全空 */

        memcpy(rec, (const void *)p, LEG_RECORD_SIZE);
        if (leg_record_valid(rec, &seq) &&
            (!have_best || (rt_int32_t)(seq - best_seq) > 0))
        {
            leg_unpack(rec);
            best_seq = seq;
            have_best = RT_TRUE;
        }
    }
    return have_best;
}

/* ------------------------- 对外接口 ------------------------- */

rt_err_t calib_store_init(void)
{
    rt_uint8_t payload[CALIB_PAYLOAD_SIZE];

    if (s_calib.inited)
        return RT_EOK;

    /* sys 域先行: 导入标志在其记录里 (两域同为 INIT_COMPONENT, 链接序
     * 不定, 这里显式先拉起, 幂等) */
    (void)param_sys_init();

    memset(&s_calib.data, 0, sizeof(s_calib.data));

    if (param_part_load(PARAM_PART_CALIB, payload, sizeof(payload), RT_NULL)
        == RT_EOK)
    {
        unpack_payload(payload);
        LOG_I("calib: loaded from W25Q64 (mag=%d baro=%d acc=%d)",
              (int)s_calib.data.mag_valid, (int)s_calib.data.baro_valid,
              (int)s_calib.data.acc_valid);
    }
    else
    {
        /* calib 分区无记录: 首次迁移时从片内扇区 7 导入旧标定一次 */
        if (!param_sys_calib_migrated() && legacy_scan_internal())
        {
            LOG_I("calib: legacy internal-flash record imported, "
                  "migrating to W25Q64");
            s_calib.inited = RT_TRUE;            /* save 内不再重入 */
            if (calib_store_save() != RT_EOK)
                LOG_W("calib: migrate save failed (RAM mirror kept)");
        }
        else
        {
            LOG_I("calib: no valid record (cold start)");
        }
    }

    param_sys_set_calib_migrated();
    s_calib.inited = RT_TRUE;
    return RT_EOK;
}

struct calib_data *calib_store_ram(void)
{
    (void)calib_store_init();
    return &s_calib.data;
}

rt_err_t calib_store_save(void)
{
    rt_uint8_t payload[CALIB_PAYLOAD_SIZE];

    (void)calib_store_init();
    pack_payload(payload);
    return param_part_save(PARAM_PART_CALIB, payload, sizeof(payload));
}

rt_err_t calib_store_erase(void)
{
    rt_err_t err = param_part_erase(PARAM_PART_CALIB);

    if (err == RT_EOK)
        memset(&s_calib.data, 0, sizeof(s_calib.data));
    return err;
}
