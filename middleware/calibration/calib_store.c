/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 校准参数持久化实现, 布局与 Flash 操作说明见 calib_store.h。
 */

#include <rtthread.h>
#include <string.h>

#include "board.h"                     /* -> stm32h7xx_hal.h -> CMSIS 设备头 */

#include "calib_store.h"
#include "timebase.h"               /* middleware/timebase 擦写窗口回绕核对 */

#define LOG_TAG "calib"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

/* ------------------------- 存储布局 ------------------------- */

#define CALIB_FLASH_BASE         0x08000000UL
#define CALIB_FLASH_SECTOR       7u                          /* 0x080E0000 */
#define CALIB_FLASH_ADDR         (CALIB_FLASH_BASE + 0xE0000UL)
#define CALIB_FLASH_SIZE         (128u * 1024u)              /* H723 末扇区 128KB */
#define CALIB_RECORD_SIZE        128u                        /* 4 x 256-bit flash word */
#define CALIB_SLOT_NUM           (CALIB_FLASH_SIZE / CALIB_RECORD_SIZE)   /* 1024 槽 */
#define CALIB_PAYLOAD_OFF        16u

#define CALIB_MAGIC              0x4C41434DUL               /* 'M','C','A','L' 小端 */
#define CALIB_VERSION            1u

/* 记录头: magic u32 | ver u16 | flags u16 | seq u32 | crc32 u32, 共 16 字节。
 * seq 为单调记录序号 (追加式日志定位最新记录用); CRC 覆盖 ver/flags/seq
 * 与参数体两段 (旧格式 crc 在偏移 8 且只盖参数体, flags 位翻转不被保护) */
#define CALIB_SEQ_OFF            8u
#define CALIB_CRC_OFF            12u

/* 记录头: magic u32 | ver u16 | flags u16 | crc32 u32 | rsv u32, 共 16 字节 */
#define CALIB_FLAG_MAG           0x0001u
#define CALIB_FLAG_BARO          0x0002u
#define CALIB_FLAG_ACC           0x0004u

/* 参数体字段偏移 (128 字节记录内, 均为小端 f32/u16) */
#define COF_MAG_BIAS             16     /* f32 x3  硬磁偏置 µT */
#define COF_MAG_SOFTIRON         28     /* f32 x9  软磁矩阵, 行主序 */
#define COF_MAG_RADIUS           64     /* f32    等效半径 µT */
#define COF_MAG_RESID            68     /* f32    拟合残差 */
#define COF_MAG_MAXRATIO         72     /* f32    主轴半径比 */
#define COF_MAG_SAMPLES          76     /* u16    样本数 */
#define COF_BARO_OFFSET          80     /* f32    压强偏移 Pa */
#define COF_BARO_CALTEMP         84     /* f32    标定温度 °C */
#define COF_BARO_MEAN            88     /* f32    标定均值 Pa */
/* C8 加计零偏 (记录 128B, 头 16 + 参数体至 92 已用, 96 起空闲) */
#define COF_ACC_BIAS             96     /* f32 x3  加计零偏 mGal (FRD) */
#define COF_ACC_TEMP             108    /* f32    采集时 IMU 温度 °C */

/* ------------------------- RAM 镜像 ------------------------- */

static struct
{
    struct calib_data data;
    rt_bool_t         inited;
    rt_uint32_t       seq;                    /* 最新有效记录的序号 (下条 = +1) */
    rt_uint32_t       write_idx;              /* 下一条追加写槽位 (== SLOT_NUM = 满) */
    struct rt_mutex   lock;                   /* 串行化 pack+save/erase (跨线程) */
    rt_bool_t         lock_ok;
} s_store;

/* ------------------------- 小工具 ------------------------- */

/* CRC32 原始更新 (反射, 多项式 0xEDB88320, 与 zlib/PNG 一致), 可跨段链式 */
static rt_uint32_t calib_crc32_update(rt_uint32_t crc, const rt_uint8_t *p,
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

static rt_uint32_t calib_crc32(const rt_uint8_t *p, rt_size_t len)
{
    return calib_crc32_update(0xFFFFFFFFUL, p, len) ^ 0xFFFFFFFFUL;
}

/* 新格式记录 CRC: 覆盖 ver/flags/seq [4,12) + 参数体 [16,128) */
static rt_uint32_t calib_record_crc(const rt_uint8_t *rec)
{
    rt_uint32_t crc = calib_crc32_update(0xFFFFFFFFUL, rec + 4u,
                                         CALIB_SEQ_OFF - 4u);

    crc = calib_crc32_update(crc, rec + CALIB_PAYLOAD_OFF,
                             CALIB_RECORD_SIZE - CALIB_PAYLOAD_OFF);
    return crc ^ 0xFFFFFFFFUL;
}

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

/* ------------------------- Flash 寄存器级操作 ------------------------- */

/* 真错误位 (检查用)。注意 EOP(bit16) 是"编程完成"成功标志, 不属错误 ——
 * 误入检查掩码会把每个成功编程的 word 判为失败 (实测 word 已写入 flash,
 * save 却中止, 只留首个 32B word 的残卷) */
#define FLASH_ERR_ALL   (FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR | \
                         FLASH_SR_INCERR | FLASH_SR_RDPERR  | FLASH_SR_RDSERR  | \
                         FLASH_SR_SNECCERR | FLASH_SR_DBECCERR | FLASH_SR_OPERR)
/* 写 1 清零用: 错误位 + EOP (顺手清成功标志) */
#define FLASH_ERR_MASK  (FLASH_ERR_ALL | FLASH_SR_EOP)

/* SWD 调试: 最近一次 flash 操作失败现场 (无串口环境下定位编程失败原因) */
volatile struct
{
    rt_uint32_t stage;      /* 1=unlock 2=erase 3=program 4=verify */
    rt_uint32_t word;       /* 32B flash word 序号 */
    rt_uint32_t sr1;        /* 失败瞬间的 SR1 错误位快照 (清零前) */
    rt_uint32_t pad;
} g_calib_dbg;

/* B5 打开 DCache 后, flash 数据区 (Normal/WT/cacheable) 的写 miss 会先
 * 回读填充 cache line —— 多 word 连续编程时, 后续 word 的填充读撞上
 * 上一 word 的 BSY (read-while-write 违例), 编程中途失败 (实测只写入
 * 首个 32B word); 读回校验同样会命中陈旧缓存行误判失败。
 * 对策: 编程/擦除窗口内整体关 DCache (clean+invalidate), 结束全无效后
 * 重开 —— 写直通无 line 填充, 校验读真实 flash。窗口由互斥锁串行化,
 * 关中断段不变。 */
static void calib_cache_off(void)
{
    SCB_DisableDCache();
}

static void calib_cache_on(void)
{
    SCB_InvalidateDCache();
    SCB_EnableDCache();
}

/* 等待 BSY/QW 清零, 迭代上限约折合数秒 (128KB 扇区擦除典型 ~1s) */
static rt_err_t flash_wait_idle(void)
{
    volatile rt_uint64_t n = 0;

    while ((FLASH->SR1 & (FLASH_SR_BSY | FLASH_SR_QW | FLASH_SR_WBNE)) != 0u)
    {
        if (++n > ((rt_uint64_t)3u << 30))
            return -RT_ETIMEOUT;
    }
    return RT_EOK;
}

static void flash_clear_errors(void)
{
    FLASH->SR1 = FLASH_ERR_MASK;          /* 写 1 清零 */
}

static rt_err_t flash_unlock(void)
{
    if ((FLASH->CR1 & FLASH_CR_LOCK) == 0u)
        return RT_EOK;

    FLASH->KEYR1 = 0x45670123U;
    FLASH->KEYR1 = 0xCDEF89ABU;
    __DSB();
    __ISB();

    return (FLASH->CR1 & FLASH_CR_LOCK) ? (-RT_ERROR) : RT_EOK;
}

static void flash_lock(void)
{
    FLASH->CR1 |= FLASH_CR_LOCK;
}

static rt_err_t flash_erase_sector7(void)
{
    rt_err_t err;

    if ((err = flash_wait_idle()) != RT_EOK)
    {
        g_calib_dbg.stage = 2u;
        g_calib_dbg.sr1   = FLASH->SR1;
        return err;
    }
    flash_clear_errors();

    FLASH->CR1 &= ~(FLASH_CR_PG | FLASH_CR_SER | FLASH_CR_SNB | FLASH_CR_BER);
    FLASH->CR1 |= (CALIB_FLASH_SECTOR << FLASH_CR_SNB_Pos) | FLASH_CR_SER;
    FLASH->CR1 |= FLASH_CR_START;

    err = flash_wait_idle();

    if (err == RT_EOK && (FLASH->SR1 & FLASH_ERR_ALL) != 0u)
        err = -RT_ERROR;
    if (err != RT_EOK)
    {
        g_calib_dbg.stage = 2u;
        g_calib_dbg.sr1   = FLASH->SR1;
    }

    FLASH->CR1 &= ~(FLASH_CR_SER | FLASH_CR_SNB);
    return err;
}

/* buf 长度须为 32 的倍数, 地址 32B 对齐, 按 256-bit flash word 逐行编程 */
static rt_err_t flash_program(rt_uint32_t addr, const rt_uint8_t *buf,
                              rt_uint32_t len)
{
    for (rt_uint32_t off = 0; off < len; off += 32u)
    {
        const volatile rt_uint64_t *src = (const rt_uint64_t *)(const void *)(buf + off);
        volatile rt_uint64_t *dst = (rt_uint64_t *)(addr + off);
        rt_err_t err;

        if ((err = flash_wait_idle()) != RT_EOK)
        {
            g_calib_dbg.stage = 3u;
            g_calib_dbg.word  = off / 32u;
            g_calib_dbg.sr1   = FLASH->SR1;
            return err;
        }
        flash_clear_errors();

        FLASH->CR1 |= FLASH_CR_PG;

        /* 连续写 4 个 64 位填满一个 flash word, 其间取指停等属正常 */
        dst[0] = src[0];
        dst[1] = src[1];
        dst[2] = src[2];
        dst[3] = src[3];

        err = flash_wait_idle();

        if (err == RT_EOK && (FLASH->SR1 & FLASH_ERR_ALL) != 0u)
            err = -RT_ERROR;
        if (err != RT_EOK)
        {
            g_calib_dbg.stage = 3u;
            g_calib_dbg.word  = off / 32u;
            g_calib_dbg.sr1   = FLASH->SR1;
        }

        FLASH->CR1 &= ~FLASH_CR_PG;
        flash_clear_errors();

        if (err != RT_EOK)
            return err;
    }
    return RT_EOK;
}

/* ------------------------- 序列化 ------------------------- */

static void pack_record(rt_uint8_t *rec, rt_uint32_t seq)
{
    struct calib_data *d = &s_store.data;
    rt_uint16_t flags = 0;

    memset(rec, 0, CALIB_RECORD_SIZE);

    if (d->mag_valid)
        flags |= CALIB_FLAG_MAG;
    if (d->baro_valid)
        flags |= CALIB_FLAG_BARO;
    if (d->acc_valid)
        flags |= CALIB_FLAG_ACC;

    for (int i = 0; i < 3; i++)
        put_u32(rec + COF_MAG_BIAS + 4u * i, f2u(d->mag_bias_ut[i]));
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            put_u32(rec + COF_MAG_SOFTIRON + 12u * i + 4u * j, f2u(d->mag_softiron[i][j]));
    put_u32(rec + COF_MAG_RADIUS,   f2u(d->mag_radius_ut));
    put_u32(rec + COF_MAG_RESID,    f2u(d->mag_resid));
    put_u32(rec + COF_MAG_MAXRATIO, f2u(d->mag_maxratio));
    put_u16(rec + COF_MAG_SAMPLES,  d->mag_samples);

    put_u32(rec + COF_BARO_OFFSET,  f2u(d->baro_offset_pa));
    put_u32(rec + COF_BARO_CALTEMP, f2u(d->baro_cal_temp));
    put_u32(rec + COF_BARO_MEAN,    f2u(d->baro_mean_pa));
    for (int i = 0; i < 3; i++)
        put_u32(rec + COF_ACC_BIAS + 4u * i, f2u(d->acc_bias_mgal[i]));
    put_u32(rec + COF_ACC_TEMP,     f2u(d->acc_cal_temp));

    put_u32(rec + 0, CALIB_MAGIC);
    put_u16(rec + 4, CALIB_VERSION);
    put_u16(rec + 6, flags);
    put_u32(rec + CALIB_SEQ_OFF, seq);
    put_u32(rec + CALIB_CRC_OFF, calib_record_crc(rec));
}

/* 纯字段解析 (无校验, 仅供 record_valid 通过后调用) */
static void unpack_record(const rt_uint8_t *rec)
{
    struct calib_data *d = &s_store.data;
    rt_uint16_t flags = get_u16(rec + 6);

    d->mag_valid = (flags & CALIB_FLAG_MAG) ? RT_TRUE : RT_FALSE;
    for (int i = 0; i < 3; i++)
        d->mag_bias_ut[i] = u2f(get_u32(rec + COF_MAG_BIAS + 4u * i));
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            d->mag_softiron[i][j] = u2f(get_u32(rec + COF_MAG_SOFTIRON + 12u * i + 4u * j));
    d->mag_radius_ut = u2f(get_u32(rec + COF_MAG_RADIUS));
    d->mag_resid     = u2f(get_u32(rec + COF_MAG_RESID));
    d->mag_maxratio  = u2f(get_u32(rec + COF_MAG_MAXRATIO));
    d->mag_samples   = get_u16(rec + COF_MAG_SAMPLES);

    d->baro_valid    = (flags & CALIB_FLAG_BARO) ? RT_TRUE : RT_FALSE;
    d->baro_offset_pa = u2f(get_u32(rec + COF_BARO_OFFSET));
    d->baro_cal_temp  = u2f(get_u32(rec + COF_BARO_CALTEMP));
    d->baro_mean_pa   = u2f(get_u32(rec + COF_BARO_MEAN));

    d->acc_valid = (flags & CALIB_FLAG_ACC) ? RT_TRUE : RT_FALSE;
    for (int i = 0; i < 3; i++)
        d->acc_bias_mgal[i] = u2f(get_u32(rec + COF_ACC_BIAS + 4u * i));
    d->acc_cal_temp = u2f(get_u32(rec + COF_ACC_TEMP));
}

/*
 * 记录有效性检查: 新格式 (seq + CRC 覆盖 ver/flags/seq/参数体) 或
 * 旧格式兼容 (升级前的记录: crc 在偏移 8 只盖参数体, 偏移 12 保留 0)。
 * 通过时 *seq 输出记录序号 (旧格式恒 0)。
 * 两种格式互相误判的概率: 新记录被旧分支接受需新 CRC==0 (2^-32),
 * 旧记录被新分支接受需凑巧匹配 (2^-32), 忽略。
 */
static rt_bool_t record_valid(const rt_uint8_t *rec, rt_uint32_t *seq)
{
    if (get_u32(rec + 0) != CALIB_MAGIC || get_u16(rec + 4) != CALIB_VERSION)
        return RT_FALSE;

    if (get_u32(rec + CALIB_CRC_OFF) == calib_record_crc(rec))
    {
        *seq = get_u32(rec + CALIB_SEQ_OFF);
        return RT_TRUE;
    }

    if (get_u32(rec + 12u) == 0u &&
        get_u32(rec + 8u) == calib_crc32(rec + CALIB_PAYLOAD_OFF,
                                         CALIB_RECORD_SIZE - CALIB_PAYLOAD_OFF))
    {
        *seq = 0u;
        return RT_TRUE;
    }
    return RT_FALSE;
}

/* ------------------------- 对外接口 ------------------------- */

rt_err_t calib_store_init(void)
{
    if (s_store.inited)
        return RT_EOK;

    memset(&s_store.data, 0, sizeof(s_store.data));
    s_store.seq = 0;
    s_store.write_idx = CALIB_SLOT_NUM;      /* 扫描前视为满 */
    s_store.lock_ok = (rt_mutex_init(&s_store.lock, "calib",
                                     RT_IPC_FLAG_FIFO) == RT_EOK);

    /*
     * 追加式日志扫描: 逐槽检查, 取 "CRC 通过 且 seq 最大" 的记录为最新;
     * 首个未编程槽 (前 8 字节全 FF) 为追加写位置。编程按 32B 行顺序,
     * 掉电撕裂只会留下 "前缀已写" 的坏记录 (CRC 拦下, 跳过), 其后槽
     * 仍可用 —— 掉电窗口内旧记录始终在位, 这是追加式对 "先擦后写"
     * 的核心优势。升级前的旧格式单记录由兼容分支读取。
     */
    {
        rt_uint8_t rec[CALIB_RECORD_SIZE];
        rt_uint32_t best_seq = 0;
        rt_bool_t have_best = RT_FALSE;

        for (rt_uint32_t i = 0; i < CALIB_SLOT_NUM; i++)
        {
            const volatile rt_uint8_t *p = (const volatile rt_uint8_t *)
                (CALIB_FLASH_ADDR + i * CALIB_RECORD_SIZE);
            rt_uint32_t seq = 0;

            if (p[0] == 0xFFu && p[1] == 0xFFu && p[2] == 0xFFu &&
                p[3] == 0xFFu && p[4] == 0xFFu && p[5] == 0xFFu &&
                p[6] == 0xFFu && p[7] == 0xFFu)
            {
                s_store.write_idx = i;       /* 首个空闲槽 */
                break;
            }

            memcpy(rec, (const void *)p, CALIB_RECORD_SIZE);
            if (record_valid(rec, &seq) &&
                (!have_best || (rt_int32_t)(seq - best_seq) > 0))
            {
                unpack_record(rec);
                best_seq = seq;
                have_best = RT_TRUE;
            }
        }
        s_store.seq = best_seq;

        if (have_best)
            LOG_I("calib: record loaded (slot seq=%u, next write slot=%u%s)",
                  best_seq, s_store.write_idx,
                  best_seq == 0u ? ", legacy format" : "");
        else
            LOG_I("calib: no valid record (sector blank/cold start)");
    }

    s_store.inited = RT_TRUE;
    return RT_EOK;
}

struct calib_data *calib_store_ram(void)
{
    return &s_store.data;
}

rt_err_t calib_store_save(void)
{
    rt_uint8_t rec[CALIB_RECORD_SIZE] __attribute__((aligned(32)));
    rt_uint32_t addr;
    rt_err_t err = RT_EOK;
    rt_bool_t erased = RT_FALSE;

    (void)calib_store_init();
    if (s_store.lock_ok)
        rt_mutex_take(&s_store.lock, RT_WAITING_FOREVER);

    /* pack 与写入在同一锁内: mag/baro 线程并发 save 时, 避免用旧打包
     * 快照覆盖对方刚写入 flash 的参数 (关中断只保 flash 操作原子,
     * pack 在锁外仍会读到中间态) */
    pack_record(rec, s_store.seq + 1u);

    g_calib_dbg.stage = 0u;
    g_calib_dbg.word  = 0u;
    g_calib_dbg.sr1   = 0u;

    /* 编程期间 DCache 必须关闭 (见 calib_cache_off 注释) */
    calib_cache_off();

    while (err == RT_EOK)
    {
        rt_base_t level;
        rt_uint64_t t_win;

        /* 扇区写满: 整擦一次重写槽 0。该窗口掉电会丢全部记录 —— 发生
         * 频率为旧 "每次 save 都整擦" 实现的 1/1024, 且擦+写收在同一
         * 关中断窗口内最小化暴露时长 */
        if (s_store.write_idx >= CALIB_SLOT_NUM && !erased)
        {
            t_win = timebase_irq_window_begin();
            level = rt_hw_interrupt_disable();

            if ((err = flash_unlock()) == RT_EOK)
            {
                err = flash_erase_sector7();
                flash_lock();
            }

            rt_hw_interrupt_enable(level);
            timebase_irq_window_end(t_win);

            if (err == RT_EOK)
            {
                s_store.write_idx = 0;
                erased = RT_TRUE;
            }
            else
                break;
        }

        /* 追加写新记录 (不动旧记录): 掉电至多损失这一条 */
        addr = CALIB_FLASH_ADDR + s_store.write_idx * CALIB_RECORD_SIZE;
        t_win = timebase_irq_window_begin();
        level = rt_hw_interrupt_disable();

        if ((err = flash_unlock()) == RT_EOK)
        {
            err = flash_program(addr, rec, CALIB_RECORD_SIZE);
            flash_lock();
        }

        rt_hw_interrupt_enable(level);
        timebase_irq_window_end(t_win);
        break;
    }

    /* 校验读回在 DCache 关闭下进行, 读的是真实 flash (见函数头注释) */
    if (err == RT_EOK)
    {
        const volatile rt_uint8_t *chk = (const volatile rt_uint8_t *)addr;
        rt_bool_t ok = RT_TRUE;

        for (rt_uint32_t i = 0; i < CALIB_RECORD_SIZE && ok; i++)
            if (chk[i] != rec[i])
                ok = RT_FALSE;

        if (!ok)
        {
            g_calib_dbg.stage = 4u;
            g_calib_dbg.word  = 0xFFFFFFFFu;    /* 校验失败: 记录级 */
            g_calib_dbg.sr1   = 0u;
            LOG_E("calib: flash verify failed");
            err = -RT_ERROR;
        }
    }

    calib_cache_on();

    if (err == RT_EOK)
    {
        s_store.write_idx++;
        s_store.seq++;
    }
    else
        LOG_E("calib: flash write failed (%d)", (int)err);

    if (s_store.lock_ok)
        rt_mutex_release(&s_store.lock);
    return err;
}

rt_err_t calib_store_erase(void)
{
    rt_err_t err;
    rt_base_t level;
    rt_uint64_t t_win;

    (void)calib_store_init();
    if (s_store.lock_ok)
        rt_mutex_take(&s_store.lock, RT_WAITING_FOREVER);

    calib_cache_off();              /* 擦除窗口与 save 同规则 */

    t_win = timebase_irq_window_begin();
    level = rt_hw_interrupt_disable();

    if ((err = flash_unlock()) == RT_EOK)
    {
        err = flash_erase_sector7();
        flash_lock();
    }

    rt_hw_interrupt_enable(level);
    timebase_irq_window_end(t_win);

    calib_cache_on();

    if (err == RT_EOK)
    {
        memset(&s_store.data, 0, sizeof(s_store.data));
        s_store.seq = 0;
        s_store.write_idx = 0;
    }

    if (s_store.lock_ok)
        rt_mutex_release(&s_store.lock);
    return err;
}
