/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * W25Q64 参数分区存储引擎实现, 分区布局/记录格式见 param_part.h。
 *
 * 分层: 本引擎只管 "槽位追加 + CRC 校验 + 回卷整擦" 的通用记录纪律,
 * 各参数域 (calib/nav/sys) 的字段序列化由域模块完成 (param_calib.c 等)。
 * 总线串行化由 sensor_w25q64.c 的器件互斥锁保证, 本层再加引擎锁把
 * "打包+写入" 与 "扫描/擦除" 串行化 (与原 calib_store 相同的并发模型)。
 */

#include <rtthread.h>
#include <string.h>

#include "sensor_w25q64.h"
#include "param_part.h"

#define LOG_TAG "param"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

/* ------------------------- 分区定义 (编译期权威) ------------------------- */

/* 名字段不足 8 字节由 C 数组初始化自动零填充 */
static const struct param_part_def s_defs[PARAM_PART_NUM] =
{
    {"ptbl",  PARAM_PART_PTBL,  0,    0,    0x000000u, 4u * 1024u},
    {"calib", PARAM_PART_CALIB, 1,    128u, 0x001000u, 16u * 1024u},
    {"nav",   PARAM_PART_NAV,   1,    128u, 0x005000u, 16u * 1024u},
    {"sys",   PARAM_PART_SYS,   1,    64u,  0x009000u, 8u * 1024u},
    {"ctrl",  PARAM_PART_CTRL,  1,    256u, 0x00B000u, 20u * 1024u},
};

/* 引擎支持的最大槽长 (栈缓冲上限, 表内槽长不得超过) */
#define PARAM_SLOT_MAX          256u

/* 记录头: magic u32 | ver u16 | part u16 | seq u32 | len u16 | rsv u16
 *         | crc u32, 共 20 字节; CRC 覆盖 [+4,+16) + payload */
#define REC_MAGIC               0x4D524150UL              /* 'P','A','R','M' */
#define REC_FMT_VER             1u
#define REC_HDR_SIZE            20u
#define REC_OFF_VER             4u
#define REC_OFF_PART            6u
#define REC_OFF_SEQ             8u
#define REC_OFF_LEN             12u
#define REC_OFF_CRC             16u

/* 扇区 0 分区表副本: magic u32 | ver u16 | count u16 | crc u32 (覆盖
 * [+4,+8) + 条目区), 条目 24B: name[8] addr u32 size u32 slot u16
 * id u8 record u8 rsv u16 —— 自描述, 仅供工具/自检, 不作为权威 */
#define PTBL_MAGIC              0x4C425450UL              /* 'P','T','B','L' */
#define PTBL_VER                1u
#define PTBL_ENTRY_SIZE         24u
#define PTBL_IMAGE_SIZE         (12u + PTBL_ENTRY_SIZE * PARAM_PART_NUM)

/* ------------------------- 引擎状态 ------------------------- */

struct part_state
{
    struct param_part_stat stat;
    rt_uint32_t            slot_num;   /* 分区槽数 (记录区) */
    rt_bool_t              broken;     /* 扫描期读失败: 本次上电禁回卷擦写 */
};

static struct
{
    rt_bool_t           inited;        /* init 已执行 (结果无关) */
    rt_bool_t           ready;         /* W25Q64 在位且扫描完成 */
    struct rt_mutex     lock;
    rt_bool_t           lock_ok;
    struct part_state   part[PARAM_PART_NUM];
} s_eng;

/* ------------------------- 小工具 ------------------------- */

/* CRC32 (反射, 多项式 0xEDB88320, 与 zlib/PNG 一致), 可跨段链式 */
static rt_uint32_t param_crc32_update(rt_uint32_t crc, const rt_uint8_t *p,
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

static rt_uint32_t param_crc32(const rt_uint8_t *p, rt_size_t len)
{
    return param_crc32_update(0xFFFFFFFFUL, p, len) ^ 0xFFFFFFFFUL;
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

/* 记录 CRC: 覆盖头 [+4,+16) 与 payload[0,len) */
static rt_uint32_t record_crc(const rt_uint8_t *rec, rt_uint16_t len)
{
    rt_uint32_t crc = param_crc32_update(0xFFFFFFFFUL, rec + REC_OFF_VER,
                                         REC_OFF_CRC - REC_OFF_VER);

    crc = param_crc32_update(crc, rec + REC_HDR_SIZE, len);
    return crc ^ 0xFFFFFFFFUL;
}

/*
 * 槽内记录有效性检查: 槽空闲 (头 4 字节全 FF) 输出 *free; 记录合法
 * 输出 *seq 与 *plen。len 越界 (超槽容量) 直接判坏, 防止 CRC 计算越槽。
 */
static rt_bool_t slot_parse(const rt_uint8_t *slot, rt_uint16_t slot_size,
                            rt_bool_t *free, rt_uint32_t *seq,
                            rt_uint16_t *plen)
{
    rt_uint16_t len;

    *free = RT_FALSE;
    if (slot[0] == 0xFFu && slot[1] == 0xFFu &&
        slot[2] == 0xFFu && slot[3] == 0xFFu)
    {
        *free = RT_TRUE;
        return RT_FALSE;
    }

    len = get_u16(slot + REC_OFF_LEN);
    if (get_u32(slot) != REC_MAGIC ||
        get_u16(slot + REC_OFF_VER) != REC_FMT_VER ||
        len == 0u || len > slot_size - REC_HDR_SIZE)
        return RT_FALSE;
    if (get_u32(slot + REC_OFF_CRC) != record_crc(slot, len))
        return RT_FALSE;

    *seq  = get_u32(slot + REC_OFF_SEQ);
    *plen = len;
    return RT_TRUE;
}

/* ------------------------- 分区表副本 (扇区 0) ------------------------- */

static void ptbl_pack(rt_uint8_t *img)
{
    memset(img, 0xFF, PTBL_IMAGE_SIZE);
    put_u32(img + 0, PTBL_MAGIC);
    put_u16(img + 4, PTBL_VER);
    put_u16(img + 6, PARAM_PART_NUM);

    for (rt_uint32_t i = 0; i < PARAM_PART_NUM; i++)
    {
        rt_uint8_t *e = img + 12u + i * PTBL_ENTRY_SIZE;

        memcpy(e, s_defs[i].name, sizeof(s_defs[i].name));
        put_u32(e + 8, s_defs[i].addr);
        put_u32(e + 12, s_defs[i].size);
        put_u16(e + 16, s_defs[i].slot_size);
        e[18] = s_defs[i].id;
        e[19] = s_defs[i].record_part;
        put_u16(e + 20, 0);
    }

    put_u32(img + 8, param_crc32(img + 4, 4u + PTBL_ENTRY_SIZE * PARAM_PART_NUM));
}

/* 校对扇区 0 副本与编译期表: 空白/损坏/布局变更时重写 (一次性擦写) */
static rt_err_t ptbl_sync(void)
{
    rt_uint8_t img[PTBL_IMAGE_SIZE];
    rt_uint8_t cur[PTBL_IMAGE_SIZE];
    rt_err_t err;

    ptbl_pack(img);

    err = w25q64_read(s_defs[PARAM_PART_PTBL].addr, cur, sizeof(cur));
    if (err == RT_EOK && memcmp(cur, img, sizeof(img)) == 0)
        return RT_EOK;                          /* 副本一致 */

    if (err == RT_EOK &&
        get_u32(cur) == PTBL_MAGIC &&
        get_u16(cur + 4) == PTBL_VER)
        LOG_I("partition layout changed, refreshing sector 0 copy");
    else
        LOG_I("writing partition table copy to sector 0 (blank/stale)");

    if ((err = w25q64_erase_sector(s_defs[PARAM_PART_PTBL].addr)) != RT_EOK)
        return err;
    return w25q64_write(s_defs[PARAM_PART_PTBL].addr, img, sizeof(img));
}

/* ------------------------- 记录分区扫描 ------------------------- */

/*
 * 逐 4KB 扇区批量读入扫描 (malloc 失败时退化为逐槽读): 取 "CRC 通过
 * 且 seq 最大" 的记录为最新, 首个空闲槽为追加写位置。撕裂记录 (非 FF
 * 头 + CRC 失败) 跳过继续 —— 追加式日志里其后仍可能有后写的有效记录。
 */
static rt_err_t scan_record_part(rt_uint8_t id)
{
    const struct param_part_def *def = &s_defs[id];
    struct part_state *ps = &s_eng.part[id];
    rt_uint8_t *sec = RT_NULL;
    rt_uint32_t slot_num = def->size / def->slot_size;
    rt_uint32_t best_seq = 0;
    rt_bool_t have_best = RT_FALSE;

    ps->slot_num = slot_num;
    ps->stat.write_idx = slot_num;               /* 扫描前视为满 */
    ps->stat.have_record = RT_FALSE;
    ps->stat.seq = 0;
    ps->stat.last_len = 0;

    if (def->size % W25Q64_SECTOR_SIZE != 0u ||
        W25Q64_SECTOR_SIZE % def->slot_size != 0u)
        return -RT_EINVAL;                       /* 槽不得跨扇区 */

    if (def->slot_size <= PARAM_SLOT_MAX)
        sec = rt_malloc(W25Q64_SECTOR_SIZE);

    for (rt_uint32_t i = 0; i < slot_num; i++)
    {
        rt_uint8_t stack[PARAM_SLOT_MAX];
        const rt_uint8_t *slot;
        rt_bool_t is_free = RT_FALSE;
        rt_uint32_t seq = 0;
        rt_uint16_t plen = 0;
        rt_bool_t ok;

        if (sec != RT_NULL)
        {
            if (i % (W25Q64_SECTOR_SIZE / def->slot_size) == 0u)
            {
                if (w25q64_read(def->addr + (i / (W25Q64_SECTOR_SIZE /
                     def->slot_size)) * W25Q64_SECTOR_SIZE,
                                sec, W25Q64_SECTOR_SIZE) != RT_EOK)
                {
                    /* 读失败: 标记 broken, 本次上电拒绝回卷擦写 —— 分区内
                     * 可能仍有完好记录, 不允许 save 的整擦路径毁掉它们 */
                    ps->broken = RT_TRUE;
                    LOG_W("part %s sector read failed during scan",
                          def->name);
                    break;
                }
            }
            slot = sec + (i % (W25Q64_SECTOR_SIZE / def->slot_size)) *
                   def->slot_size;
        }
        else
        {
            if (w25q64_read(def->addr + i * def->slot_size,
                            stack, def->slot_size) != RT_EOK)
            {
                ps->broken = RT_TRUE;
                LOG_W("part %s slot read failed during scan", def->name);
                break;
            }
            slot = stack;
        }

        ok = slot_parse(slot, def->slot_size, &is_free, &seq, &plen);
        if (is_free)
        {
            ps->stat.write_idx = i;              /* 首个空闲槽 */
            break;
        }
        if (ok && (!have_best || (rt_int32_t)(seq - best_seq) > 0))
        {
            best_seq = seq;
            have_best = RT_TRUE;
            ps->stat.best_slot = i;
            ps->stat.last_len = plen;
        }
    }

    if (sec != RT_NULL)
        rt_free(sec);

    if (have_best)
    {
        ps->stat.have_record = RT_TRUE;
        ps->stat.seq = best_seq;
    }
    return RT_EOK;
}

/* ------------------------- 对外接口 ------------------------- */

const struct param_part_def *param_part_def(rt_uint8_t id)
{
    return (id < PARAM_PART_NUM) ? &s_defs[id] : RT_NULL;
}

rt_err_t param_store_init(void)
{
    rt_uint8_t id[3];

    if (s_eng.inited)
        return s_eng.ready ? RT_EOK : -RT_EIO;

    s_eng.inited = RT_TRUE;
    s_eng.lock_ok = (rt_mutex_init(&s_eng.lock, "paramst",
                                   RT_IPC_FLAG_FIFO) == RT_EOK);

    if (!w25q64_is_ready())
    {
        LOG_E("W25Q64 not ready, parameter storage disabled "
              "(calib/nav/sys keep RAM defaults, save unavailable)");
        return -RT_EIO;
    }
    if (w25q64_read_jedec_id(id) != RT_EOK)
    {
        LOG_E("W25Q64 JEDEC read failed, parameter storage disabled");
        return -RT_EIO;
    }

    if (ptbl_sync() != RT_EOK)
        LOG_W("partition table copy write failed (params still usable)");

    for (rt_uint32_t p = 0; p < PARAM_PART_NUM; p++)
    {
        if (!s_defs[p].record_part)
            continue;
        if (scan_record_part((rt_uint8_t)p) != RT_EOK)
            LOG_W("partition %s scan failed", s_defs[p].name);
        else if (s_eng.part[p].stat.have_record)
            LOG_I("part %-5s: record seq=%u slot=%u/%u", s_defs[p].name,
                  (unsigned)s_eng.part[p].stat.seq,
                  (unsigned)s_eng.part[p].stat.best_slot,
                  (unsigned)s_eng.part[p].slot_num);
        else
            LOG_I("part %-5s: empty", s_defs[p].name);
    }

    s_eng.ready = RT_TRUE;
    return RT_EOK;
}

rt_bool_t param_store_ready(void)
{
    return s_eng.ready;
}

rt_err_t param_part_save(rt_uint8_t id, const rt_uint8_t *payload,
                         rt_uint16_t len)
{
    const struct param_part_def *def = param_part_def(id);
    struct part_state *ps;
    rt_uint8_t rec[PARAM_SLOT_MAX];
    rt_uint8_t chk[PARAM_SLOT_MAX];
    rt_uint32_t seq_new;
    rt_err_t err = RT_EOK;

    if (def == RT_NULL || !def->record_part || payload == RT_NULL || len == 0u)
        return -RT_EINVAL;
    if (len > def->slot_size - REC_HDR_SIZE || def->slot_size > sizeof(rec))
        return -RT_EFULL;

    (void)param_store_init();
    if (s_eng.lock_ok)
        rt_mutex_take(&s_eng.lock, RT_WAITING_FOREVER);

    ps = &s_eng.part[id];
    if (!s_eng.ready)
    {
        err = -RT_EIO;
        goto out;
    }
    if (ps->broken)
    {
        /* 扫描期读失败的分区不写 (可能触发整擦毁掉完好记录), 下次上电重扫 */
        LOG_E("part %s marked broken at scan, save refused", def->name);
        err = -RT_EIO;
        goto out;
    }

    /* 打包与写入同锁: 与原 calib_store 相同的并发模型, 防两个域线程
     * 用旧快照覆盖对方刚写入的记录 */
    seq_new = ps->stat.seq + 1u;
    memset(rec, 0xFF, def->slot_size);
    memcpy(rec + REC_HDR_SIZE, payload, len);
    put_u32(rec + 0, REC_MAGIC);
    put_u16(rec + REC_OFF_VER, REC_FMT_VER);
    put_u16(rec + REC_OFF_PART, id);
    put_u32(rec + REC_OFF_SEQ, seq_new);
    put_u16(rec + REC_OFF_LEN, len);
    put_u16(rec + REC_OFF_LEN + 2u, 0u);
    put_u32(rec + REC_OFF_CRC, record_crc(rec, len));

    /* 分区写满: 逐扇区整擦后回写槽 0。该窗口掉电丢整分区记录 ——
     * 发生频率 = 1/槽数 每次保存 (calib 128 槽/nav 128/sys 128/ctrl 80),
     * 且擦写全程不关中断 (OCTOSPI 间接模式, CPU 照常运行) */
    if (ps->stat.write_idx >= ps->slot_num)
    {
        for (rt_uint32_t off = 0; off < def->size; off += W25Q64_SECTOR_SIZE)
        {
            if ((err = w25q64_erase_sector(def->addr + off)) != RT_EOK)
            {
                LOG_E("part %s wrap erase failed (%d)", def->name, (int)err);
                goto out;
            }
        }
        ps->stat.write_idx = 0;
    }

    err = w25q64_write(def->addr + ps->stat.write_idx * def->slot_size,
                       rec, REC_HDR_SIZE + len);
    if (err == RT_EOK)
    {
        /* 读回校验: 撕裂/位坏当场发现 (槽尾保持 FF 不比对) */
        err = w25q64_read(def->addr + ps->stat.write_idx * def->slot_size,
                          chk, REC_HDR_SIZE + len);
        if (err == RT_EOK && memcmp(chk, rec, REC_HDR_SIZE + len) != 0)
        {
            LOG_E("part %s verify failed", def->name);
            err = -RT_ERROR;
        }
    }

    if (err == RT_EOK)
    {
        ps->stat.seq = seq_new;
        ps->stat.best_slot = ps->stat.write_idx;
        ps->stat.last_len = len;
        ps->stat.have_record = RT_TRUE;
        ps->stat.write_idx++;
    }
    else
    {
        /* 失败槽已写脏 (或写入残卷), 越过该槽下次写下一槽, seq 不推进
         * (旧记录仍在位, 掉电/失败不损失已保存参数) */
        LOG_E("part %s write failed (%d), skip slot %u",
              def->name, (int)err, (unsigned)ps->stat.write_idx);
        ps->stat.write_idx++;
    }

out:
    if (s_eng.lock_ok)
        rt_mutex_release(&s_eng.lock);
    return err;
}

rt_err_t param_part_load(rt_uint8_t id, rt_uint8_t *payload, rt_uint16_t cap,
                         rt_uint16_t *len_out)
{
    const struct param_part_def *def = param_part_def(id);
    struct part_state *ps;
    rt_uint8_t rec[PARAM_SLOT_MAX];
    rt_err_t err;

    if (def == RT_NULL || !def->record_part || payload == RT_NULL)
        return -RT_EINVAL;
    if (def->slot_size > sizeof(rec))
        return -RT_EFULL;

    (void)param_store_init();
    if (s_eng.lock_ok)
        rt_mutex_take(&s_eng.lock, RT_WAITING_FOREVER);

    ps = &s_eng.part[id];
    if (!s_eng.ready)
    {
        err = -RT_EIO;
        goto out;
    }
    if (!ps->stat.have_record)
    {
        err = -RT_ERROR;                         /* 无记录, 调用方用缺省 */
        goto out;
    }

    err = w25q64_read(def->addr + ps->stat.best_slot * def->slot_size,
                      rec, def->slot_size);
    if (err == RT_EOK)
    {
        rt_bool_t is_free = RT_FALSE;
        rt_uint32_t seq = 0;
        rt_uint16_t plen = 0;

        /* 重验一次: 扫描后若有并发写覆盖 (save 持同锁, 理论不可达),
         * 以 CRC 拦截, 宁可返回无记录也不用脏数据 */
        if (!slot_parse(rec, def->slot_size, &is_free, &seq, &plen) ||
            plen > cap)
        {
            err = (plen > cap) ? -RT_EFULL : -RT_ERROR;
            goto out;
        }
        memcpy(payload, rec + REC_HDR_SIZE, plen);
        if (len_out != RT_NULL)
            *len_out = plen;
    }

out:
    if (s_eng.lock_ok)
        rt_mutex_release(&s_eng.lock);
    return err;
}

rt_err_t param_part_erase(rt_uint8_t id)
{
    const struct param_part_def *def = param_part_def(id);
    struct part_state *ps;
    rt_err_t err = RT_EOK;

    if (def == RT_NULL || !def->record_part)
        return -RT_EINVAL;

    (void)param_store_init();
    if (s_eng.lock_ok)
        rt_mutex_take(&s_eng.lock, RT_WAITING_FOREVER);

    ps = &s_eng.part[id];
    if (!s_eng.ready)
    {
        err = -RT_EIO;
        goto out;
    }

    for (rt_uint32_t off = 0; off < def->size && err == RT_EOK;
         off += W25Q64_SECTOR_SIZE)
        err = w25q64_erase_sector(def->addr + off);

    if (err == RT_EOK)
    {
        ps->stat.seq = 0;
        ps->stat.write_idx = 0;
        ps->stat.best_slot = 0;
        ps->stat.last_len = 0;
        ps->stat.have_record = RT_FALSE;
    }

out:
    if (s_eng.lock_ok)
        rt_mutex_release(&s_eng.lock);
    return err;
}

const struct param_part_stat *param_part_stat(rt_uint8_t id)
{
    if (id >= PARAM_PART_NUM)
        return RT_NULL;
    return &s_eng.part[id].stat;
}

/* ------------------------- 上电自启 + MSH 命令 ------------------------- */

static int param_store_boot(void)
{
    (void)param_store_init();
    return 0;
}
INIT_COMPONENT_EXPORT(param_store_boot);

/*
 * param                      查看分区表与各分区状态
 * param erase <ptbl|calib|nav|sys|ctrl|all>   整擦分区 (恢复出厂;
 *            RAM 镜像不立即清零, 各域命令或下次上电生效)
 */
static void param(int argc, char **argv)
{
    if (argc >= 3 && !rt_strcmp(argv[1], "erase"))
    {
        rt_err_t err = -RT_EINVAL;
        rt_bool_t matched = RT_FALSE;

        if (!rt_strcmp(argv[2], "all"))
        {
            matched = RT_TRUE;
            err = RT_EOK;
            for (rt_uint32_t p = 1; p < PARAM_PART_NUM && err == RT_EOK; p++)
                err = param_part_erase((rt_uint8_t)p);
        }
        else
        {
            for (rt_uint32_t p = 0; p < PARAM_PART_NUM; p++)
            {
                if (!rt_strcmp(argv[2], s_defs[p].name))
                {
                    matched = RT_TRUE;
                    err = param_part_erase(s_defs[p].id);
                    break;
                }
            }
        }
        if (matched)
            LOG_I("param erase: %s (%d)", err == RT_EOK ? "OK" : "FAIL",
                  (int)err);
        else
            LOG_W("no such partition: %s", argv[2]);
        return;
    }

    LOG_I("=== W25Q64 参数分区 (ready=%d) ===", (int)param_store_ready());
    for (rt_uint32_t p = 0; p < PARAM_PART_NUM; p++)
    {
        const struct param_part_def *def = &s_defs[p];
        const struct param_part_stat *st = &s_eng.part[p].stat;

        if (!def->record_part)
        {
            LOG_I("part %-5s @0x%06X %4uKB  partition table copy",
                  def->name, (unsigned)def->addr,
                  (unsigned)(def->size / 1024u));
        }
        else
        {
            LOG_I("part %-5s @0x%06X %4uKB  slot=%uB x%u  %s  seq=%u next=%u",
                  def->name, (unsigned)def->addr,
                  (unsigned)(def->size / 1024u),
                  (unsigned)def->slot_size,
                  (unsigned)s_eng.part[p].slot_num,
                  st->have_record ? "valid" : "empty",
                  (unsigned)st->seq, (unsigned)st->write_idx);
        }
    }
    LOG_I("usage: param erase <calib|nav|sys|ctrl|all>");
}
MSH_CMD_EXPORT(param, W25Q64 parameter partitions: show / erase);
