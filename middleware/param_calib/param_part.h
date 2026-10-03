/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * W25Q64 参数分区存储引擎
 *
 * 飞控参数按分区持久化到外部 W25Q64 (middleware/sensor/sensor_w25q64.c,
 * OCTOSPI1)。扇区 0 存一份分区表自描述副本 (供上位机工具/自检, 编译期
 * 表为唯一权威); 其余分区统一为 "追加式日志" 记录区:
 *
 *   分区布局 (编译期常量, 见 param_part.c PARAM_PART_DEFS):
 *     id0 ptbl  0x000000  4KB   分区表副本 (信息性, 非记录区)
 *     id1 calib 0x001000  16KB  传感器标定 (param_calib.c, 迁移自片内
 *                               Flash 扇区 7, 含遗留记录一次性导入)
 *     id2 nav   0x005000  16KB  导航参数 (param_nav.c: 磁偏角/无 GNSS
 *                               部署位置/轴向映射/观测开关)
 *     id3 sys   0x009000  8KB   系统参数 (param_sys.c: 启动计数/固件
 *                               标识/迁移标志)
 *     id4 ctrl  0x00B000  20KB  控制参数预留 (middleware/control 控制
 *                               律 2026-10-03 已落地, 增益持久化未接入,
 *                               暂仅占用分区号)
 *     其余 0x010000..0x7FFFFF 未分配 (未来黑匣子日志/航点任务, 建议
 *                               从片尾向前规划, 与参数区互不侵占)
 *
 *   记录格式 (定长槽, slot_size 见分区表; 均小端):
 *     +0   u32 magic 'P','A','R','M'
 *     +4   u16 fmt ver
 *     +6   u16 part id
 *     +8   u32 seq          单调记录序号 (分区独立)
 *     +12  u16 payload len
 *     +14  u16 rsv (0)
 *     +16  u32 crc32        覆盖 [+4,+16) 与 payload
 *     +20  payload (≤ slot_size - 20)
 *
 *   写入: 新记录追加到下一空闲槽 (不擦旧记录, 掉电至多损失最新一条),
 *         读回校验; 分区写满 (每 size/slot_size 次 save) 才逐扇区整擦
 *         重写槽 0 —— 掉电原子性与磨损均衡由此同时获得。上电扫描取
 *         "CRC 通过且 seq 最大" 的记录; 撕裂/坏记录跳过, 首个未编程
 *         槽 (头 4 字节全 FF) 即追加写位置。
 *
 * 与片内 Flash 实现 (原 calibration/calib_store.c) 的关键差异: W25Q64
 * 经 OCTOSPI 间接模式访问, 编程/擦除期间 CPU 照常取指、中断照常响应,
 * 无需 DCache 开关与关中断窗口; 器件忙等待在驱动线程上下文 mdelay
 * 轮询。W25Q64 缺失时引擎降级: 所有分区视为空, save 返回 -RT_EIO,
 * 系统其余部分不受影响 (标定仅本次上电无效)。
 */

#ifndef __PARAM_PART_H__
#define __PARAM_PART_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------- 分区号 ------------------------- */

#define PARAM_PART_PTBL         0u      /* 分区表副本 (非记录区) */
#define PARAM_PART_CALIB        1u      /* 传感器标定 */
#define PARAM_PART_NAV          2u      /* 导航参数 */
#define PARAM_PART_SYS          3u      /* 系统参数 */
#define PARAM_PART_CTRL         4u      /* 控制参数 (预留) */
#define PARAM_PART_NUM          5u

/* ------------------------- 分区定义 (编译期权威) ------------------------- */

struct param_part_def
{
    char        name[8];                /* 展示名 (零结尾) */
    rt_uint8_t  id;
    rt_uint8_t  record_part;            /* 1=追加式记录区; 0=非记录区 (ptbl) */
    rt_uint16_t slot_size;              /* 记录槽大小 (记录区有效), B */
    rt_uint32_t addr;                   /* W25Q64 内偏移, 4KB 对齐 */
    rt_uint32_t size;                   /* 分区大小, 4KB 的整数倍 */
};

const struct param_part_def *param_part_def(rt_uint8_t id);

/* 分区运行状态 (param 命令展示) */
struct param_part_stat
{
    rt_uint32_t seq;                    /* 最新有效记录序号 (0=无记录) */
    rt_uint32_t write_idx;              /* 下一追加槽号; == 槽数 = 待回卷 */
    rt_uint32_t best_slot;              /* 最新有效记录所在槽号 */
    rt_uint16_t last_len;               /* 最新有效记录 payload 长度 */
    rt_bool_t   have_record;            /* 分区内存在有效记录 */
};

/* ------------------------- 引擎接口 ------------------------- */

/*
 * 上电初始化 (INIT_COMPONENT_EXPORT, 亦被各域模块惰性触发, 幂等):
 * 校验 W25Q64 在位 -> 校对/补写扇区 0 分区表副本 -> 逐记录分区扫描。
 * W25Q64 缺失时记录错误并完成 (后续 save 返回 -RT_EIO)。
 */
rt_err_t param_store_init(void);

/* 引擎就绪 (W25Q64 探测成功且已完成扫描) */
rt_bool_t param_store_ready(void);

/* 追加写一条记录 (payload 定长槽内序列化由各域模块负责) */
rt_err_t param_part_save(rt_uint8_t id, const rt_uint8_t *payload,
                         rt_uint16_t len);

/*
 * 取最新有效记录的 payload; cap 为调用方缓冲容量, 不足返回 -RT_EFULL;
 * 分区无有效记录返回 -RT_ERROR (调用方用缺省值)。
 */
rt_err_t param_part_load(rt_uint8_t id, rt_uint8_t *payload, rt_uint16_t cap,
                         rt_uint16_t *len_out);

/* 整擦一个记录分区 (恢复出厂); 非记录区 id 拒绝 */
rt_err_t param_part_erase(rt_uint8_t id);

/* 分区状态快照 (引擎未初始化时为零); 无此分区返回 RT_NULL */
const struct param_part_stat *param_part_stat(rt_uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* __PARAM_PART_H__ */
