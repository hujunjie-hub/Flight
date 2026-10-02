/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 系统参数域实现, 布局与用途见 param_sys.h。
 */

#include <rtthread.h>
#include <string.h>

#include "param_part.h"
#include "param_sys.h"

#define LOG_TAG "syspar"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

/* payload 布局: boot u32 | flags u16 | rsv u16 | fw_id u32 */
#define SYS_PAYLOAD_SIZE        12u
#define SOF_BOOT                0
#define SOF_FLAGS               4
#define SOF_FWID                8

static struct
{
    struct param_sys_data data;
    rt_bool_t             inited;
} s_sys;

/* ------------------------- 小工具 ------------------------- */

static void put_u16(rt_uint8_t *p, rt_uint16_t v)
{
    p[0] = (rt_uint8_t)v;
    p[1] = (rt_uint8_t)(v >> 8);
}

static rt_uint16_t get_u16(const rt_uint8_t *p)
{
    return (rt_uint16_t)(p[0] | (p[1] << 8));
}

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

/* 固件标识: __DATE__ " " __TIME__ 的 FNV-1a (编译期常量折叠) */
static rt_uint32_t fw_identity(void)
{
    const char *s = __DATE__ " " __TIME__;
    rt_uint32_t h = 2166136261u;

    while (*s)
    {
        h ^= (rt_uint32_t)(unsigned char)*s++;
        h *= 16777619u;
    }
    return h;
}

static rt_err_t sys_save(void)
{
    rt_uint8_t payload[SYS_PAYLOAD_SIZE];

    put_u32(payload + SOF_BOOT, s_sys.data.boot_count);
    put_u16(payload + SOF_FLAGS, s_sys.data.flags);
    put_u16(payload + SOF_FLAGS + 2u, 0u);
    put_u32(payload + SOF_FWID, s_sys.data.fw_id);
    return param_part_save(PARAM_PART_SYS, payload, sizeof(payload));
}

/* ------------------------- 对外接口 ------------------------- */

rt_err_t param_sys_init(void)
{
    rt_uint8_t payload[SYS_PAYLOAD_SIZE];
    rt_uint32_t fw = fw_identity();

    if (s_sys.inited)
        return RT_EOK;

    memset(&s_sys.data, 0, sizeof(s_sys.data));
    s_sys.data.fw_id = fw;                 /* 记录侧固件标识恒为当前固件 */

    if (param_part_load(PARAM_PART_SYS, payload, sizeof(payload), RT_NULL)
        == RT_EOK)
    {
        /* 记录内 fw_id 是 "写入该条记录的固件" 的标识, 仅信息性,
         * 不覆盖本次要写入的当前固件标识 */
        s_sys.data.boot_count = get_u32(payload + SOF_BOOT);
        s_sys.data.flags      = get_u16(payload + SOF_FLAGS);
    }

    /* 本次上电计数追加一条 (无 W25Q64 时静默失败, RAM 计数仍有效) */
    s_sys.data.boot_count++;
    s_sys.inited = RT_TRUE;
    if (sys_save() != RT_EOK)
        LOG_W("sys: boot count save failed (flash absent?)");

    LOG_I("sys: boot #%u fw_id=0x%08X migrated=%d",
          (unsigned)s_sys.data.boot_count, (unsigned)s_sys.data.fw_id,
          (int)param_sys_calib_migrated());
    return RT_EOK;
}

const struct param_sys_data *param_sys_info(void)
{
    (void)param_sys_init();
    return &s_sys.data;
}

rt_bool_t param_sys_calib_migrated(void)
{
    (void)param_sys_init();
    return (rt_bool_t)((s_sys.data.flags & PARAM_SYS_FLAG_CALIB_MIGRATED) != 0u);
}

void param_sys_set_calib_migrated(void)
{
    (void)param_sys_init();
    if (param_sys_calib_migrated())
        return;                           /* 已置位: 不追加记录省磨损 */

    s_sys.data.flags |= PARAM_SYS_FLAG_CALIB_MIGRATED;
    if (sys_save() != RT_EOK)
        LOG_W("sys: migrated flag save failed (will retry next boot)");
}

/* ------------------------- 上电自启 + MSH 命令 ------------------------- */

static int param_sys_boot(void)
{
    (void)param_sys_init();
    return 0;
}
INIT_COMPONENT_EXPORT(param_sys_boot);

static void sysinfo(void)
{
    const struct param_sys_data *d = param_sys_info();

    LOG_I("=== 系统参数 (sys 分区) ===");
    LOG_I("boot_count=%u  fw_id=0x%08X  flags=0x%04X (calib_migrated=%d)",
          (unsigned)d->boot_count, (unsigned)d->fw_id,
          (unsigned)d->flags, (int)param_sys_calib_migrated());
}
MSH_CMD_EXPORT(sysinfo, system params in W25Q64 sys partition);
