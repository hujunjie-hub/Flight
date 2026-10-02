/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 系统参数域 (W25Q64 分区 sys, 见 param_part.h)
 *
 * 记录三类全局信息 (64B 槽追加式, 每次上电追加一条):
 *   - boot_count: 累计上电次数 (寿命/问题复现定位)
 *   - fw_id:      固件标识 (构建时间戳 FNV 哈希, 关联参数与固件版本)
 *   - flags:      一次性动作标志 —— bit0: 片内 Flash 旧标定已迁移
 *                 (calib 域查询; `param erase calib` 不会复活旧值)
 *
 * 每次上电一条记录, 128 槽/磨损周期 x 10 万次擦写 = 千万次上电容限。
 */

#ifndef __PARAM_SYS_H__
#define __PARAM_SYS_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PARAM_SYS_FLAG_CALIB_MIGRATED    0x0001u

struct param_sys_data
{
    rt_uint32_t boot_count;             /* 累计上电次数 (含本次) */
    rt_uint16_t flags;                  /* PARAM_SYS_FLAG_* */
    rt_uint32_t fw_id;                  /* __DATE__/__TIME__ FNV-1a */
};

/* 上电加载 + 计数追加保存 (INIT_COMPONENT_EXPORT, 幂等; calib 域会显式
 * 先拉起本初始化, 链接序无关) */
rt_err_t param_sys_init(void);

/* 只读镜像 (未初始化时 boot_count=0) */
const struct param_sys_data *param_sys_info(void);

/* 迁移标志查询/置位 (置位且未置过时追加一条记录) */
rt_bool_t param_sys_calib_migrated(void);
void param_sys_set_calib_migrated(void);

#ifdef __cplusplus
}
#endif

#endif /* __PARAM_SYS_H__ */
