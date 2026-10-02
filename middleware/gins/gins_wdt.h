/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * 独立看门狗 (IWDG1)
 *
 * 飞控安全兜底: gins 解算线程存活即喂狗; 线程卡死/跑飞/调度器冻结
 * 超时后硬件复位。复位后导航 ~7s 出解 / ~60s 全稳定 (2026-09-29 收敛
 * 实测), 看门狗周期取 15s 量级远大于该恢复时间。
 *
 * 调试友好: 上电置 DBGMCU IWDG1 冻结位 —— SWD 挂起/断点/烧录时
 * 看门狗停走, 不会在调试会话里误复位。
 *
 * SWD 遥测:
 *   gins_wdt_boot_rst_flags()  上电捕获的 RCC->RSR 复位源位图
 *   g_wdt_test_hold            SWD 写 1 停喂, 用于验证复位链路
 *                              (.bss 复位自动清零, 不会卡测试态)
 */

#ifndef __GINS_WDT_H__
#define __GINS_WDT_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 上电启动看门狗 (gins_bridge_init 调用, 越早越好) */
void gins_wdt_start(void);

/* 喂狗 (gins 解算线程每循环一次; g_wdt_test_hold 非零时停喂) */
void gins_wdt_feed(void);

/* 本次上电捕获的复位源 (RCC_RSR_xxxRSTF 位图, 已写 RMVF 清除) */
rt_uint32_t gins_wdt_boot_rst_flags(void);

/* 测试钩子: 非零 = 喂狗旁路 (SWD -w32 写 1 验证复位链路) */
extern volatile rt_uint32_t g_wdt_test_hold;

#ifdef __cplusplus
}
#endif

#endif /* __GINS_WDT_H__ */
