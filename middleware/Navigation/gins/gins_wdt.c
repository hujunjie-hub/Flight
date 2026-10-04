/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * 独立看门狗实现, 设计说明见 gins_wdt.h
 *
 * 时基: LSI ~32kHz, 预分频 256 -> 125Hz (8ms/tick), RLR 12 位
 * (最大 32.7s)。LSI 出厂偏差 ~±5%, 超时按标称值计。
 */

#include <rtthread.h>
#include "stm32h7xx.h"
#include "gins_config.h"
#include "gins_bridge.h"        /* GINS_BRIDGE_ENABLE: 唯一喂狗点与之绑定 */
#include "gins_wdt.h"

volatile rt_uint32_t g_wdt_test_hold;

static rt_uint32_t s_boot_rst_flags;
static rt_bool_t    s_started;

/* IWDG PR 寄存器编码: 6 = /256 (LSI 32kHz -> 125Hz) */
#define WDT_PR_DIV256       6u
#define WDT_TICK_HZ         32000u / 256u

void gins_wdt_start(void)
{
    rt_uint32_t rlr;
    rt_uint32_t spin;

    if (s_started)
        return;                     /* 复位源快照只认最早一次 */

    /* 复位源快照后写 RMVF 清除, 只反映本次上电真实复位源 */
    s_boot_rst_flags = RCC->RSR;
    SET_BIT(RCC->RSR, RCC_RSR_RMVF);

    /* SWD 挂起时冻结 IWDG: 调试/烧录会话不误复位 */
    SET_BIT(DBGMCU->APB4FZ1, DBGMCU_APB4FZ1_DBG_IWDG1);

    rlr = (rt_uint32_t)((rt_uint64_t)GINS_WDT_TIMEOUT_MS * WDT_TICK_HZ / 1000u);
    if (rlr == 0u)
        rlr = 1u;
    if (rlr > 0xFFFu)
        rlr = 0xFFFu;

    WRITE_REG(IWDG1->KR, 0x5555u);              /* 解锁 PR/RLR */

    /* 等 PVU 清零 (上电后 PR 写窗口很短, 带超时防死等) */
    for (spin = 0; spin < 10000u; spin++)
    {
        if (READ_BIT(IWDG1->SR, IWDG_SR_PVU) == 0u)
            break;
    }
    WRITE_REG(IWDG1->PR, WDT_PR_DIV256);

    for (spin = 0; spin < 10000u; spin++)
    {
        if (READ_BIT(IWDG1->SR, IWDG_SR_RVU) == 0u)
            break;
    }
    WRITE_REG(IWDG1->RLR, rlr);

    WRITE_REG(IWDG1->KR, 0xAAAAu);              /* 装载 */
    WRITE_REG(IWDG1->KR, 0xCCCCu);              /* 启动 (IWDG 只能靠复位停) */

    s_started = RT_TRUE;
}

void gins_wdt_feed(void)
{
    if (s_started && g_wdt_test_hold == 0u)
        WRITE_REG(IWDG1->KR, 0xAAAAu);
}

rt_uint32_t gins_wdt_boot_rst_flags(void)
{
    return s_boot_rst_flags;
}

/* INIT_BOARD 级自启动: 早于一切组件/驱动/应用初始化 —— 后续任何模块
 * 的启动期 HardFault/卡死都会在超时后被硬复位拉起 (2026-09-30 实测:
 * sensor 框架启动期函数指针损坏卡死在 fault 处理器, 旧启动点在
 * gins_bridge_init, 挂点更早, 看门狗未启动只能人工断电) */
static int gins_wdt_early_init(void)
{
#if !GINS_BRIDGE_ENABLE
    /* 组合导航桥接关闭时唯一喂狗点 (gins 解算线程) 不编入固件, 启动 IWDG
     * 会陷入 15s 复位循环 —— 不启动 (代价: 此调试配置下无启动期卡死保护) */
    rt_kprintf("wdt: GINS_BRIDGE_ENABLE=0, IWDG not started (no feeder)\n");
#else
    gins_wdt_start();
#endif
    return 0;
}
INIT_BOARD_EXPORT(gins_wdt_early_init);
