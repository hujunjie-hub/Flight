/*
 * Copyright (c) 2006-2024 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-11-25     hywing       first version
 */

/*
 * 应用入口: LED 心跳 + 按键 + 调试输出链路装配。
 * 各 USART1 输出链路 (线程/初始化/FinSH 命令) 在 out_*.c, 公共设施 (共享
 * 写互斥/STREAM 兜底清除/定点格式化) 在 app_out.c, FinSH 辅助命令在
 * cmd_*.c; 组合导航解算线程由 middleware/gins 的 INIT_ENV_EXPORT 自启。
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>                 /* GET_PIN(): LED/按键 引脚号 (PA/PB/PC 宏) */
#include "app_out.h"

/* ulog 日志 (原 rt_kprintf 全部替换): LOG_E/LOG_W/LOG_I/LOG_D, 行尾自动补 \r\n */
#define LOG_TAG "main"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#define LED0_PIN    GET_PIN(B, 0)
#define LED1_PIN    GET_PIN(E, 1)
#define LED2_PIN    GET_PIN(B, 14)
#define USER_KEY    GET_PIN(C, 13)
#define DELAY       100

static void irq_callback(void *arg)
{
    RT_UNUSED(arg);
    if (rt_pin_read(USER_KEY) == 1)
    {
        LOG_I("Key pressed!");     /* 中断上下文日志: 依赖 ULOG_USING_ISR_LOG */
    }
}

int main(void)
{
    /* set GPIO pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(USER_KEY, PIN_MODE_INPUT_PULLDOWN);
    rt_pin_attach_irq(USER_KEY, PIN_IRQ_MODE_RISING, irq_callback, RT_NULL);
    rt_pin_irq_enable(USER_KEY, PIN_IRQ_ENABLE);

    LOG_I("Flight: UM982 + KF-GINS tagged print on USART1 @460800 "
          "(VOFA JustFloat / magout 同口)");

    /* USART1 共享写互斥 + STREAM 标志清除 (各输出链路整行/整帧写保护) */
    app_out_init();

    /* 启动融合数据推送链路: 失败时仅打印原因, LED 心跳继续 */
    vofa_link_init();

    /* IMU 原始数据链路 (IMUOUT_ENABLE=0 时为空操作) */
    imuout_link_init();

    /* UM982 定位解打印链路 (无数据时线程静默等待) */
    gnssout_link_init();

    /* 启动磁力计原始+校准打印链路 (mag_data -> mag_calib_apply -> USART1) */
    magout_link_init();

    /* KF-GINS 解算结果文本打印链路 */
    gins_fused_data_link_init();

    /* 气压计原始+校准打印链路 (无数据时静默等待, baro_calib_data 后缀) */
    barout_link_init();

    /* 主线程只做 LED 心跳, 数据推送在 "vofa" 线程里按固定节拍进行 */
    while (1)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_pin_write(LED1_PIN, PIN_LOW);
        rt_pin_write(LED2_PIN, PIN_LOW);
        rt_thread_mdelay(DELAY);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_pin_write(LED1_PIN, PIN_HIGH);
        rt_pin_write(LED2_PIN, PIN_LOW);
        rt_thread_mdelay(DELAY);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_pin_write(LED1_PIN, PIN_LOW);
        rt_pin_write(LED2_PIN, PIN_HIGH);
        rt_thread_mdelay(DELAY);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_pin_write(LED1_PIN, PIN_HIGH);
        rt_pin_write(LED2_PIN, PIN_LOW);
        rt_thread_mdelay(DELAY);
    }
}
