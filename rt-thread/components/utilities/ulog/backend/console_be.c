/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-09-04     armink       the first version
 */

#include <rthw.h>
#include <ulog.h>

#ifdef ULOG_BACKEND_USING_CONSOLE

#if defined(ULOG_ASYNC_OUTPUT_BY_THREAD) && ULOG_ASYNC_OUTPUT_THREAD_STACK < 384
#error "The thread stack size must more than 384 when using async output by thread (ULOG_ASYNC_OUTPUT_BY_THREAD)"
#endif

static struct ulog_backend console = { 0 };

/*
 * 控制台整行串行化钩子 (本地定制): 缺省无操作, 应用侧用共享 TX 互斥提供
 * 强实现 (Flight: applications/app_out.c 的 uart1wr) —— 日志行与同口
 * 高频数据帧 (vofa/magout 等) 互不插断, UTF-8 多字节序列不再被拆断成
 * 乱码。ISR/调度器未起上下文由强实现自行放行 (弱钩子不感知上下文)。
 */
__attribute__((weak)) void ulog_console_tx_lock(void)
{
}
__attribute__((weak)) void ulog_console_tx_unlock(void)
{
}

void ulog_console_backend_output(struct ulog_backend *backend, rt_uint32_t level, const char *tag, rt_bool_t is_raw,
        const char *log, rt_size_t len)
{
    ulog_console_tx_lock();
    rt_kputs(log);
    ulog_console_tx_unlock();
}

int ulog_console_backend_init(void)
{
    ulog_init();
    console.output = ulog_console_backend_output;

    ulog_backend_register(&console, "console", RT_TRUE);

    return 0;
}
INIT_PREV_EXPORT(ulog_console_backend_init);

#endif /* ULOG_BACKEND_USING_CONSOLE */
