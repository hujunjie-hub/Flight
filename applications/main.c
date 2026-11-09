/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-05     whj4674672   first version
 */

#include <rtthread.h>
#ifdef FLIGHT_RUNTIME_ENABLE
#include "runtime.h"
#define LOG_TAG "main"
#include <ulog.h>
#endif

int main(void)
{
#ifdef FLIGHT_RUNTIME_ENABLE
    if (runtime_start() != RT_EOK)
        LOG_E("Flight runtime initialization failed; output inhibited.");
#endif
    /* The application tasks own periodic work after startup. */
    while (1)
    {
        rt_thread_mdelay(1000);
    }

    return RT_EOK;
}
