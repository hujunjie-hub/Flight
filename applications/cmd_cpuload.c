/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CPU 负载遥测 (路线图 B6): FinSH `cpuload [秒 1-10]`
 *
 * 免标定原理: idle 钩子对空闲线程空转计数, 测量时先统计正常运行下的
 * N_idle (每秒空闲自旋数), 再起一个最高线程优先级 (1, 低于 ISR 高于一切
 * 应用线程) 的满载自旋线程统计同循环体的 N_busy —— 即本核"空闲容量",
 * load = 1-N_idle/N_busy。
 * 满载窗 (~2s) 会饿死传感器链路一小段: data_cntr 如实记丢拍, IWDG
 * 15s 超时不受威胁, 属按需诊断动作。
 */

#include <rtthread.h>

#define LOG_TAG "cpuload"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static volatile rt_uint32_t s_cpuload_idle_spins;

static void cpuload_idle_hook(void)
{
    s_cpuload_idle_spins++;
}

struct cpuload_busy_ctx
{
    rt_uint32_t spins;                      /* 满载自旋计数 */
    rt_uint32_t ms;                         /* 自旋时长 */
    rt_sem_t   done;
};

static void cpuload_busy_entry(void *p)
{
    struct cpuload_busy_ctx *c = (struct cpuload_busy_ctx *)p;
    rt_tick_t t0 = rt_tick_get();

    while ((rt_tick_t)(rt_tick_get() - t0) < rt_tick_from_millisecond((rt_int32_t)c->ms))
        c->spins++;

    rt_sem_release(c->done);
}

/* 满载自旋测本核空闲容量, 返回每秒自旋数 (0 = 线程创建失败) */
static double cpuload_capacity(rt_uint32_t ms)
{
    struct cpuload_busy_ctx c;
    rt_thread_t tid;

    c.spins = 0;
    c.ms = ms;
    c.done = rt_sem_create("cpld", 0, RT_IPC_FLAG_FIFO);
    if (c.done == RT_NULL)
        return 0.0;

    tid = rt_thread_create("cpldbusy", cpuload_busy_entry, &c, 1024, 1, 10);
    if (tid == RT_NULL)
    {
        rt_sem_delete(c.done);
        return 0.0;
    }
    rt_thread_startup(tid);
    rt_sem_take(c.done, RT_WAITING_FOREVER);
    rt_sem_delete(c.done);

    return (double)c.spins * 1000.0 / (double)ms;
}

static void cpuload(int argc, char **argv)
{
    rt_uint32_t win_ms = 2000;
    rt_uint32_t i0, i1;
    double n_idle, cap, load;

    if (argc >= 2)
    {
        const char *p = argv[1];
        rt_uint32_t v = 0;

        while (*p >= '0' && *p <= '9')
            v = v * 10u + (rt_uint32_t)(*p++ - '0');
        if (v >= 1u && *p == '\0')
            win_ms = v * 1000u;
    }

    rt_thread_idle_sethook(cpuload_idle_hook);

    i0 = s_cpuload_idle_spins;                  /* 阶段 1: 正常运行空闲率 */
    rt_thread_mdelay((rt_int32_t)win_ms);
    i1 = s_cpuload_idle_spins;
    n_idle = (double)(i1 - i0) * 1000.0 / (double)win_ms;

    cap = cpuload_capacity(win_ms);             /* 阶段 2: 满载容量 */

    rt_thread_idle_delhook(cpuload_idle_hook);

    if (cap < 1.0)
    {
        LOG_E("cpuload: 满载线程失败");
        return;
    }

    load = 1.0 - n_idle / cap;
    if (load < 0.0)
        load = 0.0;
    if (load > 1.0)
        load = 1.0;

    LOG_I("CPU load: %.1f%% (idle %.0f/s, capacity %.0f/s, 窗口 %ums)",
          load * 100.0, n_idle, cap, win_ms);
}
MSH_CMD_EXPORT(cpuload, CPU load via idle-spin counting: cpuload [sec 1-10]);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */
