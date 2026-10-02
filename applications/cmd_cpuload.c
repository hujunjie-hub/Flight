/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CPU 负载遥测 (路线图 B6): FinSH `cpuload [秒 1-10]`
 *
 * 免标定原理: idle 钩子对空闲线程空转计数, 测量时先统计正常运行下的
 * N_idle (每秒空闲自旋数), 再与"本核满载容量"N_busy 比较,
 * load = 1-N_idle/N_busy。
 * 容量只在本板上电后首次测量 (定频 550MHz 下是常量, 见 board.c
 * SystemClock_Config): 首次起一个最高线程优先级 (1, 低于 ISR 高于一切
 * 应用线程) 的满载自旋线程统计同循环体的 N_busy 并缓存; 后续调用只测
 * idle 率即时返回, 不再以 2s 满载饿死传感器链路。容量缓存失效场景
 * (改主频) 重上电即可。
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

/* 满载自旋测本核空闲容量, 返回每秒自旋数 (0 = 线程创建失败)。
 * 仅容量缓存未建立时调用 (每上电一次), 满载窗内应用线程被饿死属
 * 按需诊断动作: data_cntr 如实记丢拍, IWDG 15s 超时不受威胁 */
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
    static double s_capacity = 0.0;         /* 本核容量缓存 (定频常量) */
    static rt_bool_t s_cap_valid = RT_FALSE;

    rt_uint32_t win_ms = 2000;
    rt_uint32_t i0, i1;
    double n_idle, cap, load;

    if (argc >= 2)
    {
        const char *p = argv[1];
        rt_uint32_t v = 0;

        while (*p >= '0' && *p <= '9')
            v = v * 10u + (rt_uint32_t)(*p++ - '0');
        if (v >= 1u && v <= 10u && *p == '\0')
            win_ms = v * 1000u;
    }

    rt_thread_idle_sethook(cpuload_idle_hook);

    if (!s_cap_valid)
    {
        /* 首次: 测容量 (2s 满载窗) 后缓存, 之后调用不再饿死系统 */
        cap = cpuload_capacity(win_ms);
        if (cap >= 1.0)
        {
            s_capacity = cap;
            s_cap_valid = RT_TRUE;
        }
    }
    else
    {
        cap = s_capacity;
    }

    i0 = s_cpuload_idle_spins;              /* 正常运行空闲率 */
    rt_thread_mdelay((rt_int32_t)win_ms);
    i1 = s_cpuload_idle_spins;
    n_idle = (double)(i1 - i0) * 1000.0 / (double)win_ms;

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

    LOG_I("CPU load: %.1f%% (idle %.0f/s, capacity %.0f/s%s, 窗口 %ums)",
          load * 100.0, n_idle, cap,
          s_cap_valid ? ", 缓存" : ", 首测",
          win_ms);
}
MSH_CMD_EXPORT(cpuload, CPU load via idle-spin counting: cpuload [sec 1-10]);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */
